#!/usr/bin/env python3
"""
Scripted SIH flight on real hardware: switch the board to a SIH airframe
(simulator runs ON the FMU, SYS_HITL=2), fly an auto mission end to end, and
restore the original configuration.

Covers the flight-logic paths the rest of the bench suite cannot: commander
arming, navigator/mission progression, auto takeoff, RTL, land detection and
auto-disarm, all on real NuttX scheduling. The physics is simulated; real
sensor drivers and real outputs are not exercised (pwm_out_sim replaces them,
so nothing on the output rails is ever driven).

With --viewer, every received MAVLink frame is teed to UDP (default 19410)
and the SIH viewer streams are enabled on the board, so a locally running
Hawkeye (`hawkeye -udp 19410 -mc`) renders the flight live.

Requires firmware built with CONFIG_MODULES_SIMULATION_SIMULATOR_SIH=y.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import px4bench
from px4bench import Reporter, MavlinkShell, add_connection_args, connect
from px4bench import ftp as bench_ftp
from px4bench import missions
from px4bench.ftp import LOG_ROOT, ULOG_MAGIC7
from px4bench.missions import BASE_LAT, BASE_LON, Item
from px4bench.params import (drain_param_values, read_param, set_param_int32,
                             wait_param_echo)
from px4bench import firmware as fw_gate

from pymavlink import mavutil

MAV_CMD_NAV_TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
MAV_CMD_NAV_RETURN_TO_LAUNCH = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
MAV_LANDED_STATE_IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
MAV_LANDED_STATE_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
MAV_MODE_FLAG_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

SIH_QUAD_AIRFRAME = 1100        # 1100_rc_quad_x_sih.hil
EKF_CONVERGE_TIMEOUT_S = 90
ARM_TIMEOUT_S = 15
TAKEOFF_TIMEOUT_S = 45
WAYPOINT_TIMEOUT_S = 120
LAND_TIMEOUT_S = 180
WP_OFFSET_DEG = 0.0005          # ~55 m legs


def build_flight_mission(alt):
    """Takeoff, 3-waypoint square leg, RTL. Reuses px4bench.missions.Item."""
    home_lat = int(BASE_LAT * 1e7)
    home_lon = int(BASE_LON * 1e7)
    off = int(WP_OFFSET_DEG * 1e7)

    items = []

    takeoff = Item(0, home_lat, home_lon, alt, 1)
    takeoff.command = MAV_CMD_NAV_TAKEOFF
    takeoff.param2 = 0.0
    items.append(takeoff)

    for seq, (dlat, dlon) in enumerate([(off, 0), (off, off), (0, off)], start=1):
        items.append(Item(seq, home_lat + dlat, home_lon + dlon, alt, 0))

    rtl = Item(len(items), 0, 0, 0, 0)
    rtl.command = MAV_CMD_NAV_RETURN_TO_LAUNCH
    # RTL carries no coordinates; the position frame would (correctly) fail
    # the parameter validation added in #27541 with INVALID_PARAM5_X.
    rtl.frame = mavutil.mavlink.MAV_FRAME_MISSION
    rtl.param2 = 0.0
    items.append(rtl)
    return items


def request_stream(mav, msg_id, rate_hz):
    """MAV_CMD_SET_MESSAGE_INTERVAL; best effort, no ACK check."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, 1e6 / rate_hz, 0, 0, 0, 0, 0)


def wait_landed_state(mav, want, timeout):
    """Wait for EXTENDED_SYS_STATE.landed_state == want. Returns elapsed or None."""
    start = time.monotonic()
    deadline = start + timeout
    while time.monotonic() < deadline:
        m = mav.recv_match(type='EXTENDED_SYS_STATE', blocking=True,
                           timeout=max(0.1, deadline - time.monotonic()))
        if m is not None and m.landed_state == want:
            return time.monotonic() - start
    return None


def wait_disarmed(mav, timeout):
    start = time.monotonic()
    deadline = start + timeout
    while time.monotonic() < deadline:
        m = mav.recv_match(type='HEARTBEAT', blocking=True,
                           timeout=max(0.1, deadline - time.monotonic()))
        if m is None:
            continue
        if m.get_srcSystem() != mav.target_system or m.get_srcComponent() != 1:
            continue
        if not (m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED):
            return time.monotonic() - start
    return None


def shell_cmd(report, shell, cmd, name, timeout=10):
    out, timed_out = shell.run(cmd, timeout=timeout)
    if timed_out:
        report.fail(name, "'{}' stalled after {}s".format(cmd, timeout))
        return None
    return out


def save_params(report, mav, label):
    """Explicit 'param save' so a set survives an immediate reboot.

    Param autosave is asynchronous; rebooting within its window silently
    drops the change (observed on fmu-v6xrt: a restore raced the reboot
    and the board came back on the SIH airframe).
    """
    shell = MavlinkShell(mav)
    if not shell.open(timeout=5):
        report.fail(label, 'could not open nsh shell for param save')
        return False
    _, timed_out = shell.run('param save', timeout=10)
    shell.close()
    if timed_out:
        report.fail(label, "'param save' stalled")
        return False
    return True


def set_and_verify(report, mav, name, value, label):
    drain_param_values(mav)
    set_param_int32(mav, name, value)
    matched, seen = wait_param_echo(mav, name, value)
    report.check(label, matched,
                 '{} = {} (saw: {})'.format(name, value, seen))
    return matched


def enter_sih(report, mav, conn_str, baud, airframe):
    """Record original config, switch to the SIH airframe, reboot.

    Returns (mav, original_autostart, original_hitl) or (mav, None, None).
    """
    original_autostart, _ = read_param(mav, 'SYS_AUTOSTART')
    original_hitl, _ = read_param(mav, 'SYS_HITL')
    if original_autostart is None or original_hitl is None:
        report.fail('sih_enter_snapshot', 'could not read SYS_AUTOSTART/SYS_HITL')
        return mav, None, None
    report.info('original config: SYS_AUTOSTART={} SYS_HITL={}'.format(
        original_autostart, original_hitl))

    if not set_and_verify(report, mav, 'SYS_AUTOSTART', airframe, 'sih_enter_set'):
        return mav, None, None

    if not save_params(report, mav, 'sih_enter_save'):
        return mav, None, None

    try:
        mav, elapsed = px4bench.reboot_and_reconnect(mav, conn_str, baud, timeout=60)
    except TimeoutError as e:
        report.fail('sih_enter_reboot', str(e))
        return mav, None, None
    report.info('rebooted into SIH airframe in {:.1f}s'.format(elapsed))

    hitl, _ = read_param(mav, 'SYS_HITL')
    report.check('sih_active', hitl == 2,
                 'SYS_HITL = {} after reboot (expected 2)'.format(hitl))
    if hitl != 2:
        return mav, original_autostart, original_hitl
    return mav, original_autostart, original_hitl


def restore_config(report, mav, conn_str, baud, original_autostart, original_hitl):
    """Restore SYS_AUTOSTART and SYS_HITL, reboot, verify heartbeat."""
    ok1 = set_and_verify(report, mav, 'SYS_AUTOSTART', original_autostart,
                         'restore_autostart')
    ok2 = set_and_verify(report, mav, 'SYS_HITL', original_hitl,
                         'restore_hitl')
    if not (ok1 and ok2):
        return mav
    if not save_params(report, mav, 'restore_save'):
        return mav
    try:
        mav, elapsed = px4bench.reboot_and_reconnect(mav, conn_str, baud, timeout=60)
        report.check('restore_reboot', True,
                     'back on original config in {:.1f}s'.format(elapsed))
    except TimeoutError as e:
        report.fail('restore_reboot', str(e))
    return mav


def fly(report, mav, shell, alt, report_dir):
    """Upload the mission, arm via nsh commander, track it to touchdown."""
    # Clear any stored mission first: re-uploading an identical mission
    # matches the stored mission CRC and PX4 keeps the completed progress
    # (mission-resume semantics), so the vehicle arms into finished=true
    # and never takes off.
    ok, detail = missions.clear_mission(mav)
    report.check('mission_clear', ok, detail)
    if not ok:
        return False

    items = build_flight_mission(alt)
    ok, duration, detail = missions.upload_mission(report, mav, items, 0)
    report.check('mission_upload', ok, detail or 'uploaded in {:.1f}s'.format(duration))
    if not ok:
        return False
    n_wp = len(items)

    request_stream(mav, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)
    request_stream(mav, mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 2)

    # EKF needs a converged global position before arming is allowed.
    report.info('waiting for global position estimate (up to {}s)'.format(
        EKF_CONVERGE_TIMEOUT_S))
    m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                       timeout=EKF_CONVERGE_TIMEOUT_S)
    if m is None:
        report.fail('ekf_position', 'no GLOBAL_POSITION_INT within {}s'.format(
            EKF_CONVERGE_TIMEOUT_S))
        return False
    report.check('ekf_position', True, 'global position available')

    out = shell_cmd(report, shell, 'commander mode auto:mission', 'set_mode')
    if out is None:
        return False

    armed = False
    deadline = time.monotonic() + ARM_TIMEOUT_S
    while time.monotonic() < deadline and not armed:
        out = shell_cmd(report, shell, 'commander arm', 'arm_cmd')
        if out is None:
            return False
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is not None and (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
        else:
            time.sleep(1)
    report.check('armed', armed, 'vehicle armed in AUTO.MISSION')
    if not armed:
        return False

    elapsed = wait_landed_state(mav, MAV_LANDED_STATE_IN_AIR, TAKEOFF_TIMEOUT_S)
    report.check('takeoff', elapsed is not None,
                 'airborne in {:.1f}s'.format(elapsed) if elapsed is not None
                 else 'not airborne within {}s'.format(TAKEOFF_TIMEOUT_S))
    if elapsed is None:
        # Capture why: commander/navigator view of the world at the moment
        # the takeoff did not happen.
        for cmd in ('commander status', 'listener vehicle_status -n 1',
                    'listener mission_result -n 1',
                    'listener vehicle_control_mode -n 1'):
            out, _ = shell.run(cmd, timeout=10)
            fname = cmd.replace(' ', '_').replace('-', '') + '.txt'
            with open(os.path.join(report_dir, 'takeoff_fail_' + fname), 'w') as f:
                f.write(out)
        report.info('takeoff failure diagnostics saved to {}'.format(report_dir))
        return False

    # Track waypoint progression; item indices 1..n_wp-2 are the square legs.
    reached = set()
    sampled_mid_flight = False
    deadline = time.monotonic() + WAYPOINT_TIMEOUT_S * (n_wp - 2)
    while len(reached) < n_wp - 2 and time.monotonic() < deadline:
        m = mav.recv_match(type='MISSION_ITEM_REACHED', blocking=True, timeout=5)
        if m is None:
            continue
        if m.seq not in reached:
            reached.add(m.seq)
            report.info('reached mission item {}'.format(m.seq))
        if not sampled_mid_flight:
            sampled_mid_flight = True
            out, _ = shell.run('uorb top -1', timeout=10)
            with open(os.path.join(report_dir, 'uorb_top_inflight.txt'), 'w') as f:
                f.write(out)
            report.info('captured in-flight uorb top')
    report.check('waypoints', len(reached) >= n_wp - 2,
                 'reached {} of {} pre-RTL items'.format(len(reached), n_wp - 2))
    if len(reached) < n_wp - 2:
        return False

    elapsed = wait_landed_state(mav, MAV_LANDED_STATE_ON_GROUND, LAND_TIMEOUT_S)
    report.check('rtl_land', elapsed is not None,
                 'landed {:.1f}s after last waypoint'.format(elapsed)
                 if elapsed is not None
                 else 'no touchdown within {}s'.format(LAND_TIMEOUT_S))
    if elapsed is None:
        return False

    elapsed = wait_disarmed(mav, 30)
    report.check('auto_disarm', elapsed is not None,
                 'auto-disarmed {:.1f}s after touchdown'.format(elapsed)
                 if elapsed is not None else 'still armed 30s after landing')
    return elapsed is not None


def download_flight_log(report, mav, report_dir):
    """Pull the newest ULog into the report dir: the log is the only ground
    truth for post-flight verification, so fetch it on failure too (it is
    the post-mortem). Best effort; a missing log is a FAIL, not a crash.
    """
    from pymavlink import mavftp
    try:
        ftp = mavftp.MAVFTP(mav, target_system=mav.target_system,
                            target_component=1)
        dirs = [e.name for e in bench_ftp.ftp_list(ftp, LOG_ROOT)
                if e.is_dir and not e.name.startswith('.')]
        if not dirs:
            report.fail('flight_log', 'no log directories on SD')
            return
        log_dir = '{}/{}'.format(LOG_ROOT, sorted(dirs)[-1])
        ulogs = sorted(e.name for e in bench_ftp.ftp_list(ftp, log_dir)
                       if e.name.endswith('.ulg'))
        if not ulogs:
            report.fail('flight_log', 'no .ulg in {}'.format(log_dir))
            return
        remote = '{}/{}'.format(log_dir, ulogs[-1])
        local = os.path.join(report_dir, ulogs[-1])
        elapsed, err = bench_ftp.ftp_download(mav, ftp, remote, local, report)
        if err is not None:
            report.fail('flight_log', 'download failed: {}'.format(err))
            return
        with open(local, 'rb') as f:
            magic_ok = f.read(7) == ULOG_MAGIC7
        report.check('flight_log', magic_ok,
                     '{} -> {} ({} B, {:.1f}s)'.format(
                         remote, local, os.path.getsize(local), elapsed))
    except Exception as e:  # noqa: BLE001 - never let log retrieval kill restore
        report.fail('flight_log', 'exception: {}'.format(e))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_connection_args(parser)
    parser.add_argument('--airframe', type=int, default=SIH_QUAD_AIRFRAME,
                        help='SIH airframe SYS_AUTOSTART (default: %(default)s)')
    parser.add_argument('--board-dev', default='/dev/ttyACM0',
                        help='mavlink device name ON the board for stream '
                             'commands (default: %(default)s)')
    parser.add_argument('--alt', type=float, default=20.0,
                        help='takeoff/waypoint altitude AMSL-relative m '
                             '(default: %(default)s)')
    parser.add_argument('--viewer', action='store_true',
                        help='tee MAVLink to UDP and enable SIH viewer streams '
                             '(watch with: hawkeye -udp 19410 -mc)')
    parser.add_argument('--viewer-port', type=int, default=19410)
    parser.add_argument('--keep-config', action='store_true',
                        help='stay on the SIH airframe when done')
    parser.add_argument('--expect-hash', metavar='PREFIX', default=None,
                        help='verify the board runs this git hash before flying '
                             '(prefix match); mismatch aborts. No flashing here.')
    parser.add_argument('--allow-arming', action='store_true', default=False,
                        help='skip the arming confirmation prompt (automation; '
                             'board must be bare, nothing on the output rails)')
    parser.add_argument('--report-dir', default='bench_reports')
    args = parser.parse_args()

    report = Reporter('flight_mission')
    report_dir = px4bench.make_report_dir(args.report_dir, 'flight_mission')
    report.info('report dir: {}'.format(report_dir))

    try:
        mav = connect(args.connection, args.baudrate)
    except (TimeoutError, OSError) as e:
        report.fail('connect', str(e))
        return report.finish()
    report.check('connect', True, 'heartbeat from {}'.format(args.connection))

    ident, ident_err = fw_gate.board_identity(mav)
    if ident is None:
        report.fail('board_identity', ident_err)
        return report.finish()
    report.info('firmware on board:\n' + fw_gate.format_identity(ident))
    fw_gate.stamp_identity(report_dir, ident)
    if args.expect_hash:
        ok = fw_gate.hashes_match(ident['git_hash'], args.expect_hash)
        report.check('firmware_identity', ok,
                     'board reports {} , expected {}'.format(
                         ident['git_hash'], args.expect_hash))
        if not ok:
            mav.close()
            return report.finish()

    # Probe the live firmware for the SIH module before touching any config.
    # nsh replies 'command not found' when the module is not in this build.
    probe_shell = MavlinkShell(mav)
    if not probe_shell.open(timeout=5):
        report.fail('sih_probe', 'nsh shell did not respond within 5s')
        return report.finish()
    present, probe_out = px4bench.shell_command_exists(
        probe_shell, 'simulator_sih status')
    probe_shell.close()
    if present is None:
        report.fail('sih_probe', "'simulator_sih status' stalled (probe hung)")
        return report.finish()
    if present is False:
        report.info('WARNING: this firmware does not include the SIH module '
                    '(simulator_sih: command not found); skipping the flight '
                    'test. Build with CONFIG_MODULES_SIMULATION_SIMULATOR_SIH=y '
                    'or use a bench variant target.')
        mav.close()
        return px4bench.EXIT_SKIP
    report.info('SIH module present in this firmware')

    # Arming gate: the vehicle arms (simulated flight, pwm_out_sim), so the
    # board must be bare. Declining is a skip, not a failure.
    if not px4bench.arming_gate(args.allow_arming,
                                'fly the simulated SIH mission on this board'):
        mav.close()
        return px4bench.EXIT_SKIP

    original_autostart = original_hitl = None
    try:
        mav, original_autostart, original_hitl = enter_sih(
            report, mav, args.connection, args.baudrate, args.airframe)
        if original_autostart is None or report.num_failed > 0:
            return report.finish()

        if args.viewer:
            px4bench.attach_viewer_tee(mav, port=args.viewer_port)
            report.info('viewer tee on udp:127.0.0.1:{}'.format(args.viewer_port))

        shell = MavlinkShell(mav)
        if not shell.open(timeout=5):
            report.fail('shell', 'could not open nsh shell')
            return report.finish()

        if args.viewer:
            for cmd in ('mavlink stream -d {} -s HIL_STATE_QUATERNION -r 25'
                        .format(args.board_dev),
                        'mavlink stream -d {} -s HIL_ACTUATOR_CONTROLS -r 200'
                        .format(args.board_dev)):
                shell_cmd(report, shell, cmd, 'viewer_stream')
            report.info('viewer streams on; watch with: hawkeye -udp {} -mc'
                        .format(args.viewer_port))

        ok = fly(report, mav, shell, args.alt, report_dir)
        if not ok:
            # Best effort: never leave a (simulated) vehicle armed.
            shell.run('commander disarm -f', timeout=5)
        shell.close()

        # Post-flight verification starts from the ULog; always retrieve it.
        download_flight_log(report, mav, report_dir)

    finally:
        if original_autostart is not None and not args.keep_config:
            try:
                mav = restore_config(report, mav, args.connection,
                                     args.baudrate, original_autostart,
                                     original_hitl)
            except Exception as e:
                report.fail('restore', 'exception during restore: {}'.format(e))
        try:
            if mav is not None:
                mav.close()
        except Exception:
            pass

    return report.finish()


if __name__ == '__main__':
    sys.exit(main())

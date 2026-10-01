#!/usr/bin/env python3
"""
Grade the GNSS failovers in a ULog. Every injected GNSS failure (failure_injection) is checked against what the
selection, EKF2, commander and the heading should do: the switch away from a failed selected receiver, one EKF2 reset
by the offset between the receivers, a valid position, the mode, no return while armed, GNSS yaw. A failure of the
standby must change nothing, and a loss of every receiver must invalidate the position until one recovers. Works on
SIH and flight logs; the hold check needs SIH ground truth.

    Tools/gnss_failover_report.py log.ulg [--checks selection,reset,position,held,heading,reporting]
"""

import argparse
import math
import sys

import numpy as np
from pyulog import ULog

FAILURE_UNIT_SENSOR_GPS = 4
FAILURE_TYPE_NAMES = {0: 'ok', 1: 'off', 2: 'stuck', 3: 'garbage', 4: 'wrong', 5: 'slow', 6: 'delayed',
                      7: 'intermittent', 8: 'drift'}

# vehicle_gnss.selection_reason
SELECTION_REASONS = {0: 'preferred', 1: 'only', 2: 'ranked', 3: 'timeout', 4: 'unhealthy', 5: 'requirements',
                     6: 'RTK fixed'}
FAILURE_REASONS = {3, 4}
RANKING_REASONS = {5, 6}

# estimator_status_flags.gnss_fusion_state
GNSS_FUSION_FUSED = 0
GNSS_FUSION_NO_DATA = 1
GNSS_FUSION_UNUSABLE = 2
# The state a GNSS failure type leaves while the failed receiver is still selected
FUSION_STATE_OF_FAILURE = {'off': GNSS_FUSION_NO_DATA, 'wrong': GNSS_FUSION_UNUSABLE}

NAV_STATE_AUTO_LOITER = 4
ARMING_STATE_ARMED = 2
GNSS_HEIGHT_REFERENCE = 1  # EKF2_HGT_REF

# The selector replaces a silent receiver at its 2 s timeout and one without usable samples after 2 s
SWITCH_DEADLINE_S = 4.
# Without a primary, it moves to a receiver one ranking level higher after the ranking hold
RANKING_HOLD_ARMED_S = 10.
RANKING_HOLD_DISARMED_S = 2.
RANKING_EARLY_S = 1.
RANKING_LATE_S = 3.
# Injections of one receiver closer than this are one intermittent failure
MERGE_GAP_S = 2.
# A reset belongs to a switch when it follows within the estimator delay
RESET_AFTER_SWITCH_S = 1.
# Window after the switch in which nothing else may happen
SETTLE_S = 10.
# Position loss after every receiver failed: EKF2_NOAID_TOUT plus this; recovery once one is back within this
LOSS_MARGIN_S = 3.

RESET_TOLERANCE_M = 0.5
HELD_TOLERANCE_M = 1.
EARTH_RADIUS_M = 6371000.

CHECK_GROUPS = ('selection', 'reset', 'position', 'held', 'heading', 'reporting')


class Log:
    def __init__(self, path):
        self.ulog = ULog(path)
        self.t0 = self.ulog.start_timestamp

    def param(self, name, t, default=None):
        """Parameter value at log time t [s]"""
        value = self.ulog.initial_parameters.get(name, default)
        for timestamp, changed, changed_value in self.ulog.changed_parameters:
            if changed == name and (timestamp - self.t0) * 1e-6 <= t:
                value = changed_value
        return value

    def topic(self, name, instance=0):
        for data in self.ulog.data_list:
            if data.name == name and data.multi_id == instance:
                return data.data
        return None

    def instances(self, name):
        return sorted(data.multi_id for data in self.ulog.data_list if data.name == name)

    def seconds(self, timestamp_us):
        return (np.asarray(timestamp_us, dtype=np.float64) - self.t0) * 1e-6


def value_at(times, values, t):
    """Latest value at or before t, or None"""
    i = np.searchsorted(times, t, side='right') - 1
    return None if i < 0 else values[i]


def changes(times, values):
    """(time, previous, new) for every change of a value"""
    idx = np.where(np.diff(values.astype(np.int64)) != 0)[0] + 1
    return [(times[i], values[i - 1], values[i]) for i in idx]


def event_id(name):
    """PX4 event ID of an autopilot event name (events::ID())"""
    value = 0x811c9dc5
    for c in name.encode():
        value = ((value ^ c) * 0x01000193) & 0xffffffff
    return (value & 0xffffff) | (1 << 24)


EVENT_RECEIVER_SWITCHED = event_id('gnss_receiver_switched')


def reason_name(reason):
    return SELECTION_REASONS.get(int(reason), f'reason {int(reason)}')


class Result:
    def __init__(self, groups):
        self.rows = []
        self.failed = False
        self.groups = groups

    def add(self, window, check, ok, detail, group):
        if group not in self.groups:
            return
        state = 'n/a' if ok is None else ('PASS' if ok else 'FAIL')
        self.failed = self.failed or ok is False
        self.rows.append((window, check, state, detail))

    def info(self, window, check, detail):
        self.rows.append((window, check, 'info', detail))

    def print(self):
        width = max((len(r[1]) for r in self.rows), default=10)
        window = None
        for row in self.rows:
            if row[0] != window:
                window = row[0]
                print('\n' + window)
            print(f'  {row[2]:4}  {row[1]:{width}}  {row[3]}')


class Vehicle:
    """Vehicle state the grading needs, by log time"""

    def __init__(self, log):
        self.log = log
        gnss = log.topic('vehicle_gnss')
        self.sel_times = log.seconds(gnss['timestamp'])
        self.sel_instance = gnss['selected_instance']
        self.sel_count = gnss['selection_count']
        self.sel_reason = gnss['selection_reason']

        self.status = log.topic('vehicle_status')
        self.st_times = log.seconds(self.status['timestamp'])
        self.land = log.topic('vehicle_land_detected')
        self.land_times = log.seconds(self.land['timestamp']) if self.land is not None else None
        self.failsafe_flags = log.topic('failsafe_flags')
        self.ff_times = log.seconds(self.failsafe_flags['timestamp']) if self.failsafe_flags is not None else None

    def selected(self, t):
        return int(value_at(self.sel_times, self.sel_instance, t))

    def switches(self, t_start, t_end=math.inf):
        """(time, new instance, reason) of every selection change in [t_start, t_end]"""
        result = []
        for t, _, _ in changes(self.sel_times, self.sel_count):
            if t_start <= t <= t_end:
                i = np.searchsorted(self.sel_times, t)
                result.append((t, int(self.sel_instance[i]), int(self.sel_reason[i])))
        return result

    def armed(self, t):
        return value_at(self.st_times, self.status['arming_state'], t) == ARMING_STATE_ARMED

    def in_air(self, t):
        return self.land is not None and not value_at(self.land_times, self.land['landed'], t)

    def flag_any(self, name, t_start, t_end):
        if self.failsafe_flags is None:
            return None
        sel = (self.ff_times >= t_start) & (self.ff_times <= t_end)
        return bool(np.any(self.failsafe_flags[name][sel]))

    def commanded_mode_change(self, t_start):
        """Time of the first mode change after t_start that no failsafe caused, or infinity"""
        nav = self.status['nav_state']
        for i in range(1, len(nav)):
            if self.st_times[i] > t_start and nav[i] != nav[i - 1] and not self.status['failsafe'][i]:
                return self.st_times[i]
        return math.inf

    def first_flag(self, name, value, t_start, t_end):
        """First time in [t_start, t_end] the failsafe flag has this value"""
        if self.failsafe_flags is None:
            return None
        sel = np.where((self.ff_times >= t_start) & (self.ff_times <= t_end)
                       & (self.failsafe_flags[name].astype(bool) == value))[0]
        return self.ff_times[sel[0]] if len(sel) else None


def injections(log):
    """GNSS injections per receiver (0-based), with intermittent ones merged into one window"""
    data = log.topic('failure_injection')
    if data is None:
        return []

    times = log.seconds(data['timestamp'])
    active = {}
    windows = []

    for k, t in enumerate(times):
        now = {}
        for i in range(int(data['count'][k])):
            if int(data[f'unit[{i}]'][k]) != FAILURE_UNIT_SENSOR_GPS:
                continue
            mask = int(data[f'instance_mask[{i}]'][k])
            for instance in range(16):
                if mask & (1 << instance):
                    now[instance] = FAILURE_TYPE_NAMES.get(int(data[f'failure_type[{i}]'][k]), '?')

        for instance, (start, kind) in list(active.items()):
            if now.get(instance) != kind:
                windows.append({'start': start, 'end': t, 'type': kind, 'instance': instance, 'toggles': 1})
                del active[instance]

        for instance, kind in now.items():
            if instance not in active:
                active[instance] = (t, kind)

    for instance, (start, kind) in active.items():
        windows.append({'start': start, 'end': math.inf, 'type': kind, 'instance': instance, 'toggles': 1})

    merged = []
    for window in sorted(windows, key=lambda w: (w['instance'], w['start'])):
        last = merged[-1] if merged else None
        if (last and last['instance'] == window['instance'] and last['type'] == window['type']
                and window['start'] - last['end'] < MERGE_GAP_S):
            last['end'] = window['end']
            last['toggles'] += 1
        else:
            merged.append(dict(window))

    return sorted(merged, key=lambda w: (w['start'], w['instance']))


def lever_arm_offset(log, instance, t):
    """NED position of receiver instance's antenna relative to the centre of gravity at t"""
    offset = np.array([log.param(f'SENS_GNSS{instance}_OFF{axis}', t, 0.) for axis in 'XYZ'])
    attitude = log.topic('vehicle_attitude')
    if attitude is None or not np.any(offset):
        return offset
    i = max(np.searchsorted(log.seconds(attitude['timestamp']), t) - 1, 0)
    w, x, y, z = (attitude[f'q[{k}]'][i] for k in range(4))
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return rotation @ offset


def antenna_offset(log, from_instance, to_instance, t):
    """NED offset of to_instance's antenna position from from_instance's last sample before t. The other receiver's
    nearest sample is moved to the same measurement time with its velocity."""
    a = log.topic('sensor_gnss', from_instance)
    b = log.topic('sensor_gnss', to_instance)
    if a is None or b is None:
        return None
    i = np.searchsorted(log.seconds(a['timestamp']), t, side='right') - 1
    if i < 0:
        return None
    t_sample = float(a['timestamp_sample'][i])
    j = int(np.argmin(np.abs(b['timestamp_sample'].astype(np.float64) - t_sample)))
    dt = (t_sample - float(b['timestamp_sample'][j])) * 1e-6
    lat_a = a['latitude'][i]
    return np.array([
        math.radians(b['latitude'][j] - lat_a) * EARTH_RADIUS_M + b['vel_north'][j] * dt,
        math.radians(b['longitude'][j] - a['longitude'][i]) * EARTH_RADIUS_M * math.cos(math.radians(lat_a))
        + b['vel_east'][j] * dt,
        -(b['altitude_msl'][j] - a['altitude_msl'][i]) + b['vel_down'][j] * dt])


def receiver_offset(log, from_instance, to_instance, t):
    """NED offset of to_instance's position from from_instance's last sample before t, after lever arms. The other
    receiver's nearest sample is moved to the same measurement time with its velocity."""
    offset = antenna_offset(log, from_instance, to_instance, t)
    if offset is None:
        return None
    return offset - lever_arm_offset(log, to_instance, t) + lever_arm_offset(log, from_instance, t)


def expected_inconsistency(log, selected, other, t):
    """How far the other receiver disagrees with the selected one without attitude: the horizontal distance between
    their positions less the horizontal distance between their antennas"""
    offset = antenna_offset(log, selected, other, t)
    if offset is None:
        return None
    antennas = [np.array([log.param(f'SENS_GNSS{i}_OFF{axis}', t, 0.) for axis in 'XY']) for i in (selected, other)]
    return abs(math.hypot(offset[0], offset[1]) - float(np.linalg.norm(antennas[1] - antennas[0])))


def resets_between(log, t_start, t_end):
    """(time, 'xy' or 'z', delta) of every EKF2 position reset in [t_start, t_end]"""
    lpos = log.topic('vehicle_local_position')
    times = log.seconds(lpos['timestamp'])
    resets = []
    for axis, deltas in (('xy', ('delta_xy[0]', 'delta_xy[1]')), ('z', ('delta_z',))):
        for t, _, _ in changes(times, lpos[f'{axis}_reset_counter']):
            if t_start <= t <= t_end:
                i = np.searchsorted(times, t)
                resets.append((t, axis, [float(lpos[d][i]) for d in deltas]))
    return sorted(resets)


def grade_failure(log, vehicle, result, name, window):
    """The selected receiver failed: switch to the standby, one reset by the receiver offset, nothing else"""
    t_inject, t_end, failed = window['start'], window['end'], window['instance']
    switches = vehicle.switches(t_inject)
    first = switches[0] if switches else None
    describe = (f'{first[0] - t_inject:.2f} s after the failure, to instance {first[1] + 1}, '
                f'{reason_name(first[2])}') if first else 'no switch'

    if first is not None and first[2] in RANKING_REASONS:
        # The failure only lowered the receiver's ranking level: the selection moves after the ranking hold
        hold = RANKING_HOLD_ARMED_S if vehicle.armed(t_inject) else RANKING_HOLD_DISARMED_S
        switched = hold - RANKING_EARLY_S <= first[0] - t_inject <= hold + RANKING_LATE_S
        check = f'switch after the {hold:.0f} s ranking hold'

    else:
        # An intermittent failure is caught by the availability, any time while it lasts
        deadline = (t_end - t_inject + RESET_AFTER_SWITCH_S) if window['toggles'] > 1 else SWITCH_DEADLINE_S
        switched = first is not None and first[0] - t_inject <= deadline and first[2] in FAILURE_REASONS
        check = 'switch to the standby'

    switched = switched and first[1] != failed
    result.add(name, check, switched, describe, 'selection')
    if not switched:
        return

    t_switch, standby, _ = first

    while_failed = [s for s in switches[1:] if s[0] < t_end]
    result.add(name, 'one switch while failed', not while_failed,
               ', '.join(f'{s[0] - t_inject:.1f} s to instance {s[1] + 1}' for s in while_failed) or 'one',
               'selection')

    # While armed, a receiver the selection left is picked again only when the current one fails
    if math.isfinite(t_end):
        returns = [s for s in switches[1:] if s[0] >= t_end and s[1] == failed and vehicle.armed(s[0])
                   and s[2] not in FAILURE_REASONS]
        result.add(name, 'no return while armed', not returns,
                   f'returned {returns[0][0] - t_end:.1f} s after recovery, {reason_name(returns[0][2])}'
                   if returns else 'no return', 'selection')

    # Resets: horizontal once, height once when GNSS is the height reference. A mode change that isn't a failsafe
    # was commanded and ends the window.
    window_end = min(t_switch + SETTLE_S, vehicle.commanded_mode_change(t_inject))
    resets = resets_between(log, t_inject, window_end)
    xy = [r for r in resets if r[1] == 'xy']
    z = [r for r in resets if r[1] == 'z']
    expected = receiver_offset(log, failed, standby, t_inject)
    result.add(name, 'one horizontal reset at the switch',
               len(xy) == 1 and 0. <= xy[0][0] - t_switch <= RESET_AFTER_SWITCH_S,
               f'{len(xy)} resets' + (f', {xy[0][0] - t_switch:.2f} s after the switch' if xy else ''), 'reset')
    if xy and expected is not None:
        error = math.hypot(xy[0][2][0] - expected[0], xy[0][2][1] - expected[1])
        result.add(name, 'horizontal reset delta = receiver offset', error < RESET_TOLERANCE_M,
                   f'delta ({xy[0][2][0]:.2f}, {xy[0][2][1]:.2f}) m, offset ({expected[0]:.2f}, {expected[1]:.2f}) m',
                   'reset')

    if int(log.param('EKF2_HGT_REF', t_inject, -1)) == GNSS_HEIGHT_REFERENCE:
        ok = len(z) == 1 and expected is not None and abs(z[0][2][0] - expected[2]) < RESET_TOLERANCE_M
        result.add(name, 'one height reset = receiver height offset', ok,
                   f'{len(z)} resets' + (f', delta {z[0][2][0]:.2f} m' if z else '')
                   + (f', offset {expected[2]:.2f} m' if expected is not None else ''), 'reset')
    else:
        result.add(name, 'no height reset (height reference not GNSS)', not z, f'{len(z)} resets', 'reset')

    # Position validity, failsafes and mode
    invalid = vehicle.flag_any('local_position_invalid', t_inject, window_end) or \
        vehicle.flag_any('global_position_invalid', t_inject, window_end)
    result.add(name, 'position stays valid', not invalid, 'invalid at times' if invalid else 'valid', 'position')

    nav = vehicle.status['nav_state']
    sel = (vehicle.st_times >= t_inject) & (vehicle.st_times < window_end)
    mode_before = int(value_at(vehicle.st_times, nav, t_inject))
    modes = {int(m) for m in nav[sel]} | {mode_before}
    failsafe = bool(np.any(vehicle.status['failsafe'][sel]))
    gnss_lost = bool(vehicle.flag_any('gnss_lost', t_inject, window_end))
    result.add(name, 'mode kept, no failsafe but gnss_lost', modes == {mode_before} and (not failsafe or gnss_lost),
               f'nav_state {sorted(modes)}, failsafe {failsafe}, gnss_lost {gnss_lost}', 'position')

    # In Hold the vehicle doesn't move: the setpoint follows the reset
    truth = log.topic('vehicle_local_position_groundtruth')
    if truth is not None and mode_before == NAV_STATE_AUTO_LOITER and vehicle.in_air(t_inject):
        gt_times = log.seconds(truth['timestamp'])
        before = np.searchsorted(gt_times, t_inject)
        after = np.searchsorted(gt_times, window_end) - 1
        moved = math.hypot(truth['x'][after] - truth['x'][before], truth['y'][after] - truth['y'][before])
        climbed = -(truth['z'][after] - truth['z'][before])
        result.add(name, 'held in place (ground truth)', moved < HELD_TOLERANCE_M and abs(climbed) < HELD_TOLERANCE_M,
                   f'moved {moved:.2f} m horizontally, {climbed:+.2f} m vertically', 'held')

    grade_heading(log, result, name, t_inject, window_end)
    if math.isfinite(t_end):
        grade_heading_recovery(log, result, name, t_inject, t_end, t_end + 2 * SETTLE_S)

    grade_reporting(log, result, name, window, t_switch, standby, failed_unusable=first[2] in FAILURE_REASONS)


def grade_heading(log, result, name, t_start, t_end):
    """A heading source that fails stops GNSS yaw without a yaw reset, and the heading continues on the mag"""
    flags = log.topic('estimator_status_flags')
    if flags is None or 'cs_gnss_yaw' not in flags:
        return
    times = log.seconds(flags['timestamp'])
    if not value_at(times, flags['cs_gnss_yaw'], t_start):
        return
    sel = (times >= t_start) & (times <= t_end)
    if not np.any(sel):
        return
    stopped = not bool(flags['cs_gnss_yaw'][sel][-1])
    result.add(name, 'GNSS yaw fusion stops', stopped, 'stopped' if stopped else 'still fused', 'heading')

    mag = [k for k in ('cs_mag_hdg', 'cs_mag_3d', 'cs_mag') if k in flags]
    on_mag = any(bool(flags[k][sel][-1]) for k in mag)
    result.add(name, 'heading continues on the magnetometer', on_mag, 'mag fused' if on_mag else 'no mag fusion',
               'heading')

    attitude = log.topic('vehicle_attitude')
    att_times = log.seconds(attitude['timestamp'])
    resets = [t for t, _, _ in changes(att_times, attitude['quat_reset_counter']) if t_start <= t <= t_end]
    result.add(name, 'no yaw reset', not resets, f'{len(resets)} yaw resets', 'heading')


def grade_heading_recovery(log, result, name, t_inject, t_recover, t_end):
    flags = log.topic('estimator_status_flags')
    if flags is None or 'cs_gnss_yaw' not in flags:
        return
    times = log.seconds(flags['timestamp'])
    if not value_at(times, flags['cs_gnss_yaw'], t_inject) or value_at(times, flags['cs_gnss_yaw'], t_recover):
        return
    resumed = [t for t, _, new in changes(times, flags['cs_gnss_yaw']) if new and t_recover <= t <= t_end]
    result.add(name, 'GNSS yaw fusion resumes', bool(resumed),
               f'{resumed[0] - t_recover:.1f} s after recovery' if resumed else 'not resumed', 'heading')


def grade_reporting(log, result, name, window, t_switch, standby, failed_unusable):
    """Loss reporting: the switch event, why GNSS wasn't fused until the switch, the receivers' inconsistency"""
    t_inject = window['start']

    events = log.topic('event')
    if events is None:
        result.add(name, 'one receiver switch event', None, 'no events in this log', 'reporting')
    else:
        times = log.seconds(events['timestamp'])
        sel = (times >= t_inject) & (times <= t_switch + RESET_AFTER_SWITCH_S)
        count = int(np.sum(events['id'][sel] == EVENT_RECEIVER_SWITCHED))
        result.add(name, 'one receiver switch event', count == 1, f'{count} gnss_receiver_switched events',
                   'reporting')

    flags = log.topic('estimator_status_flags')
    if flags is None or 'gnss_fusion_state' not in flags:
        result.add(name, 'gnss_fusion_state reports the gap', None, 'not in this log', 'reporting')
    else:
        times = log.seconds(flags['timestamp'])
        sel = (times >= t_inject) & (times <= t_switch)
        states = sorted({int(s) for s in flags['gnss_fusion_state'][sel]})

        # A receiver that stays usable (a lower ranking, a late but fused sample) or drops out for less than the
        # no-data time leaves nothing to report
        wanted = FUSION_STATE_OF_FAILURE.get(window['type']) if failed_unusable and window['toggles'] == 1 else None
        result.add(name, 'gnss_fusion_state reports the gap', (wanted in states) if wanted is not None else None,
                   f'states {states}' + (f', expected {wanted}' if wanted is not None else ''), 'reporting')
        after = value_at(times, flags['gnss_fusion_state'], t_switch + SETTLE_S)
        result.add(name, 'GNSS fused again after the switch', after == GNSS_FUSION_FUSED, f'state {after}', 'reporting')

    status = log.topic('sensors_status_gnss')
    expected = expected_inconsistency(log, window['instance'], standby, t_inject)
    if status is None or f'inconsistency[{standby}]' not in status:
        result.add(name, 'inconsistency matches the receivers', None, 'not in this log', 'reporting')
    elif expected is not None:
        # The failed receiver may stop publishing, so the last value before the switch is from before the failure
        times = log.seconds(status['timestamp'])
        values = status[f'inconsistency[{standby}]']
        before = [v for t, v in zip(times, values) if t_inject - SWITCH_DEADLINE_S <= t < t_switch and np.isfinite(v)]
        ok = bool(before) and abs(before[-1] - expected) < RESET_TOLERANCE_M
        result.add(name, 'inconsistency matches the receivers', ok,
                   (f'{before[-1]:.2f} m' if before else 'none before the switch') + f', expected {expected:.2f} m',
                   'reporting')
        selected_after = value_at(times, values, t_switch + RESET_AFTER_SWITCH_S)
        result.add(name, 'inconsistency 0 for the new selection', selected_after == 0., f'{selected_after}',
                   'reporting')


def grade_standby(log, vehicle, result, name, window):
    """A failure of the standby changes nothing; a standby that got better may be ranked up after the hold"""
    t_inject = window['start']
    end = min(window['end'], t_inject + SETTLE_S)
    switches = vehicle.switches(t_inject, end)
    ranked_up = [s for s in switches if s[1] == window['instance'] and s[2] in RANKING_REASONS]

    if ranked_up:
        hold = RANKING_HOLD_ARMED_S if vehicle.armed(t_inject) else RANKING_HOLD_DISARMED_S
        delay = ranked_up[0][0] - t_inject
        result.add(name, f'ranked up after the {hold:.0f} s ranking hold',
                   hold - RANKING_EARLY_S <= delay <= hold + RANKING_LATE_S and len(switches) == 1,
                   f'{delay:.2f} s after the change, {reason_name(ranked_up[0][2])}', 'selection')
        resets = [r for r in resets_between(log, t_inject, end) if r[1] == 'xy']
        result.add(name, 'one horizontal reset', len(resets) == 1, f'{len(resets)} resets', 'reset')
        return

    result.add(name, 'no switch', not switches, f'{len(switches)} switches', 'selection')
    resets = resets_between(log, t_inject, end)
    result.add(name, 'no reset', not resets, f'{len(resets)} resets', 'reset')


def grade_total_loss(log, vehicle, result, name, windows):
    """Every receiver failed: the position goes invalid after EKF2_NOAID_TOUT, and comes back with a receiver"""
    t_inject = windows[0]['start']
    no_aid_timeout = float(log.param('EKF2_NOAID_TOUT', t_inject, 5000000)) * 1e-6
    t_lost = vehicle.first_flag('local_position_invalid', True, t_inject, t_inject + no_aid_timeout + LOSS_MARGIN_S)
    result.add(name, 'position invalid after EKF2_NOAID_TOUT', t_lost is not None,
               f'{t_lost - t_inject:.1f} s after the loss' if t_lost is not None else 'still valid', 'position')

    sel = (vehicle.st_times >= t_inject) & (vehicle.st_times <= t_inject + no_aid_timeout + LOSS_MARGIN_S)
    failsafe = bool(np.any(vehicle.status['failsafe'][sel]))
    result.add(name, 'failsafe', failsafe, str(failsafe), 'position')

    t_recover = min(w['end'] for w in windows)
    if math.isfinite(t_recover):
        t_valid = vehicle.first_flag('local_position_invalid', False, t_recover, t_recover + LOSS_MARGIN_S)
        result.add(name, 'position valid again once a receiver is back', t_valid is not None,
                   f'{t_valid - t_recover:.1f} s after the first recovery' if t_valid is not None else 'still invalid',
                   'position')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', help='ULog file')
    parser.add_argument('--checks', default=','.join(CHECK_GROUPS),
                        help=f'comma-separated check groups to grade, from {",".join(CHECK_GROUPS)}')
    args = parser.parse_args()

    groups = set(args.checks.split(','))
    unknown = groups - set(CHECK_GROUPS)
    if unknown:
        parser.error(f'unknown check groups {sorted(unknown)}')

    log = Log(args.log)
    if log.topic('vehicle_gnss') is None:
        print('No vehicle_gnss in the log')
        return 1

    vehicle = Vehicle(log)
    receivers = set(log.instances('sensor_gnss'))
    result = Result(groups)
    graded_switches = set()
    windows = injections(log)
    handled = set()

    for k, window in enumerate(windows):
        if k in handled or window['instance'] not in receivers:
            continue

        t = window['start']
        name = f"{t:.1f} s: gps {window['type']} instance {window['instance'] + 1}" + \
               (f" ({window['toggles']} times)" if window['toggles'] > 1 else '') + \
               (f", cleared at {window['end']:.1f} s" if math.isfinite(window['end']) else '')

        # Every receiver failing at once is a total loss, not a failover
        together = [j for j, w in enumerate(windows) if abs(w['start'] - t) < 0.1 and w['instance'] in receivers]
        if len(receivers) > 1 and {windows[j]['instance'] for j in together} == receivers:
            handled.update(together)
            name = f"{t:.1f} s: gps {window['type']} on every receiver"
            grade_total_loss(log, vehicle, result, name, [windows[j] for j in together])
            graded_switches.update(s[0] for s in vehicle.switches(t, t + SETTLE_S))
            continue

        if window['instance'] == vehicle.selected(t):
            grade_failure(log, vehicle, result, name, window)
            graded_switches.update(s[0] for s in vehicle.switches(t)[:1])
        else:
            grade_standby(log, vehicle, result, name, window)
            graded_switches.update(s[0] for s in vehicle.switches(t, min(window['end'], t + SETTLE_S)))

        if not vehicle.armed(t):
            result.info(name, 'disarmed at the failure', '')

    # Switches with no injection behind them: a real receiver failure, a ranking, or a return on disarm
    for t, instance, reason in vehicle.switches(0.):
        if t not in graded_switches:
            result.info(f'{t:.1f} s: switch to instance {instance + 1}', 'reason',
                        reason_name(reason) + ('' if vehicle.armed(t) else ', disarmed'))

    if not result.rows:
        print('No GNSS injection or receiver switch in the log')
        return 0

    result.print()
    return 1 if result.failed else 0


if __name__ == '__main__':
    sys.exit(main())

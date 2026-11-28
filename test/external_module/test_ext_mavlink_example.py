#!/usr/bin/env python3
"""
SITL test for the out-of-tree MAVLink extension points.

Requires a build made with EXTERNAL_MODULES_LOCATION=test/external_module.
Starts PX4 with the SIH quadcopter and opens the GCS and gimbal UDP links, then checks:
  - EXT_EXAMPLE_STATUS streams at its registered 2 Hz (measured against the 1 Hz HEARTBEAT)
    and its status_seq is strictly increasing on every link
  - EXT_EXAMPLE_PING is answered by EXT_EXAMPLE_PONG with the same seq/payload on all links,
    so pongs_sent is an exact multiple of pings_received
  - SET_MESSAGE_INTERVAL on the GCS link stops, speeds up and restores the stream there
    while the gimbal link keeps streaming at the registered rate
"""

import argparse
import importlib.util
import os
import shutil
import socket
import subprocess
import sys
import time

PX4_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# PX4 SITL instance 0 links (ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink): (our port, PX4's port).
# The gimbal link carries only a handful of streams, so Python keeps up with it while it is not being read.
GCS_LINK = (14550, 18570)
GIMBAL_LINK = (13280, 13030)
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_RESULT_ACCEPTED = 0


class Failure(Exception):
    pass


def load_dialect(build_dir, work_dir):
    """Generate the Python bindings for the staged dialect and import them."""
    xml = os.path.join(build_dir, 'mavlink', 'message_definitions', 'v1.0', 'ext_example.xml')

    if not os.path.isfile(xml):
        raise Failure(f"{xml} missing: build with EXTERNAL_MODULES_LOCATION={PX4_DIR}/test/external_module")

    mavgen = os.path.join(PX4_DIR, 'src', 'modules', 'mavlink', 'mavlink', 'pymavlink', 'tools', 'mavgen.py')
    out = os.path.join(work_dir, 'ext_example')
    subprocess.check_call([sys.executable, mavgen, '--lang', 'Python3', '--wire-protocol', '2.0',
                           '--no-validate', '--output', out, xml], stdout=subprocess.DEVNULL)

    spec = importlib.util.spec_from_file_location('ext_example', out + '.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Link:
    """One MAVLink UDP link to a SITL mavlink instance, speaking the generated dialect."""

    def __init__(self, dialect, name, ports, mav_type, src_system):
        self.dialect = dialect
        self.name = name
        self.mav_type = mav_type
        self.remote = ('127.0.0.1', ports[1])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', ports[0]))
        self.sock.settimeout(0.1)
        self.mav = dialect.MAVLink(self, srcSystem=src_system, srcComponent=190)
        self.mav.robust_parsing = True
        self.last_status_seq = None

    def write(self, data):
        self.sock.sendto(data, self.remote)

    def close(self):
        self.sock.close()

    def heartbeat(self):
        self.mav.heartbeat_send(self.mav_type, self.dialect.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def messages(self, duration):
        """Yield messages received within `duration` seconds, checking status_seq never repeats or goes back."""
        deadline = time.monotonic() + duration

        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue

            for msg in self.mav.parse_buffer(data) or []:
                if msg.get_type() == 'EXT_EXAMPLE_STATUS':
                    if self.last_status_seq is not None and msg.status_seq <= self.last_status_seq:
                        raise Failure(f"{self.name}: status_seq {msg.status_seq} after {self.last_status_seq}")

                    self.last_status_seq = msg.status_seq

                yield msg

    def drain(self, duration=1.0):
        """Discard what was buffered while this link was not being read."""
        deadline = time.monotonic() + duration

        while time.monotonic() < deadline:
            try:
                self.sock.recvfrom(65535)
            except socket.timeout:
                return

    def connect(self, timeout):
        """PX4 replies to whoever it last heard from: announce ourselves until the autopilot HEARTBEAT arrives."""
        deadline = time.monotonic() + timeout

        while True:
            self.heartbeat()
            try:
                self.wait_for('HEARTBEAT', 1, lambda m: m.get_srcSystem() == 1)
                return
            except Failure:
                if time.monotonic() > deadline:
                    raise Failure(f"{self.name}: no autopilot HEARTBEAT within {timeout}s")

    def wait_for(self, name, timeout, predicate=lambda m: True):
        for msg in self.messages(timeout):
            if msg.get_type() == name and predicate(msg):
                return msg

        raise Failure(f"no {name} within {timeout}s")

    def count(self, name, duration):
        return sum(1 for m in self.messages(duration) if m.get_type() == name)

    def status_per_heartbeat(self, duration):
        """Ratio of EXT_EXAMPLE_STATUS to the autopilot's 1 Hz HEARTBEAT on this link.

        Both come from the same mavlink main loop, so the ratio is the stream
        rate in Hz regardless of how fast lockstep time runs on the host.
        """
        status = heartbeats = 0

        for msg in self.messages(duration):
            if msg.get_type() == 'EXT_EXAMPLE_STATUS':
                status += 1
            elif msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == 1:
                heartbeats += 1

        if heartbeats == 0:
            raise Failure(f"{self.name}: no autopilot HEARTBEAT in {duration}s")

        return status, status / heartbeats

    def set_message_interval(self, msg_id, interval_us):
        self.mav.command_long_send(1, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, interval_us, 0, 0, 0, 0, 0)
        ack = self.wait_for('COMMAND_ACK', 5, lambda m: m.command == MAV_CMD_SET_MESSAGE_INTERVAL)

        if ack.result != MAV_RESULT_ACCEPTED:
            raise Failure(f"SET_MESSAGE_INTERVAL({msg_id}, {interval_us}) rejected: result {ack.result}")


def start_px4(build_dir, work_dir, log):
    rootfs = os.path.join(work_dir, 'rootfs')
    shutil.rmtree(rootfs, ignore_errors=True)
    os.makedirs(rootfs)

    env = dict(os.environ, PX4_SIM_MODEL='sihsim_quadx')
    cmd = [os.path.join(build_dir, 'bin', 'px4'), os.path.join(build_dir, 'etc'),
           '-s', 'etc/init.d-posix/rcS', '-d']
    return subprocess.Popen(cmd, cwd=rootfs, env=env, stdout=log, stderr=subprocess.STDOUT)


def expect_range(what, value, low, high):
    if not low <= value <= high:
        raise Failure(f"{what}: got {value}, expected {low}..{high}")


def run(gcs, gimbal, dialect, verbose):
    def log(text):
        if verbose:
            print(text, flush=True)

    status_id = dialect.MAVLINK_MSG_ID_EXT_EXAMPLE_STATUS
    pings = 5

    gcs.connect(120)
    gimbal.connect(30)
    log("autopilot heartbeat received on both links")

    # Registered interval 500 ms.
    first = gcs.wait_for('EXT_EXAMPLE_STATUS', 10)
    n, rate = gcs.status_per_heartbeat(4)
    expect_range("GCS EXT_EXAMPLE_STATUS rate at default (Hz per HEARTBEAT)", rate, 1.2, 3.0)
    log(f"default rate: {n} status messages, {rate:.1f} Hz (status_seq started at {first.status_seq})")

    # Handler + one-shot reply: the PONG must reach every link, so both links see each seq.
    for seq in range(1, pings + 1):
        payload = [(seq * 7 + i) & 0xFF for i in range(16)]
        gcs.mav.ext_example_ping_send(1, 1, seq, payload)

        for link in (gcs, gimbal):
            pong = link.wait_for('EXT_EXAMPLE_PONG', 3, lambda m, seq=seq: m.seq == seq)

            if list(pong.payload) != payload or pong.payload_sum != sum(payload):
                raise Failure(f"{link.name} PONG {seq}: payload {list(pong.payload)} sum {pong.payload_sum}, "
                              f"expected {payload} sum {sum(payload)}")

    # One PONG per running mavlink instance, so pongs_sent is an exact multiple of pings_received.
    status = gcs.wait_for('EXT_EXAMPLE_STATUS', 3, lambda m: m.pings_received == pings)

    if status.pongs_sent % pings != 0 or status.pongs_sent // pings < 2:
        raise Failure(f"pongs_sent {status.pongs_sent} is not pings_received ({pings}) times the link count")

    links = status.pongs_sent // pings
    log(f"{pings} pings answered on {links} links (pongs_sent {status.pongs_sent})")

    # Per-link rate control: SET_MESSAGE_INTERVAL on the GCS link must leave the gimbal link alone.
    gcs.set_message_interval(status_id, -1)
    gcs.count('EXT_EXAMPLE_STATUS', 0.5)  # drain in-flight
    n, _ = gcs.status_per_heartbeat(3)
    expect_range("GCS EXT_EXAMPLE_STATUS while disabled", n, 0, 0)
    gimbal.drain()
    n, rate = gimbal.status_per_heartbeat(4)
    expect_range("gimbal EXT_EXAMPLE_STATUS rate while GCS disabled (Hz per HEARTBEAT)", rate, 1.2, 3.0)
    log(f"GCS stream disabled; gimbal link still {n} status messages, {rate:.1f} Hz")

    gcs.set_message_interval(status_id, 100000)
    n, rate = gcs.status_per_heartbeat(4)
    expect_range("GCS EXT_EXAMPLE_STATUS rate at 100 ms (Hz per HEARTBEAT)", rate, 6.0, 14.0)
    log(f"100 ms interval: {n} status messages, {rate:.1f} Hz")

    gcs.set_message_interval(status_id, 0)
    gcs.count('EXT_EXAMPLE_STATUS', 0.5)
    n, rate = gcs.status_per_heartbeat(4)
    expect_range("GCS EXT_EXAMPLE_STATUS rate after restoring default (Hz per HEARTBEAT)", rate, 1.2, 3.0)
    log(f"default restored: {n} status messages, {rate:.1f} Hz")

    status = gcs.wait_for('EXT_EXAMPLE_STATUS', 3)

    if status.pings_received != pings or status.pongs_sent != pings * links:
        raise Failure(f"counters changed: pings_received {status.pings_received}, pongs_sent {status.pongs_sent}")

    log(f"final counters: pings_received {status.pings_received}, pongs_sent {status.pongs_sent}, "
        f"status_seq {status.status_seq}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--build-dir', default=os.path.join(PX4_DIR, 'build', 'px4_sitl_default'))
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    build_dir = os.path.abspath(args.build_dir)
    work_dir = os.path.join(build_dir, 'ext_module_test')
    os.makedirs(work_dir, exist_ok=True)

    px4 = None
    links = []

    try:
        dialect = load_dialect(build_dir, work_dir)
        gcs = Link(dialect, 'gcs', GCS_LINK, dialect.MAV_TYPE_GCS, 255)
        gimbal = Link(dialect, 'gimbal', GIMBAL_LINK, dialect.MAV_TYPE_ONBOARD_CONTROLLER, 1)
        links = [gcs, gimbal]

        with open(os.path.join(work_dir, 'px4.log'), 'w') as log:
            px4 = start_px4(build_dir, work_dir, log)
            run(gcs, gimbal, dialect, args.verbose)

        print("PASS: external module MAVLink handler, one-shot send and per-link stream")
        return 0

    except (Failure, subprocess.CalledProcessError) as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 1

    finally:
        for link in links:
            link.close()

        if px4 and px4.poll() is None:
            px4.terminate()

            try:
                px4.wait(10)
            except subprocess.TimeoutExpired:
                px4.kill()


if __name__ == '__main__':
    sys.exit(main())

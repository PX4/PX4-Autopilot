#!/usr/bin/env python3
"""Exercise the real PTGL-100 SITL driver through a virtual UART.

Requires pexpect, pymavlink and pyulog. No hardware is opened or flashed.
"""

# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import argparse
import os
from pathlib import Path
import re
import select
import socket
import tempfile
import termios
import threading
import time

import pexpect
from pymavlink import mavutil
from pyulog import ULog


START = bytes.fromhex("55 aa 69 ff ff ff ff 65")
STOP = bytes.fromhex("55 aa 8e ff ff ff ff 8a")


def reply(command=0x69, status=1, distance=12300):
    data = bytes((0x55, 0xAA, command, status, distance >> 16, (distance >> 8) & 0xFF, distance & 0xFF))
    return data + bytes((sum(data) & 0xFF,))


def wait_for(predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("timed out waiting for virtual sensor state")


class Sensor:
    def __init__(self):
        self.master, self.slave = os.openpty()
        os.set_blocking(self.master, False)
        self.path = os.ttyname(self.slave)
        self.ready = False
        self.streaming = False
        self.mode = "valid"
        self.distance = 12300
        self.start_times = []
        self.stop_count = 0
        self.restart_requested = False
        self.error = None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        received = bytearray()
        pending = b""
        next_sample = time.monotonic()
        try:
            while not self.done.wait(0.002):
                if select.select([self.master], [], [], 0)[0]:
                    received.extend(os.read(self.master, 256))
                while len(received) >= 8:
                    frame = bytes(received[:8])
                    del received[:8]
                    assert frame in (START, STOP), f"unexpected command: {frame.hex()}"
                    if frame == START:
                        self.start_times.append(time.monotonic())
                        self.streaming = self.ready
                    else:
                        self.stop_count += 1
                        self.streaming = False

                if self.restart_requested:
                    self.restart_requested = False
                    self.streaming = False
                    # Power-on self-test success, including the header in checksum.
                    pending = bytes.fromhex("55 aa 80 01 00 00 00 80")

                if pending:
                    written = os.write(self.master, pending)
                    pending = pending[written:]
                    continue

                now = time.monotonic()
                if not self.streaming or now < next_sample:
                    continue
                next_sample = now + 0.01
                if self.mode == "silent":
                    continue
                frame = reply(status=0 if self.mode == "failed" else 1, distance=self.distance)
                if self.mode == "corrupt":
                    frame = frame[:-1] + bytes((frame[-1] ^ 1,))
                elif self.mode == "control":
                    frame = reply(command=0x8E, status=0, distance=0xFFFFFF)
                elif self.mode == "legacy":
                    # Old-format frames must never be interpreted as millimetres.
                    frame = bytes.fromhex("55 aa 89 01 ff 01 2c b5")
                elif self.mode == "noise":
                    # Truncated prefix, garbage and two back-to-back valid frames.
                    frame = b"\x55\xaa\x69\x00\x55" + frame + frame
                # Deliberately split frames across driver reads.
                written = os.write(self.master, frame[:3])
                pending = frame[written:]
        except BaseException as error:  # Propagate background failures to the test.
            self.error = error

    def close(self):
        self.done.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


class Console:
    def __init__(self, executable, directory, transcript):
        startup = directory / "startup.sh"
        startup.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        env = dict(os.environ, TERM="dumb")
        self.process = pexpect.spawn(
            str(executable),
            ["-s", str(startup), "-w", str(directory), "-i", str(os.getpid())],
            encoding="utf-8", timeout=15, env=env,
        )
        self.process.logfile = transcript
        self.process.expect_exact("pxh> ")

    def command(self, command):
        self.process.sendline(command)
        # PXH redraws its prompt after each input character. Wait for the full
        # echoed line first, then for the prompt after command completion.
        self.process.expect_exact(command + "\r\n")
        self.process.expect_exact("pxh> ")
        return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", self.process.before)

    def samples(self):
        status = self.command("ptgl100 status")
        match = re.search(r"ptgl100: samples: (\d+) events", status)
        assert match, status
        return int(match[1])

    def open_sensor_descriptors(self, path):
        count = 0
        for descriptor in Path(f"/proc/{self.process.pid}/fd").iterdir():
            try:
                count += os.readlink(descriptor) == path
            except FileNotFoundError:
                pass  # Another module may close a descriptor while enumerating.
        return count

    def distance(self, expected, quality=-1):
        output = self.command("listener distance_sensor -n 1")
        match = re.search(r"current_distance: ([\d.]+)", output)
        assert match and abs(float(match[1]) - expected) < 0.001, output
        assert re.search(rf"signal_quality: {quality}\b", output), output
        assert re.search(r"orientation: 25\b", output), output
        assert re.search(r"min_distance: 0\.200", output), output
        assert re.search(r"max_distance: 100\.000", output), output

    def close(self):
        if self.process.isalive():
            self.process.sendline("shutdown")
            try:
                self.process.expect(pexpect.EOF, timeout=10)
            except pexpect.TIMEOUT:
                self.process.terminate(force=True)
        self.process.close()


def run(executable, directory):
    sensor = Sensor()
    console = None
    link = None
    # A predictable UART suffix also exercises the complete DeviceId bus number.
    serial_path = directory / "ttyS29"
    serial_path.symlink_to(sensor.path)
    transcript = (directory / "console.txt").open("w", encoding="utf-8")
    try:
        console = Console(executable, directory, transcript)
        console.command("param set SDLOG_PROFILE 1")
        output = console.command(f"ptgl100 start -d {serial_path} -R 25")
        assert "Task start failed" not in output, output
        wait_for(lambda: len(sensor.start_times) >= 2)
        assert console.samples() == 0
        assert all(b - a >= 0.95 for a, b in zip(sensor.start_times, sensor.start_times[1:]))

        config = termios.tcgetattr(sensor.slave)
        assert config[4] == config[5] == termios.B460800
        assert config[2] & termios.CSIZE == termios.CS8
        assert not config[2] & (termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        assert console.open_sensor_descriptors(sensor.path) == 1
        print("PASS: 460800 8N1, delayed power-on and rate-limited retries")

        sensor.ready = True
        wait_for(lambda: sensor.streaming)
        time.sleep(0.3)
        console.distance(12.3)
        assert console.samples() >= 10
        console.command("logger start -f -m file")
        print("PASS: fragmented 100 Hz measurements publish distance_sensor")

        # Use normal MAVLink mode: DISTANCE_SENSOR must appear without a stream command.
        link = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        remote_port = link.port.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            local_port = probe.getsockname()[1]
        console.command("dataman start")
        console.command(f"mavlink start -u {local_port} -o {remote_port} -t 127.0.0.1 -m 0 -r 100000")
        console.command("mavlink boot_complete")
        message = link.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=8)
        assert message is not None, "default MAVLink DISTANCE_SENSOR stream missing"
        assert abs(message.current_distance - 1230) <= 1, message
        assert message.min_distance == 20 and message.max_distance == 10000, message
        assert message.type == 0 and message.orientation == 25, message
        print("PASS: default MAVLink stream, units, limits and orientation")

        for mode in ("failed", "corrupt", "control", "legacy", "silent"):
            sensor.mode = mode
            time.sleep(0.1)
            before = console.samples()
            starts_before = len(sensor.start_times)
            time.sleep(1.15)
            assert console.samples() == before, f"{mode} refreshed distance"
            if mode == "failed":
                assert len(sensor.start_times) == starts_before, "failure replies triggered restart"
            else:
                assert len(sensor.start_times) > starts_before, f"{mode} did not trigger retry"
            sensor.mode = "valid"
            time.sleep(0.2)
            assert console.samples() > before
        print("PASS: failed/corrupt/control/legacy replies and outages never republish stale distance")

        sensor.mode = "noise"
        sensor.distance = 23400
        time.sleep(0.2)
        console.distance(23.4)
        sensor.mode = "valid"
        print("PASS: noise, truncation and concatenated frame recovery")

        for raw, expected, quality in (
            (199, 0.199, 0), (200, 0.2, -1), (65535, 65.535, -1),
            (65536, 65.536, -1), (100000, 100.0, -1), (100001, 100.001, 0),
        ):
            sensor.distance = raw
            time.sleep(0.1)
            console.distance(expected, quality)

        # Keep a value requiring the third byte long enough for both output paths.
        sensor.distance = 100000
        time.sleep(1.2)
        message = link.recv_match(
            type="DISTANCE_SENSOR", condition="DISTANCE_SENSOR.current_distance == 10000",
            blocking=True, timeout=8,
        )
        assert message is not None, "24-bit 100 m measurement missing from MAVLink"
        sensor.distance = 12300

        starts_before = len(sensor.start_times)
        sensor.restart_requested = True
        wait_for(lambda: len(sensor.start_times) > starts_before)
        wait_for(lambda: sensor.streaming)
        time.sleep(0.15)
        console.distance(12.3)
        print("PASS: range validity and module power-cycle recovery")

        before = console.samples()
        console.command(f"ptgl100 start -d {serial_path}")
        assert "460800" in console.command("ptgl100 status")
        assert console.samples() >= before
        for _ in range(3):
            stops_before = sensor.stop_count
            console.command("ptgl100 stop")
            wait_for(lambda: sensor.stop_count > stops_before)
            assert console.open_sensor_descriptors(sensor.path) == 0
            console.command(f"ptgl100 start -d {serial_path}")
            time.sleep(0.15)
            assert console.open_sensor_descriptors(sensor.path) == 1
            console.distance(12.3)
        console.command("ptgl100 stop")
        print("PASS: duplicate start protection, stop command and repeated start/stop")

        console.command("logger stop")
        console.command("mavlink stop-all")
        console.command("dataman stop")
        logs = list(directory.rglob("*.ulg"))
        assert logs, f"ULog missing from {directory}"
        datasets = [data for log in logs for data in ULog(str(log)).data_list if data.name == "distance_sensor"]
        assert datasets, "distance_sensor missing from default ULog profile"
        assert any(any(abs(value - 12.3) < 0.001 for value in data.data["current_distance"]) for data in datasets)
        assert any(any(abs(value - 100.0) < 0.001 for value in data.data["current_distance"]) for data in datasets)
        assert all((int(value) >> 16) & 0xFF == 0xF4 for data in datasets for value in data.data["device_id"])
        assert all((int(value) >> 3) & 0x1F == 29 for data in datasets for value in data.data["device_id"])
        print("PASS: default ULog contains measurements and the PTGL-100 serial device ID")
        assert sensor.error is None, sensor.error
    finally:
        if console:
            console.close()
        if link:
            link.close()
        sensor.close()
        transcript.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="Keep console transcript and ULog in this new directory")
    args = parser.parse_args()
    executable = args.px4.resolve(strict=True)
    if args.output:
        directory = args.output.resolve()
        directory.mkdir(parents=True, exist_ok=False)
        run(executable, directory)
        print(f"Test artefacts: {directory}")
    else:
        with tempfile.TemporaryDirectory(prefix="ptgl100-test-") as temporary:
            run(executable, Path(temporary))


if __name__ == "__main__":
    main()

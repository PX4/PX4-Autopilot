#!/usr/bin/env python3
"""Exercise the actual driver over a pseudo-UART (requires pymavlink and SITL)."""
import argparse
import os
from pathlib import Path
import pty
import re
import subprocess
import tempfile
import threading
import time

from pymavlink.dialects.v20 import common as mavlink

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--build", default="build/px4_sitl_nolockstep")
parser.add_argument("--instance", type=int, default=79)
args = parser.parse_args()
root = Path(__file__).resolve().parents[4]
binary_dir = (root / args.build / "bin").resolve()
instance = str(args.instance)


def command(name, *arguments, check=True):
    result = subprocess.run(
        [str(binary_dir / ("px4-" + name)), "--instance", instance, *map(str, arguments)],
        capture_output=True, text=True, timeout=8,
    )
    if check and result.returncode:
        raise AssertionError(result.stdout + result.stderr)
    return result


def topic(name):
    return command("listener", name, "-n", "1").stdout


def require(text, pattern):
    if not re.search(pattern, text):
        raise AssertionError(f"Missing {pattern!r} in:\n{text}")


with tempfile.TemporaryDirectory(prefix="anello-sitl-") as directory:
    work = Path(directory)
    startup = work / "start.sh"
    startup.write_text("#!/bin/sh\nexit 0\n")
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    logfile = (work / "px4.log").open("w+")
    process = subprocess.Popen(
        [str(binary_dir / "px4"), "-d", "-i", instance, "-w", directory, "-s", str(startup)],
        cwd=directory, stdout=logfile, stderr=subprocess.STDOUT,
    )
    stopped = threading.Event()
    stream = {"attitude": True, "position": True, "lat": 370000000, "alt": 123450}
    thread = None
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if process.poll() is not None:
                logfile.seek(0)
                raise AssertionError(logfile.read())
            # Wait for our own server startup, never configure an existing instance.
            if "Startup script returned successfully" in (work / "px4.log").read_text():
                break
            time.sleep(0.1)
        else:
            raise AssertionError("PX4 did not start")

        def param(name, value):
            command("param", "set", name, value)

        param("ANELLO_BAUD", 921600)
        # Default EKF configuration must prevent accidental primary takeover.
        param("ANELLO_MODE", 1)
        param("EKF2_EN", 1)
        result = command("anello", "start", "-d", device, check=False)
        assert result.returncode != 0
        require(result.stdout + result.stderr, "primary requires")

        param("ANELLO_MODE", 0)
        command("anello", "start", "-d", device)

        def send():
            encoder = mavlink.MAVLink(None, srcSystem=42, srcComponent=1)
            while not stopped.is_set():
                milliseconds = int(time.monotonic() * 1000) & 0xFFFFFFFF
                messages = []
                if stream["attitude"]:
                    messages.append(mavlink.MAVLink_attitude_quaternion_message(
                        milliseconds, 1., 0., 0., 0., 0., 0., 0.))
                if stream["position"]:
                    messages.append(mavlink.MAVLink_global_position_int_message(
                        milliseconds, stream["lat"], -1220000000,
                        stream["alt"], 999999, 123, -234, 345, 65535))
                for message in messages:
                    os.write(master, message.pack(encoder))
                    encoder.seq = (encoder.seq + 1) & 255
                stopped.wait(0.02)

        thread = threading.Thread(target=send, daemon=True)
        thread.start()
        time.sleep(0.3)
        require(topic("external_ins_attitude"), r"q: \[1\.00000")
        require(topic("external_ins_local_position"), r"xy_valid: False")
        require(topic("vehicle_attitude"), r"never published")
        command("anello", "stop")

        for name, value in [
            ("ANELLO_MODE", 1), ("EKF2_EN", 0), ("ATT_EN", 0),
            ("SENS_IMU_MODE", 1), ("ANELLO_EPH", 2),
            ("ANELLO_EPV", 3), ("ANELLO_EV", 0.5),
        ]:
            param(name, value)

        command("anello", "start", "-d", device)
        time.sleep(0.3)
        require(topic("vehicle_attitude"), r"q: \[1\.00000")
        local = topic("vehicle_local_position")
        require(local, r"xy_valid: True")
        require(local, r"vz: 3\.450")
        require(local, r"z: 0\.000")
        require(topic("vehicle_global_position"), r"alt: 123\.450")
        stream["lat"] += 1000
        stream["alt"] += 1000
        time.sleep(0.15)
        local = topic("vehicle_local_position")
        require(local, r"x: 11\.1")
        require(local, r"z: -1\.000")

        # Loss of attitude must invalidate both attitude and navigation.
        stream["attitude"] = False
        time.sleep(0.35)
        require(topic("vehicle_attitude"), r"q: \[0\.00000, 0\.00000, 0\.00000, 0\.00000\]")
        require(topic("vehicle_local_position"), r"xy_valid: False")
        require(topic("estimator_status"), r"pre_flt_fail_innov_heading: True")

        stream["attitude"] = True
        time.sleep(0.15)
        require(topic("vehicle_local_position"), r"xy_valid: True")
        # Position loss must not be hidden by a continuing attitude stream.
        stream["position"] = False
        time.sleep(1.1)
        require(topic("vehicle_attitude"), r"q: \[1\.00000")
        require(topic("vehicle_global_position"), r"lat_lon_valid: False")
        require(topic("vehicle_local_position"), r"v_xy_valid: False")
        stream["position"] = True
        time.sleep(0.15)
        local = topic("vehicle_local_position")
        require(local, r"xy_valid: True")
        require(local, r"x: 11\.1")  # Origin survives stream loss.
        require(local, r"z: -1\.000")
        require(command("anello", "status").stdout, r"read errors: 0, last errno: 0")
        command("anello", "stop")
        # Existing vehicle advertisements prevent a silent estimator takeover.
        result = command("anello", "start", "-d", device, check=False)
        assert result.returncode != 0
        require(result.stdout + result.stderr, "already advertised")
        print("ANELLO SITL serial integration tests passed")
    except Exception:
        print(command("anello", "status", check=False).stdout)
        print(command("work_queue", "status", check=False).stdout)
        logfile.flush()
        logfile.seek(0)
        print(logfile.read())
        raise
    finally:
        stopped.set()
        if thread:
            thread.join(timeout=2)
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)
        logfile.close()

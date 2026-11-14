#!/usr/bin/env python3
"""Linux SITL reboot regression.

Run: python3 test/test_sitl_reboot.py [path/to/px4].

Requires pymavlink. Starts an isolated instance with temporary
parameters; no running simulator or hardware is used.
"""

import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil


def unused_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def run(binary):
    instance = 77
    with tempfile.TemporaryDirectory(prefix="sitl-reboot-") as directory:
        root = Path(directory)
        work = root / "rootfs"
        work.mkdir()
        mav = mavutil.mavlink_connection(
            "udpin:127.0.0.1:0", source_system=250)
        receive_port = mav.port.getsockname()[1]
        (work / "rcS").write_text(
            ". px4-alias.sh\n"
            "param select params.bson\n"
            "[ ! -f params.bson ] || param import\n"
            "simulator_sih start\n"
            "dataman start\n"
            "commander start\n"
            f"mavlink start -u {unused_port()} -o {receive_port} "
            "-t 127.0.0.1\n"
            "mavlink boot_complete\n"
            "echo boot >> boots\n"
        )
        with (root / "px4.log").open("w+") as log:
            # Relative executable, working directory and script exercise
            # launch-context restoration.
            process = subprocess.Popen(
                [os.path.relpath(binary, root), "-d", "-i", str(instance),
                 "-w", "rootfs", "-s", "rcS"],
                cwd=root, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
            )

            def client(command, *args):
                return subprocess.check_output(
                    [str(binary.with_name("px4-" + command)),
                     "--instance", str(instance), *args],
                    text=True, timeout=10,
                )

            def heartbeat():
                message = mav.wait_heartbeat(timeout=15)
                assert message is not None, "No heartbeat"
                return message

            def command(command_id, *params):
                while mav.recv_match(blocking=False) is not None:
                    pass
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component, command_id, 0,
                    *(list(params) + [0] * (7 - len(params))),
                )
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    ack = mav.recv_match(
                        type="COMMAND_ACK", blocking=True, timeout=1)
                    if ack is not None and ack.command == command_id:
                        return ack.result
                raise AssertionError(f"No acknowledgement for {command_id}")

            try:
                assert not (
                    heartbeat().base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                reboot = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                assert command(reboot, 3) == mavutil.mavlink.MAV_RESULT_DENIED
                # Force-arm this isolated instance to test the armed guard.
                arm = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                client("commander", "arm", "-f")
                assert (
                    heartbeat().base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                assert command(reboot, 1) == mavutil.mavlink.MAV_RESULT_DENIED
                assert command(
                    arm, 0, 21196) == mavutil.mavlink.MAV_RESULT_ACCEPTED
                client("param", "set", "COM_RC_LOSS_T", "9")
                client("param", "save")
                fd_count = len(list(Path(f"/proc/{process.pid}/fd").iterdir()))

                for boot in range(2, 5):
                    if boot == 4:
                        # An open MAVLink console redirects stdout/stderr.
                        data = b"ver all\n"
                        mav.mav.serial_control_send(
                            mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                            mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND, 0, 0,
                            len(data), list(data.ljust(70, b"\0")),
                        )
                        assert mav.recv_match(
                            type="SERIAL_CONTROL", blocking=True, timeout=5)
                    assert command(
                        reboot, 1) == mavutil.mavlink.MAV_RESULT_ACCEPTED
                    deadline = time.monotonic() + 20
                    while (
                        len((work / "boots").read_text().splitlines()) < boot
                    ):
                        assert process.poll() is None, (
                            "PX4 exited instead of restarting"
                        )
                        assert time.monotonic() < deadline, (
                            "PX4 did not restart"
                        )
                        time.sleep(0.1)
                    assert not (
                        heartbeat().base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    assert float(
                        client("param", "show", "-q", "COM_RC_LOSS_T")) == 9
                    assert (
                        len(list(Path(f"/proc/{process.pid}/fd").iterdir()))
                        <= fd_count
                    )
                    print(
                        f"PASS: boot {boot}, same PID, parameters retained, "
                        "MAVLink reconnected"
                    )

                log.seek(0)
                assert log.read().count("px4 starting.") == 4, (
                    "Console output was not restored"
                )
                assert command(
                    reboot, 2) == mavutil.mavlink.MAV_RESULT_ACCEPTED
                assert process.wait(timeout=10) == 0
                print(
                    "PASS: bootloader/armed reboot denied; "
                    "ordinary shutdown still exits"
                )
            except BaseException:
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                mav.close()
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == "__main__":
    run(Path(
        sys.argv[1] if len(sys.argv) > 1
        else "build/px4_sitl_default/bin/px4"
    ).resolve())

"""AutopilotTester for sensor calibration on SIH.

`simulator_sih orient`, sent over the MAVLink shell, poses the vehicle the way a person would
during accelerometer and magnetometer calibration. The tester follows the calibration like a
GCS, through the `[cal]` status texts and the command acks.
"""

import json
import threading
from typing import Any, List, Optional

from mavsdk.plugins.shell import Device, Shell  # type: ignore

from autopilot_tester import MAV_COMP_ID_AUTOPILOT1, MAV_RESULT_ACCEPTED, AutopilotTester, log

MAV_CMD_PREFLIGHT_CALIBRATION = 241
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8

# PX4 only sends STATUSTEXT while it hears a GCS. MAVSDK's 1 Hz heartbeat is in host time, which at
# a high speed factor is longer than PX4's GCS timeout in simulation time.
HEARTBEAT_INTERVAL_S = 0.05

START_ACK_TIMEOUT_S = 5.0


class AutopilotTesterCalibration(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._shell: Any = None
        self._lock = threading.Lock()
        self._acks: List[int] = []
        self._texts: List[str] = []
        self._heartbeat_thread = threading.Thread(target=self._send_heartbeats, daemon=True)

    def connect(self, url: str) -> None:
        super().connect(url)
        self._heartbeat_thread.start()
        self._shell = Shell(self._system)
        # Without a subscriber MAVSDK does not ask for a response, and PX4 closes the shell
        # before it has run the command.
        self._shell.subscribe_receive(self._on_shell_output)
        self._mavlink_direct.subscribe_message("COMMAND_ACK", self._on_ack)
        self._telemetry.subscribe_status_text(self._on_status_text)

    def close(self) -> List[str]:
        self._should_exit.set()
        if self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join()
        return super().close()

    def _send_heartbeats(self) -> None:
        while not self._should_exit.wait(HEARTBEAT_INTERVAL_S):
            self.send_mavlink_message("HEARTBEAT", {
                "type": MAV_TYPE_GCS, "autopilot": MAV_AUTOPILOT_INVALID, "base_mode": 0,
                "custom_mode": 0, "system_status": 0, "mavlink_version": 3})

    def _on_shell_output(self, receive: Any, _user_data: Any) -> None:
        for line in receive.data.splitlines():
            if line.strip():
                log("Shell: {}".format(line.strip()))

    def _on_ack(self, message: Any, _user_data: Any) -> None:
        ack = json.loads(message.fields_json)
        if ack["command"] == MAV_CMD_PREFLIGHT_CALIBRATION:
            with self._lock:
                self._acks.append(int(ack["result"]))

    def _on_status_text(self, status_text: Any, _user_data: Any) -> None:
        text = str(status_text.text)
        if text.startswith("[cal]"):
            log(text)
            with self._lock:
                self._texts.append(text)

    def shell(self, command: str) -> None:
        log("Shell: {}".format(command))
        self._shell.send(command + "\n", Device.SHELL)

    def start_calibration(self, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0,
                          param4: float = 0.0, param5: float = 0.0, param6: float = 0.0,
                          param7: float = 0.0) -> None:
        """Send one start, without retries: PX4 rejects a retry once the calibration runs, and
        that late ack would be counted against the cancel."""
        self._send_calibration_command(param1, param2, param3, param4, param5, param6, param7)
        assert self.poll_condition_with_timeout(lambda: bool(self.calibration_acks()),
                                                START_ACK_TIMEOUT_S), \
            "No ack for the calibration start"
        assert self.calibration_acks() == [MAV_RESULT_ACCEPTED]

    def cancel_calibration(self) -> None:
        """Send one cancel, without retries, so its acks can be counted."""
        self._send_calibration_command()

    def _send_calibration_command(self, param1: float = 0.0, param2: float = 0.0,
                                  param3: float = 0.0, param4: float = 0.0, param5: float = 0.0,
                                  param6: float = 0.0, param7: float = 0.0) -> None:
        """Forget earlier acks and send the command once."""
        with self._lock:
            self._acks.clear()

        self.send_mavlink_message("COMMAND_LONG", {
            "target_system": self._target_system_id(),
            "target_component": MAV_COMP_ID_AUTOPILOT1,
            "command": MAV_CMD_PREFLIGHT_CALIBRATION, "confirmation": 0,
            "param1": param1, "param2": param2, "param3": param3, "param4": param4,
            "param5": param5, "param6": param6, "param7": param7,
        })

    def calibration_acks(self) -> List[int]:
        with self._lock:
            return list(self._acks)

    def has_text(self, text: str) -> bool:
        with self._lock:
            return any(text in t for t in self._texts)

    def wait_for_text(self, text: str, timeout_s: float) -> None:
        assert self.poll_condition_with_timeout(lambda: self.has_text(text), timeout_s), \
            "No '{}' within {} s".format(text, timeout_s)

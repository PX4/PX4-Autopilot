#!/usr/bin/env python3
"""
acceleration_control.py
=======================
Python API for sending external NED acceleration setpoints to PX4
in Position Mode or Altitude Mode (no OFFBOARD mode required).

Protocol
--------
Uses MAVLink SET_POSITION_TARGET_LOCAL_NED with bit 12 set in type_mask
to signal "apply in current mode". The PX4 firmware side publishes the
acc_sp_external uORB topic which FlightTaskManualAltitude and
FlightTaskManualAcceleration subscribe to.

Usage example
-------------
    from acceleration_control import AccelerationControl

    ac = AccelerationControl('udp://:14550')
    ac.connect()
    ac.arm()
    ac.set_position_mode()          # or set_altitude_mode()

    # Fly forward (North) at 2 m/s²
    for _ in range(50):             # 50 * 0.02 s = 1 s
        ac.send_acceleration_ned(ax=2.0, ay=0.0, az=0.0)
        time.sleep(0.02)

    ac.hover()                      # zero acceleration
    ac.land()
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

try:
    from pymavlink import mavutil
except ImportError as e:
    raise ImportError("pymavlink is required: pip install pymavlink") from e


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Bit 12 in type_mask: "apply in current mode" (custom PX4 extension)
_ACC_SP_EXTERNAL_FLAG = 1 << 12

# type_mask: ignore position (bits 0-2), ignore velocity (bits 3-5),
# use acceleration (bits 6-8 cleared), ignore yaw rate (bit 10)
_TYPE_MASK_ACC_ONLY = (
    (1 << 0) | (1 << 1) | (1 << 2)   # ignore x, y, z position
    | (1 << 3) | (1 << 4) | (1 << 5)  # ignore vx, vy, vz
    # bits 6-8 clear = use acceleration fields
    # bit 9 (FORCE_SET) must be clear — setting it triggers rejection
    | (1 << 10)                        # YAW_IGNORE (hold current heading)
    | (1 << 11)                        # YAW_RATE_IGNORE
    | _ACC_SP_EXTERNAL_FLAG            # apply outside OFFBOARD
)

_TYPE_MASK_ACC_WITH_YAW = (
    (1 << 0) | (1 << 1) | (1 << 2)
    | (1 << 3) | (1 << 4) | (1 << 5)
    # bit 10 (YAW_IGNORE) clear = use provided yaw value
    | (1 << 11)                        # YAW_RATE_IGNORE
    | _ACC_SP_EXTERNAL_FLAG
)

# MAVLink well-known constants (avoid dependency on mavutil at call time)
_MAV_FRAME_LOCAL_NED = 1  # MAV_FRAME_LOCAL_NED per MAVLink spec

# PX4 custom mode values
_PX4_MAIN_MODE_AUTO     = 4
_PX4_MAIN_MODE_POSCTL   = 3
_PX4_MAIN_MODE_ALTCTL   = 2
_PX4_CUSTOM_SUB_MODE_POSCTL = 0
_PX4_CUSTOM_SUB_MODE_ALTCTL = 0


@dataclass
class AccelerationNed:
    """NED frame acceleration setpoint (m/s²)."""
    north_m_s2: float = 0.0
    east_m_s2:  float = 0.0
    down_m_s2:  float = 0.0
    yaw_rad:    Optional[float] = None  # None = hold current yaw


# ---------------------------------------------------------------------------
# AccelerationControl class
# ---------------------------------------------------------------------------

class AccelerationControl:
    """
    Sends external NED acceleration setpoints to PX4 Position/Altitude mode.

    Parameters
    ----------
    connection_string : str
        pymavlink connection string, e.g. 'udp://:14550' or
        'serial:///dev/ttyUSB0:921600'
    source_system : int
        MAVLink system ID of this GCS (default 255)
    """

    def __init__(self, connection_string: str = 'udp://:14550',
                 source_system: int = 255):
        self._conn_str = connection_string
        self._source_system = source_system
        self._mav: Optional[mavutil.mavfile] = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, timeout_s: float = 10.0) -> None:
        """Open MAVLink connection and wait for heartbeat."""
        self._mav = mavutil.mavlink_connection(
            self._conn_str,
            source_system=self._source_system
        )
        print(f"[AccCtrl] Connecting to {self._conn_str} ...")
        self._mav.wait_heartbeat(timeout=timeout_s)
        print(f"[AccCtrl] Heartbeat received from system "
              f"{self._mav.target_system} component "
              f"{self._mav.target_component}")

    def disconnect(self) -> None:
        if self._mav:
            self._mav.close()
            self._mav = None

    # ------------------------------------------------------------------
    # Vehicle commands
    # ------------------------------------------------------------------

    def set_param_float(self, name: str, value: float) -> None:
        """Set a PX4 float/int parameter via MAVLink PARAM_SET."""
        if self._mav is None:
            raise RuntimeError("Not connected.")
        self._mav.mav.param_set_send(
            self._mav.target_system,
            self._mav.target_component,
            name.encode(),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def arm(self, wait_s: float = 3.0) -> None:
        """Arm the vehicle."""
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )
        time.sleep(wait_s)
        print("[AccCtrl] Armed")

    def disarm(self) -> None:
        """Disarm the vehicle."""
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        print("[AccCtrl] Disarmed")

    def takeoff(self, altitude_m: float = 10.0) -> None:
        """Switch to AUTO mode and command takeoff to target altitude (m AGL).
        Does not wait for completion — caller must poll altitude.
        """
        # PX4 requires AUTO mode for MAV_CMD_NAV_TAKEOFF to be accepted
        self._set_flight_mode(_PX4_MAIN_MODE_AUTO, 0)
        time.sleep(1.0)
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            param7=float(altitude_m)
        )
        print(f"[AccCtrl] Takeoff commanded to {altitude_m:.0f}m")

    def land(self) -> None:
        """Command landing."""
        self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)
        print("[AccCtrl] Land commanded")

    def set_position_mode(self, wait_s: float = 1.0) -> None:
        """Switch to Position Mode (requires GPS/VIO)."""
        self._set_flight_mode(_PX4_MAIN_MODE_POSCTL, 0)
        time.sleep(wait_s)
        print("[AccCtrl] Position Mode set")

    def set_altitude_mode(self, wait_s: float = 1.0) -> None:
        """Switch to Altitude Mode (barometer only, no GPS required)."""
        self._set_flight_mode(_PX4_MAIN_MODE_ALTCTL, 0)
        time.sleep(wait_s)
        print("[AccCtrl] Altitude Mode set")

    # ------------------------------------------------------------------
    # Acceleration setpoint
    # ------------------------------------------------------------------

    def send_acceleration_ned(self,
                               ax: float, ay: float, az: float,
                               yaw_rad: Optional[float] = None) -> None:
        """
        Send a single NED acceleration setpoint to PX4.

        Parameters
        ----------
        ax : float  North acceleration (m/s²)
        ay : float  East  acceleration (m/s²)
        az : float  Down  acceleration (m/s²), positive = downward
        yaw_rad : float | None
            Desired heading in radians (-π..π). None = hold current yaw.

        Notes
        -----
        - Must be called at ≥ 2 Hz to avoid watchdog timeout (500 ms default).
        - Recommended rate: 50 Hz.
        - Call hover() or stop publishing to return to stick control.
        """
        if self._mav is None:
            raise RuntimeError("Not connected. Call connect() first.")

        if yaw_rad is not None:
            type_mask = _TYPE_MASK_ACC_WITH_YAW
            yaw = float(yaw_rad)
        else:
            type_mask = _TYPE_MASK_ACC_ONLY
            yaw = 0.0

        self._mav.mav.set_position_target_local_ned_send(
            time_boot_ms=0,
            target_system=self._mav.target_system,
            target_component=self._mav.target_component,
            coordinate_frame=_MAV_FRAME_LOCAL_NED,
            type_mask=type_mask,
            x=0.0, y=0.0, z=0.0,
            vx=0.0, vy=0.0, vz=0.0,
            afx=float(ax), afy=float(ay), afz=float(az),
            yaw=yaw,
            yaw_rate=0.0
        )

    def send_acceleration(self, cmd: AccelerationNed) -> None:
        """Convenience overload accepting an AccelerationNed dataclass."""
        self.send_acceleration_ned(
            cmd.north_m_s2, cmd.east_m_s2, cmd.down_m_s2, cmd.yaw_rad
        )

    def hover(self) -> None:
        """Send zero acceleration (stop accelerating, maintain current velocity)."""
        self.send_acceleration_ned(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send_command_long(self, command: int,
                           param1: float = 0.0, param2: float = 0.0,
                           param3: float = 0.0, param4: float = 0.0,
                           param5: float = 0.0, param6: float = 0.0,
                           param7: float = 0.0) -> None:
        if self._mav is None:
            raise RuntimeError("Not connected.")
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            command, 0,
            param1, param2, param3, param4, param5, param6, param7
        )

    def _set_flight_mode(self, main_mode: int, sub_mode: int) -> None:
        """Set PX4 flight mode via SET_MODE message (#11).
        custom_mode encoding: bits 16-23 = main_mode, bits 24-31 = sub_mode.
        """
        custom_mode = (sub_mode << 24) | (main_mode << 16)
        self._mav.mav.set_mode_send(
            self._mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode
        )

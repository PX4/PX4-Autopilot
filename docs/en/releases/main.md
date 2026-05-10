# PX4-Autopilot Main Release Notes

<Badge type="danger" text="Alpha" />

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page is on a release branch, and hence probably out of date. <a href="https://docs.px4.io/main/en/releases/main">See the latest version</a>.</p>
  </div>
</div>

This contains changes to PX4 `main` branch since the last major release ([PX v1.16](../releases/1.16.md)).

::: warning
PX4 v1.17 is in alpha/beta testing.
Update these notes with features that are going to be in `main` (PX4 v1.18 or later) but not the PX4 v1.17 release.
:::

## Read Before Upgrading

- Log rotation is now enabled by default. Previously, a single log file grew for the entire flight and old logs were only deleted at boot once free space fell below a fixed 300 MB floor. The logger now caps each log file at [SDLOG_MAX_SIZE](../advanced_config/parameter_reference.md#SDLOG_MAX_SIZE) (new parameter, default `1024` MB) and keeps a configurable percentage of the disk free via [SDLOG_ROTATE](../advanced_config/parameter_reference.md#SDLOG_ROTATE) (new parameter, default `90`, so at least 10% free). Cleanup runs at log start rather than boot, so logs can still be downloaded via FTP before they are deleted. See [Log Cleanup](../dev_log/logging.md#log-cleanup) for details.
- `SDLOG_DIRS_MAX` behaviour changed: it is now an orthogonal directory-count cap that runs on top of the new space-based cleanup, and the default is `0` (disabled). Previously it enforced a fixed ~300 MB free-space floor even when set to `0`. If you relied on that implicit floor, set [SDLOG_ROTATE](../advanced_config/parameter_reference.md#SDLOG_ROTATE) instead.

Please continue reading for [upgrade instructions](#upgrade-guide).

## Major Changes

- TBD

## Upgrade Guide

## Other changes

### Hardware Support

- TBD

### Common

- [Remote ID (Open Drone ID) in-flight failsafe](../peripherals/remote_id.md): extended [COM_ARM_ODID](../advanced_config/parameter_reference.md#COM_ARM_ODID) to also trigger a configurable failsafe action (Return, Land, or Terminate) if the Remote ID heartbeat is lost while airborne. Users previously on `COM_ARM_ODID=2` retain the same arming behaviour; set to `3` or higher to enable the in-flight action. ([PX4-Autopilot#27029](https://github.com/PX4/PX4-Autopilot/pull/27029))
- [QGroundControl Bootloader Update](../advanced_config/bootloader_update.md#qgc-bootloader-update-sys-bl-update) via the [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE) parameter has been re-enabled after being broken for a number of releases. ([PX4-Autopilot#25032: build: romf: fix generation of rc.board_bootloader_upgrade](https://github.com/PX4/PX4-Autopilot/pull/25032)).
- [Feature: Allow prioritization of manual control inputs based on their instance number in ascending or descending order](../config/manual_control.md#px4-configuration). ([PX4-Autopilot#25602: Ascending and descending manual control input priorities](https://github.com/PX4/PX4-Autopilot/pull/25602)).

### Control

- Added new flight mode(s): [Altitude Cruise (MC)](../flight_modes_mc/altitude_cruise.md), Altitude Cruise (FW).
  For fixed-wing the mode behaves the same as Altitude mode but you can disable the manual control loss failsafe. ([PX4-Autopilot#25435: Add new flight mode: Altitude Cruise](https://github.com/PX4/PX4-Autopilot/pull/25435)).

### Safety

- Rotary-wing vehicles now support uncommanded altitude loss detection: if the vehicle descends more than [FD_ALT_LOSS](../advanced_config/parameter_reference.md#FD_ALT_LOSS) meters below its setpoint in altitude-controlled flight, flight termination (and parachute deployment) is triggered. See [Altitude Loss Trigger](../config/safety.md#altitude-loss-trigger). ([PX4-Autopilot#26837](https://github.com/PX4/PX4-Autopilot/pull/26837))

### Estimation

- Added [EKF2_POS_LOCK](../advanced_config/parameter_reference.md#EKF2_POS_LOCK) to force constant position fusion while landed, useful for vehicles relying on dead-reckoning sensors (airspeed, optical flow) that provide no aiding on the ground.

### Sensors

- Add [sbgECom INS driver](../sensor/sbgecom.md) ([PX4-Autopilot#24137](https://github.com/PX4/PX4-Autopilot/pull/24137))
- Quick magnetometer calibration now supports specifying an arbitrary initial heading ([PX4-Autopilot#24637](https://github.com/PX4/PX4-Autopilot/pull/24637))

### Simulation

- SIH: Add option to set wind velocity ([PX4-Autopilot#26467](https://github.com/PX4-Autopilot/pull/26467))

<!-- MOVED THIS TO v1.17

- Overhaul rover simulation:
  - Add synthetic differential rover model: [PX4-gazebo-models#107](https://github.com/PX4/PX4-gazebo-models/pull/107)
  - Add synthetic mecanum rover model: [PX4-gazebo-models#113](https://github.com/PX4/PX4-gazebo-models/pull/113)
  - Update synthetic ackermann rover model: [PX4-gazebo-models#117](https://github.com/PX4/PX4-gazebo-models/pull/117)

-->

### Debug & Logging

- [Asset Tracking](../debug/asset_tracking.md): Automatic tracking and logging of external device information including vendor name, firmware and hardware version, serial numbers. Currently supports DroneCAN devices. ([PX4-Autopilot#25617](https://github.com/PX4/PX4-Autopilot/pull/25617))
- Logger: support for small flash storage (e.g. 128 MB W25N NAND on kakuteh7mini, kakuteh7v2, airbrainh743). Logs can now be written directly to an internal littlefs volume instead of requiring an SD card.
- Logger: reworked log rotation and cleanup. Log rotation is now on by default, and cleanup runs at log start rather than boot so logs can be downloaded via FTP before being deleted.
  - New [SDLOG_MAX_SIZE](../advanced_config/parameter_reference.md#SDLOG_MAX_SIZE) (default `1024` MB) caps the size of a single log file; once reached, the logger closes the current file and starts a new one.
  - New [SDLOG_ROTATE](../advanced_config/parameter_reference.md#SDLOG_ROTATE) (default `90`) sets the maximum disk usage percentage. Cleanup guarantees `(100 - SDLOG_ROTATE)%` of the disk stays free at all times, even while writing a new log file. Set `0` to disable space-based cleanup, `100` to allow filling the disk completely.
  - `SDLOG_DIRS_MAX` is now an orthogonal cap on the number of log directories (default `0` = disabled), on top of the space-based cleanup driven by `SDLOG_ROTATE` and `SDLOG_MAX_SIZE`. SITL defaults to `7`.
- New `mklittlefs` systemcmd for reformatting a littlefs volume from the NSH console, analogous to `mkfatfs` for FAT filesystems.

### Ethernet

- TBD

### uXRCE-DDS / Zenoh / ROS2

- TBD

<!-- MOVED THIS TO v1.17

- [PX4 ROS 2 Interface Library](../ros2/px4_ros2_control_interface.md) support for [Fixed Wing lateral/longitudinal setpoint](../ros2/px4_ros2_control_interface.md#fixed-wing-lateral-and-longitudinal-setpoint-fwlaterallongitudinalsetpointtype) (`FwLateralLongitudinalSetpointType`) and [VTOL transitions](../ros2/px4_ros2_control_interface.md#controlling-a-vtol). ([PX4-Autopilot#24056](https://github.com/PX4/PX4-Autopilot/pull/24056)).
- [PX4 ROS 2 Interface Library](../ros2/px4_ros2_control_interface.md) support for [ROS-based waypoint missions](../ros2/px4_ros2_waypoint_missions.md).
-->

### MAVLink

- Removed support for deprecated request commands `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`, `MAV_CMD_REQUEST_PROTOCOL_VERSION`, `MAV_CMD_GET_HOME_POSITION`, `MAV_CMD_REQUEST_FLIGHT_INFORMATION`, `MAV_CMD_REQUEST_STORAGE_INFORMATION` (Replaced by `MAV_CMD_REQUEST_MESSAGE`).
  ([PX4-Autopilot#27251: fix(mavlink): Remove deprecated MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES](https://github.com/PX4/PX4-Autopilot/pull/27251), [PX4-Autopilot#27252: fix(mavlink): Remove legacy mavlink message requestors#27252](https://github.com/PX4/PX4-Autopilot/pull/27252))

### RC

- Parse ELRS Status and Link Statistics TX messages in the CRSF parser.

### Multi-Rotor

- Removed parameters `MPC_{XY/Z/YAW}_MAN_EXPO` and use default value instead, as they were not deemed necessary anymore. ([PX4-Autopilot#25435: Add new flight mode: Altitude Cruise](https://github.com/PX4/PX4-Autopilot/pull/25435)).
- Renamed `MPC_HOLD_DZ` to `MAN_DEADZONE` to have it globally available in modes that allow for a dead zone. ([PX4-Autopilot#25435: Add new flight mode: Altitude Cruise](https://github.com/PX4/PX4-Autopilot/pull/25435)).

### VTOL

- TBD

### Fixed-wing

- TBD

<!-- MOVED THIS TO v1.17
- [Fixed Wing Takeoff mode](../flight_modes_fw/takeoff.md) will now keep climbing with level wings on position loss.
  A target takeoff waypoint can be set to control takeoff course and loiter altitude. ([PX4-Autopilot#25083](https://github.com/PX4/PX4-Autopilot/pull/25083)).
- Automatically suppress angular rate oscillations using [Gain compression](../features_fw/gain_compression.md). ([PX4-Autopilot#25840: FW rate control: add gain compression algorithm](https://github.com/PX4/PX4-Autopilot/pull/25840))
-->

### Rover

- TBD

<!-- MOVED THIS TO v1.17

- Removed deprecated rover module ([PX4-Autopilot#25054](https://github.com/PX4/PX4-Autopilot/pull/25054)).
- Add support for [Apps & API](../flight_modes_rover/api.md) ([PX4-Autopilot#25074](https://github.com/PX4/PX4-Autopilot/pull/25074), [PX4-ROS2-Interface-Lib#140](https://github.com/Auterion/px4-ros2-interface-lib/pull/140)).
- Update [rover simulation](../frames_rover/index.md#simulation) ([PX4-Autopilot#25644](https://github.com/PX4/PX4-Autopilot/pull/25644)) (see [Simulation](#simulation) release note for details).

-->

### ROS 2

- TBD

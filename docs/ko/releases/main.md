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

This contains changes to the PX4 `main` branch that are not included in the next release ([PX4 v1.18](../releases/1.18.md)).

:::warning
PX4 v1.18 is in beta testing.
Update these notes with features that are going to be in `main` (PX4 v2.0 or later) but not the PX4 v1.18 release.
:::

## Read Before Upgrading

Please continue reading for [upgrade instructions](#upgrade-guide).

## Major Changes

- **[Motor failure recovery for hexarotors](../config/motor_failure_recovery.md).** On a detected single motor failure the control allocator removes the failed motor and, on a hexarotor, additionally stops ([CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1`) or reverses (`2`) the motor opposite it, recovering the yaw authority that is otherwise lost. Reversing keeps the opposite motor in the allocation and needs a reverse-capable ESC; the reverse thrust the allocator expects from a forward propeller is set with [CA_REV_THR_FRAC](../advanced_config/parameter_reference.md#CA_REV_THR_FRAC) (default `0.4`). Disabled by default. ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))

## Upgrade Guide

- `COM_ARM_TRAFF` has been replaced by `COM_TRAFF_AVOID`. The old value 3 ("enforce for mission modes only") is migrated to `COM_TRAFF_AVOID=2`, which blocks arming in all modes, not just mission modes. If you relied on being able to arm manually with traffic detected, set `COM_TRAFF_AVOID=1` (warning only) instead.
- `PCA9685_SCHD_HZ` has been removed. `PCA9685_PWM_FREQ` now sets both the PWM frequency of the PCA9685 and the rate at which values are pushed to it; if you had `PCA9685_SCHD_HZ` set to a non-default value, set `PCA9685_PWM_FREQ` to it after upgrading. Frequencies above 400 Hz (previously only usable in duty-cycle mode) are no longer supported.
- **Flight controller firmware no longer embeds CAN node firmware.** The `px4_fmu-v5_uavcanv0periph` build, which carried CUAV CAN GPS v1 firmware in its ROMFS so the flight controller could update the GPS over DroneCAN, and the `CONFIG_BOARD_UAVCAN_PERIPHERALS` board option behind it have been removed. Use the standard `px4_fmu-v5_default` firmware and update the GPS from the SD card with `cuav_can-gps-v1_default.uavcan.bin` from the release instead; see [DroneCAN Firmware Update](../dronecan/index.md#firmware-update). ([PX4-Autopilot#28870](https://github.com/PX4/PX4-Autopilot/pull/28870))
- **jMAVSim has been removed.** `make px4_sitl jmavsim` and the `10017_jmavsim_iris` airframe (`SYS_AUTOSTART=10017`) no longer exist, and the setup scripts no longer install Java or `ant`. Use [SIH](../sim_sih/index.md) with the [Hawkeye](../sim_hawkeye/index.md) visualizer instead (`make px4_sitl_sih sihsim_quadx`), or [Gazebo](../sim_gazebo_gz/index.md).
- **Re-check motor failure handling on hexarotors.** [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1` now also stops the motor opposite the failed one on a hexarotor (previously only the failed motor was removed from the allocation). Other airframes are unaffected, and `CA_FAILURE_MODE=0` (the default) is unchanged. See [Motor Failure Recovery](../config/motor_failure_recovery.md). ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))
- **Dual-antenna GNSS heading must be reconfigured.** `GPS_YAW_OFFSET`, `SEP_YAW_OFFS`, `SEP_PITCH_OFFS` and `EKF2_GPS_YAW_OFF` are removed, including on DroneCAN GNSS nodes, and are not migrated: GNSS yaw is not used until [SENS_GNSSn_HDG](../advanced_config/parameter_reference.md#SENS_GNSS0_HDG) is set on the autopilot for the receiver reporting the heading. Until then, a warning names what is missing: `SENS_GNSSn_HDG` or the antenna positions. The baseline comes from the antenna positions (`SENS_GNSSn_OFFX/Y/Z` of both receivers for a moving base, plus `SENS_GNSSn_AUXX/Y/Z` for the second antenna of a dual-antenna receiver), and headings whose reported baseline is more than 20% off it are rejected. See [GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source). A DroneCAN node running older firmware still subtracts its own `GPS_YAW_OFFSET`; set that to 0. Heading is taken only from `sensor_gnss_relative` (u-blox, Unicore UM982, Septentrio, DroneCAN `RelPosHeading`): the Trimble MB-Two, Femtomes, NMEA `HDT`, SBG and MicroStrain headings and the heading in DroneCAN `Fix2` are no longer used, and the `heading`, `heading_offset` and `heading_accuracy` fields of `sensor_gps` are removed. ([PX4-Autopilot#27102](https://github.com/PX4/PX4-Autopilot/pull/27102))
- **GNSS topics, messages and parameters are renamed.**
  The per-receiver topic `sensor_gps` is now `sensor_gnss` ([SensorGnss](../msg_docs/SensorGnss.md)).
  `vehicle_gps_position` is now `vehicle_gnss` ([VehicleGnss](../msg_docs/VehicleGnss.md)), which nests the selected receiver's sample as `receiver` (for example `vehicle_gnss.receiver.latitude`) and adds the corrected `timestamp_sample` and `antenna_offset`.
  Renamed fields: `latitude_deg` → `latitude`, `longitude_deg` → `longitude`, `altitude_msl_m` → `altitude_msl`, `altitude_ellipsoid_m` → `altitude_ellipsoid`, `s_variance_m_s` → `speed_accuracy`, `c_variance_rad` → `course_accuracy`, `noise_per_ms` → `noise`, `vel_m_s` → `ground_speed`, `vel_n_m_s`/`vel_e_m_s`/`vel_d_m_s` → `vel_north`/`vel_east`/`vel_down` and `cog_rad` → `course`; `antenna_offset_x/y/z` moved to `vehicle_gnss.antenna_offset`.
  ROS 2 applications must subscribe to `/fmu/out/vehicle_gnss` (`px4_msgs/msg/VehicleGnss`) instead of `/fmu/out/vehicle_gps_position` (`px4_msgs/msg/SensorGps`).
  Read the timestamps from the top level of `VehicleGnss`: only `timestamp` and `timestamp_sample` are converted to ROS time, while `receiver.timestamp` and `receiver.timestamp_sample` stay on the PX4 boot clock.
  `timestamp` is now when the sensors module published the sample; use `timestamp_sample` for the measurement time.
  `SENS_GPS_MASK`, `SENS_GPS_TAU`, `SENS_GPS_PRIME` and `SENS_GPSn_ID/OFFX/OFFY/OFFZ/DELAY` are now `SENS_GNSS_*` and `SENS_GNSSn_*` (the driver `GPS_*` parameters are unchanged).
  Saved parameters are migrated automatically, but loading a QGC parameter file with the old names does not restore them.
  The log analysis scripts in the tree read both the old and the new names.
  The uORB-over-Cyphal registers are renamed from `uorb.sensor_gps` to `uorb.sensor_gnss` (`uavcan.sub.uorb.sensor_gnss.0.id`, `uavcan.pub.uorb.sensor_gnss.0.id`); `UCAN1_UORB_GPS` and `UCAN1_UORB_GPS_P` are unchanged. ([PX4-Autopilot#24399](https://github.com/PX4/PX4-Autopilot/pull/24399))
- **The GNSS check parameters moved from EKF2 to the sensors module.**
  `EKF2_GPS_CHECK` is now [GNSS_CHECK](../advanced_config/parameter_reference.md#GNSS_CHECK), `EKF2_REQ_EPH`, `EKF2_REQ_EPV`, `EKF2_REQ_NSATS`, `EKF2_REQ_PDOP`, `EKF2_REQ_HDRIFT`, `EKF2_REQ_VDRIFT`, `EKF2_REQ_FIX` and `EKF2_REQ_SACC` are now `GNSS_REQ_*`, and `EKF2_REQ_GPS_H` is now [GNSS_REQ_TIME](../advanced_config/parameter_reference.md#GNSS_REQ_TIME).
  Saved parameters are migrated automatically, but loading a QGC parameter file with the old names does not restore them. ([PX4-Autopilot#28520](https://github.com/PX4/PX4-Autopilot/pull/28520), [PX4-Autopilot#28798](https://github.com/PX4/PX4-Autopilot/pull/28798))
- **`DO_JUMP` can only loop back and its repeat count must fit.**
  A mission is now rejected if a `DO_JUMP` targets itself or a later mission item, even if its repeat count is 0, and if its repeat count is negative, above 65535 or not a number (it is no longer narrowed to a different count).
  Missions stored on the vehicle with such jumps must be edited and uploaded again.
- **`MAV_CMD_MISSION_START` no longer arms the vehicle.**
  The command switches to [Mission mode](../flight_modes_mc/mission.md) at the requested item, and the mission starts once the vehicle is armed.
  This also applies to `MAV_CMD_DO_LAND_START`, which uses it internally.
  Ground stations and scripts that relied on the command to arm must send `MAV_CMD_COMPONENT_ARM_DISARM` separately (QGroundControl, MAVSDK and the PX4 ROS 2 Interface Library already do).
  See the [MAVLink spec clarification](https://github.com/mavlink/mavlink/pull/2648).
- **The primary GNSS receiver defaults to Auto.**
  [SENS_GNSS_PRIME](../advanced_config/parameter_reference.md#SENS_GNSS_PRIME) now defaults to `-1` (Auto) instead of `0`, and saved values are not changed: with Auto the moving base of a moving base pair is the primary receiver, and without one, a receiver that passes the strict checks ([GNSS_REQ_EPH](../advanced_config/parameter_reference.md#GNSS_REQ_EPH) and the other `GNSS_REQ_*` parameters) is used, then one with an RTK fixed solution.
  To keep preferring the receiver on the main serial port, set `SENS_GNSS_PRIME` to `0`.
  In flight the primary receiver is now also left when it has failed the strict checks for 10 s while the other receiver passed them. ([PX4-Autopilot#28798](https://github.com/PX4/PX4-Autopilot/pull/28798))
- **Re-check the motor failure trigger thresholds.**
  `MOTFAIL_OFF` and `MOTFAIL_TIME` are replaced by a separate band and trigger time per direction ([MOTFAIL_OVER](../advanced_config/parameter_reference.md#MOTFAIL_OVER)/[MOTFAIL_OVR_TIME](../advanced_config/parameter_reference.md#MOTFAIL_OVR_TIME) and [MOTFAIL_UNDER](../advanced_config/parameter_reference.md#MOTFAIL_UNDER)/[MOTFAIL_UND_TIME](../advanced_config/parameter_reference.md#MOTFAIL_UND_TIME)), and are not migrated: the new parameters start at their defaults. The check now compares a low-pass filtered current residual instead of the instantaneous value, so the thresholds are worth re-deriving: see [Motor Failure Detection > Upgrading from PX4 v1.18](../config/motor_failure_detection.md#upgrading-from-px4-v1-18). ([PX4-Autopilot#27917](https://github.com/PX4/PX4-Autopilot/pull/27917))

## Other changes

- Fast mission Return modes ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 2 and 4) now skip `DO_JUMP` commands (loops) while following the mission path. ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))
- A mission Return mode ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 1, 2, 3 or 4) now reads the current mission on the cycle that activates it, so a mission uploaded or changed right as Return starts is used for the return type, the geofence avoidance destination and the starting item. The fast variants also discard the item index they recorded from an earlier mission when a different one is current. ([PX4-Autopilot#28748](https://github.com/PX4/PX4-Autopilot/pull/28748))
- A `DO_JUMP` whose counter cannot be written to storage, for example on a failing SD card, is now skipped with a message to the operator. Previously the jump was taken again on every pass, repeating the same mission segment until a failsafe stopped the mission. ([PX4-Autopilot#28752](https://github.com/PX4/PX4-Autopilot/pull/28752))

### Hardware Support

- [AIFoundry Erbium](../flight_controller/aifoundry_erbium.md): new experimental target for the AIFoundry Erbium RISC-V (ET-Minion) chip, which runs the SIH quadcopter on the `erbium_emu` emulator until silicon is available. ([PX4-Autopilot#29094](https://github.com/PX4/PX4-Autopilot/pull/29094))
- [DroneCAN ESCs](../dronecan/escs.md) no longer need to set `UAVCAN_PUB_ARM` as `ArmingStatus` is published automatically whenever `UAVCAN_ENABLE` is `3` (ESC output enabled). ([PX4-Autopilot#28364](https://github.com/PX4/PX4-Autopilot/pull/28364))
- [DroneCAN ESC](../dronecan/escs.md) channels assigned a non-motor function (e.g. _Peripheral via Actuator Set_) can be driven bidirectionally via `UAVCAN_EC_SIGNED` (signed RawCommand, 0 = neutral). Motors continue to use `CA_R_REV`. ([PX4-Autopilot#26903](https://github.com/PX4/PX4-Autopilot/pull/26903))
- [DroneCAN](../dronecan/index.md) batteries that set the charging flag in `BatteryInfo` now report a charging state (shown in MAVLink `BATTERY_STATUS.charge_state`) as long as the pack has no low, critical or emergency warning. Charging does not count as a battery warning, so it does not affect the [battery level failsafe](../config/safety.md#battery-level-failsafe) or the arming checks. ([PX4-Autopilot#28422](https://github.com/PX4/PX4-Autopilot/pull/28422), [PX4-Autopilot#29012](https://github.com/PX4/PX4-Autopilot/pull/29012))
- [Serial Passthrough](../uart/serial_passthrough.md): new reference [bridge application](../uart/serial_passthrough.md#bridge-application), `Tools/mavlink_serial_bridge.py`, exposes a flight controller serial port or ESC signal pin as a virtual serial port on a Linux/macOS host, so ESC configuration and GPS tools can connect to it over MAVLink.
  ([PX4-Autopilot#28782: feat(tools): add mavlink_serial_bridge.py reference bridge application](https://github.com/PX4/PX4-Autopilot/pull/28782))
- [Agam FloRange](../dronecan/agam_florange.md): new DroneCAN optical flow, distance sensor and IMU module (PAA3905, AFBR-S50LV85D, ICM-42688-P). ([PX4-Autopilot#28901](https://github.com/PX4/PX4-Autopilot/pull/28901))

### 공통

- The unit and functional test suite builds, links and runs on macOS (`make tests`), and `px4_poll()` timeouts on macOS wait for their full duration instead of returning at once. The blended GPS timestamp is accumulated in double and rounded once instead of truncating each weighted term. ([PX4-Autopilot#28516](https://github.com/PX4/PX4-Autopilot/pull/28516))
- The AlphaFilter library takes its sample interval and time constant in microseconds, and the float seconds overloads are removed, so a value in the wrong unit no longer compiles. Out-of-tree code that constructs the filter with seconds needs updating. ([PX4-Autopilot#28421](https://github.com/PX4/PX4-Autopilot/pull/28421))
- The dataman module now hands out 16-bit client IDs instead of 8-bit ones. Each switch between mission-based return types and each MAVLink instance started in flight used up IDs, and once 254 had been handed out, every new client failed its requests with "Max Dataman clients reached". ([PX4-Autopilot#28944](https://github.com/PX4/PX4-Autopilot/pull/28944))

### 제어

- [Gain compression](../features_mc/gain_compression.md) is now available on multicopters ([MC_GC_EN](../advanced_config/parameter_reference.md#MC_GC_EN)). When an oscillation is detected on the torque setpoint the rate loop gain is dynamically reduced instead of requiring a manual retune, and recovers to 1.0 once the oscillation stops. The lower bound is set with [MC_GC_GAIN_MIN](../advanced_config/parameter_reference.md#MC_GC_GAIN_MIN) (default `0.3`). Disabled by default.

### 안전 설정

- [Geofence Aware Return mode](../flight_modes/return.md#geofence_awareness). ([PX4-Autopilot#27145: feat(navigator): Geofence Aware RTL](https://github.com/PX4/PX4-Autopilot/pull/27145), [PX4-Autopilot#28001: docs(navigator): [geofence] added some more warnings about limitations](https://github.com/PX4/PX4-Autopilot/pull/28001)).
- [Flight termination](../advanced_config/flight_termination.md) can now be used instead of a Descent mode as a fallback failsafe mode, allowing safer landing for unpiloted vehicles that carry a parachute.
  See [Battery level failsafe](../config/safety.md#battery-level-failsafe) ([COM_LOW_BAT_ACT](../advanced_config/parameter_reference.md#COM_LOW_BAT_ACT)) and [Position Loss Failsafe Action](../config/safety.md#position-loss-failsafe-action) (new [COM_POS_FS_ACT](../advanced_config/parameter_reference.md#COM_POS_FS_ACT)). ([PX4-Autopilot#28064: feat(commander): add terminate options for critical battery and lost position failsafes](https://github.com/PX4/PX4-Autopilot/pull/28064)).
- [Failure injection](../debug/failure_injection.md) ( [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN)) has been significantly extended. (PX4-Autopilot#27572, PX4-Autopilot#27832, PX4-Autopilot#27950)
  - Now applied on real hardware, not just simulators (injection hooks live in the shared sensor drivers).
  - Command handling is centralized behind a dedicated failure-injection manager module.
  - Multiple sensor instances can be failed simultaneously via a bitmask, and failures can be triggered from an RC switch.
  - Changed default behaviour of injected WRONG failure for Batteries, to publish a wrong level, and not stop publishing
- [Motor failure recovery](../config/motor_failure_recovery.md) for hexarotors: on a single motor failure the control allocator removes the failed motor and additionally stops ([CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1`) or reverses (`2`) the motor opposite it to recover the lost yaw authority. Mode `2` requires a reverse-capable ESC and models the reverse thrust of a forward propeller with the new [CA_REV_THR_FRAC](../advanced_config/parameter_reference.md#CA_REV_THR_FRAC) (default `0.4`). Reversible motor outputs on DroneCAN are now sent as signed `RawCommand` values (negative is reverse). ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))
- Added `RTL_TYPE=6` for battery-aware home priority return ([PX4-Autopilot#26968](https://github.com/PX4/PX4-Autopilot/pull/26968)).
  Returns to home if the estimated flight time to home is within the remaining battery time; otherwise returns to the closest rally point.
  Falls back to the closest safe point (home or rally) if battery time remaining is unavailable.
- [Secure boot](../advanced_config/bootloader_secure_boot.md#multi-part-signed-images) now supports applications that can boot securely from SD card or eMMC. ([PX4-Autopilot#28228](https://github.com/PX4/PX4-Autopilot/pull/28228))
- [Motor failure detection](../config/motor_failure_detection.md) now compares a low-pass filtered current residual against a separate trip band and trigger time for the undercurrent and overcurrent directions, instead of one symmetric band on the instantaneous current, and the current model gains an offset term. See [Upgrading from PX4 v1.18](../config/motor_failure_detection.md#upgrading-from-px4-v1-18). ([PX4-Autopilot#27711](https://github.com/PX4/PX4-Autopilot/pull/27711), [PX4-Autopilot#27917](https://github.com/PX4/PX4-Autopilot/pull/27917))

### Estimation

- EKF2 fuses dual-antenna GNSS heading from its own topic, `vehicle_gnss_heading`, at the heading's rate and measurement time instead of with each position sample. GNSS yaw fusion is gated on the heading itself (its baseline checks, and the spoofing and jamming state of the receiver providing it under [GNSS_CHECK](../advanced_config/parameter_reference.md#GNSS_CHECK), which stop it like position fusion), not on the position checks, and it keeps running when position and velocity fusion stop. `GPS_RAW_INT` and `GPS2_RAW` report the body-frame heading of the receiver that provides it. ([PX4-Autopilot#27102](https://github.com/PX4/PX4-Autopilot/pull/27102))
- When the local position estimate is lost in flight because of GNSS, commander now reports the reason as an event: either the receiver checks that failed, or that the receiver stopped sending data. The ground station and the log show the cause ahead of the "GNSS data fusion stopped" message. ([PX4-Autopilot#28873](https://github.com/PX4/PX4-Autopilot/pull/28873))

### 센서

- Enable [u-blox Diagnostics with u-center](../gps_compass/u-center.md) while the vehicle's GPS runs as usual. ([PX4-Autopilot#28280](https://github.com/PX4/PX4-Autopilot/pull/28280)).
- Disabling the selected IMU at runtime (setting its [CAL_ACCn_PRIO](../advanced_config/parameter_reference.md#CAL_ACC0_PRIO) or [CAL_GYROn_PRIO](../advanced_config/parameter_reference.md#CAL_GYRO0_PRIO) to `0`) now hands over to the next IMU without a sensor failure report, including while the vehicle is armed. A re-enabled IMU reappears in `sensors_status_imu` and the IMU consistency check only once it publishes new data. ([PX4-Autopilot#28811](https://github.com/PX4/PX4-Autopilot/pull/28811))

### 시뮬레이션

- jMAVSim has been removed in favour of [SIH](../sim_sih/index.md) with the [Hawkeye](../sim_hawkeye/index.md) visualizer. See the [Upgrade Guide](#upgrade-guide).
- Gazebo: the GNSS failure injection commands (`failure gps off`, `stuck` and `wrong`) now apply to the NavSat data published by the gz bridge, consistent with the other simulator paths. ([PX4-Autopilot#28398](https://github.com/PX4/PX4-Autopilot/pull/28398))

### Debug & Logging

- [Offline motor failure replay](../debug/motor_failure_replay.md): a host tool that runs the real motor failure detector over a recorded `.ulg`, for calibrating and verifying the [motor failure detection](../config/motor_failure_detection.md) thresholds against a vehicle's own flights. ([PX4-Autopilot#27917](https://github.com/PX4/PX4-Autopilot/pull/27917))

### Ethernet

- TBD

### uXRCE-DDS / Zenoh / ROS 2

- (uXRCE-DDS): SITL multi vehicle automatic namespace prefix is now `uav_{px4_instance}` and it is aligned with the behaviour of `UXRCE_DDS_NS_IDX`. ([PX4-Autopilot#28338](https://github.com/PX4/PX4-Autopilot/pull/28338))

### MAVLink

- ESC/motor health is now reported in [SYS_STATUS](https://mavlink.io/en/messages/common.html#SYS_STATUS) using the [MAV_SYS_STATUS_SENSOR_PROPULSION](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_PROPULSION) bit instead of [MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS).
  ([PX4-Autopilot#29192](https://github.com/PX4/PX4-Autopilot/pull/29192)).
- [SYS_STATUS](https://mavlink.io/en/messages/common.html#SYS_STATUS) now sets the [MAV_SYS_STATUS_EXTENSION_USED](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_EXTENSION_USED) flag in `onboard_control_sensors_present` when extended sensor fields are used.

### RC

- TBD

### Multi-Rotor

- The neural network controller (`mc_nn_control`) now maps its actions to motor commands over the real motor range, so action -1 idles a motor instead of stopping it and the top of the range no longer runs past full scale. The two RPM limits and the thrust coefficient (`MC_NN_MIN_RPM`, `MC_NN_MAX_RPM`, `MC_NN_THRST_COEF`) are now checked against each other, and the module has its first unit tests. ([PX4-Autopilot#28433](https://github.com/PX4/PX4-Autopilot/pull/28433))
- The neural network controller (`mc_nn_control`) now rejects at load a model whose input or output shape does not match what the controller expects, checks the position, attitude and angular velocity observations each cycle for validity, age and finiteness, and reports a fault on a bad observation or a non-finite output instead of stopping one motor. ([PX4-Autopilot#28504](https://github.com/PX4/PX4-Autopilot/pull/28504))
- The multicopter position controller now moves its stored fallback setpoint along with the current setpoint when the estimator resets its position, velocity or heading, so when the setpoint is unusable at the moment of a reset the controller no longer flies a fallback setpoint that is offset by the amount of the reset. ([PX4-Autopilot#28810](https://github.com/PX4/PX4-Autopilot/pull/28810))

### 수직이착륙기(VTOL)

- TBD

### Fixed-wing

- TECS underspeed protection now starts at the minimum airspeed ([FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN)) and is fully active at 90% of it, or halfway to [FW_AIRSPD_STALL](../advanced_config/parameter_reference.md#FW_AIRSPD_STALL) if that is higher. Previously it only started 15% of trim airspeed below the minimum airspeed. The roll angle is limited towards 20° (or [FW_R_LIM](../advanced_config/parameter_reference.md#FW_R_LIM) if lower) while undersped, and the altitude reference is reset to avoid a pitch-up on recovery. ([PX4-Autopilot#26859](https://github.com/PX4/PX4-Autopilot/pull/26859))

### Rover

- TBD

### ROS 2

- TBD

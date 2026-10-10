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

::: warning
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
  `SENS_GPS_MASK`, `SENS_GPS_TAU`, `SENS_GPS_PRIME` and `SENS_GPSn_ID/OFFX/OFFY/OFFZ/DELAY` are now `SENS_GNSS_*` and `SENS_GNSSn_*` (the driver `GPS_*` and `EKF2_*` parameters are unchanged).
  Saved parameters are migrated automatically, but loading a QGC parameter file with the old names does not restore them.
  The log analysis scripts in the tree read both the old and the new names.
  The uORB-over-Cyphal registers are renamed from `uorb.sensor_gps` to `uorb.sensor_gnss` (`uavcan.sub.uorb.sensor_gnss.0.id`, `uavcan.pub.uorb.sensor_gnss.0.id`); `UCAN1_UORB_GPS` and `UCAN1_UORB_GPS_P` are unchanged. ([PX4-Autopilot#24399](https://github.com/PX4/PX4-Autopilot/pull/24399))

## Other changes

- Fast mission Return modes ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 2 and 4) now skip `DO_JUMP` commands (loops) while following the mission path. ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))
- A mission Return mode ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 1, 2 or 4) now reads the current mission on the cycle that activates it, so a mission uploaded or changed right as Return starts is used for the return type, the geofence avoidance destination and the starting item. The fast variants also discard the item index they recorded from an earlier mission when a different one is current. ([PX4-Autopilot#28748](https://github.com/PX4/PX4-Autopilot/pull/28748))
- A `DO_JUMP` whose counter cannot be written to storage, for example on a failing SD card, is now skipped with a message to the operator instead of being taken again on every pass, which repeated the same mission segment until a failsafe stopped it. ([PX4-Autopilot#28752](https://github.com/PX4/PX4-Autopilot/pull/28752))

### Hardware Support

- [DroneCAN ESCs](../dronecan/escs.md) no longer need to set `UAVCAN_PUB_ARM` as `ArmingStatus` is published automatically whenever `UAVCAN_ENABLE` is `3` (ESC output enabled). ([PX4-Autopilot#28364](https://github.com/PX4/PX4-Autopilot/pull/28364))
- [DroneCAN ESC](../dronecan/escs.md) channels assigned a non-motor function (e.g. _Peripheral via Actuator Set_) can be driven bidirectionally via `UAVCAN_EC_SIGNED` (signed RawCommand, 0 = neutral). Motors continue to use `CA_R_REV`. ([PX4-Autopilot#26903](https://github.com/PX4/PX4-Autopilot/pull/26903))
- [Serial Passthrough](../uart/serial_passthrough.md): new reference [bridge application](../uart/serial_passthrough.md#bridge-application), `Tools/mavlink_serial_bridge.py`, exposes a flight controller serial port or ESC signal pin as a virtual serial port on a Linux/macOS host, so ESC configuration and GPS tools can connect to it over MAVLink.
  ([PX4-Autopilot#28782: feat(tools): add mavlink_serial_bridge.py reference bridge application](https://github.com/PX4/PX4-Autopilot/pull/28782))

### Common

- The unit and functional test suite builds, links and runs on macOS (`make tests`), and `px4_poll()` timeouts on macOS wait for their full duration instead of returning at once. The blended GPS timestamp is accumulated in double and rounded once instead of truncating each weighted term. ([PX4-Autopilot#28516](https://github.com/PX4/PX4-Autopilot/pull/28516))
- The AlphaFilter library takes its sample interval and time constant in microseconds, and the float seconds overloads are removed, so a value in the wrong unit no longer compiles. Out-of-tree code that constructs the filter with seconds needs updating. ([PX4-Autopilot#28421](https://github.com/PX4/PX4-Autopilot/pull/28421))

### Control

- [Gain compression](../features_mc/gain_compression.md) is now available on multicopters ([MC_GC_EN](../advanced_config/parameter_reference.md#MC_GC_EN)). When an oscillation is detected on the torque setpoint the rate loop gain is dynamically reduced instead of requiring a manual retune, and recovers to 1.0 once the oscillation stops. The lower bound is set with [MC_GC_GAIN_MIN](../advanced_config/parameter_reference.md#MC_GC_GAIN_MIN) (default `0.3`). Disabled by default.

### Safety

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

### Estimation

- EKF2 fuses dual-antenna GNSS heading from its own topic, `vehicle_gnss_heading`, at the heading's rate and measurement time instead of with each position sample. GNSS yaw fusion is gated on the heading itself (its baseline checks, and the spoofing and jamming state of the receiver providing it under [EKF2_GPS_CHECK](../advanced_config/parameter_reference.md#EKF2_GPS_CHECK), which stop it like position fusion), not on the position checks, and it keeps running when position and velocity fusion stop. `GPS_RAW_INT` and `GPS2_RAW` report the body-frame heading of the receiver that provides it. ([PX4-Autopilot#27102](https://github.com/PX4/PX4-Autopilot/pull/27102))
- When the local position estimate is lost in flight because of GNSS, commander now reports the reason as an event: either the receiver checks that failed, or that the receiver stopped sending data. The ground station and the log show the cause ahead of the "GNSS data fusion stopped" message. ([PX4-Autopilot#28873](https://github.com/PX4/PX4-Autopilot/pull/28873))

### Sensors

- Enable [u-blox Diagnostics with u-center](../gps_compass/u-center.md) while the vehicle's GPS runs as usual. ([PX4-Autopilot#28280](https://github.com/PX4/PX4-Autopilot/pull/28280)).
- Disabling the selected magnetometer at runtime (setting its [CAL_MAGn_PRIO](../advanced_config/parameter_reference.md#CAL_MAG0_PRIO) to `0`) now hands over to the next magnetometer without a sensor failure report, and a magnetometer disabled at boot no longer prevents those after it from being selected. ([PX4-Autopilot#28750](https://github.com/PX4/PX4-Autopilot/pull/28750))

### Simulation

- jMAVSim has been removed in favour of [SIH](../sim_sih/index.md) with the [Hawkeye](../sim_hawkeye/index.md) visualizer. See the [Upgrade Guide](#upgrade-guide).
- Gazebo: the GNSS failure injection commands (`failure gps off`, `stuck` and `wrong`) now apply to the NavSat data published by the gz bridge, consistent with the other simulator paths. ([PX4-Autopilot#28398](https://github.com/PX4/PX4-Autopilot/pull/28398))

### Debug & Logging

- TBD

### Ethernet

- TBD

### uXRCE-DDS / Zenoh / ROS 2

- (uXRCE-DDS): SITL multi vehicle automatic namespace prefix is now `uav_{px4_instance}` and it is aligned with the behaviour of `UXRCE_DDS_NS_IDX`. ([PX4-Autopilot#28338](https://github.com/PX4/PX4-Autopilot/pull/28338))

### MAVLink

- TBD

### RC

- TBD

### Multi-Rotor

- The neural network controller (`mc_nn_control`) maps its actions to motor commands over the real motor range, so action -1 idles a motor instead of stopping it and the top of the range no longer runs past full scale, and the three rpm parameters are checked against each other. The module got its first unit tests. ([PX4-Autopilot#28433](https://github.com/PX4/PX4-Autopilot/pull/28433))

### VTOL

- TBD

### Fixed-wing

- TBD

### Rover

- TBD

### ROS 2

- TBD

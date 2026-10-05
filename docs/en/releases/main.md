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
- **[GNSS receiver selection](../gps_compass/index.md#multiple-receivers).** The sensors module checks every receiver and passes one of them to EKF2. While disarmed the primary receiver ([SENS_GNSS_PRIME](../advanced_config/parameter_reference.md#SENS_GNSS_PRIME)) is selected whenever it publishes, and in flight it is kept until it fails; without a primary receiver, the selection moves to a receiver that meets the `GNSS_REQ_*` accuracy requirements, and then to one with an RTK fixed solution. A receiver fails after 2 s without a usable sample, or when it is usable markedly less often than the other one, and is not selected again before disarm unless the other one fails. EKF2 resets its horizontal position to the new receiver, and the sensors module reports each switch and its reason as an event. `GPS_RAW_INT` and `GPS2_RAW` stream one receiver each for the whole session and do not follow the selection. ([PX4-Autopilot#28939](https://github.com/PX4/PX4-Autopilot/pull/28939), [PX4-Autopilot#28940](https://github.com/PX4/PX4-Autopilot/pull/28940), [PX4-Autopilot#28798](https://github.com/PX4/PX4-Autopilot/pull/28798), [PX4-Autopilot#28954](https://github.com/PX4/PX4-Autopilot/pull/28954))

## Upgrade Guide

- `COM_ARM_TRAFF` has been replaced by `COM_TRAFF_AVOID`. The old value 3 ("enforce for mission modes only") is migrated to `COM_TRAFF_AVOID=2`, which blocks arming in all modes, not just mission modes. If you relied on being able to arm manually with traffic detected, set `COM_TRAFF_AVOID=1` (warning only) instead.
- `PCA9685_SCHD_HZ` has been removed. `PCA9685_PWM_FREQ` now sets both the PWM frequency of the PCA9685 and the rate at which values are pushed to it; if you had `PCA9685_SCHD_HZ` set to a non-default value, set `PCA9685_PWM_FREQ` to it after upgrading. Frequencies above 400 Hz (previously only usable in duty-cycle mode) are no longer supported.
- **Flight controller firmware no longer embeds CAN node firmware.** The `px4_fmu-v5_uavcanv0periph` build, which carried CUAV CAN GPS v1 firmware in its ROMFS so the flight controller could update the GPS over DroneCAN, and the `CONFIG_BOARD_UAVCAN_PERIPHERALS` board option behind it have been removed. Use the standard `px4_fmu-v5_default` firmware and update the GPS from the SD card with `cuav_can-gps-v1_default.uavcan.bin` from the release instead; see [DroneCAN Firmware Update](../dronecan/index.md#firmware-update). ([PX4-Autopilot#28870](https://github.com/PX4/PX4-Autopilot/pull/28870))
- **jMAVSim has been removed.** `make px4_sitl jmavsim` and the `10017_jmavsim_iris` airframe (`SYS_AUTOSTART=10017`) no longer exist, and the setup scripts no longer install Java or `ant`. Use [SIH](../sim_sih/index.md) with the [Hawkeye](../sim_hawkeye/index.md) visualizer instead (`make px4_sitl_sih sihsim_quadx`), or [Gazebo](../sim_gazebo_gz/index.md).
- **Re-check motor failure handling on hexarotors.** [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1` now also stops the motor opposite the failed one on a hexarotor (previously only the failed motor was removed from the allocation). Other airframes are unaffected, and `CA_FAILURE_MODE=0` (the default) is unchanged. See [Motor Failure Recovery](../config/motor_failure_recovery.md). ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))
- **Dual-antenna GNSS heading must be reconfigured.** `GPS_YAW_OFFSET`, `SEP_YAW_OFFS`, `SEP_PITCH_OFFS` and `EKF2_GPS_YAW_OFF` are removed, including on DroneCAN GNSS nodes, and are not migrated: GNSS yaw is not used until [SENS_GNSSn_HDG](../advanced_config/parameter_reference.md#SENS_GNSS0_HDG) is set on the autopilot for the receiver reporting the heading. The baseline comes from the antenna positions (`SENS_GNSSn_OFFX/Y/Z` of both receivers for a moving base, plus `SENS_GNSSn_AUXX/Y/Z` for the second antenna of a dual-antenna receiver), and headings whose reported baseline is more than 20% off it are rejected. See [GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source). A DroneCAN node running older firmware still subtracts its own `GPS_YAW_OFFSET`; set that to 0. Heading is taken only from `sensor_gnss_relative` (u-blox, Unicore UM982, Septentrio, DroneCAN `RelPosHeading`): the Trimble MB-Two, Femtomes, NMEA `HDT`, SBG and MicroStrain headings and the heading in DroneCAN `Fix2` are no longer used, and the `heading`, `heading_offset` and `heading_accuracy` fields of `sensor_gps` are removed. ([PX4-Autopilot#27102](https://github.com/PX4/PX4-Autopilot/pull/27102))
- **GNSS topics, messages and parameters are renamed.**
  The per-receiver topic `sensor_gps` is now `sensor_gnss` ([SensorGnss](../msg_docs/SensorGnss.md)).
  `vehicle_gps_position` is now `vehicle_gnss` ([VehicleGnss](../msg_docs/VehicleGnss.md)), which nests the selected receiver's sample as `receiver` (for example `vehicle_gnss.receiver.latitude`) and adds the corrected `timestamp_sample` and `antenna_offset`.
  Renamed fields: `latitude_deg` → `latitude`, `longitude_deg` → `longitude`, `altitude_msl_m` → `altitude_msl`, `altitude_ellipsoid_m` → `altitude_ellipsoid`, `s_variance_m_s` → `speed_accuracy`, `c_variance_rad` → `course_accuracy`, `noise_per_ms` → `noise`, `vel_m_s` → `ground_speed`, `vel_n_m_s`/`vel_e_m_s`/`vel_d_m_s` → `vel_north`/`vel_east`/`vel_down` and `cog_rad` → `course`; `antenna_offset_x/y/z` moved to `vehicle_gnss.antenna_offset`.
  ROS 2 applications must subscribe to `/fmu/out/vehicle_gnss` (`px4_msgs/msg/VehicleGnss`) instead of `/fmu/out/vehicle_gps_position` (`px4_msgs/msg/SensorGps`).
  Read the timestamps from the top level of `VehicleGnss`: only `timestamp` and `timestamp_sample` are converted to ROS time, while `receiver.timestamp` and `receiver.timestamp_sample` stay on the PX4 boot clock.
  `timestamp` is now when the sensors module published the sample; use `timestamp_sample` for the measurement time.
  `SENS_GPS_PRIME` and `SENS_GPSn_ID/OFFX/OFFY/OFFZ/DELAY` are now `SENS_GNSS_PRIME` and `SENS_GNSSn_*` (the driver `GPS_*` parameters are unchanged).
  Saved parameters are migrated automatically, but loading a QGC parameter file with the old names does not restore them.
  The log analysis scripts in the tree read both the old and the new names.
  The uORB-over-Cyphal registers are renamed from `uorb.sensor_gps` to `uorb.sensor_gnss` (`uavcan.sub.uorb.sensor_gnss.0.id`, `uavcan.pub.uorb.sensor_gnss.0.id`); `UCAN1_UORB_GPS` and `UCAN1_UORB_GPS_P` are unchanged. ([PX4-Autopilot#24399](https://github.com/PX4/PX4-Autopilot/pull/24399))
- **GNSS blending is removed.** `SENS_GPS_MASK` and `SENS_GPS_TAU` no longer exist: EKF2 fuses one [selected receiver](../gps_compass/index.md#multiple-receivers). ([PX4-Autopilot#28921](https://github.com/PX4/PX4-Autopilot/pull/28921))
- **The GNSS quality checks moved from EKF2 to the sensors module**, which runs them for every receiver.
  `EKF2_GPS_CHECK` and `EKF2_REQ_EPH/EPV/NSATS/PDOP/HDRIFT/VDRIFT/FIX` are now [GNSS_CHECK](../advanced_config/parameter_reference.md#GNSS_CHECK) and [`GNSS_REQ_*`](../advanced_config/tuning_the_ecl_ekf.md#gnss-performance-requirements), and saved values are migrated.
  [EKF2_REQ_SACC](../advanced_config/parameter_reference.md#EKF2_REQ_SACC) and [EKF2_REQ_GPS_H](../advanced_config/parameter_reference.md#EKF2_REQ_GPS_H) remain for EKF2's own threshold and wait; a saved value is also copied to [GNSS_REQ_SACC](../advanced_config/parameter_reference.md#GNSS_REQ_SACC) and [GNSS_REQ_TIME](../advanced_config/parameter_reference.md#GNSS_REQ_TIME).
  `estimator_gps_status` is removed: the check result of each sample is `vehicle_gnss.usable` and `vehicle_gnss.failed_checks`, and that of each receiver is on `sensors_status_gnss`. ([PX4-Autopilot#28520](https://github.com/PX4/PX4-Autopilot/pull/28520), [PX4-Autopilot#28935](https://github.com/PX4/PX4-Autopilot/pull/28935))
- **[SENS_GNSS_PRIME](../advanced_config/parameter_reference.md#SENS_GNSS_PRIME) defaults to `-1` (Auto)** instead of `0`.
  With Auto the moving base of a moving base pair is the primary receiver; without one, the receivers rank by the accuracy requirements and an RTK fixed solution.
  Set it to `0` to keep the main serial receiver as the primary. ([PX4-Autopilot#28798](https://github.com/PX4/PX4-Autopilot/pull/28798))
- **`estimator_status.gps_check_fail_flags` is removed**, with its `GPS_CHECK_FAIL_*` constants.
  Read `vehicle_gnss.failed_checks` or `sensors_status_gnss.failed_checks` instead, whose bits follow [GNSS_CHECK](../advanced_config/parameter_reference.md#GNSS_CHECK). ([PX4-Autopilot#28954](https://github.com/PX4/PX4-Autopilot/pull/28954))
- **SIH simulates a second GNSS receiver only with [SIM_GNSS_NUM](../advanced_config/parameter_reference.md#SIM_GNSS_NUM) set to `2`**, no longer when `SENS_GNSS1_OFFX` or `SENS_GNSS1_OFFY` is non-zero. ([PX4-Autopilot#28955](https://github.com/PX4/PX4-Autopilot/pull/28955))

## Other changes

- Fast mission Return modes ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 2 and 4) now skip `DO_JUMP` commands (loops) while following the mission path. ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))

### Hardware Support

- [DroneCAN ESCs](../dronecan/escs.md) no longer need to set `UAVCAN_PUB_ARM` as `ArmingStatus` is published automatically whenever `UAVCAN_ENABLE` is `3` (ESC output enabled). ([PX4-Autopilot#28364](https://github.com/PX4/PX4-Autopilot/pull/28364))

### Common

- The unit and functional test suite builds, links and runs on macOS (`make tests`), and `px4_poll()` timeouts on macOS wait for their full duration instead of returning at once. ([PX4-Autopilot#28516](https://github.com/PX4/PX4-Autopilot/pull/28516))
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
  - GNSS `wrong` can also set the reported eph, epv, speed accuracy, satellite count and spoofing state (`SYS_FAIL_GPS_EPH`, `_EPV`, `_SAC`, `_SAT`, `_SPF`), and [SYS_FAIL_GPS_WRG](../advanced_config/parameter_reference.md#SYS_FAIL_GPS_WRG) `0` leaves the fix type unchanged. GNSS `slow` publishes one sample in [SYS_FAIL_GPS_DIV](../advanced_config/parameter_reference.md#SYS_FAIL_GPS_DIV). GNSS failures also apply to the receiver's heading. ([PX4-Autopilot#28955](https://github.com/PX4/PX4-Autopilot/pull/28955))
- The [GNSS check failsafe](../config/safety.md#gnss-check-failsafe) counts the receivers that pass their quality checks, not those with a 3D fix. ([PX4-Autopilot#28520](https://github.com/PX4/PX4-Autopilot/pull/28520))
- The divergence test of the [GNSS check failsafe](../config/safety.md#gnss-check-failsafe) compares each receiver with the selected one, using the distance the sensors module publishes after removing the antenna separation (`sensors_status_gnss.inconsistency`). Before arming, a configured primary GNSS receiver that is not publishing raises the warning `Primary GPS offline`, which does not block arming. ([PX4-Autopilot#28954](https://github.com/PX4/PX4-Autopilot/pull/28954))
- The home position, the geofence with [GF_SOURCE](../advanced_config/parameter_reference.md#GF_SOURCE) set to GPS, and the precision takeoff home check take only GNSS samples that passed the quality checks (`vehicle_gnss.usable`). ([PX4-Autopilot#28954](https://github.com/PX4/PX4-Autopilot/pull/28954))
- [Motor failure recovery](../config/motor_failure_recovery.md) for hexarotors: on a single motor failure the control allocator removes the failed motor and additionally stops ([CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1`) or reverses (`2`) the motor opposite it to recover the lost yaw authority. Mode `2` requires a reverse-capable ESC and models the reverse thrust of a forward propeller with the new [CA_REV_THR_FRAC](../advanced_config/parameter_reference.md#CA_REV_THR_FRAC) (default `0.4`). Reversible motor outputs on DroneCAN are now sent as signed `RawCommand` values (negative is reverse). ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))
- Added `RTL_TYPE=6` for battery-aware home priority return ([PX4-Autopilot#26968](https://github.com/PX4/PX4-Autopilot/pull/26968)).
  Returns to home if the estimated flight time to home is within the remaining battery time; otherwise returns to the closest rally point.
  Falls back to the closest safe point (home or rally) if battery time remaining is unavailable.

### Estimation

- EKF2 fuses dual-antenna GNSS heading from its own topic, `vehicle_gnss_heading`, at the heading's rate and measurement time instead of with each position sample. GNSS yaw fusion is gated on the heading itself (its baseline checks, and the spoofing and jamming state of the receiver providing it under [GNSS_CHECK](../advanced_config/parameter_reference.md#GNSS_CHECK), which stop it like position fusion), not on the position checks, and it keeps running when position and velocity fusion stop. `GPS_RAW_INT` and `GPS2_RAW` report the body-frame heading of the receiver that provides it. ([PX4-Autopilot#27102](https://github.com/PX4/PX4-Autopilot/pull/27102))
- EKF2 publishes why it fused the latest GNSS sample or not as [`estimator_status_flags.gnss_fusion_state`](../advanced_config/tuning_the_ecl_ekf.md#gps-quality-checks). When the local position is lost in flight, one event names the receiver and that reason. ([PX4-Autopilot#28873](https://github.com/PX4/PX4-Autopilot/pull/28873), [PX4-Autopilot#28954](https://github.com/PX4/PX4-Autopilot/pull/28954))
- EKF2 skips GNSS samples with a velocity component above [EKF2_VEL_LIM](../advanced_config/parameter_reference.md#EKF2_VEL_LIM), in flight too, instead of failing a pre-flight GNSS check on it. ([PX4-Autopilot#28663](https://github.com/PX4/PX4-Autopilot/pull/28663))

### Sensors

- Enable [u-blox Diagnostics with u-center](../gps_compass/u-center.md) while the vehicle's GPS runs as usual. ([PX4-Autopilot#28280](https://github.com/PX4/PX4-Autopilot/pull/28280)).
- The heading of a moving base rover is dropped while its moving base publishes no data. ([PX4-Autopilot#28955](https://github.com/PX4/PX4-Autopilot/pull/28955))

### Simulation

- jMAVSim has been removed in favour of [SIH](../sim_sih/index.md) with the [Hawkeye](../sim_hawkeye/index.md) visualizer. See the [Upgrade Guide](#upgrade-guide).
- Gazebo: the GNSS failure injection commands (`failure gps off`, `stuck` and `wrong`) now apply to the NavSat data published by the gz bridge, consistent with the other simulator paths. ([PX4-Autopilot#28398](https://github.com/PX4/PX4-Autopilot/pull/28398))
- SIH simulates each GNSS receiver with its own noise and position error ([SIM_GNSS0_BIAS_N](../advanced_config/parameter_reference.md#SIM_GNSS0_BIAS_N), `_E`, `_D`), and a GNSS heading for a receiver with [SENS_GNSSn_HDG](../advanced_config/parameter_reference.md#SENS_GNSS0_HDG) set. ([PX4-Autopilot#28955](https://github.com/PX4/PX4-Autopilot/pull/28955))

### Debug & Logging

- [`Tools/gnss_failover_report.py`](../debug/failure_injection.md#log-report) grades the GNSS failure injections in a ULog (receiver switch, estimator resets, position validity and reporting), and the [`gnss_failover`](../debug/failure_injection.md#companion-computer-tool) companion tool injects one GNSS failure in flight. `failure_injection` is logged. ([PX4-Autopilot#28955](https://github.com/PX4/PX4-Autopilot/pull/28955))

### Ethernet

- TBD

### uXRCE-DDS / Zenoh / ROS 2

- (uXRCE-DDS): SITL multi vehicle automatic namespace prefix is now `uav_{px4_instance}` and it is aligned with the behaviour of `UXRCE_DDS_NS_IDX`. ([PX4-Autopilot#28338](https://github.com/PX4/PX4-Autopilot/pull/28338))

### MAVLink

- `HIGH_LATENCY2` sets its GPS failure flag while EKF2 has no GNSS data or rejects the samples as unusable or above the velocity limit (`gnss_fusion_state`), and its estimator failure flag no longer includes GNSS check failures. ([PX4-Autopilot#28954](https://github.com/PX4/PX4-Autopilot/pull/28954))

### RC

- TBD

### Multi-Rotor

- TBD

### VTOL

- TBD

### Fixed-wing

- TBD

### Rover

- TBD

### ROS 2

- TBD

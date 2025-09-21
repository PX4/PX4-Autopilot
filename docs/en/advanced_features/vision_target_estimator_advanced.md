# Vision Target Estimator Deep Dive

This guide expands on the [Vision Target Estimator module overview](../advanced_features/vision_target_estimator.md) and targets advanced users who want to tune, extend, and debug the estimator in detail. It documents the system architecture of the Vision Target Estimator, explains how each measurement source is incorporated, and outlines proven workflows for log analysis and sensor integration.

**Table of Contents**
- [Vision Target Estimator Deep Dive](#vision-target-estimator-deep-dive)
	- [System architecture](#system-architecture)
	- [Log analysis and expected plots](#log-analysis-and-expected-plots)
	- [Adding new measurement sources](#adding-new-measurement-sources)
	- [Development and debugging tips](#development-and-debugging-tips)
		- [Regenerating the symbolic model](#regenerating-the-symbolic-model)
	- [Troubleshooting checklist](#troubleshooting-checklist)

## System architecture

The implementation is split across a thin scheduler and two independent Kalman filters:

- `VisionTargetEst` (`src/modules/vision_target_estimator/VisionTargetEst.cpp`) owns the work-queue task, parameter hot-reload logic, and orchestration. Its main loop batches vehicle inputs (`vehicle_attitude`, `vehicle_acceleration`, `vehicle_local_position`, range, and angular rates), downsamples acceleration, and dispatches the samples to the position and orientation filters at the periods defined by [`VTE_POS_RATE`](../advanced_config/parameter_reference.md#VTE_POS_RATE) and [`VTE_YAW_RATE`](../advanced_config/parameter_reference.md#VTE_YAW_RATE).
- `VTEPosition` (`Position/VTEPosition.cpp`) implements the per-axis filters, observation buffers, timeout logic, and the publication of `landing_target_pose`, `vision_target_est_position`, and every `vte_aid_*` innovation topic. Helper unions (`SensorFusionMaskU`, `ObsValidMaskU`) mirror the bit layout of [`VTE_AID_MASK`](../advanced_features/vision_target_estimator.md#sensor-fusion-selection), making it straightforward to add new observation types.
- `VTEOrientation` (`Orientation/VTEOrientation.cpp`) handles yaw fusion, using a simplified state vector but the same measurement staging helpers as the position filter.
- Shared utilities live in `common.h` and the generated SymForce headers under `Position/vtest_derivation/` (copied into the build directory during configuration). Avoid editing the generated headers directly; see [Regenerating the symbolic model](#regenerating-the-symbolic-model).

Key architectural details that matter when contributing:

- **Task activation**: `VisionTargetEstTaskMaskU` selects between precision-landing and continuous debug modes. When the debug bit is set the estimators never self-stop, making it ideal for bench testing.
- **Timeout policy**: `VTEPosition::timedOut()` and its orientation counterpart build on the shared `hasTimedOut()` helper. Any new measurement source must interact with `_meas_recent_timeout_us` and `_meas_updated_timeout_us` so that stale data never reaches the update step.
- **Input logging**: The module publishes `vision_target_est_input`, which captures the rotated acceleration sample and quaternion used for each prediction step. This is invaluable when correlating estimator behaviour with vehicle dynamics in logs.

Start in `VisionTargetEst::updateEstimators()` when tracing the execution flow: it polls vehicle data, updates per-filter caches, then calls `VTEPosition::update()` and `VTEOrientation::update()` which in turn perform prediction, and sequential fusion.



## Log analysis and expected plots

Plot Juggler (or the PX4 DevTools log viewer) is the easiest way to inspect the estimator. Create linked subplots so that state variables, measurements, innovations, and fused flags share a common time axis; this makes it obvious which sensor is driving the solution at any given moment.

**Estimator outputs**

- `vision_target_est_position`: relative position (`*_rel`), vehicle velocity, GNSS bias, and (when enabled) target velocity/acceleration states with per-axis variances.
- `vision_target_est_orientation`: yaw, yaw rate, and variances from the orientation filter.
- `landing_target_pose`: controller-facing pose plus `rel_pos_valid`, `rel_vel_valid`, and `fused` booleans.
- `vision_target_est_input`: downsampled acceleration in NED and the quaternion fed to each prediction step. Can be used to correlate estimator behaviour with vehicle attitude changes.
- `vte_aid_*`: one topic per fused source (`vte_aid_gps_pos_target`, `vte_aid_gps_pos_mission`, `vte_aid_gps_vel_uav`, `vte_aid_gps_vel_target`, `vte_aid_fiducial_marker`, `vte_aid_uwb`, `vte_aid_irlock`, `vte_aid_ev_yaw`). Each publishes the innovation, variance, observation, observation variance, chi-squared `test_ratio`, timestamps, and the `fused` flag (true when the update passed the gate).

**Input feeds worth overlaying**

- Vision: `fiducial_marker_pos_report` / `fiducial_marker_yaw_report`
- UWB: `sensor_uwb`
- GNSS on the target: `target_gnss`
- Vehicle GNSS: `sensor_gps` (used to convert absolute GNSS measurements to relative vehicle-carried NED measurements)
- Mission position: `position_setpoint_triplet`
- `vehicle_local_position` and `vehicle_attitude` (used for frame transforms and timeout checks)

What to look for in logs:

1. **Relative state vs. observations**: Overlay `vision_target_est_position.x_rel` with `vte_aid_*.observation[0]` (e.g. `vte_aid_fiducial_marker.observation[0]` or the relevant UWB, IRLock, GNSS observation). The traces should converge after a short transient. Large steady offsets point to calibration errors or incorrect body-to-NED transforms.
2. **Innovation behaviour**: `vte_aid_*.innovation` (e.g. `vte_aid_fiducial_marker.innovation`) should be centred at zero and resemble white noise. Slowly drifting innovations signal bias that is not observable (enable a second non-GNSS sensor) or incorrect noise assumptions.
3. **Measurement acceptance**: Inspect the `fused` boolean alongside `test_ratio`. If the NIS gate rejected the measurements frequently, compare the observation variance being logged with the expected sensor accuracy and update the noise parameters.
4. **Time alignment**: Compare `timestamp_sample` (measurement time of validity) with `time_last_fuse` (contains the prediction time) on the same `vte_aid_*` topic. The difference should stay within a few milliseconds of the estimator loop period. Persistent offsets are a sign of delayed sensor delivery or missing time synchronisation.
5. **Coordinate transforms**: Plot the raw measurement (e.g. `fiducial_marker_pos_report.x_rel`) together with `vte_aid_fiducial_marker.observation`. Ensure that there is no mistake in the rotation, range scaling, or offsets applied in the handler.
6. **Prediction inputs**: Use `vision_target_est_input` to track the downsampled acceleration and quaternion. Compare them to `vehicle_attitude.q` and `vehicle_acceleration` to ensure the estimator sees the expected attitude, especially when diagnosing timestamp mismatches.

When reviewing logs in Flight Review or PX4 DevTools, group all `vte_aid_*` topics in a single pane. This reveals which sensors are contributing, which are timing out, and whether the estimator is running (`fused` toggling together with `rel_pos_valid`). Remember that gaps longer than [`VTE_TGT_TOUT`](../advanced_config/parameter_reference.md#VTE_TGT_TOUT) clear the validity flags, and exceeding [`VTE_BTOUT`](../advanced_config/parameter_reference.md#VTE_BTOUT) stops the estimator entirely until new data arrives.

**All Observations Dashboard**

The next four dashboards provide hints on how to anaylse logs of the Vision Target Estimator.

- **Top row (lateral position)**: Plot the x-axis observations (`vte_aid_*.observation[0]`) from vision, IRLock, UWB, and target GNSS so you immediately see whether the sensors agree.
- **Second row (GNSS bias estimate)**: `vision_target_est_position.x_bias` should settle to a steady value once both a relative observation and target GNSS are fused. A non-zero bias is expected; what matters is that it remains constant so the corrected GNSS still points to the pad if vision temporarily drops out.
- **Third row (sensor variances)**: Compare `vte_aid_*.observation_variance[0]` across sensors. Large gaps mean one source is trusted far more than the others. Tune [`VTE_EVP_NOISE`](../advanced_config/parameter_reference.md#VTE_EVP_NOISE), [`VTE_IRL_NOISE`](../advanced_config/parameter_reference.md#VTE_IRL_NOISE), [`VTE_UWB_P_NOISE`](../advanced_config/parameter_reference.md#VTE_UWB_P_NOISE), or [`VTE_GPS_P_NOISE`](../advanced_config/parameter_reference.md#VTE_GPS_P_NOISE) until variances reflect the real-world accuracy.
- **Bottom row (approach context)**: `vehicle_local_position.dist_bottom` indicates the descent phase and helps correlate changes in variance or bias with altitude.

![VTEST all observations](../../assets/vision_target_estimator/VtestAllObservations.png)

**Innovation Consistency**

- **Top row (x innovations)**: `vte_aid_fiducial_marker.innovation[0]`, `vte_aid_irlock.innovation[0]`, and `vte_aid_gps_pos_target.innovation[0]` should fluctuate around zero. Large steps that line up with vision-only segments often point to poor camera pose calibration.
- **Second row (y innovations)**: The same signals for index 1 highlight lateral mismatches. White-noise like behaviour indicates the assumed noise matches reality.
- **Third row (z innovations)**: Index 2 is only populated by sources that provide altitude information. A persistent offset—such as the ~2 m bias from `vte_aid_fiducial_marker` in the example—means the measurement model or camera mounting is wrong. Either recalibrate or loosen the gating using [`VTE_POS_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_POS_NIS_THRE), depending on whether the sensor or the estimator is at fault.
- **Bottom row (fusion decisions)**: The boolean `*.fused` arrays confirm that the Kalman update is skipped whenever the NIS exceeds the gate. If a good sensor is frequently rejected, revisit the variance floors listed earlier.

![VTEST innovations](../../assets/vision_target_estimator/VtestInnovations.png)


**Rejecting a Corrupted Measurement**

- **Top row (y observation)**: `vte_aid_fiducial_marker.observation[1]` tracks `vision_target_est_position.y_rel`, showing how a healthy measurement pulls the estimate.
- **Second row (z observation)**: `vte_aid_fiducial_marker.observation[2]` deviates strongly from `vision_target_est_position.z_rel`; the estimator sensibly refuses to follow it.
- **Third row (fusion flag)**: `vte_aid_fiducial_marker.fused[2]` drops to zero whenever the vertical observation disagrees with the filter prediction, confirming that the gate is working.
- **Bottom row (test ratios)**: `vte_aid_fiducial_marker.test_ratio[1]` stays near zero, while index 2 spikes above one, breaching [`VTE_POS_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_POS_NIS_THRE). Investigate the vision range estimate or adjust [`VTE_EVP_NOISE`](../advanced_config/parameter_reference.md#VTE_EVP_NOISE) if this pattern occurs frequently.

![VTEST measurement not fused](../../assets/vision_target_estimator/VtestMeasNotFused.png)

**Precision Landing Alignment**

- **Top row (north alignment)**: Compare `landing_target_pose.x_abs` with `vehicle_local_position.x`. The curves should overlay once the vehicle captures the pad. Persistent offsets suggest mission setpoint vs. estimator disagreement.
- **Second row (east alignment)**: `landing_target_pose.y_abs` versus `vehicle_local_position.y` reveals lateral errors. If the controller struggles to follow, review the [multicopter position tuning guide](../config_mc/pid_tuning_guide_multicopter.md).
- **Third row (yaw alignment)**: `vision_target_est_orientation.theta` should remain steady while `trajectory_setpoint.yaw` tracks it when [`PLD_YAW_EN`](../advanced_config/parameter_reference.md#PLD_YAW_EN) is enabled.
- **Bottom row (descent context)**: `vehicle_local_position.dist_bottom` indicates when the final approach begins, providing context for any transients above.

![VTEST precision landing](../../assets/vision_target_estimator/VtestPrecland.png)

**Orientation Filter Dashboard**

- **Top row (yaw observation vs. state)**: `vte_aid_ev_yaw.observation` compared with `vision_target_est_orientation.theta`. The smoothed state should stay close without mirroring the high-frequency noise; if it does, increase [`VTE_EVA_NOISE`](../advanced_config/parameter_reference.md#VTE_EVA_NOISE).
- **Second row (innovation)**: `vte_aid_ev_yaw.innovation` should resemble white noise. Long biases indicate an incorrect camera-to-vehicle rotation.
- **Third row (test ratio)**: `vte_aid_ev_yaw.test_ratio` highlights altitude-driven noise. Brief spikes near touchdown are acceptable, but sustained high ratios mean yaw gating might be too tight.
- **Bottom row (fusion flag)**: `vte_aid_ev_yaw.fused` shows when yaw measurements actually update the state. Loss of fusion at high altitude is common; ensure it returns before `dist_bottom` goes below the final approach threshold.

![VTEST orientation](../../assets/vision_target_estimator/VtestOrientation.png)

## Adding new measurement sources

To integrate a new sensor:

1. **Define a uORB message** that carries the measurement in either the vehicle body frame or NED, complete with variance estimates and timestamps.
2. **Extend the fusion mask**: add a bit to `SensorFusionMaskU` (`src/modules/vision_target_estimator/common.h`) and update the parameter comment block for [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) (see also [sensor fusion selection](../advanced_features/vision_target_estimator.md#sensor-fusion-selection)) in `vision_target_estimator_params.c`.
3. **Augment observation enums**: append the new entry to the relevant `ObsType` enum (`VTEPosition.h` or `VTEOrientation.h`), update `ObsValidMaskU`, and touch helper functions such as `hasNewNonGpsPositionSensorData()` and `selectInitialPosition()` if the measurement can provide a pose seed.
4. **Subscribe and validate**: add a `uORB::Subscription` to the filter, check for finite values, and reject samples that are older than `_meas_recent_timeout_us` or timestamped in the future before marking the observation valid.
5. **Implement the handler** in `processObservations()`. Convert the measurement into NED coordinates, populate `TargetObs::meas_xyz`, `meas_unc_xyz`, and the observation Jacobian (`meas_h_xyz` or `meas_h_theta`), and set the fusion-mask flag only after the data passes validation.
6. **Provide tunable noise**: declare a parameter (e.g. `VTE_<SENSOR>_NOISE`) and clamp it with `kMinObservationNoise` so the estimator never believes a measurement is perfect.
7. **Log the innovations**: add a publication member and ORB topic (see `vte_aid_irlock` for reference) so that logs include the innovation, variance, and fused flag for the new sensor.
8. **Exercise SITL**: update the Gazebo (or other) simulation so that replay tests produce the new measurement. This keeps CI coverage intact and provides a reference data set for tuning.
9. **Document the workflow**: update this deep dive and any setup how-tos so users know how to enable the new bit, calibrate the sensor, and interpret its logs.

Every measurement must be time-aligned. Reject samples older than [`VTE_M_REC_TOUT`](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT) and double-check `timestamp_sample` vs. `time_last_fuse` in logs to confirm the new sensor fits inside the estimator deadlines.

## Development and debugging tips

- To print `PX4_DEBUG` statements from the module, launch SITL with `PX4_LOG_LEVEL=debug` (for example, `PX4_LOG_LEVEL=debug make px4_sitl_visionTargetEst`). On hardware builds, compile with the debug configuration or enable the console log level before running tests so the additional diagnostics appear on the shell.
- Keep the estimator alive on the bench by setting [`VTE_TASK_MASK`](../advanced_config/parameter_reference.md#VTE_TASK_MASK)=3; the debug bit forces the work item to run continuously, even without a precision-landing mission item.
- Use shell helpers while iterating: `listener landing_target_pose` to confirm state freshness, `listener vte_aid_fiducial_marker ` (or the relevant `vte_aid_*`) to inspect innovations, `listener vision_target_est_input 5` for prediction inputs, and `vision_target_estimator status` to ensure both filters are running.

### Regenerating the symbolic model

The generated headers (`predictState.h`, `predictCov.h`, `computeInnovCov.h`, `syncState.h`, `state.h`) are copied into the build directory and should never be edited by hand. To regenerate them:

1. Configure CMake with `-DVTEST_SYMFORCE_GEN=ON` (automatically enabled when `CONFIG_VTEST_MOVING=y`) and ensure SymForce is available in the Python environment.
2. Reconfigure (`cmake --fresh ...`) so the custom command in `src/modules/vision_target_estimator/CMakeLists.txt` runs. The outputs land under `build/<target>/src/modules/vision_target_estimator/vtest_derivation/generated/`.
3. If you need to refresh the committed defaults, set `-DVTEST_UPDATE_COMMITTED_DERIVATION=ON` and commit the regenerated files in `Position/vtest_derivation/generated*/` once vetted.

If the build fails during regeneration, inspect the CMake output for the SymForce invocation and rerun it manually inside `Position/vtest_derivation/` to catch Python errors. After regenerating, rebuild the module to ensure the Jacobians and code stay in sync.

## Troubleshooting checklist

Start by confirming that the estimator is running (`vision_target_estimator status`) and that the relevant `vte_aid_*` topic is present in the log. If a topic is missing, verify that the upstream sensor message is publishing and that the fusion mask bit is enabled.

| Symptom | Likely cause | Remedy |
| --- | --- | --- |
| Vehicle misses the pad or ignores target yaw | Mission land waypoint not set to precision mode or yaw alignment disabled | In QGroundControl set the land waypoint `Precision landing` field (or `MAV_CMD_NAV_LAND` `param2`) to Opportunistic/Required as described in [Precision landing missions](../advanced_features/precland.md#mission), and enable [`PLD_YAW_EN`](../advanced_config/parameter_reference.md#PLD_YAW_EN). In logs verify that `trajectory_setpoint.x/y` converge to `landing_target_pose.x_abs/y_abs` and that `trajectory_setpoint.yaw` follows `vision_target_est_orientation.theta`. |
| Frequent innovation rejections | Incorrect noise floors or timestamp skew | Verify sensor variances, check that message timestamps track `vehicle_acceleration.timestamp_sample`. |
| Bias does not converge | No secondary position source | Ensure vision, UWB, or IRLock fusion is enabled so the filter can observe GNSS bias. |
| Orientation estimate drifts | Missing yaw measurements or low NIS gate | Enable [`VTE_YAW_EN`](../advanced_config/parameter_reference.md#VTE_YAW_EN) and ensure vision yaw or UWB azimuth data is present. Increase [`VTE_YAW_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_YAW_NIS_THRE) if legitimate data is being rejected. |
| `rel_pos_valid` toggles during descent | Position measurements arriving too slowly | Increase [`VTE_TGT_TOUT`](../advanced_config/parameter_reference.md#VTE_TGT_TOUT) or improve the measurement rate so updates remain inside [`VTE_M_REC_TOUT`](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT). |
| No `vte_aid_*` topics in the log | Sensor not publishing or fusion mask disabled | Use `listener` on the raw sensor topic, confirm [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) includes the relevant bit, and rerun the test with the debug task active. |
| Estimator never starts | Task mask disabled or mission not requesting precision landing | Set [`VTE_TASK_MASK`](../advanced_config/parameter_reference.md#VTE_TASK_MASK)=1 (or 3 for continuous debugging) and verify that new measurements arrive with valid timestamps. |

# Vision Target Estimator (VTEST)

The Vision Target Estimator (VTEST) fuses vision-based relative target measurements with absolute positioning sources (vehicle GNSS, a mission landing waypoint, and/or target-mounted GNSS). It runs alongside the primary vehicle estimator and publishes

- `landing_target_pose` - relative and absolute target pose in NED for precision landing.
- `vision_target_est_position` - the full position-state vector (with covariances) for logging and tuning.
- `vision_target_est_orientation` - target yaw and yaw-rate estimates with associated uncertainty.
- `vte_aid_*` - innovation logs per sensor, used for observability checks and noise tuning.

When [`VTE_EKF_AID`](../advanced_config/parameter_reference.md#VTE_EKF_AID)=1 and the target is static, the relative velocity estimate is forwarded to the main EKF2 estimator (see [EKF2 aiding](#ekf2-aiding) for the required conditions).

> [!WARNING]
> VTEST is a beta feature, disabled in default board configurations, and should only be enabled on custom builds after careful bench and flight testing.

For tuning workflows, log analysis guidance, and instructions on extending the estimator, see the [Vision Target Estimator deep dive](../advanced_features/vision_target_estimator_advanced.md).

## Table of Contents
- [Vision Target Estimator (VTEST)](#vision-target-estimator-vtest)
  - [Table of Contents](#table-of-contents)
  - [Architecture Overview](#architecture-overview)
  - [Building the Module](#building-the-module)
  - [Dynamic Models](#dynamic-models)
    - [Position state](#position-state)
    - [Orientation state](#orientation-state)
    - [Outlier detection](#outlier-detection)
    - [Time alignment](#time-alignment)
  - [Estimator Lifecycle](#estimator-lifecycle)
  - [Measurement Sources](#measurement-sources)
    - [Measurements validity](#measurements-validity)
  - [Configuration](#configuration)
    - [Module enable and scheduling](#module-enable-and-scheduling)
    - [Task selection](#task-selection)
    - [Sensor fusion selection](#sensor-fusion-selection)
    - [Noise and gating](#noise-and-gating)
    - [Sensor-specific settings](#sensor-specific-settings)
    - [EKF2 aiding](#ekf2-aiding)
    - [MAVLink Messages](#mavlink-messages)
      - [TARGET\_RELATIVE (ID 511)](#target_relative-id-511)
      - [TARGET\_ABSOLUTE (ID 510)](#target_absolute-id-510)
    - [Moving-target projection parameters (experimental)](#moving-target-projection-parameters-experimental)
  - [Gazebo Classic Simulation](#gazebo-classic-simulation)
  - [Generated SymForce Functions](#generated-symforce-functions)
  - [Monitoring](#monitoring)
  - [Operational Notes](#operational-notes)

## Architecture Overview

VTEST is implemented as two tightly coupled, but independent, estimators managed by the `VisionTargetEst` work item:

- **Position filter (`VTEPosition`)** - three decoupled 1D Kalman filters (one per NED axis) that estimate relative position, vehicle velocity, and GNSS bias. When the firmware is built with `CONFIG_VTEST_MOVING`, target velocity and acceleration states are also estimated (experimental).
- **Orientation filter (`VTEOrientation`)** - a planar yaw filter that estimates the target's heading and yaw rate.

The work item re-schedules both filters at the rates commanded by [`VTE_POS_RATE`](../advanced_config/parameter_reference.md#VTE_POS_RATE) and [`VTE_YAW_RATE`](../advanced_config/parameter_reference.md#VTE_YAW_RATE) (default 50 Hz). Each cycle it provides vehicle acceleration (downsampled and rotated to NED), attitude, local position, local velocity, and range-sensor updates, while also enforcing timeouts, parameter reloads, and task activation.

> [!WARNING]
> The moving-target mode `CONFIG_VTEST_MOVING=y` is experimental and has not yet been flight tested.

## Building the Module

The estimator is not part of the default PX4 board configurations. Build a board variant that enables the module, or add `CONFIG_MODULES_VISION_TARGET_ESTIMATOR=y` to a custom `.px4board` file.

Set `CONFIG_MAVLINK_DIALECT="development"` so the [MAVLink messages](#mavlink-messages) for relative and absolute target measurements are available, and disable the legacy landing target estimator by adding `CONFIG_MODULES_LANDING_TARGET_ESTIMATOR=n` (both modules publish `landing_target_pose` and will conflict if enabled together).

Common build targets that already include the module are:
- `make px4_fmu-v6c_visionTargetEst`
- `make px4_sitl_visionTargetEst`

Other boards provide the same `_visionTargetEst` suffix. Enable `CONFIG_VTEST_MOVING=y` in the board configuration if you need the experimental moving-target states. Orientation estimation can be enabled at runtime with [`VTE_YAW_EN`](../advanced_config/parameter_reference.md#VTE_YAW_EN), while [`VTE_POS_EN`](../advanced_config/parameter_reference.md#VTE_POS_EN) controls the position filter.

## Dynamic Models

### Position state

For a static target the per-axis state is $x = [ r, v^{uav}, b ]^T$, where $r$ is the relative NED displacement (target minus vehicle), $v^{uav}$ is the vehicle velocity, and $b$ represents the steady GNSS bias between the GNSS-based observations and the relative observations (vision). The discrete prediction model, assuming constant NED acceleration input $a^{uav}$ over the integration interval $dt$, is

$$
\begin{aligned}
r_{k+1} &= r_k - dt\,v^{uav}_k - \tfrac{1}{2}\,dt^2\,a^{uav} \\
v^{uav}_{k+1} &= v^{uav}_k + dt\,a^{uav} \\
b_{k+1} &= b_k
\end{aligned}
$$

When `CONFIG_VTEST_MOVING` is enabled, two additional states capture the target dynamics: $x = [ r, v^{uav}, b, a^{t}, v^{t} ]^T$, where $a^{t}$ and $v^{t}$ are the target acceleration and velocity along the axis. The prediction model becomes

$$
\begin{aligned}
r_{k+1} &= r_k - dt\,(v^{uav}_k - v^{t}_k) - \tfrac{1}{2}\,dt^2\,(a^{uav} - a^{t}_k) \\
v^{uav}_{k+1} &= v^{uav}_k + dt\,a^{uav} \\
v^{t}_{k+1} &= v^{t}_k + dt\,a^{t}_k \\
a^{t}_{k+1} &= a^{t}_k \\
b_{k+1} &= b_k
\end{aligned}
$$

Process noise is assumed diagonal. The parameters [`VTE_ACC_D_UNC`](../advanced_config/parameter_reference.md#VTE_ACC_D_UNC), [`VTE_ACC_T_UNC`](../advanced_config/parameter_reference.md#VTE_ACC_T_UNC), and [`VTE_BIAS_UNC`](../advanced_config/parameter_reference.md#VTE_BIAS_UNC) set the input variance on the vehicle acceleration, target acceleration (moving mode), and GNSS bias terms. Initial variances are seeded from [`VTE_POS_UNC_IN`](../advanced_config/parameter_reference.md#VTE_POS_UNC_IN), [`VTE_VEL_UNC_IN`](../advanced_config/parameter_reference.md#VTE_VEL_UNC_IN), [`VTE_BIA_UNC_IN`](../advanced_config/parameter_reference.md#VTE_BIA_UNC_IN), and [`VTE_ACC_UNC_IN`](../advanced_config/parameter_reference.md#VTE_ACC_UNC_IN) (moving mode).

### Orientation state

The yaw filter tracks $x = [ \psi, \dot{\psi} ]^T$ and propagates it with a constant-rate model:

$$
\begin{aligned}
\psi_{k+1} &= \text{wrap}\!\left(\psi_k + dt\,\dot{\psi}_k\right) \\
\dot{\psi}_{k+1} &= \dot{\psi}_k
\end{aligned}
$$

Yaw angles are wrapped to $[-\pi, \pi]$.

### Outlier detection

Measurement residuals are gated with the Normalized Innovation Squared (NIS) test. [`VTE_POS_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_POS_NIS_THRE) applies to every position-axis update, [`VTE_YAW_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_YAW_NIS_THRE) applies to yaw updates, and the defaults (3.84) correspond to a $95\%$ chi-squared confidence interval. Innovations and gate ratios are published on the `vte_aid_*` topics for log review.

### Time alignment

Before fusing any measurement the filters rewind the state to the sample timestamp $t_m$ using the inverse dynamics.

For the static-target position model, letting $\Delta t = t_k - t_{\text{m}}$,

$$
\begin{aligned}
r(t_{\text{m}}) &= r(t_k) + \Delta t\,v^{uav}(t_k) - \tfrac{1}{2}\,\Delta t^2\,a^{uav}, \\
v^{uav}(t_{\text{m}}) &= v^{uav}(t_k) - \Delta t\,a^{uav}, \\
b(t_{\text{m}}) &= b(t_k).
\end{aligned}
$$

When moving-target states are enabled,

$$
\begin{aligned}
r(t_{\text{m}}) &= r(t_k) + \Delta t\,(v^{uav}(t_k) - v^{t}(t_k)) + \tfrac{1}{2}\,\Delta t^2\,(a^{t}(t_k) - a^{uav}), \\
v^{uav}(t_{\text{m}}) &= v^{uav}(t_k) - \Delta t\,a^{uav}, \\
v^{t}(t_{\text{m}}) &= v^{t}(t_k) - \Delta t\,a^{t}, \\
a^{t}(t_{\text{m}}) &= a^{t}(t_k), \\
b(t_{\text{m}}) &= b(t_k).
\end{aligned}
$$

For the orientation filter,

$$
\begin{aligned}
\psi(t_{\text{m}}) &= \text{wrap}\!\left(\psi(t_k) - \Delta t\,\dot{\psi}(t_k)\right), \\
\dot{\psi}(t_{\text{m}}) &= \dot{\psi}(t_k).
\end{aligned}
$$

## Estimator Lifecycle

1. **Task activation**: `VisionTargetEst` enables the position and/or orientation filters when the UAV is performing a task selected in [`VTE_TASK_MASK`](../advanced_config/parameter_reference.md#VTE_TASK_MASK). Bit 0 activates precision landing, while bit 1 keeps the estimator running continuously for debugging; future task bits can enable additional mission profiles.
2. **Initialization**: The position filter waits until at least one position-like observation (vision or GNSS) becomes available. Priority is given to non-GNSS sources so the GNSS bias can be resolved later from the difference between the GNSS baseline and that first relative measurement.
3. **Bias estimation**: When a GNSS observation is valid and a second independent position source becomes available, the GNSS bias vector is set to `pos_rel_gnss - pos_rel_ref`. From that moment GNSS absolute observations include the bias component inside the measurement model.

   In typical precision-landing setups the vehicle hovers within tens of metres from the pad for less than a minute. During that window the atmospheric conditions that shift the output of the GNSS receiver are assumed constant, so the estimator treats the bias as a steady offset. Once the bias has been observed with the help of the relative measurements, it continues to correct the absolute GNSS data even if the vision feed momentarily drops out, allowing the descent to remain centred on the target.
4. **Prediction**: Predictions are performed at the rates commanded by [`VTE_POS_RATE`](../advanced_config/parameter_reference.md#VTE_POS_RATE) and [`VTE_YAW_RATE`](../advanced_config/parameter_reference.md#VTE_YAW_RATE) (default 50 Hz).
5. **Update**: When new sensor observations are available, measurement handlers check the validity of the measurement (timestamp, value, and fusion requested in [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK)), process it, and populate observation buffers. The observations that pass the NIS gates defined by [`VTE_POS_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_POS_NIS_THRE) and [`VTE_YAW_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_YAW_NIS_THRE) are then fused.
6. **Timeout handling** - if no position measurement is fused for [`VTE_BTOUT`](../advanced_config/parameter_reference.md#VTE_BTOUT), the filters are stopped; the published pose/yaw is flagged invalid once [`VTE_TGT_TOUT`](../advanced_config/parameter_reference.md#VTE_TGT_TOUT) elapses without fresh data.

## Measurement Sources

All measurements are fused sequentially. For each observation `z` a one-row Jacobian is formed and applied to a single axis (position filter) or to the yaw state (orientation filter). Enabled sensors are defined by the [`VTE_AID_MASK`](#sensor-fusion-selection) bitmask.

| Source | uORB topic | H structure | Notes |
| --- | --- | --- | --- |
| Target GNSS position | `target_gnss` | $z = r + b$ once the bias is observable, otherwise $z = r$ | The vehicle GNSS sample is interpolated to the target timestamp using the vehicle velocity so the two receivers share a common epoch. Requires [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 0. |
| Mission landing waypoint | `position_setpoint_triplet` | $z = r$ | Provides a fallback absolute reference when target GNSS is unavailable. Enable [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 3 and avoid combining it with target GNSS because only one GNSS bias can be estimated. |
| Vision pose | `fiducial_marker_pos_report` | $z = r$ after rotating the body-frame measurement into NED | Uses the message variances, or scales with altitude when [`VTE_EV_NOISE_MD`](../advanced_config/parameter_reference.md#VTE_EV_NOISE_MD)=1. Recent vision fusions are required for EKF aiding. |
| Vehicle GNSS velocity | `sensor_gps` | $z = v^{uav}$ | Removes rotation-induced velocity using the EKF2 GPS offset (`EKF2_GPS_POS_*`). Enable [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 1. |
| Target GNSS velocity (moving mode) | `target_gnss` | $z = v^{t}$ | Available only when `CONFIG_VTEST_MOVING=y` and [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 4 is set. |
| Vision yaw | `fiducial_marker_yaw_report` | $z = \psi$ | Variance taken from the message or scaled with altitude via [`VTE_EVA_NOISE`](../advanced_config/parameter_reference.md#VTE_EVA_NOISE). |

All innovation data are published on dedicated topics (`vte_aid_gps_pos_target`, `vte_aid_fiducial_marker`, `vte_aid_ev_yaw`, etc.), making it easy to inspect residuals and test ratios in logs. Rejected measurements still emit innovation messages with `fused=false` so tuning sessions can identify time skew or noise mismatches.

> [!NOTE]
> UWB and IRLock are candidates for future development once representative test data is available.

### Measurements validity

Two time horizons guard incoming data: [`VTE_M_REC_TOUT`](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT) defines how old a sample can be and still be considered for fusion, while [`VTE_M_UPD_TOUT`](../advanced_config/parameter_reference.md#VTE_M_UPD_TOUT) bounds how long cached measurements remain valid in the estimator state. The published outputs toggle `*_valid` false after [`VTE_TGT_TOUT`](../advanced_config/parameter_reference.md#VTE_TGT_TOUT) (and the estimators fully stop once [`VTE_BTOUT`](../advanced_config/parameter_reference.md#VTE_BTOUT) is exceeded). Measurements timestamped in the future relative to the latest prediction are always rejected and reported on the innovation topics.

## Configuration

### Module enable and scheduling

- [`VTE_EN`](../advanced_config/parameter_reference.md#VTE_EN) - global module enable (reboot required).
- [`VTE_POS_EN`](../advanced_config/parameter_reference.md#VTE_POS_EN) / [`VTE_YAW_EN`](../advanced_config/parameter_reference.md#VTE_YAW_EN) - enable the position and orientation filters respectively (reboot required).
- [`VTE_POS_RATE`](../advanced_config/parameter_reference.md#VTE_POS_RATE), [`VTE_YAW_RATE`](../advanced_config/parameter_reference.md#VTE_YAW_RATE) - desired update rates for the position and yaw filters; the work item adapts its scheduling to maintain these targets.
- [`VTE_BTOUT`](../advanced_config/parameter_reference.md#VTE_BTOUT), [`VTE_TGT_TOUT`](../advanced_config/parameter_reference.md#VTE_TGT_TOUT) - timeouts for estimator shutdown and published validity flags.
- [`VTE_M_REC_TOUT`](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT), [`VTE_M_UPD_TOUT`](../advanced_config/parameter_reference.md#VTE_M_UPD_TOUT) - maximum ages for measurements to be fused or retained.

### Task selection

[`VTE_TASK_MASK`](../advanced_config/parameter_reference.md#VTE_TASK_MASK)selects runtime tasks during which the estimators perform computations and estimate the state of the target.

| Bit | Value | Meaning |
| --- | --- | --- |
| 0 | `1` | precision landing |
| 1 | `2` | DEBUG, always active |

> [!IMPORTANT]
> Precision landing yaw control is disabled by default. Enable [PLD_YAW_EN](../advanced_config/parameter_reference.md#PLD_YAW_EN) when you want the mission controller to align the vehicle with the target heading, and configure the landing waypoint for precision landing (see [Mission precision landing](../advanced_features/precland.md#mission)). Without both settings the aircraft will only track the position estimate from the Vision Target Estimator.

### Sensor fusion selection

[`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) defines which measurements can be fused:

| Bit | Value | Meaning |
| --- | --- | --- |
| 0 | `1` | Target GNSS position |
| 1 | `2` | Vehicle GNSS velocity |
| 2 | `4` | Vision-relative position |
| 3 | `8` | Mission landing waypoint |
| 4 | `16` | Target GNSS velocity (moving mode only) |

### Noise and gating

- Adjust [`VTE_POS_UNC_IN`](../advanced_config/parameter_reference.md#VTE_POS_UNC_IN), [`VTE_VEL_UNC_IN`](../advanced_config/parameter_reference.md#VTE_VEL_UNC_IN), [`VTE_BIA_UNC_IN`](../advanced_config/parameter_reference.md#VTE_BIA_UNC_IN), [`VTE_ACC_UNC_IN`](../advanced_config/parameter_reference.md#VTE_ACC_UNC_IN) to reflect initial uncertainty. Larger numbers slow initial convergence but reduce the chance of aggressive transients.
- [`VTE_ACC_D_UNC`](../advanced_config/parameter_reference.md#VTE_ACC_D_UNC), [`VTE_ACC_T_UNC`](../advanced_config/parameter_reference.md#VTE_ACC_T_UNC), [`VTE_BIAS_UNC`](../advanced_config/parameter_reference.md#VTE_BIAS_UNC) - process noise for vehicle acceleration, target acceleration, and GNSS bias. Increase [`VTE_ACC_D_UNC`](../advanced_config/parameter_reference.md#VTE_ACC_D_UNC) if the estimator lags behind real motion, and raise [`VTE_BIAS_UNC`](../advanced_config/parameter_reference.md#VTE_BIAS_UNC) if GNSS bias corrections should respond more quickly. In moving-target builds, use [`VTE_ACC_T_UNC`](../advanced_config/parameter_reference.md#VTE_ACC_T_UNC) to match expected target manoeuvrability.
- [`VTE_POS_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_POS_NIS_THRE), [`VTE_YAW_NIS_THRE`](../advanced_config/parameter_reference.md#VTE_YAW_NIS_THRE) - chi-squared thresholds (e.g. 3.84 corresponds to $95\%$ confidence).
- Measurement variance floors: [`VTE_GPS_P_NOISE`](../advanced_config/parameter_reference.md#VTE_GPS_P_NOISE), [`VTE_GPS_V_NOISE`](../advanced_config/parameter_reference.md#VTE_GPS_V_NOISE), [`VTE_EVP_NOISE`](../advanced_config/parameter_reference.md#VTE_EVP_NOISE), [`VTE_EVA_NOISE`](../advanced_config/parameter_reference.md#VTE_EVA_NOISE). Parameter updates below the hard-coded `kMinObservationNoise = 1e-2` (see `src/modules/vision_target_estimator/common.h`) are rejected to keep the Kalman gains bounded.
- [`VTE_EV_NOISE_MD`](../advanced_config/parameter_reference.md#VTE_EV_NOISE_MD) - when set to 1 the estimator ignores variances from the vision message and scales from parameters using range data.

### Sensor-specific settings

- **GNSS antenna offsets** - `VisionTargetEst` reads [`EKF2_GPS_POS_X`](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [`EKF2_GPS_POS_Y`](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y), and [`EKF2_GPS_POS_Z`](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) to remove position and rotation-induced velocity offsets before forming GNSS measurements, so configure EKF2 with the correct antenna location.
- **Vision range scaling** - with [`VTE_EV_NOISE_MD`](../advanced_config/parameter_reference.md#VTE_EV_NOISE_MD)=1, the vision position noise scales with the measured range, guarding against under-reported uncertainties as the vehicle gains altitude.

### EKF2 aiding

When [`VTE_EKF_AID`](../advanced_config/parameter_reference.md#VTE_EKF_AID)=1, `VisionTargetEst` sets `landing_target_pose.rel_vel_valid` so EKF2 can fuse the relative target velocity. This requires all of the following:

- `landing_target_pose.is_static = true`, which is only the case when `CONFIG_VTEST_MOVING` is disabled.
- `landing_target_pose.rel_pos_valid = true` and a recent relative measurement (within [`VTE_M_REC_TOUT`](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT)).
- A vision-relative position measurement has been fused.

If any condition fails, `rel_vel_valid` is cleared and EKF2 ignores the input.

### MAVLink Messages

To run the vision target estimator, `CONFIG_MAVLINK_DIALECT="development"` is required to expose `TARGET_RELATIVE` for vision-based observations and `TARGET_ABSOLUTE` for target-mounted GNSS data, two PX4-specific messages that feed the estimator. Note that there are open discussions to include them into the common MAVLink dialect.

#### TARGET_RELATIVE (ID 511)

`TARGET_RELATIVE` extends the `LANDING_TARGET` message with a full 3D report that includes orientation and measurement uncertainty:

- A coordinate frame selector (`TARGET_OBS_FRAME`) and a sensor quaternion (`q_sensor`) that, when provided, rotates the measurement into vehicle-carried NED.
- Target pose (`x`, `y`, `z`, `q_target`) and variances (`pos_std`, `yaw_std`), collected from onboard vision pipelines.

`mavlink_receiver` validates the frame/type and handles the message differently depending on the Vision Target Estimator status:

- When `VTE_EN=0`, the measurement is rotated (using `q_sensor` or the vehicle attitude for body-frame reports) and published straight to `landing_target_pose` so precision-landing can operate without the estimator.
- When `VTE_EN=1`, the message is split into `fiducial_marker_pos_report` and `fiducial_marker_yaw_report`, preserving the rotated pose and yaw variance. `VisionTargetEst` consumes these uORB topics to drive the position and orientation filters.

#### TARGET_ABSOLUTE (ID 510)

`TARGET_ABSOLUTE` reports the target's absolute state when it carries its own GNSS (and optionally IMU). A capability bitmap advertises which fields are valid. PX4 maps the available content into the `target_gnss` uORB topic:

- Bit 0 (position) triggers publication of latitude/longitude/altitude along with the horizontal and vertical accuracy estimates (`position_std`).
- Bit 1 (velocity) forwards the body-frame velocity vector (`vel`) and its standard deviations (`vel_std`).
- Additional fields (acceleration, quaternion `q_target`, rates, uncertainties) are not supported and reserved  for future fusion logic once flight testing is available.

### Moving-target projection parameters (experimental)

When `CONFIG_VTEST_MOVING` is active, [`VTE_MOVING_T_MIN`](../advanced_config/parameter_reference.md#VTE_MOVING_T_MIN) and [`VTE_MOVING_T_MAX`](../advanced_config/parameter_reference.md#VTE_MOVING_T_MAX) determine how far ahead the target position is projected when publishing the absolute landing setpoint. The estimator computes an intersection time $\Delta t$ by constraining $|z_{rel}| / |v_{descent}|$ between the two parameters, then shifts the absolute landing target by $\Delta t$ along the estimated target velocity (with a correction from the target acceleration state). The goal is to command the vehicle towards where the moving target will be at touchdown, not where it was at the last measurement.

## Gazebo Classic Simulation

Run the SITL world `gazebo-classic_iris_irlock` to simulate precision landing using the VTEST fusing vision (ArUco-based) and target GNSS aiding. The world name is retained for historical reasons. The models were introduced in [PX4/PX4-SITL_gazebo-classic#950](https://github.com/PX4/PX4-SITL_gazebo-classic/pull/950).

> [!TIP]
> The ArUco vision observation path implemented in `Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_aruco_plugin.cpp` provides a concrete example of how to obtain a vision-based observation of a target and how to publish the `TARGET_RELATIVE` message.

1. Launch the simulator with:

   ```sh
   make px4_sitl gazebo-classic_iris_irlock
   ```

2. If CMake cannot locate the expected OpenCV version, query the version present on your system (`opencv_version --short` or `pkg-config --modversion opencv4`) and update `Tools/simulation/gazebo-classic/sitl_gazebo-classic/CMakeLists.txt` accordingly, for example:

   ```cmake
   find_package(OpenCV 4.2.0 REQUIRED EXACT)
   ```

   Re-run the build after adjusting the `find_package` line.

> [!TIP]Tips
> - **Pad visibility**: In `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/land_pad/land_pad.sdf`, increase the visual box size to `1.5 1.5 0.01` so the pad stays in view longer while the vehicle descends.
> - **Mission waypoint bias**: Enable vision and mission position aiding in [`VTE_AID_MASK`](../advanced_config/parameter_reference.md#VTE_AID_MASK) (set bits 2 and 3, disable bit 0). Place the landing waypoint 3 to 4 m away from the pad in QGroundControl to watch the UAV correct towards the pad once it is detected. In the logs, observe how the GNSS bias compensates for the distance between the land waypoint and the actual pad.
> - **Measurement noise experiments**: The ArUco plugin publishes nominal standard deviations through `set_std_x` and `set_std_y` in `Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_aruco_plugin.cpp`. Modify these assignments, and optionally the camera noise block in `.../models/aruco_cam/aruco_cam.sdf`, to see how innovation gates react to noisier vision.
> - **Moving target trials**: To emulate a moving pad, edit `<initial_velocity>` in `.../models/land_pad/land_pad.sdf` (for example `0.5 0 0`) and enable `CONFIG_VTEST_MOVING` so the estimator tracks the target velocity.

## Generated SymForce Functions

`src/modules/vision_target_estimator/Position/vtest_derivation/derivation.py` contains the symbolic model. During the build:

- Pre-generated Jacobians under `vtest_derivation/generated` are copied into the build tree by default.
- Setting `VTEST_SYMFORCE_GEN=ON` (and having SymForce available) regenerates the functions at configure time.
- When both `CONFIG_VTEST_MOVING` and SymForce are enabled, the moving-target Jacobians are generated on the fly so that a 5-state model is available.
- Developers can refresh the committed reference outputs with `-DVTEST_UPDATE_COMMITTED_DERIVATION=ON`.

The generated files (`predictState.h`, `predictCov.h`, `computeInnovCov.h`, `syncState.h`, `state.h`) are included through the build directory and must not be edited manually. See the [Vision Target Estimator deep dive](../advanced_features/vision_target_estimator_advanced.md#regenerating-the-symbolic-model) for regeneration prerequisites and troubleshooting tips.

## Monitoring

For a detailed log analysis guidance, see the [Vision Target Estimator deep dive](../advanced_features/vision_target_estimator_advanced.md).

- `landing_target_pose.rel_pos_valid` and `.abs_pos_valid` indicate whether recent measurements support relative and absolute positioning.
- `vision_target_est_position` exposes every state component (relative position, vehicle velocity, GNSS bias, and optional target motion) together with diagonal covariance entries.
- `vision_target_est_orientation` provides yaw, yaw-rate, and their variances.
- Innovations published on `vte_aid_*` topics include the raw measurement, innovation, innovation variance, and chi-squared test ratio for post-flight analysis.

## Operational Notes

- Accurate timestamp alignment between measurement sources is critical. Large skews will cause innovations to fail the NIS gate and be rejected.
- Absolute target pose is only published when `vehicle_local_position` reports a valid local frame.
- When you expect yaw alignment during landing, enable [`PLD_YAW_EN`](../advanced_config/parameter_reference.md#PLD_YAW_EN) and configure the mission land item for precision landing as described in [Precision landing](../advanced_features/precland.md#mission). In practice this means setting the QGroundControl land waypoint `Precision landing` drop-down (or `MAV_CMD_NAV_LAND` `param2`) to Opportunistic or Required so the controller requests the estimator output. Otherwise only positional cues are used.
- For extended parameter descriptions, log analysis checklists, and developer workflows, refer to the [Vision Target Estimator deep dive](../advanced_features/vision_target_estimator_advanced.md).

> [!WARNING]
> The moving-target mode is experimental and has not been flight tested; use it only for controlled experiments.

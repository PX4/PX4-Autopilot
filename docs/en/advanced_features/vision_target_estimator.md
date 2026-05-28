# Vision Target Estimator (VTE)

<Badge type="tip" text="PX4 v1.18" /> <Badge type="warning" text="Experimental" />

The Vision Target Estimator (VTE) estimates where a target is, relative to the vehicle, by combining a camera-based detection of the target (for example an ArUco fiducial marker) with one or more absolute position references: the vehicle's own GNSS, the mission landing waypoint, and/or a GNSS receiver mounted on the target itself.
Its main use is [precision landing](../advanced_features/precland.md), where the vehicle needs to touch down on a small, well-defined spot rather than the approximate mission waypoint.

This page is for developers who want to enable VTE on a custom build, tune it for their vehicle, or integrate a new vision pipeline that publishes landing-target observations.
It assumes familiarity with PX4 and basic state estimation, but does not require prior knowledge of the module itself.
For more depth see the [Vision Target Estimator deep dive](../advanced_features/vision_target_estimator_advanced.md).

::: warning
VTE is a beta feature, disabled in default board configurations, and should only be enabled on custom builds after careful bench and flight testing.
:::

## Table of Contents

[[toc]]

## Building the Module

The estimator is not part of the default PX4 board configurations.

To enable a build that includes the module you need to modify the [KConfig board configuration](../hardware/porting_guide_config.md) for your target board (or create a custom `.px4board` file).
The keys and values that need to be present are:

- `CONFIG_MODULES_VISION_TARGET_ESTIMATOR=y`: _Enable_ VTE in firmware
- `CONFIG_MODULES_LANDING_TARGET_ESTIMATOR=n`: _Disable_ the landing target estimator (both modules publish [`landing_target_pose`](../msg_docs/LandingTargetPose.md) and will conflict if enabled together).
- `CONFIG_MAVLINK_DIALECT="development"`: Enable using the development dialect.
  Note that this step will no longer be required once the MAVLink messages used by VTE have been validated.

Two prebuilt targets are available:

- `make px4_fmu-v6c_visionTargetEstStatic`: static-target hardware build.
- `make px4_sitl_default`: SITL static-target build.

For the experimental moving-target build, see [Moving-target mode](../advanced_features/vision_target_estimator_advanced.md#moving-target-mode-experimental).

## Estimator Overview

This section describes how the filter works internally.
It is recommended background reading to understand the underlying mechanics, though it is not strictly required to set up or operate the feature.

### Architecture and Core Loop

The Vision Target Estimator runs two independent estimators at a fixed 50 Hz:

- **Position filter:** Tracks where the target is relative to the vehicle.
  It is structured as three decoupled 1D Kalman filters (one per NED axis).
- **Orientation filter:** Tracks the target yaw on its own state.

Each filter alternates between two operations:

<a id="dynamic-models"></a>

- **Prediction step:** Propagates the state forward using the vehicle motion model.

  ::: details Click to view the mathematical model

  The per-axis position state is $x = [ r, v^{uav}, b ]^T$: the relative NED displacement (target minus vehicle), the vehicle velocity, and the offset between the absolute target reference (GNSS or mission waypoint) and the vision-derived target position.
  Assuming constant NED acceleration input $a^{uav}$ over the integration interval $dt$:

  $$
  \begin{aligned}
  r_{k+1} &= r_k - dt\thinspace v^{uav}_k - \tfrac{1}{2}\thinspace dt^2\thinspace a^{uav} \\
  v^{uav}_{k+1} &= v^{uav}_k + dt\thinspace a^{uav} \\
  b_{k+1} &= b_k
  \end{aligned}
  $$

  Once the bias is known, GNSS can keep the estimate centred on the target even if vision drops out.

  The yaw filter tracks $x = [ \psi, \dot{\psi} ]^T$ with a constant-rate prediction (yaw wrapped to $[-\pi, \pi]$):

  $$
  \begin{aligned}
  \psi_{k+1} &= \text{wrap}\negthinspace\left(\psi_k + dt\thinspace\dot{\psi}_k\right) \\
  \dot{\psi}_{k+1} &= \dot{\psi}_k
  \end{aligned}
  $$

  Unknown physical disturbances are modelled as continuous-time Gaussian white noise.
  The runtime spectral densities ([VTE_ACC_D_UNC](../advanced_config/parameter_reference.md#VTE_ACC_D_UNC), [VTE_BIAS_UNC](../advanced_config/parameter_reference.md#VTE_BIAS_UNC), [VTE_YAW_ACC_UNC](../advanced_config/parameter_reference.md#VTE_YAW_ACC_UNC)) and the initial-variance parameters are listed in [Noise](#noise); for the full derivation see [Dynamic model process noise](../advanced_features/vision_target_estimator_advanced.md#dynamic-model-process-noise).

  For the experimental moving-target mode that adds target velocity and acceleration states, see [Moving-target mode](../advanced_features/vision_target_estimator_advanced.md#moving-target-mode-experimental).
  :::

- **Update step:** Corrects the state whenever a new sensor observation arrives and is accepted for fusion.
  See [Aid-source diagnostics](../advanced_features/vision_target_estimator_advanced.md#aid-source-diagnostics) to debug the fusion update step.

### Initialization and Task Scheduling

Fusion starts as soon as the filter is initialized.
The position filter requires a recent vehicle velocity estimate along with at least one position-like observation to begin.
The orientation filter starts immediately upon receiving the first valid vision yaw sample.

The estimators only run while a runtime **task** is active.
Tasks are evaluated in priority order, and the first task whose readiness conditions are satisfied is the one that executes.
See [Task Selection](#task-selection) to learn how to select tasks via bitmask.

### Bias Estimation {#bias-estimation}

The position filter actively estimates the bias between the absolute target reference (GNSS or mission waypoint) and the vision-derived target position.
Once this bias is observed, the corrected absolute reference effectively becomes a second relative-position sensor.
This allows the vehicle to safely touch down on the target even if vision is briefly lost (e.g., due to motion blur, partial occlusion, or the marker leaving the camera's field of view near the ground).
See [Bias Initialisation](#bias-initialisation) and [Noise](#noise) for bias configuration.

::: details Click to view the bias initialization logic

The GNSS bias $b$ becomes observable only when both GNSS and vision are available, and VTE takes one of two paths depending on which source arrived first.

- **Vision-first:** When vision is already the active position reference, the bias is activated immediately on the first joint sample.
- **GNSS-first:** When GNSS is active first, VTE low-pass filters the early raw samples (tuned by [VTE_BIA_AVG_THR](../advanced_config/parameter_reference.md#VTE_BIA_AVG_THR) and [VTE_BIA_AVG_TOUT](../advanced_config/parameter_reference.md#VTE_BIA_AVG_TOUT)) so that vision is only fused once the offset has settled.

For the full state-reset rules, the LPF exit condition, and the stale-GNSS fallback, see [Bias initialization design](../advanced_features/vision_target_estimator_advanced.md#bias-initialization-design).
:::

### Time Alignment (Latency Compensation) {#time-alignment}

Vision and GNSS measurements often reach the autopilot with non-negligible transport or processing latency.
VTE compensates for this by fusing delayed samples against the predicted state at their _original_ timestamp, rather than the current one.
This relies on an **Out-of-Sequence Measurements (OOSM)** approximation using a history-consistent projected correction strategy.
For buffer sizing and algorithm specifics, see [OOSM Implementation](../advanced_features/vision_target_estimator_advanced.md#oosm-implementation).

### Fallbacks and Timeouts

If no measurement is fused for a sustained period, the affected filter will coast for a brief window before resetting.
Once reset, it will automatically retry as soon as an enabled fusion source becomes available again.
See [Timeouts](#timeouts) to configure the duration of these fallback windows.

## Configuration

### Module Enable

Set the [VTE_EN](../advanced_config/parameter_reference.md#VTE_EN) parameter `1` to run the estimator (reboot required).

To enable position and/or yaw tracking set the following parameters (then reboot):

- [VTE_POS_EN](../advanced_config/parameter_reference.md#VTE_POS_EN): Set to `1` to enable position estimation (track the target position relative to the vehicle).
- [VTE_YAW_EN](../advanced_config/parameter_reference.md#VTE_YAW_EN): Set to `1` to enable orientation estimation (track the target yaw).

  This is disabled by default.
  Enable it if your vision pipeline reports a target heading.

### Timeouts

The default timeouts are appropriate for a typical precision-landing approach where measurements keep arriving until touchdown.
Raise them only when you expect a gap in fresh data.
A common example is a vision-only setup where the marker leaves the camera frame in the final metres of descent.

When that applies:

- Raise [VTE_TGT_TOUT](../advanced_config/parameter_reference.md#VTE_TGT_TOUT) past the expected gap so the published target pose stays marked valid.
  The precision-landing controller stops following the target once this expires.
- Raise [VTE_BTOUT](../advanced_config/parameter_reference.md#VTE_BTOUT) past the expected gap if you want the filter to coast on its last estimate.
  Otherwise the filter is reset and will restart as soon as enabled fusion sources reappear.

Measurements timestamped in the future relative to the latest filter prediction are always rejected.

### Measurement Freshness

[VTE_M_REC_TOUT](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT) and [VTE_M_UPD_TOUT](../advanced_config/parameter_reference.md#VTE_M_UPD_TOUT) decide how long an incoming sample stays usable:

- [VTE_M_REC_TOUT](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT) is the maximum age at which a new measurement is still eligible for fusion.
- [VTE_M_UPD_TOUT](../advanced_config/parameter_reference.md#VTE_M_UPD_TOUT) bounds how long a cached observation remains valid inside the filter state.

These are independent of the timeouts above.
Tune them to the dynamics of your platform and target, not to a fixed value:

- For fast-moving targets and aggressive vehicles, keep both values **short**: stale data describes a world that has already changed, so fusing it pulls the filter in the wrong direction.
- For larger vehicles with slow dynamics and modest roll, pitch, yaw rates, the defaults (or even larger values) work fine: a slightly older sample is still informative because the relative geometry has not significantly changed.

If you are unsure, leave the defaults and watch the logs.
Tracking lag or overshoot on a fast platform usually means these values are too large.
Legitimate fusions being missed because samples arrive just outside the window means they are too small.

### Task Selection

[VTE_TASK_MASK](../advanced_config/parameter_reference.md#VTE_TASK_MASK) is a bitmask that selects the runtime tasks for which the estimators perform computations and estimate the state of the target.
Set the indicated bit to enable the corresponding task.

| Bit | Task                  |
| --- | --------------------- |
| 0   | Precision landing     |
| 1   | DEBUG (always active) |

::: warning
The controller only consumes the VTE output once the landing waypoint is configured for precision landing (see [Mission precision landing](../advanced_features/precland.md#mission)).
Without it, the VTE keeps running but its estimates are ignored.

Yaw alignment is additionally gated by [PLD_YAW_EN](../advanced_config/parameter_reference.md#PLD_YAW_EN), which is disabled by default.
With precision landing enabled but `PLD_YAW_EN` disabled, only the position estimate is tracked.
:::

### Sensor Fusion Selection

[VTE_AID_MASK](../advanced_config/parameter_reference.md#VTE_AID_MASK) is a bitmask that defines which measurements can be fused:

| Bit | Meaning                                                                                                                             |
| --- | ----------------------------------------------------------------------------------------------------------------------------------- |
| 0   | Target GNSS position                                                                                                                |
| 1   | Vehicle GNSS velocity                                                                                                               |
| 2   | Vision-based relative pose                                                                                                          |
| 3   | Mission landing waypoint                                                                                                            |
| 4   | Target GNSS velocity ([moving mode](../advanced_features/vision_target_estimator_advanced.md#moving-target-mode-experimental) only) |

Bit 2 also enables processing of `fiducial_marker_yaw_report` in the orientation filter.

**Enable more than one source when you can.**
Each source observes a different combination of states (see the per-source observation models in the details block below), so multiple sources keep the filter robust through momentary dropouts.

- **Vehicle GNSS velocity is important.** Only disable this source when you have a specific reason, for example no GNSS available.
  Without it, vision dropouts cause a visible relative-position drift that snaps back when vision returns.
  The deep dive plots both cases side by side in [Vision dropout behaviour](../advanced_features/vision_target_estimator_advanced.md#vision-dropout-behaviour).

- **An absolute reference makes precision landing robust to vision loss.** Fusing an absolute reference (target GNSS or the mission landing waypoint) lets the filter estimate the bias between the absolute frame and the vision frame.
  Once the bias is observed, the corrected absolute observation effectively becomes a second relative-position sensor: the vehicle can still touch down on the target even when vision is no longer available (for example because the marker leaves the camera field of view in the final metres of descent, or because of motion blur or partial occlusion).
  This is especially important for large targets where vision is expected to drop out before touchdown.
  The deep dive analyses this on a real flight in [Vision occlusion during descent](../advanced_features/vision_target_estimator_advanced.md#vision-occlusion-during-descent).

::: info

**Target GNSS position and mission landing waypoint are mutually exclusive.**
Both provide the absolute reference for the same GNSS/vision bias, and only one bias can be estimated at a time.
If both bits are set, the module disables the mission landing waypoint at startup and prints a warning, but it is cleaner to pick one explicitly: use target GNSS when a receiver is mounted on the target (most accurate), and the mission landing waypoint when no receiver is available.
:::

<a id="measurement-sources"></a>

:::: details Click to view measurement source specifications and uORB topics

All measurements are fused sequentially.
For each observation $z$ a one-row Jacobian is formed and applied to a single axis (position filter) or to the yaw state (orientation filter).
Enabled sensors are defined by the [VTE_AID_MASK](../advanced_config/parameter_reference.md#VTE_AID_MASK) bitmask.
The state symbols used in the `H` column ($r$, $v^{uav}$, $b$, $v^{t}$, $\psi$) are defined in the [Dynamic models](#dynamic-models) block above.

| Source                             | uORB topic                                                                   | H structure                                                           | Notes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| ---------------------------------- | ---------------------------------------------------------------------------- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Target GNSS position               | [`target_gnss`](../msg_docs/TargetGnss.md)                                   | $z = r + b$ once the bias is observable, otherwise $z = r$            | The vehicle GNSS sample is interpolated to the target timestamp using the vehicle velocity so the two receivers share a common epoch. Requires [VTE_AID_MASK](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 0. Before bias activation, this source is held back if the estimator is already vision-referenced.                                                                                                                                                                                                                                                                                                                                       |
| Mission landing waypoint           | `navigator_mission_item` with validated `position_setpoint_triplet` fallback | $z = r$                                                               | Provides a fallback absolute reference when target GNSS is unavailable. At precision-land task start VTE caches the logical landing waypoint published by [Navigator](../modules/modules_controller.md#navigator) and keeps using that cached point even after precland rewrites the live triplet. The triplet remains a fallback for modes that do not publish `navigator_mission_item`. Enable [VTE_AID_MASK](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 3 and avoid combining it with target GNSS because only one GNSS bias can be estimated. Before bias activation, this source is held back if the estimator is already vision-referenced. |
| Vision pose                        | [`fiducial_marker_pos_report`](../msg_docs/FiducialMarkerPosReport.md)       | $z = r$ after rotating the measurement (`rel_pos`) into NED using `q` | Uses the message variances, lower-bounded by [VTE_EVP_NOISE](../advanced_config/parameter_reference.md#VTE_EVP_NOISE). Recent vision fusions are required for EKF aiding. During the initial GNSS/vision bias averaging phase, valid vision samples update the bias low-pass filter but are not fused into the position state yet. This averaging phase only exists when GNSS became the active reference first.                                                                                                                                                                                                                                                   |
| Vehicle GNSS velocity              | `sensor_gps`                                                                 | $z = v^{uav}$                                                         | Removes rotation-induced velocity using the vehicle GPS antenna offset parameters (`SENS_GPS0_OFF*`). Enable [VTE_AID_MASK](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 1.                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| Target GNSS velocity (moving mode) | `target_gnss`                                                                | $z = v^{t}$                                                           | Only used by the experimental [Moving-target mode](../advanced_features/vision_target_estimator_advanced.md#moving-target-mode-experimental).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| Vision yaw                         | [`fiducial_marker_yaw_report`](../msg_docs/FiducialMarkerYawReport.md)       | $z = \psi$                                                            | Only source used by the orientation filter. Requires [VTE_YAW_EN](../advanced_config/parameter_reference.md#VTE_YAW_EN) and [VTE_AID_MASK](../advanced_config/parameter_reference.md#VTE_AID_MASK) bit 2. Variance is taken from the message and lower-bounded by [VTE_EVA_NOISE](../advanced_config/parameter_reference.md#VTE_EVA_NOISE).                                                                                                                                                                                                                                                                                                                        |

All innovation data are published on dedicated topics (`vte_aid_gps_pos_target`, `vte_aid_fiducial_marker`, `vte_aid_ev_yaw`, etc.), making it easy to inspect residuals and test ratios in logs.
Every fusion attempt is published, including rejections: the per-axis `fusion_status` enum records the outcome (fused immediately, fused via OOSM history replay, rejected by the NIS gate, rejected as too old or too new, etc.), so tuning sessions can isolate time skew, noise mismatch, or buffer staleness without guessing.

::: info
UWB and IRLock are candidates for future development once representative test data is available.
:::

::::

### Noise

This section covers how the [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) at the heart of VTE weighs sensor measurements against its own prediction.
Noise parameters set how much trust to place in each input.
The wrong balance shows up as either an over-responsive estimator that chases per-sample jitter or one that lags real motion.
Skip this section if the filter performs well with default settings on your platform.

Start from the defaults: they are tuned for a typical setup of consumer GNSS plus a marker-based vision pipeline.
Re-tune only when log analysis points at a specific symptom.
The deep dive walks through what a healthy filter looks like in [Expected Plot Dashboards](../advanced_features/vision_target_estimator_advanced.md#expected-plot-dashboards) and how to balance the parameters in [Balancing process and observation noise](../advanced_features/vision_target_estimator_advanced.md#balancing-process-and-observation-noise).

::: details Click to view a guide for advanced noise tuning

- **Initial state variances** ([VTE_POS_UNC_IN](../advanced_config/parameter_reference.md#VTE_POS_UNC_IN), [VTE_VEL_UNC_IN](../advanced_config/parameter_reference.md#VTE_VEL_UNC_IN), [VTE_BIA_UNC_IN](../advanced_config/parameter_reference.md#VTE_BIA_UNC_IN), [VTE_ACC_UNC_IN](../advanced_config/parameter_reference.md#VTE_ACC_UNC_IN), [VTE_YAW_UNC_IN](../advanced_config/parameter_reference.md#VTE_YAW_UNC_IN)) seed the filter at initialization or reset.
  - Lower them only if you see aggressive transients on `vte_position` immediately after activation.
  - Raise them if convergence is very slow.
    Updates while the estimator is already running take effect on the next start.
- **Process noise** ([VTE_ACC_D_UNC](../advanced_config/parameter_reference.md#VTE_ACC_D_UNC), [VTE_BIAS_UNC](../advanced_config/parameter_reference.md#VTE_BIAS_UNC), [VTE_YAW_ACC_UNC](../advanced_config/parameter_reference.md#VTE_YAW_ACC_UNC), and [VTE_ACC_T_UNC](../advanced_config/parameter_reference.md#VTE_ACC_T_UNC) for moving-target builds) controls how fast the predicted state variance grows between measurements, which in turn sets how strongly each new observation moves the filter.
  - Lower them when the state follows per-sample jitter rather than the underlying trend.
  - Raise them when the estimator lags real motion or repeated innovations show that the prediction uncertainty is too optimistic.
  - For a step-by-step recipe with worked examples, see [Balancing process and observation noise](../advanced_features/vision_target_estimator_advanced.md#balancing-process-and-observation-noise).
- **Sensor noise floors** ([VTE_GPS_P_NOISE](../advanced_config/parameter_reference.md#VTE_GPS_P_NOISE), [VTE_GPS_V_NOISE](../advanced_config/parameter_reference.md#VTE_GPS_V_NOISE), [VTE_EVP_NOISE](../advanced_config/parameter_reference.md#VTE_EVP_NOISE), [VTE_EVA_NOISE](../advanced_config/parameter_reference.md#VTE_EVA_NOISE)) are lower bounds on the per-sample standard deviation each sensor is allowed to report.
  - Do not push these towards zero: a very small floor tells the filter to trust every sample fully, which makes it chase per-sample jitter and can drive the controller into oscillations.
    The runtime enforces a hard minimum to keep Kalman gains bounded, but the safer practice is to set a realistic floor that matches the actual sensor accuracy.
  - Raise the floor for a sensor that under-reports its noise (you will see the filter chasing every sample of that sensor in `vte_position` and the corresponding `vte_aid_*.observation`)
  - Do not modify the floor if the reported variance already exceeds the floor.
  - See [Between observation sources](../advanced_features/vision_target_estimator_advanced.md#between-observation-sources) for typical mis-tunes.

:::

### Outlier Detection

Gating thresholds decide which samples to fuse and which to reject as outliers.
It is controlled by the position and yaw NIS gates ([VTE_POS_NIS_THRE](../advanced_config/parameter_reference.md#VTE_POS_NIS_THRE), [VTE_YAW_NIS_THRE](../advanced_config/parameter_reference.md#VTE_YAW_NIS_THRE)), which default to a 95% chi-squared confidence interval.
There is no need to modify the default unless log analysis shows repeated `fusion_status = STATUS_REJECT_NIS` on valid samples or corrupted samples still passing fusion.
For the gating logic and tuning workflow, see [Outlier Detection](../advanced_features/vision_target_estimator_advanced.md#outlier-detection).

### Bias Initialisation

**Bias averaging** ([VTE_BIA_AVG_THR](../advanced_config/parameter_reference.md#VTE_BIA_AVG_THR), [VTE_BIA_AVG_TOUT](../advanced_config/parameter_reference.md#VTE_BIA_AVG_TOUT)) only takes effect when GNSS becomes active before vision.
The defaults work for typical consumer GNSS plus marker-based vision.
See [Bias initialization design](../advanced_features/vision_target_estimator_advanced.md#bias-initialization-design) for the full state-machine behaviour.

- Set `VTE_BIA_AVG_TOUT=0` to skip averaging and activate the bias on the first joint sample.
- Raise `VTE_BIA_AVG_THR` if a noisy vision pipeline cannot satisfy the stability criterion within the timeout.

### Sensor-specific Settings

- **GNSS antenna offsets**: The VTE removes position- and rotation-induced velocity offsets before forming GNSS measurements.
  Configure the vehicle GPS antenna location with [SENS_GPS0_OFFX](../advanced_config/parameter_reference.md#SENS_GPS0_OFFX), [SENS_GPS0_OFFY](../advanced_config/parameter_reference.md#SENS_GPS0_OFFY), and [SENS_GPS0_OFFZ](../advanced_config/parameter_reference.md#SENS_GPS0_OFFZ).

During the final descent, GNSS-derived horizontal velocity can degrade (multipath, low-altitude geometry) right when the vehicle needs it most.
VTE can feed its vision-derived relative velocity to [EKF2](../advanced_config/tuning_the_ecl_ekf.md) as an auxiliary measurement to keep the local-position estimate stable when GNSS quality drops near the ground.

::: details Click for more details on EKF2 aiding (auxiliary velocity)

For a static target, the relative velocity tracked by the position filter (see [Estimator overview](#estimator-overview)) can be fed into the main vehicle state estimator ([EKF2](../advanced_config/tuning_the_ecl_ekf.md)) as an auxiliary velocity measurement.
This is especially useful during the final descent, where the camera is typically the most accurate source of horizontal velocity available to the vehicle.

VTE exposes the auxiliary velocity to EKF2 when:

- the target is static,
- the published relative position and relative velocity are both valid, and
- a vision measurement has been fused recently (within [VTE_M_REC_TOUT](../advanced_config/parameter_reference.md#VTE_M_REC_TOUT)).

The recent-vision requirement is intentional.
GNSS and mission position can keep the target estimate valid, but they are not true relative measurements and must not drive EKF2 relative-velocity aiding by themselves.

EKF2 then decides whether to actually fuse the signal.
This decision is gated by [EKF2_AVEL_EN](../advanced_config/parameter_reference.md#EKF2_AVEL_EN): with that parameter disabled, EKF2 ignores the input even when VTE flags it as valid.

:::

## MAVLink Messages

The estimator receives target information through two MAVLink messages emitted by an onboard companion (vision pipeline and/or external GNSS receiver):

- [TARGET_RELATIVE](https://mavlink.io/en/messages/development.html#TARGET_RELATIVE) carries each vision-based observation of the target.
- [TARGET_ABSOLUTE](https://mavlink.io/en/messages/development.html#TARGET_ABSOLUTE) carries absolute target state from a GNSS receiver mounted on the target.

::: info
These messages are currently in the MAVLink development dialect.
To make them available in PX4 builds the `CONFIG_MAVLINK_DIALECT="development"` key must be set in the build configuration.
:::

::: details Click to view MAVLink message integration (TARGET_RELATIVE, TARGET_ABSOLUTE)

**TARGET_RELATIVE (ID 511)**

[TARGET_RELATIVE](https://mavlink.io/en/messages/development.html#TARGET_RELATIVE) extends the [LANDING_TARGET](https://mavlink.io/en/messages/common.html#LANDING_TARGET) message with a full 3D report that includes orientation and measurement uncertainty:

- A coordinate frame selector (`TARGET_OBS_FRAME`) and a sensor quaternion (`q_sensor`) that, when provided, rotates the measurement into vehicle-carried NED.
- Target pose (`x`, `y`, `z`, `q_target`) and variances (`pos_std`, `yaw_std`), collected from onboard vision pipelines.

`mavlink_receiver` validates the frame/type and handles the message differently depending on the Vision Target Estimator status:

- When `VTE_EN=0`, the measurement is rotated (using `q_sensor` or the vehicle attitude for body-frame reports) and published straight to `landing_target_pose` so precision-landing can operate without the estimator.
- When `VTE_EN=1`, the message is split into `fiducial_marker_pos_report` and `fiducial_marker_yaw_report`.
  `VisionTargetEst` consumes these uORB topics to drive the position and orientation filters (using `fiducial_marker_pos_report.q` to rotate `fiducial_marker_pos_report.rel_pos` into NED at `timestamp_sample`).

**TARGET_ABSOLUTE (ID 510)**

[TARGET_ABSOLUTE](https://mavlink.io/en/messages/development.html#TARGET_ABSOLUTE) reports the target's absolute state when it carries its own GNSS (and optionally IMU).
A capability bitmap advertises which fields are valid.
PX4 maps the available content into the `target_gnss` uORB topic:

- Bit 0 (position) triggers publication of latitude/longitude/altitude along with the horizontal and vertical accuracy estimates (`position_std`).
- Bit 1 (velocity) forwards the NED velocity vector (`vel`) and its standard deviations (`vel_std`).
- Additional fields (acceleration, quaternion `q_target`, rates, uncertainties) are not supported and reserved for future fusion logic once flight testing is available.

:::

### Measurement Rates

The estimator runs its prediction step at 50 Hz regardless of the measurement cadence, and each new sample is only fused if it is fresher than the [Measurement Freshness](#measurement-freshness) window.
The minimum data rate is application-dependent: a slow approach with a static target tolerates a few Hz, while fast-dynamics setups (such as the moving-target filter tracking a manoeuvring platform) typically need tens of Hz so the prediction does not drift too far between updates.
To check whether your rate is sufficient, watch how `vte_position` behaves between fused samples.
If the state visibly diverges between measurements and snaps back when a new sample arrives, the prediction is doing more work than the data supports and you should publish faster.

### Companion Computer Responsibilities

- **Timestamp** each sample at capture in a clock that is synchronised with the autopilot (typically through `mavlink_timesync`).
  The Out-of-Sequence Measurements ([OOSM](../advanced_features/vision_target_estimator_advanced.md#oosm-implementation)) history replay tolerates transport latency, but only if timestamps are consistent.
- **Report consistent variances** in `pos_std` and `yaw_std` (for [TARGET_RELATIVE](https://mavlink.io/en/messages/development.html#TARGET_RELATIVE)) and `position_std` / `vel_std` (for `TARGET_ABSOLUTE`).
  Under-reporting variance is the most common cause of overshoots.
  The [Sensor noise floors](#noise) only clamp the lower bound and cannot rescue an over-confident sensor.
- **Set the coordinate frame** for [TARGET_RELATIVE](https://mavlink.io/en/messages/development.html#TARGET_RELATIVE) (`TARGET_OBS_FRAME`) and provide the `q_sensor` rotation when the camera frame differs from vehicle-carried NED.

### Bypassing MAVLink

If you publish target observations from a different onboard pipeline (for example a ROS 2 node running on the companion), you can write the same data directly to the uORB topics that the estimator subscribes to: `fiducial_marker_pos_report` and `fiducial_marker_yaw_report` for vision, and `target_gnss` for an external receiver.
The timing and variance guidance above still applies.

## Gazebo Classic Simulation

Run the SITL world `gazebo-classic_iris_irlock` to simulate precision landing using the VTE fusing vision (ArUco-based) and target GNSS aiding.

::: tip
The ArUco vision observation path implemented in `Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_aruco_plugin.cpp` provides a concrete example of how to obtain a vision-based observation of a target and how to publish the [TARGET_RELATIVE](https://mavlink.io/en/messages/development.html#TARGET_RELATIVE) message.
:::

1. Launch the simulator with:

   ```sh
   make px4_sitl gazebo-classic_iris_irlock
   ```

2. If CMake cannot locate the expected OpenCV version, query the version present on your system (`opencv_version --short` or `pkg-config --modversion opencv4`) and update `Tools/simulation/gazebo-classic/sitl_gazebo-classic/CMakeLists.txt` accordingly, for example:

   ```cmake
   find_package(OpenCV 4.2.0 REQUIRED EXACT)
   ```

   Re-run the build after adjusting the `find_package` line.

::: tip

- **Pad visibility**: In `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/land_pad/land_pad.sdf`, increase the visual box size to `1.5 1.5 0.01` so the pad stays in view longer while the vehicle descends.
  If vision still detects the pad too late in the descent, complement this by planning a lower-altitude mission so the marker enters the camera field of view earlier.
- **Acceptance radius**: If the vehicle hovers above the pad without ever transitioning to descent, raise [PLD_HACC_RAD](../advanced_config/parameter_reference.md#PLD_HACC_RAD).
  Try 2 m first to confirm the rest of the chain works, then tighten it once the filter is well tuned.
- **Mission waypoint bias**: Enable vision and mission position aiding in [VTE_AID_MASK](../advanced_config/parameter_reference.md#VTE_AID_MASK) (set bits 2 and 3, disable bit 0).
  Place the landing waypoint 3 to 4 m away from the pad in QGroundControl to watch the UAV correct towards the pad once it is detected.
  In the logs, observe how the GNSS bias compensates for the distance between the land waypoint and the actual pad.
- **Measurement noise experiments**: The ArUco plugin publishes nominal standard deviations through `set_std_x` and `set_std_y` in `Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_aruco_plugin.cpp`.
  Modify these, and optionally the camera noise block in `.../models/aruco_cam/aruco_cam.sdf`, to see how innovation gates react to noisier vision.

:::

## Monitoring

For detailed log-analysis guidance, see [Log analysis and expected plots](../advanced_features/vision_target_estimator_advanced.md#log-analysis-and-expected-plots) in the deep dive.

On the autopilot shell, the first command to run is

```sh
vision_target_estimator status
```

which reports whether the module is alive, which task is currently active, which filters are running, and which aid sources are enabled.
It is the quickest way to confirm a configuration is taking effect before opening a log.

For deeper inspection, the following uORB topics are the most useful entry points into a log:

- `landing_target_pose.rel_pos_valid` and `.abs_pos_valid` indicate whether recent measurements support relative and absolute positioning.
- `vte_position` exposes every state component (relative position, vehicle velocity, GNSS bias, and optional target motion) together with diagonal covariance entries.
- `vte_orientation` provides yaw, yaw-rate, and their variances.
- `vte_input` records the downsampled NED acceleration and attitude quaternion actually fed to the position prediction step.
- `vte_bias_init_status` shows raw and filtered GNSS/vision bias while the initial averaging phase is active.
- Innovations published on `vte_aid_*` topics include the raw measurement, innovation, innovation variance, chi-squared test ratio, the `fusion_status` enum (per-axis on the 3D variant), and Out-of-Sequence Measurements ([OOSM](../advanced_features/vision_target_estimator_advanced.md#oosm-implementation)) diagnostics (`time_since_meas_ms`, `history_steps`).
  See [aid-source diagnostics](../advanced_features/vision_target_estimator_advanced.md#aid-source-diagnostics) for the full status table.

## Operational Notes

- Accurate timestamp alignment between measurement sources is critical.
  Large skews will cause innovations to fail the NIS gate and be rejected.
- Absolute target pose is only published when `vehicle_local_position` reports a valid local frame.
- Configure the mission land item for precision landing as described in [Precision landing](../advanced_features/precland.md#mission) so the controller actually consumes the VTE output.
  In practice this means setting the QGroundControl land waypoint `Precision landing` drop-down (or `MAV_CMD_NAV_LAND` `param2`) to Opportunistic or Required.
  Without it, the VTE estimates are ignored even when the filter is running.
- For yaw alignment during landing, also enable [PLD_YAW_EN](../advanced_config/parameter_reference.md#PLD_YAW_EN).
  With precision landing enabled but `PLD_YAW_EN` disabled, only the position estimate is tracked.
- For extended parameter tuning see [Balancing process and observation noise](../advanced_features/vision_target_estimator_advanced.md#balancing-process-and-observation-noise), for log-analysis checklists see [Troubleshooting checklist](../advanced_features/vision_target_estimator_advanced.md#troubleshooting-checklist), and for developer workflows see [Development and debugging tips](../advanced_features/vision_target_estimator_advanced.md#development-and-debugging-tips).

# 3D Wind Estimator

<Badge type="warning" text="Experimental" />

`wind_estimator_3d` is a fixed-wing module that estimates the full 3D wind vector (including the vertical/updraft component) from a simple aerodynamic model of the airframe, combined with attitude, acceleration, GNSS-derived local velocity, and airspeed measurements.

This is different from PX4's default wind estimate: the [wind](../msg_docs/Wind.md) uORB topic published by [EKF2](../advanced_config/tuning_the_ecl_ekf.md) (and used throughout PX4, e.g. by the airspeed validation checks, `MAVLink WIND_COV`, and health checks) only estimates the horizontal wind components. `wind_estimator_3d` instead publishes body-relative airflow and the full 3D wind estimate on the separate [airflow](../msg_docs/Airflow.md) topic.

## When to Use It

Most fixed-wing users do not need this module: PX4's standard horizontal wind estimate (from EKF2) and airspeed handling already cover normal flight.

`wind_estimator_3d` is useful when the vertical wind component itself is of interest, for example autonomous updraft-seeking/dynamic-soaring flight, or research into fixed-wing wind estimation. The estimator formulation is described in:

- Harms, M., Lim, J., Rohr, D., Rockenbauer, F., Lawrance, N. and Siegwart, R., 2025. _Robust Optimization-based Autonomous Dynamic Soaring with a Fixed-Wing UAV_. arXiv preprint arXiv:2512.06610.

## Usage

`wind_estimator_3d` is disabled by default and is not included in stock firmware for real vehicles (it is only enabled in the SITL default configuration). To use it on hardware you need a custom build:

1. Enable the module for your target, either by adding `CONFIG_MODULES_WIND_ESTIMATOR_3D=y` to the board's `.px4board` file, or interactively:

   ```sh
   make px4_fmu-v6x_default boardconfig
   ```

   and enabling `wind_estimator_3d` in the module list.

2. Rebuild and flash the firmware (see [Building the Code](../dev_setup/building_px4.md)). Once included, the module starts automatically with the other fixed-wing apps (`rc.fw_apps`) — there is no separate enable parameter.

Because it publishes to the `airflow` topic rather than `wind`, `wind_estimator_3d` does not feed into (or interfere with) any existing consumer of the system-wide wind estimate (EKF2, airspeed validation, failsafes, MAVLink). Nothing needs to be disabled to run it alongside the default 2D wind estimate.

::: info
The `airflow` topic is logged by default, and in Gazebo simulation the equivalent ground-truth airflow is published on `airflow_groundtruth`. This means the estimator's output can be validated against ground truth directly from a SITL flight log, even before tuning it for a specific real airframe.
:::

## Tuning

The aerodynamic model requires the following airframe-specific parameters:

- [FW_W_MASS](../advanced_config/parameter_reference.md#FW_W_MASS): Vehicle mass
- [FW_W_AREA](../advanced_config/parameter_reference.md#FW_W_AREA): Vehicle reference (wing) area
- [FW_W_CY_B](../advanced_config/parameter_reference.md#FW_W_CY_B): Sideslip force coefficient (side slip)
- [FW_W_CL_0](../advanced_config/parameter_reference.md#FW_W_CL_0): Lift coefficient (zero angle of attack)
- [FW_W_CL_A](../advanced_config/parameter_reference.md#FW_W_CL_A): Lift coefficient (angle of attack)

::: warning
PX4 does not currently provide an automated identification/calibration pipeline to derive these coefficients for a given airframe. They must be obtained externally (for example from wind-tunnel testing, CFD, or manual system identification) and entered by hand.
:::

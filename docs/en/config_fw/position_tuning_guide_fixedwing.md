# Fixed-wing Altitude/Position Controller Tuning

This guide offers some help in tuning the high-level fixed-wing controllers needed for flying missions and in altitude/position control mode.
PX4 uses TECS for altitude and airspeed control, and NPFG for horizontal heading/position control.

::: info
An incorrectly set gain during tuning can make altitude or heading control unstable.
A pilot tuning the TECS gains should therefore be able to fly and land the plane in stabilized control mode.
:::

:::tip
All parameters are documented in the [Parameter Reference](../advanced_config/parameter_reference.md#fw-tecs).
The most important parameters are covered in this guide.
:::

## TECS Tuning (Altitude and Airspeed)

TECS (Total Energy Control System) is a guidance algorithm for fixed-wing aircraft that coordinates throttle and pitch angle setpoints to control the aircraft's altitude and airspeed.
For a detailed description of the TECS algorithm and the control diagram, see [Controller Diagrams](../flight_stack/controller_diagrams.md).

A well-tuned attitude controller is required before tuning TECS: [PID Tuning Guide](../config_fw/pid_tuning_guide_fixedwing.md).

Tuning TECS is mainly about setting the airframe limitations correctly.
Those limitations can be specified in terms of parameters that can be determined from a sequence of flight maneuvers, which are described below.
Most of the maneuvers required the plane to be flown by a pilot in [Stabilized flight mode](../flight_modes_fw/stabilized.md).

:::tip
It is highly beneficial to have a person available who can read and take note of telemetry data while the pilot is flying the maneuvers.
To improve accuracy we also recommended that you verify the data obtained during flight with the data recorded in the flight logs.
:::

#### 1st: Trim Conditions

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and find trim values for both throttle and pitch angle for level flight at trim airspeed.
Use throttle to adjust airspeed and pitch to keep level flight.

Set the following parameters:

- [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) - set to the desired trim airspeed flown during the maneuver.
- [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM) - set to the throttle required to fly at trim airspeed.
- [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) - set to the pitch angle required to maintain level flight.

#### 2nd: Airspeed & Throttle Limits

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and increase throttle while maintaining level flight using pitch control - until the vehicle reaches
the maximum allowed airspeed.

Set the following parameters:

- [FW_THR_MAX](../advanced_config/parameter_reference.md#FW_THR_MAX) - set to the throttle you applied to reach maximum airspeed during level flight.
- [FW_THR_MIN](../advanced_config/parameter_reference.md#FW_THR_MIN) - set to the minimum throttle the plane should fly at.
- [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX) - set to the maximum airspeed you achieved during level flight at `FW_THR_MAX`.

#### 3rd: Pitch & Climb Rate Limits

:::warning
Do not use [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX), [FW_T_SINK_MAX](../advanced_config/parameter_reference.md#FW_T_SINK_MAX) or [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) to specify the desired climb or sink performance you would like to get from the vehicle!
The parameters define the operating limitations and they should be set during the tuning phase, as described below.
:::

Fly in stabilized mode, apply full throttle (`FW_THR_MAX`) and slowly increase the pitch angle of the vehicle until the airspeed reaches `FW_AIRSPD_TRIM`.

- [FW_P_LIM_MAX](../advanced_config/parameter_reference.md#FW_P_LIM_MAX) - set to the pitch angle required to climb at trim airspeed when applying `FW_THR_MAX`.
- [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX) - set to the climb rate achieved during the climb at `FW_AIRSPD_TRIM`.

Fly in stabilized mode, reduce the throttle to `FW_THR_MIN` and slowly decrease the pitch angle until the vehicle reaches `FW_AIRSPD_MAX`.

- [FW_P_LIM_MIN](../advanced_config/parameter_reference.md#FW_P_LIM_MIN) - set to the pitch angle required to reach `FW_AIRSPD_MAX` at `FW_THR_MIN`.
- [FW_T_SINK_MAX](../advanced_config/parameter_reference.md#FW_T_SINK_MAX) - set to the sink rate achieved during the descent.

Fly in stabilized mode, reduce throttle to `FW_THR_MIN` and adjust the pitch angle such that the plane maintains `FW_AIRSPD_TRIM`.

- [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) - set to the sink rate achieved while maintaining `FW_AIRSPD_TRIM`.

Specify the target climb and sink rate for autonomous missions by adjusting [FW_T_CLMB_R_SP](../advanced_config/parameter_reference.md#FW_T_CLMB_R_SP) and [FW_T_SINK_R_SP](../advanced_config/parameter_reference.md#FW_T_SINK_R_SP).
These specify the height rates at which the vehicle will climb or descend in order to change altitude.
Furthermore, these two values define the height rate limits commanded by the user in [Altitude mode](../flight_modes_fw/altitude.md) and [Position mode](../flight_modes_fw/position.md).

### FW Path Control Tuning (Position)

All path control parameters are described [here](../advanced_config/parameter_reference.md#fw-path-control).

- [NPFG_PERIOD](../advanced_config/parameter_reference.md#NPFG_PERIOD) - This is the previously called L1 distance and defines the tracking point ahead of the aircraft it's following.
  A value of 10-20 meters works for most aircraft.
  Shorten slowly during tuning until response is sharp without oscillation.
  Vehicles with a slow roll dynamic should have this value increased.

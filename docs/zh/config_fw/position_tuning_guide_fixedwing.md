# Fixed-wing Altitude/Position Controller Tuning

本指南为调整飞行任务和高度/位置控制模式下所需的高级固定翼控制器提供一些帮助。
PX4 uses TECS for altitude and airspeed control, and NPFG for horizontal heading/position control.

:::info
An incorrectly set gain during tuning can make altitude or heading control unstable.
因此，调节 TECS 增益的飞行员应该能够以稳定的控制模式飞行和降落飞机。
:::

:::tip
All parameters are documented in the [Parameter Reference](../advanced_config/parameter_reference.md#fw-tecs).
本指南将介绍所有重要参数。
:::

## TECS 调节（高度和空速）

TECS（总能量控制系统）是一种用于固定翼飞机的制导算法，该算法通过协调油门和俯仰角设定值来控制飞机的高度和空速。
For a detailed description of the TECS algorithm and the control diagram, see [Controller Diagrams](../flight_stack/controller_diagrams.md).

A well-tuned attitude controller is required before tuning TECS: [PID Tuning Guide](../config_fw/pid_tuning_guide_fixedwing.md).

调整 TECS 主要是正确地设置机身限制。
这些限制可以通过如下所述的一系列飞行操作确定的参数来指定。
Most of the maneuvers required the plane to be flown by a pilot in [Stabilized flight mode](../flight_modes_fw/stabilized.md).

:::tip
It is highly beneficial to have a person available who can read and take note of telemetry data while the pilot is flying the maneuvers.
为了提高准确性，我们还建议您使用飞行日志中记录的数据来验证飞行期间获得的数据。
:::

#### 1st：平衡条件

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and find trim values for both throttle and pitch angle for level flight at trim airspeed.
使用油门去调节空速和俯仰以保持水平飞行。

设置以下参数：

- [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) - set to the desired trim airspeed flown during the maneuver.
- [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM) - set to the throttle required to fly at trim airspeed.
- [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) - set to the pitch angle required to maintain level flight.

#### 2nd: Airspeed & Throttle Limits

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and increase throttle while maintaining level flight using pitch control - until the vehicle reaches
the maximum allowed airspeed.

设置以下参数：

- [FW_THR_MAX](../advanced_config/parameter_reference.md#FW_THR_MAX) - set to the throttle you applied to reach maximum airspeed during level flight.
- [FW_THR_MIN](../advanced_config/parameter_reference.md#FW_THR_MIN) - set to the minimum throttle the plane should fly at.
- [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX) - set to the maximum airspeed you achieved during level flight at `FW_THR_MAX`.

#### 3rd: Pitch & Climb Rate Limits

:::warning
Do not use [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX), [FW_T_SINK_MAX](../advanced_config/parameter_reference.md#FW_T_SINK_MAX) or [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) to specify the desired climb or sink performance you would like to get from the vehicle!这些参数定义了操作限制，应在调试阶段进行设置，如下所述。
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
这些参数指定了飞行器改变高度时的爬升率和下降率。
Furthermore, these two values define the height rate limits commanded by the user in [Altitude mode](../flight_modes_fw/altitude.md) and [Position mode](../flight_modes_fw/position.md).

### 固定翼轨迹控制调整（位置）

All path control parameters are described [here](../advanced_config/parameter_reference.md#fw-path-control).

- [NPFG_PERIOD](../advanced_config/parameter_reference.md#NPFG_PERIOD) - This is the previously called L1 distance and defines the tracking point ahead of the aircraft it's following.
  大多数飞机适用于10-20米的数值范围。
  调整期间缓慢缩短，直到响应迅速没有振荡。
  飞机动态特性缓慢的该数值应该增加。

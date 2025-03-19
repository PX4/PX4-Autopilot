# 固定翼配平指南

配平校准是指在配平状态下计算/设置舵面偏度（配平状态包含相对空速、空气密度、攻击角、飞机配置等）。
一架具有合适配平设置的飞机在配平状态下飞行将自动保持其姿态，不需要飞行员或增稳计算机进行任何的控制操作。

General aviation, commercial and large unmanned planes trim their control surfaces using [trim tabs](https://en.wikipedia.org/wiki/Trim_tab) while small UAVs simply add an offset to the actuator of the control surface.

The [Basic trimming](#basic-trimming) section explains the purpose of each trim parameter and how to find the correct value.
The [Advanced Trimming](#advanced-trimming) section introduces parameters that can be set to automatically adjust the trims based on the measured airspeed and flap position.

## 基础配平

操作者可通过若干个参数来恰当地配平固定翼飞机。
以下是这些参数及其使用场景的概述：

- [RCx_TRIM](../advanced_config/parameter_reference.md#RC1_TRIM) applies trim to the signal received from the RC transmitter.
  These parameters are set automatically during [RC calibration](../config/radio.md).
- [CA_SV_CSx_TRIM](../advanced_config/parameter_reference.md#CA_SV_CS0_TRIM) applies trim to a control surfaces channel.
  该参数用于在飞行前精确地将舵面与默认角度对齐。
- [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) applies an offset to the pitch setpoint.
  该参数用来设定飞机在巡航速度飞行时所需要的攻角。
- [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) is used by the rate controllers to scale their output depending on the measured airspeed.
  See [Airspeed Scaling](../flight_stack/controller_diagrams.md#airspeed-scaling) for more details.
- [TRIM_ROLL](../advanced_config/parameter_reference.md#TRIM_ROLL), [TRIM_PITCH](../advanced_config/parameter_reference.md#TRIM_PITCH) and [TRIM_YAW](../advanced_config/parameter_reference.md#TRIM_YAW) apply trim to the control signals _before_ mixing.
  For example, if you have two servos for the elevator, `TRIM_PITCH` applies trim to both of them.
  这些参数在您的舵面已对齐，但在手动飞行期间出现飞机俯仰/滚轮/偏航/上下/左右动作(不稳定)时，或在增稳飞行期间控制信号有恒定偏移时使用。

设置上述参数的正确顺序是：

1. Trim the servos by physically adjusting the linkages lengths if possible and fine tune by trimming the PWM channels (use `PWM_MAIN/AUX_TRIMx`) on the bench to properly set the control surfaces to their theoretical position.
2. Fly in stabilized mode at cruise speed and set the pitch setpoint offset (`FW_PSP_OFF`) to desired angle of attack.
  巡航速度飞行下的需用攻角为飞机在平飞状态下保持固定高度时的实际飞行迎角。
  If you are using an airspeed sensor, also set the correct cruise airspeed (`FW_AIRSPD_TRIM`).
3. Look at the actuator controls in the log file (upload it to [Flight Review](https://logs.px4.io) and check the _Actuator Controls_ plot for example) and set the pitch trim (`TRIM_PITCH`).
  将该值设置为平飞时俯仰角度的平均值。

步骤3可以在步骤2之前执行，如果你不想查看日志， 或者如果您在手动模式下感觉舒适。
You can then trim your remote (with the trim switches) and report the values to `TRIM_PITCH` (and remove the trims from your transmitter) or update `TRIM_PITCH` directly during flight via telemetry and QGC.

## 高级配平

在空速增加过程中，会因非对称翼型和使用襟翼而引入俯仰通道低头力矩，因此飞机需要根据测量空速和襟翼位置重新计算配平。
为此，可以使用以下参数定义一个关于空速的双线性曲线函数和关于襟翼状态的俯仰修正增量函数（参见下图）：

- [FW_DTRIM\_\[R/P/Y\]\_\[VMIN/VMAX\]](../advanced_config/parameter_reference.md#FW_DTRIM_R_VMIN) are the roll/pitch/yaw trim value added to `TRIM_ROLL/PITCH/YAW` at min/max airspeed (defined by [FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN) and [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX)).
- [CA_SV_CSx_FLAP](../advanced_config/parameter_reference.md#CA_SV_CS0_FLAP) and [CA_SV_CSx_SPOIL](../advanced_config/parameter_reference.md#CA_SV_CS0_SPOIL) are the trimming values that are applied to these control surfaces if the flaps or the spoilers are fully deployed, respectively.

![Dtrim Curve](../../assets/config/fw/fixedwing_dtrim.png)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/15AbscUF1kRdWMh8ONcCRu6QBwGbqVGfl/view?usp=sharing
Request access from dev team. -->

A perfectly symmetrical airframe would only require pitch trim increments, but since a real airframe is never perfectly symmetrical, roll and yaw trims increments are also sometimes required.

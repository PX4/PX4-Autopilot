# Acro Mode (Fixed-Wing)

<img src="../../assets/site/difficulty_hard.png" title="Hard to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;

_Acro mode_ is the manual mode for performing acrobatic maneuvers e.g. rolls, flips, stalls and acrobatic figures.

The roll, pitch and yaw sticks control the rate of angular rotation around the respective axes and throttle is passed directly to control allocation. 当操纵杆居中时，飞机将停止旋转，但保持其当前朝向（在其侧面，倒置或任何其他方向）并根据当前动量移动。
When sticks are centered the vehicle will stop rotating, but remain in its current orientation (on its side, inverted, or whatever) and moving according to its current momentum.

![FW Manual Acrobatic Flight](../../assets/flight_modes/acrobatic_fw.png)

## 技术描述

用于执行杂技动作的遥控模式，例如滚动、翻转、摊位和杂技图形。

RPY摇杆输入被转换为角速度命令，通过自动驾驶仪稳定。
Throttle is passed directly to control allocation.

## 参数

| 参数                                                                                                                                                                   | 描述                                                                                                                                                                                                                                  |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FW_ACRO_X_MAX"></a>[FW_ACRO_X_MAX](../advanced_config/parameter_reference.md#FW_ACRO_X_MAX)    | 机体轴x轴最大速率（用户在acro模式下施加滚转轴满杆操纵时，控制器试图达到的机体轴x轴速率） 默认：90度 Default: 90 degrees.                                                                                                                         |
| <a id="FW_ACRO_Y_MAX"></a>[FW_ACRO_Y_MAX](../advanced_config/parameter_reference.md#FW_ACRO_Y_MAX)    | Acro body y max rate (the body y rate the controller is trying to achieve if the user applies full pitch stick input in acro mode). Default: 90 degrees.         |
| <a id="FW_ACRO_Z_MAX"></a>[FW_ACRO_Z_MAX](../advanced_config/parameter_reference.md#FW_ACRO_Z_MAX)    | 机体轴z轴最大速率（用户在acro模式下施加偏航轴满杆操纵时，控制器试图达到的机体轴z轴速率） 默认：45度 Default: 45 degrees.                                                                                                                         |
| <a id="FW_ACRO_YAW_EN"></a>[FW_ACRO_YAW_EN](../advanced_config/parameter_reference.md#FW_ACRO_YAW_EN) | Enable yaw rate controller (if disabled the pilot commands directly the yaw actuator). `0`: Disabled (Default), `1`: Enabled. |

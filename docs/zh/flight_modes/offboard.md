# Offboard 模式

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

飞行器根据飞行控制栈外部（如机载计算机）提供的设定值控制位置、速度、加速度、姿态以及推力/力矩。
The setpoints may be provided using MAVLink (or a MAVLink API such as [MAVSDK](https://mavsdk.mavlink.io/)) or by [ROS 2](../ros2/index.md).

PX4 requires that the external controller provides a continuous 2Hz "proof of life" signal, by streaming any of the supported MAVLink setpoint messages or the ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) message.
PX4只有在收到该种信号超过1秒后才有效，如果该种信号停止飞行控制栈将重新获得控制权（脱离Offboard模式）。

::: info

- 此模式需要位置或位/姿信息 - 例如 GPS、光流、视觉惯性里程计、mocap 等。
- RC control is disabled except to change modes (you can also fly without any manual controller at all by setting the parameter [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) to 4: Stick input disabled).
- The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) messages before arming in offboard mode or switching to offboard mode when flying.
- The vehicle will exit offboard mode if MAVLink setpoint messages or `OffboardControlMode` are not received at a rate of > 2Hz.
- 并非所有 MAVLink 支持的坐标系和字段值都被设定值消息和飞行器支持。
  Read the sections below _carefully_ to ensure only supported values are used.

:::

## 描述

Offboard模式通过设置位置、速度、加速、姿态、姿态角速率或力/扭矩设定值来控制飞行器的位置和姿态。

PX4 must receive a stream of MAVLink setpoint messages or the ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) at 2 Hz as proof that the external controller is healthy.
该消息必须持续发送1秒以上PX4才能在Offboard模式下解锁或在飞行中切换至Offboard模式。
If the rate falls below 2Hz while under external control PX4 will switch out of offboard mode after a timeout ([COM_OF_LOSS_T](#COM_OF_LOSS_T)), and attempt to land or perform some other failsafe action.
The action depends on whether or not RC control is available, and is defined in the parameter [COM_OBL_RC_ACT](#COM_OBL_RC_ACT).

当使用 MAVLink 时，设定值消息既传达了外部控制器"正常运行"的信号也传达了设定值本身。
Offboard模式下要保持位置，飞行器必须接收到一个包含当前位置设定值的消息指令。

When using ROS 2 the proof that the external source is alive is provided by a stream of [OffboardControlMode](../msg_docs/OffboardControlMode.md) messages, while the actual setpoint is provided by publishing to one of the setpoint uORB topics, such as [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md).
In order to hold position in this case the vehicle must receive a stream of `OffboardControlMode` but would only need the `TrajectorySetpoint` once.

请注意，Offboard模式只支持比较有限的 MAVLink 指令和消息。
其他操作如起飞、降落、返航，最好使用适当的模式来处理。
像上传、下载任务这样的操作可以在任何模式下执行。

## ROS 2 消息

以下 ROS 2 消息及其特定字段和字段值在特定的框架下是允许的。
In addition to providing heartbeat functionality, `OffboardControlMode` has two other main purposes:

1. Controls the level of the [PX4 control architecture](../flight_stack/controller_diagrams.md) at which offboard setpoints must be injected, and disables the bypassed controllers.
2. 确定需要哪种有效估计(位置或速度)，以及应该使用哪种设定值消息。

The `OffboardControlMode` message is defined as shown.

```sh
# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool position
bool velocity
bool acceleration
bool attitude
bool body_rate
bool thrust_and_torque
bool direct_actuator
```

The fields are ordered in terms of priority such that `position` takes precedence over `velocity` and later fields, `velocity` takes precedence over `acceleration`, and so on.
第一个非零字段(从上到下) 定义了Offboard模式所需的有效估计以及可用的设定值消息。
For example, if the `acceleration` field is the first non-zero value, then PX4 requires a valid `velocity estimate`, and the setpoint must be specified using the `TrajectorySetpoint` message.

| 期望控制对象                         | 位置                          | 速度                          | 加速度                         | 姿态                          | 姿态角速率                       | 执行器字段                       | 直接给电机                       | 所需状态估计 | 所需消息                                                                                                                            |
| ------------------------------ | --------------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------- |
| 位置 (NED)    | &check; | -                           | -                           | -                           | -                           | -                           | -                           | 位置     | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| 速度 (NED)    | &cross; | &check; | -                           | -                           | -                           | -                           | -                           | 速度     | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| 加速度（NED）                       | &cross; | &cross; | &check; | -                           | -                           | -                           | -                           | 速度     | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| 姿态(FRD)     | &cross; | &cross; | &cross; | &check; | -                           | -                           | -                           | 无      | [VehicleAttitudeSetpoint](../msg_docs/VehicleAttitudeSetpoint.md)                                                               |
| 体轴角速率 (FRD) | &cross; | &cross; | &cross; | &cross; | &check; | -                           | -                           | 无      | [VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md)                                                                     |
| 推力和力矩(FRD)  | &cross; | &cross; | &cross; | &cross; | &cross; | &check; | -                           | 无      | [VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md) and [VehicleTorqueSetpoint](../msg_docs/VehicleTorqueSetpoint.md) |
| 直接给电机和舵机                       | &cross; | &cross; | &cross; | &cross; | &cross; | &cross; | &check; | 无      | [ActuatorMotors](../msg_docs/ActuatorMotors.md) and [ActuatorServos](../msg_docs/ActuatorServos.md)                             |

where ✓ means that the bit is set, ✘ means that the bit is not set and `-` means that the bit is value is irrelevant.

:::info
Before using offboard mode with ROS 2, please spend a few minutes understanding the different [frame conventions](../ros2/user_guide.md#ros-2-px4-frame-conventions) that PX4 and ROS 2 use.
:::

### 旋翼机

- [px4_msgs::msg::TrajectorySetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TrajectorySetpoint.msg)

  - 支持以下输入组合：

    - Position setpoint (`position` different from `NaN`). Non-`NaN` values of velocity and acceleration are used as feedforward terms for the inner loop controllers.
    - Velocity setpoint (`velocity` different from `NaN` and `position` set to `NaN`). Non-`NaN` values acceleration are used as feedforward terms for the inner loop controllers.
    - Acceleration setpoint (`acceleration` different from `NaN` and `position` and `velocity` set to `NaN`)

  - 所有值都是基于NED(北, 东, 地)坐标系，位置、速度和加速的单位分别为\[m\], \[m/s\] 和\[m/s^2\] 。

- [px4_msgs::msg::VehicleAttitudeSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAttitudeSetpoint.msg)

  - 支持以下输入组合：

    - quaternion `q_d` + thrust setpoint `thrust_body`.
      Non-`NaN` values of `yaw_sp_move_rate` are used as feedforward terms expressed in Earth frame and in \[rad/s\].

  - 姿态四元数表示无人机机体坐标系FRD(前、右、下) 与NED坐标系之间的旋转。 这个推力是在无人机体轴FRD坐标系下，并归一化为 \[-1, 1\] 。

- [px4_msgs::msg::VehicleRatesSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleRatesSetpoint.msg)

  - 支持以下输入组合：

    - `roll`, `pitch`, `yaw` and `thrust_body`.

  - 所有值都表示在无人机体轴FRD坐标系下。 角速率(roll, pitch, yaw) 单位为\[rad/s\] ，thrust_body归一化为 \[-1, 1\]。

### Generic Vehicle

下面的offboard控制模式会绕过所有PX4内部的控制环，应当非常谨慎地使用。

- [px4_msgs::msg::VehicleThrustSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleThrustSetpoint.msg) + [px4_msgs::msg::VehicleTorqueSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleTorqueSetpoint.msg)

  - 支持以下输入组合：
    - `xyz` for thrust and `xyz` for torque.
  - 所有值都在无人机体轴 FRD 坐标系中表示，并且归一化为\[-1, 1\]。

- [px4_msgs::msg::ActuatorMotors](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorMotors.msg) + [px4_msgs::msg::ActuatorServos](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorServos.msg)
  - 直接控制电机输出和/或伺服系统（舵机）输出。
  - Currently works at lower level than then `control_allocator` module. Do not publish these messages when not in offboard mode.
  - 所有值归一化为\[-1, 1\]。 For outputs that do not support negative values, negative entries map to `NaN`.
  - `NaN` maps to disarmed.

## MAVLink 消息

下面的 MAVLink 消息及其特定字段和字段值在特定的帧下是允许的。

### 直升机/垂直起降

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

  - The following input combinations are supported: <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->

    - Position setpoint (only `x`, `y`, `z`)
    - Velocity setpoint (only `vx`, `vy`, `vz`)
    - Acceleration setpoint (only `afx`, `afy`, `afz`)
    - Position setpoint **and** velocity setpoint (the velocity setpoint is used as feedforward; it is added to the output of the position controller and the result is used as the input to the velocity controller).
    - Position setpoint **and** velocity setpoint **and** acceleration (the velocity and the acceleration setpoints are used as feedforwards; the velocity setpoint is added to the output of the position controller and the result is used as the input to the velocity controller; the acceleration setpoint is added to the output of the velocity controller and the result used to compute the thrust vector).

  - PX4 supports the following `coordinate_frame` values (only): [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) and [MAV_FRAME_BODY_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_NED).

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

  - The following input combinations are supported: <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->

    - Position setpoint (only `lat_int`, `lon_int`, `alt`)

    - Velocity setpoint (only `vx`, `vy`, `vz`)

    - _Thrust_ setpoint (only `afx`, `afy`, `afz`)

      ::: info
      Acceleration setpoint values are mapped to create a normalized thrust setpoint (i.e. acceleration setpoints are not "properly" supported).

:::

    - Position setpoint **and** velocity setpoint (the velocity setpoint is used as feedforward; it is added to the output of the position controller and the result is used as the input to the velocity controller).

  - PX4 supports the following `coordinate_frame` values (only): [MAV_FRAME_GLOBAL](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL).

- [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)
  - 支持以下输入组合：
    - Attitude/orientation (`SET_ATTITUDE_TARGET.q`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).
    - Body rate (`SET_ATTITUDE_TARGET` `.body_roll_rate` ,`.body_pitch_rate`, `.body_yaw_rate`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).

### Fixed-wing

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

  - The following input combinations are supported (via `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->

    - Position setpoint (`x`, `y`, `z` only; velocity and acceleration setpoints are ignored).

      - Specify the _type_ of the setpoint in `type_mask` (if these bits are not set the vehicle will fly in a flower-like pattern):
        ::: info
        Some of the _setpoint type_ values below are not part of the MAVLink standard for the `type_mask` field.

:::

        值为：

        - 292：滑动设定值。
          这会将 TECS 配置为空速优先于高度，以便在没有推力时使无人机滑行（即控制俯仰以调节空速）。
          It is equivalent to setting `type_mask` as `POSITION_TARGET_TYPEMASK_Z_IGNORE`, `POSITION_TARGET_TYPEMASK_VZ_IGNORE`, `POSITION_TARGET_TYPEMASK_AZ_IGNORE`.
        - 4096：起飞设定值。
        - 8192：降落设定值。
        - 12288：悬停设定值（以设定值为中心绕圈飞行）。
        - 16384：空闲设定值（油门为0， 横滚 / 俯仰为0）。

  - PX4 supports the coordinate frames (`coordinate_frame` field): [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) and [MAV_FRAME_BODY_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_NED).

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

  - The following input combinations are supported (via `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->

    - Position setpoint (only `lat_int`, `lon_int`, `alt`)

      - Specify the _type_ of the setpoint in `type_mask` (if these bits are not set the vehicle will fly in a flower-like pattern):

        ::: info
        The _setpoint type_ values below are not part of the MAVLink standard for the `type_mask` field.

:::

        值为：

        - 4096：起飞设定值。
        - 8192：降落设定值。
        - 12288：悬停设定值（以设定值为中心绕圈飞行）。
        - 16384：空闲设定值（油门为0， 横滚 / 俯仰为0）。

  - PX4 supports the following `coordinate_frame` values (only): [MAV_FRAME_GLOBAL](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL).

- [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)
  - 支持以下输入组合：
    - Attitude/orientation (`SET_ATTITUDE_TARGET.q`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).
    - Body rate (`SET_ATTITUDE_TARGET` `.body_roll_rate` ,`.body_pitch_rate`, `.body_yaw_rate`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).

### 无人车

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

  - The following input combinations are supported (in `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->

    - Position setpoint (only `x`, `y`, `z`)

      - Specify the _type_ of the setpoint in `type_mask`:

        ::: info
        The _setpoint type_ values below are not part of the MAVLink standard for the `type_mask` field.
        ::

        值为：

        - 12288：悬停设定值（无人机足够接近设定值时会停止）。

    - Velocity setpoint (only `vx`, `vy`, `vz`)

  - PX4 supports the coordinate frames (`coordinate_frame` field): [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) and [MAV_FRAME_BODY_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_NED).

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

  - The following input combinations are supported (in `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->
    - Position setpoint (only `lat_int`, `lon_int`, `alt`)

  - Specify the _type_ of the setpoint in `type_mask` (not part of the MAVLink standard).
    值为：

    - 下面的比特位没有置位，是正常表现。
    - 12288：悬停设定值（无人机足够接近设定值时会停止）。

  - PX4 supports the coordinate frames (`coordinate_frame` field): [MAV_FRAME_GLOBAL](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL).

- [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)
  - 支持以下输入组合：
    - Attitude/orientation (`SET_ATTITUDE_TARGET.q`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).
      ::: info
      Only the yaw setting is actually used/extracted.

:::

## Offboard参数

_Offboard mode_ is affected by the following parameters:

| 参数                                                                                                                                                                      | 描述                                                                                                                                                                                                                                                                    |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_OF_LOSS_T"></a>[COM_OF_LOSS_T](../advanced_config/parameter_reference.md#COM_OF_LOSS_T)       | Time-out (in seconds) to wait when offboard connection is lost before triggering offboard lost failsafe (`COM_OBL_RC_ACT`)                                                                                                      |
| <a id="COM_OBL_RC_ACT"></a>[COM_OBL_RC_ACT](../advanced_config/parameter_reference.md#COM_OBL_RC_ACT)    | Flight mode to switch to if offboard control is lost (Values are - `0`: _Position_, `1`: _Altitude_, `2`: _Manual_, `3`: \*Return, `4`: \*Land\*). |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                      | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md). 默认情况下未启用此功能。                                                                        |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | The amount of stick movement that causes a transition to [Position mode](../flight_modes_mc/position.md) (if [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) is enabled).                            |
| <a id="COM_RCL_EXCEPT"></a>[COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT)                         | 该参数指定一种模式，在该模式下将忽略RC丢失及不会触发RC丢失的失效保护动作。 Set bit `2` to ignore RC loss in Offboard mode.                                                                                                                                                               |

## 开发者资源

Typically developers do not directly work at the MAVLink layer, but instead use a robotics API like [MAVSDK](https://mavsdk.mavlink.io/) or [ROS](http://www.ros.org/) (these provide a developer friendly API, and take care of managing and maintaining connections, sending messages and monitoring responses - the minutiae of working with _Offboard mode_ and MAVLink).

以下资源可能对开发者有用：

- [Offboard Control from Linux](../ros/offboard_control.md)
- [ROS/MAVROS Offboard Example (C++)](../ros/mavros_offboard_cpp.md)
- [ROS/MAVROS Offboard Example (Python)](../ros/mavros_offboard_python.md)

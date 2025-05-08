# 오프보드(Offboard) 모드

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The vehicle obeys position, velocity, acceleration, attitude, attitude rates or thrust/torque setpoints provided by some source that is external to the flight stack, such as a companion computer.
The setpoints may be provided using MAVLink (or a MAVLink API such as [MAVSDK](https://mavsdk.mavlink.io/)) or by [ROS 2](../ros2/index.md).

PX4 requires that the external controller provides a continuous 2Hz "proof of life" signal, by streaming any of the supported MAVLink setpoint messages or the ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) message.
PX4 enables offboard control only after receiving the signal for more than a second, and will regain control if the signal stops.

::: info

- 이 모드에는 위치 또는 자세/태도 정보(GPS, 광학 흐름, 시각-관성 주행 거리 측정, 모캡 등)가 필요합니다.
- RC control is disabled except to change modes (you can also fly without any manual controller at all by setting the parameter [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) to 4: Stick input disabled).
- The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) messages before arming in offboard mode or switching to offboard mode when flying.
- The vehicle will exit offboard mode if MAVLink setpoint messages or `OffboardControlMode` are not received at a rate of > 2Hz.
- Not all coordinate frames and field values allowed by MAVLink are supported for all setpoint messages and vehicles.
  Read the sections below _carefully_ to ensure only supported values are used.

:::

## 설명

Offboard mode is used for controlling vehicle movement and attitude, by setting position, velocity, acceleration, attitude, attitude rates or thrust/torque setpoints.

PX4 must receive a stream of MAVLink setpoint messages or the ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) at 2 Hz as proof that the external controller is healthy.
The stream must be sent for at least a second before PX4 will arm in offboard mode, or switch to offboard mode when flying.
If the rate falls below 2Hz while under external control PX4 will switch out of offboard mode after a timeout ([COM_OF_LOSS_T](#COM_OF_LOSS_T)), and attempt to land or perform some other failsafe action.
The action depends on whether or not RC control is available, and is defined in the parameter [COM_OBL_RC_ACT](#COM_OBL_RC_ACT).

When using MAVLink the setpoint messages convey both the signal to indicate that the external source is "alive", and the setpoint value itself.
In order to hold position in this case the vehicle must receive a stream of setpoints for the current position.

When using ROS 2 the proof that the external source is alive is provided by a stream of [OffboardControlMode](../msg_docs/OffboardControlMode.md) messages, while the actual setpoint is provided by publishing to one of the setpoint uORB topics, such as [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md).
In order to hold position in this case the vehicle must receive a stream of `OffboardControlMode` but would only need the `TrajectorySetpoint` once.

Note that offboard mode only supports a very limited set of MAVLink commands and messages.
Operations, like taking off, landing, return to launch, may be best handled using the appropriate modes.
Operations like uploading, downloading missions can be performed in any mode.

## ROS 2 Messages

The following ROS 2 messages and their particular fields and field values are allowed for the specified frames.
In addition to providing heartbeat functionality, `OffboardControlMode` has two other main purposes:

1. Controls the level of the [PX4 control architecture](../flight_stack/controller_diagrams.md) at which offboard setpoints must be injected, and disables the bypassed controllers.
2. Determines which valid estimates (position or velocity) are required, and also which setpoint messages should be used.

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
The first field that has a non-zero value (from top to bottom) defines what valid estimate is required in order to use offboard mode, and the setpoint message(s) that can be used.
For example, if the `acceleration` field is the first non-zero value, then PX4 requires a valid `velocity estimate`, and the setpoint must be specified using the `TrajectorySetpoint` message.

| desired control quantity                                | position field | velocity field | acceleration field | attitude field | body_rate field | thrust_and_torque field | direct_actuator field | required estimate | required message                                                                                                                |
| ------------------------------------------------------- | -------------- | -------------- | ------------------ | -------------- | ------------------------------------ | ----------------------------------------------------------------- | ------------------------------------------ | ----------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| position (NED)                       | ✓              | -              | -                  | -              | -                                    | -                                                                 | -                                          | position          | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| velocity (NED)                       | ✗              | ✓              | -                  | -              | -                                    | -                                                                 | -                                          | velocity          | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| acceleration (NED)                   | ✗              | ✗              | ✓                  | -              | -                                    | -                                                                 | -                                          | velocity          | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| attitude (FRD)                       | ✗              | ✗              | ✗                  | ✓              | -                                    | -                                                                 | -                                          | none              | [VehicleAttitudeSetpoint](../msg_docs/VehicleAttitudeSetpoint.md)                                                               |
| body_rate (FRD) | ✗              | ✗              | ✗                  | ✗              | ✓                                    | -                                                                 | -                                          | none              | [VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md)                                                                     |
| thrust and torque (FRD)              | ✗              | ✗              | ✗                  | ✗              | ✗                                    | ✓                                                                 | -                                          | none              | [VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md) and [VehicleTorqueSetpoint](../msg_docs/VehicleTorqueSetpoint.md) |
| direct motors and servos                                | ✗              | ✗              | ✗                  | ✗              | ✗                                    | ✗                                                                 | ✓                                          | none              | [ActuatorMotors](../msg_docs/ActuatorMotors.md) and [ActuatorServos](../msg_docs/ActuatorServos.md)                             |

where ✓ means that the bit is set, ✘ means that the bit is not set and `-` means that the bit is value is irrelevant.

:::info
Before using offboard mode with ROS 2, please spend a few minutes understanding the different [frame conventions](../ros2/user_guide.md#ros-2-px4-frame-conventions) that PX4 and ROS 2 use.
:::

### 콥터

- [px4_msgs::msg::TrajectorySetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TrajectorySetpoint.msg)

  - 다음 입력 조합이 지원됩니다.

    - Position setpoint (`position` different from `NaN`). Non-`NaN` values of velocity and acceleration are used as feedforward terms for the inner loop controllers.
    - Velocity setpoint (`velocity` different from `NaN` and `position` set to `NaN`). Non-`NaN` values acceleration are used as feedforward terms for the inner loop controllers.
    - Acceleration setpoint (`acceleration` different from `NaN` and `position` and `velocity` set to `NaN`)

  - All values are interpreted in NED (Nord, East, Down) coordinate system and the units are \[m\], \[m/s\] and \[m/s^2\] for position, velocity and acceleration, respectively.

- [px4_msgs::msg::VehicleAttitudeSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAttitudeSetpoint.msg)

  - The following input combination is supported:

    - quaternion `q_d` + thrust setpoint `thrust_body`.
      Non-`NaN` values of `yaw_sp_move_rate` are used as feedforward terms expressed in Earth frame and in \[rad/s\].

  - The quaternion represents the rotation between the drone body FRD (front, right, down) frame and the NED frame. The thrust is in the drone body FRD frame and expressed in normalized \[-1, 1\] values.

- [px4_msgs::msg::VehicleRatesSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleRatesSetpoint.msg)

  - The following input combination is supported:

    - `roll`, `pitch`, `yaw` and `thrust_body`.

  - All the values are in the drone body FRD frame. The rates are in \[rad/s\] while thrust_body is normalized in \[-1, 1\].

### Generic Vehicle

The following offboard control modes bypass all internal PX4 control loops and should be used with great care.

- [px4_msgs::msg::VehicleThrustSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleThrustSetpoint.msg) + [px4_msgs::msg::VehicleTorqueSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleTorqueSetpoint.msg)

  - The following input combination is supported:
    - `xyz` for thrust and `xyz` for torque.
  - All the values are in the drone body FRD frame and normalized in \[-1, 1\].

- [px4_msgs::msg::ActuatorMotors](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorMotors.msg) + [px4_msgs::msg::ActuatorServos](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorServos.msg)
  - You directly control the motor outputs and/or servo outputs.
  - Currently works at lower level than then `control_allocator` module. Do not publish these messages when not in offboard mode.
  - All the values normalized in \[-1, 1\]. For outputs that do not support negative values, negative entries map to `NaN`.
  - `NaN` maps to disarmed.

## MAVLink Messages

The following MAVLink messages and their particular fields and field values are allowed for the specified vehicle frames.

### 멀티콥터/VTOL

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
  - 다음 입력 조합이 지원됩니다.
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

        값들은 다음과 같습니다:

        - 292 : 글라이딩 설정점.
          이는 추력이 없을 때 기체가 미끄러지도록하기 위해 고도보다 대기 속도를 우선하도록 TECS를 구성합니다 (즉, 속도를 조절하기 위해 피치가 제어 됨).
          It is equivalent to setting `type_mask` as `POSITION_TARGET_TYPEMASK_Z_IGNORE`, `POSITION_TARGET_TYPEMASK_VZ_IGNORE`, `POSITION_TARGET_TYPEMASK_AZ_IGNORE`.
        - 4096 : 이륙 설정점.
        - 8192: 착륙 설정점.
        - 12288 : Loiter 설정점 (설정점을 중심으로 선회 비행합니다).
        - 16384 : 유휴 설정점 (제로 스로틀, 제로 롤/피치).

  - PX4 supports the coordinate frames (`coordinate_frame` field): [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) and [MAV_FRAME_BODY_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_NED).

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

  - The following input combinations are supported (via `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->

    - Position setpoint (only `lat_int`, `lon_int`, `alt`)

      - Specify the _type_ of the setpoint in `type_mask` (if these bits are not set the vehicle will fly in a flower-like pattern):

        ::: info
        The _setpoint type_ values below are not part of the MAVLink standard for the `type_mask` field.

:::

        값들은 다음과 같습니다:

        - 4096 : 이륙 설정점.
        - 8192: 착륙 설정점.
        - 12288 : Loiter 설정점 (설정점을 중심으로 선회 비행합니다).
        - 16384 : 유휴 설정점 (제로 스로틀, 제로 롤/피치).

  - PX4 supports the following `coordinate_frame` values (only): [MAV_FRAME_GLOBAL](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL).

- [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)
  - 다음 입력 조합이 지원됩니다.
    - Attitude/orientation (`SET_ATTITUDE_TARGET.q`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).
    - Body rate (`SET_ATTITUDE_TARGET` `.body_roll_rate` ,`.body_pitch_rate`, `.body_yaw_rate`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).

### 탐사선

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

  - The following input combinations are supported (in `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->

    - Position setpoint (only `x`, `y`, `z`)

      - Specify the _type_ of the setpoint in `type_mask`:

        ::: info
        The _setpoint type_ values below are not part of the MAVLink standard for the `type_mask` field.
        ::

        값들은 다음과 같습니다:

        - -12288 : Loiter 설정점 (설정점에 매우 가까워지면 기체는 멈춤).

    - Velocity setpoint (only `vx`, `vy`, `vz`)

  - PX4 supports the coordinate frames (`coordinate_frame` field): [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) and [MAV_FRAME_BODY_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_NED).

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

  - The following input combinations are supported (in `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->
    - Position setpoint (only `lat_int`, `lon_int`, `alt`)

  - Specify the _type_ of the setpoint in `type_mask` (not part of the MAVLink standard).
    값들은 다음과 같습니다:

    - 다음 비트가 설정되지 않으면 정상적인 동작입니다.
    - -12288 : Loiter 설정점 (설정점에 매우 가까워지면 기체는 멈춤).

  - PX4 supports the coordinate frames (`coordinate_frame` field): [MAV_FRAME_GLOBAL](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL).

- [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)
  - 다음 입력 조합이 지원됩니다.
    - Attitude/orientation (`SET_ATTITUDE_TARGET.q`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).
      ::: info
      Only the yaw setting is actually used/extracted.

:::

## 오프보드 매개변수

_Offboard mode_ is affected by the following parameters:

| 매개변수                                                                                                                                                                    | 설명                                                                                                                                                                                                                                                                    |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_OF_LOSS_T"></a>[COM_OF_LOSS_T](../advanced_config/parameter_reference.md#COM_OF_LOSS_T)       | Time-out (in seconds) to wait when offboard connection is lost before triggering offboard lost failsafe (`COM_OBL_RC_ACT`)                                                                                                      |
| <a id="COM_OBL_RC_ACT"></a>[COM_OBL_RC_ACT](../advanced_config/parameter_reference.md#COM_OBL_RC_ACT)    | Flight mode to switch to if offboard control is lost (Values are - `0`: _Position_, `1`: _Altitude_, `2`: _Manual_, `3`: \*Return, `4`: \*Land\*). |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                      | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md). 기본적으로 오프보드 모드에서는 활성화되지 않습니다.                                        |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | The amount of stick movement that causes a transition to [Position mode](../flight_modes_mc/position.md) (if [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) is enabled).                            |
| <a id="COM_RCL_EXCEPT"></a>[COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT)                         | Specify modes in which RC loss is ignored and the failsafe action not triggered. Set bit `2` to ignore RC loss in Offboard mode.                                                                                                      |

## 개발자 리소스

Typically developers do not directly work at the MAVLink layer, but instead use a robotics API like [MAVSDK](https://mavsdk.mavlink.io/) or [ROS](http://www.ros.org/) (these provide a developer friendly API, and take care of managing and maintaining connections, sending messages and monitoring responses - the minutiae of working with _Offboard mode_ and MAVLink).

The following resources may be useful for a developer audience:

- [Offboard Control from Linux](../ros/offboard_control.md)
- [ROS/MAVROS Offboard Example (C++)](../ros/mavros_offboard_cpp.md)
- [ROS/MAVROS Offboard Example (Python)](../ros/mavros_offboard_python.md)

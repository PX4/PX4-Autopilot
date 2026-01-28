# Offboard Mode (Generic/All Frames)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

Апарат зберігає данні про положення, швидкість, прискорення, орієнтацію, значення сили тяги, відповідно заданим значенням, наданим деяким джерелом, зовнішнім по відношенню до польотного контролера, наприклад комп’ютером.
The setpoints may be provided using MAVLink (or a MAVLink API such as [MAVSDK](https://mavsdk.mavlink.io/)) or by [ROS 2](../ros2/index.md).

PX4 requires that the external controller provides a continuous 2Hz "proof of life" signal, by streaming any of the supported MAVLink setpoint messages or the ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) message.
PX4 вмикає функції в оф-борді лише після отримання сигналу протягом більш ніж секунди, і відновлює керування якщо зупиняється сигнал.

::: info

- Для цього режиму потрібна інформація про позицію або орієнтацію за допомогою вказівника, наприклад, від GPS, оптичного потоку, візуально-інерційної одометрії, MoCap та ін.
- RC control is disabled except to change modes (you can also fly without any manual controller at all by setting the parameter [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) to 4: Stick input disabled).
- The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) messages before arming in offboard mode or switching to offboard mode when flying.
- The vehicle will exit offboard mode if MAVLink setpoint messages or `OffboardControlMode` are not received at a rate of > 2Hz.
- Не всі значення координат та параметрів, дозволені MAVLink підтримуються для усіх повідомлень та транспортних засобів.
  Read the sections below _carefully_ to ensure only supported values are used.

:::

## Опис

Режим офборду використовується для керування транспортним засобом, встановленням положення, швидкості, прискорення, відносним положенням або індексом тяги/заданими значеннями крутного моменту.

PX4 must receive a stream of MAVLink setpoint messages or the ROS 2 [OffboardControlMode](../msg_docs/OffboardControlMode.md) at 2 Hz as proof that the external controller is healthy.
Потік повинен бути відправлений як мінімум за секунду, перш ніж PX4 буде задіяно в режимі офборду, або переключено на режим офборду при польоті.
If the rate falls below 2Hz while under external control PX4 will switch out of offboard mode after a timeout ([COM_OF_LOSS_T](#COM_OF_LOSS_T)), and attempt to land or perform some other failsafe action.
The action depends on whether or not RC control is available, and is defined in the parameter [COM_OBL_RC_ACT](#COM_OBL_RC_ACT).

При використанні MAVLink повідомлення передають обидва сигнали, щоб вказати, що зовнішнє джерело є "живим" і значення має цінність.
Для того, щоб утримувати позицію в даному випадку, апарат повинен отримати потік заданих точок для поточного положення.

When using ROS 2 the proof that the external source is alive is provided by a stream of [OffboardControlMode](../msg_docs/OffboardControlMode.md) messages, while the actual setpoint is provided by publishing to one of the setpoint uORB topics, such as [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md).
In order to hold position in this case the vehicle must receive a stream of `OffboardControlMode` but would only need the `TrajectorySetpoint` once.

Зверніть увагу, що офборд режим підтримує дуже обмежений набір команд MAVLink і повідомлень.
Операції, як-от зліт, посадка, повернення на місце запуску, можуть найкраще бути виконаними з використанням відповідних режимів.
Операції такі як завантаження, місії можуть бути виконані в будь-якому режимі.

## Повідомлення ROS 2

Наступні повідомлення ROS 2 та їх конкретні поля та значення полів допускаються для вказаних кадрів.
In addition to providing heartbeat functionality, `OffboardControlMode` has two other main purposes:

1. Controls the level of the [PX4 control architecture](../flight_stack/controller_diagrams.md) at which offboard setpoints must be injected, and disables the bypassed controllers.
2. Визначає, які допустимі оцінки (положення або швидкості) необхідні, а також які повідомлення відповідно до заданих значень мають бути використані.

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

:::warning
The following list shows the `OffboardControlMode` options for copter, fixed-wing, and VTOL.
For rovers see the [rover section](#rover).
:::

The fields are ordered in terms of priority such that `position` takes precedence over `velocity` and later fields, `velocity` takes precedence over `acceleration`, and so on.
Перше поле, яке має ненульове значення (зверху вниз), визначає, яка допустима оцінка необхідна для використання режиму безпілотного керування, а також повідомлення заданих значень, які можуть бути використані.
For example, if the `acceleration` field is the first non-zero value, then PX4 requires a valid `velocity estimate`, and the setpoint must be specified using the `TrajectorySetpoint` message.

| бажана кількість контролю                       | поле положення | поле швидкості | поле прискорення | поле орієнтації | поле кутової швидкості тіла | поле тяги та крутного момент | поле прямого приводу | необхідна оцінка | необхідне повідомлення                                                                                                          |
| ----------------------------------------------- | -------------- | -------------- | ---------------- | --------------- | --------------------------- | ---------------------------- | -------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| положення (NED)              | ✓              | -              | -                | -               | -                           | -                            | -                    | положення        | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| швидкість (NED)              | ✗              | ✓              | -                | -               | -                           | -                            | -                    | швидкість        | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| прискорення (NED)            | ✗              | ✗              | ✓                | -               | -                           | -                            | -                    | швидкість        | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                         |
| орієнтація (FRD)             | ✗              | ✗              | ✗                | ✓               | -                           | -                            | -                    | нічого           | [VehicleAttitudeSetpoint](../msg_docs/VehicleAttitudeSetpoint.md)                                                               |
| кутова швидкість (FRD)       | ✗              | ✗              | ✗                | ✗               | ✓                           | -                            | -                    | нічого           | [VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md)                                                                     |
| тяга та крутний момент (FRD) | ✗              | ✗              | ✗                | ✗               | ✗                           | ✓                            | -                    | нічого           | [VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md) and [VehicleTorqueSetpoint](../msg_docs/VehicleTorqueSetpoint.md) |
| двигуни та серво                                | ✗              | ✗              | ✗                | ✗               | ✗                           | ✗                            | ✓                    | нічого           | [ActuatorMotors](../msg_docs/ActuatorMotors.md) and [ActuatorServos](../msg_docs/ActuatorServos.md)                             |

where ✓ means that the bit is set, ✘ means that the bit is not set and `-` means that the bit is value is irrelevant.

:::info
Before using offboard mode with ROS 2, please spend a few minutes understanding the different [frame conventions](../ros2/user_guide.md#ros-2-px4-frame-conventions) that PX4 and ROS 2 use.
:::

### Коптер

- [px4_msgs::msg::TrajectorySetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/TrajectorySetpoint.msg)
  - Підтримуються наступні вхідні комбінації:
    - Position setpoint (`position` different from `NaN`). Non-`NaN` values of velocity and acceleration are used as feedforward terms for the inner loop controllers.
    - Velocity setpoint (`velocity` different from `NaN` and `position` set to `NaN`). Non-`NaN` values acceleration are used as feedforward terms for the inner loop controllers.
    - Acceleration setpoint (`acceleration` different from `NaN` and `position` and `velocity` set to `NaN`)

  - All values are interpreted in NED (Nord, East, Down) coordinate system and the units are `[m]`, `[m/s]` and `[m/s^2]` for position, velocity and acceleration, respectively.

- [px4_msgs::msg::VehicleAttitudeSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleAttitudeSetpoint.msg)
  - Підтримується наступна комбінація введення:
    - quaternion `q_d` + thrust setpoint `thrust_body`.
      Non-`NaN` values of `yaw_sp_move_rate` are used as feedforward terms expressed in Earth frame and in `[rad/s]`.

  - Кватерніон представляє обертання між корпусом дрона у системі координат FRD (перед, праворуч, вниз) та системою координат NED.
    Тяга у корпусі дрона виражена у системі координат FRD та у нормалізованих значеннях.

- [px4_msgs::msg::VehicleRatesSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleRatesSetpoint.msg)
  - Підтримується наступна комбінація введення:
    - `roll`, `pitch`, `yaw` and `thrust_body`.

  - Всі значення подані в для дрона в системі FRD.
    The rates are in `[rad/s]` while thrust_body is normalized in `[-1, 1]`.

### Ровер

Rover modules must set the control mode using `OffboardControlMode` and use the appropriate messages to configure the corresponding setpoints.
The approach is similar to other vehicle types, but the allowed control mode combinations and setpoints are different:

| Category                                                                               | Використання            | Setpoints                                                                                                                                                                                                                                                                                                                                                                  |
| -------------------------------------------------------------------------------------- | ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| (Recommended) [Rover Setpoints](#rover-setpoints)                   | General rover control   | [RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md), [RoverSpeedSetpoint](../msg_docs/RoverSpeedSetpoint.md), [RoverAttitudeSetpoint](../msg_docs/RoverAttitudeSetpoint.md), [RoverRateSetpoint](../msg_docs/RoverRateSetpoint.md), [RoverThrottleSetpoint](../msg_docs/RoverThrottleSetpoint.md), [RoverSteeringSetpoint](../msg_docs/RoverSteeringSetpoint.md) |
| [Actuator Setpoints](#actuator-setpoints)                                              | Direct actuator control | [ActuatorMotors](../msg_docs/ActuatorMotors.md), [ActuatorServos](../msg_docs/ActuatorServos.md)                                                                                                                                                                                                                                                                           |
| (Deprecated) [Trajectory Setpoint](#deprecated-trajectory-setpoint) | General vehicle control | [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                                                                                                                                                                                                                                                                                                                    |

#### Rover Setpoints

The rover modules use a hierarchical structure to propagate setpoints:

![Rover Control Structure](../../assets/middleware/ros2/px4_ros2_interface_lib/rover_control_structure.svg)

The "highest" setpoint that is provided will be used within the PX4 rover modules to generate the setpoints that are below it (overriding them!).
With this hierarchy there are clear rules for providing a valid control input:

- Provide a position setpoint **or**
- One of the setpoints on the "left" (speed **or** throttle) **and** one of the setpoints on the "right" (attitude, rate **or** steering).
  All combinations of "left" and "right" setpoints are valid.

The following are all valid setpoint combinations and their respective control flags that must be set through [OffboardControlMode](../msg_docs/OffboardControlMode.md) (set all others to _false_).
Additionally, for some combinations we require certain setpoints to be published with `NAN` values so that the setpoints of interest are not overridden by the rover module (due to the hierarchy above).
&check; are the relevant setpoints we publish, and &cross; are the setpoint that need to be published with `NAN` values.

| Setpoint Combination | Control Flag                                                | [RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md) | [RoverSpeedSetpoint](../msg_docs/RoverSpeedSetpoint.md) | [RoverThrottleSetpoint](../msg_docs/RoverThrottleSetpoint.md) | [RoverAttitudeSetpoint](../msg_docs/RoverAttitudeSetpoint.md) | [RoverRateSetpoint](../msg_docs/RoverRateSetpoint.md) | [RoverSteeringSetpoint](../msg_docs/RoverSteeringSetpoint.md) |
| -------------------- | ----------------------------------------------------------- | ------------------------------------------------------------- | ------------------------------------------------------- | ------------------------------------------------------------- | ------------------------------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------- |
| Положення            | положення                                                   | &check;                                   |                                                         |                                                               |                                                               |                                                       |                                                               |
| Speed + Attitude     | швидкість                                                   |                                                               | &check;                             |                                                               | &check;                                   |                                                       |                                                               |
| Speed + Rate         | швидкість                                                   |                                                               | &check;                             |                                                               | &cross;                                   | &check;                           |                                                               |
| Speed + Steering     | швидкість                                                   |                                                               | &check;                             |                                                               | &cross;                                   | &cross;                           | &check;                                   |
| Throttle + Attitude  | attitude                                                    |                                                               |                                                         | &check;                                   | &check;                                   |                                                       |                                                               |
| Throttle + Rate      | body_rate                              |                                                               |                                                         | &check;                                   |                                                               | &check;                           |                                                               |
| Throttle + Steering  | thrust_and_torque |                                                               |                                                         | &check;                                   |                                                               |                                                       | &check;                                   |

:::info
If you intend to use the rover setpoints, we recommend using the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) instead since it simplifies the publishing of these setpoints.
:::

#### Actuator Setpoints

Instead of controlling the vehicle using position, speed, rate and other setpoints, you can directly control the motors and actuators using [ActuatorMotors](../msg_docs/ActuatorMotors.md) and [ActuatorServos](../msg_docs/ActuatorServos.md).
In [OffboardControlMode](../msg_docs/OffboardControlMode.md) set `direct_actuator` to _true_ and all other flags to _false_.

:::info
This bypasses the rover modules including any limits on steering rates or accelerations and the inverse kinematics step.
We recommend using [RoverSteeringSetpoint](../msg_docs/RoverSteeringSetpoint.md) and [RoverThrottleSetpoint](../msg_docs/RoverThrottleSetpoint.md) instead for low level control (see [Rover Setpoints](#rover-setpoints)).
:::

#### (Deprecated) Trajectory Setpoint

:::warning
The [Rover Setpoints](#rover-setpoints) are a replacement for the [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md) and we highly recommend using those instead as they have a well defined behaviour and offer more flexibility.
:::

The rover modules support the _position_, _velocity_ and _yaw_ fields of the [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md).
However, only one of the fields is active at a time and is defined by the flags of [OffboardControlMode](../msg_docs/OffboardControlMode.md):

| Control Mode Flag | Active Trajectory Setpoint Field |
| ----------------- | -------------------------------- |
| положення         | положення                        |
| швидкість         | швидкість                        |
| attitude          | yaw                              |

:::info
Ackermann rovers do not support the yaw setpoint.
:::

### Універсальний апарат

Наступні режими керування з відбором оминуть всі внутрішні контрольні системи PX4 і повинні використовуватися з великою обережністю.

- [px4_msgs::msg::VehicleThrustSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleThrustSetpoint.msg) + [px4_msgs::msg::VehicleTorqueSetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleTorqueSetpoint.msg)
  - Підтримується наступна комбінація введення:
    - `xyz` for thrust and `xyz` for torque.
  - Усі значення виражені у системі координат тіла дрона FRD та нормалізовані у діапазоні \[-1, 1\].

- [px4_msgs::msg::ActuatorMotors](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ActuatorMotors.msg) + [px4_msgs::msg::ActuatorServos](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ActuatorServos.msg)
  - Ви безпосередньо керуєте вихідними сигналами моторів та/або сервоприводів.
  - Currently works at lower level than then `control_allocator` module.
    Do not publish these messages when not in offboard mode.
  - All the values normalized in `[-1, 1]`.
    For outputs that do not support negative values, negative entries map to `NaN`.
  - `NaN` maps to disarmed.

## Повідомлення MAVLink

Наступні повідомлення MAVLink та їх конкретні поля та значення полів дозволені для вказаних кадрів літального апарату.

### Коптер/ВТОЛ

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
  - Підтримуються наступні вхідні комбінації:
    - Attitude/orientation (`SET_ATTITUDE_TARGET.q`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).
    - Body rate (`SET_ATTITUDE_TARGET` `.body_roll_rate` ,`.body_pitch_rate`, `.body_yaw_rate`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).

### Літак з фіксованим крилом

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
  - The following input combinations are supported (via `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->
    - Position setpoint (`x`, `y`, `z` only; velocity and acceleration setpoints are ignored).
      - Specify the _type_ of the setpoint in `type_mask` (if these bits are not set the vehicle will fly in a flower-like pattern):
        ::: info
        Some of the _setpoint type_ values below are not part of the MAVLink standard for the `type_mask` field.

:::

        Значення:

        - 292: планування.Це налаштовує TECS на пріорітезацію швидкості над висотою, щоб змусити безпілотник планувати, коли немає тяги (тобто кут крену контролюється для регулювання швидкості).
          It is equivalent to setting `type_mask` as `POSITION_TARGET_TYPEMASK_Z_IGNORE`, `POSITION_TARGET_TYPEMASK_VZ_IGNORE`, `POSITION_TARGET_TYPEMASK_AZ_IGNORE`.
        - 4096: Точка взльоту.
        - 8192: Точка посадки.
        - 12288: Задання Loiter (політ по колу, центрованому на заданій точці).
        - 16384: Задання бездіяльності (нульовий газ, нульовий крен/тангаж).

  - PX4 supports the coordinate frames (`coordinate_frame` field): [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) and [MAV_FRAME_BODY_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_NED).

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
  - The following input combinations are supported (via `type_mask`): <!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp#L166-L170 -->
    - Position setpoint (only `lat_int`, `lon_int`, `alt`)
      - Specify the _type_ of the setpoint in `type_mask` (if these bits are not set the vehicle will fly in a flower-like pattern):

        ::: info
        The _setpoint type_ values below are not part of the MAVLink standard for the `type_mask` field.

:::

        Значення:

        - 4096: Точка взльоту.
        - 8192: Точка посадки.
        - 12288: Задання Loiter (політ по колу, центрованому на заданій точці).
        - 16384: Задання бездіяльності (нульовий газ, нульовий крен/тангаж).

  - PX4 supports the following `coordinate_frame` values (only): [MAV_FRAME_GLOBAL](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL).

- [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)
  - Підтримуються наступні вхідні комбінації:
    - Attitude/orientation (`SET_ATTITUDE_TARGET.q`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).
    - Body rate (`SET_ATTITUDE_TARGET` `.body_roll_rate` ,`.body_pitch_rate`, `.body_yaw_rate`) with thrust setpoint (`SET_ATTITUDE_TARGET.thrust`).

### Ровер

Rover does not support a MAVLink offboard API (ROS2 is supported).

## Параметри для відключення

_Offboard mode_ is affected by the following parameters:

| Параметр                                                                                                                                                                | Опис                                                                                                                                                                                                                                                                  |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_OF_LOSS_T"></a>[COM_OF_LOSS_T](../advanced_config/parameter_reference.md#COM_OF_LOSS_T)       | Time-out (in seconds) to wait when offboard connection is lost before triggering offboard lost failsafe (`COM_OBL_RC_ACT`)                                                                                                      |
| <a id="COM_OBL_RC_ACT"></a>[COM_OBL_RC_ACT](../advanced_config/parameter_reference.md#COM_OBL_RC_ACT)    | Flight mode to switch to if offboard control is lost (Values are - `0`: _Position_, `1`: _Altitude_, `2`: _Manual_, `3`: \*Return, `4`: \*Land\*). |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                      | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md). За замовчуванням це неможливо ввімкнути в режимі автопілоту.        |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | Кількість рухів стиків, яка викликає перехід у [режим Положення](../flight_modes_mc/position.md) (якщо [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) увімкнено).                                   |
| <a id="COM_RCL_EXCEPT"></a>[COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT)                         | Вкажіть режими, в яких втрата радіокерування ігнорується, а запобіжний алгоритм не виконуватиметься. Set bit `2` to ignore RC loss in Offboard mode.                                                                                  |

## Ресурси Розробника

Typically developers do not directly work at the MAVLink layer, but instead use a robotics API like [MAVSDK](https://mavsdk.mavlink.io/) or [ROS](https://www.ros.org/) (these provide a developer friendly API, and take care of managing and maintaining connections, sending messages and monitoring responses - the minutiae of working with _Offboard mode_ and MAVLink).

Наступні ресурси можуть бути корисними для аудиторії розробників:

- [Offboard Control from Linux](../ros/offboard_control.md)
- [ROS/MAVROS Offboard Example (C++)](../ros/mavros_offboard_cpp.md)
- [ROS/MAVROS Offboard Example (Python)](../ros/mavros_offboard_python.md)

# Інжекція помилки системи

System failure injection allows you to induce different types of sensor and system failures, either programmatically using the [MAVSDK failure plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_failure.html), or "manually" via a PX4 console like the [MAVLink shell](../debug/mavlink_shell.md#mavlink-shell).
This enables easier testing of [safety failsafe](../config/safety.md) behaviour, and more generally, of how PX4 behaves when systems and sensors stop working correctly.

Failure injection is disabled by default, and can be enabled using the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.

:::warning
Failure injection still in development.
На момент написання (PX4 v1.14):

- Це можна використовувати лише в симуляції (підтримка як для впровадження в реальному польоті запланована).
- Потребує підтримки в симуляторі.
  Це підтримується в Gazebo Classic
- Багато типів відмов не широко реалізовані.
  У таких випадках команда повернеться з повідомленням "unsupported".

:::

## Команда системи збою

Failures can be injected using the [failure system command](../modules/modules_command.md#failure) from any PX4 console/shell, specifying both the target and type of the failure.

### Синтаксис

The full syntax of the [failure](../modules/modules_command.md#failure) command is:

```sh
failure <component> <failure_type> [-i <instance_number>]
```

де:

- _component_:
  - Датчики:
    - `gyro`: Gyro.
    - `accel`: Accelerometer.
    - `mag`: Magnetometer
    - `baro`: Barometer
    - `gps`: GPS
    - `optical_flow`: Optical flow.
    - `vio`: Visual inertial odometry.
    - `distance_sensor`: Distance sensor (rangefinder).
    - `airspeed`: Airspeed sensor.
  - Системи:
    - `battery`: Battery.
    - `motor`: Motor.
    - `servo`: Servo.
    - `avoidance`: Avoidance.
    - `rc_signal`: RC Signal.
    - `mavlink_signal`: MAVLink signal (data telemetry).
- _failure_type_:
  - `ok`: Publish as normal (Disable failure injection).
  - `off`: Stop publishing.
  - `stuck`: Report same value every time (_could_ indicate a malfunctioning sensor).
  - `garbage`: Publish random noise. Це схоже на читання неініціалізованої пам'яті.
  - `wrong`: Publish invalid values (that still look reasonable/aren't "garbage").
  - `slow`: Publish at a reduced rate.
  - `delayed`: Publish valid data with a significant delay.
  - `intermittent`: Publish intermittently.
- _instance number_ (optional): Instance number of affected sensor.
  0 (за замовчуванням) вказує на всі сенсори вказаного типу.

### Приклад

Щоб симулювати втрату сигналу RC без вимкнення вашого пульта керування RC:

1. Enable the parameter [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN).
2. Enter the following commands on the MAVLink console or SITL _pxh shell_:

  ```sh
  # Fail RC (turn publishing off)
  failure rc_signal off

  # Restart RC publishing
  failure rc_signal ok
  ```

## MAVSDK відлагоджувальний плагін

The [MAVSDK failure plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_failure.html) can be used to programmatically inject failures.
It is used in [PX4 Integration Testing](../test_and_ci/integration_testing_mavsdk.md) to simulate failure cases (for example, see [PX4-Autopilot/test/mavsdk_tests/autopilot_tester.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester.cpp)).

API плагіна - це пряме відображення команди збою, показаної вище, з деякими додатковими сигналами про помилки, пов'язані з підключенням.

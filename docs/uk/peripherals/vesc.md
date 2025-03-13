# VESC ESCs (DroneCAN)

The [VESC project](https://vesc-project.com/) is a fully open source hardware and software design for advanced FOC motor controllers.
While it can be controlled using traditional PWM input, it also supports being connected over CAN bus using [DroneCAN](../dronecan/index.md).

## Де купити

[Vesc Project > Hardware](https://vesc-project.com/Hardware)

## Налаштування програмного забезпечення

### Підключення

ЕСС підключені до шини CAN за допомогою роз'єму CAN VESC. Note that this is _not_ the Pixhawk standard 4 pin JST GH connector. For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions. Порядок ESC не має значення.

## Налаштування прошивки

The preferred tool for motor enumeration is the [VESC tool](https://vesc-project.com/vesc_tool).
Крім звичайної конфігурації двигуна, яку вам потрібно буде налаштувати в інструменті VESC, вам також потрібно належним чином налаштувати конфігурацію додатка.
Рекомендоване налаштування додатку виглядає наступним чином:

| Параметр                             | Опції                  |
| ------------------------------------ | ---------------------- |
| Додаток для використання             | `No App`               |
| VESC ID                              | `1,2,...`              |
| Режим повідомлення статусу Can       | `CAN_STATUS_1_2_3_4_5` |
| Швидкість передачі даних по шині CAN | `CAN_BAUD_500K`        |
| CAN режим                            | `UAVCAN`               |
| Індекс UAVCAN ESC                    | `0,1,...`              |

VESC ID should have the same motor numbering as in PX4 convention, starting at `1` for top-right motor, `2` for bottom-left motor etc.
However the `UAVCAN ESC Index` starts from `0`, and as such it is always one index lower than the `VESC ID`.
For example, in a quadcopter the bottom left motor will have `VESC ID = 2` and `UAVCAN ESC Index = 1`.

Finally the `CAN Baud Rate` must match the value set in [UAVCAN_BITRATE](../advanced_config/parameter_reference.md#UAVCAN_BITRATE).

## Налаштування польотного контролера

### Увімкнути DroneCAN

Підключіть ESC до шини CAN Pixhawk. Power up the entire vehicle using a battery or power supply (not just the flight controller over USB) and enable the DroneCAN driver by setting the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `3` to enable both dynamic node ID allocation and DroneCAN ESC output.

### Конфігурація PX4

Assign motors to outputs using the [Acutator](../config/actuators.md#actuator-testing) configuration screen.

<!-- removed as there is no info for it in linked doc -->

<!--
## Troubleshooting

See DroneCAN Troubleshooting - (index.md#troubleshooting).
-->

## Подальша інформація

- [VESC Project ESCs](https://vesc-project.com/)
- [Benjamin Vedder's blog](http://vedder.se) (project owner)

# ARK CANnode

[ARK CANnode](https://arkelectron.com/product/ark-cannode/) is an open source generic [DroneCAN](../dronecan/index.md) node that includes a 6 degree of freedom IMU.
Його основна мета - дозволити використання датчиків, що не є CAN (I2C, SPI, UART) на шині CAN.
Також він має виходи PWM для розширення вихідних сигналів транспортного засобу за кількістю та фізичною відстанню.

![ARK CANnode](../../assets/hardware/can_nodes/ark_cannode.jpg)

## Де купити

Замовте цей модуль з:

- [ARK Electronics](https://arkelectron.com/product/ark-cannode/) (US)

## Характеристики обладнання

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_CANNODE)
- Датчики
  - Bosch BMI088 6-Axis IMU або Invensense ICM-42688-P 6-Axis IMU
- STM32F412CGU6 MCU
  - 1MB Flash
- Два роз'єми стандарту CAN для Pixhawk
  - 4-контактний JST-GH
- Роз'єм для налагодження стандарту Pixhawk I2C
  - 4-контактний JST-GH
- Стандартний коннектор UART/I2C для Pixhawk (Основний порт GPS)
  - 6-контактний JST-GH
- Роз'єм стандарту SPI для Pixhawk
  - 7-контактний JST-GH
- Коннектор PWM
  - 10-контактний JST-SH
  - 8 PWM виводів
  - Відповідно до схеми підключення штирьових роз'ємів Pixhawk 4 PWM
- Роз'єм для налагодження стандарту Pixhawk
  - 6-контактний JST-GH
- Малий форм-фактор
  - 3см x 3см x 1.3см
- LED індикатори
- USA Built
- Вимоги до живлення
  - 5V
  - Сила струму залежить від підключених пристроїв

## Налаштування програмного забезпечення

### Підключення

ARK CANnode підключений до шини CAN за допомогою стандартного кабелю JST GH з чотирма контактами Pixhawk.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

## Налаштування прошивки

ARK CANnode runs the [PX4 DroneCAN Firmware](px4_cannode_fw.md).
As such, it supports firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).

ARK CANnode boards ship with recent firmware pre-installed, but if you want to build and flash the latest firmware yourself see [PX4 DroneCAN Firmware > Building the Firmware](px4_cannode_fw.md#building-the-firmware).

- Firmware target: `ark_cannode_default`
- Bootloader target: `ark_cannode_canbootloader`

## Налаштування режиму польоту

### Увімкнути DroneCAN

In order to use the ARK CANnode board, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

Кроки наступні:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Підключіть ARK CANnode CAN до Pixhawk CAN.

Після активації модуль буде виявлено при завантаженні.

DroneCAN configuration in PX4 is explained in more detail in [DroneCAN > Enabling DroneCAN](../dronecan/index.md#enabling-dronecan).

### Увімкнення датчику

Вам потрібно буде увімкнути підписника, відповідного для кожного з сенсорів, які підключені до ARK CANnode.

This is done using the the parameters named like `UAVCAN_SUB_*` in the parameter reference (such as [UAVCAN_SUB_ASPD](../advanced_config/parameter_reference.md#UAVCAN_SUB_ASPD), [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO) etc.).

## Конфігурування CANNode Ark

На ARK CANnode вам може знадобитися налаштувати наступні параметри:

| Параметр                                                                                                             | Опис                                           |
| -------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| <a id="CANNODE_TERM"></a>[CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM) | Вбудована завершення шини CAN. |

## Значення LED індикаторів

Ви побачите як червоні, так і сині світлодіоди на ARK CANnode, коли він прошивається, і сталий синій світлодіод, якщо він працює належним чином.

Якщо ви бачите червоний світлодіод, це означає, що виникла помилка, і вам слід перевірити наступне:

- Переконайтеся, що у польотному контролері встановлено SD-картку.
- Make sure the ARK CANnode has `ark_cannode_canbootloader` installed prior to flashing `ark_cannode_default`.
- Видаліть бінарні файли з кореневих та ufw директорій SD-карти та спробуйте зібрати та знову прошити.

## Дивіться також

- [ARK CANnode Documentation](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-cannode) (ARK Docs)

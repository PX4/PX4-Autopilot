# ARK GPS (DroneCAN)

ARK GPS is an open source [DroneCAN](index.md) [GNSS/GPS](../gps_compass/index.md), magnetometer, IMU, barometer, buzzer, and safety switch module.

![ARK GPS](../../assets/hardware/gps/ark/ark_gps.jpg)

## Де купити

Замовте цей модуль з:

- [ARK Electronics](https://arkelectron.com/product/ark-gps/) (US)

## Характеристики обладнання

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_GPS)
- Датчики
  - Ublox M9N GPS
    - Надзвичайно надійна геоприв'язка на рівні метра за допомогою супутникової навігації
    - Максимальна доступність позиції з одночасним прийомом 4 супутників
    - Просунуте виявлення підробки сигналу та перешкод
    - Відмінне запобігання RF-перешкодам
  - Bosch BMM150 Magnetometer
  - Bosch BMP388 Barometer
  - Invensense ICM-42688-P 6-Axis IMU
- STM32F412CEU6 MCU
- Кнопка безпеки
- Зумер
- Два роз'єми стандарту CAN для Pixhawk (4 контакти JST GH)
- Роз'єм для відлагодження стандарту Pixhawk (6 контактів JST SH)
- Малий форм-фактор
  - 5см x 5см x 1см
- LED індикатори
  - Індикатор безпеки
  - GPS Fix
  - RGB Статус системи
- USA Built
- Вимоги до живлення
  - 5V
  - Середній струм 110мA
  - 117мА Макс.

## Налаштування програмного забезпечення

### Підключення

ARK GPS підключений до шини CAN за допомогою стандартного кабелю Pixhawk 4 pin JST GH.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Встановлення

The recommended mounting orientation is with the connectors on the board pointing towards the **back of vehicle**.

The sensor can be mounted anywhere on the frame, but you will need to specify its position, relative to vehicle centre of gravity, during [PX4 configuration](#px4-configuration).

## Налаштування прошивки

ARK GPS runs the [PX4 DroneCAN Firmware](px4_cannode_fw.md).
As such, it supports firmware update over the CAN bus and [dynamic node allocation](../dronecan/index.md#node-id-allocation).

ARK GPS boards ship with recent firmware pre-installed, but if you want to build and flash the latest firmware yourself see [PX4 DroneCAN Firmware > Building the Firmware](px4_cannode_fw.md#building-the-firmware).

- Firmware target: `ark_can-gps_default`
- Bootloader target: `ark_can-gps_canbootloader`

## Конфігурація PX4

You need to set necessary [DroneCAN](index.md) parameters and define offsets if the sensor is not centred within the vehicle.
Необхідні налаштування наведено нижче.

:::info
The ARK GPS will not boot if there is no SD card in the flight controller when powered on.
:::

### Увімкнути DroneCAN

In order to use the ARK GPS board, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

Кроки наступні:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Підключіть ARK GPS CAN до Pixhawk CAN.

Після активації модуль буде виявлено при завантаженні.
Дані GPS повинні надходити з частотою 10 Гц.

DroneCAN configuration in PX4 is explained in more detail in [DroneCAN > Enabling DroneCAN](../dronecan/index.md#enabling-dronecan).

### Конфігурація позиції датчика

Якщо датчик не знаходиться у центрі пристрою, вам також потрібно буде визначити зміщення датчика:

- Enable GPS yaw fusion by setting bit 3 of [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to true.
- Enable [UAVCAN_SUB_GPS](../advanced_config/parameter_reference.md#UAVCAN_SUB_GPS), [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG), and [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO).
- Set [CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM) to `1` if this is that last node on the CAN bus.
- The parameters [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) can be set to account for the offset of the ARK GPS from the vehicles centre of gravity.

## Значення LED індикаторів

Ви побачите зелені, сині та червоні світлодіоди на ARK GPS під час прошивки, а також мигаючий зелений світлодіод, якщо все працює належним чином.

Якщо ви бачите червоний світлодіод, це означає, що виникла помилка, і вам слід перевірити наступне:

- Переконайтеся, що у польотному контролері встановлено SD-картку.
- Make sure the ARK GPS has `ark_can-gps_canbootloader` installed prior to flashing `ark_can-gps_default`.
- Видаліть бінарні файли з кореневих та ufw директорій SD-карти та спробуйте зібрати та знову прошити.

## Дивіться також

- [ARK GPS](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-gps) (ARK Docs)

# ARK X20 RTK GPS

[ARK X20 RTK GPS](https://docs.arkelectron.com/gps/ark-x20-rtk-gps) is an open source [DroneCAN](index.md) [RTK GPS](../gps_compass/rtk_gps.md), [u-blox ZED-X20P all-band high precision GNSS module](https://www.u-blox.com/en/product/zed-x20p-module), magnetometer, barometer, IMU, buzzer, and safety switch module.

![ARK X20 RTK GPS](../../assets/hardware/gps/ark/ark_x20_rtk_gps.jpg)

## Де купити

Замовте цей модуль з:

- [ARK Electronics](https://arkelectron.com/product/ark-x20-rtk-gps/) (US)

## Характеристики обладнання

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_RTK_GPS)
- Датчики
  - Ublox ZED-X20P
    - All-band all constellation GNSS receiver
    - Best position accuracy and availability in different environments
    - RTK, PPP-RTK and PPP algorithms expanding the limits of performance
    - Highest quality GNSS raw data
    - u-blox end-to-end hardened security
    - 25Hz update rate
  - ST IIS2MDC Magnetometer
  - Bosch BMP390 Barometer
  - Invensense ICM-42688-P 6-Axis IMU
- STM32F412VGH6 MCU
- Кнопка безпеки
- Зумер
- Два роз'єми стандарту CAN для Pixhawk (4 контакти JST GH)
- X20 “UART 2” Connector
  - 4-контактний JST-GH
  - TX, RX, PPS, GND
- I2C Expansion Connector
  - 4-контактний JST-GH
  - 5.0V, SCL, SDA, GND
- Роз'єм для відлагодження стандарту Pixhawk (6 контактів JST SH)
- LED індикатори
  - Індикатор безпеки
  - GPS Fix
  - Статус RTK
  - RGB Статус системи
- USA Built
- Вимоги до живлення
  - 5V
  - 144mA Average
  - 157mA Max

## Налаштування програмного забезпечення

### Підключення

The ARK X20 RTK GPS is connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable. For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Встановлення

The recommended mounting orientation is with the connectors on the board pointing towards the **back of vehicle**.

The sensor can be mounted anywhere on the frame, but you will need to specify its position, relative to vehicle centre of gravity, during [PX4 configuration](#px4-configuration).

## Налаштування прошивки

ARK X20 RTK GPS runs the [PX4 cannode firmware](px4_cannode_fw.md). As such, it supports firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).

ARK X20 RTK GPS boards ship with recent firmware pre-installed, but if you want to build and flash the latest firmware yourself, refer to the [cannode firmware build instructions](px4_cannode_fw.md#building-the-firmware).

Firmware target: `ark_can-rtk-gps_default`
Bootloader target: `ark_can-rtk-gps_canbootloader`

## Налаштування польотного контролера

### Увімкнення DroneCAN

In order to use the ARK X20 RTK GPS, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

Кроки наступні:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect ARK X20 RTK GPS CAN to the Pixhawk CAN.

Після активації модуль буде виявлено при завантаженні.
Дані GPS повинні надходити з частотою 10 Гц.

### Конфігурація PX4

You need to set necessary [DroneCAN](index.md) parameters and define offsets if the sensor is not centred within the vehicle:

- Enable GPS yaw fusion by setting bit 3 of [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to true.
- Enable GPS blending to ensure the heading is always published by setting [SENS_GPS_MASK](../advanced_config/parameter_reference.md#SENS_GPS_MASK) to 7 (all three bits checked).
- Enable [UAVCAN_SUB_GPS](../advanced_config/parameter_reference.md#UAVCAN_SUB_GPS), [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG), and [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO).
- The parameters [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) can be set to account for the offset of the ARK X20 RTK GPS from the vehicles centre of gravity.

### ARK X20 RTK GPS Configuration

You may need to [configure the following parameters](../dronecan/index.md#qgc-cannode-parameter-configuration) on the ARK X20 RTK GPS itself:

| Параметр                                                                                                                                           | Опис                                                                                                                                                                                                                        |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="CANNODE_NODE_ID"></a>[CANNODE_NODE_ID](../advanced_config/parameter_reference.md#CANNODE_NODE_ID) | CAN node ID (0 for dynamic allocation). If set to 0 (default), dynamic node allocation is used. Set to 1-127 to use a static node ID. |
| <a id="CANNODE_TERM"></a>[CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM)                               | Вбудована завершення шини CAN. Set to `1` if this is the last node on the CAN bus.                                                                                                          |

### Setting Up Rover and Fixed Base

Position of the rover is established using RTCM messages from the RTK base module (the base module is connected to QGC, which sends the RTCM information to PX4 via MAVLink).

Параметри PX4 DroneCAN:

- [UAVCAN_PUB_RTCM](../advanced_config/parameter_reference.md#UAVCAN_PUB_RTCM):
  - Makes PX4 publish RTCM messages ([RTCMStream](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#rtcmstream)) to the bus (which it gets from the RTK base module via QGC).

Rover module parameters (also [set using QGC](../dronecan/index.md#qgc-cannode-parameter-configuration)):

- [CANNODE_SUB_RTCM](../advanced_config/parameter_reference.md#CANNODE_SUB_RTCM) tells the rover that it should subscribe to [RTCMStream](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#rtcmstream) RTCM messages on the bus (from the moving base).

:::info
Use [UAVCAN_PUB_MBD](../advanced_config/parameter_reference.md#UAVCAN_PUB_MBD) and [CANNODE_SUB_MBD](../advanced_config/parameter_reference.md#CANNODE_SUB_MBD) instead if you want to implement moving base (see below) at the same time.
:::

For more information see [Rover and Fixed Base](../dronecan/index.md#rover-and-fixed-base) in the DroneCAN guide.

## Значення LED індикаторів

- Світлодіоди статусу GPS розташовані праворуч від роз'ємів
  - Миготіння зеленого - це фіксація GPS
  - Миготіння синього - це отримані корекції та RTK Float
  - Сталий синій - це RTK зафіксовано

- Світлодіоди статусу CAN розташовані зверху ліворуч від роз'ємів
  - Повільне блимання зеленого - чекає на підключення CAN
  - Швидко блимаюче зелене світло - нормальна робота
  - Повільне блимання зеленим і синім - перелік CAN
  - Fast blinking blue and red is firmware update in progress
  - Миготливий червоний - помилка
    - Якщо ви бачите червоний світлодіод, це означає, що виникла помилка, і вам слід перевірити наступне
      - Переконайтеся, що у польотному контролері встановлено SD-картку
      - Make sure the ARK X20 RTK GPS has `ark_can-rtk-gps_canbootloader` installed prior to flashing `ark_can-rtk-gps_default`
      - Видаліть бінарні файли з кореневих та ufw директорій SD-карти та спробуйте зібрати та знову прошити

## Дивіться також

- [ARK X20 RTK GPS Documentation](https://docs.arkelectron.com/gps/ark-x20-rtk-gps) (ARK Docs)

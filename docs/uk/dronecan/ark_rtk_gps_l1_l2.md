# ARK RTK GPS L1 L5

[ARK RTK GPS L1 L5](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-rtk-gps) is an open source [DroneCAN](index.md) [RTK GPS](../gps_compass/rtk_gps.md), [u-blox F9P](https://www.u-blox.com/en/product/zed-f9p-module), magnetometer, barometer, IMU, buzzer, and safety switch module.

![ARK RTK GPS L1 L5](../../assets/hardware/gps/ark/ark_rtk_gps_l1_l5.jpg)

## Де купити

Замовте цей модуль з:

- [ARK Electronics](https://arkelectron.com/product/ark-rtk-gps-l1-l5/) (US)

## Характеристики обладнання

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_RTK_GPS)
- Датчики
  - Ublox F9P GPS
    - Multi-band GNSS receiver delivers centimetre level accuracy in seconds
    - Одночасний прийом GPS, GLONASS, Galileo та BeiDou
    - Багатосмуговий RTK зі швидкими часами збіжності та надійною продуктивністю
    - Висока швидкість оновлення для високодинамічних додатків
    - Centimetre accuracy in a small and energy efficient module
    - Does not Support Moving Base for Heading
  - Bosch BMM150 Magnetometer
  - Bosch BMP388 Barometer
  - Invensense ICM-42688-P 6-Axis IMU
- STM32F412CEU6 MCU
- Кнопка безпеки
- Зумер
- Два роз'єми стандарту CAN для Pixhawk (4 контакти JST GH)
- F9P `UART 2` Connector
  - 3-контактний JST-GH
  - TX, RX, GND
- Роз'єм для відлагодження стандарту Pixhawk (6 контактів JST SH)
- LED індикатори
  - Індикатор безпеки
  - GPS Fix
  - Статус RTK
  - RGB Статус системи
- USA Built
- Вимоги до живлення
  - 5V
  - Середній струм 170мA
  - 180мА Макс

## Налаштування програмного забезпечення

### Підключення

The ARK RTK GPS L1 L5 is connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable. For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Встановлення

The recommended mounting orientation is with the connectors on the board pointing towards the **back of vehicle**.

The sensor can be mounted anywhere on the frame, but you will need to specify its position, relative to vehicle centre of gravity, during [PX4 configuration](#px4-configuration).

## Налаштування прошивки

ARK RTK GPS L1 L5 runs the [PX4 cannode firmware](px4_cannode_fw.md). As such, it supports firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).

ARK RTK GPS L1 L5 boards ship with recent firmware pre-installed, but if you want to build and flash the latest firmware yourself, refer to the [cannode firmware build instructions](px4_cannode_fw.md#building-the-firmware).

Firmware target: `ark_can-rtk-gps_default`
Bootloader target: `ark_can-rtk-gps_canbootloader`

## Налаштування польотного контролера

### Увімкнення DroneCAN

In order to use the ARK RTK GPS L1 L5, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

Кроки наступні:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect ARK RTK GPS L1 L5 CAN to the Pixhawk CAN.

Після активації модуль буде виявлено при завантаженні.
Дані GPS повинні надходити з частотою 10 Гц.

### Конфігурація PX4

You need to set necessary [DroneCAN](index.md) parameters and define offsets if the sensor is not centred within the vehicle:

- Enable GPS yaw fusion by setting bit 3 of [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to true.
- Enable GPS blending to ensure the heading is always published by setting [SENS_GPS_MASK](../advanced_config/parameter_reference.md#SENS_GPS_MASK) to 7 (all three bits checked).
- Enable [UAVCAN_SUB_GPS](../advanced_config/parameter_reference.md#UAVCAN_SUB_GPS), [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG), and [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO).
- The parameters [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) can be set to account for the offset of the ARK RTK GPS L1 L5 from the vehicles centre of gravity.

### ARK RTK GPS L1 L5 Configuration

You may need to [configure the following parameters](../dronecan/index.md#qgc-cannode-parameter-configuration) on the ARK RTK GPS L1 L5 itself:

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
      - Make sure the ARK RTK GPS L1 L5 has `ark_can-rtk-gps_canbootloader` installed prior to flashing `ark_can-rtk-gps_default`
      - Видаліть бінарні файли з кореневих та ufw директорій SD-карти та спробуйте зібрати та знову прошити

### Оновлення модуля Ublox F9P

ARK RTK GPS L1 L5 comes with the Ublox F9P module up to date with version 1.13 or newer. Проте ви можете перевірити версію та оновити прошивку за бажанням.

Кроки наступні:

1. [Download u-center from u-blox.com](https://www.u-blox.com/en/product/u-center) and install on your PC (Windows only)
2. Open the [u-blox ZED-F9P website](https://www.u-blox.com/en/product/zed-f9p-module#tab-documentation-resources)
3. Прокрутіть вниз і клацніть на поле "Show Legacy Documents"
4. Прокрутіть вниз ще раз до Оновлення прошивки та завантажте потрібну прошивку (потрібна версія не нижче 1.13)
5. While holding down the safety switch on the ARK RTK GPS L1 L5, connect it to power via one of its CAN ports and hold until all 3 LEDs blink rapidly
6. Connect the ARK RTK GPS L1 L5 to your PC via its debug port with a cable such as the Black Magic Probe or an FTDI
7. Open u-center, select the COM port for the ARK RTK GPS L1 L5 and connect
   ![U-Center Connect](../../assets/hardware/gps/ark/ark_rtk_gps_ucenter_connect.png)
8. Check the current firmware version by selecting View, Messages View, UBX, MON, VER
   ![Check Version](../../assets/hardware/gps/ark/ark_rtk_gps_ublox_version.png)
9. Для оновлення прошивки:
   1. Виберіть Tools, Firmware Update
   2. Поле зображення прошивки повинно бути файлом .bin, завантаженим зі сторінки веб-сайту u-blox ZED-F9P
   3. Поставте прапорець "Використовувати цю швидкість передачі для оновлення" та виберіть 115200 зі списку
   4. Переконайтеся, що інші прапорці відображаються так, як показано нижче
   5. Натисніть зелену кнопку GO внизу зліва
   6. "Firmware Update SUCCESS" should be displayed if it updated successfully
      ![Firmware Update](../../assets/hardware/gps/ark/ark_rtk_gps_ublox_f9p_firmware_update.png)

## Дивіться також

- [ARK RTK GPS L1 L5 Documentation](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-rtk-gps) (ARK Docs)

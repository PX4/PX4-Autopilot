# ARK G5 RTK GPS

:::info
This GPS module is made in the USA and NDAA compliant.
:::

[ARK G5 RTK GPS](https://arkelectron.com/product/ark-g5-rtk-gps/) is a [DroneCAN](index.md) quad-band [RTK GPS](../gps_compass/rtk_gps.md).

The module incorporates the [Septentrio mosaic-G5 P3 Ultra-compact high-precision GPS/GNSS receiver module](https://www.u-blox.com/en/product/zed-x20p-module), magnetometer, barometer, IMU, and buzzer module.

![ARK G5 RTK GPS](../../assets/hardware/gps/ark/ark_g5_rtk_gps.png)

## Де купити

Замовте цей модуль з:

- [ARK Electronics](https://arkelectron.com/product/ark-g5-rtk-gps/) (US)

## Характеристики обладнання

- [DroneCAN](index.md) RTK GNSS, Magnetometer, Barometer, IMU, and Buzzer Module
- [Dronecan Firmware Updating](../dronecan/index.md#firmware-update)
- Датчики
  - [Septentrio mosaic-G5 P3 Ultra-compact high-precision GPS/GNSS receiver module](https://www.septentrio.com/en/products/gnss-receivers/gnss-receiver-modules/mosaic-G5-P3)
    - All-band all constellation GNSS receiver
    - All-in-view satellite tracking: multi-constellation, quad-band GNSS module receiver
    - Full raw data with positioning measurements and Galileo HAS positioning service compatibility
    - Best-in-class RTK cm-level positioning accuracy
    - Advanced GNSS+ algorithms
    - 20Hz update rate
  - [ST IIS2MDC Magnetometer](https://www.st.com/en/mems-and-sensors/iis2mdc.html)
  - [Bosch BMP390 Barometer](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/)
  - [Invensense ICM-42688-P 6-Axis IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- STM32F412VGH6 MCU
- Кнопка безпеки
- Зумер
- Two CAN Connectors (Pixhawk Connector Standard 4-pin JST GH)
- G5 "UART 2" Connector
  - 4-pin JST GH
  - TX, RX, PPS, GND
- G5 USB C
- Debug Connector (Pixhawk Connector Standard 6-pin JST SH)
- LED індикатори
  - GPS Fix
  - Статус RTK
  - RGB Статус системи
- USA Built
- NDAA Compliant
- Вимоги до живлення
  - 5V
    - 270mA
- Розміри
  - Without Antenna
    - 48.0mm x 40.0mm x 15.4mm
    - 13.0g
  - With Antenna
    - 48.0mm x 40.0mm x 51.0mm
    - 43.5g
- Includes
  - CAN Cable (Pixhawk Connector Standard 4-pin)
  - Full-Frequency Helical GPS Antenna

## Налаштування програмного забезпечення

### Підключення

The ARK G5 RTK GPS is connected to the CAN bus using a [Pixhawk connector standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) 4-pin JST GH cable.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Встановлення

The recommended mounting orientation is with the connectors on the board pointing towards the **back of vehicle**.

The sensor can be mounted anywhere on the frame, but you will need to specify its position, relative to vehicle centre of gravity, during [PX4 Configuration](#px4-configuration).

## Налаштування прошивки

The Septentrio G5 module firmware can be updated using the Septentrio [RxTools](https://www.septentrio.com/en/products/gps-gnss-receiver-software/rxtools) application.

## Налаштування польотного контролера

### Увімкнення DroneCAN

In order to use the ARK G5 RTK GPS, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

Кроки наступні:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect ARK G5 RTK GPS CAN to the Pixhawk CAN.

Після активації модуль буде виявлено при завантаженні.

There is also CAN built-in bus termination via [CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM)

### Конфігурація PX4

You need to set necessary [DroneCAN](index.md) parameters and define offsets if the sensor is not centred within the vehicle:

- Enable [UAVCAN_SUB_GPS](../advanced_config/parameter_reference.md#UAVCAN_SUB_GPS), [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG), and [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO).
- The parameters [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) can be set to account for the offset of the ARK G5 RTK GPS from the vehicle's centre of gravity.

## Значення LED індикаторів

The GPS status lights are located to the right of the connectors:

- Миготіння зеленого - це фіксація GPS
- Миготіння синього - це отримані корекції та RTK Float
- Сталий синій - це RTK зафіксовано

## Дивіться також

- [ARK G5 RTK GPS Documentation](https://docs.arkelectron.com/gps/ark-g5-rtk-gps) (ARK Docs)

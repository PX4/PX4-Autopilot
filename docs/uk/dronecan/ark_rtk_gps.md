# ARK RTK GPS

[ARK RTK GPS](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-rtk-gps) is an open source [DroneCAN](index.md) [RTK GPS](../gps_compass/rtk_gps.md), [u-blox F9P](https://www.u-blox.com/en/product/zed-f9p-module), magnetometer, barometer, IMU, buzzer, and safety switch module.

![ARK RTK GPS](../../assets/hardware/gps/ark/ark_rtk_gps.jpg)

## Де купити

Замовте цей модуль з:

- [ARK Electronics](https://arkelectron.com/product/ark-rtk-gps/) (US)

## Характеристики обладнання

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_RTK_GPS)
- Датчики
  - Ublox F9P GPS
    - Multi-band GNSS receiver delivers centimetre level accuracy in seconds
    - Одночасний прийом GPS, GLONASS, Galileo та BeiDou
    - Багатосмуговий RTK зі швидкими часами збіжності та надійною продуктивністю
    - Висока швидкість оновлення для високодинамічних додатків
    - Centimetre accuracy in a small and energy efficient module
  - Bosch BMM150 Magnetometer
  - Bosch BMP388 Barometer
  - Invensense ICM-42688-P 6-Axis IMU
- STM32F412CEU6 MCU
- Кнопка безпеки
- Зумер
- Два роз'єми стандарту CAN для Pixhawk (4 контакти JST GH)
- Роз'єм F9P "UART 2"
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

ARK RTK GPS підключений до шини CAN за допомогою стандартного кабелю Pixhawk 4 pin JST GH. For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Встановлення

The recommended mounting orientation is with the connectors on the board pointing towards the **back of vehicle**.

The sensor can be mounted anywhere on the frame, but you will need to specify its position, relative to vehicle centre of gravity, during [PX4 configuration](#px4-configuration).

## Налаштування прошивки

ARK RTK GPS runs the [PX4 cannode firmware](px4_cannode_fw.md). As such, it supports firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).

ARK RTK GPS boards ship with recent firmware pre-installed, but if you want to build and flash the latest firmware yourself, refer to the [cannode firmware build instructions](px4_cannode_fw.md#building-the-firmware).

Firmware target: `ark_can-rtk-gps_default`
Bootloader target: `ark_can-rtk-gps_canbootloader`

## Налаштування польотного контролера

### Увімкнення DroneCAN

In order to use the ARK RTK GPS, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

Кроки наступні:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Підключіть ARK RTK GPS CAN до шини CAN Pixhawk.

Після активації модуль буде виявлено при завантаженні.
Дані GPS повинні надходити з частотою 10 Гц.

### Конфігурація PX4

You need to set necessary [DroneCAN](index.md) parameters and define offsets if the sensor is not centred within the vehicle:

- Enable GPS yaw fusion by setting bit 3 of [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to true.
- Enable GPS blending to ensure the heading is always published by setting [SENS_GPS_MASK](../advanced_config/parameter_reference.md#SENS_GPS_MASK) to 7 (all three bits checked).
- Enable [UAVCAN_SUB_GPS](../advanced_config/parameter_reference.md#UAVCAN_SUB_GPS), [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG), and [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO).
- The parameters [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) can be set to account for the offset of the ARK RTK GPS from the vehicles centre of gravity.
- Set [CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM) to `1` on the GPS if this it that last node on the CAN bus.

### Setting Up Moving Baseline & GPS Heading

Найпростіший спосіб налаштування рухомого базису та напрямку GPS з двома модулями GPS ARK RTK відбувається через CAN, хоча можна зробити це через UART, щоб зменшити обсяг даних на шині CAN, якщо це потрібно.

Зверніть увагу, що заголовок виводиться лише у випадку, якщо Rover знаходиться в режимі фіксованого RTX. Він не виведе заголовок у RTK Float.

Налаштування через CAN:

- Переконайтеся, що ARK RTK GPS модулі підключені до Pixhawk через CAN (один може бути підключений до вторинного порту CAN). Два ARK RTK GPS повинні бути підключені до однієї і тієї ж CAN шини для надсилання корекцій.
- Choose one ARK RTK GPS to be the _Rover_ and one to be the _Moving Base_.
- Reopen QGroundControl, go to parameters, and select `Standard` to hide that dropdown and select `Component ##` to view each of your ARK RTK GPS's CAN node parameters
  ::: info
  `Component ##` won't be visible unless the ARK RTK GPS is connected to the Pixhawk prior to opening QGroundControl.

:::
- On the _Rover_, set the following:
  - [GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) to `3`
  - [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) to `0` if your _Rover_ is in front of your _Moving Base_, `90` if _Rover_ is right of _Moving Base_, `180` if _Rover_ is behind _Moving Base_, or `270` if _Rover_ is left of _Moving Base_.
  - [CANNODE_SUB_MBD](../advanced_config/parameter_reference.md#CANNODE_SUB_MBD) to `1`.
- On the _Moving Base_, set the following:
  - [GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) to `4`.
  - [CANNODE_PUB_MBD](../advanced_config/parameter_reference.md#CANNODE_PUB_MBD) to `1`.

Налаштування через UART:

- Переконайтеся, що модулі GPS ARK RTK підключені до Pixhawk через CAN.
- Переконайтеся, що модулі GPS ARK RTK підключені один до одного через їх порт UART2 (схема виводів UART2 показана нижче).
  Зверніть увагу, що TX одного модуля потрібно підключити до RX іншого.

| Pin | Назва |
| --- | ----- |
| 1   | TX    |
| 2   | RX    |
| 3   | GND   |

- On the _Rover_, set the following:
  - [GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) to `1`
  - [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) to `0` if your _Rover_ is in front of your _Moving Base_, `90` if _Rover_ is right of _Moving Base_, `180` if _Rover_ is behind _Moving Base_, or `270` if _Rover_ is left of _Moving Base_.
- On the _Moving Base_, set the following:
  - [GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) to `2`.

## Значення LED індикаторів

- Світлодіоди статусу GPS розташовані праворуч від роз'ємів

  - Миготіння зеленого - це фіксація GPS
  - Миготіння синього - це отримані корекції та RTK Float
  - Сталий синій - це RTK зафіксовано

- Світлодіоди статусу CAN розташовані зверху ліворуч від роз'ємів
  - Повільне блимання зеленого - чекає на підключення CAN
  - Швидко блимаюче зелене світло - нормальна робота
  - Повільне блимання зеленим і синім - перелік CAN
  - Повільне блимання зеленого, синього і червоного - оновлення прошивки в процесі
  - Миготливий червоний - помилка
    - Якщо ви бачите червоний світлодіод, це означає, що виникла помилка, і вам слід перевірити наступне
      - Переконайтеся, що у польотному контролері встановлено SD-картку
      - Make sure the ARK RTK GPS has `ark_can-rtk-gps_canbootloader` installed prior to flashing `ark_can-rtk-gps_default`
      - Видаліть бінарні файли з кореневих та ufw директорій SD-карти та спробуйте зібрати та знову прошити

### Оновлення модуля Ublox F9P

ARK RTK GPS поставляється з модулем Ublox F9P з версією 1.13 або новішою. Проте ви можете перевірити версію та оновити прошивку за бажанням.

Кроки наступні:

- [Download u-center from u-blox.com](https://www.u-blox.com/en/product/u-center) and install on your PC (Windows only)
- Open the [u-blox ZED-F9P website](https://www.u-blox.com/en/product/zed-f9p-module#tab-documentation-resources)
- Прокрутіть вниз і клацніть на поле "Show Legacy Documents"
- Прокрутіть вниз ще раз до Оновлення прошивки та завантажте потрібну прошивку (потрібна версія не нижче 1.13)
- Підтримуючи перемикач безпеки на ARK RTK GPS, підключіть його до живлення через один з його портів CAN і утримуйте до тих пір, поки всі 3 світлодіода не почнуть швидко мигати
- Підключіть ARK RTK GPS до комп'ютера за допомогою його порту відладки через кабель, такого як Black Magic Probe або FTDI
- Open u-center, select the COM port for the ARK RTK GPS and connect
  ![U-Center Connect](../../assets/hardware/gps/ark/ark_rtk_gps_ucenter_connect.png)
- Check the current firmware version by selecting View, Messages View, UBX, MON, VER
  ![Check Version](../../assets/hardware/gps/ark/ark_rtk_gps_ublox_version.png)
- Для оновлення прошивки:
  - Виберіть Tools, Firmware Update
  - Поле зображення прошивки повинно бути файлом .bin, завантаженим зі сторінки веб-сайту u-blox ZED-F9P
  - Поставте прапорець "Використовувати цю швидкість передачі для оновлення" та виберіть 115200 зі списку
  - Переконайтеся, що інші прапорці відображаються так, як показано нижче
  - Натисніть зелену кнопку GO внизу зліва
  - "Firmware Update SUCCESS" should be displayed if it updated successfully
    ![Firmware Update](../../assets/hardware/gps/ark/ark_rtk_gps_ublox_f9p_firmware_update.png)

## Дивіться також

- [ARK RTK GPS Documentation](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-rtk-gps) (ARK Docs)

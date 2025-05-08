# RTK GNSS (GPS)

[Real Time Kinematic (RTK)](https://en.wikipedia.org/wiki/Real_Time_Kinematic) GNSS/GPS systems provide centimeter-level accuracy, allowing PX4 to be used in applications like precision surveying (where pinpoint accuracy is essential).

This feature requires _QGroundControl_ running on a laptop/PC and a vehicle with a WiFi or Telemetry radio link to the ground station laptop.

:::info
Some RTK GNSS setups can provide yaw/heading information, as an alternative to the compass:

- [RTK GPS Heading with Dual u-blox F9P](../gps_compass/u-blox_f9p_heading.md).
- GPS безпосередньо виводить курс (див. таблицю нижче).

:::

## Пристрої, що підтримуються

PX4 supports the [u-blox M8P](https://www.u-blox.com/en/product/neo-m8p), [u-blox F9P](https://www.u-blox.com/en/product/zed-f9p-module) and the [Trimble MB-Two](https://www.trimble.com/Precision-GNSS/MB-Two-Board.aspx) GPS, and products that incorporate them.

Список сумісних пристроїв RTK нижче, які очікуються для роботи з PX4 (він виключає припинені пристрої).
Таблиця вказує пристрої, які також виводять курсову відмітку, а також можуть надавати курсову відмітку, коли використовуються дві одиниці на транспортному засобі.
Він також відзначає пристрої, які підключаються через CAN шину, та ті, які підтримують PPK (пост-процесуальну кінематику).

| Пристрій                                                                                                             |          GPS         |  Компас  | [DroneCAN](../dronecan/index.md) | [GPS Yaw](#configuring-gps-as-yaw-heading-source) | PPK |
| :------------------------------------------------------------------------------------------------------------------- | :------------------: | :------: | :------------------------------: | :-----------------------------------------------: | :-: |
| [ARK RTK GPS](../dronecan/ark_rtk_gps.md)                                                                            |          F9P         |  BMM150  |                 ✓                |                [Dual F9P][DualF9P]                |     |
| [ARK MOSAIC-X5 RTK GPS](../dronecan/ark_mosaic__rtk_gps.md)                                                          |       Mosaic-X5      |  IIS2MDC |                 ✓                |       [Septentrio Dual Antenna][SeptDualAnt]      |     |
| [CUAV C-RTK GPS](../gps_compass/rtk_gps_cuav_c-rtk.md)                                                               |        M8P/M8N       |     ✓    |                                  |                                                   |     |
| [CUAV C-RTK2](../gps_compass/rtk_gps_cuav_c-rtk2.md)                                                                 |          F9P         |     ✓    |                                  |                [Dual F9P][DualF9P]                |     |
| [CUAV C-RTK 9Ps GPS](../gps_compass/rtk_gps_cuav_c-rtk-9ps.md)                                                       |          F9P         |  RM3100  |                                  |                [Dual F9P][DualF9P]                |     |
| [CUAV C-RTK2 PPK/RTK GNSS](../gps_compass/rtk_gps_cuav_c-rtk.md)                                                     |          F9P         |  RM3100  |                                  |                                                   |  ✓  |
| [CubePilot Here+ RTK GPS](../gps_compass/rtk_gps_hex_hereplus.md)                                                    |          M8P         |  HMC5983 |                                  |                                                   |     |
| [CubePilot Here3 CAN GNSS GPS (M8N)](https://www.cubepilot.org/#/here/here3)                      |          M8P         | ICM20948 |                 ✓                |                                                   |     |
| [Drotek SIRIUS RTK GNSS ROVER (F9P)](https://store-drotek.com/911-sirius-rtk-gnss-rover-f9p.html) |          F9P         |  RM3100  |                                  |                [Dual F9P][DualF9P]                |     |
| [DATAGNSS GEM1305 RTK Receiver][DATAGNSS GEM1305 RTK]                                                                |        TAU951M       |     ✘    |                                  |                         ✘                         |     |
| [Femtones MINI2 Receiver](../gps_compass/rtk_gps_fem_mini2.md)                                                       |     FB672, FB6A0     |     ✓    |                                  |                                                   |     |
| [Freefly RTK GPS](../gps_compass/rtk_gps_freefly.md)                                                                 |          F9P         |  IST8310 |                                  |                                                   |     |
| [Holybro H-RTK ZED-F9P RTK Rover (DroneCAN variant)](../dronecan/holybro_h_rtk_zed_f9p_gps.md)    |          F9P         |  RM3100  |                 ✓                |                [Dual F9P][DualF9P]                |     |
| [Holybro H-RTK ZED-F9P RTK Rover](https://holybro.com/collections/h-rtk-gps/products/h-rtk-zed-f9p-rover)            |          F9P         |  RM3100  |                                  |                [Dual F9P][DualF9P]                |     |
| [Holybro H-RTK F9P Ultralight](https://holybro.com/products/h-rtk-f9p-ultralight)                                    |          F9P         |  IST8310 |                                  |                [Dual F9P][DualF9P]                |     |
| [Holybro H-RTK F9P Helical or Base](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md)                                     |          F9P         |  IST8310 |                                  |                [Dual F9P][DualF9P]                |     |
| [Holybro DroneCAN H-RTK F9P Helical](https://holybro.com/products/dronecan-h-rtk-f9p-helical)                        |          F9P         |  BMM150  |                 ✓                |                [Dual F9P][DualF9P]                |     |
| [Holybro H-RTK F9P Rover Lite](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md)                                          |          F9P         |  IST8310 |                                  |                                                   |     |
| [Holybro DroneCAN H-RTK F9P Rover](https://holybro.com/products/dronecan-h-rtk-f9p-rover)                            |          F9P         |  BMM150  |                                  |                [Dual F9P][DualF9P]                |     |
| [Holybro H-RTK M8P GNSS](../gps_compass/rtk_gps_holybro_h-rtk-m8p.md)                                                |          M8P         |  IST8310 |                                  |                                                   |     |
| [Holybro H-RTK Unicore UM982 GPS](../gps_compass/rtk_gps_holybro_unicore_um982.md)                                   |         UM982        |  IST8310 |                                  |       [Unicore Dual Antenna][UnicoreDualAnt]      |     |
| [LOCOSYS Hawk R1](../gps_compass/rtk_gps_locosys_r1.md)                                                              |      MC-1612-V2b     |          |                                  |                                                   |     |
| [LOCOSYS Hawk R2](../gps_compass/rtk_gps_locosys_r2.md)                                                              |      MC-1612-V2b     |  IST8310 |                                  |                                                   |     |
| [mRo u-blox ZED-F9 RTK L1/L2 GPS](https://store.mrobotics.io/product-p/m10020d.htm)                                  |          F9P         |     ✓    |                                  |                [Dual F9P][DualF9P]                |     |
| [Navisys L1/L2 ZED-F9P RTK - Base only](https://www.navisys.com.tw/productdetail?name=GR901&class=RTK)         |          F9P         |          |                                  |                                                   |     |
| [RaccoonLab L1/L2 ZED-F9P][RaccoonLab L1/L2 ZED-F9P]                                                                 |          F9P         |  RM3100  |                 ✓                |                                                   |     |
| [RaccoonLab L1/L2 ZED-F9P with external antenna][RaccnLabL1L2ZED-F9P ext_ant]                                        |          F9P         |  RM3100  |                 ✓                |                                                   |     |
| [Septentrio AsteRx-m3 Pro](../gps_compass/septentrio_asterx-rib.md)                                                  |        AsteRx        |     ✓    |                                  |       [Septentrio Dual Antenna][SeptDualAnt]      |  ✓  |
| [Septentrio mosaic-go](../gps_compass/septentrio_mosaic-go.md)                                                       | mosaic X5 / mosaic H |     ✓    |                                  |       [Septentrio Dual Antenna][SeptDualAnt]      |  ✓  |
| [SIRIUS RTK GNSS ROVER (F9P)](https://store-drotek.com/911-sirius-rtk-gnss-rover-f9p.html)        |          F9P         |     ✓    |                                  |                [Dual F9P][DualF9P]                |     |
| [SparkFun GPS-RTK2 Board - ZED-F9P](https://www.sparkfun.com/products/15136)                                         |          F9P         |     ✓    |                                  |                [Dual F9P][DualF9P]                |     |
| [Trimble MB-Two](../gps_compass/rtk_gps_trimble_mb_two.md)                                                           |          F9P         |     ✓    |                                  |                         ✓                         |     |

<!-- links used in above table -->

[RaccnLabL1L2ZED-F9P ext_ant]: https://docs.raccoonlab.co/guide/gps_mag_baro/gnss_external_antenna_f9p_v320.html
[RaccoonLab L1/L2 ZED-F9P]: https://docs.raccoonlab.co/guide/gps_mag_baro/gps_l1_l2_zed_f9p.html
[DualF9P]: ../gps_compass/u-blox_f9p_heading.md
[SeptDualAnt]: ../gps_compass/septentrio.md#gnss-based-heading
[UnicoreDualAnt]: ../gps_compass/rtk_gps_holybro_unicore_um982.md#enable-gps-heading-yaw
[DATAGNSS GEM1305 RTK]: ../gps_compass/rtk_gps_gem1305.md

Примітки:

- ✓ or a specific part number indicate that a features is supported, while ✘ or empty show that the feature is not supported.
  "?" означає "невідомо".
- Where possible and relevant the part name is used (i.e. ✓ in the GPS column indicates that a GPS module is present but the part is not known).
- Деякі RTK-модулі можна використовувати лише в певній ролі (база або ровер), тоді як інші можна використовувати як взаємозамінні.
- У списку може бути відсутнє деяке зняте з виробництва обладнання, яке все ще підтримується.
  For example [CubePilot Here+ RTK GPS](../gps_compass/rtk_gps_hex_hereplus.md) is discontinued and may be removed from the list in a future release.
  Перевірте попередні версії, якщо тут не згадано модуль, який перестали випускати.

## Налаштування/Конфігурація розташування

RTK positioning requires a _pair_ of [RTK GNSS devices](#supported-devices): a "base" for the ground station and a "rover" for the vehicle.

Крім того, вам знадобиться:

- A _laptop/PC_ with QGroundControl (QGroundControl for Android/iOS do not support RTK)
- Транспортний засіб із WiFi або телеметричним радіозв'язком з ноутбуком.

:::info
_QGroundControl_ with a base module can theoretically enable RTK GPS for multiple vehicles/rover modules.
На момент написання цього випадку використання цей випадок не був протестований.
:::

### Налаштування програмного забезпечення

#### Модуль Rover RTK (Транспортний)

The connection method and cables/connectors required depends on the selected RTK module (and on the [flight controller](../flight_controller/index.md)).

Більшість з'єднані через порт GPS контролера польоту, так само, як будь-який інший модуль GPS.
Some are connected to the [CAN](../can/index.md) bus (i.e. using [DroneCAN](../dronecan/index.md)).

See [documentation for the selected device](#supported-devices), general [GNSS Hardware/Configuration Setup](../gps_compass/index.md#hardware-setup), and [DroneCAN](../dronecan/index.md) for more information on wiring and configuration.

#### Базовий модуль RTK (наземний)

Connect the base module to _QGroundControl_ via USB.
Модуль бази не повинен зміщуватися, коли його використовують.

:::tip
Choose a position where the base module won't need to be moved, has a clear view of the sky, and is well separated from any buildings.
Часто корисно підняти базовий GPS, використовуючи штатив або монтувавши його на дах.
:::

#### Телеметрійне радіо/WiFi

The vehicle and ground control laptop must be connected via [wifi or a radio telemetry link](../telemetry/index.md).

The link _must_ use the MAVLink 2 protocol as it makes more efficient use of the channel.
This should be set by default, but if not, follow the [MAVLink2 configuration instructions](#mavlink2) below.

### Процес підключення RTK

Підключення RTK GPS насправді просте:

1. Start _QGroundControl_ and attach the base RTK GPS via USB to the ground station.
  Пристрій визнається автоматично.

2. Start the vehicle and make sure it is connected to _QGroundControl_.

  :::tip
  _QGroundControl_ displays an RTK GPS status icon in the top icon bar while an RTK GPS device is connected (in addition to the normal GPS status icon).
  Іконка червона, поки налаштовується RTK, а потім змінюється на білу, коли RTK GPS активний.
  Ви можете натиснути на піктограму, щоб побачити поточний стан та точність RTK.

:::

3. _QGroundControl_ then starts the RTK setup process (known as "Survey-In").

  Survey-In - це процедура запуску для отримання точної оцінки положення базової станції.
  The process typically takes several minutes (it ends after reaching the minimum time and accuracy specified in the [RTK settings](#rtk-gps-settings)).

  Ви можете відстежити прогрес, натиснувши на піктограму стану RTK GPS.

  ![survey-in](../../assets/qgc/setup/rtk/qgc_rtk_survey-in.png)

4. Після завершення опитування:

  - The RTK GPS icon changes to white and _QGroundControl_ starts to stream position data to the vehicle:

    ![RTK streaming](../../assets/qgc/setup/rtk/qgc_rtk_streaming.png)

  - Транспортний GPS переходить у режим RTK.
    The new mode is displayed in the _normal_ GPS status icon (`3D RTK GPS Lock`):

    ![RTK GPS Status](../../assets/qgc/setup/rtk/qgc_rtk_gps_status.png)

### Налаштування GPS як Джерело розділення/Курсування

GPS can be used as a source for yaw fusion when using a single device with two antenna where _yaw output is supported by the device_, or when using some [RTK GPS Setups with Dual u-blox F9P](../gps_compass/u-blox_f9p_heading.md).
Using GPS as a heading source has the benefit that yaw calculations are not impacted by magnetic interference.

Both approaches work comparing the time taken for a GNSS signal to reach two separated antennas.
The minimum distance between antenna depends on the device but is of the order of 50 cm (check manufacturer documentation).

The devices that can be used are listed in this way are listed in the **GPS Yaw** column of the table above, such as [Septentrio AsteRx-m3 Pro](../gps_compass/septentrio_asterx-rib.md), [Holybro H-RTK Unicore UM982 GPS](../gps_compass/rtk_gps_holybro_unicore_um982.md), and [Trimble MB-Two](../gps_compass/rtk_gps_trimble_mb_two.md).
The links in the table take you to the device-specific PX4 configuration.

Generally when using a GNSS as a source of yaw information you will need to configure the following parameters:

| Параметр                           | Налаштування                                                                                                                                                |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [GPS\_YAW\_OFFSET][GPS_YAW_OFFSET] | The angle made by the _baseline_ (the line between the two GPS antennas) relative to the vehicle x-axis (front/back axis, as shown [here][fc_orientation]). |
| [EKF2\_GPS\_CTRL][EKF2_GPS_CTRL]   | Set bit position 3 "Dual antenna heading" to `1` (i.e. add 8 to the parameter value).    |

<!-- links used in table above -->

[GPS_YAW_OFFSET]: ../advanced_config/parameter_reference.md#GPS_YAW_OFFSET
[EKF2_GPS_CTRL]: ../advanced_config/parameter_reference.md#EKF2_GPS_CTRL
[fc_orientation]: ../config/flight_controller_orientation.md#calculating-orientation

:::tip
If using this feature, all other configuration should be setup up as normal (e.g. [RTK Positioning](../gps_compass/rtk_gps.md#positioning-setup-configuration)).
:::

### Додаткова конфігурація PX4

The following settings may need to be changed (using _QGroundControl_).

#### Налаштування RTK GPS

The RTK GPS settings are specified in the _QGroundControl_ [General Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html#rtk_gps) (**SettingsView > General Settings > RTK GPS**).

![RTK GPS Setup](../../assets/qgc/setup/rtk/settings_view_general_rtk_gps.jpg)

Ці параметри визначають мінімальну тривалість та мінімальну точність для завершення процесу налаштування RTK GPS (відомий як "Survey-In).

:::tip
You can save and reuse a base position in order to save time: perform Survey-In once, select _Use Specified Base Position_ and press **Save Current Base Position** to copy in the values for the last survey.
Значення будуть збережені після перезавантажень QGC до тих пір, поки їх не змінять.
:::

#### MAVLink2

Протокол MAVLink2 повинен бути використаний, оскільки він ефективніше використовує канали з низькою пропускною здатністю.
Це має бути увімкнено за замовчуванням на останніх збірках.

Для забезпечення використання MAVLink2:

- Update the telemetry module firmware to the latest version (see [QGroundControl > Setup > Firmware](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html)).
- Set [MAV_PROTO_VER](../advanced_config/parameter_reference.md#MAV_PROTO_VER) to 2 (see [QGroundControl Setup > Parameters](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/parameters.html))

#### Вдосконалення

Вам може додатково знадобитися налаштувати деякі параметри, оскільки параметри за замовчуванням налаштовані з припущенням точності GPS в порядку метрів, а не сантиметрів.
For example, you can decrease [EKF2_GPS_V_NOISE](../advanced_config/parameter_reference.md#EKF2_GPS_V_NOISE) and [EKF2_GPS_P_NOISE](../advanced_config/parameter_reference.md#EKF2_GPS_P_NOISE) to 0.2.

#### Подвійні приймачі

Другий приймач GPS може бути використаний як резервний (RTK або не RTK).
See the [Using PX4's Navigation Filter (EKF2) > GPS](../advanced_config/tuning_the_ecl_ekf.md#gps) section.

<!--
- Video demonstration would be nice.
- something that shows positioning of base, connection of RTK rover, survey in process. Some sort of short precision survey.
-->

## Подальша інформація

- [RTK-GPS (PX4-Integration)](../advanced/rtk_gps.md): Developer information about integrating RTK-GPS support into PX4.
- [Real Time Kinematic](https://en.wikipedia.org/wiki/Real_Time_Kinematic) (Wikipedia)

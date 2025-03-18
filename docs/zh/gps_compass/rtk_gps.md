# RTK GPS

[Real Time Kinematic (RTK)](https://en.wikipedia.org/wiki/Real_Time_Kinematic) GNSS/GPS systems provide centimeter-level accuracy, allowing PX4 to be used in applications like precision surveying (where pinpoint accuracy is essential).

This feature requires _QGroundControl_ running on a laptop/PC and a vehicle with a WiFi or Telemetry radio link to the ground station laptop.

:::info
Some RTK GNSS setups can provide yaw/heading information, as an alternative to the compass:

- [RTK GPS Heading with Dual u-blox F9P](../gps_compass/u-blox_f9p_heading.md).
- GPS directly output yaw (see table below).

:::

## 支持的 RTK 设备

PX4 supports the [u-blox M8P](https://www.u-blox.com/en/product/neo-m8p), [u-blox F9P](https://www.u-blox.com/en/product/zed-f9p-module) and the [Trimble MB-Two](https://www.trimble.com/Precision-GNSS/MB-Two-Board.aspx) GPS, and products that incorporate them.

The RTK compatible devices below that are expected to work with PX4 (it omits discontined devices).
The table indicates devices that also output yaw, and that can provide yaw when two on-vehicle units are used.
It also highlights devices that connect via the CAN bus, and those which support PPK (Post-Processing Kinematic).

| 设备                                                                                                                   |          GPS         |              罗盘             | [DroneCAN](../dronecan/index.md) | [GPS Yaw](#configuring-gps-as-yaw-heading-source) |             PPK             |
| :------------------------------------------------------------------------------------------------------------------- | :------------------: | :-------------------------: | :------------------------------: | :-----------------------------------------------: | :-------------------------: |
| [ARK RTK GPS](../dronecan/ark_rtk_gps.md)                                                                            |          F9P         |            BMM150           |    &check;   |                [Dual F9P][DualF9P]                |                             |
| [ARK MOSAIC-X5 RTK GPS](../dronecan/ark_mosaic__rtk_gps.md)                                                          |       Mosaic-X5      |           IIS2MDC           |    &check;   |       [Septentrio Dual Antenna][SeptDualAnt]      |                             |
| [CUAV C-RTK GPS](../gps_compass/rtk_gps_cuav_c-rtk.md)                                                               |        M8P/M8N       | &check; |                                  |                                                   |                             |
| [CUAV C-RTK2](../gps_compass/rtk_gps_cuav_c-rtk2.md)                                                                 |          F9P         | &check; |                                  |                [Dual F9P][DualF9P]                |                             |
| [CUAV C-RTK 9Ps GPS](../gps_compass/rtk_gps_cuav_c-rtk-9ps.md)                                                       |          F9P         |            RM3100           |                                  |                [Dual F9P][DualF9P]                |                             |
| [CUAV C-RTK2 PPK/RTK GNSS](../gps_compass/rtk_gps_cuav_c-rtk.md)                                                     |          F9P         |            RM3100           |                                  |                                                   | &check; |
| [CubePilot Here+ RTK GPS](../gps_compass/rtk_gps_hex_hereplus.md)                                                    |          M8P         |           HMC5983           |                                  |                                                   |                             |
| [CubePilot Here3 CAN GNSS GPS (M8N)](https://www.cubepilot.org/#/here/here3)                      |          M8P         |           ICM20948          |    &check;   |                                                   |                             |
| [Drotek SIRIUS RTK GNSS ROVER (F9P)](https://store-drotek.com/911-sirius-rtk-gnss-rover-f9p.html) |          F9P         |            RM3100           |                                  |                [Dual F9P][DualF9P]                |                             |
| [DATAGNSS GEM1305 RTK Receiver][DATAGNSS GEM1305 RTK]                                                                |        TAU951M       |              ✘              |                                  |                         ✘                         |                             |
| [Femtones MINI2 Receiver](../gps_compass/rtk_gps_fem_mini2.md)                                                       |     FB672, FB6A0     | &check; |                                  |                                                   |                             |
| [Freefly RTK GPS](../gps_compass/rtk_gps_freefly.md)                                                                 |          F9P         |           IST8310           |                                  |                                                   |                             |
| [Holybro H-RTK ZED-F9P RTK Rover (DroneCAN variant)](../dronecan/holybro_h_rtk_zed_f9p_gps.md)    |          F9P         |            RM3100           |    &check;   |                [Dual F9P][DualF9P]                |                             |
| [Holybro H-RTK ZED-F9P RTK Rover](https://holybro.com/collections/h-rtk-gps/products/h-rtk-zed-f9p-rover)            |          F9P         |            RM3100           |                                  |                [Dual F9P][DualF9P]                |                             |
| [Holybro H-RTK F9P Ultralight](https://holybro.com/products/h-rtk-f9p-ultralight)                                    |          F9P         |           IST8310           |                                  |                [Dual F9P][DualF9P]                |                             |
| [Holybro H-RTK F9P Helical or Base](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md)                                     |          F9P         |           IST8310           |                                  |                [Dual F9P][DualF9P]                |                             |
| [Holybro DroneCAN H-RTK F9P Helical](https://holybro.com/products/dronecan-h-rtk-f9p-helical)                        |          F9P         |            BMM150           |    &check;   |                [Dual F9P][DualF9P]                |                             |
| [Holybro H-RTK F9P Rover Lite](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md)                                          |          F9P         |           IST8310           |                                  |                                                   |                             |
| [Holybro DroneCAN H-RTK F9P Rover](https://holybro.com/products/dronecan-h-rtk-f9p-rover)                            |          F9P         |            BMM150           |                                  |                [Dual F9P][DualF9P]                |                             |
| [Holybro H-RTK M8P GNSS](../gps_compass/rtk_gps_holybro_h-rtk-m8p.md)                                                |          M8P         |           IST8310           |                                  |                                                   |                             |
| [Holybro H-RTK Unicore UM982 GPS](../gps_compass/rtk_gps_holybro_unicore_um982.md)                                   |         UM982        |           IST8310           |                                  |       [Unicore Dual Antenna][UnicoreDualAnt]      |                             |
| [LOCOSYS Hawk R1](../gps_compass/rtk_gps_locosys_r1.md)                                                              |      MC-1612-V2b     |                             |                                  |                                                   |                             |
| [LOCOSYS Hawk R2](../gps_compass/rtk_gps_locosys_r2.md)                                                              |      MC-1612-V2b     |           IST8310           |                                  |                                                   |                             |
| [mRo u-blox ZED-F9 RTK L1/L2 GPS](https://store.mrobotics.io/product-p/m10020d.htm)                                  |          F9P         | &check; |                                  |                [Dual F9P][DualF9P]                |                             |
| [Navisys L1/L2 ZED-F9P RTK - Base only](https://www.navisys.com.tw/productdetail?name=GR901&class=RTK)         |          F9P         |                             |                                  |                                                   |                             |
| [RaccoonLab L1/L2 ZED-F9P][RaccoonLab L1/L2 ZED-F9P]                                                                 |          F9P         |            RM3100           |    &check;   |                                                   |                             |
| [RaccoonLab L1/L2 ZED-F9P with external antenna][RaccnLabL1L2ZED-F9P ext_ant]                                        |          F9P         |            RM3100           |    &check;   |                                                   |                             |
| [Septentrio AsteRx-m3 Pro](../gps_compass/septentrio_asterx-rib.md)                                                  |        AsteRx        | &check; |                                  |       [Septentrio Dual Antenna][SeptDualAnt]      | &check; |
| [Septentrio mosaic-go](../gps_compass/septentrio_mosaic-go.md)                                                       | mosaic X5 / mosaic H | &check; |                                  |       [Septentrio Dual Antenna][SeptDualAnt]      | &check; |
| [SIRIUS RTK GNSS ROVER (F9P)](https://store-drotek.com/911-sirius-rtk-gnss-rover-f9p.html)        |          F9P         | &check; |                                  |                [Dual F9P][DualF9P]                |                             |
| [SparkFun GPS-RTK2 Board - ZED-F9P](https://www.sparkfun.com/products/15136)                                         |          F9P         | &check; |                                  |                [Dual F9P][DualF9P]                |                             |
| [Trimble MB-Two](../gps_compass/rtk_gps_trimble_mb_two.md)                                                           |          F9P         | &check; |                                  |            &check;            |                             |

<!-- links used in above table -->

[RaccnLabL1L2ZED-F9P ext_ant]: https://docs.raccoonlab.co/guide/gps_mag_baro/gnss_external_antenna_f9p_v320.html
[RaccoonLab L1/L2 ZED-F9P]: https://docs.raccoonlab.co/guide/gps_mag_baro/gps_l1_l2_zed_f9p.html
[DualF9P]: ../gps_compass/u-blox_f9p_heading.md
[SeptDualAnt]: ../gps_compass/septentrio.md#gnss-based-heading
[UnicoreDualAnt]: ../gps_compass/rtk_gps_holybro_unicore_um982.md#enable-gps-heading-yaw
[DATAGNSS GEM1305 RTK]: ../gps_compass/rtk_gps_gem1305.md

备注：

- ✓ or a specific part number indicate that a features is supported, while ✘ or empty show that the feature is not supported.
  "?" indicates "unknown".
- Where possible and relevant the part name is used (i.e. ✓ in the GPS column indicates that a GPS module is present but the part is not known).
- Some RTK modules can only be used in a particular role (base or rover), while others can be used interchangeably.
- The list may omit some discontinued hardware that is still supported.
  For example [CubePilot Here+ RTK GPS](../gps_compass/rtk_gps_hex_hereplus.md) is discontinued and may be removed from the list in a future release.
  Check earlier versions if a discontinued module is not mentioned here.

## Positioning Setup/Configuration

RTK positioning requires a _pair_ of [RTK GNSS devices](#supported-devices): a "base" for the ground station and a "rover" for the vehicle.

In addition you will need:

- A _laptop/PC_ with QGroundControl (QGroundControl for Android/iOS do not support RTK)
- A vehicle with a WiFi or Telemetry radio link to the laptop.

:::info
_QGroundControl_ with a base module can theoretically enable RTK GPS for multiple vehicles/rover modules.
At time of writing this use case has not been tested.
:::

### 硬件安装

#### Rover RTK Module (Vehicle)

The connection method and cables/connectors required depends on the selected RTK module (and on the [flight controller](../flight_controller/index.md)).

Most are connected via the flight controller's GPS port, in the same way as any other GPS module.
Some are connected to the [CAN](../can/index.md) bus (i.e. using [DroneCAN](../dronecan/index.md)).

See [documentation for the selected device](#supported-devices), general [GNSS Hardware/Configuration Setup](../gps_compass/index.md#hardware-setup), and [DroneCAN](../dronecan/index.md) for more information on wiring and configuration.

#### Base RTK Module (Ground)

Connect the base module to _QGroundControl_ via USB.
The base module must not be moved while it is being used.

:::tip
Choose a position where the base module won't need to be moved, has a clear view of the sky, and is well separated from any buildings.
Often it is helpful to elevate the base GPS, by using a tripod or mounting it on a roof.
:::

#### Telemetry Radio/WiFi

The vehicle and ground control laptop must be connected via [wifi or a radio telemetry link](../telemetry/index.md).

The link _must_ use the MAVLink 2 protocol as it makes more efficient use of the channel.
This should be set by default, but if not, follow the [MAVLink2 configuration instructions](#mavlink2) below.

### RTK Connection Process

The RTK GPS connection is essentially plug and play:

1. Start _QGroundControl_ and attach the base RTK GPS via USB to the ground station.
  电脑会自动识别设备。

2. Start the vehicle and make sure it is connected to _QGroundControl_.

  :::tip
  _QGroundControl_ displays an RTK GPS status icon in the top icon bar while an RTK GPS device is connected (in addition to the normal GPS status icon).
  The icon is red while RTK is being set up, and then changes to white once RTK GPS is active.
  You can click the icon to see the current state and RTK accuracy.

:::

3. _QGroundControl_ then starts the RTK setup process (known as "Survey-In").

  测量是一个获得基站准确位置的设置过程。
  The process typically takes several minutes (it ends after reaching the minimum time and accuracy specified in the [RTK settings](#rtk-gps-settings)).

  你也可以点击 RTK状态按钮查看。

  ![survey-in](../../assets/qgc/setup/rtk/qgc_rtk_survey-in.png)

4. 测量完成：

  - The RTK GPS icon changes to white and _QGroundControl_ starts to stream position data to the vehicle:

    ![RTK streaming](../../assets/qgc/setup/rtk/qgc_rtk_streaming.png)

  - Vehicle GPS switches to RTK mode.
    The new mode is displayed in the _normal_ GPS status icon (`3D RTK GPS Lock`):

    ![RTK GPS Status](../../assets/qgc/setup/rtk/qgc_rtk_gps_status.png)

### Configuring GPS as Yaw/Heading Source

GPS can be used as a source for yaw fusion when using a single device with two antenna where _yaw output is supported by the device_, or when using some [RTK GPS Setups with Dual u-blox F9P](../gps_compass/u-blox_f9p_heading.md).
Using GPS as a heading source has the benefit that yaw calculations are not impacted by magnetic interference.

Both approaches work comparing the time taken for a GNSS signal to reach two separated antennas.
The minimum distance between antenna depends on the device but is of the order of 50 cm (check manufacturer documentation).

The devices that can be used are listed in this way are listed in the **GPS Yaw** column of the table above, such as [Septentrio AsteRx-m3 Pro](../gps_compass/septentrio_asterx-rib.md), [Holybro H-RTK Unicore UM982 GPS](../gps_compass/rtk_gps_holybro_unicore_um982.md), and [Trimble MB-Two](../gps_compass/rtk_gps_trimble_mb_two.md).
The links in the table take you to the device-specific PX4 configuration.

Generally when using a GNSS as a source of yaw information you will need to configure the following parameters:

| 参数                                 | 设置                                                                                                                                                          |
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

### Optional PX4 Configuration

The following settings may need to be changed (using _QGroundControl_).

#### RTK GPS settings

The RTK GPS settings are specified in the _QGroundControl_ [General Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html#rtk_gps) (**SettingsView > General Settings > RTK GPS**).

![RTK GPS Setup](../../assets/qgc/setup/rtk/settings_view_general_rtk_gps.jpg)

These settings define the minimum duration and minimum accuracy for completing the RTK GPS setup process (known as "Survey-In).

:::tip
You can save and reuse a base position in order to save time: perform Survey-In once, select _Use Specified Base Position_ and press **Save Current Base Position** to copy in the values for the last survey.
The values will then persist across QGC reboots until they are changed.
:::

#### MAVLink2

The MAVLink2 protocol must be used because it makes more efficient use of lower-bandwidth channels.
This should be enabled by default on recent builds.

To ensure MAVLink2 is used:

- Update the telemetry module firmware to the latest version (see [QGroundControl > Setup > Firmware](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html)).
- Set [MAV_PROTO_VER](../advanced_config/parameter_reference.md#MAV_PROTO_VER) to 2 (see [QGroundControl Setup > Parameters](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/parameters.html))

#### 调试

You may also need to tune some parameters as the default parameters are tuned assuming a GPS accuracy in the order of meters, not centimeters.
For example, you can decrease [EKF2_GPS_V_NOISE](../advanced_config/parameter_reference.md#EKF2_GPS_V_NOISE) and [EKF2_GPS_P_NOISE](../advanced_config/parameter_reference.md#EKF2_GPS_P_NOISE) to 0.2.

#### 双 GPS 接收器

A second GPS receiver can be used as a backup (either RTK or non RTK).
See the [Using PX4's Navigation Filter (EKF2) > GPS](../advanced_config/tuning_the_ecl_ekf.md#gps) section.

<!--
- Video demonstration would be nice.
- something that shows positioning of base, connection of RTK rover, survey in process. Some sort of short precision survey.
-->

## 更多信息

- [RTK-GPS (PX4-Integration)](../advanced/rtk_gps.md): Developer information about integrating RTK-GPS support into PX4.
- [Real Time Kinematic](https://en.wikipedia.org/wiki/Real_Time_Kinematic) (Wikipedia)

# RTK GPS Heading with Dual u-blox F9P

Two u-blox F9P [RTK GPS](../gps_compass/rtk_gps.md) modules mounted on a vehicle can be used to accurately compute a heading angle (i.e. an alternative to compass-based heading estimation).
The two GPS devices in this scenario are referred to as the _Moving Base_ and _Rover_.

## Supported Devices

This feature works on F9P devices that support CAN or expose the GPS UART2 port.

The following devices are supported:

- [ARK RTK GPS](https://arkelectron.com/product/ark-rtk-gps/) (arkelectron.com)
- [SparkFun GPS-RTK2 Board - ZED-F9P](https://www.sparkfun.com/products/15136) (www.sparkfun.com)
- [SIRIUS RTK GNSS ROVER (F9P)](https://store-drotek.com/911-sirius-rtk-gnss-rover-f9p.html) (store-drotek.com)
- [mRo u-blox ZED-F9 RTK L1/L2 GPS](https://store.mrobotics.io/product-p/m10020d.htm) (store.mrobotics.io)
- [Holybro H-RTK F9P Helical or Base](https://holybro.com/products/h-rtk-f9p-gnss-series) (Holybro Store)
- [Holybro DroneCAN H-RTK F9P Rover or Helical](https://holybro.com/collections/dronecan-h-rtk) (Holybro Store)
- [CUAV C-RTK 9Ps](https://store.cuav.net/shop/c-rtk-9ps/) (CUAV Store)

::: info

- [Freefly RTK GPS](../gps_compass/rtk_gps_freefly.md) and [Holybro H-RTK F9P Rover Lite](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md) cannot be used because they do not expose the CAN or UART2 port.
- Supported devices are also listed in [RTK GNSS (GPS) > Supported Devices](../gps_compass/rtk_gps.md#supported-devices).
  :::

## Setup

Ideally the two antennas should be identical, on the same level/horizontal plane and oriented the same way, and on an identical ground plane size and shape ([Application note](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf), section _System Level Considerations_).

- The application note does not state the minimal required separation between modules (50cm has been used in test vehicles running PX4).
- The antennas can be positioned as needed, but the [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) must be configured:
  [RTK GPS > GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source).

### UART Setup

- The UART2 of the GPS devices need to be connected together (TXD2 of the "Moving Base" to RXD2 of the "Rover")
- Connect UART1 on each of the GPS to (separate) unused UART's on the autopilot, and configure both of them as GPS with baudrate set to `Auto`.
  The mapping is as follows:
  - Main GPS = Rover
  - Secondary GPS = Moving Base
- Set [GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) to `Heading` (1)
- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) parameter bit 3 must be set (see [RTK GPS > GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source)).
- [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) may need to be set (see [RTK GPS > GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source)).
- Reboot and wait until both devices have GPS reception.
  `gps status` should then show the Main GPS going into RTK mode, which means the heading angle is available.

### CAN Setup

Refer to the CAN RTK GPS documentation for each specific device for the setup instructions (such as [ARK RTK GPS > Setting Up Moving Baseline & GPS Heading](../dronecan/ark_rtk_gps.md#setting-up-moving-baseline-gps-heading))

::: info
If using RTK with a fixed base station the secondary GPS will show the RTK state w.r.t. the base station.
:::

## Further Information

- [ZED-F9P Moving base applications (Application note)](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf) - General setup/instructions.
- [RTK GPS > GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source)

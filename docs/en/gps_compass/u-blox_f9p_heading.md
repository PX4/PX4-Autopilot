# RTK GPS Heading with Dual u-blox F9P

Two u-blox F9P [RTK GPS](../gps_compass/rtk_gps.md) modules mounted on a vehicle can be used to accurately compute a heading angle (i.e. an alternative to compass-based heading estimation).
The two GPS devices in this scenario are referred to as the _Moving Base_ and _Rover_.

## Supported Devices

This feature works on F9P devices that support CAN or expose the GPS UART2 port.

The following devices are supported:

- [ARK RTK GPS](https://arkelectron.com/product/ark-rtk-gps/) (arkelectron.com)
- [SparkFun GPS-RTK2 Board - ZED-F9P](https://www.sparkfun.com/sparkfun-gps-rtk2-board-zed-f9p-qwiic-gps-15136.html) (www.sparkfun.com)
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

- The UART2 of the GPS devices need to be connected together (TXD2 of the "Moving Base" to RXD2 of the "Rover").
- Connect UART1 on each of the GPS to (separate) unused UARTs on the autopilot, and configure both of them as GPS with the serial port baudrate set to `Auto` (the driver probes the link, then moves the receiver to [GPS_UBX_BAUD1](../advanced_config/parameter_reference.md#GPS_UBX_BAUD1)).
  The mapping is as follows:
  - Main GPS = Rover
  - Secondary GPS = Moving Base
- Set [GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) to `Heading` (1).
- Configure [GPS_UBX_BAUD1](../advanced_config/parameter_reference.md#GPS_UBX_BAUD1) / [GPS_UBX_BAUD2](../advanced_config/parameter_reference.md#GPS_UBX_BAUD2) as needed (see [u-blox UART baudrates](#u-blox-uart-baudrates)).
- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) parameter bit 3 must be set (see [RTK GPS > GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source)).
- [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) may need to be set (see [RTK GPS > GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source)).
- Reboot and wait until both devices have GPS reception.
  `gps status` should then show the Main GPS going into RTK mode, which means the heading angle is available.

### CAN Setup

Refer to the CAN RTK GPS documentation for each specific device for the setup instructions (such as [ARK RTK GPS > Setting Up Moving Baseline & GPS Heading](../dronecan/ark_rtk_gps.md#setting-up-moving-baseline-gps-heading)).
Set [GPS_UBX_BAUD1](../advanced_config/parameter_reference.md#GPS_UBX_BAUD1) / [GPS_UBX_BAUD2](../advanced_config/parameter_reference.md#GPS_UBX_BAUD2) on each CAN GPS node (see [u-blox UART baudrates](#u-blox-uart-baudrates)).

::: info
If using RTK with a fixed base station the secondary GPS will show the RTK state w.r.t. the base station.
:::

### u-blox UART baudrates

The GPS driver auto-detects the UART1 link, then moves the receiver to a target rate. That target is set by parameters — not by the flight-controller serial-port baud (keep that at `Auto` so probing still works).

| Parameter | Port | Default | When to change |
| --- | --- | --- | --- |
| [GPS_UBX_BAUD1](../advanced_config/parameter_reference.md#GPS_UBX_BAUD1) | UART1 (to FC or CAN-node MCU) | `0` → 115200 | Raise for high nav rates or when RTCM shares UART1 ([GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) 3/4). Keep lower on long or unreliable serial cables. |
| [GPS_UBX_BAUD2](../advanced_config/parameter_reference.md#GPS_UBX_BAUD2) | UART2 (between modules, or corrections) | 230400 | Used when UART2 carries RTCM (modes 1/2 and similar). Lower if attaching a telemetry radio to UART2. |

Notes:

- `GPS_UBX_BAUD1` is the UART1 target in every mode (heading modes do not hard-code a baudrate).
- On ARK DroneCAN GPS modules the MCU↔receiver link is on-board and short; firmware board defaults set `GPS_UBX_BAUD1` to `921600`. Only lower it if you rewire UART1 over a long off-board cable.
- For dual serial F9P with RTCM on UART2 (mode 1), the default UART1 rate is usually enough; raise `GPS_UBX_BAUD1` if you enable a high [GPS_UBX_RATE](../advanced_config/parameter_reference.md#GPS_UBX_RATE) and see dropouts.
- On a flight controller these parameters apply to the vehicle GPS driver(s). On a DroneCAN GPS they are set on the **node** ([QGC Component parameters](../dronecan/index.md#qgc-cannode-parameter-configuration)).

## Further Information

- [ZED-F9P Moving base applications (Application note)](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf) - General setup/instructions.
- [RTK GPS > GPS as Yaw/Heading Source](../gps_compass/rtk_gps.md#configuring-gps-as-yaw-heading-source)

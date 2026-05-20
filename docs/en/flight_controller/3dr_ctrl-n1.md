# 3DR Control N1 Flight Controller

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact [3DR](https://3dr.com/) for hardware support or compliance issues.
:::

The _3DR Control N1_ is a compact, high-performance, low-profile and lightweight flight controller designed and assembled in USA.
It runs PX4 on the [NuttX](https://nuttx.apache.org/) OS.

![3DR Control N1](../../assets/flight_controller/3dr_ctrl-n1/3dr_ctrl-n1.jpg)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

::: info
This flight controller requires a carrier board to work correctly, visit [3DR](https://3dr.com/) for more information.
:::

## Key Features

- **Processor:** STMicro STM32H743 Arm Cortex-M7 up to 480 MHz, 1 MB RAM, 2 MB Flash, 8 MB external flash
- **Sensors:**
  - 2x InvenSense IIM-42653 IMU (6-DoF)
  - Asahi Kasei AK09940A magnetometer
  - Infineon DPS368 barometer
- **Interfaces:**
  - 12x PWM motor outputs (DShot, bidirectional DShot, GPIO)
  - 7x serial ports (3x with hardware flow control)
  - 2x FD-CAN (up to 8 Mbps)
  - 2x I2C (up to 400 kHz)
  - 1x SPI (up to 40 MHz)
  - 4x GPIO (push-pull, direct from MCU)
  - 1x USB Full Speed (native STM32H7 port)
  - 1x SDIO/SDMMC interface
  - 1x addressable LED output (NeoPixel-compatible)
  - Serial Wire Debug (SWD)
- **Dimensions:** 28 × 17.6 × 3.7 mm
- **Weight:** 2.32 g
- **Input Voltage:** 4.8 – 6.0 V
- **Operating Temperature:** -20 to 75 °C

## Building Firmware

::: tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make 3dr_ctrl-n1_default
```

## Connectors

The Control N1 uses three Hirose DF40-series board-to-board connectors:

| Connector | Pins | Signals                                                |
| --------- | ---- | ------------------------------------------------------ |
| J100      | 30   | USART1, USART2, USART3, UART4, SDMMC, USB              |
| J200      | 30   | USART6, FDCAN1, SPI, SWD, miscellaneous                |
| J300      | 80   | UART7, UART8, FDCAN2, I2C, ADC, Motor outputs CH1–CH12 |

For more information about the pinout visit [3DR Control N1 Pinout Tool](https://docs.3dr.com/autopilots/control-n1/#pinout).

## Serial Port Mapping

| Port | Connector | Peripheral | Flow Control | Default Function |
| ---- | --------- | ---------- | ------------ | ---------------- |
| SG1  | J100      | USART2     | Yes          | TELEM1           |
| SG2  | J100      | UART4      | Yes          | TELEM2           |
| SG3  | J300      | UART7      | Yes          | User             |
| SG4  | J100      | USART3     | No           | GPS2             |
| SG5  | J100      | USART1     | No           | User             |
| SG6  | J200      | USART6     | No           | RC input         |
| SG7  | J300      | UART8      | No           | GPS              |

## Debug Port

The [SWD debug port](../debug/swd_debug.md) is exposed on connector **J200**.
A BOOT0 pin is also available on the same connector for firmware recovery.

## Further Information

- [3DR Control N1 documentation](https://docs.3dr.com/autopilots/control-n1)

# NWBlue Pro H757

<Badge type="tip" text="PX4 v1.18" />

:::warning
PX4 does not manufacture this (or any) autopilot. Contact the manufacturer for hardware support or compliance issues.
:::

The _NWBlue Pro H757_ is a 30 x 30 mm FPV flight controller built around a [CubePilot CubeNode H757](https://docs.cubepilot.org/user-guides/cubenode/pin-descriptions) module carrying an STM32H757 microcontroller.

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Key Features

- **MCU:** STM32H757 (32-bit Arm® Cortex®-M7 at 480 MHz, 2 MB Flash, 1 MB RAM), on a CubePilot CubeNode module
- **IMU:** InvenSense ICM-45686 (on SPI3, inside the CubeNode module)
- **Barometer:** Infineon DPS368 (on SPI3)
- **Magnetometer:** ST IIS2MDC (on I2C3, internal)
- **Storage:** microSD card (SDMMC2)
- **Interfaces:**
  - 6x UARTs (TEL1, TEL2, TEL3, GPS1, RC, and a spare camera backchannel)
  - 1x CAN (DroneCAN)
  - 1x external I2C (on the GPS connector, for an external compass) and 1x internal I2C (magnetometer)
  - 9x PWM outputs, [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) capable
  - USB
  - Buzzer output and RGB status LEDs
- **Power:** Powered from the ESC connector. Battery voltage is monitored through an onboard 21:1 divider; battery current comes from a current sensor wired to the ESC connector.

## Where to Buy

Check [NWBlue](https://nwblue.com/products/pro-h757-fpv-flight-controller) for availability.

## Manufacturer Documentation

Connector pinouts, wiring diagrams and mechanical drawings are in the [NWBlue Pro H757 documentation](https://docs.nwblue.com/nw-blue/products/autopilots/proh757/overview).

## Building Firmware

::: tip
Most users will not need to build this firmware.
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target from source:

```sh
make nwblue_pro-h757_default
```

## Serial Port Mapping

| UART   | Device     | PX4 Default | Connector | Pins (TX / RX) |
| ------ | ---------- | ----------- | --------- | -------------- |
| USART1 | /dev/ttyS0 | TEL3        | ESC       | — / PB7        |
| USART2 | /dev/ttyS1 | —           | VTX       | — / PA3        |
| UART4  | /dev/ttyS2 | RC          | RC        | PA0 / PB8      |
| USART6 | /dev/ttyS3 | TEL2        | VTX       | PC6 / PC7      |
| UART7  | /dev/ttyS4 | TEL1        | TELEM1    | PE8 / PE7      |
| UART8  | /dev/ttyS5 | GPS1        | GPS       | PE1 / PE0      |

No port has flow control.
USART1 and USART2 are receive-only: only the RX pin is broken out.

TEL3 is the ESC telemetry input and is configured for it by default ([DSHOT_TEL_CFG](../advanced_config/parameter_reference.md#DSHOT_TEL_CFG) = `103`).

## PWM Outputs

The board provides 9 PWM outputs, all of which support [DShot](../peripherals/dshot.md).

The outputs are split across 4 timer groups:

| Outputs    | Timer  |
| ---------- | ------ |
| 1, 2, 3, 4 | Timer1 |
| 5, 6       | Timer4 |
| 7, 8       | Timer3 |
| 9          | Timer2 |

All outputs within the same group must use the same protocol and update rate.

[Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) works on every output except output 6.
That output is TIM4_CH4, and the STM32H7 DMAMUX has no request line for it, so the capture DMA needed for the ESC response cannot be set up.
Use output 6 for plain DShot or PWM.

## OSD

There is no analog OSD chip on this board, so analog video cannot be overlaid.
An HD VTX can render the OSD itself over MSP DisplayPort: connect it to the VTX connector (USART6, TEL2) and set [MSP_OSD_CONFIG](../advanced_config/parameter_reference.md#MSP_OSD_CONFIG) to `102`.
See [OSD](../peripherals/osd.md) for details.

The port is not claimed by default, so it can be used for MAVLink or any other protocol instead.

## Battery Monitoring

The voltage divider is set by default ([BAT1_V_DIV](../advanced_config/parameter_reference.md#BAT1_V_DIV) = `21.0`).

Current sensing is fed from the ESC connector, so [BAT1_A_PER_V](../advanced_config/parameter_reference.md#BAT1_A_PER_V) depends on the current sensor in use and has to be [calibrated](../config/battery.md).

## Debug Port

There is no dedicated debug UART.
The system console and MAVLink both run over USB.

<a id="bootloader"></a>

## PX4 Bootloader Update

Boards that ship without the PX4 bootloader must have it flashed before PX4 firmware can be installed.
Download the [nwblue_pro-h757_bootloader.bin](https://github.com/PX4/PX4-Autopilot/blob/main/boards/nwblue/pro-h757/extras/nwblue_pro-h757_bootloader.bin) bootloader binary and follow the [DFU Bootloader Update](../advanced_config/bootloader_update_from_betaflight.md#dfu-bootloader-update) instructions.

Once the PX4 bootloader is flashed, firmware can be installed normally via _QGroundControl_.

# CyberCraft CyberX-v10

::: warning
PX4 does not manufacture this (or any) autopilot.
:::

## Introduction

The CyberX-v10 is an advanced autopilot manufactured by [CyberCraft International Limited](http://int.woocoo.vip/).

The autopilot is recommended for commercial system integration, but is also suitable for academic research and other applications.
It brings you ultimate performance, stability, and reliability in every aspect.

![CyberX-v10](../../assets/flight_controller/cyberx_v10/cyberx_v10_left.png)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Technical Specification

### Processors & Sensors

- FMU Processor: STM32H743IIK6
  - 32 Bit Arm® Cortex®-M7, 400MHz, 2MB Flash, 1MB RAM
- IO Processor: STM32F103
  - 32 Bit Arm® Cortex®-M3, 72MHz, 128KB Flash, 20KB SRAM
- On-board sensors
  - Accel/Gyro: BMI088
  - Accel/Gyro: ICM-42688-P
  - Accel/Gyro: ICM-20689
  - Mag: IST8310
  - Barometer: BMP581, ICP-20100

### Interfaces

- 16x PWM Servo Outputs (8 IO + 8 FMU)
- 1x Dedicated RC Input for Spektrum / DSM and SBUS
- 1x Analog/PWM RSSI Input
- 2x Telemetry Ports (`TEL1` and `TEL2`, with full flow control)
- 6x UART Ports (`TEL1`/`TEL2` with flow control) plus dedicated RC input
- 2x GPS Ports
  - 2x Basic GPS Ports (with I2C, `GPS1` and `GPS2`)
- 1x USB Port (Type-C)
- 1x Ethernet Port
  - Transformer application
  - 100Mbps
- 2x I2C Bus Ports
- 1x SPI Bus
  - 2x Chip Select Lines
- 2x CAN Ports
- 2x Power Input Ports
  - ADC Power Input
  - I2C Power Input
- 2x AD Ports
  - Analog Input (3.3V)
  - Analog Input (6.6V)
- 1x Dedicated Debug Port
  - FMU Debug
  - IO Debug
- 1x microSD card slot

### Dimensions

- Size: 83mm x 57mm x 15.1mm
- Weight: 72.4g

## Purchase Channels {#store}

Order from [CyberCraft International Limited](http://int.woocoo.vip/).

## Radio Control

A Radio Control (RC) system is required if you want to manually control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to select a compatible transmitter/receiver and then bind them so that they communicate (read the instructions that come with your specific transmitter/receiver).

Spektrum/DSM receivers connect to the DSM/SBUS RC input.
PPM or SBUS receivers connect to the RCIN input port.
CRSF receivers must be wired to a spare UART port on the flight controller.
You can then bind the transmitter and receiver together.

## Serial Port Mapping

| UART   | Device     | Port    |
| ------ | ---------- | ------- |
| USART1 | /dev/ttyS0 | `GPS1`  |
| USART2 | /dev/ttyS1 | `TEL1`  |
| USART3 | /dev/ttyS2 | `TEL2`  |
| UART4  | /dev/ttyS3 | `GPS2`  |
| UART5  | /dev/ttyS4 | `Uart5` |
| USART6 | /dev/ttyS5 | PX4IO   |
| UART7  | /dev/ttyS6 | EXT2    |
| UART8  | /dev/ttyS7 | RC      |

## PWM Output

The CyberX-v10 supports up to 16 PWM outputs.
The first 8 outputs (labelled `M1` to `M8`) are controlled by the dedicated STM32F103 IO controller.
The remaining 8 outputs (labelled `M9` to `M16`) are the "auxiliary" outputs directly attached to the STM32H743 FMU.

All 16 outputs support normal PWM.
The FMU outputs `M9` to `M14` support DShot.
The IO outputs `M1` to `M8`, and the FMU outputs `M15` and `M16` (no DMA), do not support DShot.
Outputs `M9` and `M11` support bi-directional DShot.

The 8 IO PWM outputs are in 3 groups:

- Outputs `M1`, `M2` in group 1
- Outputs `M3`, `M4` in group 2
- Outputs `M5`, `M6`, `M7`, `M8` in group 3

The 8 FMU PWM outputs are in 3 groups:

- Outputs `M9`, `M10`, `M11` and `M12` in group 1
- Outputs `M13` and `M14` in group 2
- Outputs `M15` and `M16` in group 3

Channels within the same group need to use the same output rate.
If any channel in a group uses DShot then all channels in the group need to use DShot.

## Electrical Data

- Voltage Ratings:
  - Max input voltage: 5.5V
  - USB Power Input: 4.75 ~ 5.25V
- Current Ratings:
  - `TEL1` and `TEL2` combined output current limiter: 1.5A
  - All other ports combined output current limiter: 1.5A

## Battery Monitoring

The board has connectors for 2 power monitors.

- `PWR1` -- ADC
- `PWR2` -- I2C

The board is configured by default for an analog power monitor on `PWR1`.
An INA228 I2C power monitor (address 0x40) on `PWR2` is also started by default.
Other I2C power monitors, such as the INA226 or INA238, must be started manually.

The default PDB included with the v10 is analog and must be connected to `PWR1`.

## Building Firmware

To [build PX4](../dev_setup/building_px4.md) for this target, execute:

```sh
make cyberx_v10_default
```

## Debug Port

The debug port uses a 6-pin JST GH (1.25mm pitch) connector.

| Pin     | Signal    | Voltage |
| ------- | --------- | ------- |
| 1 (red) | 5V+       | +5V     |
| 2 (blk) | FMU_SWDIO | +3.3V   |
| 3 (blk) | FMU_SWCLK | +3.3V   |
| 4 (blk) | IO_SWDIO  | +3.3V   |
| 5 (blk) | IO_SWCLK  | +3.3V   |
| 6 (blk) | GND       | GND     |

## Pinouts

![CyberX-v10 line drawing showing the connector end](../../assets/flight_controller/cyberx_v10/cyberx_v10_left.png)

![CyberX-v10 line drawing showing the servo rail end](../../assets/flight_controller/cyberx_v10/cyberx_v10_right.png)

![CyberX-v10 pinout diagram](../../assets/flight_controller/cyberx_v10/cyberx_v10_pinout.png)

## Supported Platforms / Airframes

Any multi-rotor/airplane/rover or boat that can be controlled using normal RC servos or Futaba SBUS servos.
The complete set of supported configurations can be found in the [Airframe Reference](../airframes/airframe_reference.md).

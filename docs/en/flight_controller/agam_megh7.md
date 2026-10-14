# Agam MegH7

<Badge type="tip" text="main (PX4 v2.0)" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.agamrobotics.com/) for hardware support or compliance issues.
:::

The [Agam MegH7](https://www.agamrobotics.com/agammegh7) is an STM32H743 flight controller from [Agam Robotics](https://www.agamrobotics.com/), in a 30.5 x 30.5 mm form factor, with an onboard IMU, barometer, analog OSD, microSD slot, CAN bus and seven UARTs.
This page describes hardware revision v1.2.

The board has no internal magnetometer, and provides two external I2C buses for a GPS/compass module or other peripherals.

![Agam MegH7](../../assets/flight_controller/agam_megh7/agam_megh7.png)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Specifications {#specifications}

### Processor {#processor}

- **Main FMU processor:** [STM32H743VI](https://www.st.com/en/microcontrollers-microprocessors/stm32h743vi.html) (32-bit Arm Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM)

### Sensors {#sensors}

- **IMU:** [ICM-45686](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-45686/) (SPI2)
- **Barometer:** [BMP390](https://www.bosch-sensortec.com/en/products/environmental-sensors/pressure-sensors/bmp390/) (I2C2)
- **Magnetometer:** none (connect an external compass to `GPS` or `I2C4`)
- **OSD:** AT7456E (SPI3)

### Interfaces {#interfaces}

- **PWM outputs:** 8, plus an addressable LED strip output on `B/LD`
- **Serial ports:** 7 (`GPS`, `TELEM`, `RX`, `DIGI VTX`, the `T3`/`R3` and `T4`/`R4` pads, and ESC telemetry on `ESC`)
- **I2C buses:** 3 (I2C2 internal; I2C1 on the `GPS` connector and I2C4 on the `I2C4` connector, both external)
- **SPI buses:** 4 (SPI1 reserved for the optional dataflash, SPI2 and SPI3 internal, SPI4 external on the `SPI4` connector with 1 chip select and 1 data-ready line)
- **CAN buses:** 1
- **USB:** Type-C
- **RC input:** serial RC or PPM on `RX` (USART6)
- **Parameter storage:** microSD card (a card is required)
- **SD card:** microSD slot
- **ESC connector:** JST-SH 1.0 mm 8-pin, carrying `M1`-`M4` (single or 4-in-1 ESCs); `M5`-`M8` are on solder pads
- **HD video connector:** JST-SH 1.0 mm 6-pin, for systems such as Caddx Vista and DJI Air Unit
- **Dataflash:** optional onboard W25Q128 on SPI1 (assembly option, not fitted on every board, not used by PX4)

### Electrical Data {#electrical_data}

- **Input voltage:** 2S-8S
- **BEC outputs:** 5 V, 3 A continuous; 10 V, 3 A continuous
- **Power monitoring:** 1 analog battery voltage and current input

### Mechanical Data {#mechanical_data}

- **Dimensions:** 37.5 x 39 x 8.8 mm
- **Weight:** 10 g
- **Mounting hole spacing:** 30.5 x 30.5 mm (Φ4 mm holes, with Φ3 mm grommets)

## Where to Buy {#store}

The board can be bought from:

- [Agam Robotics](https://www.agamrobotics.com/shop)

## Connectors

![Agam MegH7 connector side, with the ESC, GPS, TELEM, RX, CAN, DIGI VTX, SPI4, I2C4 and B/LD connectors](../../assets/flight_controller/agam_megh7/agam_megh7_connectors.jpg)

All the plug-and-play connectors are on the bottom of the board.
The `GPS`, `TELEM`, `CAN` and `RX` connectors are JST-GH 1.25 mm.
The `ESC`, `DIGI VTX`, `SPI4`, `I2C4` and `B/LD` connectors are JST-SH 1.0 mm.
Signal pins are 3.3 V.

| Connector  | Type         | Function                                          |
| ---------- | ------------ | ------------------------------------------------- |
| USB Type-C |              | Firmware and configuration                        |
| microSD    |              | Logging and parameter storage                     |
| ESC        | JST-SH 8-pin | Motor outputs, current sense, ESC telemetry       |
| GPS        | JST-GH 6-pin | GPS and I2C1                                      |
| TELEM      | JST-GH 6-pin | Telemetry, with flow control                      |
| RX         | JST-GH 4-pin | Serial RC or PPM input                            |
| CAN        | JST-GH 4-pin | CAN bus                                           |
| DIGI VTX   | JST-SH 6-pin | HD video system, with 10 V supply                 |
| SPI4       | JST-SH 8-pin | External SPI, with interrupt and data-ready lines |
| I2C4       | JST-SH 4-pin | External I2C                                      |
| B/LD       | JST-SH 4-pin | Passive buzzer and addressable LED strip          |

The top of the board also breaks out solder pads: the motor outputs `M1`-`M8`, the UART pads `T1`/`R1` through `T8`/`R8`, the I2C pads `SC1`/`SD1` and `SC4`/`SD4`, the analog camera and VTX groups, the analog inputs `RSS`, `A1` and `A2`, the spare user IO pads `P2`, `P3`, `C13`, `C14` and `C15`, and power and ground pads.
CAN, SPI4 and the UART7 flow control lines are on their connectors only.
See [Solder Pads](#solder_pads) for the pinout.

::: info
The buzzer and LED strip outputs are powered from the 5 V BEC, so neither is active when the board is powered from USB alone.
:::

PX4 drives only the first LED of a strip connected to `B/LD`, and uses it as the RGB status LED.

The 10 V supply on `DIGI VTX` pin 1 and on the analog VTX `10V` pad is switched by the processor (PE5).
PX4 turns it on at boot.

## Pinouts {#pinouts}

### ESC

| Pin | Signal                   | MCU pin |
| --- | ------------------------ | ------- |
| 1   | VBAT                     |         |
| 2   | GND                      |         |
| 3   | Current sense            | PC1     |
| 4   | ESC telemetry (UART8 RX) | PE0     |
| 5   | M1                       | PB0     |
| 6   | M2                       | PB1     |
| 7   | M3                       | PA0     |
| 8   | M4                       | PA1     |

### GPS

| Pin | Signal    | MCU pin |
| --- | --------- | ------- |
| 1   | +5V       |         |
| 2   | USART1 TX | PA9     |
| 3   | USART1 RX | PA10    |
| 4   | I2C1 SCL  | PB6     |
| 5   | I2C1 SDA  | PB7     |
| 6   | GND       |         |

### TELEM

| Pin | Signal    | MCU pin |
| --- | --------- | ------- |
| 1   | +5V       |         |
| 2   | UART7 TX  | PE8     |
| 3   | UART7 RX  | PE7     |
| 4   | UART7 CTS | PE10    |
| 5   | UART7 RTS | PE9     |
| 6   | GND       |         |

### RX

| Pin | Signal          | MCU pin |
| --- | --------------- | ------- |
| 1   | USART6 TX       | PC6     |
| 2   | USART6 RX / PPM | PC7     |
| 3   | +5V             |         |
| 4   | GND             |         |

### CAN {#can_pinout}

| Pin | Signal | MCU pin                       |
| --- | ------ | ----------------------------- |
| 1   | +5V    |                               |
| 2   | CAN H  | CAN1 via transceiver (PD1 TX) |
| 3   | CAN L  | CAN1 via transceiver (PD0 RX) |
| 4   | GND    |                               |

### DIGI VTX

| Pin | Signal                             | MCU pin |
| --- | ---------------------------------- | ------- |
| 1   | +10V (switched by PE5)             |         |
| 2   | GND                                |         |
| 3   | USART2 TX                          | PD5     |
| 4   | USART2 RX                          | PD6     |
| 5   | GND                                |         |
| 6   | USART3 RX (SBUS from the air unit) | PD9     |

### SPI4

| Pin | Signal | MCU pin |
| --- | ------ | ------- |
| 1   | +5V    |         |
| 2   | SCK    | PE12    |
| 3   | MISO   | PE13    |
| 4   | MOSI   | PE14    |
| 5   | CS     | PB12    |
| 6   | DRDY   | PD4     |
| 7   | EXTI   | PD3     |
| 8   | GND    |         |

PX4 does not configure pin 7 (`EXTI`): the SPI bus configuration has a slot for the data-ready line on pin 6 but none for a second interrupt, so PD3 is left as an unused GPIO.

### I2C4

| Pin | Signal   | MCU pin |
| --- | -------- | ------- |
| 1   | +5V      |         |
| 2   | I2C4 SCL | PD12    |
| 3   | I2C4 SDA | PD13    |
| 4   | GND      |         |

### B/LD

| Pin | Signal                                    | MCU pin |
| --- | ----------------------------------------- | ------- |
| 1   | +5V                                       |         |
| 2   | Passive buzzer, switched low side (`BZ-`) | PA15    |
| 3   | LED strip data                            | PA8     |
| 4   | GND                                       |         |

### Solder Pads {#solder_pads}

The UART pads are marked `Tn` and `Rn`, where `n` is the port number: `Tn` is the transmit pin of UART `n` and `Rn` the receive pin.

| Pad           | Signal              | MCU pin     |
| ------------- | ------------------- | ----------- |
| `M1` - `M4`   | Motor outputs 1-4   | PB0, PB1, PA0, PA1 |
| `M5` - `M8`   | Motor outputs 5-8   | PA2, PA3, PD14, PD15 |
| `LED`         | Addressable LED strip data | PA8  |
| `BZ-`         | Buzzer, switched low side | PA15  |
| `T1` / `R1`   | USART1 TX / RX      | PA9 / PA10  |
| `T2` / `R2`   | USART2 TX / RX      | PD5 / PD6   |
| `T3` / `R3`   | USART3 TX / RX      | PD8 / PD9   |
| `T4` / `R4`   | UART4 TX / RX       | PB9 / PB8   |
| `T6` / `R6`   | USART6 TX / RX      | PC6 / PC7   |
| `T7` / `R7`   | UART7 TX / RX       | PE8 / PE7   |
| `T8` / `R8`   | UART8 TX / RX       | PE1 / PE0   |
| `SC1` / `SD1` | I2C1 SCL / SDA      | PB6 / PB7   |
| `SC4` / `SD4` | I2C4 SCL / SDA      | PD12 / PD13 |
| `CR`          | Battery current sense | PC1       |
| `D` / `C`     | SWDIO / SWCLK       | PA13 / PA14 |
| `P2`, `P3`    | Spare user IO, not configured by PX4 | PE15, PE6 |
| `C13`, `C14`, `C15` | Spare user IO, not configured by PX4 | PC13, PC14, PC15 |

`CAM`, `CC` and `VTX` are the analog video pads and do not connect to the processor; see [OSD](#osd).
The remaining pads are power and ground.

### Analog Inputs

The `A1` and `A2` pads connect directly to the processor, with no voltage divider, so their input must not exceed 3.3 V.

| Pad   | MCU pin | Use in PX4                                                                                                                                                  |
| ----- | ------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RSS` | PC5     | Analog RSSI input. Used only if the receiver does not report RSSI and `RC_RSSI_PWM_CHAN` is not set. PX4 starts using it once the input has exceeded 2.5 V. |
| `A1`  | PC4     | Analog airspeed sensor, enabled by setting [SENS_DPRES_ANSC](../advanced_config/parameter_reference.md#SENS_DPRES_ANSC)                                     |
| `A2`  | PA4     | Not used                                                                                                                                                    |

## Block Diagram {#block_diagram}

The block diagram below shows the main components and how they connect to the STM32H743VI.

![Agam MegH7 block diagram: the STM32H743VI with its IMU, barometer, OSD, optional dataflash and microSD card, the external connectors and their buses, the motor timer groups, the solder pads, and the power tree from the battery and USB through the 5 V and 10 V BECs to the 3.3 V rail](../../assets/flight_controller/agam_megh7/block_diagram.svg "Agam MegH7 Block Diagram")

## Power {#power}

The battery connects either through the `ESC` connector (pin 1 `VBAT`, pin 2 `GND`) or to the `BAT` and `G` pads.
The board accepts 2S-8S LiPo, and its BECs supply 5 V and 10 V at up to 3 A each.

Battery voltage and current are measured by analog inputs, with current from the ESC's current sensor on `ESC` pin 3.
PX4 sets [BAT1_V_DIV](../advanced_config/parameter_reference.md#BAT1_V_DIV) to `11` and [BAT1_A_PER_V](../advanced_config/parameter_reference.md#BAT1_A_PER_V) to `40` by default.
Set `BAT1_A_PER_V` to match your ESC's current sensor, and see [Battery Estimation Tuning](../config/battery.md) to calibrate both.

## Voltage Ratings {#voltage_ratings}

The board is powered either from the battery, through `ESC` pin 1 or the `BAT` and `G` pads, or from USB.
Both BECs are step-down SMPS regulators running directly off the battery, and the 3.3 V rail comes from an LDO on the 5 V rail.

**Normal Operation Maximum Ratings**

Under these conditions the board powers normally:

1. Battery input: 5.5 V to 36 V, with 2S to 8S LiPo recommended (6.6 V to 33.6 V)
2. USB input: 4.75 V to 5.25 V

**Absolute Maximum Ratings**

Under these conditions the board is not damaged, but may not operate:

1. Battery input: 40 V, the input absolute maximum of the BEC regulators
2. USB input: 6 V, the input absolute maximum of the LDO

::: info
The BECs are step-down only, with a maximum duty cycle of 92%, so each rail needs headroom above its output voltage.
The 5 V rail needs roughly 5.5 V in, which is the lower figure above.
A 2S pack is flat before it reaches that, so the pack sets the practical floor, not the regulator.
:::

::: warning
The outputs fed from the 10 V BEC (`DIGI VTX` pin 1, the switched `10V` pad in the analog VTX group, and the unswitched `10V` pad) have a narrower input range than the board itself.
That rail needs roughly 11 V in, so it is unavailable on 2S and marginal on 3S.
The battery figures above apply to a board with nothing on those outputs; with an HD air unit or an analog VTX fitted, use 4S or above.
:::

## PWM Outputs {#pwm_outputs}

`M1`-`M4` are on the `ESC` connector, and all of `M1`-`M8` are also exposed as solder pads.
All outputs support [DShot](../peripherals/dshot.md).
[Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) works on `M1`-`M7` only: use `M8` for plain DShot or PWM.

Outputs that share a timer form a group, and must use the same output rate and protocol (PWM or DShot).

| Output | MCU pin | Timer group |
| ------ | ------- | ----------- |
| M1     | PB0     | 1 (TIM3)    |
| M2     | PB1     | 1 (TIM3)    |
| M3     | PA0     | 2 (TIM5)    |
| M4     | PA1     | 2 (TIM5)    |
| M5     | PA2     | 2 (TIM5)    |
| M6     | PA3     | 2 (TIM5)    |
| M7     | PD14    | 3 (TIM4)    |
| M8     | PD15    | 3 (TIM4)    |

## SD Card {#sd_card}

The board has a microSD card slot on the top of the board.
PX4 stores both flight logs and parameters on the card, so a card must be fitted: without one, parameter changes are lost at reboot.
See [SD Cards](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory) for recommended cards.

## Serial Port Mapping {#serial_port_mapping}

| UART   | Device       | Port                                                   | Flow Control |
| ------ | ------------ | ------------------------------------------------------ | ------------ |
| USART1 | `/dev/ttyS0` | GPS1 (`GPS` connector)                                 | No           |
| USART2 | `/dev/ttyS1` | TELEM2 (`DIGI VTX` connector)                          | No           |
| USART3 | `/dev/ttyS2` | TELEM3 (`T3` / `R3` pads, RX also on `DIGI VTX` pin 6) | No           |
| UART4  | `/dev/ttyS3` | TELEM4 (`T4` / `R4` pads)                              | No           |
| USART6 | `/dev/ttyS4` | RC input (`RX` connector)                              | No           |
| UART7  | `/dev/ttyS5` | TELEM1 (`TELEM` connector)                             | Yes          |
| UART8  | `/dev/ttyS6` | EXT2 (ESC telemetry on `ESC` pin 4, `T8` / `R8` pads)  | No           |

::: warning
`rc.board_extras` always starts `dshot telemetry` on UART8, because that is where the `ESC` connector's telemetry pin goes.
UART8 is also offered as `EXT2` in the serial port configuration, so assigning `EXT2` to another driver gives two users of the same port and neither will work reliably.
Leave `EXT2` unassigned unless you are not using ESC telemetry.
:::

## Assembly {#assembly}

### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to manually control your vehicle (PX4 does not require a radio system for autonomous flight modes).

Connect the RC receiver to the `RX` connector (JST-GH 4-pin, USART6).
PX4 detects SBUS, DSM, ST24, SUMD, CRSF, GHST and PPM receivers on this port automatically.
Both USART6 lines are on the connector, so ExpressLRS and Crossfire receivers also get telemetry back to the handset.
PPM shares the USART6 RX pin (`RX` pin 2) with the serial input, so only one of the two can be connected at a time.
PWM receivers (one wire per channel) must be connected through a PPM encoder.
The board cannot power-cycle Spektrum receivers for binding.

::: tip
To use SBUS from a DJI or Caddx air unit on `DIGI VTX` pin 6 instead, set [RC_PORT_CONFIG](../advanced_config/parameter_reference.md#RC_PORT_CONFIG) to `TELEM 3` (see [DIGI VTX](#digi-vtx)).
:::

See [Radio Control Systems](../getting_started/rc_transmitter_receiver.md) for more information about selecting a radio system and receiver.

### GPS & Compass {#gps_compass}

Connect a GPS/compass module to the `GPS` connector (GPS1 on USART1, compass on I2C1).
The board has no internal magnetometer, so an external compass is needed for most flight modes.
PX4 also probes for an external compass on `I2C4`.
See [Mounting a Compass](../assembly/mount_gps_compass.md) for placement.

### Telemetry Radios (Optional) {#telemetry}

Connect a telemetry radio to the `TELEM` connector (TELEM1, with flow control).
MAVLink runs on TELEM1 by default, so no configuration is needed.
See [Telemetry Radios](../telemetry/index.md).

### CAN {#can}

The `CAN` connector provides one CAN bus for [DroneCAN](../dronecan/index.md) peripherals.
DroneCAN is disabled by default: enable it with [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE).

### OSD {#osd}

The onboard AT7456E analog OSD overlays flight data on the analog video passing from the camera pad (`CAM`) to the VTX pad (`VTX`).
It is disabled by default: set [OSD_ATXXXX_CFG](../advanced_config/parameter_reference.md#OSD_ATXXXX_CFG) to `1` (NTSC) or `2` (PAL) to match your camera, and reboot.

For an HD air unit on the `DIGI VTX` connector, the firmware also includes the MSP OSD driver.
Set [MSP_OSD_CONFIG](../advanced_config/parameter_reference.md#MSP_OSD_CONFIG) to `TELEM 2` to run it on USART2, which is the connector's serial port.
The two OSDs are independent and can run at the same time, one on the analog video path and one on the air unit.

### VTX {#vtx}

SmartAudio and Tramp video transmitters can be controlled over a serial port.
Set [VTX_SER_CFG](../advanced_config/parameter_reference.md#VTX_SER_CFG) to the port the transmitter is wired to, and [VTX_PROTOCOL](../advanced_config/parameter_reference.md#VTX_PROTOCOL) to match it.
The port is used in single-wire mode, so wire the transmitter to the `T` pad of the pair and leave `R` unconnected.

Band, channel and power are then set with [VTX_BAND](../advanced_config/parameter_reference.md#VTX_BAND), [VTX_CHANNEL](../advanced_config/parameter_reference.md#VTX_CHANNEL) and [VTX_POWER](../advanced_config/parameter_reference.md#VTX_POWER), or from an RC switch with [VTX_MAP_CONFIG](../advanced_config/parameter_reference.md#VTX_MAP_CONFIG).

## PX4 Bootloader Update {#bootloader}

The board ships with non-PX4 firmware pre-installed.
Before PX4 firmware can be installed, the _PX4 bootloader_ must be flashed.
Download the [agam_megh7_bootloader.bin](https://github.com/PX4/PX4-Autopilot/raw/main/boards/agam/megh7/extras/agam_megh7_bootloader.bin) bootloader binary and read [this page](../advanced_config/bootloader_update_from_betaflight.md) for flashing instructions.

## Building Firmware {#building_firmware}

::: tip
From PX4 v2.0, most users will not need to build this firmware.
It will be pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make agam_megh7_default
```

## Installing PX4 Firmware

Firmware can be installed in any of the normal ways:

- Build and upload the source:

  ```sh
  make agam_megh7_default upload
  ```

- [Load the firmware](../config/firmware.md) using _QGroundControl_.
  You can use either pre-built firmware or your own custom firmware.

## PX4 Configuration

In addition to the [basic configuration](../config/index.md), the following parameters are important:

| Parameter                                                            | Setting                                                                                                              |
| -------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG) | Set to `0` by default, since the board has no internal magnetometer. Set it to `1` after attaching an external compass. |

## Debug Port {#debug_port}

### System Console

There is no [system console](../debug/system_console.md) UART on this board.
The NSH shell is available over USB through the [MAVLink Shell](../debug/mavlink_shell.md), for example the **MAVLink Console** in _QGroundControl_.

### SWD

The [SWD interface](../debug/swd_debug.md) (JTAG) pins are exposed as solder pads on the top of the board, next to the boxed `SW` label.
The pads do not follow either of the Pixhawk Debug connector standards, so wires must be soldered to them:

- `SWDIO`: pad marked `D`
- `SWCLK`: pad marked `C`
- `GND`: any pad marked `G`

The board has no 3.3 V solder pad: the `3V3` marking on the right-hand edge is a power rail indicator LED, not a pad.

## Further Information {#further_information}

- [Agam MegH7 product page](https://www.agamrobotics.com/agammegh7)

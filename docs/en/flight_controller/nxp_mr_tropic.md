# NXP MR-VMU-TROPIC

<Badge type="tip" text="PX4 v1.17" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.nxp.com) for hardware support or compliance issues.
:::

The [MR-VMU-TROPIC](https://github.com/NXP-Robotics/MR-VMU-TROPIC) is NXP's open source reference design of a versatile VMU that can be used for drones, rovers, boats, and more.
As a reference/evaluation design, it is intended to be replicated, modified or integrated in production volume by others.
Since it is a reference design, this particular board is not produced in high volume.
Similar variants will be available from NXP licensees.

The board combines an NXP i.MX RT1064 with sensor technology from Bosch® and InvenSense®, with the FMU, IMUs and other sensors on one compact board.
It is suitable for controlling any autonomous vehicle, in both academic and commercial applications.
The high-performance, low-noise IMUs are designed for better stabilisation.

Besides the standard interfaces, it has 100BASE-T1 (two-wire) automotive Ethernet and 8 PWM outputs, all with [DShot](../peripherals/dshot.md).

The MR-VMU-TROPIC reference design suits developers at corporate research labs, startups, academia and commercial companies who want to try out 100BASE-T1 (two-wire) automotive Ethernet.

![MR-VMU-TROPIC side view](../../assets/flight_controller/nxp_mr_tropic/mr_vmu_tropic_side.jpg)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Specifications {#specifications}

### Processor {#processor}

- **Main FMU processor:** [NXP i.MX RT1064](https://www.nxp.com/products/i.MX-RT1064) Crossover MCU (32-bit Arm® Cortex®-M7 at 600 MHz, 1 MB on-chip SRAM with up to 512 KB configurable as tightly coupled memory (TCM), 4 MB on-chip flash)

### Sensors {#sensors}

- **IMU:** TDK InvenSense ICM-45686 (SPI3), Bosch BMI088 (SPI4).
  Hardware versions with a TDK InvenSense ICM-42686-P in place of the ICM-45686 are also supported.
- **Barometer:** Bosch BMP390 (I2C4)
- **Magnetometer:** Bosch BMM350 (I2C4)

### Interfaces {#interfaces}

- **PWM outputs:** 8, all with DShot and bidirectional DShot
- **Serial ports:** 5 (`GPS`, `TELEM1`, `TELEM2`, `AUX`, `RC`), plus the debug console and an ESC telemetry input; `TELEM1` and `TELEM2` have full flow control
- **I2C buses:** 3 (I2C4 internal; I2C1 and I2C3 external), with 1 external port (`I2C`) for additional sensors and payloads
- **SPI buses:** 2 internal (SPI3, SPI4), plus 1 external (`SPI (OSD)`) with 1 chip select line, 1 data-ready line and 1 reset line, not used by the default firmware
- **CAN buses:** 1 (CAN FD, NXP TJA1462ATK transceiver, 120 Ω termination resistor on board)
- **Ethernet:** 100BASE-T1 (two-wire), 100 Mbps, for transformerless applications and high-speed communication with mission computers (NXP TJA1103 PHY)
- **USB:** USB-C, high speed (480 Mbit/s)
- **RC input:** Yes, dedicated input for CRSF/ELRS
- **Parameter storage:** flash
- **SD card:** microSD slot

### Electrical Data {#electrical_data}

- **Input voltage:** 5.5 V maximum
- **USB power input:** 5 V
- **Current draw:** typically 0.25–0.35 A at 5 V
- **Power monitoring:** 1 digital input on `POWER` (INA226, INA228 or INA238, detected automatically); no analog input

### Mechanical Data {#mechanical_data}

- **Dimensions:** 46 x 45 x 1.57 mm (standard baseboard)
- **Weight:** 12 g (standard baseboard)
- **Operating and storage temperature:** 0–85 °C

## Pinouts {#pinouts}

The `TELEM1`, `TELEM2`, `AUX`, `GPS`, `I2C`, `CAN` and `DEBUG` connectors follow the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) (DS-009).
The ESC headers (`PWM_A` and `PWM_B`) follow the [Betaflight ESC connector standard](https://betaflight.com/docs/development/manufacturer/connector-standard#standard-esc-pin-configuration).

The pinout of every connector is given in NXP's [MR-VMU-TROPIC hardware reference](https://github.com/NXP-Robotics/MR-VMU-TROPIC#board-overview) (also as a [PDF](https://github.com/NXP-Robotics/MR-VMU-TROPIC/blob/main/MR-VMU-Tropic-hardware-reference.pdf) and a one-page [cheat sheet](https://github.com/NXP-Robotics/MR-VMU-TROPIC/blob/main/MR-VMU-Tropic-cheatsheet.pdf)).
The schematic is [SPF-94468_F.pdf](https://github.com/NXP-Robotics/MR-VMU-TROPIC/blob/main/schematics_and_production_files/tropic_rev_F/SPF-94468_F.pdf) (rev F).

### Top Side Connectors

| Connector | Label       | Function                                                            | Type                | Standard            |
| --------- | ----------- | ------------------------------------------------------------------- | ------------------- | ------------------- |
| `J2`      | `POWER`     | Power module input: 5 V and battery monitor I2C                     | Molex CLIK-Mate 1x6 | —                   |
| `J3`      | `DEBUG`     | SWD and console                                                     | JST-SH 1x10         | DS-009 (Debug Full) |
| `J4`      | `GPS`       | GPS, compass, safety switch and buzzer                              | JST-GH 1x10         | DS-009              |
| `J5`      | `CAN`       | CAN FD, CAN-SIC (Signal Improvement) transceiver, 120 Ω termination | JST-GH 1x4          | DS-009              |
| `J6`      | `SPI (OSD)` | External SPI port for an OSD board                                  | JST-GH 1x8          | —                   |
| `J7`      | `AUX`       | Spare serial port                                                   | JST-GH 1x6          | DS-009              |
| `J8`      | `TELEM1`    | Telemetry port 1, with flow control                                 | JST-GH 1x6          | DS-009              |
| `J9`      | `TELEM2`    | Telemetry port 2, with flow control                                 | JST-GH 1x6          | DS-009              |
| `J10`     | `I2C`       | External I2C bus, shared with `GPS`                                 | JST-GH 1x4          | DS-009              |
| `J11`     | `RC`        | Serial RC receiver                                                  | JST-GH 1x4          | —                   |
| `P1`      | `ETH`       | 100BASE-T1 (two-wire) 100 Mbps Ethernet                             | JST-GH 1x2          | —                   |

![MR-VMU-TROPIC top-side connectors](../../assets/flight_controller/nxp_mr_tropic/overview_top.svg)

### Bottom Side Connectors

| Connector | Label     | Function                       | Type               | Standard       |
| --------- | --------- | ------------------------------ | ------------------ | -------------- |
| `J1`      | `USB`     | USB 2.0 device port            | USB-C              | —              |
| `J12`     | `PWM_B`   | Motor outputs 5–8 (ESC header) | JST-SH 1x8         | Betaflight ESC |
| `J13`     | `PWM_A`   | Motor outputs 1–4 (ESC header) | JST-SH 1x8         | Betaflight ESC |
| `CN1`     | `MICROSD` | microSD card slot              | microSD, push-push | —              |

![MR-VMU-TROPIC bottom-side connectors](../../assets/flight_controller/nxp_mr_tropic/overview_bottom.svg)

## Power {#power}

The board has two power rails: **POWER1** and **USB**.
The **POWER1** rail is supplied via the `J2` connector (the 6-circuit [2.00 mm pitch CLIK-Mate wire-to-board PCB receptacle](https://www.molex.com/en-us/products/part-detail/5024430670)) and directly powers the following ports: I2C (`J10`), GPS (`J4`), AUX (`J7`), CAN (`J5`), TELEM1 (`J8`), TELEM2 (`J9`), RC (`J11`) and SPI-OSD (`J6`).

::: warning
Feed only 5 V into `POWER` (`J2`): there is no regulator behind this connector, and a higher voltage damages the board.
Every other 5 V pin on the board is an output: never feed power into it.
Any device connected to these ports must be compatible with the **POWER1** voltage rail.
:::

Battery monitoring is digital, over I2C (I2C3 on the `POWER` port).
An INA226, INA228 or INA238 power monitor is detected automatically.
To start a specific one instead, set [SENS_EN_INA226](../advanced_config/parameter_reference.md#SENS_EN_INA226), [SENS_EN_INA228](../advanced_config/parameter_reference.md#SENS_EN_INA228) or [SENS_EN_INA238](../advanced_config/parameter_reference.md#SENS_EN_INA238) to `1`.

::: info
Analog battery monitoring via an ADC is not supported on this board.
:::

See [Battery and Power Module Setup](../config/battery.md) for configuration.

## Voltage Ratings {#voltage_ratings}

### Normal Operation Maximum Ratings

| Rail       | Minimum | Maximum |
| ---------- | ------- | ------- |
| **USB**    | 4.5 V   | 5.5 V   |
| **POWER1** | 4.9 V   | 5.5 V   |

### Absolute Maximum Ratings

| Rail       | Minimum | Maximum |
| ---------- | ------- | ------- |
| **USB**    | 0 V     | 6 V     |
| **POWER1** | 0 V     | 6 V     |

## PWM Outputs {#pwm_outputs}

The board has 8 PWM outputs.
All of them support [DShot](../peripherals/dshot.md) and [bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The outputs are divided into 4 groups:

- Outputs 1-2
- Outputs 3-4
- Outputs 5-6
- Outputs 7-8

All outputs within the same group must use the same output protocol and rate.

Outputs 1–4 are on `PWM_A` (`J13`) and outputs 5–8 on `PWM_B` (`J12`).
Both headers follow the [Betaflight ESC connector standard](https://betaflight.com/docs/development/manufacturer/connector-standard#standard-esc-pin-configuration), so a 4-in-1 ESC with a standard 8-pin cable fits.
The headers have no 5 V pin, and their battery voltage and current pins (1 and 3) are not connected: the board doesn't use the ESC's BEC or read battery voltage or current from the ESC.

Serial ESC telemetry is received on pin 4 of both headers (`/dev/ttyS5`, ESC ONEWIRE), and is enabled by default.

## Ethernet {#ethernet}

The `P1` port provides 100BASE-T1 (two-wire) Ethernet.
The other end needs a 100BASE-T1 device or a media converter.
MAVLink runs on Ethernet by default, using UDP port 14550 ([MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG) is set to `Ethernet`).

See [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md) for more information.

## SD Card (Optional) {#sd_card}

The microSD card slot (`CN1`) is on the bottom of the board.
See [SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).

## Serial Port Mapping {#serial_port_mapping}

| UART    | Device     | Port        | Flow Control |
| ------- | ---------- | ----------- | ------------ |
| LPUART6 | /dev/ttyS0 | Debug       | No           |
| LPUART2 | /dev/ttyS1 | `GPS`       | No           |
| LPUART3 | /dev/ttyS2 | `TELEM1`    | Yes          |
| LPUART4 | /dev/ttyS3 | `AUX`       | No           |
| LPUART5 | /dev/ttyS4 | `TELEM2`    | Yes          |
| LPUART7 | /dev/ttyS5 | ESC ONEWIRE | No           |
| LPUART8 | /dev/ttyS6 | `RC`        | No           |

::: info
The `AUX` port is `TELEM 3` in the [serial port configuration](../peripherals/serial_configuration.md).
:::

## Assembly {#assembly}

### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).
You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate.

Connect the receiver to the `RC` port (`J11`), which is the FMU serial port `/dev/ttyS6`.
CRSF receivers are enabled on this port by default, with telemetry.

The default firmware also includes DSM, GHST and S.BUS drivers.
To use one of these protocols, set [RC_CRSF_PRT_CFG](../advanced_config/parameter_reference.md#RC_CRSF_PRT_CFG) to `Disabled`, and then set [RC_DSM_PRT_CFG](../advanced_config/parameter_reference.md#RC_DSM_PRT_CFG), [RC_GHST_PRT_CFG](../advanced_config/parameter_reference.md#RC_GHST_PRT_CFG) or [RC_SBUS_PRT_CFG](../advanced_config/parameter_reference.md#RC_SBUS_PRT_CFG) to `Radio Controller`.
Only one protocol can be active on a port.

### GPS & Compass {#gps_compass}

Connect the GPS module to the `GPS` port (`J4`), which carries the GPS serial port (`/dev/ttyS1`) and the I2C1 bus for an external compass.
The port also carries the safety switch and its LED, and the buzzer.
The `I2C` port (`J10`) is on the same I2C bus, so devices on the two ports share its addresses.

See [Mounting the GPS/Compass](../assembly/mount_gps_compass.md) for more information.

### Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) can be connected to `TELEM1` (`J8`) or `TELEM2` (`J9`).
`TELEM1` needs no further configuration.

### CAN {#can}

Connect [DroneCAN](../dronecan/index.md) peripherals to the `CAN` port (`J5`).
DroneCAN is disabled by default; enable it with [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE).

## PX4 Bootloader Update {#bootloader}

NXP's [PX4 setup guide](https://github.com/NXP-Robotics/MR-VMU-TROPIC/blob/main/PX4_Autopilot.md) says that a board that doesn't come with the PX4 bootloader needs it flashed before PX4 can be installed.

To flash it over USB, follow [Bootloader Update Pixhawk V6X-RT via USB](../advanced_config/bootloader_update_v6xrt.md), with these changes:

- Use `nxp_mr-tropic_bootloader.bin` (see [Building Firmware](#building_firmware)).
- Select the MIMXRT1064 in place of the MIMXRT1170.
- Skip the _Boot Memory Configuration_ step.
- To start the serial downloader, hold the `BOOT` button (`SW2`) while powering the board.

You can also flash `nxp_mr-tropic_bootloader.bin` at address `0x70000000` with an SWD debugger on the `DEBUG` port.

## Building Firmware {#building_firmware}

::: tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make nxp_mr-tropic_default
```

To build the bootloader:

```sh
make nxp_mr-tropic_bootloader
```

## Debug Port {#debug_port}

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the FMU debug port (`J3`).

The pinouts and connector comply with the [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) interface defined in the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) (JST SM10B connector).

| Pin      | Signal           | Volt  |
| -------- | ---------------- | ----- |
| 1 (red)  | `Vtref`          | +3.3V |
| 2 (blk)  | Console TX (OUT) | +3.3V |
| 3 (blk)  | Console RX (IN)  | +3.3V |
| 4 (blk)  | `SWDIO`          | +3.3V |
| 5 (blk)  | `SWCLK`          | +3.3V |
| 6 (blk)  | `SWO`            | +3.3V |
| 7 (blk)  | NC               | -     |
| 8 (blk)  | NC               | -     |
| 9 (blk)  | nRST             | +3.3V |
| 10 (blk) | `GND`            | GND   |

## Further Information {#further_information}

- [MR-VMU-TROPIC hardware reference](https://github.com/NXP-Robotics/MR-VMU-TROPIC) (NXP Robotics GitHub)
- [MR-VMU-TROPIC PX4 setup guide](https://github.com/NXP-Robotics/MR-VMU-TROPIC/blob/main/PX4_Autopilot.md) (NXP Robotics GitHub)
- [MR-VMU-TROPIC schematic, rev F](https://github.com/NXP-Robotics/MR-VMU-TROPIC/blob/main/schematics_and_production_files/tropic_rev_F/SPF-94468_F.pdf)
- [MR-VMU-TROPIC](https://nxp.gitbook.io/vmu-rt1176/tropic) (NXP gitbook)
- [NXP Community](https://community.nxp.com/) (hardware support)
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)

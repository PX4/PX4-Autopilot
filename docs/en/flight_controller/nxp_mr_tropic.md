# NXP MR-VMU-TROPIC

<Badge type="tip" text="PX4 v1.18" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.nxp.com) for hardware support or compliance issues.
:::

The [MR-VMU-TROPIC](https://github.com/NXP-Robotics/MR-VMU-TROPIC) is NXP's open source reference design of a versatile VMU that can be used for drones, rovers, boats, and more.
As a reference/evaluation design, it is intended to be replicated, modified or integrated in production volume by others.
Since it is a reference design, this particular board is not produced in high volume.
Similar variants will be available from NXP licensees.

It has already gone through all FCC/CE ROHS REACH UKCA, EMI/RFI ESD certifications and is available globally.

The board combines an NXP i.MX RT1064 with sensor technology from Bosch® and InvenSense®, with the FMU, IMUs and other sensors on one compact board.
It is suitable for controlling any autonomous vehicle, in both academic and commercial applications.
The high-performance, low-noise IMUs are designed for better stabilisation.

Besides the standard interfaces, the carrier board offers 100BASE-T1 (two-wire) automotive Ethernet.
It also removes the IO processor to enable 8 PWM ports, all of them providing DShot capability.

The MR-VMU-TROPIC reference design suits developers at corporate research labs, startups, academia and commercial companies who want to try out 100BASE-T1 (two-wire) automotive Ethernet.

![MR-VMU-TROPIC side view](../../assets/flight_controller/nxp_mr_tropic/mr_vmu_tropic_side.jpg)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Specifications {#specifications}

### Processor {#processor}

- **Main FMU processor:** [NXP i.MX RT1064](https://www.nxp.com/products/i.MX-RT1064) Crossover MCU (32-bit Arm® Cortex®-M7 at 600 MHz, 1 MB on-chip SRAM with up to 512 KB configurable as tightly coupled memory (TCM), 4 MB on-chip flash)

### Sensors {#sensors}

- **IMU:** TDK InvenSense ICM-45686, Bosch BMI088 (on separate buses)
- **Barometer:** Bosch BMP390
- **Magnetometer:** Bosch BMM350

### Interfaces {#interfaces}

- **PWM outputs:** 8, all with DShot
- **Serial ports:** 3 general-purpose (2 with full flow control), plus 1 GPS port with safety switch
- **I2C buses:** 1 external port, for additional sensors and payloads
- **SPI buses:** 1 (OSD), with 1 chip select line, 1 data-ready line and 1 reset line
- **CAN buses:** 1 (CAN FD, NXP TJA1462ATK transceiver, 120 Ω termination resistor on board)
- **Ethernet:** 100BASE-T1 (two-wire), 100 Mbps, for transformerless applications and high-speed communication with mission computers (NXP TJA1103 PHY)
- **USB:** USB-C, high speed (480 Mbit/s)
- **RC input:** Yes, dedicated input for CRSF/ELRS
- **SD card:** microSD slot

### Electrical Data {#electrical_data}

- **Input voltage:** 5.5 V maximum
- **USB power input:** 5 V
- **Current draw:** typically 0.25–0.35 A at 5 V
- **Power monitoring:** 1 power input with SMBus (digital I2C), enabled by default

### Mechanical Data {#mechanical_data}

- **Dimensions:** 46 x 45 x 1.57 mm (standard baseboard)
- **Weight:** 12 g (standard baseboard)
- **Operating and storage temperature:** 0–85 °C

## Pinouts {#pinouts}

MR-VMU-TROPIC connectors follow a mix of Pixhawk (e.g. debug) and Betaflight (e.g. motor control) connectors.
All connectors are described in detail on the [NXP Robotics GitHub page](https://github.com/NXP-Robotics/MR-VMU-TROPIC).

### Top Side Connectors

- `J2`: Power input
- `J3`: Debug interface compliant with the Pixhawk Debug Full standard (DCD-F), including SWD and console access
- `J4`: Primary GPS port
- `J5`: CAN: Dual CAN-FD interfaces employing CAN-SIC (Signal Improvement) CAN transceivers
- `J6`: OSD: SPI expansion connector
- `J7`: Auxiliary UART connector
- `J8`: Telemetry port 1
- `J9`: Telemetry port 2
- `J11`: RC receiver connector
- `P1`: T1-ETH: 100BASE-T1 two-wire 100 Mbps Ethernet

![MR-VMU-TROPIC top-side connectors](../../assets/flight_controller/nxp_mr_tropic/overview_top.svg)

### Bottom Side Connectors

- `J1`: USB-C interface
- `J12`: PWM_A
- `J13`: PWM_B
- `CN1`: microSD card slot

![MR-VMU-TROPIC bottom-side connectors](../../assets/flight_controller/nxp_mr_tropic/overview_bottom.svg)

## Power {#power}

The board has two power rails: **POWER1** and **USB**.
The **POWER1** rail is supplied via the `J2` connector (the 6-circuit [2.00 mm pitch CLIK-Mate wire-to-board PCB receptacle](https://www.molex.com/en-us/products/part-detail/5024430670)) and directly powers the following ports: I2C (`J10`), GPS (`J4`), AUX (`J7`), CAN (`J5`), TELEM1 (`J8`), TELEM2 (`J9`), RC (`J11`) and SPI-OSD (`J6`).

::: warning
Any device connected to these ports must be compatible with the **POWER1** voltage rail.
:::

Digital I2C battery monitoring is enabled by default (see [Quickstart > Power](../assembly/quick_start_pixhawk6x.md#power)).

::: info
Analog battery monitoring via an ADC is not supported on this board.
:::

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

## Serial Port Mapping {#serial_port_mapping}

| UART  | Device     | Port        |
| ----- | ---------- | ----------- |
| UART6 | /dev/ttyS0 | Debug       |
| UART2 | /dev/ttyS1 | `GPS`       |
| UART3 | /dev/ttyS2 | `TELEM1`    |
| UART4 | /dev/ttyS3 | `AUX`       |
| UART5 | /dev/ttyS4 | `TELEM2`    |
| UART7 | /dev/ttyS5 | ESC ONEWIRE |
| UART8 | /dev/ttyS6 | `RC`        |

::: info
The `AUX` port is displayed as TEL3 in PX4 Firmware Configurator.
:::

## Assembly {#assembly}

Wiring is similar to the [Holybro Pixhawk 6X](../flight_controller/pixhawk6x.md#connections) and other boards that follow the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

## Building Firmware {#building_firmware}

::: tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make nxp_mr-tropic
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
- [NXP Community](https://community.nxp.com/) (hardware support)
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)

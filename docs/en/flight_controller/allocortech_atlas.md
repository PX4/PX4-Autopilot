# Allocortech Atlas Flight Computer

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://allocor.tech) for hardware support or compliance issues.
:::

The USA-built Atlas Flight Computer is a low-cost, ruggedised flight computer based on the [Pixhawk FMUv6X open source standard](https://github.com/pixhawk/Pixhawk-Standards).
In addition to the flight management unit (FMU), it integrates a Linux mission computer, air data sensors, a 5-port Ethernet switch, and an internal GPS in a single enclosure.

It offers a wide variety of interfaces including RS-232, RS-422, CAN, Ethernet and PWM, allowing it to handle both flight and payload management.

![Allocortech Atlas Flight Computer, top view showing the J1 and J2 D-Sub connectors, J3 SMA connector, pressure ports and mounting flanges](../../assets/flight_controller/allocortech_atlas/atlas_top_sma_flanges.png) ![Allocortech Atlas Flight Computer, isometric view of the enclosure and connectors](../../assets/flight_controller/allocortech_atlas/atlas_isometric.png)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

<Badge type="tip" text="main (PX4 v2.0)" />

## Specifications {#specifications}

### Processor {#processor}

- FMU processor: STM32H753II (Arm Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM)
- I/O processor: STM32F103
- Mission computer: [Toradex Verdin](https://www.toradex.com/computer-on-modules/verdin-arm-family) System on Module running Linux
  - Verdin AM62, 2 GB RAM, 16 GB eMMC

### Sensors {#sensors}

- [Bosch BMI088](https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi088/) IMU
- [InvenSense IIM-42652](https://invensense.tdk.com/products/motion-tracking/6-axis/iim-42652/) industrial IMU
- [InvenSense ICM-42688-P](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/) IMU
- [Bosch BMM350](https://www.bosch-sensortec.com/en/products/motion-sensors/magnetometers/bmm350) magnetometer
- Amphenol [AUAV-L30D](https://www.allsensors.com/hubfs/Product-Data-Sheets/DS-432.pdf) barometer and differential pressure sensor (static and pitot ports)
- Internal u-blox MAX-M10S GPS

The three IMUs are on separate SPI buses for sensor redundancy.

### Other Features

- Integrated 5-port Ethernet switch
- Two redundant, prioritised power inputs
- E-fused power outputs
- Sensor heater

::: warning
The Atlas does not have heater over-temperature protection, and the heater can overheat the FMU.
The heater is controlled by PX4, and is off by default.
If you enable it by setting [SENS_EN_THERMAL](../advanced_config/parameter_reference.md#SENS_EN_THERMAL) to `1`, PX4 regulates the temperature to [HEATER1_TEMP](../advanced_config/parameter_reference.md#HEATER1_TEMP) (55 °C by default).
Contact Allocortech for details.
:::

### Part Number {#part_number}

This page describes the Atlas part number `100-0104-002`, which is the standard production configuration.

### Architecture {#architecture}

The Atlas combines three systems in one enclosure: the FMU, the mission computer and the Wasp sensor hub.
They are connected by an internal Ethernet switch and share a common power supply.

![Atlas system architecture block diagram](../../assets/flight_controller/allocortech_atlas/atlas_system_architecture.png)

- **Mission computer:** connected to the FMU by the `TELEM2` serial link and the internal Ethernet switch.
- **Wasp:** a sensor hub on the `CAN2` bus.
  It provides the air data sensors, switches the output loads, monitors the input and system voltages, and selects between the internal and external GPS.
  It holds the [DroneCAN](../dronecan/index.md) configuration for the Atlas, so its parameters can be adjusted over CAN (see [Wasp Configuration](#wasp)).

### Interfaces {#interfaces}

- **RS-232:** 4
- **RS-422:** 2 (`TELEM1` and `GPS2`, both terminated)
- **CAN:** 2
- **PWM out:** 12
  - `PWM_OUT_1`-`8`: I/O processor (PX4 `MAIN` outputs)
  - `PWM_OUT_9`-`12`: FMU (PX4 `AUX` outputs)
- **General purpose out:** 6
  - `GPO_1`-`4`: FMU, PX4 `AUX` 5-8 (PWM capable)
- **General purpose in:** 6
- **Analog in (0-5 V):** 4
- **5 V, 500 mA out:** 2
- **28 V (nominal), 2 A out:** 2 (not regulated; follows the input voltage)
- **Ethernet:** internal 5-port switch connecting the FMU (10/100 Mb/s), the mission computer (1 Gb/s) and three external ports (one 1 Gb/s, two 10/100 Mb/s)
- **PX4IO/RC control signals:** 7
- **RC input:** through the I/O processor (see [Radio Control](#radio_control))
- **GPS 1PPS in:** 1
- **Debug USB:** 2
- **Storage (FMU):** 32 GB eMMC
- **Storage (Mission computer):** 128 GB eMMC
- **Parameter storage:** FM25V02A FRAM

### Electrical Data {#electrical_data}

- Input voltage: 9-36 V (28 V nominal)
- Power consumption: <20 W, varies with configuration

| Parameter                             | Min   | Nom  | Max    | Unit |
| ------------------------------------- | ----- | ---- | ------ | ---- |
| Input voltage                         | 9     | 28   | 36     | V    |
| Input current (Vin = 28 V, loads off) |       |      | 0.72   | A    |
| Input current (Vin = 28 V, loads on)  |       |      | 4.9    | A    |
| Input current (Vin = 9 V, loads off)  |       |      | 2.2    | A    |
| Input current (Vin = 9 V, loads on)   |       |      | 6.8    | A    |
| Input power (loads off)               |       |      | 20     | W    |
| Input power (loads on)                |       |      | 170    | W    |
| Heater current                        |       | 1.1  |        | A    |
| PWM output voltage                    | 0     |      | 5      | V    |
| PWM output impedance                  |       | 150  |        | Ω    |
| GPO voltage                           | 0     |      | 5      | V    |
| GPO impedance                         |       | 150  |        | Ω    |
| GPI V<sub>IH</sub>                    | 3     | 5    | 50     | V    |
| GPI V<sub>IL</sub>                    | 0     | 2    |        | V    |
| GPI impedance (constant current sink) | >1000 |      |        | Ω    |
| ADC input voltage                     | 0     |      | 5      | V    |
| `+5V_OUT-x` current                   |       | 500  | 750\*  | mA   |
| `+28V_OUT-x` current                  |       | 2000 | 3000\* | mA   |

\*E-fuse setpoint. Not recommended to exceed the nominal value.

::: warning
The E-fuses are set above the rated current limit.
Do not exceed the rated limit.
:::

### Mechanical Data {#mechanical_data}

- Dimensions: 6.5 x 3.82 x 1.47 in (165 x 97 x 37 mm)
- Mass: approx. 386 g (with enclosure)
- Operating temperature: -20 °C to 71 °C
- Storage temperature: -40 °C to 85 °C

![Atlas dimensions in inches (6.50 x 3.82 x 1.47)](../../assets/flight_controller/allocortech_atlas/atlas_dimensions.png)

![Atlas mounting interface: four 0.17 in through holes](../../assets/flight_controller/allocortech_atlas/atlas_mounting.png)

## Where to Buy {#store}

Contact [Allocortech](https://www.allocor.tech/contact) (US).

## Pinouts {#pinouts}

| Connector | Description                                  | Type                                                   |
| --------- | -------------------------------------------- | ------------------------------------------------------ |
| J1        | PWM, serial comms, GPIO                      | DD78S3S600X (78-pin D-Sub, socket); mate DD78M00000/AA |
| J2        | Power in/out, Ethernet, CAN, USB, RC-IO, ADC | DD78M3S600X (78-pin D-Sub, pin); mate DD78S00000/AA    |
| J3        | GPS antenna                                  | SMA female                                             |

![78-pin high-density D-Sub connector face](../../assets/flight_controller/allocortech_atlas/atlas_db78_connector.png)

### J1: GPIO and Serial Comms

| Pin   | Name              | Origin           | Notes                              |
| ----- | ----------------- | ---------------- | ---------------------------------- |
| 1-8   | `PWM_OUT_1`-`8`   | I/O processor    | PX4 `MAIN` 1-8                     |
| 9-12  | `PWM_OUT_9`-`12`  | FMU              | PX4 `AUX` 1-4                      |
| 13-16 | `GPO_1`-`4`       | FMU              | PX4 `AUX` 5-8 (PWM capable)        |
| 17-18 | `GPO_5`-`6`       | Mission computer |                                    |
| 19    | `FREE-232-TX`     | FMU              | `TELEM4` in PX4                    |
| 20    | `FREE-232-RX`     | FMU              | `TELEM4` in PX4                    |
| 21-39 | `GND`             |                  |                                    |
| 40-45 | `GPI_1`-`6`       | Mission computer |                                    |
| 46    | `TELEM3-232-TX`   | FMU              |                                    |
| 47    | `TELEM3-232-RX`   | FMU              |                                    |
| 48    | `GPS1-232-TX`     | FMU              |                                    |
| 49    | `GPS1-232-RX`     | FMU              |                                    |
| 50    | `TORADEX-232-TX`  | Mission computer |                                    |
| 51    | `TORADEX-232-RX`  | Mission computer |                                    |
| 52    | `TELEM1-422-TX_P` | FMU              |                                    |
| 53    | `TELEM1-422-TX_N` | FMU              |                                    |
| 54    | `TELEM1-422-RX_P` | FMU              |                                    |
| 55    | `TELEM1-422-RX_N` | FMU              |                                    |
| 56    | `GPS2-422-TX_P`   | FMU              |                                    |
| 57    | `GPS2-422-TX_N`   | FMU              |                                    |
| 58    | `GPS2-422-RX_P`   | FMU              |                                    |
| 59    | `GPS2-422-RX_N`   | FMU              |                                    |
| 60-66 | `GND`             |                  |                                    |
| 67    | `H_MCU_Recovery`  | Wasp             |                                    |
| 68    | `GND`             |                  |                                    |
| 69    | `H_EEPROM_1WIRE`  | Wasp             |                                    |
| 70    | `GND`             |                  |                                    |
| 71    | RFU               |                  | Reserved for future use            |
| 72    | `GND`             |                  |                                    |
| 73-75 | RFU               |                  | Reserved for future use            |
| 76    | `GPS_PPS_P`       |                  | 1PPS input from external GPS, LVDS |
| 77    | `GPS_PPS_N`       |                  | 1PPS input from external GPS, LVDS |
| 78    | `GND`             |                  |                                    |

### J2: Power, Ethernet, CAN

| Pin   | Name                 | Origin           | Notes                                 |
| ----- | -------------------- | ---------------- | ------------------------------------- |
| 1     | `ETH1_A_P`           | Ethernet switch  | 1 Gb/s port                           |
| 2     | `ETH1_A_N`           | Ethernet switch  |                                       |
| 3     | `ETH1_B_P`           | Ethernet switch  |                                       |
| 4     | `ETH1_B_N`           | Ethernet switch  |                                       |
| 5     | `ETH1_C_P`           | Ethernet switch  |                                       |
| 6     | `ETH1_C_N`           | Ethernet switch  |                                       |
| 7     | `ETH1_D_N`           | Ethernet switch  |                                       |
| 8     | `ETH1_D_P`           | Ethernet switch  |                                       |
| 9     | `ETH2_TX_P`          | Ethernet switch  | 10/100 Mb/s port                      |
| 10    | `ETH2_TX_N`          | Ethernet switch  |                                       |
| 11    | `ETH2_RX_N`          | Ethernet switch  |                                       |
| 12    | `ETH2_RX_P`          | Ethernet switch  |                                       |
| 13    | `ETH3_RX_N`          | Ethernet switch  | 10/100 Mb/s port                      |
| 14    | `ETH3_RX_P`          | Ethernet switch  |                                       |
| 15    | `ETH3_TX_N`          | Ethernet switch  |                                       |
| 16    | `ETH3_TX_P`          | Ethernet switch  |                                       |
| 17    | `GND`                |                  |                                       |
| 18    | `CAN1_N`             |                  |                                       |
| 19    | `CAN1_P`             |                  |                                       |
| 20    | `GND`                |                  |                                       |
| 21    | `+5V_OUT-A`          |                  | 500 mA max recommended, 750 mA E-fuse |
| 22    | `GND`                |                  |                                       |
| 23    | `+5V_OUT-B`          |                  | 500 mA max recommended, 750 mA E-fuse |
| 24    | `GND`                |                  |                                       |
| 25    | `+28V_OUT-A`         |                  | 2 A max recommended, 3 A E-fuse       |
| 26    | `GND`                |                  |                                       |
| 27    | `+28V_OUT-B`         |                  | 2 A max recommended, 3 A E-fuse       |
| 28    | `GND`                |                  |                                       |
| 29    | `F_USB_P`            | FMU              |                                       |
| 30    | `F_USB_N`            | FMU              |                                       |
| 31    | `F_VBUS_SENSE`       | FMU              |                                       |
| 32    | `GND`                |                  |                                       |
| 33    | `T_USB1_VBUS`        | Mission computer |                                       |
| 34    | `T_USB1_ID`          | Mission computer |                                       |
| 35    | `T_USB1_D_P`         | Mission computer |                                       |
| 36    | `T_USB1_D_N`         | Mission computer |                                       |
| 37-39 | `GND`                |                  |                                       |
| 40-43 | `VIN-A`              |                  | Primary power input                   |
| 44-47 | `GND`                |                  | Input power ground                    |
| 48-51 | `VIN-B`              |                  | Backup power input                    |
| 52    | `GND`                |                  |                                       |
| 53    | `F_FMU_SAFETY_IN`    | FMU              |                                       |
| 54    | `F_FMU_nSAFETY_LED`  | FMU              |                                       |
| 55    | `F_IO_VDD_SERVO1`    | FMU              |                                       |
| 56    | `F_DSM_INPUT`        | I/O processor    |                                       |
| 57    | `F_RC_INPUT`         | I/O processor    |                                       |
| 58    | `F_SBUS_OUT_RSSI_IN` | I/O processor    |                                       |
| 59    | `F_nARMED`           | FMU              |                                       |
| 60-63 | `VIN-A`              |                  | Primary power input                   |
| 64-67 | `GND`                |                  | Input power ground                    |
| 68-71 | `VIN-B`              |                  | Backup power input                    |
| 72    | `GND`                |                  |                                       |
| 73    | `CAN2_N`             |                  | Shared with the Wasp                  |
| 74    | `CAN2_P`             |                  | Shared with the Wasp                  |
| 75-78 | `ADC_IN0`-`3`        | FMU              | 0-5 V                                 |

### J3: GPS Antenna

| Pin  | Name  | Notes                                   |
| ---- | ----- | --------------------------------------- |
| Ring | `GND` |                                         |
| Pin  | `RF`  | Optional 3.3 V bias for active antennas |

## Power {#power}

The Atlas has two power inputs, `VIN-A` (primary) and `VIN-B` (backup).
It runs from `VIN-A` when it is present, and switches to `VIN-B` automatically if `VIN-A` fails.

It also supplies two 28 V outputs (`+28V_OUT-A`/`B`), which follow the input voltage, and two 5 V outputs (`+5V_OUT-A`/`B`).
All four outputs are protected by E-fuses, and can be switched on or off with the Wasp [power output parameters](#wasp_power_outputs).

![Atlas power distribution block diagram](../../assets/flight_controller/allocortech_atlas/atlas_power_distribution.png)

::: warning
The Atlas does not report battery voltage or current to PX4.
Without a battery monitor, PX4 does not show the battery level and cannot trigger low-battery warnings or [battery failsafes](../config/safety.md#battery-failsafes).
To enable them, connect a [DroneCAN power monitor](../dronecan/index.md) to `CAN1` and set [UAVCAN_SUB_BAT](../advanced_config/parameter_reference.md#UAVCAN_SUB_BAT) to `2`.
:::

## Voltage Ratings {#voltage_ratings}

`VIN-A` and `VIN-B` each accept 9 V to 36 V (28 V nominal).

The PWM and general purpose outputs are 5 V signals: the Atlas does not supply power to servos through them.

## PWM Outputs {#pwm_outputs}

The Atlas has 12 PWM outputs, plus 4 general purpose outputs that can also be used as PWM outputs.

| Signal           | Processor     | PX4 output |
| ---------------- | ------------- | ---------- |
| `PWM_OUT_1`-`8`  | I/O processor | `MAIN` 1-8 |
| `PWM_OUT_9`-`12` | FMU           | `AUX` 1-4  |
| `GPO_1`-`4`      | FMU           | `AUX` 5-8  |

[DShot](../peripherals/dshot.md) and bidirectional DShot are supported on `AUX` 1-6 only (`PWM_OUT_9`-`12`, `GPO_1` and `GPO_2`).

The FMU outputs are in the following groups:

- `AUX` 1-4 (Timer5): `PWM_OUT_9`-`12`
- `AUX` 5-6 (Timer4): `GPO_1`-`2`
- `AUX` 7-8 (Timer12): `GPO_3`-`4`

All outputs within a group must use the same protocol and rate.

## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) can be connected to `TELEM1` (RS-422), `TELEM3` (RS-232) or `TELEM4` (RS-232, labelled `FREE-232` on `J1`).
`TELEM2` is connected internally to the mission computer, and is configured for MAVLink at 57600 baud by default.

::: warning
The Atlas serial ports use RS-232 and RS-422 signal levels, not 3.3 V TTL.
Most telemetry radios and GPS modules have TTL serial ports, and need an RS-232 or RS-422 level converter.
:::

## Ethernet {#ethernet}

The Atlas has an internal 5-port Ethernet switch that connects the FMU (10/100 Mb/s), the mission computer (1 Gb/s) and three external ports on `J2` (one 1 Gb/s, two 10/100 Mb/s).

PX4 is configured for Ethernet by default:

- FMU IP address: `10.41.10.2` (gateway `10.41.10.254`)
- MAVLink: UDP port `14550`, broadcast ([MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG) = `1000`)
- [uXRCE-DDS](../middleware/uxrce_dds.md) client: connects to an agent at `10.41.10.1` ([UXRCE_DDS_CFG](../advanced_config/parameter_reference.md#UXRCE_DDS_CFG) = `1000`)

See [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md) for more information.

## Serial Port Mapping {#serial_port_mapping}

| UART   | Device     | Port                                                    |
| ------ | ---------- | ------------------------------------------------------- |
| USART1 | /dev/ttyS0 | GPS1 (RS-232) or internal GPS, selected by `gps_sel_en` |
| USART2 | /dev/ttyS1 | TELEM3 (RS-232)                                         |
| USART3 | /dev/ttyS2 | Debug console                                           |
| UART4  | /dev/ttyS3 | TELEM4 (RS-232, `FREE-232`)                             |
| UART5  | /dev/ttyS4 | TELEM2 (mission computer)                               |
| USART6 | /dev/ttyS5 | PX4IO                                                   |
| UART7  | /dev/ttyS6 | TELEM1 (RS-422)                                         |
| UART8  | /dev/ttyS7 | GPS2 (RS-422)                                           |

## Building Firmware {#building_firmware}

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make allocortech_atlas-v6x_default
```

## Debug Port {#debug_port}

The [PX4 System Console](../debug/system_console.md) runs on `USART3` (`/dev/ttyS2`) at 57600 baud.

<!-- placeholder
Which connectors carry the PX4 System Console and SWD (FMU and IO), whether they follow Pixhawk Debug Full or Mini, and, if not, the connector, pinout and cable.
Good examples: agam_v6xrt.md#debug_port, ark_v6xrt.md#debug_port, cuav_pixhawk_v6x.md#debug_port
-->

## Assembly {#assembly}

All connections to the Atlas are made through the `J1` and `J2` D-Sub connectors (see [Pinouts](#pinouts)), so a wiring harness is needed for power, PX4 connections and peripherals.

### Power Wiring {#power_wiring}

Connect the primary supply to `VIN-A` and, optionally, a backup supply to `VIN-B` (see [Power](#power)).
Each input is spread over several `J2` pins to carry the full input current, and all of them should be connected:

| Signal             | `J2` pins    |
| ------------------ | ------------ |
| `VIN-A`            | 40-43, 60-63 |
| `VIN-B`            | 48-51, 68-71 |
| Input power ground | 44-47, 64-67 |

### Connecting to QGroundControl {#qgc}

The Atlas does not have a USB connector on the enclosure.
To connect _QGroundControl_ over USB (for example to [load firmware](../config/firmware.md)), the harness must bring out the FMU USB signals `F_USB_P`, `F_USB_N` and `F_VBUS_SENSE` (`J2` pins 29-31).

Alternatively, connect over Ethernet: give the ground station computer a static IP address on the `10.41.10.x` network that is not already in use, and _QGroundControl_ will connect automatically to the MAVLink stream that the FMU broadcasts on UDP port `14550` (see [Ethernet](#ethernet)).

### Radio Control {#radio_control}

RC input is connected to the I/O processor, not directly to the FMU.
The I/O processor decodes the RC signal and passes it to PX4 over the PX4IO link (`USART6`), in the same way as on other [Pixhawk FMUv6X](../flight_controller/pixhawk6x.md) boards.
Connect the receiver to `F_RC_INPUT` (PPM, S.Bus) or `F_DSM_INPUT` (Spektrum DSM) on `J2`.

The I/O processor input is receive-only, so receivers that need a bidirectional UART, such as CRSF or GHST, are not supported.
The default firmware does not include the FMU RC drivers.

### GPS & Compass {#gps_compass}

The Atlas has an internal u-blox MAX-M10S GPS, with its antenna connected to the `J3` SMA connector.
The antenna bias voltage (3.3 V) is off by default: if you use an active antenna, enable it with the Wasp `gps_ant_en` parameter (see [GPS and CAN](#wasp_gps_can)).

External GPS modules can be connected to `GPS1` (RS-232) or `GPS2` (RS-422) on `J1`.
The internal GPS and `GPS1` share the FMU's `USART1` (`/dev/ttyS0`), and the Wasp `gps_sel_en` parameter selects which one is connected (the internal GPS by default).
`GPS2` (`UART8`, `/dev/ttyS7`) is always external.
The Wasp `pps_sel_en` parameter selects the 1PPS source in the same way.
An external 1PPS signal (LVDS) can be connected to `GPS_PPS_P`/`N` on `J1`.

The BMM350 magnetometer is inside the Atlas, on the FMU.
The Atlas has no external I2C port, so external compasses must be [DroneCAN](../dronecan/index.md) devices.

For general mounting guidance, see [Mounting a GPS/Compass](../assembly/mount_gps_compass.md).

### Safety Switch {#safety_switch}

A safety switch and its LED can be connected to `F_FMU_SAFETY_IN` and `F_FMU_nSAFETY_LED` (`J2` pins 53-54).
PX4 does not require a safety switch by default.
To make it a precondition for arming, set [CBRK_IO_SAFETY](../advanced_config/parameter_reference.md#CBRK_IO_SAFETY) to `0` (see [Arm, Disarm, Prearm Configuration](../advanced_config/prearm_arm_disarm.md#arming-sequence-pre-arm-mode-safety-button)).

### CAN Peripherals {#can}

[DroneCAN](../dronecan/index.md) peripherals such as ESCs, power monitors and compasses can be connected to `CAN1` (`J2` pins 18-19).

`CAN2` (`J2` pins 73-74) is shared with the internal [Wasp](#wasp), which uses node ID `100` by default.
If you connect other devices to `CAN2`, make sure that none of them use the Wasp's node ID.

## Wasp Configuration {#wasp}

The Wasp is a [DroneCAN](../dronecan/index.md) node on the `CAN2` bus, with node ID `100` by default.
It publishes the air data and magnetometer readings to PX4, and its parameters control the power outputs, GPS selection and CAN termination.

### Enable DroneCAN

To use the Wasp sensors, enable the PX4 DroneCAN driver and the matching sensor subscriptions:

1. In _QGroundControl_ set [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` (or `3` if you also use [DroneCAN ESCs](../dronecan/escs.md)) and reboot.
2. Enable the subscriptions for the Wasp sensors:
   - [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO): barometer
   - [UAVCAN_SUB_DPRES](../advanced_config/parameter_reference.md#UAVCAN_SUB_DPRES): differential pressure (airspeed)
   - [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG): magnetometer (enabled by default)
3. Reboot the vehicle.

DroneCAN configuration in PX4 is explained in more detail in [DroneCAN > PX4 Configuration](../dronecan/index.md#px4-configuration).

### Wasp Parameters

The Wasp parameters are set on the Wasp node, not on PX4.
They can be changed in _QGroundControl_ (see [DroneCAN > QGC CANNODE Parameter Configuration](../dronecan/index.md#qgc-cannode-parameter-configuration)), or with any DroneCAN tool such as the [DroneCAN GUI Tool](https://dronecan.github.io/GUI_Tool/Overview/).

::: warning
The Wasp only applies the output, GPS and termination parameters at startup.
After changing them, restart the Wasp node (for example from the DroneCAN GUI Tool) or power-cycle the Atlas.
:::

#### Power Outputs {#wasp_power_outputs}

| Parameter   | Output       | Default |
| ----------- | ------------ | ------- |
| `load_0_en` | `+28V_OUT-A` | 1 (on)  |
| `load_1_en` | `+28V_OUT-B` | 1 (on)  |
| `load_2_en` | `+5V_OUT-A`  | 1 (on)  |
| `load_3_en` | `+5V_OUT-B`  | 1 (on)  |

Set a parameter to `0` to disable the output, or `1` to enable it.
The outputs cannot be switched in flight: they are set once, when the Wasp starts.

#### GPS and CAN {#wasp_gps_can}

| Parameter             | Description                                                                      | Default |
| --------------------- | -------------------------------------------------------------------------------- | ------- |
| `gps_sel_en`          | FMU `USART1` source: `0` internal GPS, `1` external GPS on `GPS1`.               | 0       |
| `pps_sel_en`          | FMU 1PPS input source: `0` internal GPS, `1` external GPS.                       | 0       |
| `gps_ant_en`          | Enables the bias voltage on the `J3` GPS antenna connector, for active antennas. | 0 (off) |
| `can1_term_en`        | `CAN1` bus termination.                                                          | 1 (on)  |
| `can2_term_en`        | `CAN2` bus termination.                                                          | 1 (on)  |
| `dronecan_node_id`    | Wasp DroneCAN node ID (1-125).                                                   | 100     |
| `dronecan_mag_select` | Magnetometers to publish: `0` or `1` primary, `2` secondary, `3` all.            | 0       |

The Atlas ships with the internal GPS selected for both the GPS data and the 1PPS signal.

#### Power Monitoring

The Wasp can report the input voltages (`VIN`, `VIN-A` and `VIN-B`) as DroneCAN [CircuitStatus](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#circuitstatus) messages.
Reporting is off by default; enable it with `circuit_report` (a bitmask of the circuits to report) and `dronecan_circuit_stat_hz` (the message rate).

::: info
PX4 does not use `CircuitStatus` messages, so these voltages are not shown in _QGroundControl_.
Use a DroneCAN tool such as the [DroneCAN GUI Tool](https://dronecan.github.io/GUI_Tool/Overview/) to view them.
:::

## Further Information {#further_information}

- [Allocortech Atlas Product Page](https://www.allocor.tech/atlas-product-page)

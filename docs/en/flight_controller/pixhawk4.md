# Holybro Pixhawk 4

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

_Pixhawk 4_<sup>&reg;</sup> is an advanced autopilot designed and made in collaboration with Holybro<sup>&reg;</sup> and the PX4 team.
It is optimized to run PX4 v1.7 and later, and is suitable for academic and commercial developers.

It is based on the [Pixhawk-project](https://pixhawk.org/) **FMUv5** open hardware design and runs PX4 on the [NuttX](https://nuttx.apache.org/) OS.

<img src="../../assets/flight_controller/pixhawk4/pixhawk4_hero_upright.jpg" width="200px" title="Pixhawk4 Upright Image" /> <img src="../../assets/flight_controller/pixhawk4/pixhawk4_logo_view.jpg" width="420px" title="Pixhawk4 Image" />

::: tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::

## Specifications {#specifications}

### Processor {#processor}

- **Main FMU processor:** STM32F765 (32 Bit Arm® Cortex®-M7, 216 MHz, 2 MB memory, 512 KB RAM)
- **IO processor:** STM32F100 (32 Bit Arm® Cortex®-M3, 24 MHz, 8 KB SRAM)

### Sensors {#sensors}

- **IMU:** ICM-20689, and BMI055 or ICM-20602
- **Barometer:** MS5611
- **Magnetometer:** IST8310
- **GPS:** u-blox Neo-M8N GPS/GLONASS receiver, with integrated IST8310 magnetometer

### Interfaces {#interfaces}

- **PWM outputs:** 8-16 (8 from IO, 8 from FMU)
- **Capture inputs:** 3 dedicated PWM/capture inputs on FMU
- **RC input:** dedicated input for CPPM, and dedicated input for Spektrum/DSM and S.Bus with analog/PWM RSSI input
- **S.Bus output:** dedicated S.Bus servo output
- **Serial ports:** 5 general purpose serial ports
- **I2C ports:** 3
- **SPI buses:** 4
- **CAN buses:** up to 2, for dual CAN with serial ESC

### Electrical Data {#electrical_data}

- **Power module output:** 4.9 V to 5.5 V
- **USB power input:** 4.75 V to 5.25 V
- **Servo rail input:** 0 V to 36 V
- **Power monitoring:** analog inputs for voltage and current of 2 batteries

### Mechanical Data {#mechanical_data}

- **Dimensions:** 44 × 84 × 12 mm
- **Weight:** 15.8 g
- **Operating temperature:** -40 °C to 85 °C

![Pixhawk 4 Dimensions](../../assets/flight_controller/pixhawk4/pixhawk4_dimensions.jpg)

Additional information can be found in the [Pixhawk 4 Technical Data Sheet](https://github.com/PX4/PX4-Autopilot/blob/main/docs/assets/flight_controller/pixhawk4/pixhawk4_technical_data_sheet.pdf).

## Where to Buy {#store}

Order from [Holybro](https://holybro.com/products/pixhawk-4).

## Pinouts {#pinouts}

![Pixhawk 4 connectors](../../assets/flight_controller/pixhawk4/pixhawk4-connectors.jpg)

::: warning
The `DSM/SBUS RC` and `PPM RC` ports are for RC receivers only.
These are powered! NEVER connect any servos, power supplies or batteries (or to any connected receiver).
:::

Download _Pixhawk 4_ pinouts from [here](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Pixhawk4-Pinouts.pdf).

::: info
Connector pin assignments are left to right (i.e. Pin 1 is the left-most pin).
The exception is the [debug port(s)](#debug_port) (pin 1 is the right-most, as shown below).
:::

## Voltage Ratings {#voltage_ratings}

_Pixhawk 4_ can be triple-redundant on the power supply if three power sources are supplied.
The three power rails are: `POWER1`, `POWER2` and `USB`.

::: info
The output power rails `FMU PWM OUT` and `I/O PWM OUT` (0 V to 36 V) do not power the flight controller board (and are not powered by it).
You must supply power to one of `POWER1`, `POWER2` or `USB` or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:

1. `POWER1` and `POWER2` inputs (4.9 V to 5.5 V)
1. `USB` input (4.75 V to 5.25 V)

**Absolute Maximum Ratings**

Under these conditions the system will not draw any power (will not be operational), but will remain intact.

1. `POWER1` and `POWER2` inputs (operational range 4.1 V to 5.7 V, 0 V to 10 V undamaged)
1. `USB` input (operational range 4.1 V to 5.7 V, 0 V to 6 V undamaged)
1. Servo input: `VDD_SERVO` pin of `FMU PWM OUT` and `I/O PWM OUT` (0 V to 42 V undamaged)

## Serial Port Mapping {#serial_port_mapping}

| UART   | Device       | Port                    |
| ------ | ------------ | ----------------------- |
| UART1  | `/dev/ttyS0` | `GPS`                   |
| USART2 | `/dev/ttyS1` | `TELEM1` (flow control) |
| USART3 | `/dev/ttyS2` | `TELEM2` (flow control) |
| UART4  | `/dev/ttyS3` | `TELEM4`                |
| USART6 | `/dev/ttyS4` | RC SBUS                 |
| UART7  | `/dev/ttyS5` | Debug Console           |
| UART8  | `/dev/ttyS6` | PX4IO                   |

## Assembly {#assembly}

The [Pixhawk 4 Wiring Quick Start](../assembly/quick_start_pixhawk4.md) provides instructions on how to assemble required/important peripherals including GPS, Power Management Board etc.

## Building Firmware {#building_firmware}

::: tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make px4_fmu-v5_default
```

## Debug Port {#debug_port}

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the `FMU Debug` port, while the I/O console and SWD interface can be accessed via `I/O Debug` port.
In order to access these ports, the user must remove the _Pixhawk 4_ casing.

![Pixhawk 4 Debug Ports](../../assets/flight_controller/pixhawk4/pixhawk4_debug_port.jpg)

The pinout uses the standard [Pixhawk debug connector pinout](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).
For wiring information see:

- [System Console > Pixhawk Debug Port](../debug/system_console.md#pixhawk_debug_port)

## Peripherals

- [Digital Airspeed Sensor](https://store-drotek.com/793-digital-differential-airspeed-sensor-kit-.html)
- [Telemetry Radio Modules](../telemetry/index.md)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## Supported Platforms / Airframes

Any multicopter / airplane / rover or boat that can be controlled with normal RC servos or Futaba S-Bus servos.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Further Information {#further_information}

- [Pixhawk 4 Technical Data Sheet](https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/flight_controller/pixhawk4/pixhawk4_technical_data_sheet.pdf)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165).
- [Pixhawk 4 Wiring QuickStart](../assembly/quick_start_pixhawk4.md)
- [Pixhawk 4 Pinouts](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Pixhawk4-Pinouts.pdf) (Holybro)
- [Pixhawk 4 Quick Start Guide](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Pixhawk4-quickstartguide.pdf) (Holybro)

# Agam Autopilot v6X-RT

<Badge type="tip" text="main (PX4 v2.0)" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.agamrobotics.com/) for hardware support or compliance issues.
:::

The [Agam Autopilot v6X-RT](https://www.agamrobotics.com/product-page/agam-pixhawk-6x-full-set) flight controller is [Agam Robotics'](https://www.agamrobotics.com/) implementation of the [NXP-led FMUv6X-RT reference design](../flight_controller/nxp_mr_vmu_rt1176.md).

![Agam Autopilot v6X-RT](../../assets/flight_controller/agam_v6xrt/agam_v6xrt_front.png)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Introduction

Agam Autopilot v6X-RT uses the NXP i.MX RT1176 MCU, consisting of an Arm® Cortex®-M7 core at 1GHz and an Arm® Cortex®-M4 core at 400MHz, with 2MB SRAM and 64MB Octal SPI flash.
The increased processing power lets firmware developers run more complex algorithms without being constrained by the flight controller's compute budget.

The board uses sensors from InvenSense® and Bosch®, and ships in two sensor-set variants (see [Processors & Sensors](#processors-sensors)).
Redundant IMU and barometer sensors sit on separate buses, and 2 of the 3 IMUs are mounted on a shock-isolated, heated sensor board, separate from the FMU board itself, to improve accuracy in flight and across a wide temperature range.

An integrated Ethernet interface (100BASE-T1, 2-wire) provides high-speed, low-latency connectivity to companion computers (for example a Raspberry Pi or NVIDIA Jetson), cameras, or data-transmission systems.

## Key Design Points

- High-performance [NXP i.MX RT1176](https://www.nxp.com/products/i.MX-RT1170) dual-core MCU
- [NXP EdgeLock SE051](https://www.nxp.com/products/SE051) hardware secure element, field-updatable, certified to CC EAL 6+
- Triple redundant IMU (2 sensors on a shock-mounted, heated sensor board, 1 on the FMU board itself) and dual redundant barometer, each on a separate bus
- Independent power control and separate buses for each sensor domain
- 100BASE-T1 (2-wire) automotive Ethernet for high-speed companion computer integration
- 256Kbit FRAM

::: tip
This board ships with `SENS_IMU_MODE` and `EKF2_MULTI_IMU` set so that only the FMU-board IMU is used by default.
To take advantage of the triple-redundant IMU set `SENS_IMU_MODE` to `Disabled (0)`, reboot, set `EKF2_MULTI_IMU` to `3`, and reboot again.
See [Agam's IMU triple redundancy notes](https://agamrobotics.gitbook.io/docs/autopilots-flight-controller/quickstart/imu-triple-redundancy-feature) for more information.
:::

### Processors & Sensors {#processors-sensors}

- FMU Processor: [NXP i.MX RT1176 (MIMXRT1176DVMAA)](https://www.nxp.com/products/i.MX-RT1170)
  - Arm® Cortex®-M7 core @ 1GHz
  - Arm® Cortex®-M4 core @ 400MHz (secondary core)
  - 2MB SRAM
  - 64MB external Octal SPI flash
- On-board sensors — Agam Autopilot v6X-RT ships in two sensor-set variants (Sensor Set 1 and Sensor Set 3, per [Pixhawk Standards](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-020%20Pixhawk%20Autopilot%20v6X-RT%20Standard.pdf)):
  - Sensor Set 1:
    - 2x [InvenSense ICM-42688-P IMU](https://www.invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
    - [Bosch BMI088 IMU](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
    - [Bosch BMM150 Magnetometer](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf)
    - 2x [Bosch BMP388 Barometer](https://www.bosch-sensortec.com/en/products/environmental-sensors/pressure-sensors/bmp390/)
  - Sensor Set 3:
    - 2x [TDK InvenSense ICM-45686 IMU](https://www.invensense.tdk.com/en-us/products/6-axis/icm-45686)
    - [Bosch BMI088 IMU](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
    - [Bosch BMM350 Magnetometer](https://www.bosch-sensortec.com/en/products/motion-sensors/magnetometers/bmm350)
    - 2x [Bosch BMP388 Barometer](https://www.bosch-sensortec.com/en/products/environmental-sensors/pressure-sensors/bmp390/)

### Electrical data

- Voltage Ratings:
  - Max input voltage: 6V
  - USB Power Input: 4.75\~5.25V
  - Servo Rail Input: 0\~36V
- Current Ratings:
  - `TELEM1` output current limiter: 1.5A
  - All other port combined output current limiter: 1.5A

### Mechanical data

- Dimensions
  - Flight controller module: 39 x 32 x 16 mm
  - Baseboard: 95 x 50 x 19 mm
- Weight: 90 g
- Operating temperature: -40°C to 85°C

### Interfaces

- 12 PWM/DShot outputs (8 DShot-capable, 4 PWM-only)
- 1x RC input, SBUS/PPM
- 1x dedicated DSM/Spektrum RC input
- 3x TELEM ports (`TELEM1`, `TELEM2`, `TELEM3`), full flow control
- 2x GPS ports (`GPS1` full GPS plus safety switch and buzzer, `GPS2` basic)
- 4x I2C
- 3x CAN-FD
- 1x SPI bus, 2 chip-select lines, 2 data-ready lines, 1 sync line, 1 reset line
- 1x NFC antenna port (connected to the SE051 secure element)
- 100BASE-T1 Ethernet (single 2-wire port)
- USB 2.0
- microSD card slot
- Dual redundant power input (5V via Molex connectors)

## Where To Buy {#store}

Order from the [Agam Robotics store](https://www.agamrobotics.com/product-page/agam-pixhawk-6x-full-set).

## Assembly/Setup

Agam Autopilot v6X-RT is a Pixhawk-standard v6X-RT board, electrically and mechanically identical to the NXP reference design, differing from it only in enclosure.
Wiring is similar to the [Holybro Pixhawk 6X](../flight_controller/pixhawk6x.md#connections) and other boards that follow the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

For wiring and assembly, follow the [Pixhawk 6X Wiring Quick Start](../assembly/quick_start_pixhawk6x.md), which applies to this board other than the enclosure.

## Connections

The connectors follow the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf), which defines the pinout of each port type.
For the physical layout of the ports on the board see the [Agam Autopilot v6X-RT baseboard ports](https://agamrobotics.gitbook.io/docs/autopilots-flight-controller/quickstart/baseboard-port) documentation.

## Pinouts

Agam Autopilot v6X-RT's pinout is the same as other v6X-RT-family boards:

- [DS-012 Pixhawk Autopilot v6X Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-012%20Pixhawk%20Autopilot%20v6X%20Standard.pdf)
- [DS-010 Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- [DS-009 Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)
- [Agam Autopilot v6X-RT Baseboard Port pinouts](https://agamrobotics.gitbook.io/docs/autopilots-flight-controller/quickstart/baseboard-port) (agamrobotics.gitbook.io)

Notes:

- The [camera capture pin](../camera/fc_connected_camera.md#camera-capture-configuration) (`PI0`) is pin 2 on the AD&IO port, marked `FMU_CAP`.

## Serial Port Mapping

| UART     | Device     | Port    |
| -------- | ---------- | ------- |
| LPUART1  | /dev/ttyS0 | Console |
| LPUART3  | /dev/ttyS1 | GPS     |
| LPUART4  | /dev/ttyS2 | TELEM1  |
| LPUART5  | /dev/ttyS3 | GPS2    |
| LPUART6  | /dev/ttyS4 | PX4IO   |
| LPUART8  | /dev/ttyS5 | TELEM2  |
| LPUART10 | /dev/ttyS6 | TELEM3  |
| LPUART11 | /dev/ttyS7 | EXT2    |

## Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).
You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC IN` (FMU): SBUS/PPM
- `DSM RC` (FMU): Spektrum/DSM

For PPM and S.Bus receivers, a single signal wire carries all channels.
If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md).

## Voltage Ratings

_Agam Autopilot v6X-RT_ can be triple-redundant on the power supply if three power sources are supplied.
The three power rails are: **POWER1**, **POWER2** and **USB**.
The **POWER1** & **POWER2** ports use a 6-circuit [2.00mm Pitch CLIK-Mate Wire-to-Board PCB Receptacle](https://www.molex.com/en-us/products/part-detail/5024430670).

### Normal Operation Maximum Ratings

Under these conditions all power sources will be used in this order to power the system:

1. **POWER1** and **POWER2** inputs (4.9V to 5.5V)
2. **USB** input (4.75V to 5.25V)

### Absolute Maximum Ratings

Under these conditions the system will not draw any power (will not be operational), but will remain intact.

1. **POWER1** and **POWER2** inputs (operational range 4.1V to 5.7V, 0V to 10V undamaged)
2. **USB** input (operational range 4.1V to 5.7V, 0V to 6V undamaged)
3. Servo input: VDD_SERVO pin of **FMU PWM OUT** (0V to 42V undamaged)

Digital I2C battery monitoring is enabled by default.
Analog/ADC battery monitoring is also supported.

## Building Firmware

::: tip
Most users will not need to build this firmware from PX4 v2.0.
It will be pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make agam-robotics_fmu-v6xrt_default
```

## Debug Port {#debug_port}

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the **FMU Debug** port.

The pinout and connector comply with the [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) interface defined in the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) (JST SH 1mm pitch, 10-pin connector).
Pin 6 is `SPI6_MOSI_EXTERNAL1`, which is the pad's `JTAG_TDO` function and carries `SWO` when the debug port is in serial-wire mode.

| Pin        | Signal         | Volt  |
| ---------- | -------------- | ----- |
| 1 (red)    | `FMU_VDD_3V3`  | +3.3V |
| 2 (black)  | `FMU_USART_TX` | +3.3V |
| 3 (black)  | `FMU_USART_RX` | +3.3V |
| 4 (black)  | `FMU_SWD_IO`   | +3.3V |
| 5 (black)  | `FMU_SWD_CK`   | +3.3V |
| 6 (black)  | `SWO`          | +3.3V |
| 7 (black)  | NFC GPIO       | +3.3V |
| 8 (black)  | PH11           | +3.3V |
| 9 (black)  | `FMU_nRST`     | +3.3V |
| 10 (black) | `GND`          | GND   |

For information about using this port see:

- [SWD Debug Port](../debug/swd_debug.md)
- [PX4 System Console](../debug/system_console.md)

## Peripherals

- [Agam GNSS01 CAN](https://agamrobotics.gitbook.io/docs/gnss-and-rtk-systems/agam-gnss01-can)
- [Agam GNSS02 CAN RTK](https://agamrobotics.gitbook.io/docs/gnss-and-rtk-systems/agam-gnss02-can-rtk)
- [Agam GNSS Base RTK](https://agamrobotics.gitbook.io/docs/gnss-and-rtk-systems/agam-gnss-base-rtk)
- [Agam FloRange Sensor](https://agamrobotics.gitbook.io/docs/sensors/agam-florange-sensor)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)
- [Telemetry Radio Modules](https://holybro.com/collections/telemetry-radios?orderby=date)

## Further info

- [Update Pixhawk 6X-RT Bootloader](../advanced_config/bootloader_update_v6xrt.md)
- [Agam Autopilot v6X-RT Documentation](https://agamrobotics.gitbook.io/docs/autopilots-flight-controller/quickstart) (Agam Robotics Docs)
- [Agam Autopilot v6X-RT setup video playlist](https://www.youtube.com/playlist?list=PLv71BKC3TLci9fgwV4U6_2C2Uxtrm3SQE) (YouTube, Agam Robotics)
- [Pixhawk 6X Wiring QuickStart](../assembly/quick_start_pixhawk6x.md)
- [Agam Digital Power Module (DPM01)](https://agamrobotics.gitbook.io/docs/digital-power-modules-dpm/dpm01-6s)
- [Agam Analog Power Module (APM)](https://agamrobotics.gitbook.io/docs/analog-power-modules-apm/generic)
- [Announcing the Pixhawk FMUv6X-RT](https://dronecode.org/announcing-the-pixhawk-fmuv6x-rt/) (Dronecode Foundation)
- [Pixhawk FMUv6X-RT open standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-020%20Pixhawk%20Autopilot%20v6X-RT%20Standard.pdf)
- [Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)

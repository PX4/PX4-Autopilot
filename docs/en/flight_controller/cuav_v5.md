# CUAV v5 (Discontinued)

<Badge type="info" text="Discontinued" />

:::warning
This flight controller has been [discontinued](../flight_controller/autopilot_experimental.md) and is no longer commercially available.
:::

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

_CUAV v5_<sup>&reg;</sup> (previously "Pixhack v5") is an advanced autopilot designed and made by CUAV<sup>&reg;</sup>.
The board is based on the [Pixhawk-project](https://pixhawk.org/) **FMUv5** open hardware design.
It runs PX4 on the [NuttX](https://nuttx.apache.org/) OS, and is fully compatible with PX4 firmware.
It is intended primarily for academic and commercial developers.

![CUAV v5](../../assets/flight_controller/cuav_v5/pixhack_v5.jpg)

## Quick Summary

- Main FMU Processor: STM32F765
  - 32 Bit Arm® Cortex®-M7, 216MHz, 2MB memory, 512KB RAM
- IO Processor: STM32F100
  - 32 Bit Arm® Cortex®-M3, 24MHz, 8KB SRAM
- On-board sensors:

  - Accelerometer/Gyroscope: ICM-20689
  - Accelerometer/Gyroscope: BMI055
  - Magnetometer: IST8310
  - Barometer: MS5611

- Interfaces:
  - 8-14 PWM outputs (6 from IO, 8 from FMU)
  - 3 dedicated PWM/Capture inputs on FMU
  - Dedicated R/C input for CPPM
  - Dedicated R/C input for PPM and S.Bus
  - analog / PWM RSSI input
  - S.Bus servo output
  - 5 general purpose serial ports
  - 4 I2C ports
  - 4 SPI buses
  - 2 CANBuses with serial ESC
  - Analog inputs for voltage / current of 2 batteries
- Power System:
  - Power: 4.3~5.4V
  - USB Input: 4.75~5.25V
  - Servo Rail Input: 0~36V
- Weight and Dimensions:
  - Weight: 90g
  - Dimensions: 44x84x12mm
- Other Characteristics:
  - Operating temperature: -20 ~ 80°C （Measured value）

## Where to Buy

Order from [CUAV](https://cuav.taobao.com/index.htm?spm=2013.1.w5002-16371268426.2.411f26d9E18eAz).

## Connection

![CUAV v5](../../assets/flight_controller/cuav_v5/pixhack_v5_connector.jpg)

:::warning
The RCIN interface is limited to powering the rc receiver and cannot be connected to any power/load.
:::

## Voltage Ratings

_CUAV v5_ can be triple-redundant on the power supply if three power sources are supplied. The three power rails are: **POWER1**, **POWER2** and **USB**.

::: info
The output power rails **FMU PWM OUT** and **I/O PWM OUT** (0V to 36V) do not power the flight controller board (and are not powered by it).
You must supply power to one of **POWER1**, **POWER2** or **USB** or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:

1. **POWER1** and **POWER2** inputs (4.3V to 5.4V)
1. **USB** input (4.75V to 5.25V)

## Building Firmware

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v5_default
```

## Debug Port

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) operate on the **FMU Debug** port.
Simply connect the FTDI cable to the Debug & F7 SWD connector.
To access the I/O Debug port, the user must remove the CUAV v5 shell.
Both ports have standard serial pins and can be connected to a standard FTDI cable (3.3V, but 5V tolerant).

The pinout is as shown.

![CUAV v5 debug](../../assets/flight_controller/cuav_v5/pixhack_v5_debug.jpg)

| pin | CUAV v5 debug |
| --- | ------------- |
| 1   | GND           |
| 2   | FMU-SWCLK     |
| 3   | FMU-SWDIO     |
| 4   | UART7_RX      |
| 5   | UART7_TX      |
| 6   | VCC           |

## Serial Port Mapping

| UART   | Device     | Port                                  |
| ------ | ---------- | ------------------------------------- |
| UART1  | /dev/ttyS0 | GPS                                   |
| USART2 | /dev/ttyS1 | TELEM1 (flow control)                 |
| USART3 | /dev/ttyS2 | TELEM2 (flow control)                 |
| UART4  | /dev/ttyS3 | TELEM4                                |
| USART6 | /dev/ttyS4 | TX is RC input from SBUS_RC connector |
| UART7  | /dev/ttyS5 | Debug Console                         |
| UART8  | /dev/ttyS6 | PX4IO                                 |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

## Peripherals

- [Digital Airspeed Sensor](https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-16371268452.37.6d9f48afsFgGZI&id=9512463037)
- [Telemetry Radio Modules](https://cuav.taobao.com/category-158480951.htm?spm=2013.1.w5002-16371268426.4.410b7a821qYbBq&search=y&catName=%CA%FD%B4%AB%B5%E7%CC%A8)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## Supported Platforms / Airframes

Any multicopter / airplane / rover or boat that can be controlled with normal RC servos or Futaba S-Bus servos.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Further info

- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165).
- [CUAV v5 docs](http://doc.cuav.net/flight-controller/v5-autopilot/en/v5.html)
- [CUAV Github](https://github.com/cuav)

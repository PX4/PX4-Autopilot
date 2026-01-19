# CORVON V5 Autopilot

:::warning
PX4 does not manufacture this (or any) autopilot. Contact the [manufacturer](https://corvon.tech) for hardware support or compliance issues.
:::

![CORVON V5 front view](../../assets/flight_controller/corvon_V5/frontview.png)

![CORVON V5 side view 1](../../assets/flight_controller/corvon_V5/sideview_1.png)

![CORVON V5 side view 2](../../assets/flight_controller/corvon_V5/sideview_2.png)

The CORVON V5 is based on the Pixhawk FMUv5 design standard and runs PX4 on NuttX.

:::info
This flight controller is [manufacturer supported](autopilot_manufacturer_supported.md).
:::

## Quick Summary

- **Main FMU Processor:** STM32F765IIK
  - 32 Bit Arm® Cortex®-M7, 216MHz, 2MB memory, 512KB RAM

- **On-board sensors:**
  - Accel/Gyro: ICM-20689
  - Accel/Gyro: ICM-20602
  - Accel/Gyro: BMI088
  - Magnetometer: IST8310
  - Barometer: MS5611

- **Interfaces:**
  - 8 PWM outputs
  - 3 dedicated PWM/Capture inputs on FMU
  - Dedicated R/C input for CPPM
  - Dedicated R/C input for Spektrum / DSM and S.Bus
  - Analog / PWM RSSI input
  - 4 general purpose serial ports
  - 3 I2C ports
  - 4 SPI buses
  - 2 CAN Buses
  - Analog inputs for voltage / current of battery
  - 2 additional analog inputs
  - Supports nARMED

- **Power System:**
  - Power Brick Input: 4.75~5.5V
  - USB Power Input: 4.75~5.25V

- **Weight and Dimensions:**
  - Weight: 42.1g
  - Dimensions: 61.2 x 40 x 15.9mm

- **Other Characteristics:**
  - Operating temperature: -20 ~ 85°C (Measured value)

## Where to Buy

- [CORVON Store](https://corvon.tech)

## Connectors and Interfaces

![Connectors and Interfaces](../../assets/flight_controller/corvon_V5/connectors_and_interfaces.png)

## Pinouts

Download Corvon V5 pinouts from here: [corvon_V5_pinout.xlsx](../../assets/flight_controller/corvon_V5/corvon_V5_pinout.xlsx)

## Serial Port Mapping

| UART | Device | Port |
|---|---|---|
| UART1 | `/dev/ttyS0` | GPS |
| USART2 | `/dev/ttyS1` | TELEM1 |
| USART3 | `/dev/ttyS2` | TELEM2 |
| UART4 | `/dev/ttyS3` | TELEM4 |
| USART6 | `/dev/ttyS4` | TX is RC input from SBUS_RC connector |
| UART7 | `/dev/ttyS5` | Debug Console |
| UART8 | `/dev/ttyS6` | Reserved for optional onboard RTK module |

Note: UART8 is reserved for an optional onboard UM982 module footprint and is not intended for general external use.

## Voltage Ratings

CORVON V5 must be powered from the **POWER** connector during flight, and may also be powered from **USB** for bench testing.

- **POWER input:** 4.75~5.5V
- **USB input:** 4.75~5.25V

The **PM2** connector **cannot power** the flight controller. On PX4 this interface is **not supported / do not use**.

## Building Firmware

To build PX4 for this target:

```sh
make corvon_V5_default
```

## Installing PX4 Firmware

The firmware can be installed in any of the normal ways:

- **Build and upload the source**

  ```sh
  make corvon_V5_default upload
  ```

- **Load the firmware using QGroundControl.** You can use either pre-built firmware or your own custom firmware.

:::info
If this target is not listed in QGroundControl, build and upload from source or load a custom firmware file (see *Installing PX4 Main, Beta or Custom Firmware*).
:::

## Supported Platforms / Airframes

Any multicopter / airplane / rover or boat that can be controlled with normal RC servos or Futaba S-Bus servos. The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Further Information

- [Corvon Tech](https://corvon.tech)

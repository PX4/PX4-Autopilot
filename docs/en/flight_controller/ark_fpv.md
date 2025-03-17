# ARK FPV Flight Controller

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://arkelectron.com/contact-us/) for hardware support or compliance issues.
:::

The USA-built ARK FPV flight controller is based on the [ARKV6X](https://arkelectron.com/product/arkv6x/) in a 30.5mm standard mounting pattern, supporting a 3-12s battery input with a regulated 12V 2A output for video transmitters and payloads.

![ARK FPV Main Photo](../../assets/flight_controller/arkfpv/ark_fpv.jpg)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Where To Buy

Order from [Ark Electronics](https://arkelectron.com/product/arkv6x/) (US)

## Documentation

See the documentation [Ark Electronics GitBook](https://arkelectron.gitbook.io/ark-documentation/flight-controllers/ark-fpv)

## Sensors

- [Invensense IIM-42653 Industrial IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/iim-42653/)
- [Bosch BMP390 Barometer](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/)
- [ST IIS2MDC Magnetometer](https://www.st.com/en/magnetic-sensors/iis2mdc.html)

## Microprocessor

- [STM32H743IIK6 MCU](https://www.st.com/en/microcontrollers-microprocessors/stm32h743ii.html)
  - 480 MHz
  - 2 MB Flash
  - 1 MB RAM

## Connectors

- USB C
  - VBUS In, USB
- PWM
  - VBAT In, Analog Current Input, Telem RX, 4x PWM
  - JST-GH 8 Pin
- PWM EXTRA
  - 5x PWM
  - JST-SH 6 Pin
- RC INPUT
  - 5V Out, UART
  - JST-GH 4 Pin
- POWER AUX
  - 12V Out, VBAT In/Out
  - JST-GH 3 Pin
- TELEM
  - 5V Out, UART with flow control
  - JST-GH 6 Pin
- GPS
  - 5V Out, UART, I2C
  - JST-GH 6 Pin
- CAN
  - 5V Out, CAN
  - JST-GH 4 Pin
- VTX
  - 12V Out, UART TX/RX, UART RX
  - JST-GH 6 Pin
- SPI (OSD or External IMU)
  - 5V Out, SPI
  - JST-SH 8 Pin
- DEBUG
  - 3.3V Out, UART, SWD
  - JST-SH 6 Pin

## Power Requirements

- 5.5V - 54V
- 500 mA (300 mA main system, 200 mA heater)

## Additional Information

- Weight: 7.5 g g with MicroSD card
- Dimensions: 3.6 x 3.6 x 0.8 cm
- USA Built - NDAA compliant
- Heater: 1W for warming sensors in extreme cold
- LED Indicators
- MicroSD Slot

## Pinout

See the [DS-10 Pixhawk Autopilot Bus Standard](https://arkelectron.gitbook.io/ark-documentation/flight-controllers/ark-fpv/pinout)



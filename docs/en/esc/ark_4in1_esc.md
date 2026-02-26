# ARK 4IN1 ESC (with/without Connectors)

4 in 1 Electronic Speed Controller (ESC) that is made in the USA, NDAA compliant, and DIU Blue Framework listed.

The ESC comes in variants without connectors that you can solder in place, and a variant that has built-in motor and battery connectors (no soldering required).

![ARK 4IN1 ESC without connectors ](../../assets/hardware/esc/ark/ark_4_in_1_esc.jpg)![ARK 4IN1 ESC with connectors](../../assets/hardware/esc/ark/ark_4_in_1_esc_with_connectors.jpg)

## Where to Buy

Order this module from:

- [4IN1 ESC (with connectors)](https://arkelectron.com/product/ark-4in1-esc/) (ARK Electronics - US)
- [ARK Electronics (without connectors)](https://arkelectron.com/product/ark-4in1-esc-cons/) (ARK Electronics US)

## Hardware Specifications

- Battery Voltage: 3-8s
  - 6V Minimum
  - 65V Absolute Maximum
- Current Rating: 50A Continuous, 75A Burst Per Motor
- [STM32F0](https://www.st.com/en/microcontrollers-microprocessors/stm32f0-series.html)
- [AM32 Firmware](https://github.com/am32-firmware/AM32/pull/27)
- Onboard Current Sensor, Serial Telemetry
  - 100V/A
- Input Protocols
  - DShot (300, 600)
  - Bi-directional DShot
  - KISS Serial Telemetry
  - PWM
- 8 Pin JST-SH Input/Output
- 10 Pin JST-SH Debug

- Motor & Battery Connectors (with-connector version)
  - MR30 Connector Limit Per Motor: 30A Continuous, 40A Burst
  - Four MR30 Motor Connectors

- Dimensions (with connectors)
  - Size: 77.00mm x 42.00mm x 9.43mm
  - Mounting Pattern: 30.5mm
  - Weight: 24g

- Dimensions (without connectors)
  - Size: 43.00mm x 40.50mm x 7.60mm
  - Mounting Pattern: 30.5mm
  - Weight: 14.5g

Other

- Made in the USA
- Open source AM32 firmware
- [DIU Blue Framework Listed](https://www.diu.mil/blue-uas/framework)

## PX4 Configuration

The ARK 4IN1 ESC supports DShot 300/600, Bidirectional DShot, and PWM input protocols.

- **Bidirectional DShot**: Select BDShot300 or BDShot600 in the [Actuator Configuration](../config/actuators.md) to enable eRPM telemetry.
- **Extended DShot Telemetry (EDT)**: AM32 firmware supports EDT, which provides temperature, voltage, and current through the BDShot signal. Enable with `DSHOT_BIDIR_EDT=1`.
- **AM32 EEPROM Settings**: Set `DSHOT_ESC_TYPE=1` to enable reading and writing ESC firmware settings via a ground station.

See [DShot ESCs](../peripherals/dshot.md) for full setup details.

## See Also

- [ARK 4IN1 ESC CONS](https://docs.arkelectron.com/electronic-speed-controller/ark-4in1-esc) (ARK Docs)

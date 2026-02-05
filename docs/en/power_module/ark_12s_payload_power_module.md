# ARK 12S Payload Power Module

The [ARK 12S Payload Power Module](https://arkelectron.com/product/ark-12s-payload-power-module/) is a dual 5V 6A and 12V 6A power supply and digital power monitor designed for the Pixhawk Autopilot Bus Carrier boards.

This is similar to the [ARK 12S PAB Power Module](../power_module/ark_12s_pab_power_module.md) except that the additional 12V 6A supply allows easier powering of a payload.

![ARK 12S Payload Power Module](../../assets/hardware/power_module/ark_power_modules//ark_12s_payload_power.jpg)

## Where to Buy

Order this module from:

- [ARK Electronics](https://arkelectron.com/product/ark-12s-payload-power-module/) (US)

## Hardware Specifications

- **TI INA238 Digital Power Monitor**
  - 0.0001 Ohm Shunt
  - I2C Interface

- **5.2V 6A Step-Down Regulator**
  - 10V Minimum Input Voltage at 6A Out
  - Output Over-Current Protection

- **12.0V 6A Step-Down Regulator**
  - 15V Minimum Input Voltage at 6A Out
  - Output Over-Current Protection

- **75V Maximum Input Voltage**

- **Connections**
  - Solder Pads Battery Input
  - Solder Pads Battery Output
  - 6 Pin Molex CLIK-Mate Output
    - [Matches ARK PAB Carrier Power Pinout](https://arkelectron.gitbook.io/ark-documentation/flight-controllers/ark-pixhawk-autopilot-bus-carrier/pinout)
  - 4 Pin Molex CLIK-Mate 12V Output

- **Other**
  - USA Built
  - Includes 6 Pin Molex CLIK-Mate Cable

- **Additional Information**
  - Weight: 20.5 g
  - Dimensions: 3.7 cm x 3.5 cm x 1.3 cm

## PX4 Setup

- Disable the `SENS_EN_INA226` parameter if it is enabled.
- Enable the `SENS_EN_INA238` parameter.
- Reboot the flight controller.
- Set the `INA238_SHUNT` parameter to 0.0001.
- Reboot the flight controller.

## See Also

- [ARK 12S Payload Power Module Documentation](https://docs.arkelectron.com/power/ark-12s-payload-power-module) (ARK Docs)

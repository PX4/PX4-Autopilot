# DFRobot Gas concentration sensor

The driver supports DFRobot Gravity electrochemical gas sensors, including sensors for O2, CO, H2S, NH3, H2, O3, SO2, NO2, HCL, CL2, HF and PH3.


## Hardware Setup

DFRobot Gravity electrochemical gas sensors can be connected to an unused _I2C port_.

::: info
While it is possible to use those sensors with UART, the driver does not support it. Some variants of the board may required to change the operating mode by flipping a physical switch on the sensor.
:::

Build a cable following your board pinout and the DFRobot Gravity sensor pinout. You will need to connect VCC, SDA, SCL and GND pins.

| Pin | DFRobot Gravity gas sensor |
| --- | -------------------------- |
| 1   | VCC                        |
| 2   | GND                        |
| 3   | SCL                        |
| 4   | SDA                        |

::: info
The driver was tested on hardware using a DFRobot Gravity H2 sensor.
It should also support the following DFRobot Gravity electrochemical gas sensors: O2, CO, H2S, NH3, H2, O3, SO2, NO2, HCL, CL2, HF, and PH3.
:::

## Parameter Setup

The DFRobot gas sensor driver is not included in the default firmware for any board.
You must first add the driver to the firmware by enabling the following configuration option in the [PX4 Board Configuration (Kconfig)](../hardware/porting_guide_config.md#px4-menuconfig-setup).:

```plain
CONFIG_DRIVERS_GAS_SENSOR_DFROBOT_GAS=y
```
After rebuilding and flashing the firmware, enable the DFRobot Gravity electrochemical gas sensor driver using SENS_EN_DFGAS.

## Publishing

The gas concentration is published on the [SensorGasConcentration](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGasConcentration.msg) topic by default.

## Further Information

- [DFRobot Gas sensor wiki](https://wiki.dfrobot.com/)

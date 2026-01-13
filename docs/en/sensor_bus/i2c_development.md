# I2C Bus (Development Overview)

I2C is a packet-switched serial communication protocol that allows multiple master devices to connect to multiple slave devices using only 2 wires per connection.
It is intended for attaching lower-speed peripheral ICs to processors and microcontrollers in short-distance, intra-board communication.

Pixhawk/PX4 support it for:
* Connecting off board components that require higher data rates than provided by a strict serial UART, such as rangefinders.
* Compatibility with peripheral devices that only support I2C.
* Allowing multiple devices to attach to a single bus (useful for conserving ports).
  For example, LEDs, Compass, rangefinders etc.

::: info
The page [Hardware > I2C Peripherals](../sensor_bus/i2c_general.md) contains information about how to _use_ (rather than integrate) I2C peripherals and solve common setup problems.
:::

:::tip
IMUs (accelerometers/gyroscopes) should not be attached via I2C (typically the [SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus) bus is used).
The bus is not fast enough even with a single device attached to allow vibration filtering (for instance), and the performance degrades further with every additional device on the bus.
:::

## Integrating I2C Devices

Drivers should `#include <drivers/device/i2c.h>` and then provide an implementation of the abstract base class `I2C` defined in **I2C.hpp** for the target hardware (i.e. for NuttX [here](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/drivers/device/nuttx/I2C.hpp)).

A small number of drivers will also need to include headers for their type of device (**drv_*.h**) in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers) - e.g. [drv_led.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/drv_led.h). 

To include a driver in firmware you must add the driver to the board-specific cmake file that corresponds to the target you want to build for.
You can do this for a single driver:
```
CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_I2C=y
```

You can also include all drivers of a particular type.
```
CONFIG_COMMON_DISTANCE_SENSOR=y
```

:::tip
For example, you can see/search for `CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_I2C` in the [px4_fmu-v4_default](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v4/default.px4board) configuration.
:::

## I2C Driver Examples

To find I2C driver examples, search for **i2c.h** in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers).

Just a few examples are:
* [drivers/distance_sensor/lightware_laser_i2c](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_i2c) - I2C driver for [Lightware SF1XX LIDAR](../sensor/sfxx_lidar.md).
* [drivers/distance_sensor/lightware_laser_serial](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_serial) - Serial driver for [Lightware SF1XX LIDAR](../sensor/sfxx_lidar.md).
* [drivers/ms5611](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer/ms5611) - I2C Driver for the MS5611 and MS6507 barometric pressure sensor connected via I2C (or SPI).

## Further Information

* [I2C](https://en.wikipedia.org/wiki/I%C2%B2C) (Wikipedia)
* [I2C Comparative Overview](https://learn.sparkfun.com/tutorials/i2c) (learn.sparkfun.com)
* [Driver Framework](../middleware/drivers.md)

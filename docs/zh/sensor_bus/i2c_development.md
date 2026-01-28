# I2C Bus (Development Overview)

I2C 是一种分组交换串行通信协议，允许多个主设备连接到多个从属设备，每个连接只需使用2根电线。
它用于在短距离、板内通信中将低速外设 IC 连接到处理器和微控制器。

Pixhawk/PX4 支持：

- Connecting off board components that require higher data rates than provided by a strict serial UART, such as rangefinders.
- 与仅支持 I2C 的外围设备兼容。
- 允许多个设备连接到单个总线（有效保护端口）。
  例如，LED、指南针、测距仪等。

:::info
The page [Hardware > I2C Peripherals](../sensor_bus/i2c_general.md) contains information about how to _use_ (rather than integrate) I2C peripherals and solve common setup problems.
:::

:::tip
IMUs (accelerometers/gyroscopes) should not be attached via I2C (typically the [SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus) bus is used).
The bus is not fast enough even with a single device attached to allow vibration filtering (for instance), and the performance degrades further with every additional device on the bus.
:::

## 集成 I2C 设备

Drivers should `#include <drivers/device/i2c.h>` and then provide an implementation of the abstract base class `I2C` defined in **I2C.hpp** for the target hardware (i.e. for NuttX [here](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/drivers/device/nuttx/I2C.hpp)).

A small number of drivers will also need to include headers for their type of device (**drv_\*.h**) in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers) - e.g. [drv_led.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/drv_led.h).

To include a driver in firmware you must add the driver to the board-specific cmake file that corresponds to the target you want to build for.
You can do this for a single driver:

```
drivers/sf1xx
```

You can also include all drivers of a particular type.

```
CONFIG_COMMON_DISTANCE_SENSOR=y
```

:::tip
For example, you can see/search for `CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_I2C` in the [px4_fmu-v4_default](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v4/default.px4board) configuration.
:::

## I2C 驱动程序示例

To find I2C driver examples, search for **i2c.h** in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers).

Just a few examples are:

- [drivers/distance_sensor/lightware_laser_i2c](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_i2c) - I2C driver for [Lightware SF1XX LIDAR](../sensor/sfxx_lidar.md).
- [drivers/distance_sensor/lightware_laser_serial](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_serial) - Serial driver for [Lightware SF1XX LIDAR](../sensor/sfxx_lidar.md).
- [drivers/ms5611](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer/ms5611) - I2C Driver for the MS5611 and MS6507 barometric pressure sensor connected via I2C (or SPI).

## 更多信息

- [I2C](https://en.wikipedia.org/wiki/I%C2%B2C) (Wikipedia)
- [I2C Comparative Overview](https://learn.sparkfun.com/tutorials/i2c) (learn.sparkfun.com)
- [Driver Framework](../middleware/drivers.md)

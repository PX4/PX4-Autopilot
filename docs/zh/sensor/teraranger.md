# TeraRanger 测距仪

TeraRanger provide a number of lightweight distance measurement sensor based on infrared Time-of-Flight (ToF) technology.
他们通常比声纳更快、范围更大、比基于激光的系统更小、更轻。

PX4 supports:

- [TeraRanger Evo 60m](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-60m/) (0.5 – 60 m)
- [TeraRanger Evo 600Hz](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-600hz/) (0.75 - 8 m)

:::info
PX4 also supports _TeraRanger One_ (I2C adapter required).
This has been discontinued.
:::

## 购买渠道

- TBD

## 针脚定义

## 布线

All TeraRanger sensors must be connected via the I2C bus.

## 软件配置

The sensors are enabled using the parameter [SENS_EN_TRANGER](../advanced_config/parameter_reference.md#SENS_EN_TRANGER) (you can set the type of sensor or that PX4 should auto-detect the type).

:::info
If using auto-detect for Evo sensors the minimum and maximum values for the range are set to the lowest and highest possible readings across the Evo family (currently 0.5 - 60 m).
In order to use the correct max/min values the appropriate model of the Evo sensor should be set in the parameter (instead of using autodetect).
:::

:::tip
The driver for this rangefinder is usually present in firmware. If missing, you would also need to add the driver (`distance_sensor/teraranger`) to the board configuration.
:::

## 更多信息

- [Modules Reference: Distance Sensor (Driver) : teraranger](../modules/modules_driver_distance_sensor.md#teraranger)

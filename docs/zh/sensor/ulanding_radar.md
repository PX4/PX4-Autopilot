# Ainstein US-D1 标准雷达高度计

:::tip
This supersedes the _Aerotenna uLanding Radar_ (discontinued) but uses the same driver/setup.
:::

The _Ainstein_ [US-D1 Standard Radar Altimeter](https://ainstein.ai/us-d1-all-weather-radar-altimeter/) is a compact microwave rangefinder that has been optimised for use on UAVs.
With a sensing range of about 50 metres, it is useful for applications including terrain following, precision hovering (e.g. for photography), anti-collision sensing etc.
Particular advantages of this product are that it can operate effectively in all weather conditions and over all terrain types (including water).
The user manual can be found [here](https://ainstein.ai/wp-content/uploads/US-D1-Technical-User-Manual-D00.02.05.docx.pdf).

![Ainstein US-DA](../../assets/hardware/sensors/ainstein/us_d1_hero.jpg)

The rangefinder is not automatically included in most firmware, and hence cannot be used just by setting a parameter through _QGroundControl_ (as is possible with some other rangefinders).
To use it you will need to add the driver to firmware and update a configuration file to start the driver on boot.
The sections below explain how.

## 硬件安装

The rangefinder is supported by any hardware which runs a NuttX or Posix OS and which can offer a serial port for the interface.
Minimally this will include most, if not all, [Pixhawk Series](../flight_controller/pixhawk_series.md) controllers.

US-D1 can be connected to any unused _serial port_ (UART), e.g.: TELEM2, TELEM3, GPS2 etc.

## Parameter Setup

[Configure the serial port](../peripherals/serial_configuration.md) on which the lidar will run using [SENS_ULAND_CFG](../advanced_config/parameter_reference.md#SENS_ULAND_CFG).
There is no need to set the baud rate for the port, as this is configured by the driver.

::: info

If the configuration parameter is not available in _QGroundControl_ then you may need to [add the driver to the firmware](../peripherals/serial_configuration.md#parameter_not_in_firmware):

```plain
CONFIG_DRIVERS_DISTANCE_SENSOR_ULANDING_RADAR=y
```

:::

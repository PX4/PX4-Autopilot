# Benewake TFmini Lidar

The _Benewake TFmini LiDAR_ is a tiny, low cost, and low power LIDAR with 12m range.

PX4 підтримує всі три варіанти серії: TFmini-s, TFmini-i, TFmini Plus.
Ці повинні бути підключені до шини UART/серійного порту.

![TFmini LiDAR](../../assets/hardware/sensors/tfmini/tfmini_hero.jpg)

## Де купити

- [TFmini-s](https://en.benewake.com/TFminiS/index_proid_325.html)
- [TFmini-i](https://en.benewake.com/TFminii/index_proid_324.html) (industrial)
- [TFmini Plus](https://en.benewake.com/TFminiPlus/index_proid_323.html)

## Налаштування програмного забезпечення

TFmini can be connected to any unused _serial port_ (UART), such as: `TELEM2`, `TELEM3`, `GPS2` etc.

## Налаштування параметрів

[Configure the serial port](../peripherals/serial_configuration.md) on which the lidar will run using [SENS_TFMINI_CFG](../advanced_config/parameter_reference.md#SENS_TFMINI_CFG).
Не потрібно встановлювати швидкість передачі (вона зашита в програму драйвера сенсора, оскільки підтримується лише одна швидкість).

:::info
If the configuration parameter is not available in _QGroundControl_ then the [tfmini](../modules/modules_driver_distance_sensor.md#tfmini) driver may not be in firmware.
Для отримання інформації про те, як її додати, див. :

- [Serial port Configuration: Configuration Parameter Missing from QGroundControl](../peripherals/serial_configuration.md#parameter_not_in_firmware)
- [PX4 Board Configuration (Kconfig)](../hardware/porting_guide_config.md#px4-menuconfig-setup).

:::

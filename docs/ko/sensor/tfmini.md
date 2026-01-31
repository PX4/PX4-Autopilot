# Benewake TFmini 라이다

The _Benewake TFmini LiDAR_ is a tiny, low cost, and low power LIDAR with 12m range.

PX4 supports all three variants in the series: TFmini-s, TFmini-i, TFmini Plus.
These must be connected to a UART/serial bus.

![TFmini LiDAR](../../assets/hardware/sensors/tfmini/tfmini_hero.jpg)

## 구매처

- [TFmini-s](https://en.benewake.com/TFminiS/index_proid_325.html)
- [TFmini-i](https://en.benewake.com/TFminii/index_proid_324.html) (industrial)
- [TFmini Plus](https://en.benewake.com/TFminiPlus/index_proid_323.html)

## 하드웨어 설정

TFmini can be connected to any unused _serial port_ (UART), such as: `TELEM2`, `TELEM3`, `GPS2` etc.

## 매개변수 설정

[Configure the serial port](../peripherals/serial_configuration.md) on which the lidar will run using [SENS_TFMINI_CFG](../advanced_config/parameter_reference.md#SENS_TFMINI_CFG).
전송 속도를 설정할 필요가 없습니다 (단 하나의 속도만 지원되므로 센서 드라이버에 하드 코딩됨).

:::info
If the configuration parameter is not available in _QGroundControl_ then the [tfmini](../modules/modules_driver_distance_sensor.md#tfmini) driver may not be in firmware.
For information on how to add it see:

- [Serial port Configuration: Configuration Parameter Missing from QGroundControl](../peripherals/serial_configuration.md#parameter_not_in_firmware)
- [PX4 Board Configuration (Kconfig)](../hardware/porting_guide_config.md#px4-menuconfig-setup).

:::

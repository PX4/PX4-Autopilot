# Стандартний радарний висотомір Ainstein US-D1

:::tip
This supersedes the _Aerotenna uLanding Radar_ (discontinued) but uses the same driver/setup.
:::

The _Ainstein_ [US-D1 Standard Radar Altimeter](https://ainstein.ai/us-d1-all-weather-radar-altimeter/) is a compact microwave rangefinder that has been optimised for use on UAVs.
З діапазоном відчуття близько 50 метрів, він корисний для застосувань, включаючи слідування за місцевістю, точне зависання (наприклад, для фотографії), датчик запобігання зіткненням тощо.
Особливі переваги цього продукту полягають в тому, що він може ефективно працювати в усіх погодних умовах і на всіх типах місцевості (включаючи воду).
The user manual can be found [here](https://ainstein.ai/wp-content/uploads/US-D1-Technical-User-Manual-D00.02.05.docx.pdf).

![Ainstein US-DA](../../assets/hardware/sensors/ainstein/us_d1_hero.jpg)

The rangefinder is not automatically included in most firmware, and hence cannot be used just by setting a parameter through _QGroundControl_ (as is possible with some other rangefinders).
Для використання його вам потрібно додати драйвер до вбудованого програмного забезпечення та оновити файл конфігурації, щоб запустити драйвер при завантаженні.
Нижче пояснюється, як.

## Налаштування програмного забезпечення

Дальномер підтримується будь-яким обладнанням, яке працює під управлінням операційної системи NuttX або Posix і яке може пропонувати послідовний порт для інтерфейсу.
Minimally this will include most, if not all, [Pixhawk Series](../flight_controller/pixhawk_series.md) controllers.

US-D1 can be connected to any unused _serial port_ (UART), e.g.: TELEM2, TELEM3, GPS2 etc.

## Налаштування параметрів

[Configure the serial port](../peripherals/serial_configuration.md) on which the lidar will run using [SENS_ULAND_CFG](../advanced_config/parameter_reference.md#SENS_ULAND_CFG).
Немає потреби встановлювати швидкість передачі для порту, оскільки це налаштовано драйвером.

::: info

If the configuration parameter is not available in _QGroundControl_ then you may need to [add the driver to the firmware](../peripherals/serial_configuration.md#parameter_not_in_firmware):

```plain
CONFIG_DRIVERS_DISTANCE_SENSOR_ULANDING_RADAR=y
```

:::

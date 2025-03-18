# LightWare Lidar (SF1X/SF02/LW20/SF45)

LightWare розробляє широкий спектр легких, загального призначення лазерних альтиметрів ("Lidar"), які підходять для установки на БПЛА.
Ці інструменти корисні для застосувань, включаючи слідування за рельєфом, точне зависання у повітрі (наприклад, для фотографії), попередження про регуляторні висотні обмеження, антиколізійний датчик тощо.

<img src="../../assets/hardware/sensors/lidar_lightware/sf11c_120_m.jpg" width="350px" alt="LightWare SF11/C Lidar"/>![LightWare SF45 rotating Lidar](../../assets/hardware/sensors/lidar_lightware/sf45.png)

## Підтримувані плати

Наступні моделі підтримуються PX4 та можуть бути підключені до шини I2C або Serial (таблиці нижче показують, яку шину можна використовувати для кожної моделі).

| Модель                                                     | Range (m) | Шина                 | Опис                                                                                                          |
| ---------------------------------------------------------- | ---------------------------- | -------------------- | ------------------------------------------------------------------------------------------------------------- |
| [SF11/C](https://lightwarelidar.com/products/sf11-c-100-m) | 100                          | Серійна або I2C шина |                                                                                                               |
| [LW20/C](https://lightware.co.za/products/lw20-c-100-m)    | 100                          | Шина I2C             | Водонепроникний (IP67) з сервоприводом для додатків з детекцією та уникненням перешкод     |
| [SF45/B](../sensor/sf45_rotating_lidar.md)                 | 50                           | Серія                | Rotary Lidar (Used for [Collision Prevention](../computer_vision/collision_prevention.md)) |

:::details
Discontinued

The following models are supported by PX4 but are no longer available from the manufacturer.

| Модель                                                                                             | Діапазон | Шина                 |                                                                                                           |
| -------------------------------------------------------------------------------------------------- | -------- | -------------------- | --------------------------------------------------------------------------------------------------------- |
| [SF02](http://documents.lightware.co.za/SF02%20-%20Laser%20Rangefinder%20Manual%20-%20Rev%208.pdf) | 50       | Серія                |                                                                                                           |
| [SF10/A](http://documents.lightware.co.za/SF10%20-%20Laser%20Altimeter%20Manual%20-%20Rev%206.pdf) | 25       | Серійна або I2C шина |                                                                                                           |
| [SF10/B](http://documents.lightware.co.za/SF10%20-%20Laser%20Altimeter%20Manual%20-%20Rev%206.pdf) | 50       | Серійна або I2C шина |                                                                                                           |
| SF10/C                                                                                             | 100m     | Серійна або I2C шина |                                                                                                           |
| LW20/B                                                                                             | 50       | Шина I2C             | Водонепроникний (IP67) з сервоприводом для додатків з детекцією та уникненням перешкод |

:::

## Налаштування I2C

Перевірте таблиці вище, щоб підтвердити, які моделі можна підключити до порту I2C.

### Lidar Configuration (SF11/C)

The SF11/C hardware (only) does not ship with Pixhawk I2C compatibility enabled by default.
To enable support, you have to download [LightWare Studio](https://lightwarelidar.com/pages/lightware-studio) and got to **Parameters > Communication** and tick mark **I2C compatibility mode (Pixhawk)**

![LightWare SF11/C Lidar-I2C Config](../../assets/hardware/sensors/lidar_lightware/lightware_studio_i2c_config.jpg)

This step is not required for the other supported Lightware rangefinders.

### Апаратне забезпечення(Hardware)

Connect the Lidar the autopilot I2C port as shown below (in this case, for the [Pixhawk 1](../flight_controller/mro_pixhawk.md)).

![SF1XX LIDAR to I2C connection](../../assets/hardware/sensors/lidar_lightware/sf1xx_i2c.jpg)

:::info
Some older revisions cannot be used with PX4.
Specifically they may be miss-configured to have an I2C address equal to `0x55`, which conflicts with `rgbled` module.
On Linux systems you may be able to determine the address using [i2cdetect](https://linux.die.net/man/8/i2cdetect).
If the I2C address is equal to `0x66` the sensor can be used with PX4.
:::

### Parameter Setup {#i2c_parameter_setup}

Set the [SENS_EN_SF1XX](../advanced_config/parameter_reference.md#SENS_EN_SF1XX) parameter to match the rangefinder model and then reboot.

VTOL vehicles may choose to also set [SF1XX_MODE](../advanced_config/parameter_reference.md#SF1XX_MODE) to `2: Disabled during VTOL fast forward flight`.

## Serial Setup {#serial_hardware_setup}

:::tip
[SF45/B](../sensor/sf45_rotating_lidar.md) setup is covered in the linked document.
:::

### Апаратне забезпечення(Hardware)

The lidar can be connected to any unused _serial port_ (UART), e.g.: TELEM2, TELEM3, GPS2 etc.

<!-- Would be good to show serial setup! -->

### Parameter Setup {#serial_parameter_setup}

[Configure the serial port](../peripherals/serial_configuration.md) on which the lidar will run using [SENS_SF0X_CFG](../advanced_config/parameter_reference.md#SENS_SF0X_CFG).
Немає потреби встановлювати швидкість передачі для порту, оскільки це налаштовано драйвером.

:::info
If the configuration parameter is not available in _QGroundControl_ then you may need to [add the driver to the firmware](../peripherals/serial_configuration.md#parameter_not_in_firmware).
:::

Then set the [SENS_EN_SF0X](../advanced_config/parameter_reference.md#SENS_EN_SF0X) parameter to match the rangefinder model and reboot.

VTOL vehicles may choose to also set [SF1XX_MODE](../advanced_config/parameter_reference.md#SF1XX_MODE) to `2: Disabled during VTOL fast forward flight`.

## Подальша інформація

- [Modules Reference: Distance Sensor (Driver) : lightware_laser_i2c](../modules/modules_driver_distance_sensor.md#lightware-laser-i2c)
- [Modules Reference: Distance Sensor (Driver) : lightware_laser_serial](../modules/modules_driver_distance_sensor.md#lightware-laser-serial)

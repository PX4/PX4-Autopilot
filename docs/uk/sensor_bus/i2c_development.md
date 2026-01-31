# Шина I2C (Огляд розробки)

I2C - це протокол послідовного обміну даними у вигляді пакетів, який дозволяє підключати кілька головних пристроїв до кількох підчинених пристроїв, використовуючи лише 2 провідника для кожного з'єднання.
Цей протокол призначений для підключення периферійних ІС з низькою швидкістю до процесорів та мікроконтролерів у короткій дистанційній, внутрішньоплатній комунікації.

Pixhawk/PX4 підтримує її для:

- Підключення зовнішніх компонентів, які вимагають вищих швидкостей передачі даних, ніж ті, що надає строгий послідовний UART, такі як далекоміри.
- Сумісність з периферійними пристроями, які підтримують лише I2C.
- Дозвіл декількох пристроїв приєднатися до одного шини (корисно для збереження портів).
  Наприклад, світлодіоди, компас, дальномери і т. д.

:::info
The page [Hardware > I2C Peripherals](../sensor_bus/i2c_general.md) contains information about how to _use_ (rather than integrate) I2C peripherals and solve common setup problems.
:::

:::tip
IMUs (accelerometers/gyroscopes) should not be attached via I2C (typically the [SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus) bus is used).
Шина недостатньо швидка навіть з одним приєднаним пристроєм для виконання фільтрації вібрації (наприклад), і продуктивність погіршується ще більше з кожним додатковим пристроєм на шині.
:::

## Інтеграція пристроїв I2C

Drivers should `#include <drivers/device/i2c.h>` and then provide an implementation of the abstract base class `I2C` defined in **I2C.hpp** for the target hardware (i.e. for NuttX [here](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/drivers/device/nuttx/I2C.hpp)).

A small number of drivers will also need to include headers for their type of device (**drv_\*.h**) in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers) - e.g. [drv_led.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/drv_led.h).

Для включення драйвера в прошивку потрібно додати драйвер до файлу cmake, специфічного для плати, який відповідає цілі, для якої ви хочете збудувати.
Ви можете це зробити для одного драйвера:

```
CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_I2C=y
```

Ви також можете включити всі драйвери певного типу.

```
CONFIG_COMMON_DISTANCE_SENSOR=y
```

:::tip
For example, you can see/search for `CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_I2C` in the [px4_fmu-v4_default](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v4/default.px4board) configuration.
:::

## Приклади драйвера I2C

To find I2C driver examples, search for **i2c.h** in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers).

Ось декілька прикладів:

- [drivers/distance_sensor/lightware_laser_i2c](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_i2c) - I2C driver for [Lightware SF1XX LIDAR](../sensor/sfxx_lidar.md).
- [drivers/distance_sensor/lightware_laser_serial](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_serial) - Serial driver for [Lightware SF1XX LIDAR](../sensor/sfxx_lidar.md).
- [drivers/ms5611](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer/ms5611) - I2C Driver for the MS5611 and MS6507 barometric pressure sensor connected via I2C (or SPI).

## Подальша інформація

- [I2C](https://en.wikipedia.org/wiki/I%C2%B2C) (Wikipedia)
- [I2C Comparative Overview](https://learn.sparkfun.com/tutorials/i2c) (learn.sparkfun.com)
- [Driver Framework](../middleware/drivers.md)

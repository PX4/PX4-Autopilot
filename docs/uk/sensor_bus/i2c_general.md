# Периферійні пристрої шини I2C

[I2C](https://en.wikipedia.org/wiki/I2C) is a serial communication protocol that is commonly used (at least on smaller drones), for connecting peripheral components like rangefinders, LEDs, Compass, etc.

Рекомендовано для:

- Connecting offboard components that require low bandwidth and low latency communication, e.g. [rangefinders](../sensor/rangefinders.md), [magnetometers](../gps_compass/magnetometer.md), [airspeed sensors](../sensor/airspeed.md) and [tachometers](../sensor/tachometers.md).
- Сумісність з периферійними пристроями, які підтримують лише I2C.
- Можливість підключення декількох пристроїв до однієї шини, що корисно для збереження портів.

I2C дозволяє підключати декілька головних пристроїв до декількох рабочих пристроїв, використовуючи лише 2 провідника на підключення (SDA, SCL).
in theory, a bus can support 128 devices, each accessed via its unique address.

:::info
UAVCAN would normally be preferred where higher data rates are required, and on larger vehicles where sensors are mounted further from the flight controller.
:::

## Підключення

I2C використовує пару проводів: SDA (серійні дані) та SCL (серійний годинник).
Шина є типу відкритого стоку, що означає, що пристрої заземлюють лінію даних.
It uses a pull-up resistor to push it to `log.1` (idle state) - every wire has it usually located on the bus terminating devices.
Одна шина може підключати до кількох пристроїв I2C.
Індивідуальні пристрої підключені без перетину.

Для підключення (згідно зі стандартом dronecode) використовуються 4-жильні кабелі з роз'ємами JST-GH.
To ensure reliable communication and to reduce crosstalk it is advised to apply recommendations concerning [cable twisting](../assembly/cable_wiring.md#i2c-cables) and pullup resistors placement.

![Cable twisting](../../assets/hardware/cables/i2c_jst-gh_cable.jpg)

## Перевірка статусу шини та пристроїв

A useful tool for bus analysis is [i2cdetect](../modules/modules_command.md#i2cdetect).
Це список доступних пристроїв I2C за їх адресами.
Він може бути використаний для визначення доступності пристрою на шині та можливості автопілота спілкуватися з ним.

Інструмент можна запустити в терміналі PX4 за допомогою наступної команди:

```
i2cdetect -b 1
```

where the bus number is specified after `-b` parameter

## Поширені проблеми

### Конфлікти адрес

Якщо два пристрої I2C на шині мають однаковий ідентифікатор, відбудеться конфлікт, і жоден з пристроїв не буде працювати належним чином (або взагалі).
Це зазвичай трапляється тому, що користувач повинен підключити два сенсори одного типу до шини, але це також може статися, якщо пристрої використовують однакові адреси за замовчуванням.

Деякі конкретні пристрої I2C можуть дозволити вибрати нову адресу для одного з пристроїв, щоб уникнути конфлікту.
Some devices do not support this option or do not have broad options for the addresses that can be used (i.e. cannot be used to avoid a clash).

If you can't change the addresses, one option is to use an [I2C Address Translator](#i2c-address-translators).

### Недостатня пропускна здатність передачі

The bandwidth available for each device generally decreases as more devices are added. Точне зменшення залежить від пропускної здатності, використованої кожним окремим пристроєм. Therefore it is possible to connect many low-bandwidth devices, like [tachometers](../sensor/tachometers.md).
Якщо додати занадто багато пристроїв, це може призвести до помилок передачі та ненадійності мережі.

Є кілька способів зменшення проблеми:

- Dividing the devices into groups, each with approximately the same number of devices, and connecting each group to one autopilot port
- Збільшити ліміт швидкості шини (звичайно встановлений в 100кГц для зовнішнього I2C bus)

### Надмірна ємність проводки

Електрична ємність шини проводки зростає, коли додаються більше пристроїв/проводів. The exact decrease depends on the total length of bus wiring and wiring-specific capacitance.
Проблему можна проаналізувати за допомогою осцилографа, де ми бачимо, що краї сигналів SDA/SCL вже не гострі.

Є кілька способів зменшення проблеми:

- Dividing the devices into groups, each with approximately the same number of devices, and connecting each group to one autopilot port
- Using the shorter and higher quality I2C cables, see the [cable wiring page](../assembly/cable_wiring.md#i2c-cables) for details
- Separating the devices with a weak open-drain driver to smaller buses with lower capacitance by using [I2C Bus Accelerators](#i2c-bus-accelerators)

## Прискорювачі шини I2C

I2C bus accelerators are separate circuits that can be used to support longer wiring lengths on an I2C bus.
Вони працюють, фізично діливши мережу I2C на 2 частини та використовуючи свої транзистори для підсилення сигналів I2C.

Доступні прискорювачі включають:

- [Thunderfly TFI2CEXT01](https://docs.thunderfly.cz/avionics/TFI2CEXT01/):
  ![I2C bus extender](../../assets/peripherals/i2c_tfi2cext/tfi2cext01a_bottom.jpg)
  - Цей дронекод має з'єднувачі, тому це дуже легко додати до налаштування Pixhawk I2C.
  - Модуль не має налаштувань (він працює зразу після встановлення).

### I2C Level Converter function

Some I2C devices have 5V on the data lines, while the Pixhawk connector standard port expects these lines to be 3.3 V.
You can use the TFI2CEXT01 as a level converter to connect 5V devices to a Pixhawk I2C port. This feature is possible because the SCL and SDA lines of TFI2CEXT01 are 5V tolerant.

## Перетворювачі I2C адрес

Перетворювачі I2C адрес можуть використовуватися для запобігання конфліктів I2C адрес в системах, де немає іншого способу призначення унікальних адрес.
Вони працюють, слухаючи I2C комунікацію та трансформуючи адресу, коли викликається пристрій-слейв (згідно з попередньо налаштованим алгоритмом).

До підтримуваних перетворювачів I2C адрес включають:

- [Thunderfly TFI2CADT01](../sensor_bus/translator_tfi2cadt.md)
  - This has Dronecode connectors and is very easy to add to a Pixhawk I2C setup.

## I2C Bus Splitters

I2C Bus Splitters are devices that split the I2C port on your flight controller into multiple connectors.
They are useful if you want to use multiple I2C peripherals on a flight controller that has only one I2C port (or too few), such as an airspeed sensor and a distance sensor. Both devices [I2C Address Translator](../sensor_bus/translator_tfi2cadt.md) and [I2C Bus Accelerators](#i2c-bus-accelerators) could also be used as I2C splitters because they have multiple I2C connectors for connecting additional I2C devices.

## I2C Development

Software development for I2C devices is described in [I2C Bus (Development Overview)](../sensor_bus/i2c_development.md).

## Подальша інформація

- [I2C](https://en.wikipedia.org/wiki/I%C2%B2C) (Wikipedia)
- [I2C Comparative Overview](https://learn.sparkfun.com/tutorials/i2c) (learn.sparkfun.com)
- [Driver Framework](../middleware/drivers.md)

# Дальніміри TeraRanger

TeraRanger надає ряд легких сенсорів вимірювання відстані на основі інфрачервоної технології часу польоту (ToF).
Вони зазвичай швидші і мають більший діапазон, ніж ехолокатори, і менші та легші, ніж системи на основі лазера.

PX4 підтримує:

- [TeraRanger Evo 60m](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-60m/) (0.5 – 60 m)
- [TeraRanger Evo 600Hz](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-600hz/) (0.75 - 8 m)

:::info
PX4 also supports _TeraRanger One_ (I2C adapter required).
Ця функція припинена.
:::

## Де купити

- Уточнюється

## Схема розташування виводів

## Підключення

Усі сенсори TeraRanger повинні бути підключені через шину I2C.

## Конфігурація програмного забезпечення

The sensors are enabled using the parameter [SENS_EN_TRANGER](../advanced_config/parameter_reference.md#SENS_EN_TRANGER) (you can set the type of sensor or that PX4 should auto-detect the type).

:::info
If using auto-detect for Evo sensors the minimum and maximum values for the range are set to the lowest and highest possible readings across the Evo family (currently 0.5 - 60 m).
Для використання правильних максимальних/мінімальних значень слід встановити відповідну модель датчика Evo в параметрі (замість автоматичного визначення).
:::

:::tip
The driver for this rangefinder is usually present in firmware. If missing, you would also need to add the driver (`distance_sensor/teraranger`) to the board configuration.
:::

## Подальша інформація

- [Modules Reference: Distance Sensor (Driver) : teraranger](../modules/modules_driver_distance_sensor.md#teraranger)

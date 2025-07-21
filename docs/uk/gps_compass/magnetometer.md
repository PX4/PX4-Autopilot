# Магнітометр (компас) Апаратне забезпечення та налаштування

PX4 використовує магнітометр (компас) для визначення курсу та напрямку руху транспортного засобу відносно магнітного поля Землі.

[Pixhawk series](../flight_controller/pixhawk_series.md) flight controllers, and many others, include an [internal compass](#internal-compass).
This is used for automatic rotation detection of external magnetometers and for autopilot bench testing.
It should not be used otherwise, and is automatically disabled after [calibration](../config/compass.md) if an external compass is available.

На більшості апаратів, особливо на малих, ми рекомендуємо використовувати _комбінований GPS + компас_ [встановлений якомога далі від джерел живлення двигуна/ЕСК] (../assembly/mount_gps_compass.md) - як правило, на підставці або крилі (для літаків з фіксованим крилом).
Хоча ви можете використовувати [автономний зовнішній компас] (#stand-alone-compass-modules) (як вказано нижче), набагато частіше використовують [комбінований модуль GPS/компас] (#combined-gnss-compass-modules).

Магнітометри підтримують підключення до шини I2C/SPI (порти Pixhawk `GPS1` або `GPS2`) або до шини CAN.
Якщо модуль не містить "CAN" у назві, то це, ймовірно, I2C/SPI-компас.

Можна підключити до 4 внутрішніх або зовнішніх магнітометрів, хоча тільки один з них буде використовуватися як джерело курсу.
Система автоматично вибирає найкращий з доступних компасів на основі їхнього _пріоритету_ (зовнішні магнітометри мають вищий пріоритет, ніж внутрішні магнітометри).
Якщо основний компас виходить з ладу в польоті, він перемикається на наступний.
Якщо він вийде з ладу до вильоту, в приведенні в стан готовності буде відмовлено.

## Компаси, що підтримуються

### Частини компасу

PX4 можна використовувати з багатьма деталями магнітометрів, включаючи: Bosch BMM 150 MEMS (через шину I2C), HMC5883 / HMC5983 (I2C або SPI), IST8310 (I2C), LIS3MDL (I2C або SPI), RM3100 та інші.
Інші підтримувані частини магнітометра та їхні шини можна дізнатися з драйверів, перелічених у [Посилання на модулі: Магнітометр (драйвер)](../modules/modules_driver_magnetometer.md).

Ці деталі входять до складу автономних модулів компаса, комбінованих модулів компаса/ГНСС, а також до складу багатьох контролерів польоту,

### Комбіновані модулі ГНСС/компас

Список відповідних модулів див. у [Глобальні навігаційні супутникові системи (ГНСС)](../gps_compass/index.md#supported-gnss).

:::info
Якщо потрібна ГНСС, то комбінований модуль ГНСС/компас буде кращим, ніж окремі модулі, наведені нижче.
:::

### Модулі магнітного компасу автономного використання

Цей список містить самостійні модулі магнітометрів (без ГНСС).

| Пристрій                                                                                                        | Компас | DroneCan |
| :-------------------------------------------------------------------------------------------------------------- | :----: | :------: |
| [Магнітометр UAVCAN Avionics Anonymous](https://www.tindie.com/products/avionicsanonymous/uavcan-magnetometer/) |    ?   |          |
| [Компас/Магнітометр Holybro DroneCAN RM3100](https://holybro.com/products/dronecan-rm3100-compass)              | RM3100 |     ✓    |
| [RaccoonLab DroneCAN/Cyphal Magnetometer RM3100](https://holybro.com/products/dronecan-rm3100-compass)          | RM3100 |     ✓    |

Примітка:

- ✓ or a specific part number indicate that a features is supported, while ✘ or empty show that the feature is not supported.
  "?" означає "невідомо".
- Компас, який не є "DroneCAN", можна вважати SPI або I2C.

### Internal Compass

Internal compasses are not recommended for real use as a heading source, because the performance is almost always very poor.

This is particularly true on on small vehicles where the flight controller has to be mounted close to motor/ESC power lines and other sources of electromagnetic interference.
While they may be better on larger vehicles (e.g. VTOL), where it is possible to reduce electromagnetic interference by mounting the flight controller a long way from power supply lines, an external compass will almost always be better.

:::tip
They might in theory be used if there is no external magnetometer. but only with [EKF2_MAG_TYPE_INIT = Init (`6`)](../advanced_config/parameter_reference.md#EKF2_MAG_TYPE), and only if their measurements are roughly ok before arming.
:::

Internal compasses are disabled by default if an external compass is available.

## Встановлення

[Монтаж компаса](../assembly/mount_gps_compass.md) пояснює, як встановити компас або модуль GPS/компас.

## Налаштування компаса I2C/SPI

На контролерах польоту [Pixhawk Series](../flight_controller/pixhawk_series.md) ви можете підключитися до портів `GPS1` або `GPS2` (які мають виводи для I2C/SPI).
Немає потреби у додатковій конфігурації.

<!-- On flight controllers that do not follow the Pixhawk connector standard, you will need to connect to an I2C/SPI port. -->

## Налаштування компаса CAN

[DroneCAN](../dronecan/index.md) охоплює налаштування дрона для периферійних пристроїв DroneCAN, включаючи компаси.

Вам потрібно буде підключити компас до [CAN шини](../can/index.md#wiring), увімкнути DroneCAN і спеціально увімкнути магнітомери (шукайте `UAVCAN_SUB_MAG`).

## Калібрування

[Калібрування компасу](../config/compass.md) пояснює, як калібрувати всі компаси на апараті.

Процес є простим і автоматично виявляє, [встановлює обертання за замовчуванням](../advanced_config/parameter_reference.md#SENS_MAG_AUTOROT), калібрує і розставляє пріоритети для всіх підключених магнітометрів.

## Дивись також

- [Компенсація живлення компаса](../advanced_config/compass_power_compensation.md)

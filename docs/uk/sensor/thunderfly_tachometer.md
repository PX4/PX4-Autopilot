# Датчик тахометра ThunderFly TFRPM01

The [TFRPM01](https://github.com/ThunderFly-aerospace/TFRPM01) tachometer is a small, and low system demanding revolution-counter.

Сама плата не містить фактичного датчика, але може бути використана з багатьма різними типами датчиків/зондів для підрахунку обертів.
Він має роз'єм I²C для підключення до PX4 та підключений до фактичного датчика через 3-контактний роз'єм.
Також він має світлодіод, який надає базову діагностичну інформацію.

![TFRPM01A](../../assets/hardware/sensors/tfrpm/tfrpm01_electronics.jpg)

:::info
The TFRPM01 sensor is open-source hardware commercially available from [ThunderFly s.r.o.](https://www.thunderfly.cz/) (manufacturing data is [available on GitHub](https://github.com/ThunderFly-aerospace/TFRPM01)).
:::

## Налаштування програмного забезпечення

Плата обладнана (двома крізь прохідними) роз'ємами I²C для підключення до PX4 та має 3-контактний роз'єм, який можна використовувати для підключення до різних датчиків:

- TFRPM01 може бути підключений до будь-якого порту I²C.
- TFRPM01 має роз'єм з 3 контактами (з входом з підтяжкою), який може бути підключений до різних типів зондів.
  - Апаратне забезпечення датчика/зонда потребує імпульсного сигналу.
    The signal input accepts +5V TTL logic or [open collector](https://en.wikipedia.org/wiki/Open_collector) outputs.
    Максимальна частота пульса - 20 кГц з циклом роботи 50%.
  - Раз'єм зонду забезпечує живлення +5V від шини I²C, максимальна потужність, яку можна використовувати, обмежена фільтром RC (див. схеми для деталей).

Електроніка TFRPM01A обладнана сигнальним світлодіодом, який може бути використаний для перевірки правильного підключення датчика.
Світлодіод загоряється, коли вхід імпульсу заземлений або відкритий для логічного 0, тому ви можете перевірити, що датчик працює правильно, просто обертаючи ротор вручну.

### Зонд сенсора ефекту Холла

Датчики Холла (магнітно-оперовані) ідеально підходять для жорстких умов, де бруд, пил і вода можуть контактувати з відчуваним ротором.

Багато різних датчиків ефекту Холла є комерційно доступними.
For example, a 55100 Miniature Flange Mounting Proximity Sensor is a good choice.

![Example of Hall effect probe](../../assets/hardware/sensors/tfrpm/hall_probe.jpg)

### Оптичний датчик зонду

Оптичний сенсор також може бути використаний (і може бути кращим варіантом, залежно від вимог до вимірювань).
Як трансмісивний, так і рефлекторний типи сенсорів можуть бути використані для генерації імпульсів.

![Example of optical transmissive probe](../../assets/hardware/sensors/tfrpm/transmissive_probe.jpg)

## Налаштування програмного забезпечення

### Запуск драйвера

Драйвер не запускається автоматично (в будь-якій конструкції).
You will need to start it manually, either using the [QGroundControl MAVLink Console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html) or by adding the driver to the [startup script](../concept/system_startup.md#customizing-the-system-startup) on an SD card.

#### Запустіть драйвер з консолі

Start the driver from the [console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html) using the command:

```sh
pcf8583 start -X -b <bus number>
```

де:

- `-X` means that it is an external bus.
- `<bus number>` is the bus number to which the device is connected

:::info
The bus number in code `-b <bus number>` may not match the bus labels on the autopilot.
Наприклад, при використанні CUAV V5+ або CUAV Nano:

| Мітка автопілоту | -b номер |
| ---------------- | -------- |
| I2C1             | -X -b 4  |
| I2C2             | -X -b 2  |
| I2C3             | -X -b 1  |

The `pcf8583 start` command outputs the corresponding autopilot bus name/label for each bus number.
:::

### Тестування

Ви можете перевірити, що лічильник працює, використовуючи кілька методів

#### PX4 (NuttX) Консоль MAVLink

The [QGroundControl MAVLink Console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html) can also be used to check that the driver is running and the UORB topics it is outputting.

Щоб перевірити статус драйвера TFRPM01, виконайте команду:

```sh
pcf8583 status
```

Якщо драйвер працює, порт I²C буде надруковано разом з іншими основними параметрами запущеного екземпляру.
Якщо драйвер не працює, його можна запустити за допомогою процедури, описаної вище.

The [listener](../modules/modules_command.md#listener) command allows you to monitor RPM UORB messages from the running driver.

```sh
listener rpm
```

For periodic display, you can add `-n 50` parameter after the command, which prints the next 50 messages.

#### QGroundControl MAVLink Inspector

The QGroundControl [Mavlink Inspector](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html) can be used to observe MAVLink messages from PX4, including [RAW_RPM](https://mavlink.io/en/messages/common.html#RAW_RPM) emitted by the driver:

1. Start the inspector from the QGC menu: **Analyze tools > Mavlink Inspector**
2. Check that `RAW_RPM` is present in the list of messages (if it is missing, check that the driver is running).

### Налаштування параметрів

Зазвичай, сенсори можуть бути використані без конфігурації, але значення обертів на хвилину повинні відповідати кратними реальним обертам. It is because the `PCF8583_MAGNET` parameter needs to correspond to the real number of pulses per single revolution of the sensed rotor.
Якщо потрібно, наступні параметри слід налаштувати:

- [PCF8583_POOL](../advanced_config/parameter_reference.md#PCF8583_POOL) — pooling interval between readout the counted number
- [PCF8583_RESET](../advanced_config/parameter_reference.md#PCF8583_RESET) — Counter value where the counted number should be reset to zero.
- [PCF8583_MAGNET](../advanced_config/parameter_reference.md#PCF8583_MAGNET) — Number of pulses per revolution e.g. number of magnets at a rotor disc.

:::info
The parameters above appear in QGC after the driver/PX4 are restarted.

Якщо параметри конфігурації не доступні після перезапуску, то вам слід перевірити, чи відбувся запуск драйвера.
It may be that the [driver is not present in the firmware](../peripherals/serial_configuration.md#configuration-parameter-missing-from-qgroundcontrol), in which case it must be added to the board configuration:

```sh
drivers/rpm/pcf8583
```

:::

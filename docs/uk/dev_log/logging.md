# Логування

The [system logger](../modules/modules_system.md#logger) is able to log any ORB topic with all included fields.
Everything necessary is generated from the `.msg` file, so that only the topic name needs to be specified.
Необов'язковий параметр інтервалу визначає максимальну швидкість ведення журналу певної теми.
Усі існуючі екземпляри теми реєструються.

The output log format is [ULog](../dev_log/ulog_file_format.md).

[Encrypted logging](../dev_log/log_encryption.md) is also supported.

## Використання

За замовчуванням, реєстрація автоматично починається при взбиранні на охорону, і зупиняється при знятті з охорони.
Для кожної сесії готовності на SD-картці створюється новий файл журналу.
To display the current state, use `logger status` on the console.
If you want to start logging immediately, use `logger on`.
Це скасовує стан готовності, якщо система була увімкнена.
`logger off` undoes this.

If logging stops due to a write error, or reaching the [maximum file size](#file-size-limitations), PX4 will automatically restart logging in a new file.

Для отримання списку всіх підтримуваних команд та параметрів реєстратора використовуйте:

```
logger help
```

## Налаштування

The logging system is configured by default to collect sensible logs for [flight reporting](../getting_started/flight_reporting.md) with [Flight Review](http://logs.px4.io).

Logging may further be configured using the [SD Logging](../advanced_config/parameter_reference.md#sd-logging) parameters.
Параметри, які ви найімовірніше зміните, перераховані нижче.

| Параметр                                                                                      | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| --------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [SDLOG_MODE](../advanced_config/parameter_reference.md#SDLOG_MODE)       | Журналювання. Defines when logging starts and stops.<br />- `-1`: Logging disabled.<br />- `0`: Log when armed until disarm (default).<br />- `1`: Log from boot until disarm.<br />- `2`: Log from boot until shutdown.<br />- `3`: Log based on the [AUX1 RC channel](../advanced_config/parameter_reference.md#RC_MAP_AUX1).<br />- `4`: Log from first armed until shutdown. |
| [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE) | Профіль ведення журналу. Use this to enable less common logging/analysis (e.g. for EKF2 replay, high rate logging for PID & filter tuning, thermal temperature calibration).                                                                                                                                                                                                                                                                                                                                                 |
| [SDLOG_MISSION](../advanced_config/parameter_reference.md#SDLOG_MISSION) | Create very small additional "Mission Log".<br>This log can _not_ be used with [Flight Review](../log/flight_log_analysis.md#flight-review-online-tool), but is useful when you need a small log for geotagging or regulatory compliance.                                                                                                                                                                                                                                                                                                                                                           |

Корисні налаштування для конкретних випадків:

- Raw sensor data for comparison: [SDLOG_MODE=1](../advanced_config/parameter_reference.md#SDLOG_MODE) and [SDLOG_PROFILE=64](../advanced_config/parameter_reference.md#SDLOG_PROFILE).
- Disabling logging altogether: [SDLOG_MODE=`-1`](../advanced_config/parameter_reference.md#SDLOG_MODE)

### Модуль реєстрації

_Developers_ can further configure what information is logged via the [logger](../modules/modules_system.md#logger) module.
Це дозволяє, наприклад, реєструвати ваші власні теми uORB.

### Конфігурація SD-карти

Окремо, список зареєстрованих тем також може бути налаштований за допомогою файлу на картці SD.
Create a file `etc/logging/logger_topics.txt` on the card with a list of topics (For SITL, it's `build/px4_sitl_default/rootfs/fs/microsd/etc/logging/logger_topics.txt`):

```plain
<topic_name> <interval> <instance>
```

The `<interval>` is optional, and if specified, defines the minimum interval in ms between two logged messages of this topic.
Якщо не вказано, тема реєструється з повною швидкістю.

The `<instance>` is optional, and if specified, defines the instance to log.
Якщо не вказано, всі екземпляри теми реєструються.
To specify `<instance>`, `<interval>` must be specified. Може бути встановлено на 0 для реєстрації з повною швидкістю

Теми в цьому файлі замінюють всі теми за замовчуванням, які були зареєстровані.

Приклади :

```plain
sensor_accel 0 0
sensor_accel 100 1
sensor_gyro 200
sensor_mag 200 1
```

Ця конфігурація буде реєструвати sensor_accel 0 з повною швидкістю, sensor_accel 1 з частотою 10 Гц, всі екземпляри sensor_gyro з частотою 5 Гц та sensor_mag 1 з частотою 5 Гц.

## Скрипти

There are several scripts to analyze and convert logging files in the [pyulog](https://github.com/PX4/pyulog) repository.

## Обмеження розміру файлу

Максимальний розмір файлу залежить від файлової системи та ОС.
Розмір обмеження на NuttX наразі становить близько 2 ГБ.

## Відключення

Втрати логування небажані, і є кілька факторів, що впливають на кількість втрат:

- Більшість SD-карт, які ми тестували, проявляють кілька пауз кожну хвилину.
  Це проявляється як кілька 100 мс затримка під час команди запису.
  Це призводить до відключення, якщо буфер запису заповнюється протягом цього часу.
  Цей ефект залежить від SD-карти (див. нижче).
- Форматування SD-карти може допомогти у запобіганні викидань.
- Збільшення буфера журналу допомагає.
- Decrease the logging rate of selected topics or remove unneeded topics from being logged (`info.py <file>` is useful for this).

## SD-карти

Максимальний підтримуваний розмір SD-карти для NuttX - 32 ГБ (Специфікація карт пам’яті SD, версія 2.0).
The **SanDisk Extreme U3 32GB** and **Samsung EVO Plus 32** are known to be reliable cards (do not exhibit write-time spikes, and thus virtually no dropouts).

The table below shows the **mean sequential write speed [KB/s]** / **maximum write time per block (average) [ms]** for F4- (Pixracer), F7-, and H7-based flight controllers.

| SD-карта                                                                         | F4            | F7         | H7        |
| -------------------------------------------------------------------------------- | ------------- | ---------- | --------- |
| SanDisk Extreme U3 32GB                                                          | 1500 / **15** | 1800/10    | 2900/8    |
| Samsung EVO Plus 32GB                                                            | 1700/10-60    | 1800/10-60 | 1900/9-60 |
| Sandisk Ultra Class 10 8GB                                                       | 348 / 40      | ?/?        | ?/?       |
| Sandisk Class 4 8GB                                                              | 212 / 60      | ?/?        | ?/?       |
| SanDisk Class 10 32 GB (High Endurance Video Monitoring Card) | 331 / 220     | ?/?        | ?/?       |
| Lexar U1 (Class 10), 16GB High-Performance                    | 209 / 150     | ?/?        | ?/?       |
| Sandisk Ultra PLUS Class 10 16GB                                                 | 196 /500      | ?/?        | ?/?       |
| Sandisk Pixtor Class 10 16GB                                                     | 334 / 250     | ?/?        | ?/?       |
| Sandisk Extreme PLUS Class 10 32GB                                               | 332 / 150     | ?/?        | ?/?       |

Запис пропускної здатності зі стандартними темами становить близько 50 КБ/с, що задовольняє майже всі SD-карти у термінах їх середньої послідовної швидкості запису.

More important than the mean write speed is spikes (or generally high values) in the maximum write time per block (of 4 KB) or `fsync` times, as a long write time means a larger log buffer is needed to avoid dropouts.

PX4 використовує більші буфери на F7/H7 та кешування читання, що достатньо компенсує піки на багатьох поганих картках.
That said, if your card has an `fsync` or write duration of several 100ms it is should not be preferred for use with PX4.
You can check the value by running [sd_bench](../modules/modules_command.md#sd-bench) should be run with more iterations (around 100 should do).

```sh
sd_bench -r 100
```

Це визначає мінімальний розмір буфера: чим більше це максимальне значення, тим більше потрібно мати розмір буфера журналу, щоб уникнути втрат даних.
PX4 використовує більші буфери на F7/H7 та кешування читання, щоб компенсувати деякі з цих проблем.

:::info
If you have concerns about a particular card you can run the above test and report the results to https://github.com/PX4/PX4-Autopilot/issues/4634.
:::

## Потокове ведення журналу

Традиційний і все ще повністю підтримуваний спосіб ведення журналу - використання SD-карти на FMU.
Однак є альтернатива, потокове ведення журналу, яке надсилає ті ж дані журналювання через MAVLink.
Цей метод може бути використаний, наприклад, у випадках, коли FMU не має слоту для SD-карти (наприклад, Intel® Aero Ready to Fly Drone) або просто для уникнення проблем з SD-картами.
Обидва методи можуть бути використані незалежно один від одного і одночасно.

Вимога полягає в тому, що посилання забезпечує принаймні ~50KB/s, наприклад, WiFi-посилання.
І тільки один клієнт може запитати потік журналування одночасно.
Підключення не потребує надійності, протокол розроблений для обробки втрат.

Існують різні клієнти, які підтримують потокову передачу ulog:

- `mavlink_ulog_streaming.py` script in PX4-Autopilot/Tools.
- QGroundControl:
  ![QGC Log Streaming](../../assets/gcs/qgc-log-streaming.png)
- [MAVGCL](https://github.com/ecmnet/MAVGCL)

### Діагностика

- If log streaming does not start, make sure the `logger` is running (see above), and inspect the console output while starting.
- Якщо це все ще не працює, переконайтеся, що використовується MAVLink 2.
  Enforce it by setting `MAV_PROTO_VER` to 2.
- Log streaming uses a maximum of 70% of the configured MAVLink rate (`-r` parameter).
  Якщо потрібно більше, повідомлення видаляються.
  The currently used percentage can be inspected with `mavlink status` (1.8% is used in this example):

  ```sh
  instance #0:
          GCS heartbeat:  160955 us ago
          mavlink chan: #0
          type:           GENERIC LINK OR RADIO
          flow control:   OFF
          rates:
          tx: 95.781 kB/s
          txerr: 0.000 kB/s
          rx: 0.021 kB/s
          rate mult: 1.000
          ULog rate: 1.8% of max 70.0%
          accepting commands: YES
          MAVLink version: 2
          transport protocol: UDP (14556)
  ```

  Also make sure `txerr` stays at 0.
  Якщо це піде вгору, або буфер відправлення NuttX занадто малий, або фізичний зв'язок насичений, або апаратне забезпечення занадто повільне для обробки даних.

## Дивіться також

- [Encrypted logging](../dev_log/log_encryption.md)
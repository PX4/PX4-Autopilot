# Формат файлу ULog

ULog - це формат файлу, що використовується для логування повідомлень. The format is self-describing, i.e. it contains the format and [uORB](../middleware/uorb.md) message types that are logged.
Цей документ призначений для документації специфікації формату файлу ULog.
Це призначено особливо для тих, хто зацікавлений у написанні розбірника / серіалізатора ULog та потребує розкодувати / закодувати файли.

PX4 використовує ULog для ведення журналу тем uORB як повідомлення, пов'язані з (але не обмежені) наступними джерелами:

- **Device inputs:** Sensors, RC input, etc.
- **Internal states:** CPU load, attitude, EKF state, etc.
- **String messages:** `printf` statements, including `PX4_INFO()` and `PX4_ERR()`.

The format uses [little endian](https://en.wikipedia.org/wiki/Endianness) memory layout for all binary types (the least significant byte (LSB) of data type is placed at the lowest memory address).

## Типи даних

Для ведення журналу використовуються наступні типи двійкових даних. Вони всі відповідають типам у мові програмування C.

| Тип                                                         | Розмір у байтах |
| ----------------------------------------------------------- | --------------- |
| int8_t, uint8_t   | 1               |
| int16_t, uint16_t | 2               |
| int32_t, uint32_t | 4               |
| int64_t, uint64_t | 8               |
| float                                                       | 4               |
| double                                                      | 8               |
| bool, char                                                  | 1               |

Additionally the types can be used as a fixed-size array: e.g. `float[5]`.

Strings (`char[length]`) do not contain the termination NULL character `'\0'` at the end.

:::info
String comparisons are case sensitive, which should be taken into account when comparing message names when [adding subscriptions](#a-subscription-message).
:::

## Структура файлу ULog

Файли ULog мають наступні три розділи:

```
----------------------
|       Header       |
----------------------
|    Definitions     |
----------------------
|        Data        |
----------------------
```

Опис кожного розділу наведено нижче.

### Розділ заголовка

Заголовок є розділом фіксованого розміру та має наступний формат (16 байт):

```plain
----------------------------------------------------------------------
| 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x01         | uint64_t       |
| File magic (7B)                    | Version (1B) | Timestamp (8B) |
----------------------------------------------------------------------
```

- **File Magic (7 Bytes):** File type indicator that reads "ULogXYZ where XYZ is the magic bytes sequence `0x01 0x12 0x35`"
- **Version (1 Byte):** File format version (currently 1)
- **Timestamp (8 Bytes):** `uint64_t` integer that denotes when the logging started in microseconds.

### Definition & Data Section Message Header

The _Definitions and Data_ sections contain a number of **messages**. Кожне повідомлення передує цим заголовком:

```c
struct message_header_s {
  uint16_t msg_size;
  uint8_t msg_type;
};
```

- `msg_size` is the size of the message in bytes without the header.
- `msg_type` defines the content, and is a single byte.

:::info
Message sections below are prefixed with the character that corresponds to it's `msg_type`.
:::

### Розділ визначень

Розділ визначень містить основну інформацію, таку як версія програмного забезпечення, формат повідомлення, початкові значення параметрів тощо.

Основні типи повідомлень в цьому розділі є:

1. [Flag Bits](#b-flag-bits-message)
2. [Format Definition](#f-format-message)
3. [Information](#i-information-message)
4. [Multi Information](#m-multi-information-message)
5. [Parameter](#p-parameter-message)
6. [Default Parameter](#q-default-parameter-message)

#### 'B': Повідомлення флагів бітів

:::info
This message must be the **first message** right after the header section, so that it has a fixed constant offset from the start of the file!
:::

Це повідомлення надає інформацію лог-аналізатору про те, чи можна аналізувати журнал.

```c
struct ulog_message_flag_bits_s {
  struct message_header_s header; // msg_type = 'B'
  uint8_t compat_flags[8];
  uint8_t incompat_flags[8];
  uint64_t appended_offsets[3]; // file offset(s) for appended data if appending bit is set
};
```

- `compat_flags`: compatible flag bits

  - Ці прапорці вказують на наявність функцій у файлі журналу, які сумісні з будь-яким парсером ULog.
  - `compat_flags[0]`: _DEFAULT_PARAMETERS_ (Bit 0): if set, the log contains [default parameters message](#q-default-parameter-message)

  Решта бітів наразі не визначені і повинні бути встановлені на 0.
  Ці біти можуть бути використані для майбутніх змін ULog, які сумісні з існуючими парсерами.
  Наприклад, додавання нового типу повідомлення може бути вказане шляхом визначення нового біта у стандарті, а існуючі парсери ігноруватимуть новий тип повідомлення.
  Це означає, що парсери можуть просто ігнорувати біти, якщо один з невідомих бітів встановлений.

- `incompat_flags`: incompatible flag bits.

  - `incompat_flags[0]`: _DATA_APPENDED_ (Bit 0): if set, the log contains appended data and at least one of the `appended_offsets` is non-zero.

  Решта бітів наразі не визначені і повинні бути встановлені на 0.
  Це може бути використано для введення руйнівних змін, які існуючі парсери не можуть обробити. For example, when an old ULog parser that didn't have the concept of _DATA_APPENDED_ reads the newer ULog, it would stop parsing the log as the log will contain out-of-spec messages / concepts.
  Якщо парсер виявляє, що будь-який з цих бітів встановлений, що не вказано, він повинен відмовитися від аналізу журналу.

- `appended_offsets`: File offset (0-based) for appended data.
  Якщо даних не додається, всі зміщення повинні бути нульовими.
  Це може бути використано для надійного додавання даних до журналів, які можуть припинитись посеред повідомлення.
  Наприклад, дампи збоїв.

  Процес додавання даних повинен виконати:

  - set the relevant `incompat_flags` bit
  - set the first `appended_offsets` that is currently 0 to the length of the log file without the appended data, as that is where the new data will start
  - додайте будь-який тип повідомлень, які є дійсними для розділу Дані.

Можливо, що у майбутніх специфікаціях ULog буде додано ще декілька полів в кінці цього повідомлення.
Це означає, що парсер не повинен припускати фіксовану довжину цього повідомлення.
If the `msg_size` is bigger than expected (currently 40), any additional bytes must be ignored/discarded.

#### 'F': Формат повідомлення

Формат повідомлення визначає одне ім'я повідомлення та його внутрішні поля в одному рядку.

```c
struct message_format_s {
  struct message_header_s header; // msg_type = 'F'
  char format[header.msg_size];
};
```

- `format` is a plain-text string with the following format: `message_name:field0;field1;`
  - There can be an arbitrary amount of fields (minimum 1), separated by `;`.
  - `message_name`: an arbitrary non-empty string with these allowed characters: `a-zA-Z0-9_-/` (and different from any of the [basic types](#data-types)).

A `field` has the format: `type field_name`, or for an array: `type[array_length] field_name` is used (only fixed size arrays are supported).
`field_name` must consist of the characters in the set `a-zA-Z0-9_`.

A `type` is one of the [basic binary types](#data-types) or a `message_name` of another format definition (nested usage).

- Тип може бути використаний до того, як він буде визначений.
  - e.g. The message `MessageA:MessageB[2] msg_b` can come before the `MessageB:uint_8[3] data`
- There can be arbitrary nesting but **no circular dependencies**
  - e.g. `MessageA:MessageB[2] msg_b` & `MessageB:MessageA[4] msg_a`

Деякі назви полів є спеціальними:

- `timestamp`: every message format with a [Subscription Message](#a-subscription-message) must include a timestamp field (for example a message format only used as part of a nested definition by another format may not include a timestamp field)
  - Its type must be `uint64_t`.
  - Одиниця вимірювання - мікросекунди.
  - The timestamp must always be monotonic increasing for a message series with the same `msg_id` (same subscription).
- `_padding{}`: field names that start with `_padding` (e.g. `_padding[3]`) should not be displayed and their data must be ignored by a reader.
  - Ці поля можуть бути вставлені письменником для забезпечення правильного вирівнювання.
  - Якщо поле відступу є останнім полем, тоді це поле може не бути зареєстроване, щоб уникнути запису непотрібних даних.
  - This means the `message_data_s.data` will be shorter by the size of the padding.
  - Однак відступ все ще потрібен, коли повідомлення використовується во вкладеному визначенні.
- Загалом, поля повідомлень не обов'язково вирівняні (тобто зсув поля всередині повідомлення не обов'язково є кратним його розміру даних), тому читач завжди повинен використовувати відповідні методи копіювання пам'яті для доступу до окремих полів.

#### 'I': Інформаційне повідомлення

The Information message defines a dictionary type definition `key` : `value` pair for any information, including but not limited to Hardware version, Software version, Build toolchain for the software, etc.

```c
struct ulog_message_info_header_s {
  struct message_header_s header; // msg_type = 'I'
  uint8_t key_len;
  char key[key_len];
  char value[header.msg_size-1-key_len]
};
```

- `key_len`: Length of the key value
- `key`: Contains the key string in the form`type name`, e.g. `char[value_len] sys_toolchain_ver`. Valid characters for the name: `a-zA-Z0-9_-/`. The type may be one of the [basic types including arrays](#data-types).
- `value`: Contains the data (with the length `value_len`) corresponding to the `key` e.g. `9.4.0`.

:::info
A key defined in the Information message must be unique. Означає, що не повинно бути більше одного визначення з таким самим ключовим значенням.
:::

Парсери можуть зберігати інформаційні повідомлення у вигляді словника.

Попередньо визначені інформаційні повідомлення:

| key                                 | Опис                                                                                                | Приклад для значення                                              |
| ----------------------------------- | --------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| `char[value_len] sys_name`          | Назва системи                                                                                       | "PX4"                                                             |
| `char[value_len] ver_hw`            | Версія апаратного забезпечення (плата)                                           | "PX4FMU_V4"                                  |
| `char[value_len] ver_hw_subtype`    | Підстава плати (варіація)                                                        | "V2"                                                              |
| `char[value_len] ver_sw`            | Версія програмного забезпечення (git tag)                                        | "7f65e01"                                                         |
| `char[value_len] ver_sw_branch`     | git branch                                                                                          | "master"                                                          |
| `uint32_t ver_sw_release`           | Версія програмного забезпечення (див. нижче)                     | 0x010401ff                                                        |
| `char[value_len] sys_os_name`       | Назва операційної системи                                                                           | "Linux"                                                           |
| `char[value_len] sys_os_ve`r        | Версія ОС (git tag)                                                              | "9f82919"                                                         |
| `uint32_t ver_os_release`           | Версія ОС (див. нижче)                                           | 0x010401ff                                                        |
| `char[value_len] sys_toolchain`     | Назва набору інструментів                                                                           | "GNU GCC"                                                         |
| `char[value_len] sys_toolchain_ver` | Версія інструментального набору                                                                     | "6.2.1"                           |
| `char[value_len] sys_mcu`           | Назва мікросхеми та ревізія                                                                         | "STM32F42x, rev A"                                                |
| `char[value_len] sys_uuid`          | Унікальний ідентифікатор для транспортного засобу (наприклад, ідентифікатор MCU) | "392a93e32fa3"... |
| `char[value_len] log_type`          | Тип журналу (повний журнал, якщо не вказано)                                     | "mission"                                                         |
| `char[value_len] replay`            | Ім'я файлу відтвореного журналу, якщо в режимі відтворення                                          | "log001.ulg"                                      |
| `int32_t time_ref_utc`              | Зсув часу UTC в секундах                                                                            | -3600                                                             |

:::info
`value_len` represents the data size of the `value`. This is described in the `key`.
:::

- The format of `ver_sw_release` and `ver_os_release` is: 0xAABBCCTT, where AA is **major**, BB is **minor**, CC is patch and TT is the **type**.
  - **Type** is defined as following: `>= 0`: development, `>= 64`: alpha version, `>= 128`: beta version, `>= 192`: RC version, `== 255`: release version.
  - For example, `0x010402FF` translates into the release version v1.4.2.

Це повідомлення також може бути використано в розділі Дані (це, однак, переважний розділ).

#### 'M': Багатоінформаційне повідомлення

Повідомлення з багатою інформацією слугує такій же меті, як і повідомлення інформації, але для довгих повідомлень або кількох повідомлень з одним і тим же ключем.

```c
struct ulog_message_info_multiple_header_s {
  struct message_header_s header; // msg_type = 'M'
  uint8_t is_continued; // can be used for arrays
  uint8_t key_len;
  char key[key_len];
  char value[header.msg_size-2-key_len]
};
```

- `is_continued` can be used for split-up messages: if set to 1, it is part of the previous message with the same key.

Парсери можуть зберігати всю інформацію про багато повідомлень у вигляді 2D-списку, використовуючи той самий порядок, що й повідомлення в лог-файлі.

Дійсні імена та типи такі ж, як для повідомлення Інформація.

#### 'P': Повідомлення параметра

Parameter message in the _Definitions_ section defines the parameter values of the vehicle when logging is started. It uses the same format as the [Information Message](#i-information-message).

```c
struct message_info_s {
  struct message_header_s header; // msg_type = 'P'
  uint8_t key_len;
  char key[key_len];
  char value[header.msg_size-1-key_len]
};
```

If a parameter dynamically changes during runtime, this message can also be [used in the Data section](#messages-shared-with-the-definitions-section) as well.

The data type is restricted to `int32_t` and `float`. Valid characters for the name: `a-zA-Z0-9_-/`.

#### 'Q': Параметр повідомлення за замовчуванням

Повідомлення параметра за замовчуванням визначає значення параметра для вказаного транспортного засобу та налаштувань.

```c
struct ulog_message_parameter_default_header_s {
  struct message_header_s header; // msg_type = 'Q'
  uint8_t default_types;
  uint8_t key_len;
  char key[key_len];
  char value[header.msg_size-2-key_len]
};
```

- `default_types` is a bitfield and defines to which group(s) the value belongs to.
  - Принаймні один біт повинен бути встановлений:
    - `1<<0`: system wide default
    - `1<<1`: default for the current configuration (e.g. an airframe)

Журнал не може містити значень за замовчуванням для всіх параметрів.
У цих випадках значення за замовчуванням дорівнює значенню параметра, а різні типи за замовчуванням розглядаються незалежно один від одного.

Це повідомлення також може бути використане в розділі Дані, і застосовується той самий тип даних та найменування, як і для повідомлення Параметр.

This section ends before the start of the first [Subscription Message](#a-subscription-message) or [Logging](#l-logged-string-message) message, whichever comes first.

### Розділ даних

The message types in the _Data_ section are:

1. [Subscription](#a-subscription-message)
2. [Unsubscription](#r-unsubscription-message)
3. [Logged Data](#d-logged-data-message)
4. [Logged String](#l-logged-string-message)
5. [Tagged Logged String](#c-tagged-logged-string-message)
6. [Synchronization](#s-synchronization-message)
7. [Dropout Mark](#o-dropout-message)
8. [Information](#i-information-message)
9. [Multi Information](#m-multi-information-message)
10. [Parameter](#p-parameter-message)
11. [Default Parameter](#q-default-parameter-message)

#### `A`: Subscription Message

Subscribe a message by name and give it an id that is used in [Logged data Message](#d-logged-data-message).
This must come before the first corresponding [Logged data Message](#d-logged-data-message).

```c
struct message_add_logged_s {
  struct message_header_s header; // msg_type = 'A'
  uint8_t multi_id;
  uint16_t msg_id;
  char message_name[header.msg_size-3];
};
```

- `multi_id`: the same message format can have multiple instances, for example if the system has two sensors of the same type. Стандартне та перше значення повинно бути 0.
- `msg_id`: unique id to match [Logged data Message](#d-logged-data-message) data. Перше використання повинно встановити це на 0, а потім збільшувати його.
  - The same `msg_id` must not be used twice for different subscriptions.
- `message_name`: message name to subscribe to.
  Must match one of the [Format Message](#f-format-message) definitions.

#### `R`: Unsubscription Message

Відмовитися від повідомлення, щоб позначити, що воно більше не буде реєструватися (зараз не використовується).

```c
struct message_remove_logged_s {
  struct message_header_s header; // msg_type = 'R'
  uint16_t msg_id;
};
```

#### 'D': Повідомлення про зареєстровані дані

```c
struct message_data_s {
  struct message_header_s header; // msg_type = 'D'
  uint16_t msg_id;
  uint8_t data[header.msg_size-2];
};
```

- `msg_id`: as defined by a [Subscription Message](#a-subscription-message)
- `data` contains the logged binary message as defined by [Format Message](#f-format-message)

Див. вище для спеціальної обробки полів відступу.

#### 'L': Повідомлення про зареєстрований рядок

Logged string message, i.e. `printf()` output.

```c
struct message_logging_s {
  struct message_header_s header; // msg_type = 'L'
  uint8_t log_level;
  uint64_t timestamp;
  char message[header.msg_size-9]
};
```

- `timestamp`: in microseconds
- `log_level`: same as in the Linux kernel:

| Назва   | Значення рівня | Значення                           |
| ------- | -------------- | ---------------------------------- |
| EMERG   | '0'            | Система непридатна до використання |
| ALERT   | '1'            | Дії повинні бути вжиті негайно     |
| CRIT    | '2'            | Критичні умови                     |
| ERR     | '3'            | Умови помилки                      |
| WARNING | '4'            | Умови попередження                 |
| NOTICE  | '5'            | Нормальний, але значущий стан      |
| INFO    | '6'            | Інформаційний                      |
| DEBUG   | '7'            | Повідомлення рівня налагодження    |

#### 'C': позначене зареєстроване рядкове повідомлення

```c
struct message_logging_tagged_s {
  struct message_header_s header; // msg_type = 'C'
  uint8_t log_level;
  uint16_t tag;
  uint64_t timestamp;
  char message[header.msg_size-11]
};
```

- `tag`: id representing source of logged message string. Це може представляти процес, потік або клас в залежності від архітектури системи.

  - For example, a reference implementation for an onboard computer running multiple processes to control different payloads, external disks, serial devices etc can encode these process identifiers using a `uint16_t enum` into the `tag` attribute of struct as follows:

  ```c
  enum class ulog_tag : uint16_t {
    unassigned,
    mavlink_handler,
    ppk_handler,
    camera_handler,
    ptp_handler,
    serial_handler,
    watchdog,
    io_service,
    cbuf,
    ulg
  };
  ```

- `timestamp`: in microseconds

- `log_level`: same as in the Linux kernel:

| Назва   | Значення рівня | Значення                           |
| ------- | -------------- | ---------------------------------- |
| EMERG   | '0'            | Система непридатна до використання |
| ALERT   | '1'            | Дії повинні бути вжиті негайно     |
| CRIT    | '2'            | Критичні умови                     |
| ERR     | '3'            | Умови помилки                      |
| WARNING | '4'            | Умови попередження                 |
| NOTICE  | '5'            | Нормальний, але значущий стан      |
| INFO    | '6'            | Інформаційний                      |
| DEBUG   | '7'            | Повідомлення рівня налагодження    |

#### 'S': Повідомлення синхронізації

Повідомлення синхронізації, щоб читач міг відновитися від пошкодженого повідомлення, шукаючи наступне повідомлення синхронізації.

```c
struct message_sync_s {
  struct message_header_s header; // msg_type = 'S'
  uint8_t sync_magic[8];
};
```

- `sync_magic`: [0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12]

#### 'O': Повідомлення про відключення

Позначте відсутність (втрачені повідомлення журналювання) заданої тривалості в мс.

Відключення можуть виникати, наприклад, якщо пристрій не є достатньо швидким.

```c
struct message_dropout_s {
  struct message_header_s header; // msg_type = 'O'
  uint16_t duration;
};
```

#### Повідомлення, розділені розділом визначень

Оскільки Розділи Визначень та Дані використовують той же формат заголовка повідомлення, вони також діляться тими ж повідомленнями, які перераховані нижче:

- [Information Message](#i-information-message).
- [Multi Information Message](#m-multi-information-message)
- [Parameter Message](#p-parameter-message)
  - For the _Data_ section, the Parameter Message is used when the parameter value changes
- [Default Parameter Message](#q-default-parameter-message)

## Вимоги до Parsers

Дійсний розбірник ULog повинен відповідати наступним вимогам:

- Повинен ігнорувати невідомі повідомлення (але може вивести попередження)
- Розбирайте майбутні/невідомі версії формату файлу також (але може вивести попередження).
- Must refuse to parse a log which contains unknown incompatibility bits set (`incompat_flags` of [Flag Bits Message](#b-flag-bits-message)), meaning the log contains breaking changes that the parser cannot handle.
- Парсер повинен правильно обробляти журнали, які раптово закінчуються, посеред повідомлення.
  Недовершене повідомлення слід просто викинути.
- Для доданих даних: парсер може вважати, що секція Даних існує, тобто зміщення вказує на місце після секції Визначень.
  - Додані дані повинні трактуватися так, ніби вони були частиною звичайного розділу даних.

## Відомі Реалізації Парсера

- PX4-Autopilot++: C++
  - [logger module](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/logger)
  - [replay module](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/replay)
  - [hardfault_log module](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/hardfault_log): append hardfault crash data.
- [pyulog](https://github.com/PX4/pyulog): python, ULog reader and writer library with CLI scripts.
- [ulog_cpp](https://github.com/PX4/ulog_cpp): C++, ULog reader and writer library.
- [FlightPlot](https://github.com/PX4/FlightPlot): Java, log plotter.
- [MAVLink](https://github.com/mavlink/mavlink): Messages for ULog streaming via MAVLink (note that appending data is not supported, at least not for cut off messages).
- [QGroundControl](https://github.com/mavlink/qgroundcontrol): C++, ULog streaming via MAVLink and minimal parsing for GeoTagging.
- [mavlink-router](https://github.com/01org/mavlink-router): C++, ULog streaming via MAVLink.
- [MAVGAnalysis](https://github.com/ecmnet/MAVGCL): Java, ULog streaming via MAVLink and parser for plotting and analysis.
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler): C++/Qt application to plot logs and time series. Підтримує ULog з версії 2.1.3.
- [ulogreader](https://github.com/maxsun/ulogreader): Javascript, ULog reader and parser outputs log in JSON object format.
- [Foxglove Studio](https://github.com/foxglove/studio): an integrated visualization and diagnosis tool for robotics
  (Typescript ULog parser: https://github.com/foxglove/ulog).

## Історія версій формату файлу

### Зміни у версії 2

- Addition of [Multi Information Message](#m-multi-information-message) and [Flag Bits Message](#b-flag-bits-message) and the ability to append data to a log.
  - Це використовується для додавання даних про збій до існуючого журналу.
  - Якщо дані додаються до журналу, який обрізаний посередині повідомлення, їх не можна розбирати з парсерами версії 1.
- Крім того, передня та задня сумісність забезпечується, якщо парсери ігнорують невідомі повідомлення.

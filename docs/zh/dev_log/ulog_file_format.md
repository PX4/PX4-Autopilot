# ULog 文件格式

ULog is the file format used for logging messages. The format is self-describing, i.e. it contains the format and [uORB](../middleware/uorb.md) message types that are logged.
This document is meant to be the ULog File Format Spec Documentation.
It is intended especially for anyone who is interested in writing a ULog parser / serializer and needs to decode / encode files.

PX4 uses ULog to log uORB topics as messages related to (but not limited to) the following sources:

- **Device inputs:** Sensors, RC input, etc.
- **Internal states:** CPU load, attitude, EKF state, etc.
- **String messages:** `printf` statements, including `PX4_INFO()` and `PX4_ERR()`.

The format uses [little endian](https://en.wikipedia.org/wiki/Endianness) memory layout for all binary types (the least significant byte (LSB) of data type is placed at the lowest memory address).

## 数据类型

The following binary types are used for logging. They all correspond to the types in C.

| 类型                                                          | 大小（以字节为单位） |
| ----------------------------------------------------------- | ---------- |
| int8_t, uint8_t   | 1          |
| int16_t, uint16_t | 2          |
| int32_t, uint32_t | 4          |
| int64_t, uint64_t | 8          |
| float                                                       | 4          |
| double                                                      | 8          |
| bool, char                                                  | 1          |

Additionally the types can be used as a fixed-size array: e.g. `float[5]`.

Strings (`char[length]`) do not contain the termination NULL character `'\0'` at the end.

:::info
String comparisons are case sensitive, which should be taken into account when comparing message names when [adding subscriptions](#a-subscription-message).
:::

## ULog File Structure

ULog files have the following three sections:

```
----------------------
|         头         |
----------------------
|        定义        |
----------------------
|        数据        |
----------------------
```

A description of each section is provided below.

### 头部分

头是一个固定大小的部分，具有以下格式（16个字节）：

```plain
----------------------------------------------------------------------
| 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x01         | uint64_t       |
| File magic(7B)                     | Version (1B) |  Timestamp (8B) |
----------------------------------------------------------------------
```

- **File Magic (7 Bytes):** File type indicator that reads "ULogXYZ where XYZ is the magic bytes sequence `0x01 0x12 0x35`"
- **Version (1 Byte):** File format version (currently 1)
- **Timestamp (8 Bytes):** `uint64_t` integer that denotes when the logging started in microseconds.

### Definition & Data Section Message Header

The _Definitions and Data_ sections contain a number of **messages**. Each message is preceded by this header:

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

### 定义部分

The definitions section contains basic information such as software version, message format, initial parameter values, and so on.

The message types in this section are:

1. [Flag Bits](#b-flag-bits-message)
2. [Format Definition](#f-format-message)
3. [Information](#i-information-message)
4. [Multi Information](#m-multi-information-message)
5. [Parameter](#p-parameter-message)
6. [Default Parameter](#q-default-parameter-message)

#### 'B': Flag Bits Message

:::info
This message must be the **first message** right after the header section, so that it has a fixed constant offset from the start of the file!
:::

This message provides information to the log parser whether the log is parsable or not.

```c
struct ulog_message_flag_bits_s {
  struct message_header_s header; // msg_type = 'B'
  uint8_t compat_flags[8];
  uint8_t incompat_flags[8];
  uint64_t appended_offsets[3]; // file offset(s) for appended data if appending bit is set
};
```

- `compat_flags`: compatible flag bits

  - These flags indicate the presence of features in the log file that are compatible with any ULog parser.
  - `compat_flags[0]`: _DEFAULT_PARAMETERS_ (Bit 0): if set, the log contains [default parameters message](#q-default-parameter-message)

  The rest of the bits are currently not defined and must be set to 0.
  These bits can be used for future ULog changes that are compatible with existing parsers.
  For example, adding a new message type can be indicated by defining a new bit in the standard, and existing parsers will ignore the new message type.
  It means parsers can just ignore the bits if one of the unknown bits is set.

- `incompat_flags`: incompatible flag bits.

  - `incompat_flags[0]`: _DATA_APPENDED_ (Bit 0): if set, the log contains appended data and at least one of the `appended_offsets` is non-zero.

  The rest of the bits are currently not defined and must be set to 0.
  这可用于引入现有解析器无法处理的重大更改。 For example, when an old ULog parser that didn't have the concept of _DATA_APPENDED_ reads the newer ULog, it would stop parsing the log as the log will contain out-of-spec messages / concepts.
  If a parser finds any of these bits set that isn't specified, it must refuse to parse the log.

- `appended_offsets`: File offset (0-based) for appended data.
  如果没有附加数据，则所有偏移量必须为零。
  这可以用于消息中途暂停的情况下可靠的添加数据。
  For example, crash dumps.

  附加数据的过程应该做到：

  - set the relevant `incompat_flags` bit
  - set the first `appended_offsets` that is currently 0 to the length of the log file without the appended data, as that is where the new data will start
  - append any type of messages that are valid for the Data section.

It is possible that there are more fields appended at the end of this message in future ULog specifications. This means a parser must not assume a fixed length of this message. If the message is longer than expected (currently 40 bytes), the exceeding bytes must just be ignored.
这意味着解析器必须不能假定此消息的长度是固定的。
If the `msg_size` is bigger than expected (currently 40), any additional bytes must be ignored/discarded.

#### 'F': Format Message

Format message defines a single message name and its inner fields in a single string.

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

- 一个类型可以在定义之前使用。
  - e.g. The message `MessageA:MessageB[2] msg_b` can come before the `MessageB:uint_8[3] data`
- There can be arbitrary nesting but **no circular dependencies**
  - e.g. `MessageA:MessageB[2] msg_b` & `MessageB:MessageA[4] msg_a`

有些字段名是特殊的：

- `timestamp`: every message format with a [Subscription Message](#a-subscription-message) must include a timestamp field (for example a message format only used as part of a nested definition by another format may not include a timestamp field)
  - Its type must be `uint64_t`.
  - The unit is microseconds.
  - The timestamp must always be monotonic increasing for a message series with the same `msg_id` (same subscription).
- `_padding{}`: field names that start with `_padding` (e.g. `_padding[3]`) should not be displayed and their data must be ignored by a reader.
  - 写入器可以通过插入这个字段确保正确对齐。
  - If the padding field is the last field, then this field may not be logged, to avoid writing unnecessary data.
  - This means the `message_data_s.data` will be shorter by the size of the padding.
  - 但是当报文在嵌套定义中使用时任然需要填充。
- In general, message fields are not necessarily aligned (i.e. the field offset within the message is not necessarily a multiple of its data size), so a reader must always use appropriate memory copy methods to access individual fields.

#### 'I': Information Message

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
A key defined in the Information message must be unique. Meaning there must not be more than one definition with the same key value.
:::

解析器可以将报文信息存储为字典。

预定义的信息报文有：

| 键                                   | 描述                                                                            | 示例值                                                               |
| ----------------------------------- | ----------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| `char[value_len] sys_name`          | 系统名称                                                                          | "PX4"                                                             |
| `char[value_len] ver_hw`            | 硬件版本 (主板)                                                  | "PX4FMU_V4"                                  |
| `char[value_len] ver_hw_subtype`    | 主办子版本 (变化的)                                                | "V2"                                                              |
| `char[value_len] ver_sw`            | 软件版本 (git 标签)                                              | "7f65e01"                                                         |
| `char[value_len] ver_sw_branch`     | git branch                                                                    | "master"                                                          |
| `uint32_t ver_sw_release`           | 软件版本 (见下文)                                                 | 0x010401ff                                                        |
| `char[value_len] sys_os_name`       | 操作系统名称                                                                        | "Linux"                                                           |
| `char[value_len] sys_os_ve`r        | 操作系统版本 (git 标签)                                            | "9f82919"                                                         |
| `uint32_t ver_os_release`           | 操作系统版本 (见下文)                                               | 0x010401ff                                                        |
| `char[value_len] sys_toolchain`     | 工具链名称                                                                         | "GNU GCC"                                                         |
| `char[value_len] sys_toolchain_ver` | 工具链版本                                                                         | "6.2.1"                           |
| `char[value_len] sys_mcu`           | 芯片名称和修订                                                                       | "STM32F42x, rev A"                                                |
| `char[value_len] sys_uuid`          | Unique identifier for vehicle (eg. MCU ID) | "392a93e32fa3"... |
| `char[value_len] log_type`          | Type of the log (full log if not specified)                | "mission"                                                         |
| `char[value_len] replay`            | 重播日志的文件名如果处于重播模式                                                              | "log001.ulg"                                      |
| `int32_t time_ref_utc`              | UTC 时间的秒偏移量                                                                   | -3600                                                             |

:::info
`value_len` represents the data size of the `value`. This is described in the `key`.
:::

- The format of `ver_sw_release` and `ver_os_release` is: 0xAABBCCTT, where AA is **major**, BB is **minor**, CC is patch and TT is the **type**.
  - **Type** is defined as following: `>= 0`: development, `>= 64`: alpha version, `>= 128`: beta version, `>= 192`: RC version, `== 255`: release version.
  - For example, `0x010402FF` translates into the release version v1.4.2.

This message can also be used in the Data section (this is however the preferred section).

#### 'M': Multi Information Message

Multi information message serves the same purpose as the information message, but for long messages or multiple messages with the same key.

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

解析器可以将所有多报文信息存储为一个 2D 列表，使用与日志中报文相同的顺序。

Valid names and types are the same as for the Information message.

#### 'P': Parameter Message

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

#### 'Q': Default Parameter Message

The default parameter message defines the default value of a parameter for a given vehicle and setup.

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
  - At least one bit must be set:
    - `1<<0`: system wide default
    - `1<<1`: default for the current configuration (e.g. an airframe)

A log may not contain default values for all parameters.
In those cases the default is equal to the parameter value, and different default types are treated independently.

This message can also be used in the Data section, and the same data type and naming applies as for the Parameter message.

This section ends before the start of the first [Subscription Message](#a-subscription-message) or [Logging](#l-logged-string-message) message, whichever comes first.

### 数据部分

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

- `multi_id`: the same message format can have multiple instances, for example if the system has two sensors of the same type. 默认值以及第一个实例一定是0.
- `msg_id`: unique id to match [Logged data Message](#d-logged-data-message) data. 第一次使用一定要设置为 0，然后递增。
  - The same `msg_id` must not be used twice for different subscriptions.
- `message_name`: message name to subscribe to.
  Must match one of the [Format Message](#f-format-message) definitions.

#### `R`: Unsubscription Message

Unsubscribe a message, to mark that it will not be logged anymore (not used currently).

```c
struct message_remove_logged_s {
  struct message_header_s header; // msg_type = 'R'
  uint16_t msg_id;
};
```

#### 'D': Logged Data Message

```c
struct message_data_s {
  struct message_header_s header; // msg_type = 'D'
  uint16_t msg_id;
  uint8_t data[header.msg_size-2];
};
```

- `msg_id`: as defined by a [Subscription Message](#a-subscription-message)
- `data` contains the logged binary message as defined by [Format Message](#f-format-message)

有关填充字段的特殊处理，请参见上文。

#### 'L': Logged String Message

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

| 参数名     | 对应值 | 含义       |
| ------- | --- | -------- |
| EMERG   | '0' | 系统无法使用   |
| ALERT   | '1' | 操作必须立即执行 |
| CRIT    | '2' | 紧急情况     |
| ERR     | '3' | 错误情况     |
| WARNING | '4' | 警告情况     |
| NOTICE  | '5' | 正常但重要的情况 |
| INFO    | '6' | 信息       |
| DEBUG   | '7' | 调试级别的消息  |

#### 'C': Tagged Logged String Message

```c
struct message_logging_tagged_s {
  struct message_header_s header; // msg_type = 'C'
  uint8_t log_level;
  uint16_t tag;
  uint64_t timestamp;
  char message[header.msg_size-11]
};
```

- `tag`: id representing source of logged message string. It could represent a process, thread or a class depending upon the system architecture.

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

| 参数名     | 对应值 | 含义       |
| ------- | --- | -------- |
| EMERG   | '0' | 系统无法使用   |
| ALERT   | '1' | 操作必须立即执行 |
| CRIT    | '2' | 紧急情况     |
| ERR     | '3' | 错误情况     |
| WARNING | '4' | 警告情况     |
| NOTICE  | '5' | 正常但重要的情况 |
| INFO    | '6' | 信息       |
| DEBUG   | '7' | 调试级别的消息  |

#### 'S': Synchronization message

Synchronization message so that a reader can recover from a corrupt message by searching for the next sync message.

```c
struct message_sync_s {
  struct message_header_s header; // msg_type = 'S'
  uint8_t sync_magic[8];
};
```

- `sync_magic`: [0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12]

#### 'O': Dropout message

Mark a dropout (lost logging messages) of a given duration in ms.

例如当设备不够快的情况下会出现丢包。

```c
struct message_dropout_s {
  struct message_header_s header; // msg_type = 'O'
  uint16_t duration;
};
```

#### Messages shared with the Definitions Section

Since the Definitions and Data Sections use the same message header format, they also share the same messages listed below:

- [Information Message](#i-information-message).
- [Multi Information Message](#m-multi-information-message)
- [Parameter Message](#p-parameter-message)
  - For the _Data_ section, the Parameter Message is used when the parameter value changes
- [Default Parameter Message](#q-default-parameter-message)

## 解析器的要求

一个有效的 ULog 解析器必须满足以下要求:

- Must ignore unknown messages (but it can print a warning)
- 解析未来/未知的文件格式版本 (但可以打印警告) 。
- Must refuse to parse a log which contains unknown incompatibility bits set (`incompat_flags` of [Flag Bits Message](#b-flag-bits-message)), meaning the log contains breaking changes that the parser cannot handle.
- A parser must be able to correctly handle logs that end abruptly, in the middle of a message. The unfinished message should just be discarged.
  未完成的报文应该丢弃。
- 对于附加数据:解析器可以假设数据部分存在，即在定义部分之后的位置有一个偏移点。
  - 必须将附加数据视为常规数据部分的一部分。

## Known Parser Implementations

- PX4 Firmware: C++
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
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler): C++/Qt application to plot logs and time series. 自版本2.1.3支持 ULog。
- [ulogreader](https://github.com/maxsun/ulogreader): Javascript, ULog reader and parser outputs log in JSON object format.
- [Foxglove Studio](https://github.com/foxglove/studio): an integrated visualization and diagnosis tool for robotics
  (Typescript ULog parser: https://github.com/foxglove/ulog).

## 文件格式版本历史

### 版本 2 中的改变

- Addition of [Multi Information Message](#m-multi-information-message) and [Flag Bits Message](#b-flag-bits-message) and the ability to append data to a log.
  - 这被用来给现有的日志添加损坏的数据。
  - 如果从中间切开的报文数据被附加到日志中，这不能被版本 1 解析器解析。
- 除此之外，如果解析器忽略未知消息，则提供向前和向后的兼容性。

# 日志

The [system logger](../modules/modules_system.md#logger) is able to log any ORB topic with all included fields.
Everything necessary is generated from the `.msg` file, so that only the topic name needs to be specified.
An optional interval parameter specifies the maximum logging rate of a certain topic.
所有主题的实例将会被记录。

The output log format is [ULog](../dev_log/ulog_file_format.md).

[Encrypted logging](../dev_log/log_encryption.md) is also supported.

## 用法

By default, logging is automatically started when arming, and stopped when disarming.
每次解锁后的飞行对话将会在 SD 卡上生成一个新的日志文件。
To display the current state, use `logger status` on the console.
If you want to start logging immediately, use `logger on`.
This overrides the arming state, as if the system was armed.
`logger off` undoes this.

If logging stops due to a write error, or reaching the [maximum file size](#file-size-limitations), PX4 will automatically restart logging in a new file.

对于所有支持的记录器命令和参数的列表，使用：

```
logger help
```

## 配置

The logging system is configured by default to collect sensible logs for [flight reporting](../getting_started/flight_reporting.md) with [Flight Review](http://logs.px4.io).

Logging may further be configured using the [SD Logging](../advanced_config/parameter_reference.md#sd-logging) parameters.
The parameters you are most likely to change are listed below.

| 参数                                                                                            | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| --------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [SDLOG_MODE](../advanced_config/parameter_reference.md#SDLOG_MODE)       | 日志模式 Defines when logging starts and stops.<br />- `-1`: Logging disabled.<br />- `0`: Log when armed until disarm (default).<br />- `1`: Log from boot until disarm.<br />- `2`: Log from boot until shutdown.<br />- `3`: Log based on the [AUX1 RC channel](../advanced_config/parameter_reference.md#RC_MAP_AUX1).<br />- `4`: Log from first armed until shutdown. |
| [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE) | Logging profile. Use this to enable less common logging/analysis (e.g. for EKF2 replay, high rate logging for PID & filter tuning, thermal temperature calibration).                                                                                                                                                                                                                                                                                                                                |
| [SDLOG_MISSION](../advanced_config/parameter_reference.md#SDLOG_MISSION) | Create very small additional "Mission Log".<br>This log can _not_ be used with [Flight Review](../log/flight_log_analysis.md#flight-review-online-tool), but is useful when you need a small log for geotagging or regulatory compliance.                                                                                                                                                                                                                                                                                                                                  |

Useful settings for specific cases:

- Raw sensor data for comparison: [SDLOG_MODE=1](../advanced_config/parameter_reference.md#SDLOG_MODE) and [SDLOG_PROFILE=64](../advanced_config/parameter_reference.md#SDLOG_PROFILE).
- Disabling logging altogether: [SDLOG_MODE=`-1`](../advanced_config/parameter_reference.md#SDLOG_MODE)

### Logger module

_Developers_ can further configure what information is logged via the [logger](../modules/modules_system.md#logger) module.
This allows, for example, logging of your own uORB topics.

### 诊断

Separately, the list of logged topics can also be customized with a file on the SD card.
Create a file `etc/logging/logger_topics.txt` on the card with a list of topics (For SITL, it's `build/px4_sitl_default/rootfs/fs/microsd/etc/logging/logger_topics.txt`):

```plain
<topic_name> <interval> <instance>
```

The `<interval>` is optional, and if specified, defines the minimum interval in ms between two logged messages of this topic.
If not specified, the topic is logged at full rate.

The `<instance>` is optional, and if specified, defines the instance to log.
If not specified, all instances of the topic are logged.
To specify `<instance>`, `<interval>` must be specified. It can be set to 0 to log at full rate

The topics in this file replace all of the default logged topics.

By far the best card we know so far is the <strong x-id="1">SanDisk Extreme U3 32GB</strong>. This card is recommended, because it does not exhibit write time spikes (and thus virtually no dropouts). Different card sizes might work equally well, but the performance is usually different.

```plain
sensor_accel 0 0
sensor_accel 100 1
sensor_gyro 200
sensor_mag 200 1
```

This configuration will log sensor_accel 0 at full rate, sensor_accel 1 at 10Hz, all sensor_gyro instances at 5Hz and sensor_mag 1 at 5Hz.

## 脚本

There are several scripts to analyze and convert logging files in the [pyulog](https://github.com/PX4/pyulog) repository.

## File size limitations

The maximum file size depends on the file system and OS.
The size limit on NuttX is currently around 2GB.

## 丢帧

Logging dropouts are undesired and there are a few factors that influence the
amount of dropouts:

- Most SD cards we tested exhibit multiple pauses per minute. This shows itself as a several 100 ms delay during a write command. It causes a dropout if the write buffer fills up during this time. This effect depends on the SD card (see below).
  This shows itself as a several 100 ms delay during a write command.
  It causes a dropout if the write buffer fills up during this time.
  This effect depends on the SD card (see below).
- 格式化 SD 卡有助于避免丢帧。
- 增大日志缓存也有效。
- Decrease the logging rate of selected topics or remove unneeded topics from being logged (`info.py <file>` is useful for this).

## SD 卡

The maximum supported SD card size for NuttX is 32GB (SD Memory Card Specifications Version 2.0).
The **SanDisk Extreme U3 32GB** and **Samsung EVO Plus 32** are known to be reliable cards (do not exhibit write-time spikes, and thus virtually no dropouts).

The table below shows the **mean sequential write speed [KB/s]** / **maximum write time per block (average) [ms]** for F4- (Pixracer), F7-, and H7-based flight controllers.

| SD Card                                                                          | F4            | F7         | H7        |
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

Logging bandwidth with the default topics is around 50 KB/s, which almost all SD cards satisfy in terms of their mean sequential write speed.

More important than the mean write speed is spikes (or generally high values) in the maximum write time per block (of 4 KB) or `fsync` times, as a long write time means a larger log buffer is needed to avoid dropouts.

PX4 uses bigger buffers on F7/H7 and read caching, which is enough to compensate for spikes in many poor cards.
That said, if your card has an `fsync` or write duration of several 100ms it is should not be preferred for use with PX4.
You can check the value by running [sd_bench](../modules/modules_command.md#sd-bench) should be run with more iterations (around 100 should do).

```sh
sd_bench -r 100
```

并且同一时刻只能有一个客户机可以请求日志流。
PX4 uses bigger buffers on F7/H7 and read caching to make up for some of these issues.

:::info
If you have concerns about a particular card you can run the above test and report the results to https://github.com/PX4/PX4-Autopilot/issues/4634.
:::

## 日志流

The traditional and still fully supported way to do logging is using an SD card on the FMU.
However there is an alternative, log streaming, which sends the same logging data via MAVLink.
This method can be used for example in cases where the FMU does not have an SD card slot (e.g. Intel® Aero Ready to Fly Drone) or simply to avoid having to deal with SD cards.
Both methods can be used independently and at the same time.

The requirement is that the link provides at least ~50KB/s, so for example a WiFi link.
And only one client can request log streaming at the same time.
The connection does not need to be reliable, the protocol is designed to handle drops.

There are different clients that support ulog streaming:

- `mavlink_ulog_streaming.py` script in PX4-Autopilot/Tools.
- QGroundControl:
  ![QGC Log Streaming](../../assets/gcs/qgc-log-streaming.png)
- [MAVGCL](https://github.com/ecmnet/MAVGCL)

### Diagnostics

- If log streaming does not start, make sure the `logger` is running (see above), and inspect the console output while starting.
- If it still does not work, make sure that Mavlink 2 is used. Enforce it by setting <code>MAV_PROTO_VER</code> to 2.
  Enforce it by setting `MAV_PROTO_VER` to 2.
- Log streaming uses a maximum of 70% of the configured MAVLink rate (`-r` parameter).
  如果需要更大的速率，数据会丢失。
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
  Also make sure <code>txerr</code> stays at 0. If this goes up, either the NuttX sending buffer is too small, the physical link is saturated or the hardware is too slow to handle the data.

## See Also

- [Encrypted logging](../dev_log/log_encryption.md)
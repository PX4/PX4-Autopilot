# 飞行报告

PX4 logs detailed aircraft state and sensor data, which can be used to analyze performance issues.
This topic explains how you can download and analyse logs, and share them with the development team for review.

:::tip
Keeping flight logs is a legal requirement in some jurisdictions.
:::

## 从飞行控制器下载日志

Logs can be downloaded using [QGroundControl](http://qgroundcontrol.com/): **[Analyze View > Log Download](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/log_download.html)**.

![Flight Log Download](../../assets/qgc/analyze/log_download.jpg)

## 分析日志

Upload the log file to the online [Flight Review](http://logs.px4.io) tool.
After upload you'll be emailed a link to the analysis page for the log.

[Log Analysis using Flight Review](../log/flight_review.md) explains how to interpret the plots, and can help you to verify/reject the causes of common problems: excessive vibration, poor PID tuning, saturated controllers, imbalanced vehicles, GPS noise, etc.

:::info
There are many other great tools for visualising and analysing PX4 Logs.
For more information see: [Flight Analysis](../dev_log/flight_log_analysis.md).
:::

:::tip
If you have a constant high-rate MAVLink connection to the vehicle (not just a telemetry link) then you can use _QGroundControl_ to automatically upload logs directly to _Flight Review_.
For more information see [Settings > MAVLink Settings > MAVLink 2 Logging (PX4 only)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/mavlink.html#logging).
:::

## 给 PX4 开发人员共享日志文件

The [Flight Review](http://logs.px4.io) log file link can be shared for discussion in the [support forums](../contribute/support.md#forums-and-chat) or a [Github issue](../index.md#reporting-bugs-issues).

## Log Configuration

The logging system is configured by default to collect sensible logs for use with [Flight Review](http://logs.px4.io).

Logging may further be configured using the [SD Logging](../advanced_config/parameter_reference.md#sd-logging) parameters or with a file on the SD card.
Details on configuration can be found in the [logging configuration documentation](../dev_log/logging.md#configuration).

## 主链接

- [Flight Review](http://logs.px4.io)
- [Log Analysis using Flight Review](../log/flight_review.md)
- [Flight Log Analysis](../dev_log/flight_log_analysis.md)

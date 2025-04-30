# 비행 기록

PX4는 성능 분석을 위하여 상세한 항공기 상태 데이터와 센서 데이터를 로그 파일에 기록합니다.
로그 파일 다운로드, 분석, 검토 및 개발 팀과 공유하는 방법을 설명합니다.

:::tip
Keeping flight logs is a legal requirement in some jurisdictions.
:::

## 비행 콘트롤러에서 로그 다운로드

Logs can be downloaded using [QGroundControl](http://qgroundcontrol.com/): **[Analyze View > Log Download](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/log_download.html)**.

![Flight Log Download](../../assets/qgc/analyze/log_download.jpg)

:::tip
Encrypted logs cannot be downloaded with QGroundControl, or uploaded to the public Flight Review service.
The easiest way to download and extract encrypted logs is to use the [Log Encryption Tools](../dev_log/log_encryption.md).
You can also host a [private Flight Review server](../dev_log/log_encryption.md#flight-review-encrypted-logs) that automatically decrypts logs on upload using your private key.
:::

## 로그 분석

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

## PX4 개발자가 검토할 로그 파일 공유

The [Flight Review](http://logs.px4.io) log file link can be shared for discussion in the [support forums](../contribute/support.md#forums-and-chat) or a [Github issue](../index.md#reporting-bugs-issues).

## 로그 설정

The logging system is configured by default to collect sensible logs for use with [Flight Review](http://logs.px4.io).

Logging may further be configured using the [SD Logging](../advanced_config/parameter_reference.md#sd-logging) parameters or with a file on the SD card.
Details on configuration can be found in the [logging configuration documentation](../dev_log/logging.md#configuration).

## 주요 링크

- [Flight Review](http://logs.px4.io)
- [Log Analysis using Flight Review](../log/flight_review.md)
- [Flight Log Analysis](../dev_log/flight_log_analysis.md)

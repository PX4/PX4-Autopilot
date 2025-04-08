# Flight Reporting

PX4 logs detailed aircraft state and sensor data, which can be used to analyze performance issues.
This topic explains how you can download and analyse logs, and share them with the development team for review.

:::tip
Keeping flight logs is a legal requirement in some jurisdictions.
:::

## Downloading Logs from the Flight Controller

Logs can be downloaded using [QGroundControl](http://qgroundcontrol.com/): **[Analyze View > Log Download](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/log_download.html)**.

![Flight Log Download](../../assets/qgc/analyze/log_download.jpg)


## Analyzing the Logs

Upload the log file to the online [Flight Review](http://logs.px4.io) tool.
After upload you'll be emailed a link to the analysis page for the log.

[Log Analysis using Flight Review](../log/flight_review.md) explains how to interpret the plots, and can help you to verify/reject the causes of common problems: excessive vibration, poor PID tuning, saturated controllers, imbalanced vehicles, GPS noise, etc.

::: info
There are many other great tools for visualising and analysing PX4 Logs.
For more information see: [Flight Analysis](../dev_log/flight_log_analysis.md).
:::

:::tip
If you have a constant high-rate MAVLink connection to the vehicle (not just a telemetry link) then you can use *QGroundControl* to automatically upload logs directly to *Flight Review*.
For more information see [Settings > MAVLink Settings > MAVLink 2 Logging (PX4 only)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/mavlink.html#logging).
:::

## Sharing the Log Files for Review by PX4 Developers

The [Flight Review](http://logs.px4.io) log file link can be shared for discussion in the [support forums](../contribute/support.md#forums-and-chat) or a [Github issue](../index.md#reporting-bugs-issues).


## Log Configuration

The logging system is configured by default to collect sensible logs for use with [Flight Review](http://logs.px4.io).

Logging may further be configured using the [SD Logging](../advanced_config/parameter_reference.md#sd-logging) parameters or with a file on the SD card.
Details on configuration can be found in the [logging configuration documentation](../dev_log/logging.md#configuration).

## Encrypted logs
Encrypted logs cannot be downloaded from QGroundControl, you either have to extract them from the SD card or you can use the log downloader provided in the [Log Encryption Tools](../dev_log/log_encryption.md)

Once you have the logs you can either decrypt them using the provided tool or you can also use Flight Review, with your Private key being enbeeded to your local server.

You need to clone [Flight Review](https://github.com/PX4/flight_review) and follow the installation steps.

Once you have your own Flight Review you can just place ypur Private Key to
   ```sh
   flight_review/app/private_key/private_key.pem
   ```
And in the config file you should specify the path
So in

   ```sh
   flight_review/app/config_default.ini
   ```
You should have
   ```sh
   ulge_private_key = ../private_key/private_key.pem
   ```

Once you are done you can just follow the FLight Review Instructions to start your server.

## Key Links

- [Flight Review](http://logs.px4.io)
- [Log Analysis using Flight Review](../log/flight_review.md)
- [Flight Log Analysis](../dev_log/flight_log_analysis.md)

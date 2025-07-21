# Statistical Flight Log Analysis

This topic contains information and resources related to statistical flight log analysis.

## Flight Review Public Logs

[Flight Review](../log/flight_log_analysis.md#flight-review-online-tool) hosts a large set of publicly available log files that can be used for statistical analysis, machine learning, or other purposes.

The dataset contains a set of different:

- vehicle types
- PX4 versions (including development versions)
- boards
- flight modes

The logs are accessible on [logs.px4.io/browse](https://logs.px4.io/browse) and are licensed under [CC-BY PX4](https://creativecommons.org/licenses/by/4.0/).

Log files can also be downloaded in bulk with the [download_logs.py](https://github.com/PX4/flight_review/blob/main/app/download_logs.py) script.
The script allows to filter by different attributes (like flight modes, airframe name or type).
Use the `--help` flag for a full list.
The newest logs will be downloaded first, and downloads can be interrupted and resumed later on.

There are different parsing libraries, for example [pyulog](../log/flight_log_analysis.md#pyulog) can be used to read logs with Python.

---
pageClass: is-wide-page
---

# RcChannels (UORB message)

**TOPICS:** rc_channels

## Fields

| Name                                                      | Type          | Unit [Frame] | Range/Enum | Description                                                        |
| --------------------------------------------------------- | ------------- | ------------ | ---------- | ------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                       | `uint64`      |              |            | time since system start (microseconds)                             |
| <a id="fld_timestamp_last_valid"></a>timestamp_last_valid | `uint64`      |              |            | Timestamp of last valid RC signal                                  |
| <a id="fld_channels"></a>channels                         | `float32[18]` |              |            | Scaled to -1..1 (throttle: 0..1)                                   |
| <a id="fld_channel_count"></a>channel_count               | `uint8`       |              |            | Number of valid channels                                           |
| <a id="fld_rssi"></a>rssi                                 | `uint8`       |              |            | Receive signal strength index                                      |
| <a id="fld_signal_lost"></a>signal_lost                   | `bool`        |              |            | Control signal lost, should be checked together with topic timeout |
| <a id="fld_frame_drop_count"></a>frame_drop_count         | `uint32`      |              |            | Number of dropped frames                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RcChannels.msg)

::: details Click here to see original file

```c
uint64 timestamp						# time since system start (microseconds)

uint64 timestamp_last_valid					# Timestamp of last valid RC signal
float32[18] channels						# Scaled to -1..1 (throttle: 0..1)
uint8 channel_count						# Number of valid channels
uint8 rssi							# Receive signal strength index
bool signal_lost						# Control signal lost, should be checked together with topic timeout
uint32 frame_drop_count						# Number of dropped frames
```

:::

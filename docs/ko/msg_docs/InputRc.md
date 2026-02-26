---
pageClass: is-wide-page
---

# InputRc (UORB message)

**TOPICS:** input_rc

## Fields

| 명칭                                                                                  | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                                                                                                                                                                                                          |
| ----------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                           | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                                                                                                                                                                   |
| timestamp_last_signal                     | `uint64`     |                                                                  |            | last valid reception time                                                                                                                                                                                                                                                                                                                                                                   |
| channel_count                                                  | `uint8`      |                                                                  |            | number of channels actually being seen                                                                                                                                                                                                                                                                                                                                                      |
| rssi                                                                                | `int32`      |                                                                  |            | receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 100: full reception                                                                                                                                                                                     |
| rc_failsafe                                                    | `bool`       |                                                                  |            | explicit failsafe flag: true on TX failure or TX out of range , false otherwise. Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.                                                                                                     |
| rc_lost                                                        | `bool`       |                                                                  |            | RC receiver connection status: True,if no frame has arrived in the expected time, false otherwise. True usually means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems. Will remain false, if a RX with failsafe option continues to transmit frames after a link loss. |
| rc_lost_frame_count  | `uint16`     |                                                                  |            | Number of lost RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.                                                                                                                         |
| rc_total_frame_count | `uint16`     |                                                                  |            | Number of total RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.                                                                                                                        |
| rc_ppm_frame_length  | `uint16`     |                                                                  |            | Length of a single PPM frame. Zero for non-PPM systems                                                                                                                                                                                                                                                                                                                      |
| rc_frame_rate                             | `uint16`     |                                                                  |            | RC frame rate in msg/second. 0 = invalid                                                                                                                                                                                                                                                                                                                                    |
| input_source                                                   | `uint8`      |                                                                  |            | Input source                                                                                                                                                                                                                                                                                                                                                                                |
| values                                                                              | `uint16[18]` |                                                                  |            | measured pulse widths for each of the supported channels                                                                                                                                                                                                                                                                                                                                    |
| link_quality                                                   | `int8`       |                                                                  |            | link quality. Percentage 0-100%. -1 = invalid                                                                                                                                                                                                                                                                                                               |
| rssi_dbm                                                       | `float32`    |                                                                  |            | Actual rssi in units of dBm. NaN = invalid                                                                                                                                                                                                                                                                                                                                  |
| link_snr                                                       | `int8`       |                                                                  |            | link signal to noise ratio in units of dB. -1 = invalid                                                                                                                                                                                                                                                                                                                     |

## Constants

| 명칭                                                                                                                                                                  | 형식      | Value | 설명                                                                                                                               |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | ----- | -------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#RC_INPUT_SOURCE_UNKNOWN"></a> RC_INPUT_SOURCE_UNKNOWN                                      | `uint8` | 0     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_PPM"></a> RC_INPUT_SOURCE_PX4FMU_PPM           | `uint8` | 1     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4IO_PPM"></a> RC_INPUT_SOURCE_PX4IO_PPM             | `uint8` | 2     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4IO_SPEKTRUM"></a> RC_INPUT_SOURCE_PX4IO_SPEKTRUM   | `uint8` | 3     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4IO_SBUS"></a> RC_INPUT_SOURCE_PX4IO_SBUS           | `uint8` | 4     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4IO_ST24"></a> RC_INPUT_SOURCE_PX4IO_ST24           | `uint8` | 5     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_MAVLINK"></a> RC_INPUT_SOURCE_MAVLINK                                      | `uint8` | 6     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_QURT"></a> RC_INPUT_SOURCE_QURT                                            | `uint8` | 7     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_SPEKTRUM"></a> RC_INPUT_SOURCE_PX4FMU_SPEKTRUM | `uint8` | 8     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_SBUS"></a> RC_INPUT_SOURCE_PX4FMU_SBUS         | `uint8` | 9     |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_ST24"></a> RC_INPUT_SOURCE_PX4FMU_ST24         | `uint8` | 10    |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_SUMD"></a> RC_INPUT_SOURCE_PX4FMU_SUMD         | `uint8` | 11    |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_DSM"></a> RC_INPUT_SOURCE_PX4FMU_DSM           | `uint8` | 12    |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4IO_SUMD"></a> RC_INPUT_SOURCE_PX4IO_SUMD           | `uint8` | 13    |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_CRSF"></a> RC_INPUT_SOURCE_PX4FMU_CRSF         | `uint8` | 14    |                                                                                                                                  |
| <a href="#RC_INPUT_SOURCE_PX4FMU_GHST"></a> RC_INPUT_SOURCE_PX4FMU_GHST         | `uint8` | 15    |                                                                                                                                  |
| <a href="#RC_INPUT_MAX_CHANNELS"></a> RC_INPUT_MAX_CHANNELS                                          | `uint8` | 18    | Maximum number of R/C input channels in the system. S.Bus has up to 18 channels. |
| <a href="#RSSI_MAX"></a> RSSI_MAX                                                                                                              | `int8`  | 100   |                                                                                                                                  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/InputRc.msg)

:::details
Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)

uint8 RC_INPUT_SOURCE_UNKNOWN = 0
uint8 RC_INPUT_SOURCE_PX4FMU_PPM = 1
uint8 RC_INPUT_SOURCE_PX4IO_PPM = 2
uint8 RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3
uint8 RC_INPUT_SOURCE_PX4IO_SBUS = 4
uint8 RC_INPUT_SOURCE_PX4IO_ST24 = 5
uint8 RC_INPUT_SOURCE_MAVLINK = 6
uint8 RC_INPUT_SOURCE_QURT = 7
uint8 RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8
uint8 RC_INPUT_SOURCE_PX4FMU_SBUS = 9
uint8 RC_INPUT_SOURCE_PX4FMU_ST24 = 10
uint8 RC_INPUT_SOURCE_PX4FMU_SUMD = 11
uint8 RC_INPUT_SOURCE_PX4FMU_DSM = 12
uint8 RC_INPUT_SOURCE_PX4IO_SUMD = 13
uint8 RC_INPUT_SOURCE_PX4FMU_CRSF = 14
uint8 RC_INPUT_SOURCE_PX4FMU_GHST = 15

uint8 RC_INPUT_MAX_CHANNELS = 18 	# Maximum number of R/C input channels in the system. S.Bus has up to 18 channels.

uint64 timestamp_last_signal		# last valid reception time

uint8 channel_count			# number of channels actually being seen

int8 RSSI_MAX = 100
int32 rssi				# receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 100: full reception

bool rc_failsafe			# explicit failsafe flag: true on TX failure or TX out of range , false otherwise. Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
bool rc_lost				# RC receiver connection status: True,if no frame has arrived in the expected time, false otherwise. True usually means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems. Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.

uint16 rc_lost_frame_count		# Number of lost RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.
uint16 rc_total_frame_count		# Number of total RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.
uint16 rc_ppm_frame_length		# Length of a single PPM frame. Zero for non-PPM systems
uint16 rc_frame_rate			# RC frame rate in msg/second. 0 = invalid

uint8 input_source			# Input source
uint16[18] values			# measured pulse widths for each of the supported channels

int8 link_quality			# link quality. Percentage 0-100%. -1 = invalid
float32 rssi_dbm			# Actual rssi in units of dBm. NaN = invalid
int8 link_snr				# link signal to noise ratio in units of dB. -1 = invalid
```

:::

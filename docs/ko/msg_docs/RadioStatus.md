---
pageClass: is-wide-page
---

# RadioStatus (UORB message)

**TOPICS:** radio_status

## Fields

| 명칭                                | 형식       | Unit [Frame] | Range/Enum | 설명                                                        |
| --------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64` |                                                                  |            | time since system start (microseconds) |
| rssi                              | `uint8`  |                                                                  |            | local signal strength                                     |
| remote_rssi  | `uint8`  |                                                                  |            | remote signal strength                                    |
| txbuf                             | `uint8`  |                                                                  |            | how full the tx buffer is as a percentage                 |
| noise                             | `uint8`  |                                                                  |            | background noise level                                    |
| remote_noise | `uint8`  |                                                                  |            | remote background noise level                             |
| rxerrors                          | `uint16` |                                                                  |            | receive errors                                            |
| fix                               | `uint16` |                                                                  |            | count of error corrected packets                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RadioStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)

uint8 rssi				# local signal strength
uint8 remote_rssi			# remote signal strength

uint8 txbuf				# how full the tx buffer is as a percentage
uint8 noise				# background noise level

uint8 remote_noise			# remote background noise level
uint16 rxerrors				# receive errors

uint16 fix				# count of error corrected packets
```

:::

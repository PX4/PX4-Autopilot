---
pageClass: is-wide-page
---

# PpsCapture (UORB message)

**TOPICS:** pps_capture

## Fields

| Назва                              | Тип      | Unit [Frame] | Range/Enum                                             | Опис                                                                           |
| ---------------------------------- | -------- | ---------------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------------------------------ |
| timestamp                          | `uint64` |                                                                  |                                                        | time since system start (microseconds) at PPS capture event |
| rtc_timestamp | `uint64` |                                                                  |                                                        | Corrected GPS UTC timestamp at PPS capture event                               |
| `uint8`                            |          |                                                                  | Increments when PPS dt < 50ms |                                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PpsCapture.msg)

:::details
Click here to see original file

```c
uint64 timestamp			  # time since system start (microseconds) at PPS capture event
uint64 rtc_timestamp		# Corrected GPS UTC timestamp at PPS capture event
uint8  pps_rate_exceeded_counter # Increments when PPS dt < 50ms
```

:::

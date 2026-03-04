---
pageClass: is-wide-page
---

# GpioRequest (UORB message)

Request GPIO mask to be read.

**TOPICS:** gpio_request

## Fields

| Name      | Type     | Unit [Frame] | Range/Enum | Description                            |
| --------- | -------- | ------------ | ---------- | -------------------------------------- |
| timestamp | `uint64` |              |            | time since system start (microseconds) |
| device_id | `uint32` |              |            | Device id                              |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpioRequest.msg)

::: details Click here to see original file

```c
# Request GPIO mask to be read

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id
```

:::

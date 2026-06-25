---
pageClass: is-wide-page
---

# GpioIn (UORB message)

GPIO mask and state.

**TOPICS:** gpio_in

## Fields

| Name                                | Type     | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------- | -------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64` |              |            | time since system start (microseconds) |
| <a id="fld_device_id"></a>device_id | `uint32` |              |            | Device id                              |
| <a id="fld_state"></a>state         | `uint32` |              |            | pin state mask                         |

## Constants

| Name                                      | Type    | Value | Description |
| ----------------------------------------- | ------- | ----- | ----------- |
| <a id="#MAX_INSTANCES"></a> MAX_INSTANCES | `uint8` | 8     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpioIn.msg)

::: details Click here to see original file

```c
# GPIO mask and state
uint8 MAX_INSTANCES = 8

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id

uint32 state				# pin state mask
```

:::

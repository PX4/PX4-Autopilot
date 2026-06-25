---
pageClass: is-wide-page
---

# GpioOut (UORB message)

GPIO mask and state.

**TOPICS:** gpio_out

## Fields

| Name                                | Type     | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------- | -------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64` |              |            | time since system start (microseconds) |
| <a id="fld_device_id"></a>device_id | `uint32` |              |            | Device id                              |
| <a id="fld_mask"></a>mask           | `uint32` |              |            | pin mask                               |
| <a id="fld_state"></a>state         | `uint32` |              |            | pin state mask                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpioOut.msg)

::: details Click here to see original file

```c
# GPIO mask and state

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id

uint32 mask				# pin mask
uint32 state				# pin state mask
```

:::

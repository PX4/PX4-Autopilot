---
pageClass: is-wide-page
---

# UavcanFirmwareUpdate (UORB message)

**TOPICS:** uavcan_firmware_update

## Fields

| 参数名                                                                  | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                                            |
| -------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                  | `uint64` |                                                                  |            | time since system start (microseconds)                                                     |
| <a id="fld_pending_updates"></a>pending_updates | `bool`   |                                                                  |            | true when one or more nodes requiring a firmware update have been detected and the update is not yet complete |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UavcanFirmwareUpdate.msg)

:::details
Click here to see original file

```c
uint64 timestamp        # time since system start (microseconds)
bool pending_updates    # true when one or more nodes requiring a firmware update have been detected and the update is not yet complete
```

:::

---
pageClass: is-wide-page
---

# GimbalManagerStatus (UORB message)

**TOPICS:** gimbal_manager_status

## Fields

| Name                                                              | Type     | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------------------------------------- | -------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                               | `uint64` |              |            | time since system start (microseconds) |
| <a id="fld_flags"></a>flags                                       | `uint32` |              |            |
| <a id="fld_gimbal_device_id"></a>gimbal_device_id                 | `uint8`  |              |            |
| <a id="fld_primary_control_sysid"></a>primary_control_sysid       | `uint8`  |              |            |
| <a id="fld_primary_control_compid"></a>primary_control_compid     | `uint8`  |              |            |
| <a id="fld_secondary_control_sysid"></a>secondary_control_sysid   | `uint8`  |              |            |
| <a id="fld_secondary_control_compid"></a>secondary_control_compid | `uint8`  |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalManagerStatus.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint32 flags
uint8 gimbal_device_id
uint8 primary_control_sysid
uint8 primary_control_compid
uint8 secondary_control_sysid
uint8 secondary_control_compid
```

:::

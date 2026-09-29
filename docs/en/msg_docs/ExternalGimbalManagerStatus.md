---
pageClass: is-wide-page
---

# ExternalGimbalManagerStatus (UORB message)

External gimbal manager status.

Status of a gimbal manager running on another component (not us), decoded from
an incoming GIMBAL_MANAGER_STATUS message. Used to track which component
currently controls an external gimbal manager.

**TOPICS:** external_gimbal_manager_status

## Fields

| Name                                                              | Type     | Unit [Frame] | Range/Enum | Description                                 |
| ----------------------------------------------------------------- | -------- | ------------ | ---------- | ------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                               | `uint64` | us           |            | Time since system start                     |
| <a id="fld_manager_sysid"></a>manager_sysid                       | `uint8`  |              |            | System id of the external gimbal manager    |
| <a id="fld_manager_compid"></a>manager_compid                     | `uint8`  |              |            | Component id of the external gimbal manager |
| <a id="fld_flags"></a>flags                                       | `uint32` |              |            |
| <a id="fld_gimbal_device_id"></a>gimbal_device_id                 | `uint8`  |              |            |
| <a id="fld_primary_control_sysid"></a>primary_control_sysid       | `uint8`  |              |            |
| <a id="fld_primary_control_compid"></a>primary_control_compid     | `uint8`  |              |            |
| <a id="fld_secondary_control_sysid"></a>secondary_control_sysid   | `uint8`  |              |            |
| <a id="fld_secondary_control_compid"></a>secondary_control_compid | `uint8`  |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ExternalGimbalManagerStatus.msg)

::: details Click here to see original file

```c
# External gimbal manager status
#
# Status of a gimbal manager running on another component (not us), decoded from
# an incoming GIMBAL_MANAGER_STATUS message. Used to track which component
# currently controls an external gimbal manager.

uint64 timestamp # [us] Time since system start

uint8 manager_sysid # [-] System id of the external gimbal manager
uint8 manager_compid # [-] Component id of the external gimbal manager

uint32 flags # [-]
uint8 gimbal_device_id
uint8 primary_control_sysid
uint8 primary_control_compid
uint8 secondary_control_sysid
uint8 secondary_control_compid
```

:::

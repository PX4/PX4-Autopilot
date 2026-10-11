---
pageClass: is-wide-page
---

# FixedWingTakeoffStatus (UORB message)

Status of a fixed-wing takeoff. Passes information from the FixedWingModeManager to the Navigator.

**TOPICS:** fixed_wing_takeoff_status

## Fields

| Назва                                                                      | Тип      | Unit [Frame] | Range/Enum | Опис                                                                                                                                                                   |
| -------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                        | `uint64` | us                                                               |            | time since system start                                                                                                                                                |
| <a id="fld_climbout_completed"></a>climbout_completed | `bool`   |                                                                  |            | Whether the takeoff climbout is finished (altitude, or time if FW_TKO_CLMB_T is set) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FixedWingTakeoffStatus.msg)

:::details
Click here to see original file

```c
# Status of a fixed-wing takeoff
# Passes information from the FixedWingModeManager to the Navigator.

uint64 timestamp # [us] time since system start

bool climbout_completed  # Whether the takeoff climbout is finished (altitude, or time if FW_TKO_CLMB_T is set)
```

:::

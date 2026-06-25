---
pageClass: is-wide-page
---

# HealthReport (UORB message)

**TOPICS:** health_report

## Fields

| Name                                                                  | Type     | Unit [Frame] | Range/Enum | Description                                                               |
| --------------------------------------------------------------------- | -------- | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                   | `uint64` |              |            | time since system start (microseconds)                                    |
| <a id="fld_can_arm_mode_flags"></a>can_arm_mode_flags                 | `uint64` |              |            | bitfield for each flight mode (NAVIGATION*STATE*\*) if arming is possible |
| <a id="fld_can_run_mode_flags"></a>can_run_mode_flags                 | `uint64` |              |            | bitfield for each flight mode if it can run                               |
| <a id="fld_health_is_present_flags"></a>health_is_present_flags       | `uint64` |              |            | flags for each health_component_t                                         |
| <a id="fld_health_warning_flags"></a>health_warning_flags             | `uint64` |              |            |
| <a id="fld_health_error_flags"></a>health_error_flags                 | `uint64` |              |            |
| <a id="fld_arming_check_warning_flags"></a>arming_check_warning_flags | `uint64` |              |            |
| <a id="fld_arming_check_error_flags"></a>arming_check_error_flags     | `uint64` |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/HealthReport.msg)

::: details Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

uint64 can_arm_mode_flags              # bitfield for each flight mode (NAVIGATION_STATE_*) if arming is possible
uint64 can_run_mode_flags              # bitfield for each flight mode if it can run

uint64 health_is_present_flags         # flags for each health_component_t
uint64 health_warning_flags
uint64 health_error_flags
# A component is required but missing, if present==0 and error==1

uint64 arming_check_warning_flags
uint64 arming_check_error_flags
```

:::

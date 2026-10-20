---
pageClass: is-wide-page
---

# PrecTakeoffStatus (UORB message)

Precision takeoff status.

Published by: navigator (prec_takeoff.cpp).
Subscribed by: vision_target_estimator, flight_mode_manager (FlightTaskAuto).

ONGOING while a vertical takeoff setpoint is active and MIS_TKO_PREC is set. DONE is published when the
takeoff altitude is reached, then STOPPED once the takeoff is no longer active. Published every Navigator cycle.

**TOPICS:** prec_takeoff_status

## Fields

| Name                                                | Type     | Unit [Frame] | Range/Enum                                | Description                                                |
| --------------------------------------------------- | -------- | ------------ | ----------------------------------------- | ---------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                 | `uint64` | us           |                                           | Time since system start                                    |
| <a id="fld_state"></a>state                         | `uint8`  |              | [PREC_TAKEOFF_STATE](#PREC_TAKEOFF_STATE) | Current precision-takeoff state                            |
| <a id="fld_setpoint_adjusted"></a>setpoint_adjusted | `bool`   |              |                                           | True after a valid target adjusted the horizontal setpoint |

## Enums

### PREC_TAKEOFF_STATE {#PREC_TAKEOFF_STATE}

Used in field(s): [state](#fld_state)

| Name                                                                | Type    | Value | Description                                                     |
| ------------------------------------------------------------------- | ------- | ----- | --------------------------------------------------------------- |
| <a id="#PREC_TAKEOFF_STATE_STOPPED"></a> PREC_TAKEOFF_STATE_STOPPED | `uint8` | 0     | Not active                                                      |
| <a id="#PREC_TAKEOFF_STATE_ONGOING"></a> PREC_TAKEOFF_STATE_ONGOING | `uint8` | 1     | Takeoff active; the estimator may still be acquiring the target |
| <a id="#PREC_TAKEOFF_STATE_DONE"></a> PREC_TAKEOFF_STATE_DONE       | `uint8` | 2     | Takeoff altitude reached                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PrecTakeoffStatus.msg)

::: details Click here to see original file

```c
# Precision takeoff status.
#
# Published by: navigator (prec_takeoff.cpp).
# Subscribed by: vision_target_estimator, flight_mode_manager (FlightTaskAuto).
#
# ONGOING while a vertical takeoff setpoint is active and MIS_TKO_PREC is set. DONE is published when the
# takeoff altitude is reached, then STOPPED once the takeoff is no longer active. Published every Navigator cycle.

uint64 timestamp # [us] Time since system start

uint8 state # [@enum PREC_TAKEOFF_STATE] Current precision-takeoff state
bool setpoint_adjusted # True after a valid target adjusted the horizontal setpoint
uint8 PREC_TAKEOFF_STATE_STOPPED = 0 # Not active
uint8 PREC_TAKEOFF_STATE_ONGOING = 1 # Takeoff active; the estimator may still be acquiring the target
uint8 PREC_TAKEOFF_STATE_DONE = 2 # Takeoff altitude reached
```

:::

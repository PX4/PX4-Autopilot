---
pageClass: is-wide-page
---

# SurfaceVehicleThrottleSetpoint (UORB message)

Surface Vehicle Throttle Setpoint.

**TOPICS:** surface_vehicle_throttle_setpoint

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum                      | Description                                                                                    |
| ----------------------------------- | --------- | ------------ | ------------------------------- | ---------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`  | us           |                                 | Time since system start                                                                        |
| <a id="fld_x"></a>x                 | `float32` | [Body]       | [-1 (Backwards) : 1 (Forwards)] | Normalized throttle setpoint along the body X axis                                             |
| <a id="fld_y"></a>y                 | `float32` | [Body]       | [-1 (Left) : 1 (Right)]         | Mecanum only: Normalized throttle setpoint along the body Y axis (Invalid: NaN If not mecanum) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SurfaceVehicleThrottleSetpoint.msg)

::: details Click here to see original file

```c
# Surface Vehicle Throttle Setpoint

uint64 timestamp  # [us] Time since system start
float32 x         # [-] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Normalized throttle setpoint along the body X axis
float32 y         # [-] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Normalized throttle setpoint along the body Y axis
```

:::

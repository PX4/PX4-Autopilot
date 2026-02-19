---
pageClass: is-wide-page
---

# VehicleTorqueSetpoint (повідомлення UORB)

**TOPICS:** vehicle_torque_setpoint vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc

## Fields

| Назва                                 | Тип          | Unit [Frame] | Range/Enum | Опис                                                                                          |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | time since system start (microseconds)                                     |
| timestamp_sample | `uint64`     |                                                                  |            | timestamp of the data sample on which this message is based (microseconds) |
| xyz                                   | `float32[3]` |                                                                  |            | torque setpoint about X, Y, Z body axis (normalized)                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleTorqueSetpoint.msg)

:::details
Click here to see original file

```c
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)

# TOPICS vehicle_torque_setpoint
# TOPICS vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc
```

:::

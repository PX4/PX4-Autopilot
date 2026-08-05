---
pageClass: is-wide-page
---

# 载具推力设定点（UORB 消息）

Vehicle thrust setpoint.

This is the thrust setpoint provided by the controller and fed into the control allocator.

**TOPICS:** vehicle_thrust_setpoint vehicle_thrust_setpoint_virtual_fw vehicle_thrust_setpoint_virtual_mc

## Fields

| 参数名                                                                    | 类型           | Unit [Frame] | Range/Enum                                                                   | 描述                                                                                                                                 |
| ---------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                    | `uint64`     | us                                                               |                                                                              | Time since system start                                                                                                            |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`     | us                                                               |                                                                              | Timestamp of the data sample on which this message is based                                                                        |
| <a id="fld_xyz"></a>xyz                                                | `float32[3]` |                                                                  | [-1 : 1] | Thrust setpoint along X, Y, Z body axis. If set to NAN the motors affecting this axis are stopped. |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleThrustSetpoint.msg)

:::details
Click here to see original file

```c
# Vehicle thrust setpoint
#
# This is the thrust setpoint provided by the controller and fed into the control allocator.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the data sample on which this message is based

float32[3] xyz          # [-] [@range -1, 1] Thrust setpoint along X, Y, Z body axis. If set to NAN the motors affecting this axis are stopped.

# TOPICS vehicle_thrust_setpoint
# TOPICS vehicle_thrust_setpoint_virtual_fw vehicle_thrust_setpoint_virtual_mc
```

:::

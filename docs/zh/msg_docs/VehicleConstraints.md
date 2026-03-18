---
pageClass: is-wide-page
---

# 载具限制 (UORB 消息)

Local setpoint constraints in NED frame. setting something to NaN means that no limit is provided.

**TOPICS:** vehicle_constraints

## Fields

| 参数名                               | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                             |
| --------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------- |
| timestamp                         | `uint64`  |                                                                  |            | time since system start (microseconds)                                      |
| speed_up     | `float32` |                                                                  |            | in meters/sec                                                                                  |
| speed_down   | `float32` |                                                                  |            | in meters/sec                                                                                  |
| want_takeoff | `bool`    |                                                                  |            | tell the controller to initiate takeoff when idling (ignored during flight) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleConstraints.msg)

:::details
Click here to see original file

```c
# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp # time since system start (microseconds)

float32 speed_up # in meters/sec
float32 speed_down # in meters/sec

bool want_takeoff # tell the controller to initiate takeoff when idling (ignored during flight)
```

:::

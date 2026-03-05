---
pageClass: is-wide-page
---

# VehicleAttitudeSetpointV0 (UORB message)

**TOPICS:** vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint

## Fields

| Назва                                                                               | Тип          | Unit [Frame] | Range/Enum | Опис                                                                                                   |
| ----------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------ |
| timestamp                                                                           | `uint64`     |                                                                  |            | time since system start (microseconds)                                              |
| yaw_sp_move_rate     | `float32`    |                                                                  |            | rad/s (commanded by user)                                                           |
| q_d                                                            | `float32[4]` |                                                                  |            | Desired quaternion for quaternion control                                                              |
| thrust_body                                                    | `float32[3]` |                                                                  |            | Normalized thrust command in body FRD frame [-1,1] |
| reset_integral                                                 | `bool`       |                                                                  |            | Reset roll/pitch/yaw integrals (navigation logic change)                            |
| fw_control_yaw_wheel | `bool`       |                                                                  |            | control heading with steering wheel (used for auto takeoff on runway)               |

## Constants

| Назва                                                                | Тип      | Значення | Опис |
| -------------------------------------------------------------------- | -------- | -------- | ---- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/VehicleAttitudeSetpointV0.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)

float32 yaw_sp_move_rate	# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d			# Desired quaternion for quaternion control

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body		# Normalized thrust command in body FRD frame [-1,1]

bool reset_integral	# Reset roll/pitch/yaw integrals (navigation logic change)

bool fw_control_yaw_wheel	# control heading with steering wheel (used for auto takeoff on runway)

# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
```

:::

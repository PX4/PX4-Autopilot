# OffboardControlMode (UORB消息)

Offboard控制模式

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OffboardControlMode.msg)

```c
# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool position
bool velocity
bool acceleration
bool attitude
bool body_rate
bool thrust_and_torque
bool direct_actuator

```

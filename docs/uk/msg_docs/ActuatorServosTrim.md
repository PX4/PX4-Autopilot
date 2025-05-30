# ActuatorServosTrim (повідомлення UORB)

Підлаштування сервоприводів, що додаються як зміщення до виходів сервоприводів

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorServosTrim.msg)

```c
# Servo trims, added as offset to servo outputs
uint64 timestamp			# time since system start (microseconds)

uint8 NUM_CONTROLS = 8
float32[8] trim    # range: [-1, 1]

```

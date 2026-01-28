# VehicleConstraints (повідомлення UORB)

Обмеження локальної заданої точки в рамці NED
встановлення чого-небудь на NaN означає, що обмеження не надано

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleConstraints.msg)

```c
# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp # time since system start (microseconds)

float32 speed_up # in meters/sec
float32 speed_down # in meters/sec

bool want_takeoff # tell the controller to initiate takeoff when idling (ignored during flight)

```

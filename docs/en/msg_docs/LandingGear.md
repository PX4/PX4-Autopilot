# LandingGear (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LandingGear.msg)

```c
uint64 timestamp # time since system start (microseconds)

int8 GEAR_UP = 1 # landing gear up
int8 GEAR_DOWN = -1 # landing gear down
int8 GEAR_KEEP = 0 # keep the current state

int8 landing_gear

```

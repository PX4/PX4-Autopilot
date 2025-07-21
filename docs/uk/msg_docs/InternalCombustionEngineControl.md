# InternalCombustionEngineControl (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/InternalCombustionEngineControl.msg)

```c
uint64 timestamp        		# time since system start (microseconds)

bool ignition_on          		# activate/deactivate ignition (Spark Plug)
float32 throttle_control		# [0,1] - Motor should idle with 0. Includes slew rate if enabled.
float32 choke_control			# [0,1] - 1 fully closes the air inlet.
float32 starter_engine_control		# [0,1] - control value for electric starter motor.

uint8 user_request			# user intent for the ICE being on/off

```

---
pageClass: is-wide-page
---

# InternalCombustionEngineControl (UORB message)

**TOPICS:** internal_combustionengine_control

## Fields

| Назва                                                            | Тип       | Unit [Frame] | Range/Enum | Опис                                                                                                                                                                                                                                                                                                                |
| ---------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                        | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                                                                                           |
| ignition_on                                 | `bool`    |                                                                  |            | activate/deactivate ignition (spark plug)                                                                                                                                                                                                                                                        |
| throttle_control                            | `float32` |                                                                  |            | setpoint for throttle actuator, with slew rate if enabled, idles with 0 [norm] [@range 0,1] [@uncontrolled NAN to stop motor] |
| choke_control                               | `float32` |                                                                  |            | setpoint for choke actuator, 1: fully closed [norm] [@range 0,1]                                                                                                               |
| starter_engine_control | `float32` |                                                                  |            | setpoint for (electric) starter motor [norm] [@range 0,1]                                                                                                                   |
| user_request                                | `uint8`   |                                                                  |            | user intent for the ICE being on/off                                                                                                                                                                                                                                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/InternalCombustionEngineControl.msg)

:::details
Click here to see original file

```c
uint64 timestamp        		# time since system start (microseconds)

bool ignition_on          		# activate/deactivate ignition (spark plug)
float32 throttle_control		# setpoint for throttle actuator, with slew rate if enabled, idles with 0 [norm] [@range 0,1] [@uncontrolled NAN to stop motor]
float32 choke_control			# setpoint for choke actuator, 1: fully closed [norm] [@range 0,1]
float32 starter_engine_control		# setpoint for (electric) starter motor [norm] [@range 0,1]

uint8 user_request			# user intent for the ICE being on/off
```

:::

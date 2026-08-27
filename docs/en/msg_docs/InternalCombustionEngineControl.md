---
pageClass: is-wide-page
---

# InternalCombustionEngineControl (UORB message)

Internal combustion engine (ICE) actuator setpoints.

Published by the internal_combustion_engine_control module, which runs the engine
start/stop state machine and the idle RPM governor.
The mixer maps these setpoints onto the IC Engine output functions (ignition,
throttle, choke, starter), which are assigned to the flight controller outputs.
The resulting state machine state is reported in InternalCombustionEngineStatus.

**TOPICS:** internal_combustion_engine_control

## Fields

| Name                                                          | Type      | Unit [Frame] | Range/Enum | Description                                                                                                                                       |
| ------------------------------------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                           | `uint64`  | us           |            | Time since system start                                                                                                                           |
| <a id="fld_ignition_on"></a>ignition_on                       | `bool`    |              |            | Activate/deactivate the ignition (spark plug)                                                                                                     |
| <a id="fld_throttle_control"></a>throttle_control             | `float32` | norm         | [0 : 1]    | Throttle actuator setpoint, rate limited by ICE_THR_SLEW. 0 is closed throttle (Invalid: NaN Stops the engine, output goes to its disarmed value) |
| <a id="fld_choke_control"></a>choke_control                   | `float32` | norm         | [0 : 1]    | Choke actuator setpoint, 1 is fully closed                                                                                                        |
| <a id="fld_starter_engine_control"></a>starter_engine_control | `float32` | norm         | [0 : 1]    | (Electric) starter motor setpoint                                                                                                                 |
| <a id="fld_user_request_motor_on"></a>user_request_motor_on   | `bool`    |              |            | User intent for the engine to be running, from the source selected by ICE_ON_SOURCE                                                               |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/InternalCombustionEngineControl.msg)

::: details Click here to see original file

```c
# Internal combustion engine (ICE) actuator setpoints
#
# Published by the internal_combustion_engine_control module, which runs the engine
# start/stop state machine and the idle RPM governor.
# The mixer maps these setpoints onto the IC Engine output functions (ignition,
# throttle, choke, starter), which are assigned to the flight controller outputs.
# The resulting state machine state is reported in InternalCombustionEngineStatus.

uint64 timestamp # [us] Time since system start

bool ignition_on # Activate/deactivate the ignition (spark plug)
float32 throttle_control # [norm] [@range 0, 1] [@invalid NaN Stops the engine, output goes to its disarmed value] Throttle actuator setpoint, rate limited by ICE_THR_SLEW. 0 is closed throttle
float32 choke_control # [norm] [@range 0, 1] Choke actuator setpoint, 1 is fully closed
float32 starter_engine_control # [norm] [@range 0, 1] (Electric) starter motor setpoint

bool user_request_motor_on # User intent for the engine to be running, from the source selected by ICE_ON_SOURCE
```

:::

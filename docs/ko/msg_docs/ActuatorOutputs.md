---
pageClass: is-wide-page
---

# ActuatorOutputs (UORB message)

**TOPICS:** actuator_outputs actuator_outputs_sim actuator_outputs_debug

## Fields

| 명칭        | 형식            | Unit [Frame] | Range/Enum | 설명                                                        |
| --------- | ------------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`      |                                                                  |            | time since system start (microseconds) |
| noutputs  | `uint32`      |                                                                  |            | valid outputs                                             |
| output    | `float32[16]` |                                                                  |            | output data, in natural output units                      |

## Constants

| 명칭                                                                                                                                    | 형식      | Value | 설명                  |
| ------------------------------------------------------------------------------------------------------------------------------------- | ------- | ----- | ------------------- |
| <a href="#NUM_ACTUATOR_OUTPUTS"></a> NUM_ACTUATOR_OUTPUTS                                   | `uint8` | 16    |                     |
| <a href="#NUM_ACTUATOR_OUTPUT_GROUPS	"></a> NUM_ACTUATOR_OUTPUT_GROUPS | `uint8` | 4     | for sanity checking |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorOutputs.msg)

:::details
Click here to see original file

```c
uint64 timestamp				# time since system start (microseconds)
uint8 NUM_ACTUATOR_OUTPUTS		= 16
uint8 NUM_ACTUATOR_OUTPUT_GROUPS	= 4	# for sanity checking
uint32 noutputs				# valid outputs
float32[16] output				# output data, in natural output units

# actuator_outputs_sim is used for SITL, HITL & SIH (with an output range of [-1, 1])
# TOPICS actuator_outputs actuator_outputs_sim actuator_outputs_debug
```

:::

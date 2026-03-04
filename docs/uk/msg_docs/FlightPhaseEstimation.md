---
pageClass: is-wide-page
---

# FlightPhaseEstimation (повідомлення UORB)

**TOPICS:** flight_phaseestimation

## Fields

| Назва                             | Тип      | Unit [Frame] | Range/Enum | Опис                                                      |
| --------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64` |                                                                  |            | time since system start (microseconds) |
| flight_phase | `uint8`  |                                                                  |            | Estimate of current flight phase                          |

## Constants

| Назва                                                                                               | Тип     | Значення | Опис                            |
| --------------------------------------------------------------------------------------------------- | ------- | -------- | ------------------------------- |
| <a href="#FLIGHT_PHASE_UNKNOWN"></a> FLIGHT_PHASE_UNKNOWN | `uint8` | 0        | vehicle flight phase is unknown |
| <a href="#FLIGHT_PHASE_LEVEL"></a> FLIGHT_PHASE_LEVEL     | `uint8` | 1        | Vehicle is in level flight      |
| <a href="#FLIGHT_PHASE_DESCEND"></a> FLIGHT_PHASE_DESCEND | `uint8` | 2        | vehicle is in descend           |
| <a href="#FLIGHT_PHASE_CLIMB"></a> FLIGHT_PHASE_CLIMB     | `uint8` | 3        | vehicle is climbing             |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FlightPhaseEstimation.msg)

:::details
Click here to see original file

```c
uint64 timestamp               # time since system start (microseconds)

uint8 flight_phase 		# Estimate of current flight phase

uint8 FLIGHT_PHASE_UNKNOWN = 0  # vehicle flight phase is unknown
uint8 FLIGHT_PHASE_LEVEL = 1	# Vehicle is in level flight
uint8 FLIGHT_PHASE_DESCEND = 2	# vehicle is in descend
uint8 FLIGHT_PHASE_CLIMB = 3   # vehicle is climbing
```

:::

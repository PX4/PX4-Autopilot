# FlightPhaseEstimation (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FlightPhaseEstimation.msg)

```c
uint64 timestamp               # time since system start (microseconds)

uint8 flight_phase 		# Estimate of current flight phase

uint8 FLIGHT_PHASE_UNKNOWN = 0  # vehicle flight phase is unknown
uint8 FLIGHT_PHASE_LEVEL = 1	# Vehicle is in level flight
uint8 FLIGHT_PHASE_DESCEND = 2	# vehicle is in descend
uint8 FLIGHT_PHASE_CLIMB = 3   # vehicle is climbing

```

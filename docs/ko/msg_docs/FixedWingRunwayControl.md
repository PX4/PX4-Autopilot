---
pageClass: is-wide-page
---

# FixedWingRunwayControl (UORB message)

Auxiliary control fields for fixed-wing runway takeoff/landing.

**TOPICS:** fixed_wingrunway_control

## Fields

| 명칭                                                                                         | 형식        | Unit [Frame] | Range/Enum                                                                   | 설명                                                                                                         |
| ------------------------------------------------------------------------------------------ | --------- | ---------------------------------------------------------------- | ---------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                  | `uint64`  | us                                                               |                                                                              | time since system start                                                                                    |
| runway_takeoff_state                             | `uint8`   |                                                                  |                                                                              | Current state of runway takeoff state machine                                                              |
| wheel_steering_enabled                           | `bool`    |                                                                  |                                                                              | Flag that enables the wheel steering.                                                      |
| wheel_steering_nudging_rate | `float32` | FRD                                                              | [-1 : 1] | Manual wheel nudging, added to controller output. NAN is interpreted as 0. |

## Constants

| 명칭                                                                                                                             | 형식      | Value | 설명                                                                               |
| ------------------------------------------------------------------------------------------------------------------------------ | ------- | ----- | -------------------------------------------------------------------------------- |
| <a href="#STATE_THROTTLE_RAMP"></a> STATE_THROTTLE_RAMP                              | `uint8` | 0     | ramping up throttle                                                              |
| <a href="#STATE_CLAMPED_TO_RUNWAY"></a> STATE_CLAMPED_TO_RUNWAY | `uint8` | 1     | clamped to runway, controlling yaw directly (wheel or rudder) |
| <a href="#STATE_CLIMBOUT"></a> STATE_CLIMBOUT                                                             | `uint8` | 2     | climbout to safe height before navigation                                        |
| <a href="#STATE_FLYING"></a> STATE_FLYING                                                                 | `uint8` | 3     | navigate freely                                                                  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FixedWingRunwayControl.msg)

:::details
Click here to see original file

```c
# Auxiliary control fields for fixed-wing runway takeoff/landing

# Passes information from the FixedWingModeManager to the FixedWingAttitudeController (wheel control) and FixedWingLandDetector (takeoff state)

uint64 timestamp # [us] time since system start

uint8 STATE_THROTTLE_RAMP = 0		# ramping up throttle
uint8 STATE_CLAMPED_TO_RUNWAY = 1	# clamped to runway, controlling yaw directly (wheel or rudder)
uint8 STATE_CLIMBOUT = 2		# climbout to safe height before navigation
uint8 STATE_FLYING = 3			# navigate freely

uint8 runway_takeoff_state		# Current state of runway takeoff state machine

bool wheel_steering_enabled		# Flag that enables the wheel steering.
float32 wheel_steering_nudging_rate	# [norm] [@range -1, 1] [FRD] Manual wheel nudging, added to controller output. NAN is interpreted as 0.
```

:::

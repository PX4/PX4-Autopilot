---
pageClass: is-wide-page
---

# AutotuneAttitudeControlStatus (повідомлення UORB)

Autotune attitude control status.

This message is published by the fw_autotune_attitude_control and mc_autotune_attitude_control modules when the user engages autotune,
and is subscribed to by the respective attitude controllers to command rate setpoints.

The rate_sp field is consumed by the controllers, while the remaining fields (model coefficients, gains, filters, and autotune state) are used for logging and debugging.

**TOPICS:** autotune_attitudecontrol_status

## Fields

| Назва                          | Тип          | Unit [Frame] | Range/Enum      | Опис                                                                                                                 |
| ------------------------------ | ------------ | ---------------------------------------------------------------- | --------------- | -------------------------------------------------------------------------------------------------------------------- |
| timestamp                      | `uint64`     | us                                                               |                 | Time since system start                                                                                              |
| coeff                          | `float32[5]` |                                                                  |                 | Coefficients of the identified discrete-time model                                                                   |
| coeff_var | `float32[5]` |                                                                  |                 | Coefficients' variance of the identified discrete-time model                                                         |
| fitness                        | `float32`    |                                                                  |                 | Fitness of the parameter estimate                                                                                    |
| innov                          | `float32`    | rad/s                                                            |                 | Innovation (residual error between model and measured output)                                     |
| dt_model  | `float32`    | s                                                                |                 | Model sample time used for identification                                                                            |
| kc                             | `float32`    |                                                                  |                 | Proportional rate-loop gain (ideal form)                                                          |
| ki                             | `float32`    |                                                                  |                 | Integral rate-loop gain (ideal form)                                                              |
| kd                             | `float32`    |                                                                  |                 | Derivative rate-loop gain (ideal form)                                                            |
| kff                            | `float32`    |                                                                  |                 | Feedforward rate-loop gain                                                                                           |
| att_p     | `float32`    |                                                                  |                 | Proportional attitude gain                                                                                           |
| rate_sp   | `float32[3]` | rad/s                                                            |                 | Rate setpoint commanded to the attitude controller.                                                  |
| u_filt    | `float32`    |                                                                  |                 | Filtered input signal (normalized torque setpoint) used in system identification. |
| y_filt    | `float32`    | rad/s                                                            |                 | Filtered output signal (angular velocity) used in system identification.          |
| state                          | `uint8`      |                                                                  | [STATE](#STATE) | Current state of the autotune procedure.                                                             |

## Enums

### STATE {#STATE}

| Назва                                                                                                                                          | Тип     | Значення | Опис                                                                                        |
| ---------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ------------------------------------------------------------------------------------------- |
| <a href="#STATE_IDLE"></a> STATE_IDLE                                                                                     | `uint8` | 0        | Idle (not running)                                                       |
| <a href="#STATE_INIT"></a> STATE_INIT                                                                                     | `uint8` | 1        | Initialize filters and setup                                                                |
| <a href="#STATE_ROLL_AMPLITUDE_DETECTION"></a> STATE_ROLL_AMPLITUDE_DETECTION   | `uint8` | 2        | FW only: determine required excitation amplitude (roll)  |
| <a href="#STATE_ROLL"></a> STATE_ROLL                                                                                     | `uint8` | 3        | Roll-axis excitation and model identification                                               |
| <a href="#STATE_ROLL_PAUSE"></a> STATE_ROLL_PAUSE                                                    | `uint8` | 4        | Pause to return to level flight                                                             |
| <a href="#STATE_PITCH_AMPLITUDE_DETECTION"></a> STATE_PITCH_AMPLITUDE_DETECTION | `uint8` | 5        | FW only: determine required excitation amplitude (pitch) |
| <a href="#STATE_PITCH"></a> STATE_PITCH                                                                                   | `uint8` | 6        | Pitch-axis excitation and model identification                                              |
| <a href="#STATE_PITCH_PAUSE"></a> STATE_PITCH_PAUSE                                                  | `uint8` | 7        | Pause to return to level flight                                                             |
| <a href="#STATE_YAW_AMPLITUDE_DETECTION"></a> STATE_YAW_AMPLITUDE_DETECTION     | `uint8` | 8        | FW only: determine required excitation amplitude (yaw)   |
| <a href="#STATE_YAW"></a> STATE_YAW                                                                                       | `uint8` | 9        | Yaw-axis excitation and model identification                                                |
| <a href="#STATE_YAW_PAUSE"></a> STATE_YAW_PAUSE                                                      | `uint8` | 10       | Pause to return to level flight                                                             |
| <a href="#STATE_VERIFICATION"></a> STATE_VERIFICATION                                                                     | `uint8` | 11       | Verify model and candidate gains                                                            |
| <a href="#STATE_APPLY"></a> STATE_APPLY                                                                                   | `uint8` | 12       | Apply gains                                                                                 |
| <a href="#STATE_TEST"></a> STATE_TEST                                                                                     | `uint8` | 13       | Test gains in closed-loop                                                                   |
| <a href="#STATE_COMPLETE"></a> STATE_COMPLETE                                                                             | `uint8` | 14       | Tuning completed successfully                                                               |
| <a href="#STATE_FAIL"></a> STATE_FAIL                                                                                     | `uint8` | 15       | Tuning failed (model invalid or controller unstable)                     |
| <a href="#STATE_WAIT_FOR_DISARM"></a> STATE_WAIT_FOR_DISARM                     | `uint8` | 16       | Waiting for disarm before finalizing                                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/AutotuneAttitudeControlStatus.msg)

:::details
Click here to see original file

```c
# Autotune attitude control status
#
# This message is published by the fw_autotune_attitude_control and mc_autotune_attitude_control modules when the user engages autotune,
# and is subscribed to by the respective attitude controllers to command rate setpoints.
#
# The rate_sp field is consumed by the controllers, while the remaining fields (model coefficients, gains, filters, and autotune state) are used for logging and debugging.

uint64 timestamp  # [us] Time since system start

float32[5] coeff      # [-] Coefficients of the identified discrete-time model
float32[5] coeff_var  # [-] Coefficients' variance of the identified discrete-time model
float32 fitness       # [-] Fitness of the parameter estimate
float32 innov         # [rad/s] Innovation (residual error between model and measured output)
float32 dt_model      # [s] Model sample time used for identification


float32 kc    # [-] Proportional rate-loop gain (ideal form)
float32 ki    # [-] Integral rate-loop gain (ideal form)
float32 kd    # [-] Derivative rate-loop gain (ideal form)
float32 kff   # [-] Feedforward rate-loop gain
float32 att_p # [-] Proportional attitude gain

float32[3] rate_sp # [rad/s] Rate setpoint commanded to the attitude controller.

float32 u_filt  # [-] Filtered input signal (normalized torque setpoint) used in system identification.
float32 y_filt  # [rad/s] Filtered output signal (angular velocity) used in system identification.

uint8 state  # [@enum STATE] Current state of the autotune procedure.
uint8 STATE_IDLE = 0                      # Idle (not running)
uint8 STATE_INIT = 1                      # Initialize filters and setup
uint8 STATE_ROLL_AMPLITUDE_DETECTION = 2  # FW only: determine required excitation amplitude (roll)
uint8 STATE_ROLL = 3                      # Roll-axis excitation and model identification
uint8 STATE_ROLL_PAUSE = 4                # Pause to return to level flight
uint8 STATE_PITCH_AMPLITUDE_DETECTION = 5 # FW only: determine required excitation amplitude (pitch)
uint8 STATE_PITCH = 6                     # Pitch-axis excitation and model identification
uint8 STATE_PITCH_PAUSE = 7               # Pause to return to level flight
uint8 STATE_YAW_AMPLITUDE_DETECTION = 8   # FW only: determine required excitation amplitude (yaw)
uint8 STATE_YAW = 9                       # Yaw-axis excitation and model identification
uint8 STATE_YAW_PAUSE = 10                # Pause to return to level flight
uint8 STATE_VERIFICATION = 11             # Verify model and candidate gains
uint8 STATE_APPLY = 12                    # Apply gains
uint8 STATE_TEST = 13                     # Test gains in closed-loop
uint8 STATE_COMPLETE = 14                 # Tuning completed successfully
uint8 STATE_FAIL = 15                     # Tuning failed (model invalid or controller unstable)
uint8 STATE_WAIT_FOR_DISARM = 16          # Waiting for disarm before finalizing
```

:::

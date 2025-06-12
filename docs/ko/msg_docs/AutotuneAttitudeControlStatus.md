# AutotuneAttitudeControlStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/AutotuneAttitudeControlStatus.msg)

```c
uint64 timestamp                # time since system start (microseconds)

float32[5] coeff                # coefficients of the identified discrete-time model
float32[5] coeff_var            # coefficients' variance of the identified discrete-time model
float32 fitness                 # fitness of the parameter estimate
float32 innov
float32 dt_model

float32 kc
float32 ki
float32 kd
float32 kff
float32 att_p

float32[3] rate_sp

float32 u_filt
float32 y_filt

uint8 STATE_IDLE = 0
uint8 STATE_INIT = 1
uint8 STATE_ROLL = 2
uint8 STATE_ROLL_PAUSE = 3
uint8 STATE_PITCH = 4
uint8 STATE_PITCH_PAUSE = 5
uint8 STATE_YAW = 6
uint8 STATE_YAW_PAUSE = 7
uint8 STATE_VERIFICATION = 8
uint8 STATE_APPLY = 9
uint8 STATE_TEST = 10
uint8 STATE_COMPLETE = 11
uint8 STATE_FAIL = 12
uint8 STATE_WAIT_FOR_DISARM = 13

uint8 state

```

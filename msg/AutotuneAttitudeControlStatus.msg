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
uint8 STATE_ROLL_AMPLITUDE_DETECTION = 2
uint8 STATE_ROLL = 3
uint8 STATE_ROLL_PAUSE = 4
uint8 STATE_PITCH_AMPLITUDE_DETECTION = 5
uint8 STATE_PITCH = 6
uint8 STATE_PITCH_PAUSE = 7
uint8 STATE_YAW_AMPLITUDE_DETECTION = 8
uint8 STATE_YAW = 9
uint8 STATE_YAW_PAUSE = 10
uint8 STATE_VERIFICATION = 11
uint8 STATE_APPLY = 12
uint8 STATE_TEST = 13
uint8 STATE_COMPLETE = 14
uint8 STATE_FAIL = 15
uint8 STATE_WAIT_FOR_DISARM = 16

uint8 state

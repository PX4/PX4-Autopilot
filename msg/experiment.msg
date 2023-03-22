
uint64 timestamp     # time since system start (microseconds)
bool experiment_running # true if experiment is running, false otherwise

uint8 NUM_ACTUATOR_CONTROLS = 4

float32[4] input_parameter
uint8 INDEX_MAG = 0
uint8 INDEX_DUR = 1
uint8 INDEX_FSTART = 2
uint8 INDEX_FEND = 3

float32[4] control
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2
uint8 INDEX_THROTTLE = 3

uint8[3] injection_input
uint8 INDEX_STEP_INPUT = 0
uint8 INDEX_DOUBLET_INPUT = 1
uint8 INDEX_SINE_SWEEP_INPUT = 2

uint8[2] injection_output
uint8 INDEX_THROTTLE_OUTPUT = 0
uint8 INDEX_ELEVATOR_OUTPUT = 1

#Topic experiment


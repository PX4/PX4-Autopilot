# Neural control
#
# Debugging topic for the Neural controller, logs the inputs and output vectors of the neural network, and the time it takes to run
# Publisher: mc_nn_control
# Subscriber: logger

uint64 timestamp # [us] Time since system start

float32[15] observation # Observation vector (pos error (3), att (6d), lin vel (3), ang vel (3))
float32[4] network_output # Output from neural network

int32 controller_time # [us] Time spent from input to output
int32 inference_time # [us] Time spent for NN inference

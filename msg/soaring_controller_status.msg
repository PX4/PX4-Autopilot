# SOARING CONTROLLER STATE, SERVES AS FLAG IF CONTROLLER IS STILL PUBLISHING

uint64 timestamp			# time since system start (microseconds)

bool soaring_controller_running     # if true, the soaring controller is expected to publish
bool timeout_detected               # if true, the soaring controller has crashed
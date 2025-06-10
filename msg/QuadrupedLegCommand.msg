uint64 timestamp # time since system start (microseconds)

float32[4] turn_setpoints   # [-1, 1] Turn servo setpoints for each leg [LF, RF, LR, RR]
float32[4] wheel_setpoints  # [-1, 1] Wheel motor setpoints [-1: backward, 1: forward]
float32[4] rotate_setpoints # [-1, 1] Leg rotate motor setpoints
float32[4] pulley_setpoints # [-1, 1] Pulley motor setpoints

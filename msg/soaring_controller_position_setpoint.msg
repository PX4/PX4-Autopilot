# SOARING CONTROLLER POSITION SETPOINT

uint64 timestamp			# time since system start (microseconds)

float32[3] pos                             # position in ENU frame
float32[3] vel                             # velocity in ENU frame
float32[3] acc                             # acceleration in ENU frame
float32[3] f_command                       # force command inside controller
float32[3] m_command                       # moment command inside controller
float32[3] w_err                           # rotation vector to target pose



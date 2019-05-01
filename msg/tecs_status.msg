uint64 timestamp		# time since system start (microseconds)
uint8 TECS_MODE_NORMAL = 0
uint8 TECS_MODE_UNDERSPEED = 1
uint8 TECS_MODE_TAKEOFF = 2
uint8 TECS_MODE_LAND = 3
uint8 TECS_MODE_LAND_THROTTLELIM = 4
uint8 TECS_MODE_BAD_DESCENT = 5
uint8 TECS_MODE_CLIMBOUT = 6


float32 altitude_setpoint_adj
float32 altitude_state
float32 altitude_rate_setpoint
float32 vert_vel_state
float32 tas_setpoint_adj
float32 tas_state
float32 tas_rate_setpoint
float32 tas_rate

float32 total_energy_error
float32 energy_distribution_error
float32 total_energy_rate_error
float32 energy_distribution_rate_error

float32 throttle_integ_state
float32 pitch_integ_state

uint8 mode

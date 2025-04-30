# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp # time since system start (microseconds)

float32 speed_up # in meters/sec
float32 speed_down # in meters/sec

bool apply_brake
bool stopped
bool stick_input
bool terrain_hold
bool too_fast
float32 dist_to_bottom
float32 dist_to_ground_lock

bool want_takeoff # tell the controller to initiate takeoff when idling (ignored during flight)

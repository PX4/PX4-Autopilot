uint64 timestamp

float32 course_setpoint
float32 airspeed_reference_direction    # angle of desired airspeed vector [-pi, pi]
float32 lateral_acceleration_setpoint
float32 roll_sp # TODO: remove, only for testing

float32 heading_sp_runway_takeoff
bool reset_integral

# TOPICS fw_lateral_control_setpoint fw_lateral_control_status

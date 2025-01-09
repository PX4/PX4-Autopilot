uint64 timestamp

# NOTE: At least one of course_setpoint, airspeed_reference_direction, or lateral_acceleration_setpoint must be finite
float32 course_setpoint # NAN if not controlled directly, [-pi, pi]
float32 airspeed_reference_direction    # angle of desired airspeed vector, NAN if not controlled directly, used as feedforward if course setpoint is finite, [-pi, pi],
float32 lateral_acceleration_setpoint # NAN if not controlled directly, used as feedforward if either course setpoint or airspeed_reference_direction is finite, [m/s^2]
float32 roll_sp # TODO: remove, only for testing

float32 heading_sp_runway_takeoff # [-pi, pi] heading setpoint for runway takeoff
bool reset_integral # TODO: remove, resets rate controller integrals

# TOPICS fw_lateral_control_setpoint fw_lateral_control_status

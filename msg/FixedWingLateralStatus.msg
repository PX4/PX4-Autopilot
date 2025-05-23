# Fixed Wing Lateral Status message
# Published by the fw_lateral_longitudinal_control module to report the resultant lateral setpoint

uint64 timestamp                         # time since system start (microseconds)

float32 lateral_acceleration_setpoint    # [m/s^2] [FRD] resultant lateral acceleration setpoint
float32 can_run_factor 	 	         # [norm] [@range 0, 1] estimate of certainty of the correct functionality of the npfg roll setpoint

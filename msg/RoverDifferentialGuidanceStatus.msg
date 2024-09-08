uint64 timestamp # time since system start (microseconds)

float32 lookahead_distance    # [m] Lookahead distance of pure the pursuit controller
float32 heading_error_deg 	      # [deg] Heading error of the pure pursuit controller
uint8   state_machine         # Driving state of the rover [0: SPOT_TURNING, 1: DRIVING, 2: GOAL_REACHED]

# TOPICS rover_differential_guidance_status

uint64 timestamp        		# time since system start (microseconds)

bool ignition_on          		# activate/deactivate ignition (spark plug)
float32 throttle_control		# setpoint for throttle actuator, with slew rate if enabled, idles with 0 [norm] [@range 0,1] [@uncontrolled NAN to stop motor]
float32 choke_control			# setpoint for choke actuator, 1: fully closed [norm] [@range 0,1]
float32 starter_engine_control		# setpoint for (electric) starter motor [norm] [@range 0,1]

uint8 user_request			# user intent for the ICE being on/off

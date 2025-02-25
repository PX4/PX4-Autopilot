# Neural Control Message
uint64 timestamp			# time since system start (microseconds)

float32[15] observation   	  	# Observation vector (pos error (3), att (6d), lin vel (3), ang vel (3))
float32[6] wrench			# Forces and Torques before allocation
float32[4] motor_thrust   	  	# Thrust per motor

int32 controller_time   	  		# Time spent from input to output in microseconds
int32 control_inference_time		# Time spent in the control inference in microseconds
int32 allocation_inference_time 		# Time spent in the allocation inference in microseconds

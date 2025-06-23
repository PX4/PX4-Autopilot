uint64 timestamp			# time since system startup				(microseconds)

uint8 TASK_INIT = 0
uint8 TASK_DEFAULT = 1
uint8 TASK_AUTONOMOUS = 2
uint8 TASK_REMOTE_CONTROLLED = 7

uint8 task 				# task that the drone is currently in. see TASK_*

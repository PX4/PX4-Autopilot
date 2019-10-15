uint64 timestamp				# time since system start (microseconds)
uint8 NUM_MOTOR_OUTPUTS = 8

uint8 ACTION_STOP = 0				# stop motors (disable motor test mode)
uint8 ACTION_RUN = 1				# run motors (enable motor test mode)

uint8 action					# one of ACTION_* (applies to all motors)
uint32 motor_number				# number of motor to spin
float32 value					# output power, range [0..1]

uint8 driver_instance				# select output driver (for boards with multiple outputs, like IO+FMU)

uint8 ORB_QUEUE_LENGTH = 4

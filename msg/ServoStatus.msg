uint64 timestamp					# time since system start (microseconds)
uint8 CONNECTED_SERVO_MAX = 8				# The number of Servos supported. Current (Q2/2013) we support 8 Servos

uint16 counter  					# incremented by the writing thread everytime new data is stored
uint8 servo_count					# number of connected servo's
uint8 servo_online_flags				# Bitmask indicating which servo is online/offline

# servo_online_flags bit 0 : Set to 1 if SERVO0 is online
# servo_online_flags bit 1 : Set to 1 if SERVO1 is online
# servo_online_flags bit 2 : Set to 1 if SERVO2 is online
# servo_online_flags bit 3 : Set to 1 if SERVO3 is online
# servo_online_flags bit 4 : Set to 1 if SERVO4 is online
# servo_online_flags bit 5 : Set to 1 if SERVO5 is online
# servo_online_flags bit 6 : Set to 1 if SERVO6 is online
# servo_online_flags bit 7 : Set to 1 if SERVO7 is online


ServoReport[8] servo


uint64 timestamp				# time since system start (microseconds)
uint8 CONNECTED_SERVO_MAX = 8			# The number of servos supported. Current (Q2/2013) we support 8 servos

uint8 SERVO_CONNECTION_TYPE_PPM = 0		# Traditional PPM servo
uint8 SERVO_CONNECTION_TYPE_SERIAL = 1		# Serial Bus connected servo
uint8 SERVO_CONNECTION_TYPE_ONESHOT = 2		# One Shot PPM
uint8 SERVO_CONNECTION_TYPE_I2C = 3		# I2C
uint8 SERVO_CONNECTION_TYPE_CAN = 4		# CAN-Bus
uint8 SERVO_CONNECTION_TYPE_DSHOT = 5		# DShot

uint16 counter  				# incremented by the writing thread everytime new data is stored

uint8 servo_count				# number of connected servos
uint8 servo_connectiontype			# how servos connected to the system

uint8 servo_online_flags			# Bitmask indicating which servo  is online/offline
# bit 0 : Set to 1 if servo 0 is online
# bit 1 : Set to 1 if servo 1 is online
# bit 2 : Set to 1 if servo 2 is online
# bit 3 : Set to 1 if servo 3 is online
# bit 4 : Set to 1 if servo 4 is online
# bit 5 : Set to 1 if servo 5 is online
# bit 6 : Set to 1 if servo 6 is online
# bit 7 : Set to 1 if servo 7 is online

uint8 servo_armed_flags				# Bitmask indicating which SERVO is armed. For SERVO's where the arming state is not known (returned by the SERVO), the arming bits should always be set.

servo_report[8] servo

uint64 timestamp					# time since system start (microseconds)
uint8 CONNECTED_SERVO_MAX = 8				# The number of SERVOs supported via UAVCAN

uint8 SERVO_CONNECTION_TYPE_PPM = 0			# Traditional PPM SERVO
uint8 SERVO_CONNECTION_TYPE_SERIAL = 1			# Serial Bus connected SERVO
uint8 SERVO_CONNECTION_TYPE_ONESHOT = 2			# One Shot PPM
uint8 SERVO_CONNECTION_TYPE_I2C = 3			# I2C
uint8 SERVO_CONNECTION_TYPE_CAN = 4			# CAN-Bus
uint8 SERVO_CONNECTION_TYPE_DSHOT = 5			# DShot

uint16 counter  					# incremented by the writing thread everytime new data is stored
uint16 temperature_counter 				# Incremented when new temperature data is stored
uint16 power_counter 					# Incremented when new power data is stored
uint8 servo_count					# number of connected servo's

uint8 servo_connectiontype				# how SERVOs connected to the system. Currently only CAN supports servo telemtry

uint8 servo_online_flags				# Bitmask indicating which SERVO is online/offline
# servo_online_flags bit 0 : Set to 1 if SERVO0 is online
# servo_online_flags bit 1 : Set to 1 if SERVO1 is online
# servo_online_flags bit 2 : Set to 1 if SERVO2 is online
# servo_online_flags bit 3 : Set to 1 if SERVO3 is online
# servo_online_flags bit 4 : Set to 1 if SERVO4 is online
# servo_online_flags bit 5 : Set to 1 if SERVO5 is online
# servo_online_flags bit 6 : Set to 1 if SERVO6 is online
# servo_online_flags bit 7 : Set to 1 if SERVO7 is online

ServoReport[8] servo

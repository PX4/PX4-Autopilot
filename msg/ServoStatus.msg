# Servo status
#
# Overview message that contains general information about all servos.
# In addition it contains the reports of all individual servos.

uint64 timestamp  # [us] Time since system start

uint8 count  # [-] Number of connected servos
uint8 CONNECTED_SERVO_MAX = 8

uint8 online_flags  # [-] Bitmask indicating which servo is online/offline
# online_flags bit 0 : Set to 1 if SERVO0 is online
# online_flags bit 1 : Set to 1 if SERVO1 is online
# online_flags bit 2 : Set to 1 if SERVO2 is online
# online_flags bit 3 : Set to 1 if SERVO3 is online
# online_flags bit 4 : Set to 1 if SERVO4 is online
# online_flags bit 5 : Set to 1 if SERVO5 is online
# online_flags bit 6 : Set to 1 if SERVO6 is online
# online_flags bit 7 : Set to 1 if SERVO7 is online

ServoReport[8] servo

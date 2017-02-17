# definition of the different tune_id
uint8 CUSTOM = 0 # using frequency, duration and strength
uint8 STARTUP = 1
uint8 ERROR = 2
uint8 NOTIFY_POSITIVE = 3
uint8 NOTIFY_NEUTRAL = 4
uint8 NOTIFY_NEGATIVE = 5
uint8 ARMING_WARNING = 6
uint8 BATTERY_WARNING_SLOW = 7
uint8 BATTERY_WARNING_FAST = 8
uint8 GPS_WARNING = 9
uint8 ARMING_FAILURE = 10
uint8 PARACHUTE_RELEASE = 11
uint8 EKF_WARNING = 12
uint8 BARO_WARNING = 13
uint8 SINGLE_BEEP = 14
uint8 HOME_SET = 15

uint8 tune_id
uint16 frequency
uint32 duration   # in us
uint8 strength    # value between 0-100 if supported by backend

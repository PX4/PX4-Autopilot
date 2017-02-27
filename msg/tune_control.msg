# This message is used to control the tunes, when the tune_id is set to CUSTOM
# then the frequency, duration are used otherwise those values are ignored.

# definition of the different tune_id
uint8 TUNE_ID_CUSTOM = 0
uint8 TUNE_ID_STARTUP = 1
uint8 TUNE_ID_ERROR = 2
uint8 TUNE_ID_NOTIFY_POSITIVE = 3
uint8 TUNE_ID_NOTIFY_NEUTRAL = 4
uint8 TUNE_ID_NOTIFY_NEGATIVE = 5
uint8 TUNE_ID_ARMING_WARNING = 6
uint8 TUNE_ID_BATTERY_WARNING_SLOW = 7
uint8 TUNE_ID_BATTERY_WARNING_FAST = 8
uint8 TUNE_ID_GPS_WARNING = 9
uint8 TUNE_ID_ARMING_FAILURE = 10
uint8 TUNE_ID_PARACHUTE_RELEASE = 11
uint8 TUNE_ID_EKF_WARNING = 12
uint8 TUNE_ID_BARO_WARNING = 13
uint8 TUNE_ID_SINGLE_BEEP = 14
uint8 TUNE_ID_HOME_SET = 15

uint8 tune_id
uint8 tune_override  # if set to 1 the tune which is playing will be stopped and the new started
uint16 frequency     # in Hz
uint32 duration      # in us
uint8 strength       # value between 0-100 if supported by backend

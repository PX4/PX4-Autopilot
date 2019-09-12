# This message is used to control the tunes, when the tune_id is set to CUSTOM
# then the frequency, duration are used otherwise those values are ignored.

uint64 timestamp     # time since system start (microseconds)

uint8 tune_id        # tune_id corresponding to TuneID::* from the tune_defaults.h in the tunes library
uint8 tune_override  # if set to 1 the tune which is playing will be stopped and the new started
uint16 frequency     # in Hz
uint32 duration      # in us
uint32 silence       # in us
uint8 volume         # value between 0-100 if supported by backend

uint8 VOLUME_LEVEL_MIN = 0
uint8 VOLUME_LEVEL_DEFAULT = 40
uint8 VOLUME_LEVEL_MAX = 100

uint8 ORB_QUEUE_LENGTH = 3

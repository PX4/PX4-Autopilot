# Manual control setpoint message
#
# This message provides a representation of a manual control input, such as an RC controller or MAVLink controller (Joystick).
# It defines the manual_control_input topic to represent configured inputs, and the manual_control_setpoint topic ot represent the selected input.
# The message includes fields for the roll, pitch, yaw, throttle and flaps, along with auxiliary channels and button states.
#
# Note: On a common RC mode 1/2/3/4 remote/joystick the stick deflection: -1 is down/left, 1 is up/right.
# Note: QGC sends throttle/z in range [0,1000] - [0,1]. The MAVLink input conversion [0,1] to [-1,1] is at the moment kept backwards compatible.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

bool valid # True if the current data is valid.

uint8 data_source [@enum SOURCE] # Source of the manual control setpoint
uint8 SOURCE_UNKNOWN = 0 # Unknown source
uint8 SOURCE_RC = 1 # Radio control (input_rc)
uint8 SOURCE_MAVLINK_0 = 2 # MAVLink instance 0
uint8 SOURCE_MAVLINK_1 = 3 # MAVLink instance 1
uint8 SOURCE_MAVLINK_2 = 4 # MAVLink instance 2
uint8 SOURCE_MAVLINK_3 = 5 # MAVLink instance 3
uint8 SOURCE_MAVLINK_4 = 6 # MAVLink instance 4
uint8 SOURCE_MAVLINK_5 = 7 # MAVLink instance 5


float32 roll # [@range -1,1] [@invalid NaN] Positive values are generally used for: move right, positive roll rotation, right side down
float32 pitch # [@range -1,1] [@invalid NaN] Positive values are generally used for: move forward, negative pitch rotation, nose down
float32 yaw # [@range -1,1] [@invalid NaN] Positive values are generally used for: positive yaw rotation, clockwise when seen top down
float32 throttle # [@range -1,1] [@invalid NaN] Positive values are generally used for: move up, positive thrust, -1 is minimum available 0% or -100% +1 is 100% thrust

float32 flaps # [@range -1, 1] [@invalid NaN] Position of flaps switch/knob/lever

float32 aux1 # [@range -1,1] [@invalid NaN] Auxiliary channel 1
float32 aux2 # [@range -1,1] [@invalid NaN] Auxiliary channel 2
float32 aux3 # [@range -1,1] [@invalid NaN] Auxiliary channel 3
float32 aux4 # [@range -1,1] [@invalid NaN] Auxiliary channel 4
float32 aux5 # [@range -1,1] [@invalid NaN] Auxiliary channel 5
float32 aux6 # [@range -1,1] [@invalid NaN] Auxiliary channel 6

bool sticks_moving # True if any of the values are changing.

uint16 buttons # From uint16 buttons field of MAVLink MANUAL_CONTROL message

# TOPICS manual_control_setpoint manual_control_input
# DEPRECATED: float32 x
# DEPRECATED: float32 y
# DEPRECATED: float32 z
# DEPRECATED: float32 r

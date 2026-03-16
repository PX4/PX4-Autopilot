# Servo report
#
# Contains information about a specific servo (position, force, speed, load).
# Also contains auxiliary power data (voltage, current) and temperature.

uint64 timestamp  # [us] Time since system start

uint8 actuator_id  # [-] DroneCAN actuator id of servo. Set to 0 for non DroneCAN servos
uint8 node_id      # [-] DroneCAN node id of servo. Set to 0 for non DroneCAN servos

float32 position          # meter or radian
float32 force             # Newton or Newton metre
float32 speed             # meter per second or radian per second
float32 power_rating_pct  # [%] [@range 0, 100] Servo load

uint8 function  # [-] Servo output function

uint16 temperature_counter  # [-] Incremented when temperature data got updated
float32 temperature         # [degC] Servo temperature.

uint8 temperature_error_flags  # [@enum ERROR_FLAG] Temperature error flags
uint8 ERROR_FLAG_OVERHEATING = 1
uint8 ERROR_FLAG_OVERCOOLING = 2

uint16 power_counter  # [-] Incremented when power data got updated
float32 voltage       # [V] Servo voltage
float32 current       # [A] Servo current draw

uint8 power_error_flags  # [@enum ERROR_FLAG] Power error flags
uint8 ERROR_FLAG_OVERVOLTAGE  = 1
uint8 ERROR_FLAG_UNDERVOLTAGE = 2
uint8 ERROR_FLAG_OVERCURRENT  = 4
uint8 ERROR_FLAG_UNDERCURRENT = 8

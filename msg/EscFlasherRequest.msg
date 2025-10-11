uint64 timestamp					# time since system start (microseconds)

uint32 msg_id                                           # Give a unique message identifier so we can compare when getting an ACK

uint8 request                                           # Request type
# use these items as enum values, they can never change
uint8 REQUEST_ESC_INFO                             = 0  # Request ESC_INFO from one or more ESCs
uint8 REQUEST_FLASHING                             = 1  # Request flashing for one or more ESCs
uint8 REQUEST_CANCEL                               = 2  # Cancel any pending request and return control to DShot
uint8 REQUEST_FLASHING_COMPLETE                    = 3  # Flashing complete, return control to DShot

uint8 motor_flags                                       # Flags for which motors are being requested
# motor_flags bit 0 : Set to 1 if ESC0 is being flashed
# motor_flags bit 1 : Set to 1 if ESC1 is being flashed
# motor_flags bit 2 : Set to 1 if ESC2 is being flashed
# motor_flags bit 3 : Set to 1 if ESC3 is being flashed

# ESC Flasher Request Acknowledgement uORB message.
# Used for acknowledging the ESC Flasher command being received.
# Follows the MAVLink COMMAND_ACK message definition

uint64 timestamp		# time since system start (microseconds)

uint32 msg_id						# Message identifier that we are responding to

uint8 request                                           # Request type
# use these items as enum values, they can never change
uint8 REQUEST_ESC_INFO                             = 0  # Request ESC_INFO from one or more ESCs
uint8 REQUEST_FLASHING                             = 1  # Request flashing for one or more ESCs
uint8 REQUEST_CANCEL                               = 2  # Cancel any pending request and return control to DShot
uint8 REQUEST_FLASHING_COMPLETE                    = 3  # Flashing complete, return control to DShot

uint8 result						# Command acknowledge result
# Result cases. This follows the MAVLink MAV_RESULT enum definition
uint8 ESC_FLASHER_CMD_RESULT_ACCEPTED = 0		# Command ACCEPTED and EXECUTED |
uint8 ESC_FLASHER_CMD_RESULT_TEMPORARILY_REJECTED = 1	# Command TEMPORARY REJECTED/DENIED |
uint8 ESC_FLASHER_CMD_RESULT_DENIED = 2			# Command PERMANENTLY DENIED |
uint8 ESC_FLASHER_CMD_RESULT_UNSUPPORTED = 3		# Command UNKNOWN/UNSUPPORTED |
uint8 ESC_FLASHER_CMD_RESULT_FAILED = 4			# Command executed, but failed |
uint8 ESC_FLASHER_CMD_RESULT_IN_PROGRESS = 5		# Command being executed |
uint8 ESC_FLASHER_CMD_RESULT_CANCELLED = 6		# Command Canceled

uint8[16] fw_major                                      # Firmware versions major
uint8[16] fw_minor                                      # Firmware versions minor
uint16 fw_flags                                         # Bit flags for valid fw versions

uint32[16] gpio_pins                                    # GPIO pin definitions
uint16 gpio_flags                                       # Bit flags for valid gpio pins

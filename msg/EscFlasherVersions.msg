# ESC Flasher Versions uOrb message
#
uint64 timestamp		# time since system start (microseconds)

uint8 result
# use these items as enum values, they can never change
uint8 RESULT_UNKNOWN                                     = 0  #
uint8 RESULT_SUCCESS                                     = 1  #
uint8 RESULT_NO_DATA                                     = 2  #
uint8 RESULT_MISMATCH                                    = 3  #
uint8 RESULT_NEEDS_UPDATE                                = 4  #
uint8 RESULT_BROKEN_ESC                                  = 5  #

uint16 versions_valid # Bit-flags to identify valid version information

uint8[16] esc_type
# use these items as enum values, they can never change
uint8 ESC_TYPE_UNKNOWN                                   = 0  # Unknown
uint8 ESC_TYPE_AM32                                      = 1  # AM32 2.18+
uint8 ESC_TYPE_BLHELI                                    = 2  # BLHeli
uint8 ESC_TYPE_AM32_OLD                                  = 3  # AM32 2.16
uint8 ESC_TYPE_BLUEJAY                                   = 4  # BlueJay

uint8[16] versions_major                                      # Firmware versions major
uint8[16] versions_minor                                      # Firmware versions minor

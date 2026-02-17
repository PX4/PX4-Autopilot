uint64 timestamp                   # time since system start (microseconds)

uint8 esc_index                    # Current ESC being flashed (0-based)
uint8 esc_count                    # Total ESCs being flashed
uint8 progress_pct                 # 0-100 for current ESC
uint8 overall_progress_pct         # 0-100 for entire operation

uint8 status
uint8 STATUS_IDLE = 0
uint8 STATUS_CONNECTING = 1
uint8 STATUS_READING_INFO = 2
uint8 STATUS_ERASING = 3
uint8 STATUS_WRITING = 4
uint8 STATUS_COMPLETE = 5
uint8 STATUS_FAILED = 6
uint8 STATUS_REBOOT_REQUIRED = 7

uint8 error
uint8 ERROR_NONE = 0
uint8 ERROR_NOT_DISARMED = 1
uint8 ERROR_NO_FIRMWARE_FILE = 2
uint8 ERROR_COMMS_TIMEOUT = 3
uint8 ERROR_CRC_MISMATCH = 4
uint8 ERROR_VERIFICATION_FAILED = 5
uint8 ERROR_UNSUPPORTED_HARDWARE = 6
uint8 ERROR_DSHOT_STOP_FAILED = 7

uint32 current_firmware_version    # Currently installed version
uint32 new_firmware_version        # Version being written

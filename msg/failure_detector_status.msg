uint64 timestamp                    # time since system start (microseconds)

# FailureDetector status
uint8 FAILURE_NONE = 0
uint8 FAILURE_ROLL = 1              # (1 << 0)
uint8 FAILURE_PITCH = 2             # (1 << 1)
uint8 FAILURE_ALT = 4               # (1 << 2)
uint8 FAILURE_EXT = 8               # (1 << 3)
uint8 FAILURE_ARM_ESC = 16          # (1 << 4)
uint8 FAILURE_HIGH_WIND = 32        # (1 << 5)
uint8 FAILURE_BATTERY = 64          # (1 << 6)
uint8 FAILURE_IMBALANCED_PROP = 128 # (1 << 7)

uint8 failure_status                # Bitmask containing FailureDetector status

float32 imbalanced_prop_metric      # Metric of the imbalanced propeller check (low-passed)

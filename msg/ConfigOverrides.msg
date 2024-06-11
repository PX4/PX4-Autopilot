# Configurable overrides by (external) modes or mode executors
uint64 timestamp		# time since system start (microseconds)

bool disable_auto_disarm         # Prevent the drone from automatically disarming after landing (if configured)

bool defer_failsafes             # Defer all failsafes that can be deferred (until the flag is cleared)
int16 defer_failsafes_timeout_s  # Maximum time a failsafe can be deferred. 0 = system default, -1 = no timeout


int8 SOURCE_TYPE_MODE = 0
int8 SOURCE_TYPE_MODE_EXECUTOR = 1
int8 source_type

uint8 source_id                  # ID depending on source_type

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS config_overrides config_overrides_request

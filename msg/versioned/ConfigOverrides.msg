# Configurable overrides by (external) modes or mode executors
#
# All three topics this message backs are published and subscribed by the commander module (ModeManagement),
# implementing a request/confirm protocol for external mode or mode-executor overrides.
# An external mode or mode executor publishes config_overrides_request (identifying itself via source_type/source_id)
# to ask for a change to its active overrides; commander applies the change and publishes config_overrides_confirm,
# echoing the same request back to acknowledge it, and separately publishes config_overrides with the merged,
# currently-active set of overrides across all active mode executors for other consumers to observe.

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] Time since system start

bool disable_auto_disarm # Prevent drone from auto-disarming after landing (if configured)

bool defer_failsafes # Defer all deferrable failsafes (until the flag is cleared)
int16 defer_failsafes_timeout_s # [s] [@invalid -1 No timeout] Maximum time a failsafe can be deferred. 0 = system default.
bool disable_auto_set_home # Prevent the drone from automatically setting the home position on arm or takeoff

int8 source_type # [@enum SOURCE_TYPE] Origin of this override
int8 SOURCE_TYPE_MODE = 0 # Override originates from a mode
int8 SOURCE_TYPE_MODE_EXECUTOR = 1 # Override originates from a mode executor

uint8 source_id # [-] ID depending on source_type

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS config_overrides config_overrides_request config_overrides_confirm

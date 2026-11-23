# Configuration overrides from external modes or mode executors
#
# An external mode or mode executor requests overrides on config_overrides_request, identifying itself with source_type and source_id.
# Commander stores them for that mode or executor and echoes the request on config_overrides_confirm as acknowledgement.
# Commander publishes the overrides in effect on config_overrides: those of the current mode, with those of the executor in charge applied on top.

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] Time since system start

bool disable_auto_disarm # Prevent the drone from automatically disarming after landing (if configured)

bool defer_failsafes # Defer all deferrable failsafes (until the flag is cleared)
int16 defer_failsafes_timeout_s # [s] [@invalid -1 No timeout] Maximum time a failsafe can be deferred. 0 = system default.
bool disable_auto_set_home # Prevent the drone from automatically setting the home position on arm or takeoff

int8 source_type # [@enum SOURCE_TYPE] Origin of this override
int8 SOURCE_TYPE_MODE = 0 # Override originates from a mode
int8 SOURCE_TYPE_MODE_EXECUTOR = 1 # Override originates from a mode executor

uint8 source_id # [-] ID assigned at registration (RegisterExtComponentReply): mode_id for SOURCE_TYPE_MODE, mode_executor_id for SOURCE_TYPE_MODE_EXECUTOR

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS config_overrides config_overrides_request config_overrides_confirm

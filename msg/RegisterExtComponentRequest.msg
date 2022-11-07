# Request to register an external component
uint64 timestamp # time since system start (microseconds)

uint64 request_id                  # ID, set this to a random value
char[25] name                      # either the requested mode name, or component name

# Components to be registered
bool register_arming_check
bool register_mode                 # registering a mode also requires arming_check to be set
bool register_mode_executor        # registering an executor also requires a mode to be registered (which is the owned mode by the executor)

bool enable_replace_internal_mode  # set to true if an internal mode should be replaced
uint8 replace_internal_mode        # vehicle_status::NAVIGATION_STATE_*
bool activate_mode_immediately     # switch to the registered mode (can only be set in combination with an executor)


uint8 ORB_QUEUE_LENGTH = 2

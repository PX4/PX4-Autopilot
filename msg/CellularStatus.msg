# Cellular status
#
# This is currently used only for logging cell status from MAVLink.

uint64 timestamp # Time since system start [us]

uint16 status # Status bitmap [@enum STATUS_FLAG]
uint16 STATUS_FLAG_UNKNOWN = 1 # State unknown or not reportable
uint16 STATUS_FLAG_FAILED = 2 # Modem is unusable
uint16 STATUS_FLAG_INITIALIZING = 4 # Modem is being initialized
uint16 STATUS_FLAG_LOCKED = 8 # Modem is locked
uint16 STATUS_FLAG_DISABLED = 16 # Modem is not enabled and is powered down
uint16 STATUS_FLAG_DISABLING = 32 # Modem is currently transitioning to the STATUS_FLAG_DISABLED state
uint16 STATUS_FLAG_ENABLING = 64 # Modem is currently transitioning to the STATUS_FLAG_ENABLED state
uint16 STATUS_FLAG_ENABLED = 128 # Modem is enabled and powered on but not registered with a network provider and not available for data connections
uint16 STATUS_FLAG_SEARCHING = 256 # Modem is searching for a network provider to register
uint16 STATUS_FLAG_REGISTERED = 512 # Modem is registered with a network provider, and data connections and messaging may be available for use
uint16 STATUS_FLAG_DISCONNECTING = 1024 # Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
uint16 STATUS_FLAG_CONNECTING = 2048 # Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
uint16 STATUS_FLAG_CONNECTED = 4096 # One or more packet data bearers is active and connected

uint8 failure_reason # Failure reason [@enum FAILURE_REASON]
uint8 FAILURE_REASON_NONE = 0 # No error
uint8 FAILURE_REASON_UNKNOWN = 1 # Error state is unknown
uint8 FAILURE_REASON_SIM_MISSING = 2 # SIM is required for the modem but missing
uint8 FAILURE_REASON_SIM_ERROR = 3 # SIM is available, but not usable for connection

uint8 type # Cellular network radio type [@enum CELLULAR_NETWORK_RADIO_TYPE]
uint8 CELLULAR_NETWORK_RADIO_TYPE_NONE = 0
uint8 CELLULAR_NETWORK_RADIO_TYPE_GSM = 1
uint8 CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2
uint8 CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3
uint8 CELLULAR_NETWORK_RADIO_TYPE_LTE = 4

uint8 quality # Cellular network RSSI/RSRP, absolute value [dBm]
uint16 mcc # Mobile country code. [@invalid UINT16_MAX]
uint16 mnc # Mobile network code. [@invalid UINT16_MAX]
uint16 lac # Location area code. [@invalid 0]

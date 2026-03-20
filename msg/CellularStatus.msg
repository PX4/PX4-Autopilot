# Cellular status
#
# This is currently used only for logging cell status from MAVLink.

uint64 timestamp  # [us] Time since system start

uint16 status                            # [@enum STATUS_FLAG] Status bitmap
uint16 STATUS_FLAG_UNKNOWN = 1           # State unknown or not reportable
uint16 STATUS_FLAG_FAILED = 2            # Modem is unusable
uint16 STATUS_FLAG_INITIALIZING = 4      # Modem is being initialized
uint16 STATUS_FLAG_LOCKED = 8            # Modem is locked
uint16 STATUS_FLAG_DISABLED = 16         # Modem is not enabled and is powered down
uint16 STATUS_FLAG_DISABLING = 32        # Modem is currently transitioning to the STATUS_FLAG_DISABLED state
uint16 STATUS_FLAG_ENABLING = 64         # Modem is currently transitioning to the STATUS_FLAG_ENABLED state
uint16 STATUS_FLAG_ENABLED = 128         # Modem is enabled and powered on but not registered with a network provider and not available for data connections
uint16 STATUS_FLAG_SEARCHING = 256       # Modem is searching for a network provider to register
uint16 STATUS_FLAG_REGISTERED = 512      # Modem is registered with a network provider, and data connections and messaging may be available for use
uint16 STATUS_FLAG_DISCONNECTING = 1024  # Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
uint16 STATUS_FLAG_CONNECTING = 2048     # Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
uint16 STATUS_FLAG_CONNECTED = 4096      # One or more packet data bearers is active and connected

uint8 failure_reason                  # [@enum FAILURE_REASON] Failure reason
uint8 FAILURE_REASON_NONE = 0         # No error
uint8 FAILURE_REASON_UNKNOWN = 1      # Error state is unknown
uint8 FAILURE_REASON_SIM_MISSING = 2  # SIM is required for the modem but missing
uint8 FAILURE_REASON_SIM_ERROR = 3    # SIM is available, but not usable for connection

uint8 type                                   # [@enum CELLULAR_NETWORK_RADIO_TYPE] Cellular network radio type
uint8 CELLULAR_NETWORK_RADIO_TYPE_NONE = 0   # None
uint8 CELLULAR_NETWORK_RADIO_TYPE_GSM = 1    # GSM
uint8 CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2   # CDMA
uint8 CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3  # WCDMA
uint8 CELLULAR_NETWORK_RADIO_TYPE_LTE = 4    # LTE

uint8 quality  # [dBm] Cellular network RSSI/RSRP, absolute value
uint16 mcc     # [@invalid UINT16_MAX] Mobile country code
uint16 mnc     # [@invalid UINT16_MAX] Mobile network code
uint16 lac     # [@invalid 0] Location area code

uint8 band_number	# LTE frequency band number
float32 band_frequency	# LTE radio frequency in MHz. 0 for invalid
uint16 channel_number	# Channel Number (CN), Absolute radio-frequency (ARFCN) / E-UTRA (EARFCN) / UTRA (UARFCN) / New radio (NR_CH). 0 for invalid
float32 rx_level	# Receiver signal level in dBm. On 3G is Received Signal Code Power (RSCP). On LTE Reference Signal Received Power (RSRP). On 5G is New Radio Reference Signal Received Power (NR_RSRQ). 0 for invalid
float32 tx_level	# Transmitter signal absolte level in dBm. 0 for invalid
float32 rx_quality	# Received signal quality in dBm. On 3G is Receiver Quality (RxQual). On LTE is Reference Signal . 0 for invalidReceived Quality (RSRQ). On 5G is New radio Reference Signal Received Quality (NR_RSRQ). 0 for invalid
uint32 link_tx_rate	# Download rate in KiB/s. 0 for invalid
uint32 link_rx_rate	# Upload rate in KiB/s. 0 for invalid
uint16 ber		# Bit Error Rate, which is determined by comparing the erroneous bits received with the total number of bits received. 0 for invalid
uint8 id		# Cellular instance Number. Index 0 is reserved for an FMU onboard modem. Highest index is equal to MAX_INSTANCES. 0 for invalid
char[9] cell_tower_id	# ID of the currently connected cell tower. Must be NULL terminated if the length is less than 9 human-readable characters, and without NULL termination character if the length is exactly 9 characters. 0 for invalid

uint8 MAX_INSTANCES = 5

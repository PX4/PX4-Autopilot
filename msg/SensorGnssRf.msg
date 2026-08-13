# GNSS RF status
#
# Reports RF block status for a GNSS receiver, decoded from the u-blox UBX-MON-RF message.
# Includes antenna status/power, jamming/interference indicators, automatic gain control (AGC),
# noise level, and I/Q demodulation metrics. Published by the `gps` driver (UBX protocol only), once
# per RF block, via the block-specific topics below.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

uint32 device_id # Unique device ID for the sensor that does not change between power cycles

uint8 block_id  # [-] Only used for multi-topic publishing

uint8 antenna_status # [@enum ANTENNA_STATUS] Status of the antenna supervisor state machine
uint8 ANTENNA_STATUS_INIT = 0 # Initializing
uint8 ANTENNA_STATUS_UNKNOWN = 1 # Status unknown
uint8 ANTENNA_STATUS_OK = 2 # Antenna OK
uint8 ANTENNA_STATUS_SHORT = 3 # Short circuit detected
uint8 ANTENNA_STATUS_OPEN = 4 # Open circuit detected

uint8 antenna_power # [@enum ANTENNA_POWER] Antenna power state
uint8 ANTENNA_POWER_OFF = 0 # Power off
uint8 ANTENNA_POWER_ON = 1 # Power on
uint8 ANTENNA_POWER_UNKNOWN = 2 # Power state unknown

uint32 post_status # [-] Power-on self-test (POST) status word (vendor-specific bitmask)

uint16 noise_per_ms # [-] Noise level as measured by the GNSS receiver core
uint16 automatic_gain_control # [-] [@range 0, 8191] Automatic Gain Control (AGC) monitor value (8191 = 100% of max gain)
uint8 jamming_indicator # [-] [@range 0, 255] Continuous wave (CW) jamming indicator (0 = no CW jamming, 255 = strong CW jamming)

uint8 jamming_state # [@enum JAMMING_STATE] Jamming state
uint8 JAMMING_STATE_UNKNOWN = 0 # Unknown
uint8 JAMMING_STATE_OK = 1 # No jamming detected
uint8 JAMMING_STATE_WARNING = 2 # Interference visible but fix OK
uint8 JAMMING_STATE_CRITICAL = 3 # Interference visible and no fix

int8 i_offset # [-] [@range -128, 127] Imbalance of the I-part of the complex signal (0 = no imbalance)
uint8 i_magnitude # [-] [@range 0, 255] Magnitude of the I-part of the complex signal (0 = no signal, 255 = max magnitude)
int8 q_offset # [-] [@range -128, 127] Imbalance of the Q-part of the complex signal (0 = no imbalance)
uint8 q_magnitude # [-] [@range 0, 255] Magnitude of the Q-part of the complex signal (0 = no signal, 255 = max magnitude)

uint8 rf_band # [@enum RF_BAND] GNSS band associated with the reported RF block (block_id)
uint8 RF_BAND_UNKNOWN = 0 # unknown band or unsupported field
uint8 RF_BAND_L1 = 1 # L1 band
uint8 RF_BAND_L2 = 2 # L2 band
uint8 RF_BAND_L3 = 3 # L3 band
uint8 RF_BAND_L5 = 4 # L5 band

# One topic per RF block ID
# TOPICS sensor_gnss_rf_block0 sensor_gnss_rf_block1

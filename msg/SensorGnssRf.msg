# GNSS RF

uint64 timestamp                  # time since system start (microseconds)
uint64 timestamp_sample           # time since system start (microseconds)

uint32 device_id                  # unique device ID for the sensor that does not change between power cycles

uint8 block_id             # RF block ID (0 = L1 band, 1 = L2 or L5 band depending on product)
uint8 antenna_status       # Status of the antenna superior state machine (0x00=INIT, 0x01=DONTKNOW, 0x02=OK, 0x03=SHORT, 0x04=OPEN)
uint8 antenna_power        # Current power status of antenna (0x00=OFF, 0x01=ON, 0x02=DONTKNOW)
uint32 post_status         # POST status word
uint16 noise_per_ms        # Noise level as measured by the GPS core
uint16 automatic_gain_control # Automatic Gain Control monitor as percentage of maximum gain, range 0 to 8191 (100%)
uint8 jamming_indicator    # CW jamming indicator (0=no CW jamming, 255=strong CW jamming)
int8  i_offset             # Imbalance of I-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
uint8 i_magnitude          # Magnitude of I-part of complex signal (0=no signal, 255=max magnitude)
int8  q_offset             # Imbalance of Q-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
uint8 q_magnitude          # Magnitude of Q-part of complex signal (0=no signal, 255=max magnitude)

# One topic per RF block ID
# TOPICS sensor_gnss_rf_block0 sensor_gnss_rf_block1

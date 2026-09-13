# GNSS spectrum analysis
#
# Reports a basic RF spectrum analysis for a GNSS receiver, decoded from the u-blox UBX-MON-SPAN
# message (enabled via the GPS_UBX_SPECTRUM parameter).
# Published by the `gps` driver (UBX protocol only), once per RF block, via the block-specific topics below.
# The center frequency of bin i is: f(i) = center_frequency + spectrum_span * (i - 127) / 256

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

uint32 device_id # Unique device ID for the sensor that does not change between power cycles

uint8 block_id # [-] Only used for multi-topic publishing. Use center_frequency to determine which RF band it is.

uint8[256] spectrum # [(2^-2)dB] Spectrum magnitude per bin (number of points = spectrum_span / resolution)
uint32 spectrum_span # [Hz] Frequency span covered by the spectrum
uint32 resolution # [Hz] Frequency resolution of the spectrum (bin width)
uint32 center_frequency # [Hz] Center frequency of the spectrum span
uint8 programmable_gain_amplifier # [dB] Programmable gain amplifier setting

# One topic per RF block ID
# TOPICS sensor_gnss_spectrum_block0 sensor_gnss_spectrum_block1 sensor_gnss_spectrum_block2

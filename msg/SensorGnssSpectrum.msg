# GNSS spectrum

uint64 timestamp                  # time since system start (microseconds)
uint64 timestamp_sample           # time since system start (microseconds)

uint32 device_id                  # unique device ID for the sensor that does not change between power cycles

# The center frequency at each bin: f(i) = center + span * (i - 127) / 256
uint8[256] spectrum # dB Spectrum data (number of points = span/res)
uint32 spectrum_span_hz    # Hz Spectrum span
uint32 resolution_hz       # Hz Resolution of the spectrum
uint32 center_frequency_hz # Hz Center of spectrum span

uint8 programmable_gain_amplifier_db # dB Programmable gain amplifier

uint8 ORB_QUEUE_LENGTH = 2

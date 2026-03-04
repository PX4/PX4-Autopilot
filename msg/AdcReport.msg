# ADC raw data.
#
# Communicates raw data from an analog-to-digital converter (ADC) to other modules, such as battery status.

uint64 timestamp      # [us] Time since system start
uint32 device_id      # [-] unique device ID for the sensor that does not change between power cycles
int16[16] channel_id  # [-] ADC channel IDs, negative for non-existent, TODO: should be kept same as array index
int32[16] raw_data    # [-] ADC channel raw value, accept negative value, valid if channel ID is positive
uint32 resolution     # [-] ADC channel resolution
float32 v_ref         # [V] ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)

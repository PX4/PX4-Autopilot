uint64 timestamp		# time since system start (microseconds)

uint8 instance

uint32 device_id                # unique device ID for the selected magnetometer

float32[3] sample		# magnetic field in the NED board axis in Gauss

float32[3] expected_field

float32[3] calibrated_sample

float32 calibrated_sample_magnitude

float32 error

float32 declination
float32 inclination
float32 mag_strength

float32[3] bias_estimate
float32[9] scale_estimate
float32[3] bias_estimate_variance
float32[9] scale_estimate_variance


float32[3] diagonal_scale
float32[3] offdiagonal_scale
float32[3] offsets

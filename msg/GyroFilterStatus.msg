# Gyro filter status
#
# Center frequencies of the dynamic notch filters currently applied to the selected gyro
# (vehicle_angular_velocity), 0 means the filter is disabled. These change at runtime with
# motor RPM and the gyro FFT; the static filter settings and notch bandwidths are parameters.
# Published at a reduced rate for log analysis of the filter tuning.

uint64 timestamp                       # [us] Time since system start
uint64 timestamp_sample                # [us] Sampling timestamp of the last filtered gyro sample

uint8 ESC_RPM_NOTCH_ESCS = 12          # Number of ESCs in esc_rpm_notch_hz
uint8 ESC_RPM_NOTCH_HARMONICS = 7      # Number of harmonics in esc_rpm_notch_hz

float32[84] esc_rpm_notch_hz           # [Hz] ESC RPM notch center frequencies (X axis, the other axes use the same), index: harmonic * ESC_RPM_NOTCH_ESCS + esc

float32[3] fft_notch_x_hz              # [Hz] FFT notch center frequencies, X axis
float32[3] fft_notch_y_hz              # [Hz] FFT notch center frequencies, Y axis
float32[3] fft_notch_z_hz              # [Hz] FFT notch center frequencies, Z axis

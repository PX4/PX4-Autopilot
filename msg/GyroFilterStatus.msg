# Gyro filter status
#
# State of the angular velocity filters applied to the selected gyro (vehicle_angular_velocity).
# Frequencies are the ones currently applied by the filters, 0 means the filter is disabled.
# Published at a reduced rate for log analysis of the filter tuning (SDLOG_PROFILE gyro filter analysis).

uint64 timestamp                       # [us] Time since system start
uint64 timestamp_sample                # [us] Sampling timestamp of the last filtered gyro sample

uint32 device_id                       # [-] Device ID of the filtered gyro

float32 sample_rate_hz                 # [Hz] Rate the filters run at (gyro sample rate)

float32 lowpass_cutoff_hz              # [Hz] Low-pass filter cutoff (IMU_GYRO_CUTOFF)

float32[2] static_notch_hz             # [Hz] Static notch filter center frequencies (IMU_GYRO_NF0_FRQ, IMU_GYRO_NF1_FRQ)
float32[2] static_notch_bandwidth_hz   # [Hz] Static notch filter bandwidths (IMU_GYRO_NF0_BW, IMU_GYRO_NF1_BW)

uint8 ESC_RPM_NOTCH_ESCS = 12          # Number of ESCs in esc_rpm_notch_hz
uint8 ESC_RPM_NOTCH_HARMONICS = 7      # Number of harmonics in esc_rpm_notch_hz

uint8 esc_rpm_notch_harmonics          # [-] Number of ESC RPM notch harmonics in use (IMU_GYRO_DNF_HMC)
float32 esc_rpm_notch_bandwidth_hz     # [Hz] ESC RPM notch filter bandwidth (IMU_GYRO_DNF_BW)
float32[84] esc_rpm_notch_hz           # [Hz] ESC RPM notch center frequencies (X axis), index: harmonic * ESC_RPM_NOTCH_ESCS + esc

float32 fft_notch_bandwidth_hz         # [Hz] FFT notch filter bandwidth
float32[3] fft_notch_x_hz              # [Hz] FFT notch center frequencies, X axis
float32[3] fft_notch_y_hz              # [Hz] FFT notch center frequencies, Y axis
float32[3] fft_notch_z_hz              # [Hz] FFT notch center frequencies, Z axis

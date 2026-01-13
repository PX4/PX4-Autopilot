# Optical flow in XYZ body frame in SI units.

uint64 timestamp               # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id               # unique device ID for the sensor that does not change between power cycles

float32[2] pixel_flow          # (radians) accumulated optical flow in radians where a positive value is produced by a RH rotation about the body axis

float32[3] delta_angle         # (radians) accumulated gyro radians where a positive value is produced by a RH rotation of the sensor about the body axis. (NAN if unavailable)

float32 distance_m             # (meters) Distance to the center of the flow field (NAN if unavailable)

uint32 integration_timespan_us # (microseconds) accumulation timespan in microseconds

uint8 quality                  # Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

float32 max_flow_rate          # (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably

float32 min_ground_distance    # (meters) Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance    # (meters) Maximum distance from ground at which the optical flow sensor operates reliably

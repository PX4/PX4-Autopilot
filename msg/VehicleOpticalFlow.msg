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

# Per-sample diagnostics carried through from sensor_optical_flow. In the
# typical 1:1 pass-through case (SENS_FLOW_RATE >= chip rate) these are the
# values from the single underlying frame. When multiple frames are
# accumulated into one publication: shutter, mode, discard_count and
# mode_change_count are from the latest frame; motion and
# challenging_surface are OR-reduced; chip_health_ok is AND-reduced.

uint8 MODE_UNKNOWN        = 0
uint8 MODE_BRIGHT         = 1
uint8 MODE_LOWLIGHT       = 2
uint8 MODE_SUPER_LOWLIGHT = 3

uint8 mode

uint32 shutter
bool motion
bool challenging_surface
bool chip_health_ok
uint16 discard_count
uint16 mode_change_count

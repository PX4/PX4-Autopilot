uint64 timestamp               # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id               # unique device ID for the sensor that does not change between power cycles

float32[2] pixel_flow          # (radians) optical flow in radians where a positive value is produced by a RH rotation of the sensor about the body axis

float32[3] delta_angle         # (radians) accumulated gyro radians where a positive value is produced by a RH rotation about the body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
bool delta_angle_available

float32 distance_m             # (meters) Distance to the center of the flow field
bool distance_available

uint32 integration_timespan_us # (microseconds) accumulation timespan in microseconds

uint8 quality                  # quality, 0: bad quality, 255: maximum quality

uint32 error_count

float32 max_flow_rate          # (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably

float32 min_ground_distance    # (meters) Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance    # (meters) Maximum distance from ground at which the optical flow sensor operates reliably

uint8 MODE_UNKNOWN        = 0
uint8 MODE_BRIGHT         = 1
uint8 MODE_LOWLIGHT       = 2
uint8 MODE_SUPER_LOWLIGHT = 3

uint8 mode

# Per-sample diagnostics (optional; drivers that don't populate leave at 0).

uint32 shutter                 # exposure integration time in sensor clocks. UINT32_MAX = invalid.
bool motion                    # motion bit was set in this frame
bool challenging_surface       # chip-reported challenging surface (PAA3905 hardware flag; false on chips without it)
bool chip_health_ok            # chip self-reported healthy this frame
uint16 discard_count           # saturating count of frames discarded by driver false-motion guard since boot
uint16 mode_change_count       # saturating count of illumination-mode transitions since boot

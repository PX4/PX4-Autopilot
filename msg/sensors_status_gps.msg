#
# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.
#
uint64 timestamp # time since system start (microseconds)

uint32[2] device_ids
float32[2] inconsistency
bool[2] healthy
uint8[2] priority


# drift metrics
float32[2] drift_rate_hpos # Horizontal position rate magnitude checked using EKF2_REQ_HDRIFT (m/s)
float32[2] drift_rate_vpos # Vertical position rate magnitude checked using EKF2_REQ_VDRIFT (m/s)
float32[2] drift_hspd      # Filtered horizontal velocity magnitude checked using EKF2_REQ_HDRIFT (m/s)

float32[2] error_norm      # normalised gps error

bool[2] fail_fix    # true if the fix type is insufficient (no 3D solution)
bool[2] fail_nsats  # true if number of satellites used is insufficient
bool[2] fail_pdop   # true if position dilution of precision is insufficient
bool[2] fail_hacc   # true if reported horizontal accuracy is insufficient
bool[2] fail_vacc   # true if reported vertical accuracy is insufficient
bool[2] fail_sacc   # true if reported speed accuracy is insufficient
bool[2] fail_hdrift # true if horizontal drift is excessive (can only be used when stationary on ground)
bool[2] fail_vdrift # true if vertical drift is excessive (can only be used when stationary on ground)
bool[2] fail_hspeed # true if horizontal speed is excessive (can only be used when stationary on ground)
bool[2] fail_vspeed # true if vertical speed error is excessive

# Landing target GNSS position in WGS84 coordinates, and optional NED velocity, from a target-mounted receiver.
#
# Published by: mavlink_receiver (when decoding TARGET_ABSOLUTE with position/velocity capability bits set).
# Subscribed by: vision_target_estimator (VTEPosition).
#
# abs_pos_updated / vel_ned_updated tell the estimator which fields in this sample are fresh.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw observation

float64 latitude_deg # [deg] Latitude, allows centimeter level RTK precision
float64 longitude_deg # [deg] Longitude, allows centimeter level RTK precision
float32 altitude_msl_m # [m] Altitude above MSL

float32 eph # [m] GNSS horizontal position accuracy
float32 epv # [m] GNSS vertical position accuracy

bool abs_pos_updated # True if WGS84 position is updated

float32 vel_n_m_s # [m/s] GNSS North velocity
float32 vel_e_m_s # [m/s] GNSS East velocity
float32 vel_d_m_s # [m/s] GNSS Down velocity

float32 s_acc_m_s # [m/s] GNSS speed accuracy estimate

bool vel_ned_updated # True if NED velocity is updated

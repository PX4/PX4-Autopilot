uint64 timestamp                # time since system start (microseconds)

int32 gps_intended              # effective EKF2_GPS_CTRL value (0 = disabled)
int32 of_intended               # effective EKF2_OF_CTRL value (0 = disabled)
int32 ev_intended               # effective EKF2_EV_CTRL value (0 = disabled)
int32[4] agp_intended           # effective EKF2_AGPx_CTRL values (0 = disabled)
int32 baro_intended             # effective EKF2_BARO_CTRL value (0 = disabled)
int32 rng_intended              # effective EKF2_RNG_CTRL value (0 = disabled)
int32 drag_intended             # effective EKF2_DRAG_CTRL value (0 = disabled)
int32 mag_intended              # effective EKF2_MAG_TYPE value (5 = disabled/NONE)
int32 imu_intended              # effective EKF2_IMU_CTRL value (0 = disabled)

uint8 agp_active                # bitmask of AGP instances actively fusing (bit i = instance i)

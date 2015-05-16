#include <systemlib/param/param.h>

// 16 is max name length

// optical flow
PARAM_DEFINE_FLOAT(LPE_FLW_XY, 0.01f); // flow xy std dev

// optical flow sonar
PARAM_DEFINE_FLOAT(LPE_SNR_Z, 0.02f); // flow sonar z std dev

// lidar
PARAM_DEFINE_FLOAT(LPE_LDR_Z, 0.1f); // lidar z std dev

// accelerometers
PARAM_DEFINE_FLOAT(LPE_ACC_XY, 0.6f); // accel xy std dev
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.6f); // accel z std dev

// baro
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 1.5f); // baro z std dev

// gps
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 2.0f); // gps xy std dev
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 5.0f); // gps z std dev
PARAM_DEFINE_FLOAT(LPE_GPS_VXY, 1.0f); // gps vel xy std dev
PARAM_DEFINE_FLOAT(LPE_GPS_VZ, 1.0f); // gps vel z std dev

// vision
PARAM_DEFINE_FLOAT(LPE_VIS_P, 1.0f); // vision pos std dev
PARAM_DEFINE_FLOAT(LPE_VIS_V, 1.0f); // vision vel std dev

// vicon
PARAM_DEFINE_FLOAT(LPE_VIC_P, 0.05f); // vicon pos std dev

// process noise
PARAM_DEFINE_FLOAT(LPE_PN_P, 0.1f); // process noise position
PARAM_DEFINE_FLOAT(LPE_PN_V, 0.1f); // process noise velocity

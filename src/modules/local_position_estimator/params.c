#include <systemlib/param/param.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(LPE_SD_FLW_V, 1.0e-2f); // flow velocity std dev
PARAM_DEFINE_FLOAT(LPE_SD_FLW_Z, 2.0e-2f); // flow z std dev
PARAM_DEFINE_FLOAT(LPE_SD_LDR_Z, 5.0e-2f); // lidar z std dev
PARAM_DEFINE_FLOAT(LPE_SD_ACC_XY, 1.0e-3f); // accel xy std dev
PARAM_DEFINE_FLOAT(LPE_SD_ACC_Z, 1.0e-1f); // accel z std dev
PARAM_DEFINE_FLOAT(LPE_SD_BAR_Z, 5.0f); // baro z std dev
PARAM_DEFINE_FLOAT(LPE_SD_GPS_XY, 1.0f); // gps xy std dev
PARAM_DEFINE_FLOAT(LPE_SD_GPS_Z, 10.0f); // gps z std dev
PARAM_DEFINE_FLOAT(LPE_SD_GPS_VXY, 1.0f); // gps vel xy std dev
PARAM_DEFINE_FLOAT(LPE_SD_GPS_VZ, 10.0f); // gps vel z std dev

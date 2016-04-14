#include <systemlib/param/param.h>

// 16 is max name length


/**
 * Enable accelerometer integration for prediction.
 *
 * @boolean
 * @group Local Position Estimator
 */
PARAM_DEFINE_INT32(LPE_INTEGRATE, 1);

/**
 * Optical flow z offset from center
 *
 * @group Local Position Estimator
 * @unit m
 * @min -1
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FLW_OFF_Z, 0.0f);

/**
 * Optical flow xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FLW_XY, 0.01f);

/**
 * Optical flow minimum quality threshold
 *
 * @group Local Position Estimator
 * @min 0
 * @max 255
 * @decimal 0
 */
PARAM_DEFINE_INT32(LPE_FLW_QMIN, 75);

/**
 * Sonar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_SNR_Z, 0.05f);

/**
 * Sonar z offset from center of vehicle +down
 *
 * @group Local Position Estimator
 * @unit m
 * @min -1
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_SNR_OFF_Z, 0.00f);

/**
 * Lidar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_LDR_Z, 0.03f);

/**
 * Lidar z offset from center of vehicle +down
 *
 * @group Local Position Estimator
 * @unit m
 * @min -1
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_LDR_OFF_Z, 0.00f);

/**
 * Accelerometer xy standard deviation
 *
 * Data sheet sqrt(Noise power) = 150ug/sqrt(Hz)
 * std dev = (150*9.8*1e-6)*sqrt(1000 Hz) m/s^2
 * Since accels sampled at 1000 Hz.
 *
 * should be 0.0464
 *
 * @group Local Position Estimator
 * @unit m/s^2
 * @min 0.00001
 * @max 2
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_ACC_XY, 0.0454f);

/**
 * Accelerometer z standard deviation
 *
 * (see Accel x comments)
 *
 * @group Local Position Estimator
 * @unit m/s^2
 * @min 0.00001
 * @max 2
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.0454f);

/**
 * Barometric presssure altitude z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 3
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 1.0f);

/**
 * Enable GPS
 *
 * @group Local Position Estimator
 * @boolean
 */
PARAM_DEFINE_INT32(LPE_GPS_ON, 1);

/**
 * GPS delay compensaton
 *
 * @group Local Position Estimator
 * @unit sec
 * @min 0
 * @max 0.4
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_DELAY, 0.25f);


/**
 * GPS xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 2.0f);

/**
 * GPS z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 200
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 100.0f);

/**
 * GPS xy velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VXY, 0.25f);

/**
 * GPS z velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VZ, 0.25f);

/**
 * GPS max eph
 *
 * @group Local Position Estimator
 * @unit m
 * @min 1.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_EPH_MAX, 3.0f);

/**
 * Vision xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_VIS_XY, 0.5f);

/**
 * Vision z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_VIS_Z, 0.5f);

/**
 * Enabled vision correction
 *
 * @group Local Position Estimator
 * @boolean
 */
PARAM_DEFINE_INT32(LPE_VIS_ON, 1);

/**
 * Vicon position standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_VIC_P, 0.05f);

/**
 * Position propagation noise density
 *
 * @group Local Position Estimator
 * @unit m/s/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_PN_P, 0.1f);

/**
 * Velocity propagation noise density
 *
 * @group Local Position Estimator
 * @unit (m/s)/s/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_PN_V, 0.1f);

/**
 * Accel bias propagation noise density
 *
 * @group Local Position Estimator
 * @unit (m/s^2)/s/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_PN_B, 1e-3f);

/**
 * Terrain random walk noise density
 *
 * @group Local Position Estimator
 * @unit m/s/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_PN_T, 1e-3f);

/**
 * Flow gyro high pass filter cut off frequency
 *
 * @group Local Position Estimator
 * @unit Hz
 * @min 0
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FGYRO_HP, 0.1f);

/**
 * Home latitude for nav w/o GPS
 *
 * @group Local Position Estimator
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_LAT, 40.430f);

/**
 * Home longitude for nav w/o GPS
 *
 * @group Local Position Estimator
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_LON, -86.929);


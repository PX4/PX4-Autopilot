#include <systemlib/param/param.h>

// 16 is max name length


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
 * Optical flow xy standard deviation linear factor on distance
 *
 * @group Local Position Estimator
 * @unit m / m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FLW_XY_D, 0.01f);

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
 * Accelerometer xy noise density
 *
 * Data sheet noise density = 150ug/sqrt(Hz) = 0.0015 m/s^2/sqrt(Hz)
 *
 * Larger than data sheet to account for tilt error.
 *
 * @group Local Position Estimator
 * @unit m/s^2/srqt(Hz)
 * @min 0.00001
 * @max 2
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_ACC_XY, 0.0015f);

/**
 * Accelerometer z noise density
 *
 * Data sheet noise density = 150ug/sqrt(Hz) = 0.0015 m/s^2/sqrt(Hz)
 *
 * @group Local Position Estimator
 * @unit m/s^2/srqt(Hz)
 * @min 0.00001
 * @max 2
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.0015f);

/**
 * Barometric presssure altitude z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 3
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 3.0f);

/**
 * Enables GPS data, also forces alt init with GPS
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
PARAM_DEFINE_FLOAT(LPE_GPS_DELAY, 0.29f);


/**
 * Minimum GPS xy standard deviation, uses reported EPH if greater.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 1.0f);

/**
 * Minimum GPS z standard deviation, uses reported EPV if greater.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 200
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 3.0f);

/**
 * GPS xy velocity standard deviation.
 * EPV used if greater than this value.
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
 * Max EPH allowed for GPS initialization
 *
 * @group Local Position Estimator
 * @unit m
 * @min 1.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_EPH_MAX, 3.0f);

/**
 * Max EPV allowed for GPS initialization
 *
 * @group Local Position Estimator
 * @unit m
 * @min 1.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_EPV_MAX, 5.0f);

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
 * Vision correction
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
 * @min 0.0001
 * @max 1
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_VIC_P, 0.001f);

/**
 * Position propagation noise density
 *
 * Increase to trust measurements more.
 * Decrease to trust model more.
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
 * Increase to trust measurements more.
 * Decrease to trust model more.
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
 * Terrain random walk noise density, hilly/outdoor (0.1), flat/Indoor (0.001)
 *
 * @group Local Position Estimator
 * @unit (m/s)/(sqrt(hz))
 * @min 0
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_PN_T, 0.001f);

/**
 * Terrain maximum percent grade, hilly/outdoor (100 = 45 deg), flat/Indoor (0 = 0 deg)
 * Used to calculate increased terrain random walk nosie due to movement.
 *
 * @group Local Position Estimator
 * @unit %
 * @min 0
 * @max 100
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_T_MAX_GRADE, 1.0f);

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
 * Local origin latitude for nav w/o GPS
 *
 * @group Local Position Estimator
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_LAT, 40.430f);

/**
 * Local origin longitude for nav w/o GPS
 *
 * @group Local Position Estimator
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_LON, -86.929);

/**
 * Cut frequency for state publication
 *
 * @group Local Position Estimator
 * @unit Hz
 * @min 5
 * @max 1000
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(LPE_X_LP, 5.0f);

/**
 * Required xy standard deviation to publish position
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.3
 * @max 5.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(LPE_XY_PUB, 1.0f);

/**
 * Required z standard deviation to publish altitude/ terrain
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.3
 * @max 5.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(LPE_Z_PUB, 1.0f);

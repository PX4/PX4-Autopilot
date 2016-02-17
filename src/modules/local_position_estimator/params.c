#include <systemlib/param/param.h>

// 16 is max name length


/**
 * Enable local position estimator.
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_INT32(LPE_ENABLED, 1);

/**
 * Enable accelerometer integration for prediction.
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_INT32(LPE_INTEGRATE, 1);

/**
 * Optical flow xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_FLW_XY, 0.01f);

/**
 * Sonar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_SNR_Z, 0.2f);

/**
 * Lidar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_LDR_Z, 0.03f);

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
 */
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.0454f);

/**
 * Barometric presssure altitude z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 3
 */
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 1.0f);

/**
 * GPS xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 2.0f);

/**
 * GPS z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 20
 */
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 10.0f);

/**
 * GPS xy velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VXY, 0.275f);

/**
 * GPS z velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VZ, 0.237f);

/**
 * GPS max eph
 *
 * @group Local Position Estimator
 * @unit m
 * @min 1.0
 * @max 5.0
 */
PARAM_DEFINE_FLOAT(LPE_EPH_MAX, 3.0f);



/**
 * Vision xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_VIS_XY, 0.5f);

/**
 * Vision z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_VIS_Z, 0.5f);

/**
 * Circuit breaker to disable vision input.
 *
 * Set to the appropriate key (328754) to disable vision input.
 *
 * @group Local Position Estimator
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(LPE_NO_VISION, 0);

/**
 * Vicon position standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_VIC_P, 0.05f);

/**
 * Position propagation process noise power (variance*sampling rate).
 *
 * @group Local Position Estimator
 * @unit (m/s^2)-s
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_PN_P, 0.0f);

/**
 * Velocity propagation process noise power (variance*sampling rate).
 *
 * @group Local Position Estimator
 * @unit (m/s)-s
 * @min 0
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_PN_V, 0.0f);

/**
 * Accel bias propagation process noise power (variance*sampling rate).
 *
 * @group Local Position Estimator
 * @unit (m/s)-s
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_PN_B, 1e-8f);

/**
 * Fault detection threshold, for chi-squared dist.
 *
 * TODO add separate params for 1 dof, 3 dof, and 6 dof beta
 * or false alarm rate in false alarms/hr
 *
 * @group Local Position Estimator
 * @unit
 * @min 3
 * @max 1000
 */
PARAM_DEFINE_FLOAT(LPE_BETA_MAX, 1000.0f);

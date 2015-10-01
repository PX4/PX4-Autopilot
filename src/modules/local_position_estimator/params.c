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
 * Accelerometer xy noise power (variance*sampling period).
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2*s
 * @min 0.0001
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_ACC_XY, 0.000736f);

/**
 * Accelerometer z noise power (variance*sampling period).
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2*s
 * @min 0.0001
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.00137f);

/**
 * Barometric presssure altitude z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 3
 */
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 1.05f);

/**
 * GPS xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 0.530f);

/**
 * GPS z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 2.04f);

/**
 * GPS xy velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VXY, 0.152f);

/**
 * GPS z velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VZ, 0.157f);

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
 * Position propagation process noise power (variance*sampling period).
 *
 * @group Local Position Estimator
 * @unit (m/s)^2*s
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_PN_P, 0.0f);

/**
 * Velocity propagation process noise power (variance*sampling period).
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2*s
 * @min 0
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_PN_V, 0.0f);

/**
 * Accel bias process noise power (variance*sampling period).
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2*s
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_PN_B, 0.01f);

/**
 * Fault detection threshold, for chi-squared dist.
 *
 * @group Local Position Estimator
 * @unit
 * @min 3
 * @max 100
 */
PARAM_DEFINE_FLOAT(LPE_BETA_MAX, 30.0f);

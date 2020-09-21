/**
 * Learned value of magnetometer X axis bias.
 * This is the amount of X-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @volatile
 * @category system
 * @unit gauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_MAGBIAS_X, 0.0f);

/**
 * Learned value of magnetometer Y axis bias.
 * This is the amount of Y-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @volatile
 * @category system
 * @unit gauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_MAGBIAS_Y, 0.0f);

/**
 * Learned value of magnetometer Z axis bias.
 * This is the amount of Z-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @volatile
 * @category system
 * @unit gauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_MAGBIAS_Z, 0.0f);

/**
 * ID of Magnetometer the learned bias is for.
 *
 * @group EKF2
 * @volatile
 * @category system
 */
PARAM_DEFINE_INT32(EKF2_MAGBIAS_ID, 0);

/**
 * Magnetic declination
 *
 * @group EKF2
 * @volatile
 * @category system
 * @unit deg
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DECL, 0);

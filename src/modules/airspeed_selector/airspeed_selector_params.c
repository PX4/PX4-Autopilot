
/**
 * Airspeed Selector: Wind estimator wind process noise
 *
 * Wind process noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 1
 * @unit m/s/s
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSP_W_P_NOISE, 0.1f);

/**
 * Airspeed Selector: Wind estimator true airspeed scale process noise
 *
 * Airspeed scale process noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 0.1
 * @unit 1/s
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSP_SC_P_NOISE, 0.0001);

/**
 * Airspeed Selector: Wind estimator true airspeed measurement noise
 *
 * True airspeed measurement noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 4
 * @unit m/s
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSP_TAS_NOISE, 1.4);

/**
 * Airspeed Selector: Wind estimator sideslip measurement noise
 *
 * Sideslip measurement noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 1
 * @unit rad
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSP_BETA_NOISE, 0.3);

/**
 * Airspeed Selector: Gate size for true airspeed fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ARSP_TAS_GATE, 3);

/**
 * Airspeed Selector: Gate size for true sideslip fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ARSP_BETA_GATE, 1);

/**
 * Automatic airspeed scale estimation on
 *
 * Turns the automatic airspeed scale (scale from IAS to CAS/EAS) on or off. It is recommended level (keeping altitude) while performing the estimation. Set to 1 to start estimation (best when already flying). Set to 0 to end scale estimation. The estimated scale is then saved in the ARSP_ARSP_SCALE parameter.
 *
 * @boolean
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ARSP_SCALE_EST, 0);

/**
 * Airspeed scale (scale from IAS to CAS/EAS)
 *
 * Scale can either be entered manually, or estimated in-flight by setting ARSP_SCALE_EST to 1.
 *
 * @min 0.5
 * @max 1.5
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSP_ARSP_SCALE, 1.0f);

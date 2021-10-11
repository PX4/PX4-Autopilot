
/**
 * Airspeed Selector: Wind estimator wind process noise
 *
 * Wind process noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 1
 * @unit m/s^2
 * @decimal 2
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_W_P_NOISE, 0.1f);

/**
 * Airspeed Selector: Wind estimator true airspeed scale process noise
 *
 * Airspeed scale process noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 0.1
 * @unit Hz
 * @decimal 5
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_SC_P_NOISE, 0.0001f);

/**
 * Airspeed Selector: Wind estimator true airspeed measurement noise
 *
 * True airspeed measurement noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 4
 * @unit m/s
 * @decimal 1
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_TAS_NOISE, 1.4f);

/**
 * Airspeed Selector: Wind estimator sideslip measurement noise
 *
 * Sideslip measurement noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 1
 * @unit rad
 * @decimal 3
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_BETA_NOISE, 0.3f);

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
PARAM_DEFINE_INT32(ASPD_TAS_GATE, 3);

/**
 * Airspeed Selector: Gate size for sideslip angle fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_BETA_GATE, 1);

/**
 * Controls when to apply the new esstimated airspeed scale
 *
 * @value 0 Disable airspeed scale estimation completely
 * @value 1 Do not apply the new gains (logging and inside wind estimator)
 * @value 2 Apply the new scale after disarm
 * @value 3 Apply the new gains in air
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_SCALE_APPLY, 2);

/**
 * Scale of airspeed sensor 1
 *
 * This is the scale IAS --> CAS of the first airspeed sensor instance
 *
 * @min 0.5
 * @max 2.0
 * @decimal 2
 * @reboot_required true
 * @group Airspeed Validator
 * @volatile
 */
PARAM_DEFINE_FLOAT(ASPD_SCALE_1, 1.0f);

/**
 * Scale of airspeed sensor 2
 *
 * This is the scale IAS --> CAS of the second airspeed sensor instance
 *
 * @min 0.5
 * @max 2.0
 * @decimal 2
 * @reboot_required true
 * @group Airspeed Validator
 * @volatile
 */
PARAM_DEFINE_FLOAT(ASPD_SCALE_2, 1.0f);

/**
 * Scale of airspeed sensor 3
 *
 * This is the scale IAS --> CAS of the third airspeed sensor instance
 *
 * @min 0.5
 * @max 2.0
 * @decimal 2
 * @reboot_required true
 * @group Airspeed Validator
 * @volatile
 */
PARAM_DEFINE_FLOAT(ASPD_SCALE_3, 1.0f);

/**
 * Index or primary airspeed measurement source
 *
 * @value -1 Disabled
 * @value 0 Groundspeed minus windspeed
 * @value 1 First airspeed sensor
 * @value 2 Second airspeed sensor
 * @value 3 Third airspeed sensor
 *
 * @reboot_required true
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_PRIMARY, 1);


/**
 * Enable checks on airspeed sensors
 *
 * If set to true then the data comming from the airspeed sensors is checked for validity. Only applied if ASPD_PRIMARY > 0.
 *
 * @boolean
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_DO_CHECKS, 1);

/**
 * Enable fallback to sensor-less airspeed estimation
 *
 * If set to true and airspeed checks are enabled, it will use a sensor-less airspeed estimation based on groundspeed
 * minus windspeed if no other airspeed sensor available to fall back to.
 *
 * @value 0 Disable fallback to sensor-less estimation
 * @value 1 Enable fallback to sensor-less estimation
 * @boolean
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_FALLBACK_GW, 0);

/**
 * Airspeed failsafe consistency threshold
 *
 * This specifies the minimum airspeed test ratio required to trigger a failsafe. Larger values make the check less sensitive,
 * smaller values make it more sensitive. Start with a value of 1.0 when tuning. When tas_test_ratio is > 1.0 it indicates the
 * inconsistency between predicted and measured airspeed is large enough to cause the wind EKF to reject airspeed measurements.
 * The time required to detect a fault when the threshold is exceeded depends on the size of the exceedance and is controlled by the ASPD_FS_INTEG parameter.
*
 * @min 0.5
 * @max 3.0
 * @decimal 1
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_FS_INNOV, 1.0f);

/**
 * Airspeed failsafe consistency delay
 *
 * This sets the time integral of airspeed test ratio exceedance above ASPD_FS_INNOV required to trigger a failsafe.
 * For example if ASPD_FS_INNOV is 1 and estimator_status.tas_test_ratio is 2.0, then the exceedance is 1.0 and the integral will
 * rise at a rate of 1.0/second. A negative value disables the check. Larger positive values make the check less sensitive, smaller positive values
 * make it more sensitive.
 *
 * @unit s
 * @max 30.0
 * @decimal 1
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_FS_INTEG, 5.0f);

/**
 * Airspeed failsafe stop delay
 *
 * Delay before stopping use of airspeed sensor if checks indicate sensor is bad.
 *
 * @unit s
 * @group Airspeed Validator
 * @min 1
 * @max 10
 */
PARAM_DEFINE_INT32(ASPD_FS_T_STOP, 2);

/**
 * Airspeed failsafe start delay
 *
 * Delay before switching back to using airspeed sensor if checks indicate sensor is good.
 * Set to a negative value to disable the re-enabling in flight.
 *
 * @unit s
 * @group Airspeed Validator
 * @min -1
 * @max 1000
 */
PARAM_DEFINE_INT32(ASPD_FS_T_START, -1);

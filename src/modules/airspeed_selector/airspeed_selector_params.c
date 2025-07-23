
/**
 * Wind estimator wind process noise spectral density
 *
 * Wind process noise of the internal wind estimator(s) of the airspeed selector.
 * When unaided, the wind estimate uncertainty (1-sigma, in m/s) increases by this amount every second.
 *
 * @min 0
 * @max 1
 * @unit m/s^2/sqrt(Hz)
 * @decimal 2
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_WIND_NSD, 1.e-1f);

/**
 * Wind estimator true airspeed scale process noise spectral density
 *
 * Airspeed scale process noise of the internal wind estimator(s) of the airspeed selector.
 * When unaided, the scale uncertainty (1-sigma, unitless) increases by this amount every second.
 *
 * @min 0
 * @max 0.1
 * @unit 1/s/sqrt(Hz)
 * @decimal 5
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_SCALE_NSD, 1.e-4f);

/**
 * Wind estimator true airspeed measurement noise
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
 * Wind estimator sideslip measurement noise
 *
 * Sideslip measurement noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 1
 * @unit rad
 * @decimal 3
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_BETA_NOISE, 0.15f);

/**
 * Gate size for true airspeed fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_TAS_GATE, 4);

/**
 * Gate size for sideslip angle fusion
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
 * Controls when to apply the new estimated airspeed scale(s)
 *
 * @value 0 Do not automatically apply the estimated scale
 * @value 1 Apply the estimated scale after disarm
 * @value 2 Apply the estimated scale in air
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
 * @value 0 Groundspeed minus windspeed
 * @value 1 First airspeed sensor
 * @value 2 Second airspeed sensor
 * @value 3 Third airspeed sensor
 * @value 4 Thrust based airspeed
 *
 * @reboot_required true
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_PRIMARY, 1);


/**
 * Enable checks on airspeed sensors
 *
 * Controls which checks are run to check airspeed data for validity. Only applied if ASPD_PRIMARY > 0.
 *
 * @min 0
 * @max 31
 * @bit 0 Only data missing check (triggers if more than 1s no data)
 * @bit 1 Data stuck (triggers if data is exactly constant for 2s in FW mode)
 * @bit 2 Innovation check (see ASPD_FS_INNOV)
 * @bit 3 Load factor check (triggers if measurement is below stall speed)
 * @bit 4 First principle check (airspeed change vs. throttle and pitch)
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_DO_CHECKS, 7);

/**
 * Fallback options
 *
 * @value 0 Fallback only to other airspeed sensors
 * @value 1 Fallback to groundspeed-minus-windspeed airspeed estimation
 * @value 2 Fallback to thrust based airspeed estimation
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_FALLBACK, 0);

/**
 * Airspeed failure innovation threshold
 *
 * This specifies the minimum airspeed innovation required to trigger a failsafe. Larger values make the check less sensitive,
 * smaller values make it more sensitive. Large innovations indicate an inconsistency between predicted (groundspeed - windspeeed)
 * and measured airspeed.
 * The time required to detect a fault when the threshold is exceeded depends on the size of the exceedance and is controlled by the ASPD_FS_INTEG parameter.
 *
 * @unit m/s
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_FS_INNOV, 5.f);

/**
 * Airspeed failure innovation integral threshold
 *
 * This sets the time integral of airspeed innovation exceedance above ASPD_FS_INNOV required to trigger a failsafe.
 * Larger values make the check less sensitive, smaller positive values make it more sensitive.
 *
 * @unit m
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_FS_INTEG, 10.f);

/**
 * Airspeed failsafe stop delay
 *
 * Delay before stopping use of airspeed sensor if checks indicate sensor is bad.
 *
 * @unit s
 * @group Airspeed Validator
 * @min 0.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(ASPD_FS_T_STOP, 1.f);

/**
 * Airspeed failsafe start delay
 *
 * Delay before switching back to using airspeed sensor if checks indicate sensor is good.
 * Set to a negative value to disable the re-enabling in flight.
 *
 * @unit s
 * @group Airspeed Validator
 * @min -1.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(ASPD_FS_T_START, -1.f);

/**
 * Horizontal wind uncertainty threshold for synthetic airspeed.
 *
 * The synthetic airspeed estimate (from groundspeed and heading) will be declared valid
 * as soon and as long the horizontal wind uncertainty is below this value.
 *
 * @unit m/s
 * @min 0.001
 * @max 5
 * @decimal 3
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_WERR_THR, 0.55f);

/**
 * First principle airspeed check time window
 *
 * Window for comparing airspeed change to throttle and pitch change.
 * Triggers when the airspeed change within this window is negative while throttle increases
 * and the vehicle pitches down.
 * Is meant to catch degrading airspeed blockages as can happen when flying through icing conditions.
 * Relies on  FW_THR_TRIM being set accurately.
 *
 * @unit s
 * @min 0
 * @decimal 1
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_FP_T_WINDOW, 2.0f);

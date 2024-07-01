
/**
 * Multicopter air-mode
 *
 * The air-mode enables the mixer to increase the total thrust of the multirotor
 * in order to keep attitude and rate control even at low and high throttle.
 *
 * This function should be disabled during tuning as it will help the controller
 * to diverge if the closed-loop is unstable (i.e. the vehicle is not tuned yet).
 *
 * Enabling air-mode for yaw requires the use of an arming switch.
 *
 * @value 0 Disabled
 * @value 1 Roll/Pitch
 * @value 2 Roll/Pitch/Yaw
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(MC_AIRMODE, 0);

/**
 * Multicopter yaw margin percentage.
 *
 * The maximum percentage of collective thrust to sacrifice for yaw authority.
 * Note that this parameter is ignored when airmode is enabled for yaw.
 *
 * @unit %
 * @min 0
 * @max 30
 * @increment 0.1
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(MC_YAW_MARGIN, 15.0f);

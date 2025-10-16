
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
 * Wing Deploy RC AUX Channel
 *
 * RC AUX channel to use for manual wing deployment control.
 * Set to 0 to disable manual RC control of wing deployment.
 * The wing deployment will still work automatically via rocket mode manager.
 *
 * @value 0 Disabled
 * @value 1 RC_AUX1
 * @value 2 RC_AUX2
 * @value 3 RC_AUX3
 * @value 4 RC_AUX4
 * @value 5 RC_AUX5
 * @value 6 RC_AUX6
 * @min 0
 * @max 6
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(WD_RC_AUX_CH, 2);

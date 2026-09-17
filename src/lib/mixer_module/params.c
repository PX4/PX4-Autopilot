
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
 * Reduce thrust to preserve yaw when actuators saturate
 *
 * When airmode is disabled (MC_AIRMODE=0) or limited to roll/pitch (MC_AIRMODE=1),
 * sequential desaturation can reduce collective thrust by up to 15% so that some
 * yaw remains available at high throttle.
 *
 * Disabled by default: keep the commanded thrust and clip yaw instead.
 * Enable to restore the 15% yaw-margin thrust reduction.
 * Roll and pitch still reduce thrust when they saturate.
 *
 * Has no effect with full airmode (MC_AIRMODE=2), which does not use this yaw path.
 *
 * @boolean
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(MC_REDUCE_THRUST, 0);

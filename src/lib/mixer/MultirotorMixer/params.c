/**
 * MC Drone mass
 *
 * @min 0.0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Dynamic
 */
PARAM_DEFINE_FLOAT(MC_MASS, 8.5f);

/**
 * MC Inertia XX
 *
 * The inertial moment XX
 *
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Inertia
 */
PARAM_DEFINE_FLOAT(MC_INERTIA_XX, 0.04f);

/**
 * MC Inertia YY
 *
 * The inertial moment YY
 *
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Inertia
 */
PARAM_DEFINE_FLOAT(MC_INERTIA_YY, 0.04f);

/**
 * MC Inertia ZZ
 *
 * The inertial moment ZZ
 *
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Inertia
 */
PARAM_DEFINE_FLOAT(MC_INERTIA_ZZ, 0.04f);

/**
 * MC Lift coeficient
 *
 * @min 0.0
 * @max 1
 * @decimal 9
 * @increment 0.000000001
 * @group Multicopter Dynamic
 */
PARAM_DEFINE_FLOAT(MC_CT_LIFT, 0.00011f);

/**
 * MC Drag coeficient
 *
 * @min 0.0
 * @max 1
 * @decimal 9
 * @increment 0.000000001
 * @group Multicopter Dynamic
 */
PARAM_DEFINE_FLOAT(MC_CM_DRAG, 2.5e-06f);

/**
 * MC ang max
 *
 * @min -360
 * @max 360
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Dynamic
 */
PARAM_DEFINE_FLOAT(ANG_TILT_MAX, 90);

/**
 * MC ang min
 *
 * @min -360
 * @max 360
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Dynamic
 */
PARAM_DEFINE_FLOAT(ANG_TILT_MIN, -2);

/**
 * PWM TILT MAX 1
 * @min 1000
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_TILT_MAX1, 2000);

/**
 * PWM TILT MAX 2
 * @min 1000
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_TILT_MAX2, 2000);

/**
 * PWM TILT MIN 1
 * @min 1000
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_TILT_MIN1, 2000);

/**
 * PWM TILT MIN 2
 * @min 1000
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_TILT_MIN2, 2000);



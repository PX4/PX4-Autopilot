// #include <px4_platform_common/px4_config.h>
// #include <parameters/param.h>

/**
 * @file cdus_rate_control_params.c
 *
 * Parameters for multicopter rate controller
 */
/**
 * Cdus roll rate P gain
 *
 * Proportional gain for roll body rate.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_P_ROLL, 0.1f);

/**
 * Cdus pitch rate P gain
 *
 * @unit 1/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_P_PITCH, 0.1f);

/**
 * Cdus yaw rate P gain
 *
 * @unit 1/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_P_YAW, 0.05f);

/**
 * Cdus roll rate I gain
 *
 * @unit 1/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_I_ROLL, 0.05f);

/**
 * Cdus pitch rate I gain
 *
 * @unit 1/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_I_PITCH, 0.05f);

/**
 * Cdus yaw rate I gain
 *
 * @unit 1/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_I_YAW, 0.02f);

/**
 * Cdus roll rate D gain
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_D_ROLL, 0.002f);

/**
 * Cdus pitch rate D gain
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_D_PITCH, 0.002f);

/**
 * Cdus yaw rate D gain
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_D_YAW, 0.001f);

/**
 * Cdus rate controller integrator limit
 *
 * Absolute integrator limit applied to all axes.
 *
 * @unit rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group Cdus Rate Control
 */
PARAM_DEFINE_FLOAT(CDUS_INT_LIM, 0.5f);

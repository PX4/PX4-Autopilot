/**
 * @file indi_att_control_parameters.cpp
 * Parameters for indi attitude controller.
 *
 * @author Rohan Inamdar <rninamdar@wpi.edu>
 */

/**
 * INDI LMS adaptation: roll µ₂
 *
 * µ₂_roll is the adaptation gain for the roll‐axis in the LMS update:
 *   G_roll <- G_roll  –  µ₂_roll * e_roll * z^T * µ₁
 *
 * Typical order: 1e-5…1e-3
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU2_ROLL, 0.000100f);

/**
 * INDI LMS adaptation: pitch µ₂
 *
 * µ₂_pitch is the adaptation gain for the pitch‐axis in the LMS update.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU2_PITCH, 0.000100f);

/**
 * INDI LMS adaptation: yaw µ₂
 *
 * µ₂_yaw is the adaptation gain for the yaw‐axis in the LMS update.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU2_YAW, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 0
 *
 * µ₁_i are the per‐column normalization gains in the LMS update.
 * Use values ~1e-4…1e-2
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_0, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 1
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_1, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 2
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_2, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 3
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_3, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 4
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_4, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 5
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_5, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 6
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_6, 0.000100f);

/**
 * INDI LMS adaptation: µ₁ column 7
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_MU1_7, 0.000100f);

/**
 * INDI gyro filter cutoff (ωₙ)
 *
 * Second‐order Butterworth cutoff frequency for body‐rate filtering (rad/s).
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1.0
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_GYRO_WN, 50.0f);

/**
 * INDI gyro filter damping (ζ)
 *
 * Damping factor for the 2nd‐order gyro filter (≈0.5…1.0).
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_GYRO_Z, 0.55f);

/**
 * INDI ESC‐RPM filter cutoff (ωₙ)
 *
 * Second‐order Butterworth cutoff frequency for ESC‐RPM/ω measurement filtering (rad/s).
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1.0
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_RPM_WN, 50.0f);

/**
 * INDI ESC‐RPM filter damping (ζ)
 *
 * Damping factor for the 2nd‐order ESC‐RPM filter (≈0.5…1.0).
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_RPM_Z, 0.55f);

/**
 * INDI inner‐loop update period
 *
 * Time step between successive INDI inner‐loop executions (controller sample time).
 * A value of 0.0025 s corresponds to a 400 Hz update rate.
 *
 * @unit s
 * @min 0.001
 * @max 0.020
 * @decimal 4
 * @increment 0.0001
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_INNER_LOOP_FREQUENCY, 0.0025f);

/**
 * INDI inner‐loop roll rate P gain (k₁)
 *
 * Proportional gain on roll‐rate error in the INDI inner loop.
 * Units are 1/s: maps a [rad/s] error to a [rad/s²] angular acceleration correction.
 *
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 1.0
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_KP_ROLL, 50.0f);

/**
 * INDI inner‐loop pitch rate P gain (k₂)
 *
 * Proportional gain on pitch‐rate error in the INDI inner loop.
 *
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 1.0
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_KP_PITCH, 50.0f);

/**
 * INDI inner‐loop yaw rate P gain (k₃)
 *
 * Proportional gain on yaw‐rate error in the INDI inner loop.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group INDI Attitude Control
 */
PARAM_DEFINE_FLOAT(INDI_KP_YAW, 20.0f);


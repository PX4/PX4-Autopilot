/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file spacecraft_params.c
 * Parameters for spacecraft vehicle type.
 *
 * @author Pedro Roque <padr@kth.se>
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = SC_ROLLRATE_K * (SC_ROLLRATE_P * error
 * 			     + SC_ROLLRATE_I * error_integral
 * 			     + SC_ROLLRATE_D * error_derivative)
 * Set SC_ROLLRATE_P=1 to implement a PID in the ideal form.
 * Set SC_ROLLRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = SC_PITCHRATE_K * (SC_PITCHRATE_P * error
 * 			     + SC_PITCHRATE_I * error_integral
 * 			     + SC_PITCHRATE_D * error_derivative)
 * Set SC_PITCHRATE_P=1 to implement a PID in the ideal form.
 * Set SC_PITCHRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_YAWRATE_P, 10.0f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_YAWRATE_I, 0.865f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_YR_INT_LIM, 0.2f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = SC_YAWRATE_K * (SC_YAWRATE_P * error
 * 			     + SC_YAWRATE_I * error_integral
 * 			     + SC_YAWRATE_D * error_derivative)
 * Set SC_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set SC_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_YAWRATE_K, 1.0f);

/**
 * Max acro roll rate
 *
 * default: 2 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ACRO_R_MAX, 720.0f);

/**
 * Max acro pitch rate
 *
 * default: 2 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ACRO_P_MAX, 720.0f);

/**
 * Max acro yaw rate
 *
 * default 1.5 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ACRO_Y_MAX, 540.0f);

/**
 * Acro mode Expo factor for Roll and Pitch.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ACRO_EXPO, 0.69f);

/**
 * Acro mode Expo factor for Yaw.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ACRO_EXPO_Y, 0.69f);

/**
 * Acro mode SuperExpo factor for Roll and Pitch.
 *
 * SuperExpo factor for refining the input curve shape tuned using SC_ACRO_EXPO.
 *
 * 0 Pure Expo function
 * 0.7 reasonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ACRO_SUPEXPO, 0.7f);

/**
 * Acro mode SuperExpo factor for Yaw.
 *
 * SuperExpo factor for refining the input curve shape tuned using SC_ACRO_EXPO_Y.
 *
 * 0 Pure Expo function
 * 0.7 reasonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_ACRO_SUPEXPOY, 0.7f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_INT32(SC_BAT_SCALE_EN, 0);

/**
 * Manual mode maximum force.
 *
 * *
 * @min 0
 * @max 1.0
 * @decimal 2
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_MAN_F_MAX, 1.0f);

/**
 * Manual mode maximum torque.
 *
 * *
 * @min 0
 * @max 1.0
 * @decimal 2
 * @group Spacecraft Rate Control
 */
PARAM_DEFINE_FLOAT(SC_MAN_T_MAX, 1.0f);

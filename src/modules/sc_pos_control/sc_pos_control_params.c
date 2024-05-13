/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * Proportional gain for position error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_POS_P, 0.95f);

/**
 * Proportional gain for velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 2
 * @max 15
 * @decimal 2
 * @increment 0.1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VEL_P, 4.f);

/**
 * Integral gain for velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 2
 * @max 15
 * @decimal 2
 * @increment 0.1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VEL_I, 4.f);

/**
 * Derivative gain for velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 2
 * @max 15
 * @decimal 2
 * @increment 0.1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VEL_D, 4.f);

/**
 * Maximum velocity
 *
 * Absolute maximum for all velocity controlled modes.
 * Any higher value is truncated.
 *
 * @unit m/s
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VEL_MAX, 12.f);

/**
 * Overall Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * MPC_VEL_MAX or MPC_VEL_MANUAL).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -20
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VEL_ALL, -10.f);

/**
 * Cruising elocity setpoint in autonomous modes
 *
 * @unit m/s
 * @min 3
 * @max 20
 * @increment 1
 * @decimal 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VEL_CRUISE, 10.f);

/**
 * Maximum velocity setpoint in Position mode
 *
 * @unit m/s
 * @min 3
 * @max 20
 * @increment 1
 * @decimal 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VEL_MANUAL, 10.f);

/**
 * Maximum collective thrust
 *
 * Limit allowed thrust
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.05
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_THR_MAX, 1.f);

/**
 * Acceleration for autonomous and for manual modes
 *
 * When piloting manually, this parameter is only used in MPC_POS_MODE 4.
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * @increment 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_ACC, 3.f);

/**
 * Maximum accelaration in autonomous modes
 *
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * @increment 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_ACC_MAX, 5.f);

/**
 * Jerk limit in autonomous modes
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions but also limited agility.
 *
 * @unit m/s^3
 * @min 1
 * @max 80
 * @decimal 1
 * @increment 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_JERK_AUTO, 4.f);

/**
 * Maximum jerk in Position/Altitude mode
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother motions but limits agility
 * (how fast it can change directions or break).
 *
 * Setting this to the maximum value essentially disables the limit.
 *
 * Only used with smooth MPC_POS_MODE 3 and 4.
 *
 * @unit m/s^3
 * @min 0.5
 * @max 500
 * @decimal 0
 * @increment 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_JERK_MAX, 8.f);

/**
 * Max manual yaw rate for Stabilized, Altitude, Position mode
 *
 * @unit deg/s
 * @min 0
 * @max 400
 * @decimal 0
 * @increment 10
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_MAN_Y_MAX, 150.f);

/**
 * Manual yaw rate input filter time constant
 *
 * Not used in Stabilized mode
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_MAN_Y_TAU, 0.08f);

/**
 * Numerical velocity derivative low pass cutoff frequency
 *
 * @unit Hz
 * @min 0
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(SPC_VELD_LP, 5.0f);


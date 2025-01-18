/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Position/Altitude mode variant
 *
 * The supported sub-modes are:
 * Direct velocity:
 *   Sticks directly map to velocity setpoints without smoothing.
 *   Also applies to vertical direction and Altitude mode.
 *   Useful for velocity control tuning.
 * Acceleration based:
 *   Sticks map to acceleration and there's a virtual brake drag
 *
 * @value 0 Direct velocity
 * @value 4 Acceleration based
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_POS_MODE, 4);

/**
 * Maximum horizontal velocity setpoint in Position mode
 *
 * Must be smaller than MPC_XY_VEL_MAX.
 *
 * The maximum sideways and backward speed can be set differently
 * using MPC_VEL_MAN_SIDE and MPC_VEL_MAN_BACK, respectively.
 *
 * @unit m/s
 * @min 3
 * @max 20
 * @increment 1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL, 10.f);

/**
 * Maximum sideways velocity in Position mode
 *
 * If set to a negative value or larger than
 * MPC_VEL_MANUAL then MPC_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1
 * @max 20
 * @increment 1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_SIDE, -1.f);

/**
 * Maximum backward velocity in Position mode
 *
 * If set to a negative value or larger than
 * MPC_VEL_MANUAL then MPC_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1
 * @max 20
 * @increment 1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_BACK, -1.f);

/**
 * Maximum horizontal acceleration
 *
 * MPC_POS_MODE
 * 1 just deceleration
 * 4 not used, use MPC_ACC_HOR instead
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX, 5.f);

/**
 * Maximum horizontal and vertical jerk in Position/Altitude mode
 *
 * Limit the maximum jerk (acceleration change) of the vehicle.
 * A lower value leads to smoother motions but limits agility.
 *
 * Setting this to the maximum value essentially disables the limit.
 *
 * Only used with MPC_POS_MODE Acceleration based.
 *
 * @unit m/s^3
 * @min 0.5
 * @max 500
 * @decimal 0
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_JERK_MAX, 8.f);

/**
 * Deadzone for sticks in manual piloted modes
 *
 * Does not apply to manual throttle and direct attitude piloting by stick.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_DZ, 0.1f);

/**
 * Manual position control stick exponential curve sensitivity
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_MAN_EXPO, 0.6f);

/**
 * Manual control stick vertical exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_MAN_EXPO, 0.6f);

/**
 * Manual control stick yaw rotation exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_YAW_EXPO, 0.6f);

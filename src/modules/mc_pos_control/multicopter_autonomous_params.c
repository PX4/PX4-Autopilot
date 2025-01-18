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
 * Default horizontal velocity in autonomous modes
 *
 * e.g. in Missions, RTL, Goto if the waypoint does not specify differently
 *
 * @unit m/s
 * @min 3
 * @max 20
 * @decimal 0
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_CRUISE, 5.f);

/**
 * Ascent velocity in autonomous modes
 *
 * For manually controlled modes and offboard see MPC_Z_VEL_MAX_UP
 *
 * @unit m/s
 * @min 0.5
 * @max 8
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_UP, 3.f);

/**
 * Descent velocity in autonomous modes
 *
 * For manual modes and offboard, see MPC_Z_VEL_MAX_DN
 *
 * @unit m/s
 * @min 0.5
 * @max 4
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_DN, 1.5f);

/**
 * Acceleration for autonomous and for manual modes
 *
 * When piloting manually, this parameter is only used in MPC_POS_MODE Acceleration based.
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR, 3.f);

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
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_JERK_AUTO, 4.f);

/**
 * Proportional gain for horizontal trajectory position error
 *
 * @min 0.1
 * @max 1
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_TRAJ_P, 0.5f);

/**
 * Maximum horizontal error allowed by the trajectory generator
 *
 * The integration speed of the trajectory setpoint is linearly
 * reduced with the horizontal position tracking error. When the
 * error is above this parameter, the integration of the
 * trajectory is stopped to wait for the drone.
 *
 * This value can be adjusted depending on the tracking
 * capabilities of the vehicle.
 *
 * @min 0.1
 * @max 10
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_ERR_MAX, 2.f);

/**
 * Maximum yaw rate in autonomous modes
 *
 * Limits the rate of change of the yaw setpoint to avoid large
 * control output and mixer saturation.
 *
 * @unit deg/s
 * @min 5
 * @max 360
 * @decimal 0
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_YAWRAUTO_MAX, 45.f);

/**
 * Maximum yaw acceleration in autonomous modes
 *
 * Limits the acceleration of the yaw setpoint to avoid large
 * control output and mixer saturation.
 *
 * @unit deg/s^2
 * @min 5
 * @max 360
 * @decimal 0
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_YAWRAUTO_ACC, 60.f);

/**
 * Heading behavior in autonomous modes
 *
 * @min 0
 * @max 4
 * @value 0 towards waypoint
 * @value 1 towards home
 * @value 2 away from home
 * @value 3 along trajectory
 * @value 4 towards waypoint (yaw first)
 * @value 5 yaw fixed
 * @group Mission
 */
PARAM_DEFINE_INT32(MPC_YAW_MODE, 0);

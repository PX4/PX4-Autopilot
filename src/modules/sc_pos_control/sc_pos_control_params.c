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
PARAM_DEFINE_FLOAT(MPC_VEL_MAX, 12.f);

/**
 * Default velocity in autonomous modes
 *
 * e.g. in Missions, RTL, Goto if the waypoint does not specify differently
 *
 * @unit m/s
 * @min 3
 * @max 20
 * @decimal 0
 * @increment 1
 * @group Spacecraft Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_CRUISE, 5.f);

/*
    TODO:
    	(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,
		(ParamFloat<px4::params::SPC_VEL_MANUAL>)   _param_mpc_vel_manual,
		(ParamFloat<px4::params::SPC_THR_MAX>)      _param_mpc_thr_max,		
		(ParamFloat<px4::params::SPC_ACC>)      _param_mpc_acc_hor,
		(ParamFloat<px4::params::SPC_ACC_MAX>)      _param_mpc_acc_hor,
		(ParamFloat<px4::params::SPC_JERK_AUTO>)    _param_mpc_jerk_auto,
		(ParamFloat<px4::params::SPC_JERK_MAX>)     _param_mpc_jerk_max,
		(ParamFloat<px4::params::SPC_MAN_Y_MAX>)    _param_mpc_man_y_max,
		(ParamFloat<px4::params::SPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,
		(ParamFloat<px4::params::SPC_XY_VEL_ALL>)   _param_mpc_xy_vel_all,
		(ParamFloat<px4::params::SPC_Z_VEL_ALL>)    _param_mpc_z_vel_all
*/

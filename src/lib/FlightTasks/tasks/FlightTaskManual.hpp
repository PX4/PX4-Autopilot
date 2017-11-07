/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManual.hpp
 *
 * Flight task for the normal, legacy, manual position controlled flight
 * where stick inputs map basically to the velocity setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "FlightTask.hpp"
#include <mathlib/math/filter/LowPassFilter2p.hpp>

class FlightTaskManual : public FlightTask
{
public:
	FlightTaskManual(SuperBlock *parent, const char *name) :
		FlightTask(parent, name),
		_sub_manual_control_setpoint(ORB_ID(manual_control_setpoint), 0, 0, &getSubscriptions()),
		_xy_vel_man_expo(parent, "MPC_XY_MAN_EXPO", false),
		_z_vel_man_expo(parent, "MPC_Z_MAN_EXPO", false),
		_hold_dz(parent, "MPC_HOLD_DZ", false),
		_velocity_hor_manual(parent, "MPC_VEL_MANUAL", false),
		_z_vel_max_up(parent, "MPC_Z_VEL_MAX_UP", false),
		_z_vel_max_down(parent, "MPC_Z_VEL_MAX_DN", false),
		_hold_max_xy(parent, "MPC_HOLD_MAX_XY", false),
		_hold_max_z(parent, "MPC_HOLD_MAX_Z", false),
		_jerk_hor_max(parent, "MPC_JERK_MAX", false),
		_jerk_hor_min(parent, "MPC_JERK_MIN", false),
		_deceleration_hor_slow(parent, "MPC_DEC_HOR_SLOW", false),
		_acceleration_hor_max(this, "MPC_ACC_HOR_MAX", false),
		_acceleration_hor_manual(this, "MPC_ACC_HOR_MAN", false),
		_acceleration_z_max_up(this, "MPC_ACC_UP_MAX", false),
		_acceleration_z_max_down(this, "MPC_ACC_DOWN_MAX", false),
		_rc_flt_smp_rate(parent, "RC_FLT_SMP_RATE", false),
		_rc_flt_cutoff(parent, "RC_FLT_CUTOFF", false),
		_manual_direction_change_hysteresis(false),
		_filter_roll_stick(50.0f, 10.0f),
		_filter_pitch_stick(50.0f, 10.0f)
	{
		_manual_direction_change_hysteresis.set_hysteresis_time_from(false, DIRECTION_CHANGE_TIME_US);
	};
	virtual ~FlightTaskManual() {};


protected:
	matrix::Vector<float, 4> _sticks;

	float get_input_frame_yaw() { return _yaw; };

private:
	uORB::Subscription<manual_control_setpoint_s> _sub_manual_control_setpoint;

	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed */
	control::BlockParamFloat _z_vel_max_up; /**< maximal vertical velocity when flying upwards with the stick */
	control::BlockParamFloat _z_vel_max_down; /**< maximal vertical velocity when flying downwards with the stick */
	control::BlockParamFloat _hold_max_xy; /**< velocity threshold to switch into horizontal position hold */
	control::BlockParamFloat _hold_max_z; /**< velocity threshold to switch into vertical position hold */

	matrix::Vector3f _hold_position; /**< position at which the vehicle stays while the input is zero velocity */

	int _evaluate_sticks();


	/* --- Acceleration Smoothing --- */
	static constexpr uint64_t DIRECTION_CHANGE_TIME_US = 1e5; /** Time in us to switch into direction change state */

	control::BlockParamFloat _jerk_hor_max; /**< maximum jerk only applied when braking to zero */
	control::BlockParamFloat _jerk_hor_min; /**< minimum jerk only applied when braking to zero */
	control::BlockParamFloat _deceleration_hor_slow; /**< slow velocity setpoint slewrate for manual deceleration*/
	control::BlockParamFloat _acceleration_hor_max; /**< maximum velocity setpoint slewrate for auto & fast manual brake */
	control::BlockParamFloat _acceleration_hor_manual; /**< maximum velocity setpoint slewrate for manual acceleration */
	control::BlockParamFloat _acceleration_z_max_up; /**< max acceleration up */
	control::BlockParamFloat _acceleration_z_max_down; /**< max acceleration down */
	control::BlockParamFloat _rc_flt_smp_rate; /**< sample rate for stick lowpass filter */
	control::BlockParamFloat _rc_flt_cutoff; /**< cutoff frequency for stick lowpass filter */

	matrix::Vector2f _stick_input_xy_prev;
	matrix::Vector3f _vel_sp_prev; /* velocity setpoint of last loop to calculate setpoint slewrate - acceleration */
	enum stick_user_intention {
		brake,
		direction_change,
		acceleration,
		deceleration
	};
	stick_user_intention _intention_xy_prev = brake; /**< user intension derived from the xy stick input */
	stick_user_intention _user_intention_z = brake; /**< user intension derived from the z stick input */
	float _manual_jerk_limit_xy = 1.f; /**< jerk limit in manual mode dependent on stick input */
	float _manual_jerk_limit_z = 1.f; /**< jerk limit in manual mode in z */
	systemlib::Hysteresis _manual_direction_change_hysteresis;
	math::LowPassFilter2p _filter_roll_stick;
	math::LowPassFilter2p _filter_pitch_stick;

	void vel_sp_slewrate(matrix::Vector3f &vel_sp, const matrix::Vector2f &stick_xy, const float &stick_z);

	void reset_slewrate_xy();

	float get_acceleration_xy(const matrix::Vector2f &stick_xy);

	float get_acceleration_z(const float &stick_z);

};

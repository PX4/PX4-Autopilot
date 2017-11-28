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
#include <systemlib/hysteresis/hysteresis.h>
#include <uORB/topics/manual_control_setpoint.h>

class FlightTaskManual : public FlightTask
{
public:
	FlightTaskManual(control::SuperBlock *parent, const char *name);

	virtual ~FlightTaskManual() = default;

	bool initializeSubscriptions(SubscriptionArray &subscription_array) override;

	bool updateInitialize() override;

	bool update() override;

protected:
	matrix::Vector<float, 4> _sticks;
	bool _evaluateSticks();

	float get_input_frame_yaw() { return _yaw; }

private:
	uORB::Subscription<manual_control_setpoint_s> *_sub_manual_control_setpoint{nullptr};

	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed */
	control::BlockParamFloat _z_vel_max_up; /**< maximal vertical velocity when flying upwards with the stick */
	control::BlockParamFloat _z_vel_max_down; /**< maximal vertical velocity when flying downwards with the stick */
	control::BlockParamFloat _hold_max_xy; /**< velocity threshold to switch into horizontal position hold */
	control::BlockParamFloat _hold_max_z; /**< velocity threshold to switch into vertical position hold */

	matrix::Vector3f _hold_position; /**< position at which the vehicle stays while the input is zero velocity */

	bool _evaluate_sticks();


	/* --- Acceleration Smoothing --- */
	static constexpr uint64_t DIRECTION_CHANGE_TIME_US = 1e5; /**< Time in us to switch into direction change state */

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
	matrix::Vector3f _vel_sp_prev; /**< velocity setpoint of last loop to calculate setpoint slewrate - acceleration */
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

	void vel_sp_slewrate(const matrix::Vector2f &stick_xy, const float &stick_z, matrix::Vector3f &vel_sp);

	void reset_slewrate_xy();

	float get_acceleration_xy(const matrix::Vector2f &stick_xy);

	float get_acceleration_z(const float &stick_z);

};

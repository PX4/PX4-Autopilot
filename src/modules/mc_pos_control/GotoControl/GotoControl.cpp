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
 * @file GotoControl.cpp
 */

#include "GotoControl.hpp"

#include <drivers/drv_hrt.h>
#include <float.h>
#include <lib/mathlib/mathlib.h>

using namespace time_literals;

bool GotoControl::checkForSetpoint(const hrt_abstime &now, const bool enabled)
{
	_goto_setpoint_sub.update();
	const bool timestamp_initialized = _goto_setpoint_sub.get().timestamp != 0;
	const bool no_timeout = now < (_goto_setpoint_sub.get().timestamp + 500_ms);
	const bool need_to_run = timestamp_initialized && no_timeout && enabled;

	if (!need_to_run) {
		_is_initialized = false;
	}

	return need_to_run;
}

void GotoControl::update(const float dt, const matrix::Vector3f &position, const float heading)
{
	if (!_is_initialized) {
		resetPositionSmoother(position);
		resetHeadingSmoother(heading);
		_is_initialized = true;
	}

	const goto_setpoint_s &goto_setpoint = _goto_setpoint_sub.get();

	const Vector3f position_setpoint(_goto_setpoint_sub.get().position);

	if (!position_setpoint.isAllFinite()) {
		// TODO: error messaging
		_need_smoother_reset = true;
		return;
	}

	if (!position.isAllFinite()) {
		// TODO: error messaging
		_need_smoother_reset = true;
		return;
	}

	if (_need_smoother_reset) {
		resetPositionSmoother(position);
	}

	setPositionSmootherLimits(goto_setpoint);

	const Vector3f feedforward_velocity{};
	const bool force_zero_velocity_setpoint = false;
	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	_position_smoothing.generateSetpoints(position, position_setpoint, feedforward_velocity, dt,
					      force_zero_velocity_setpoint, out_setpoints);

	trajectory_setpoint_s trajectory_setpoint{};
	out_setpoints.position.copyTo(trajectory_setpoint.position);
	out_setpoints.velocity.copyTo(trajectory_setpoint.velocity);
	out_setpoints.acceleration.copyTo(trajectory_setpoint.acceleration);
	out_setpoints.jerk.copyTo(trajectory_setpoint.jerk);

	if (goto_setpoint.flag_control_heading && PX4_ISFINITE(goto_setpoint.heading) && PX4_ISFINITE(heading)) {
		if (!_controlling_heading || _need_smoother_reset) {
			resetHeadingSmoother(heading);
		}

		setHeadingSmootherLimits(goto_setpoint);
		_heading_smoothing.update(goto_setpoint.heading, dt);

		trajectory_setpoint.yaw = _heading_smoothing.getSmoothedHeading();
		trajectory_setpoint.yawspeed = _heading_smoothing.getSmoothedHeadingRate();

		_controlling_heading = true;

	} else {
		trajectory_setpoint.yaw = NAN;
		trajectory_setpoint.yawspeed = NAN;

		_controlling_heading = false;
	}

	_need_smoother_reset = false;

	trajectory_setpoint.timestamp = goto_setpoint.timestamp;
	_trajectory_setpoint_pub.publish(trajectory_setpoint);

	vehicle_constraints_s vehicle_constraints{
		.timestamp = goto_setpoint.timestamp,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false
	};
	_vehicle_constraints_pub.publish(vehicle_constraints);
}

void GotoControl::resetPositionSmoother(const matrix::Vector3f &position)
{
	if (!position.isAllFinite()) {
		// TODO: error messaging
		_need_smoother_reset = true;
		return;
	}

	const Vector3f initial_acceleration{};
	const Vector3f initial_velocity{};
	_position_smoothing.reset(initial_acceleration, initial_velocity, position);

	_need_smoother_reset = false;
}

void GotoControl::resetHeadingSmoother(const float heading)
{
	if (!PX4_ISFINITE(heading)) {
		// TODO: error messaging
		_controlling_heading = false;
		return;
	}

	const float initial_heading_rate{0.f};
	_heading_smoothing.reset(heading, initial_heading_rate);
}

void GotoControl::setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	// Horizontal constraints
	float max_horizontal_speed = _param_mpc_xy_cruise;
	float max_horizontal_accel = _param_mpc_acc_hor;

	if (goto_setpoint.flag_set_max_horizontal_speed
	    && PX4_ISFINITE(goto_setpoint.max_horizontal_speed)) {
		max_horizontal_speed = math::constrain(goto_setpoint.max_horizontal_speed, 0.f,
						       _param_mpc_xy_cruise);

		// linearly scale horizontal acceleration limit with horizontal speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (!_position_smoothing.getCurrentVelocityXY().longerThan(max_horizontal_speed)) {
			const float speed_scale = max_horizontal_speed / _param_mpc_xy_cruise;
			max_horizontal_accel = math::constrain(_param_mpc_acc_hor * speed_scale, 0.f, _param_mpc_acc_hor);
		}
	}

	_position_smoothing.setCruiseSpeed(max_horizontal_speed);
	_position_smoothing.setMaxAccelerationXY(max_horizontal_accel);

	// Vertical constraints
	float vehicle_max_vertical_speed = _param_mpc_z_v_auto_dn;
	float vehicle_max_vertical_accel = _param_mpc_acc_down_max;

	if (goto_setpoint.position[2] < _position_smoothing.getCurrentPositionZ()) { // goto higher -> more negative
		vehicle_max_vertical_speed = _param_mpc_z_v_auto_up;
		vehicle_max_vertical_accel = _param_mpc_acc_up_max;
	}

	float max_vertical_speed = vehicle_max_vertical_speed;
	float max_vertical_accel = vehicle_max_vertical_accel;

	if (goto_setpoint.flag_set_max_vertical_speed && PX4_ISFINITE(goto_setpoint.max_vertical_speed)) {
		max_vertical_speed = math::constrain(goto_setpoint.max_vertical_speed, 0.f, vehicle_max_vertical_speed);

		// linearly scale vertical acceleration limit with vertical speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (fabsf(_position_smoothing.getCurrentVelocityZ()) <= max_vertical_speed) {
			const float speed_scale = max_vertical_speed / vehicle_max_vertical_speed;
			max_vertical_accel = math::constrain(vehicle_max_vertical_accel * speed_scale, 0.f, vehicle_max_vertical_accel);
		}
	}

	_position_smoothing.setMaxVelocityZ(max_vertical_speed);
	_position_smoothing.setMaxAccelerationZ(max_vertical_accel);
}

void GotoControl::setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	float max_heading_rate = _param_mpc_yawrauto_max;
	float max_heading_accel = _param_mpc_yawrauto_acc;

	if (goto_setpoint.flag_set_max_heading_rate && PX4_ISFINITE(goto_setpoint.max_heading_rate)) {
		max_heading_rate = math::constrain(goto_setpoint.max_heading_rate, 0.f, _param_mpc_yawrauto_max);

		// linearly scale heading acceleration limit with heading rate limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (fabsf(_heading_smoothing.getSmoothedHeadingRate()) <= max_heading_rate) {
			const float rate_scale = max_heading_rate / _param_mpc_yawrauto_max;
			max_heading_accel = math::constrain(_param_mpc_yawrauto_acc * rate_scale, 0.f, _param_mpc_yawrauto_acc);
		}
	}

	_heading_smoothing.setMaxHeadingRate(max_heading_rate);
	_heading_smoothing.setMaxHeadingAccel(max_heading_accel);
}

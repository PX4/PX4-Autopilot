/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#include "PositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>

void GotoControl::update(const float dt, const matrix::Vector3f &position, const float heading,
			 const goto_setpoint_s &goto_setpoint, trajectory_setpoint_s &trajectory_setpoint)
{
	trajectory_setpoint = PositionControl::empty_trajectory_setpoint;

	const Vector3f position_setpoint(goto_setpoint.position);

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
	_position_smoother.generateSetpoints(position, position_setpoint, feedforward_velocity, dt,
					     force_zero_velocity_setpoint, out_setpoints);

	_position_smoother.getCurrentPosition().copyTo(trajectory_setpoint.position);
	_position_smoother.getCurrentVelocity().copyTo(trajectory_setpoint.velocity);
	_position_smoother.getCurrentAcceleration().copyTo(trajectory_setpoint.acceleration);
	out_setpoints.jerk.copyTo(trajectory_setpoint.jerk);

	if (goto_setpoint.flag_control_heading && PX4_ISFINITE(goto_setpoint.heading) && PX4_ISFINITE(heading)) {
		if (!_controlling_heading || _need_smoother_reset) {
			resetHeadingSmoother(heading);
		}

		setHeadingSmootherLimits(goto_setpoint);
		_heading_smoother.update(goto_setpoint.heading, dt);

		trajectory_setpoint.yaw = _heading_smoother.getSmoothedHeading();
		trajectory_setpoint.yawspeed = _heading_smoother.getSmoothedHeadingRate();

		_controlling_heading = true;

	} else {
		// TODO: error messaging for non-finite headings

		trajectory_setpoint.yaw = NAN;
		trajectory_setpoint.yawspeed = NAN;

		_controlling_heading = false;
	}

	trajectory_setpoint.timestamp = goto_setpoint.timestamp;

	_need_smoother_reset = false;
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
	_position_smoother.reset(initial_acceleration, initial_velocity, position);

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
	_heading_smoother.reset(heading, initial_heading_rate);
}

void GotoControl::setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	// constrain horizontal velocity
	float max_horizontal_speed = _vehicle_constraints.max_horizontal_speed;
	float max_horizontal_accel = _vehicle_constraints.max_horizontal_accel;

	if (goto_setpoint.flag_set_max_horizontal_speed
	    && PX4_ISFINITE(goto_setpoint.max_horizontal_speed)) {
		max_horizontal_speed = math::constrain(goto_setpoint.max_horizontal_speed, 0.f,
						       _vehicle_constraints.max_horizontal_speed);

		// linearly scale horizontal acceleration limit with horizontal speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (_position_smoother.getCurrentVelocityXY().norm() <= max_horizontal_speed) {
			const float speed_scale = max_horizontal_speed / _vehicle_constraints.max_horizontal_speed;
			max_horizontal_accel = math::constrain(_vehicle_constraints.max_horizontal_accel * speed_scale,
							       VelocitySmoothing::kMinAccel,
							       _vehicle_constraints.max_horizontal_accel);
		}
	}

	_position_smoother.setCruiseSpeed(max_horizontal_speed);
	_position_smoother.setMaxVelocityXY(max_horizontal_speed);
	_position_smoother.setMaxAccelerationXY(max_horizontal_accel);

	// constrain vertical velocity
	const bool pos_setpoint_is_below_smooth_pos = goto_setpoint.position[2] - _position_smoother.getCurrentPositionZ() >
			0.f;
	const float vehicle_max_vertical_speed = (pos_setpoint_is_below_smooth_pos) ? _vehicle_constraints.max_down_speed :
			_vehicle_constraints.max_up_speed;
	const float vehicle_max_vertical_accel = (pos_setpoint_is_below_smooth_pos) ? _vehicle_constraints.max_down_accel :
			_vehicle_constraints.max_up_accel;

	float max_vertical_speed = vehicle_max_vertical_speed;
	float max_vertical_accel = vehicle_max_vertical_accel;

	if (goto_setpoint.flag_set_max_vertical_speed && PX4_ISFINITE(goto_setpoint.max_vertical_speed)) {

		max_vertical_speed = math::constrain(goto_setpoint.max_vertical_speed, 0.f, vehicle_max_vertical_speed);

		// linearly scale vertical acceleration limit with vertical speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (fabsf(_position_smoother.getCurrentVelocityZ()) <= max_vertical_speed) {
			const float speed_scale = max_vertical_speed / vehicle_max_vertical_speed;
			max_vertical_accel = math::constrain(vehicle_max_vertical_accel * speed_scale, VelocitySmoothing::kMinAccel,
							     vehicle_max_vertical_accel);
		}
	}

	_position_smoother.setMaxVelocityZ(max_vertical_speed);
	_position_smoother.setMaxAccelerationZ(max_vertical_accel);

	_position_smoother.setMaxJerkXY(_vehicle_constraints.max_jerk);
	_position_smoother.setMaxJerkZ(_vehicle_constraints.max_jerk);
}

void GotoControl::setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	float max_heading_rate =  _vehicle_constraints.max_heading_rate;
	float max_heading_accel = _vehicle_constraints.max_heading_accel;

	if (goto_setpoint.flag_set_max_heading_rate && PX4_ISFINITE(goto_setpoint.max_heading_rate)) {
		max_heading_rate = math::constrain(goto_setpoint.max_heading_rate, HeadingSmoother::kMinHeadingRate,
						   _vehicle_constraints.max_heading_rate);

		// linearly scale heading acceleration limit with heading rate limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (fabsf(_heading_smoother.getSmoothedHeadingRate()) <= max_heading_rate) {
			const float rate_scale = max_heading_rate / _vehicle_constraints.max_heading_rate;
			max_heading_accel = math::constrain(_vehicle_constraints.max_heading_accel * rate_scale,
							    HeadingSmoother::kMinHeadingAccel, _vehicle_constraints.max_heading_accel);
		}
	}

	_heading_smoother.setMaxHeadingRate(max_heading_rate);
	_heading_smoother.setMaxHeadingAccel(max_heading_accel);
}

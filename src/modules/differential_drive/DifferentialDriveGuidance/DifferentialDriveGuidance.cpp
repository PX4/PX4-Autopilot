/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "DifferentialDriveGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

DifferentialDriveGuidance::DifferentialDriveGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();

	pid_init(&_heading_p_controller, PID_MODE_DERIVATIV_NONE, 0.001f);
}

void DifferentialDriveGuidance::computeGuidance(float yaw, float angular_velocity, float dt)
{
	if (_position_setpoint_triplet_sub.updated()) {
		_position_setpoint_triplet_sub.copy(&_position_setpoint_triplet);
	}

	if (_vehicle_global_position_sub.updated()) {
		_vehicle_global_position_sub.copy(&_vehicle_global_position);
	}

	matrix::Vector2d global_position(_vehicle_global_position.lat, _vehicle_global_position.lon);
	matrix::Vector2d current_waypoint(_position_setpoint_triplet.current.lat, _position_setpoint_triplet.current.lon);
	matrix::Vector2d next_waypoint(_position_setpoint_triplet.next.lat, _position_setpoint_triplet.next.lon);

	const float distance_to_next_wp = get_distance_to_next_waypoint(global_position(0), global_position(1),
					  current_waypoint(0),
					  current_waypoint(1));

	float desired_heading = get_bearing_to_next_waypoint(global_position(0), global_position(1), current_waypoint(0),
				current_waypoint(1));
	float heading_error = matrix::wrap_pi(desired_heading - yaw);

	if (_current_waypoint != current_waypoint) {
		_currentState = GuidanceState::TURNING;
	}

	// Make rover stop when it arrives at the last waypoint instead of loitering and driving around weirdly.
	if ((current_waypoint == next_waypoint) && distance_to_next_wp <= _param_nav_acc_rad.get()) {
		_currentState = GuidanceState::GOAL_REACHED;
	}

	float desired_speed = 0.f;

	switch (_currentState) {
	case GuidanceState::TURNING:
		desired_speed = 0.f;

		if (fabsf(heading_error) < 0.05f) {
			_currentState = GuidanceState::DRIVING;
		}

		break;

	case GuidanceState::DRIVING: {
			const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(_param_rdd_max_jerk.get(),
						   _param_rdd_max_accel.get(), distance_to_next_wp, 0.0f);
			_forwards_velocity_smoothing.updateDurations(max_velocity);
			_forwards_velocity_smoothing.updateTraj(dt);
			desired_speed = math::interpolate<float>(abs(heading_error), 0.1f, 0.8f,
					_forwards_velocity_smoothing.getCurrentVelocity(), 0.0f);
			break;
		}

	case GuidanceState::GOAL_REACHED:
		// temporary till I find a better way to stop the vehicle
		desired_speed = 0.f;
		heading_error = 0.f;
		angular_velocity = 0.f;
		_desired_angular_velocity = 0.f;
		break;
	}

	// Compute the desired angular velocity relative to the heading error, to steer the vehicle towards the next waypoint.
	float angular_velocity_pid = pid_calculate(&_heading_p_controller, heading_error, angular_velocity, 0,
				     dt) + heading_error;

	differential_drive_setpoint_s output{};
	output.speed = math::constrain(desired_speed, -_max_speed, _max_speed);
	output.yaw_rate = math::constrain(angular_velocity_pid, -_max_angular_velocity, _max_angular_velocity);
	output.closed_loop_speed_control = output.closed_loop_yaw_rate_control = true;
	output.timestamp = hrt_absolute_time();

	_differential_drive_setpoint_pub.publish(output);

	_current_waypoint = current_waypoint;
}

void DifferentialDriveGuidance::updateParams()
{
	ModuleParams::updateParams();

	pid_set_parameters(&_heading_p_controller,
			   _param_rdd_p_gain_heading.get(),  // Proportional gain
			   0,  // Integral gain
			   0,  // Derivative gain
			   0,  // Integral limit
			   200);  // Output limit

	_forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());
	_forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());
	_forwards_velocity_smoothing.setMaxVel(_max_speed);
}

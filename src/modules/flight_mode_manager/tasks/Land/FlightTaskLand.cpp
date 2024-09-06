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
 * @file FlightTaskLand.cpp
 */

#include "FlightTaskLand.hpp"

bool
FlightTaskLand::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	Vector3f vel_prev{last_setpoint.velocity};
	Vector3f pos_prev{last_setpoint.position};
	Vector3f accel_prev{last_setpoint.acceleration};
	float yaw_prev = last_setpoint.yawspeed;

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current position
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// If no acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }

		// If the yaw setpoint is unknown, set to the current yawq
		if (!PX4_ISFINITE(yaw_prev)) { yaw_prev = _yaw; }
	}

	_position_smoothing.reset(pos_prev, vel_prev, accel_prev);
	_yaw_setpoint = yaw_prev;
	// Initialize the Landing locations and parameters

	// calculate where to land based on the current velocity and acceleration constraints
	// set this as the target location for position smoothing
	_updateTrajConstraints();


	PX4_INFO("FlightTaskMyTask activate was called! ret: %d", ret); // report if activation was successful
	return ret;
}

bool
FlightTaskLand::update()
{

	// Check if we have a velocity
	_landing = _velocity.xy().norm() > 0.1f
		   || _velocity(2) < _param_mpc_z_v_auto_dn.get(); // not sure about the last parts, check!

	if (!_landing) { // smaller velocity as positive altitude is negative distance
		PX4_WARN("Landing: Vehicle is moving, slow down first");


	} else {
		CalculateLandingLocation();
		PositionSmoothing::PositionSmoothingSetpoints smoothed_setpoints;
		_position_smoothing.generateSetpoints(
			_position,
			_initial_land_position,
		{0, 0, 0}, _deltatime,
		false,
		smoothed_setpoints
		);

	}

	// update the yaw setpoint

	PX4_INFO("FlightTaskMyTask update was called!"); // report update
	return true;
}

void
FlightTaskLand::CalculateLandingLocation()
{
	// Calculate the 3D point where we until where we can slow down smoothly and then land based on the current velocities and system constraints on jerk and acceleration.

	float delay_scale = 0.2f; // delay scale factor
	const float braking_dist_xy = math::trajectory::computeBrakingDistanceFromVelocity(_velocity.xy().norm(),
				      _param_mpc_jerk_auto.get(), _param_mpc_acc_hor.get(), delay_scale * _param_mpc_jerk_auto.get());
	float braking_dist_z = 0.0f;

	if (_velocity(2) < 0.0f) {
		PX4_WARN("Moving upwards");
		braking_dist_z = math::trajectory::computeBrakingDistanceFromVelocity(_velocity(2),
				 _param_mpc_jerk_auto.get(), _param_mpc_acc_down_max.get(), delay_scale * _param_mpc_jerk_auto.get());

	} else {
		PX4_WARN("Moving downwards");
		braking_dist_z = math::trajectory::computeBrakingDistanceFromVelocity(_velocity(2),
				 _param_mpc_jerk_auto.get(), _param_mpc_acc_up_max.get(), delay_scale * _param_mpc_jerk_auto.get());
	}

	const Vector3f braking_dir = _velocity.unit_or_zero();
	const Vector3f braking_dist = {braking_dist_xy, braking_dist_xy, braking_dist_z};

	_initial_land_position = _position + braking_dir * braking_dist;
	PX4_INFO("FlightTaskMyTask CalculateLandingLocation was called!"); // report calculation
}

void
FlightTaskLand::_updateTrajConstraints()
{
	// update params of the position smoothing
	_position_smoothing.setCruiseSpeed(_param_mpc_xy_vel_max.get());
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setTargetAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());

	// Update the constraints of the trajectories
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get());
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	_position_smoothing.setMaxJerk(_param_mpc_jerk_auto.get());

	// set the constraints for the vertical direction
	// if moving up, acceleration constraint is always in deceleration direction, eg opposite to the velocity
	if (_velocity(2) < 0.0f && !_landing) {
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_up.get());

	} else if (!_landing) {
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_up_max.get());
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());

	} else {
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
	}

	// should the constraints be different when switching from an auto mode compared to an manual mode?
}

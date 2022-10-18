/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "PositionSmoothing.hpp"
#include "TrajectoryConstraints.hpp"
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <matrix/matrix/helper_functions.hpp>


void PositionSmoothing::_generateSetpoints(
	const Vector3f &position,
	const Vector3f(&waypoints)[3],
	bool is_single_waypoint,
	const Vector3f &feedforward_velocity,
	float delta_time,
	bool force_zero_velocity_setpoint,
	PositionSmoothingSetpoints &out_setpoints)
{
	Vector3f velocity_setpoint{0.f, 0.f, 0.f};

	if (!force_zero_velocity_setpoint) {
		velocity_setpoint = _generateVelocitySetpoint(position, waypoints, is_single_waypoint, feedforward_velocity);
	}

	out_setpoints.unsmoothed_velocity = velocity_setpoint;

	_generateTrajectory(
		position,
		velocity_setpoint,
		delta_time,
		out_setpoints
	);
}


bool PositionSmoothing::_isTurning(const Vector3f &target) const
{
	const Vector2f vel_traj(_trajectory[0].getCurrentVelocity(),
				_trajectory[1].getCurrentVelocity());
	const Vector2f pos_traj(_trajectory[0].getCurrentPosition(),
				_trajectory[1].getCurrentPosition());
	const Vector2f target_xy(target);
	const Vector2f u_vel_traj = vel_traj.unit_or_zero();
	const Vector2f pos_to_target = Vector2f(target_xy - pos_traj);
	const float cos_align = u_vel_traj * pos_to_target.unit_or_zero();

	// The vehicle is turning if the angle between the velocity vector
	// and the direction to the target is greater than 10 degrees, the
	// velocity is large enough and the drone isn't in the acceptance
	// radius of the last WP.
	return (vel_traj.longerThan(0.2f)
		&& cos_align < 0.98f
		&& pos_to_target.longerThan(_target_acceptance_radius));
}

float PositionSmoothing::_getMaxXYSpeed(const Vector3f(&waypoints)[3]) const
{
	Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
			  _trajectory[1].getCurrentPosition(),
			  _trajectory[2].getCurrentPosition());

	math::trajectory::VehicleDynamicLimits config;
	config.z_accept_rad = _vertical_acceptance_radius;
	config.xy_accept_rad = _target_acceptance_radius;
	config.max_acc_xy = _trajectory[0].getMaxAccel();
	config.max_jerk = _trajectory[0].getMaxJerk();
	config.max_speed_xy = _cruise_speed;
	config.max_acc_xy_radius_scale = _horizontal_trajectory_gain;

	// constrain velocity to go to the position setpoint first if the position setpoint has been modified by an external source
	// (eg. Obstacle Avoidance)

	Vector3f pos_to_waypoints[3] = {pos_traj, waypoints[1], waypoints[2]};

	return math::trajectory::computeXYSpeedFromWaypoints<3>(pos_to_waypoints, config);
}

float PositionSmoothing::_getMaxZSpeed(const Vector3f(&waypoints)[3]) const
{

	const auto &target = waypoints[1];

	Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
			  _trajectory[1].getCurrentPosition(),
			  _trajectory[2].getCurrentPosition());

	const float distance_start_target = fabs(target(2) - pos_traj(2));
	const float arrival_z_speed = 0.f;

	float max_speed = math::min(_trajectory[2].getMaxVel(), math::trajectory::computeMaxSpeedFromDistance(
					    _trajectory[2].getMaxJerk(), _trajectory[2].getMaxAccel(),
					    distance_start_target, arrival_z_speed));

	return max_speed;
}

const Vector3f PositionSmoothing::_getCrossingPoint(const Vector3f &position, const Vector3f(&waypoints)[3]) const
{
	const auto &target = waypoints[1];

	if (!_isTurning(target)) {
		return target;
	}

	// Get the crossing point using L1-style guidance
	auto l1_point = _getL1Point(position, waypoints);
	return {l1_point(0), l1_point(1), target(2)};
}

const Vector2f PositionSmoothing::_getL1Point(const Vector3f &position, const Vector3f(&waypoints)[3]) const
{
	const Vector2f pos_traj(_trajectory[0].getCurrentPosition(),
				_trajectory[1].getCurrentPosition());
	const Vector2f u_prev_to_target = Vector2f(waypoints[1] - waypoints[0]).unit_or_zero();
	const Vector2f prev_to_pos(pos_traj - Vector2f(waypoints[0]));
	const Vector2f prev_to_closest(u_prev_to_target * (prev_to_pos * u_prev_to_target));
	const Vector2f closest_pt = Vector2f(waypoints[0]) + prev_to_closest;

	// Compute along-track error using L1 distance and cross-track error
	const float crosstrack_error = Vector2f(closest_pt - pos_traj).length();

	const float l1 = math::max(_target_acceptance_radius, 5.f);
	float alongtrack_error = 0.f;

	// Protect against sqrt of a negative number
	if (l1 > crosstrack_error) {
		alongtrack_error = sqrtf(l1 * l1 - crosstrack_error * crosstrack_error);
	}

	// Position of the point on the line where L1 intersect the line between the two waypoints
	const Vector2f l1_point = closest_pt + alongtrack_error * u_prev_to_target;

	return l1_point;
}

const Vector3f PositionSmoothing::_generateVelocitySetpoint(const Vector3f &position, const Vector3f(&waypoints)[3],
		bool is_single_waypoint,
		const Vector3f &feedforward_velocity_setpoint)
{
	// Interface: A valid position setpoint generates a velocity target using conservative motion constraints.
	// If a velocity is specified, that is used as a feedforward to track the position setpoint
	// (ie. it assumes the position setpoint is moving at the specified velocity)
	// If the position setpoints are set to NAN, the values in the velocity setpoints are used as velocity targets: nothing to do here.
	const Vector3f &target = waypoints[1];
	const bool xy_target_valid = Vector2f(target).isAllFinite();
	const bool z_target_valid = PX4_ISFINITE(target(2));

	Vector3f velocity_setpoint = feedforward_velocity_setpoint;

	if (xy_target_valid && z_target_valid) {
		// Use 3D position setpoint to generate a 3D velocity setpoint
		Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
				  _trajectory[1].getCurrentPosition(),
				  _trajectory[2].getCurrentPosition());
		const Vector3f crossing_point = is_single_waypoint ? target : _getCrossingPoint(position, waypoints);
		const Vector3f u_pos_traj_to_dest{(crossing_point - pos_traj).unit_or_zero()};

		float xy_speed = _getMaxXYSpeed(waypoints);
		const float z_speed = _getMaxZSpeed(waypoints);

		if (!is_single_waypoint && _isTurning(target)) {
			// Limit speed during a turn
			xy_speed = math::min(_max_speed_previous, xy_speed);

		} else {
			_max_speed_previous = xy_speed;
		}

		Vector3f vel_sp_constrained = u_pos_traj_to_dest * sqrtf(xy_speed * xy_speed + z_speed * z_speed);
		math::trajectory::clampToXYNorm(vel_sp_constrained, xy_speed, 0.5f);
		math::trajectory::clampToZNorm(vel_sp_constrained, z_speed, 0.5f);

		for (int i = 0; i < 3; i++) {
			// If available, use the existing velocity as a feedforward, otherwise replace it
			if (PX4_ISFINITE(velocity_setpoint(i))) {
				velocity_setpoint(i) += vel_sp_constrained(i);

			} else {
				velocity_setpoint(i) = vel_sp_constrained(i);
			}
		}
	}

	else if (xy_target_valid) {
		// Use 2D position setpoint to generate a 2D velocity setpoint

		// Get various path specific vectors
		Vector2f pos_traj(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
		Vector2f crossing_point = is_single_waypoint ? Vector2f(target) : Vector2f(_getCrossingPoint(position, waypoints));
		Vector2f pos_traj_to_dest_xy = crossing_point - pos_traj;
		Vector2f u_pos_traj_to_dest_xy(pos_traj_to_dest_xy.unit_or_zero());

		float xy_speed = _getMaxXYSpeed(waypoints);

		if (_isTurning(target)) {
			// Lock speed during turn
			xy_speed = math::min(_max_speed_previous, xy_speed);

		} else {
			_max_speed_previous = xy_speed;
		}

		Vector2f vel_sp_constrained_xy = u_pos_traj_to_dest_xy * xy_speed;

		for (int i = 0; i < 2; i++) {
			// If available, use the existing velocity as a feedforward, otherwise replace it
			if (PX4_ISFINITE(velocity_setpoint(i))) {
				velocity_setpoint(i) += vel_sp_constrained_xy(i);

			} else {
				velocity_setpoint(i) = vel_sp_constrained_xy(i);
			}
		}
	}

	else if (z_target_valid) {
		// Use Z position setpoint to generate a Z velocity setpoint

		const float z_dir = matrix::sign(target(2) - _trajectory[2].getCurrentPosition());
		const float vel_sp_z = z_dir * _getMaxZSpeed(waypoints);

		// If available, use the existing velocity as a feedforward, otherwise replace it
		if (PX4_ISFINITE(velocity_setpoint(2))) {
			velocity_setpoint(2) += vel_sp_z;

		} else {
			velocity_setpoint(2) = vel_sp_z;
		}
	}

	return velocity_setpoint;
}


void PositionSmoothing::_generateTrajectory(
	const Vector3f &position,
	const Vector3f &velocity_setpoint,
	float delta_time,
	PositionSmoothingSetpoints &out_setpoints)
{
	if (!velocity_setpoint.isAllFinite()) {
		return;
	}

	/* Slow down the trajectory by decreasing the integration time based on the position error.
	 * This is only performed when the drone is behind the trajectory
	 */
	Vector2f position_trajectory_xy(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
	Vector2f position_xy(position);
	Vector2f vel_traj_xy(_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity());
	Vector2f drone_to_trajectory_xy(position_trajectory_xy - position_xy);
	float position_error = drone_to_trajectory_xy.length();

	float time_stretch = 1.f - math::constrain(position_error / _max_allowed_horizontal_error, 0.f, 1.f);

	// Don't stretch time if the drone is ahead of the position setpoint
	if (drone_to_trajectory_xy.dot(vel_traj_xy) < 0.f) {
		time_stretch = 1.f;
	}

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].updateTraj(delta_time, time_stretch);
		out_setpoints.jerk(i) = _trajectory[i].getCurrentJerk();
		out_setpoints.acceleration(i) = _trajectory[i].getCurrentAcceleration();
		out_setpoints.velocity(i) = _trajectory[i].getCurrentVelocity();
		out_setpoints.position(i) = _trajectory[i].getCurrentPosition();
	}

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].updateDurations(velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(_trajectory, 3);
}

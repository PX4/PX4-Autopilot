/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightAutoLine.cpp
 */

#include "FlightTaskAutoLineSmoothVel.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

namespace
{


struct vehicle_dynamic_limits {
	float z_accept_rad;
	float xy_accept_rad;

	float max_acc_xy;
	float max_jerk;

	float max_speed_xy;

	// TODO: remove this
	float max_acc_xy_radius_scale;
};

/*
 * Compute the maximum allowed speed at the waypoint assuming that we want to
 * connect the two lines (current-target and target-next)
 * with a tangent circle with constant speed and desired centripetal acceleration: a_centripetal = speed^2 / radius
 * The circle should in theory start and end at the intersection of the lines and the waypoint's acceptance radius.
 * This is not exactly true in reality since Navigator switches the waypoint so we have to take in account that
 * the real acceptance radius is smaller.
 *
 */
inline float computeCurrentXYSpeedFromWaypoints(const Vector3f &start_position, const Vector3f &target,
		const Vector3f &next_target, float final_speed, const vehicle_dynamic_limits &config)
{
	float speed_at_target = 0.0f;
	const float distance_target_next = (target - next_target).xy().norm();

	const bool target_next_different = distance_target_next  > 0.001f;
	const bool waypoint_overlap = (target - next_target).xy().norm() < config.xy_accept_rad;
	const bool has_reached_altitude = fabsf(target(2) - start_position(2)) < config.z_accept_rad;
	const bool altitude_stays_same = fabsf(next_target(2) - target(2)) < config.z_accept_rad;

	if (target_next_different &&
	    !waypoint_overlap &&
	    has_reached_altitude &&
	    altitude_stays_same
	   ) {
		const float max_speed_current_next = math::trajectory::computeMaxSpeedFromDistance(config.max_jerk,
						     config.max_acc_xy,
						     distance_target_next,
						     final_speed);
		const float alpha = acosf(Vector2f((target - start_position).xy()).unit_or_zero().dot(
						  Vector2f((target - next_target).xy()).unit_or_zero()));
		float accel_tmp = config.max_acc_xy_radius_scale * config.max_acc_xy;
		float max_speed_in_turn = math::trajectory::computeMaxSpeedInWaypoint(alpha,
					  accel_tmp,
					  config.xy_accept_rad);
		speed_at_target = math::min(math::min(max_speed_in_turn,
						      max_speed_current_next),
					    config.max_speed_xy);
	}

	return math::min(math::trajectory::computeMaxSpeedFromDistance(config.max_jerk,
			 config.max_acc_xy,
			 (start_position - target).xy().norm(),
			 speed_at_target), config.max_speed_xy);
}

}

bool FlightTaskAutoLineSmoothVel::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	bool ret = FlightTaskAutoMapper2::activate(last_setpoint);

	checkSetpoints(last_setpoint);
	const Vector3f vel_prev(last_setpoint.vx, last_setpoint.vy, last_setpoint.vz);
	const Vector3f pos_prev(last_setpoint.x, last_setpoint.y, last_setpoint.z);

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].reset(last_setpoint.acceleration[i], vel_prev(i), pos_prev(i));
	}

	_yaw_sp_prev = last_setpoint.yaw;
	_updateTrajConstraints();

	return ret;
}

void FlightTaskAutoLineSmoothVel::reActivate()
{
	// On ground, reset acceleration and velocity to zero
	for (int i = 0; i < 2; ++i) {
		_trajectory[i].reset(0.f, 0.f, _position(i));
	}

	_trajectory[2].reset(0.f, 0.7f, _position(2));
}

void FlightTaskAutoLineSmoothVel::checkSetpoints(vehicle_local_position_setpoint_s &setpoints)
{
	// If the position setpoint is unknown, set to the current postion
	if (!PX4_ISFINITE(setpoints.x)) { setpoints.x = _position(0); }

	if (!PX4_ISFINITE(setpoints.y)) { setpoints.y = _position(1); }

	if (!PX4_ISFINITE(setpoints.z)) { setpoints.z = _position(2); }

	// If the velocity setpoint is unknown, set to the current velocity
	if (!PX4_ISFINITE(setpoints.vx)) { setpoints.vx = _velocity(0); }

	if (!PX4_ISFINITE(setpoints.vy)) { setpoints.vy = _velocity(1); }

	if (!PX4_ISFINITE(setpoints.vz)) { setpoints.vz = _velocity(2); }

	// No acceleration estimate available, set to zero if the setpoint is NAN
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(setpoints.acceleration[i])) { setpoints.acceleration[i] = 0.f; }
	}

	if (!PX4_ISFINITE(setpoints.yaw)) { setpoints.yaw = _yaw; }
}

/**
 * EKF reset handling functions
 * Those functions are called by the base FlightTask in
 * case of an EKF reset event
 */
void FlightTaskAutoLineSmoothVel::_ekfResetHandlerPositionXY()
{
	_trajectory[0].setCurrentPosition(_position(0));
	_trajectory[1].setCurrentPosition(_position(1));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerVelocityXY()
{
	_trajectory[0].setCurrentVelocity(_velocity(0));
	_trajectory[1].setCurrentVelocity(_velocity(1));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerPositionZ()
{
	_trajectory[2].setCurrentPosition(_position(2));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerVelocityZ()
{
	_trajectory[2].setCurrentVelocity(_velocity(2));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerHeading(float delta_psi)
{
	_yaw_sp_prev += delta_psi;
}

void FlightTaskAutoLineSmoothVel::_generateSetpoints()
{
	_prepareSetpoints();
	_generateTrajectory();

	if (!PX4_ISFINITE(_yaw_setpoint) && !PX4_ISFINITE(_yawspeed_setpoint)) {
		// no valid heading -> generate heading in this flight task
		_generateHeading();
	}
}

void FlightTaskAutoLineSmoothVel::_generateHeading()
{
	// Generate heading along trajectory if possible, otherwise hold the previous yaw setpoint
	if (!_generateHeadingAlongTraj()) {
		_yaw_setpoint = _yaw_sp_prev;
	}
}

bool FlightTaskAutoLineSmoothVel::_generateHeadingAlongTraj()
{
	bool res = false;
	Vector2f vel_sp_xy(_velocity_setpoint);
	Vector2f traj_to_target = Vector2f(_target) - Vector2f(_position);

	if ((vel_sp_xy.length() > .1f) &&
	    (traj_to_target.length() > _target_acceptance_radius)) {
		// Generate heading from velocity vector, only if it is long enough
		// and if the drone is far enough from the target
		_compute_heading_from_2D_vector(_yaw_setpoint, vel_sp_xy);
		res = true;
	}

	return res;
}

/* Constrain some value vith a constrain depending on the sign of the constraint
 * Example: 	- if the constrain is -5, the value will be constrained between -5 and 0
 * 		- if the constrain is 5, the value will be constrained between 0 and 5
 */
float FlightTaskAutoLineSmoothVel::_constrainOneSide(float val, float constraint)
{
	const float min = (constraint < FLT_EPSILON) ? constraint : 0.f;
	const float max = (constraint > FLT_EPSILON) ? constraint : 0.f;

	return math::constrain(val, min, max);
}

float FlightTaskAutoLineSmoothVel::_constrainAbs(float val, float max)
{
	return math::sign(val) * math::min(fabsf(val), fabsf(max));
}

float FlightTaskAutoLineSmoothVel::_getMaxXYSpeed() const
{

	// short circuit if we have to stop anyways
	if (_param_mpc_yaw_mode.get() == 4 && !_yaw_sp_aligned) {
		return 0.f;
	}

	Vector3f pos_traj;
	pos_traj(0) = _trajectory[0].getCurrentPosition();
	pos_traj(1) = _trajectory[1].getCurrentPosition();
	pos_traj(2) = _trajectory[2].getCurrentPosition();

	vehicle_dynamic_limits config;
	config.z_accept_rad = _param_nav_mc_alt_rad.get();
	config.xy_accept_rad = _target_acceptance_radius;
	config.max_acc_xy = _trajectory[0].getMaxAccel();
	config.max_jerk = _trajectory[0].getMaxJerk();
	config.max_speed_xy = _trajectory[0].getMaxVel();
	config.max_acc_xy_radius_scale = _param_mpc_xy_traj_p.get();
	const float next_target_speed = 0.f;

	return computeCurrentXYSpeedFromWaypoints(pos_traj, _target, _next_wp, next_target_speed, config);
}

float FlightTaskAutoLineSmoothVel::_getMaxZSpeed() const
{
	const float distance_start_target = fabs(_target(2) - _trajectory[2].getCurrentPosition());

	return math::min(_trajectory[2].getMaxVel(),
			 math::trajectory::computeMaxSpeedFromDistance(_trajectory[2].getMaxJerk(),
					 _trajectory[2].getMaxAccel(),
					 distance_start_target,
					 0));
}

void FlightTaskAutoLineSmoothVel::_prepareSetpoints()
{
	// Interface: A valid position setpoint generates a velocity target using a P controller. If a velocity is specified
	// that one is used as a velocity limit.
	// If the position setpoints are set to NAN, the values in the velocity setpoints are used as velocity targets: nothing to do here.

	_want_takeoff = false;

	if (_param_mpc_yaw_mode.get() == 4 && !_yaw_sp_aligned) {
		// Wait for the yaw setpoint to be aligned
		_velocity_setpoint.setAll(0.f);

	} else {
		if (PX4_ISFINITE(_position_setpoint(0)) &&
		    PX4_ISFINITE(_position_setpoint(1))) {
			// Use position setpoints to generate velocity setpoints

			// Get various path specific vectors
			Vector3f pos_traj;
			pos_traj(0) = _trajectory[0].getCurrentPosition();
			pos_traj(1) = _trajectory[1].getCurrentPosition();
			pos_traj(2) = _trajectory[2].getCurrentPosition();
			Vector2f pos_traj_to_dest_xy = (_position_setpoint - pos_traj).xy();
			Vector2f u_pos_traj_to_dest_xy(pos_traj_to_dest_xy.unit_or_zero());

			Vector2f vel_sp_constrained_xy = u_pos_traj_to_dest_xy * _getMaxXYSpeed();

			for (int i = 0; i < 2; i++) {
				// If available, constrain the velocity using _velocity_setpoint(.)
				if (PX4_ISFINITE(_velocity_setpoint(i))) {
					_velocity_setpoint(i) = _constrainOneSide(vel_sp_constrained_xy(i), _velocity_setpoint(i));

				} else {
					_velocity_setpoint(i) = vel_sp_constrained_xy(i);
				}
			}
		}

		if (PX4_ISFINITE(_position_setpoint(2))) {
			const float z_dir = math::sign(_position_setpoint(2) - _trajectory[2].getCurrentPosition());
			const float vel_sp_z = z_dir * _getMaxZSpeed();

			// If available, constrain the velocity using _velocity_setpoint(.)
			if (PX4_ISFINITE(_velocity_setpoint(2))) {
				_velocity_setpoint(2) = _constrainOneSide(vel_sp_z, _velocity_setpoint(2));

			} else {
				_velocity_setpoint(2) = vel_sp_z;
			}

			_want_takeoff = _velocity_setpoint(2) < -0.3f;
		}
	}
}

void FlightTaskAutoLineSmoothVel::_updateTrajConstraints()
{
	// Update the constraints of the trajectories
	_trajectory[0].setMaxAccel(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
	_trajectory[1].setMaxAccel(_param_mpc_acc_hor.get());
	_trajectory[0].setMaxVel(_param_mpc_xy_vel_max.get());
	_trajectory[1].setMaxVel(_param_mpc_xy_vel_max.get());
	_trajectory[0].setMaxJerk(_param_mpc_jerk_auto.get()); // TODO : Should be computed using heading
	_trajectory[1].setMaxJerk(_param_mpc_jerk_auto.get());
	_trajectory[2].setMaxJerk(_param_mpc_jerk_auto.get());

	if (_velocity_setpoint(2) < 0.f) { // up
		_trajectory[2].setMaxAccel(_param_mpc_acc_up_max.get());
		_trajectory[2].setMaxVel(_param_mpc_z_vel_max_up.get());

	} else { // down
		_trajectory[2].setMaxAccel(_param_mpc_acc_down_max.get());
		_trajectory[2].setMaxVel(_param_mpc_z_vel_max_dn.get());
	}
}

void FlightTaskAutoLineSmoothVel::_generateTrajectory()
{
	if (!PX4_ISFINITE(_velocity_setpoint(0)) || !PX4_ISFINITE(_velocity_setpoint(1))
	    || !PX4_ISFINITE(_velocity_setpoint(2))) {
		return;
	}

	/* Slow down the trajectory by decreasing the integration time based on the position error.
	 * This is only performed when the drone is behind the trajectory
	 */
	Vector2f position_trajectory_xy(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
	Vector2f position_xy(_position);
	Vector2f vel_traj_xy(_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity());
	Vector2f drone_to_trajectory_xy(position_trajectory_xy - position_xy);
	float position_error = drone_to_trajectory_xy.length();

	float time_stretch = 1.f - math::constrain(position_error * 0.5f, 0.f, 1.f);

	// Don't stretch time if the drone is ahead of the position setpoint
	if (drone_to_trajectory_xy.dot(vel_traj_xy) < 0.f) {
		time_stretch = 1.f;
	}

	Vector3f jerk_sp_smooth;
	Vector3f accel_sp_smooth;
	Vector3f vel_sp_smooth;
	Vector3f pos_sp_smooth;

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].updateTraj(_deltatime, time_stretch);
		jerk_sp_smooth(i) = _trajectory[i].getCurrentJerk();
		accel_sp_smooth(i) = _trajectory[i].getCurrentAcceleration();
		vel_sp_smooth(i) = _trajectory[i].getCurrentVelocity();
		pos_sp_smooth(i) = _trajectory[i].getCurrentPosition();
	}

	_updateTrajConstraints();

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].updateDurations(_velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(_trajectory, 2); // Synchronize x and y only

	_jerk_setpoint = jerk_sp_smooth;
	_acceleration_setpoint = accel_sp_smooth;
	_velocity_setpoint = vel_sp_smooth;
	_position_setpoint = pos_sp_smooth;
}

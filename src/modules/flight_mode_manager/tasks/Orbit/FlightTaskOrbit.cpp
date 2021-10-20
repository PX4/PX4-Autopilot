/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskOrbit.cpp
 */

#include "FlightTaskOrbit.hpp"

#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskOrbit::FlightTaskOrbit()
{
	_sticks_data_required = false;
}

bool FlightTaskOrbit::applyCommandParameters(const vehicle_command_s &command)
{
	bool ret = true;
	// save previous velocity and roatation direction
	bool is_clockwise = _orbit_velocity > 0;

	float new_radius = _orbit_radius;
	float new_abs_velocity = fabsf(_orbit_velocity);

	// commanded radius
	if (PX4_ISFINITE(command.param1)) {
		// Note: Radius sign is defined as orbit direction in MAVLINK
		float radius = command.param1;
		is_clockwise = radius > 0;
		new_radius = fabsf(radius);
	}

	// commanded velocity, take sign of radius as rotation direction
	if (PX4_ISFINITE(command.param2)) {
		new_abs_velocity = command.param2;
	}

	float new_velocity = (is_clockwise ? 1.f : -1.f) * new_abs_velocity;
	_sanitizeParams(new_radius, new_velocity);
	_orbit_radius = new_radius;
	_orbit_velocity = new_velocity;

	// commanded heading behaviour
	if (PX4_ISFINITE(command.param3)) {
		_yaw_behaviour = command.param3;
	}

	// save current yaw estimate for ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING
	_initial_heading = _yaw;

	// commanded center coordinates
	if (PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6)) {
		if (map_projection_initialized(&_global_local_proj_ref)) {
			map_projection_project(&_global_local_proj_ref,
					       command.param5, command.param6,
					       &_center(0), &_center(1));

		} else {
			ret = false;
		}
	}

	// commanded altitude
	if (PX4_ISFINITE(command.param7)) {
		if (map_projection_initialized(&_global_local_proj_ref)) {
			_center(2) = _global_local_alt0 - command.param7;

		} else {
			ret = false;
		}
	}

	// perpendicularly approach the orbit circle again when new parameters get commanded
	if (!_is_position_on_circle()) {
		_in_circle_approach = true;
		_position_smoothing.reset({0.f, 0.f, 0.f}, _velocity, _position);
		_circle_approach_start_position = _position;
	}

	return ret;
}

bool FlightTaskOrbit::sendTelemetry()
{
	orbit_status_s orbit_status{};
	orbit_status.radius = math::signNoZero(_orbit_velocity) * _orbit_radius;
	orbit_status.frame = 0; // MAV_FRAME::MAV_FRAME_GLOBAL
	orbit_status.yaw_behaviour = _yaw_behaviour;

	if (map_projection_initialized(&_global_local_proj_ref)) {
		// local -> global
		map_projection_reproject(&_global_local_proj_ref, _center(0), _center(1), &orbit_status.x, &orbit_status.y);
		orbit_status.z = _global_local_alt0 - _position_setpoint(2);

	} else {
		return false; // don't send the message if the transformation failed
	}

	orbit_status.timestamp = hrt_absolute_time();
	_orbit_status_pub.publish(orbit_status);

	return true;
}

void FlightTaskOrbit::_sanitizeParams(float &radius, float &velocity) const
{
	// clip the radius to be within range
	radius = math::constrain(radius, _radius_min, _radius_max);
	velocity = math::constrain(velocity, -fabsf(_velocity_max), fabsf(_velocity_max));

	bool exceeds_maximum_acceleration = (velocity * velocity) >= _acceleration_max * radius;

	// value combination is not valid. Reduce velocity instead of
	// radius, as small radius + low velocity is better for safety
	if (exceeds_maximum_acceleration) {
		velocity = sign(velocity) * sqrtf(_acceleration_max * radius);
	}
}

bool FlightTaskOrbit::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);
	_orbit_radius = _radius_min;
	_orbit_velocity =  1.f;
	_sanitizeParams(_orbit_radius, _orbit_velocity);
	_center = _position;
	_initial_heading = _yaw;
	_slew_rate_yaw.setForcedValue(_yaw);
	_slew_rate_yaw.setSlewRate(math::radians(_param_mpc_yawrauto_max.get()));

	// need a valid position and velocity
	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	_position_smoothing.reset({0.f, 0.f, 0.f}, _velocity, _position);
	_circle_approach_start_position = _position;

	return ret;
}

bool FlightTaskOrbit::update()
{
	// update altitude
	bool ret = FlightTaskManualAltitude::update();

	_updateTrajectoryBoundaries();

	// stick input adjusts parameters within a fixed time frame
	float radius = _orbit_radius - _sticks.getPositionExpo()(0) * _deltatime * (_radius_max / 8.f);
	float velocity = _orbit_velocity - _sticks.getPositionExpo()(1) * _deltatime * (_velocity_max / 4.f);
	_sanitizeParams(radius, velocity);
	_orbit_radius = radius;
	_orbit_velocity = velocity;

	if (_is_position_on_circle()) {
		if (_in_circle_approach) {
			_in_circle_approach = false;
			_altitude_velocity_smoothing.reset(0, _velocity(2), _position(2));
		}

	} else {
		if (!_in_circle_approach) {
			_in_circle_approach = true;
			_position_smoothing.reset({0.f, 0.f, 0.f}, _velocity, _position);
			_circle_approach_start_position = _position;
		}
	}

	if (_in_circle_approach) {
		_generate_circle_approach_setpoints();

	} else {
		// this generates x / y setpoints
		_generate_circle_setpoints();
		_generate_circle_yaw_setpoints();

		// in case we have a velocity setpoint in altititude (from altitude parent)
		// smooth this
		if (!PX4_ISFINITE(_position_setpoint(2))) {
			_altitude_velocity_smoothing.updateDurations(_velocity_setpoint(2));
			_altitude_velocity_smoothing.updateTraj(_deltatime);
			_velocity_setpoint(2) = _altitude_velocity_smoothing.getCurrentVelocity();
			_acceleration_setpoint(2) = _altitude_velocity_smoothing.getCurrentAcceleration();
			// set orbit altitude center to expected new altitude
			_center(2) = _altitude_velocity_smoothing.getCurrentPosition();
		}
	}

	// Apply yaw smoothing
	_yaw_setpoint = _slew_rate_yaw.update(_yaw_setpoint, _deltatime);

	// publish information to UI
	sendTelemetry();

	return ret;
}

void FlightTaskOrbit::_updateTrajectoryBoundaries()
{
	// update params of the position smoothing
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setCruiseSpeed(_param_mpc_xy_cruise.get());
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setTargetAcceptanceRadius(_horizontal_acceptance_radius);

	// Update the constraints of the trajectories
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	float max_jerk = _param_mpc_jerk_auto.get();
	_position_smoothing.setMaxJerk({max_jerk, max_jerk, max_jerk}); // TODO : Should be computed using heading
	_altitude_velocity_smoothing.setMaxJerk(max_jerk);

	if (_unsmoothed_velocity_setpoint(2) < 0.f) { // up
		float z_accel_constraint = _param_mpc_acc_up_max.get();
		float z_vel_constraint = _param_mpc_z_vel_max_up.get();

		_position_smoothing.setMaxVelocityZ(z_vel_constraint);
		_position_smoothing.setMaxAccelerationZ(z_accel_constraint);
		_altitude_velocity_smoothing.setMaxVel(z_vel_constraint);
		_altitude_velocity_smoothing.setMaxAccel(z_accel_constraint);

	} else { // down
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_vel_max_dn.get());
		_altitude_velocity_smoothing.setMaxVel(_param_mpc_acc_down_max.get());
		_altitude_velocity_smoothing.setMaxAccel(_param_mpc_z_vel_max_dn.get());
	}

}

bool FlightTaskOrbit::_is_position_on_circle() const
{
	return (fabsf(Vector2f(_position - _center).length() - _orbit_radius) < _horizontal_acceptance_radius)
	       && fabsf(_position(2) - _center(2)) < _param_nav_mc_alt_rad.get();

}

void FlightTaskOrbit::_generate_circle_approach_setpoints()
{
	const Vector2f center2d = Vector2f(_center);
	const Vector2f position_to_center_xy = center2d - Vector2f(_position);
	Vector2f closest_point_on_circle = Vector2f(_position) + position_to_center_xy.unit_or_zero() *
					   (position_to_center_xy.norm() - _orbit_radius);

	const Vector3f target_circle_point{closest_point_on_circle(0), closest_point_on_circle(1), _center(2)};

	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	_position_smoothing.generateSetpoints(_position, target_circle_point,
	{0.f, 0.f, 0.f}, _deltatime, false, out_setpoints);

	_yaw_setpoint = atan2f(position_to_center_xy(1), position_to_center_xy(0));

	_position_setpoint = out_setpoints.position;
	_velocity_setpoint = out_setpoints.velocity;
}

void FlightTaskOrbit::_generate_circle_setpoints()
{
	Vector3f center_to_position = _position - _center;
	// xy velocity to go around in a circle
	Vector2f velocity_xy(-center_to_position(1), center_to_position(0));
	velocity_xy = velocity_xy.unit_or_zero();
	velocity_xy *= _orbit_velocity;

	// xy velocity adjustment to stay on the radius distance
	velocity_xy += (_orbit_radius - center_to_position.xy().norm()) * Vector2f(center_to_position).unit_or_zero();

	_position_setpoint(0) = _position_setpoint(1) = NAN;
	_velocity_setpoint.xy() = velocity_xy;
	_acceleration_setpoint.xy() = -Vector2f(center_to_position.unit_or_zero()) * _orbit_velocity * _orbit_velocity /
				      _orbit_radius;
}

void FlightTaskOrbit::_generate_circle_yaw_setpoints()
{
	Vector3f center_to_position = _position - _center;

	switch (_yaw_behaviour) {
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING:
		// make vehicle keep the same heading as when the orbit was commanded
		_yaw_setpoint = _initial_heading;
		_yawspeed_setpoint = NAN;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_UNCONTROLLED:
		// no yaw setpoint
		_yaw_setpoint = NAN;
		_yawspeed_setpoint = NAN;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE:
		_yaw_setpoint = atan2f(sign(_orbit_velocity) * center_to_position(0), -sign(_orbit_velocity) * center_to_position(1));
		_yawspeed_setpoint = _orbit_velocity / _orbit_radius;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED:
		// inherit setpoint from altitude flight task
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER:
	default:
		_yaw_setpoint = atan2f(-center_to_position(1), -center_to_position(0));
		// yawspeed feed-forward because we know the necessary angular rate
		_yawspeed_setpoint = _orbit_velocity / _orbit_radius;
		break;
	}
}

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

using namespace matrix;

FlightTaskAutoLineSmoothVel::FlightTaskAutoLineSmoothVel() :
	_sticks(this),
	_stick_acceleration_xy(this)
{}

bool FlightTaskAutoLineSmoothVel::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskAuto::activate(last_setpoint);

	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;

	Vector3f vel_prev{last_setpoint.vx, last_setpoint.vy, last_setpoint.vz};
	Vector3f pos_prev{last_setpoint.x, last_setpoint.y, last_setpoint.z};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current postion
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// No acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }
	}

	_position_smoothing.reset(accel_prev, vel_prev, pos_prev);

	_yaw_sp_prev = PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw;
	_updateTrajConstraints();
	_is_emergency_braking_active = false;

	return ret;
}

void FlightTaskAutoLineSmoothVel::reActivate()
{
	FlightTaskAuto::reActivate();

	// On ground, reset acceleration and velocity to zero
	_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.7f}, _position);
}

/**
 * EKF reset handling functions
 * Those functions are called by the base FlightTask in
 * case of an EKF reset event
 */
void FlightTaskAutoLineSmoothVel::_ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy)
{
	_position_smoothing.forceSetPosition({_position(0), _position(1), NAN});
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy)
{
	_position_smoothing.forceSetVelocity({_velocity(0), _velocity(1), NAN});
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerPositionZ(float delta_z)
{
	_position_smoothing.forceSetPosition({NAN, NAN, _position(2)});
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerVelocityZ(float delta_vz)
{
	_position_smoothing.forceSetVelocity({NAN, NAN, _velocity(2)});
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerHeading(float delta_psi)
{
	_yaw_sp_prev += delta_psi;
}

bool FlightTaskAutoLineSmoothVel::update()
{
	bool ret = FlightTaskAuto::update();
	// always reset constraints because they might change depending on the type
	_setDefaultConstraints();

	// The only time a thrust set-point is sent out is during
	// idle. Hence, reset thrust set-point to NAN in case the
	// vehicle exits idle.

	if (_type_previous == WaypointType::idle) {
		_acceleration_setpoint.setNaN();
	}

	// during mission and reposition, raise the landing gears but only
	// if altitude is high enough
	if (_highEnoughForLandingGear()) {
		_gear.landing_gear = landing_gear_s::GEAR_UP;
	}

	switch (_type) {
	case WaypointType::idle:
		_prepareIdleSetpoints();
		break;

	case WaypointType::land:
		_prepareLandSetpoints();
		break;

	case WaypointType::loiter:

	/* fallthrought */
	case WaypointType::position:
		_preparePositionSetpoints();
		break;

	case WaypointType::takeoff:
		_prepareTakeoffSetpoints();
		break;

	case WaypointType::velocity:
		_prepareVelocitySetpoints();
		break;

	default:
		_preparePositionSetpoints();
		break;
	}

	if (_param_com_obs_avoid.get()) {
		_obstacle_avoidance.updateAvoidanceDesiredSetpoints(_position_setpoint, _velocity_setpoint, (int)_type);
		_obstacle_avoidance.injectAvoidanceSetpoints(_position_setpoint, _velocity_setpoint, _yaw_setpoint,
				_yawspeed_setpoint);
	}


	_generateSetpoints();

	// update previous type
	_type_previous = _type;

	return ret;
}

void FlightTaskAutoLineSmoothVel::_generateSetpoints()
{
	_checkEmergencyBraking();
	Vector3f waypoints[] = {_prev_wp, _position_setpoint, _next_wp};

	if (isTargetModified()) {
		// In case object avoidance has injected a new setpoint, we take this as the next waypoints
		waypoints[2] = _position_setpoint;
	}

	const bool should_wait_for_yaw_align = _param_mpc_yaw_mode.get() == 4 && !_yaw_sp_aligned;
	_updateTrajConstraints();
	PositionSmoothing::PositionSmoothingSetpoints smoothed_setpoints;
	_position_smoothing.generateSetpoints(
		_position,
		waypoints,
		_velocity_setpoint,
		_deltatime,
		should_wait_for_yaw_align,
		smoothed_setpoints
	);

	_jerk_setpoint = smoothed_setpoints.jerk;
	_acceleration_setpoint = smoothed_setpoints.acceleration;
	_velocity_setpoint = smoothed_setpoints.velocity;
	_position_setpoint = smoothed_setpoints.position;

	_unsmoothed_velocity_setpoint = smoothed_setpoints.unsmoothed_velocity;
	_want_takeoff = smoothed_setpoints.unsmoothed_velocity(2) < -0.3f;

	if (!PX4_ISFINITE(_yaw_setpoint) && !PX4_ISFINITE(_yawspeed_setpoint)) {
		// no valid heading -> generate heading in this flight task
		_generateHeading();
	}
}

void FlightTaskAutoLineSmoothVel::_checkEmergencyBraking()
{
	if (!_is_emergency_braking_active) {
		// activate emergency braking if significantly outside of velocity bounds
		if (_position_smoothing.getCurrentVelocityZ() > (1.3f * _param_mpc_z_vel_max_dn.get())
		    || _position_smoothing.getCurrentVelocityZ() < -(1.3f * _param_mpc_z_vel_max_up.get())
		    || fabsf(_position_smoothing.getCurrentVelocityX()) > (1.3f * _param_mpc_xy_cruise.get())
		    || fabsf(_position_smoothing.getCurrentVelocityY()) > (1.3f * _param_mpc_xy_cruise.get())) {
			_is_emergency_braking_active = true;
		}

	} else {
		// deactivate emergency braking if all velocities and accelerations are again within their bounds
		if (_position_smoothing.getCurrentVelocityZ() < _param_mpc_z_vel_max_dn.get()
		    && _position_smoothing.getCurrentVelocityZ() > -_param_mpc_z_vel_max_up.get()
		    && _position_smoothing.getCurrentAccelerationZ() > -_param_mpc_acc_up_max.get()
		    && _position_smoothing.getCurrentAccelerationZ() < _param_mpc_acc_down_max.get()
		    && fabsf(_position_smoothing.getCurrentVelocityX()) < _param_mpc_xy_cruise.get()
		    && fabsf(_position_smoothing.getCurrentVelocityY()) < _param_mpc_xy_cruise.get()
		    && fabsf(_position_smoothing.getCurrentAccelerationX()) < _param_mpc_acc_hor.get()
		    && fabsf(_position_smoothing.getCurrentAccelerationY()) < _param_mpc_acc_hor.get()) {
			_is_emergency_braking_active = false;
		}
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
	    (traj_to_target.length() > 2.f)) {
		// Generate heading from velocity vector, only if it is long enough
		// and if the drone is far enough from the target
		_compute_heading_from_2D_vector(_yaw_setpoint, vel_sp_xy);
		res = true;
	}

	return res;
}


bool FlightTaskAutoLineSmoothVel::isTargetModified() const
{
	const bool xy_modified = (_target - _position_setpoint).xy().longerThan(FLT_EPSILON);
	const bool z_valid = PX4_ISFINITE(_position_setpoint(2));
	const bool z_modified =  z_valid && fabs((_target - _position_setpoint)(2)) > FLT_EPSILON;

	return xy_modified || z_modified;
}


void FlightTaskAutoLineSmoothVel::_updateTrajConstraints()
{
	// update params of the position smoothing
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setCruiseSpeed(_mc_cruise_speed);
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setTargetAcceptanceRadius(_target_acceptance_radius);

	// Update the constraints of the trajectories
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	float max_jerk = _param_mpc_jerk_auto.get();
	_position_smoothing.setMaxJerk({max_jerk, max_jerk, max_jerk}); // TODO : Should be computed using heading

	if (_is_emergency_braking_active) {
		// When initializing with large velocity, allow 1g of horizontal
		// acceleration for fast braking
		_position_smoothing.setMaxAcceleration({9.81f, 9.81f, 9.81f});
		_position_smoothing.setMaxJerk({9.81f, 9.81f, 9.81f});

		// If the current velocity is beyond the usual constraints, tell
		// the controller to exceptionally increase its saturations to avoid
		// cutting out the feedforward
		_constraints.speed_xy = math::max(math::max(fabsf(_position_smoothing.getCurrentVelocityX()),
						  fabsf(_position_smoothing.getCurrentVelocityY())), _param_mpc_xy_cruise.get());
		_constraints.speed_down = math::max(fabsf(_position_smoothing.getCurrentVelocityZ()), _param_mpc_z_vel_max_dn.get());
		_constraints.speed_up = math::max(fabsf(_position_smoothing.getCurrentVelocityZ()), _param_mpc_z_vel_max_up.get());

	} else if (_unsmoothed_velocity_setpoint(2) < 0.f) { // up
		float z_accel_constraint = _param_mpc_acc_up_max.get();
		float z_vel_constraint = _param_mpc_z_vel_max_up.get();

		// The constraints are broken because they are used as hard limits by the position controller, so put this here
		// until the constraints don't do things like cause controller integrators to saturate. Once the controller
		// doesn't use z speed constraints, this can go in _prepareTakeoffSetpoints(). Accel limit is to
		// emulate the motor ramp (also done in the controller) so that the controller can actually track the setpoint.
		if (_type == WaypointType::takeoff &&  _dist_to_ground < _param_mpc_land_alt1.get()) {
			z_vel_constraint = _param_mpc_tko_speed.get();
			z_accel_constraint = math::min(z_accel_constraint, _param_mpc_tko_speed.get() / _param_mpc_tko_ramp_t.get());

			// Keep the altitude setpoint at the current altitude
			// to avoid having it going down into the ground during
			// the initial ramp as the velocity does not start at 0
			_position_smoothing.forceSetPosition({NAN, NAN, _position(2)});
		}

		_position_smoothing.setMaxVelocityZ(z_vel_constraint);
		_position_smoothing.setMaxAccelerationZ(z_accel_constraint);

	} else { // down
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_vel_max_dn.get());
	}
}

void FlightTaskAutoLineSmoothVel::_prepareIdleSetpoints()
{
	// Send zero thrust setpoint
	_position_setpoint.setNaN(); // Don't require any position/velocity setpoints
	_velocity_setpoint.setNaN();
	_acceleration_setpoint = Vector3f(0.f, 0.f, 100.f); // High downwards acceleration to make sure there's no thrust
}

void FlightTaskAutoLineSmoothVel::_prepareLandSetpoints()
{
	_velocity_setpoint.setNaN(); // Don't take over any smoothed velocity setpoint

	// Slow down automatic descend close to ground
	float land_speed = math::gradual(_dist_to_ground,
					 _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
					 _param_mpc_land_speed.get(), _constraints.speed_down);

	if (_type_previous != WaypointType::land) {
		// initialize xy-position and yaw to waypoint such that home is reached exactly without user input
		_land_position = Vector3f(_target(0), _target(1), NAN);
		_land_heading = _yaw_setpoint;
		_stick_acceleration_xy.resetPosition(Vector2f(_target(0), _target(1)));
	}

	// User input assisted landing
	if (_param_mpc_land_rc_help.get() && _sticks.checkAndSetStickInputs()) {
		// Stick full up -1 -> stop, stick full down 1 -> double the speed
		land_speed *= (1 + _sticks.getPositionExpo()(2));

		_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _land_heading,
					       _sticks.getPositionExpo()(3) * math::radians(_param_mpc_man_y_max.get()), _yaw, _deltatime);
		_stick_acceleration_xy.generateSetpoints(_sticks.getPositionExpo().slice<2, 1>(0, 0), _yaw, _land_heading, _position,
				_velocity_setpoint_feedback.xy(), _deltatime);
		_stick_acceleration_xy.getSetpoints(_land_position, _velocity_setpoint, _acceleration_setpoint);

		// Hack to make sure the MPC_YAW_MODE 4 alignment doesn't stop the vehicle from descending when there's yaw input
		if (fabsf(_yawspeed_setpoint) > FLT_EPSILON) {
			_yaw_sp_aligned = true;
		}

	} else {
		// Make sure we have a valid land position even in the case we loose RC while amending it
		if (!PX4_ISFINITE(_land_position(0))) {
			_land_position.xy() = Vector2f(_position);
		}
	}

	_position_setpoint = _land_position; // The last element of the land position has to stay NAN
	_yaw_setpoint = _land_heading;
	_velocity_setpoint(2) = land_speed;
	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoLineSmoothVel::_prepareTakeoffSetpoints()
{
	// Takeoff is completely defined by target position
	_position_setpoint = _target;
	_velocity_setpoint = Vector3f(NAN, NAN, NAN);

	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoLineSmoothVel::_prepareVelocitySetpoints()
{
	// XY Velocity waypoint
	// TODO : Rewiew that. What is the expected behavior?
	_position_setpoint = Vector3f(NAN, NAN, _position(2));
	Vector2f vel_sp_xy = Vector2f(_velocity).unit_or_zero() * _mc_cruise_speed;
	_velocity_setpoint = Vector3f(vel_sp_xy(0), vel_sp_xy(1), NAN);
}

void FlightTaskAutoLineSmoothVel::_preparePositionSetpoints()
{
	// Simple waypoint navigation: go to xyz target, with standard limitations
	_position_setpoint = _target;
	_velocity_setpoint.setNaN(); // No special velocity limitations
}

void FlightTaskAutoLineSmoothVel::updateParams()
{
	FlightTaskAuto::updateParams();

	// make sure that alt1 is above alt2
	_param_mpc_land_alt1.set(math::max(_param_mpc_land_alt1.get(), _param_mpc_land_alt2.get()));
}

bool FlightTaskAutoLineSmoothVel::_highEnoughForLandingGear()
{
	// return true if altitude is above two meters
	return _dist_to_ground > 2.0f;
}

/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskTakeoff.cpp
 */

#include "FlightTaskTakeoff.hpp"

using namespace matrix;

bool FlightTaskTakeoff::activate(const trajectory_setpoint_s &last_setpoint)
{
	const bool ret = FlightTask::activate(last_setpoint);

	_yaw_hold = PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw;

	if (_takeoff_status_sub.get().takeoff_state >= takeoff_status_s::TAKEOFF_STATE_FLIGHT) {
		// Already flying, there's no ground to leave. Continue from the last setpoint.
		Vector3f acceleration{last_setpoint.acceleration};
		Vector3f velocity{last_setpoint.velocity};
		Vector3f position{last_setpoint.position};

		for (int i = 0; i < 3; i++) {
			if (!PX4_ISFINITE(acceleration(i))) { acceleration(i) = 0.f; }

			if (!PX4_ISFINITE(velocity(i))) { velocity(i) = _velocity(i); }

			if (!PX4_ISFINITE(position(i))) { position(i) = _position(i); }
		}

		startClimb(acceleration, velocity, position);

	} else {
		// Also called through reActivate() on the ground until the takeoff ramp starts
		startLevel();
	}

	return ret;
}

bool FlightTaskTakeoff::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_position_setpoint_triplet_sub.update();
	_takeoff_status_sub.update();

	return ret && _position.isAllFinite() && _velocity.isAllFinite();
}

bool FlightTaskTakeoff::update()
{
	const bool ret = FlightTask::update();
	_setDefaultConstraints();

	const float height = _ground_z - _position(2);
	const float takeoff_z = takeoffZ();

	if (_phase != Phase::climb) {
		// Climb after easing in, or earlier to not overshoot a low takeoff altitude, but never before
		// leaving the ground level.
		float climb_height = kLevelHeight + kEaseInHeight;

		if (PX4_ISFINITE(takeoff_z)) {
			climb_height = math::constrain(_ground_z - takeoff_z - kEaseInHeight, kLevelHeight, climb_height);
		}

		if (_phase == Phase::level && height >= kLevelHeight) {
			startEaseIn();
		}

		if (height >= climb_height) {
			// Continue from the current setpoints
			Vector3f acceleration{_acceleration_setpoint};
			Vector3f velocity{_velocity_setpoint};

			for (int i = 0; i < 3; i++) {
				if (!PX4_ISFINITE(acceleration(i))) { acceleration(i) = 0.f; }

				if (!PX4_ISFINITE(velocity(i))) { velocity(i) = _velocity(i); }
			}

			startClimb(acceleration, velocity, _position);
		}
	}

	switch (_phase) {
	case Phase::level:
		updateLevel();
		break;

	case Phase::ease_in:
		updateEaseIn(height);
		break;

	case Phase::climb:
		updateClimb();
		break;
	}

	_yaw_setpoint = _yaw_hold;
	_yawspeed_setpoint = NAN;

	_constraints.want_takeoff = PX4_ISFINITE(takeoff_z) && (takeoff_z < _position(2));
	return ret;
}

float FlightTaskTakeoff::takeoffZ() const
{
	const position_setpoint_s &current = _position_setpoint_triplet_sub.get().current;
	const vehicle_local_position_s &local_position = _sub_vehicle_local_position.get();

	if (current.valid && (current.type != position_setpoint_s::SETPOINT_TYPE_IDLE) && PX4_ISFINITE(current.alt)
	    && local_position.z_global && PX4_ISFINITE(local_position.ref_alt)) {
		return local_position.ref_alt - current.alt;
	}

	return NAN;
}

void FlightTaskTakeoff::startLevel()
{
	_phase = Phase::level;
	_ground_z = _position(2);
}

void FlightTaskTakeoff::startEaseIn()
{
	_phase = Phase::ease_in;
	// Start braking from where the vehicle drifted to while climbing level
	_ease_in_velocity_xy = Vector2f(_velocity);
}

void FlightTaskTakeoff::startClimb(const Vector3f &acceleration, const Vector3f &velocity, const Vector3f &position)
{
	_phase = Phase::climb;
	_position_smoothing.reset(acceleration, velocity, position);
}

void FlightTaskTakeoff::updateLevel()
{
	// No horizontal correction: zero acceleration keeps the vehicle level
	_position_setpoint.setNaN();
	_velocity_setpoint = Vector3f(NAN, NAN, -_param_mpc_tko_speed.get());
	_acceleration_setpoint = Vector3f(0.f, 0.f, NAN);
}

void FlightTaskTakeoff::updateEaseIn(const float height)
{
	// Brake the horizontal velocity setpoint to zero with an acceleration limit growing from zero,
	// which in turn limits the tilt
	// Drag through the air can brake the vehicle faster than the growing limit allows. Never ask it
	// to speed up again in the direction it's braking.
	const Vector2f direction = _ease_in_velocity_xy.unit_or_zero();
	const float speed_along = math::max(Vector2f(_velocity).dot(direction), 0.f);

	if (speed_along < _ease_in_velocity_xy.norm()) {
		_ease_in_velocity_xy = direction * speed_along;
	}

	const float progress = math::constrain((height - kLevelHeight) / kEaseInHeight, 0.f, 1.f);
	const float max_velocity_step = progress * _param_mpc_acc_hor.get() * _deltatime;

	Vector2f velocity_step = -_ease_in_velocity_xy;

	if (velocity_step.longerThan(max_velocity_step)) {
		velocity_step = velocity_step.unit_or_zero() * max_velocity_step;
	}

	_ease_in_velocity_xy += velocity_step;

	_position_setpoint.setNaN();
	_velocity_setpoint = Vector3f(_ease_in_velocity_xy(0), _ease_in_velocity_xy(1), -_param_mpc_tko_speed.get());
	_acceleration_setpoint.xy() = (_deltatime > FLT_EPSILON) ? velocity_step / _deltatime : Vector2f();
	_acceleration_setpoint(2) = NAN;
}

void FlightTaskTakeoff::updateClimb()
{
	// No horizontal target and zero horizontal velocity: brake and hold wherever the vehicle stops.
	// Without a takeoff altitude, stop vertically as well.
	const float takeoff_z = takeoffZ();
	const Vector3f target(NAN, NAN, takeoff_z);
	const Vector3f feedforward_velocity(0.f, 0.f, PX4_ISFINITE(takeoff_z) ? NAN : 0.f);

	_position_smoothing.setMaxJerk(_param_mpc_jerk_auto.get());
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get());
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setMaxAllowedVerticalError(_param_mpc_z_err_max.get());

	if (!PX4_ISFINITE(takeoff_z) || (takeoff_z < _position_smoothing.getCurrentPositionZ())) {
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_up.get());
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_up_max.get());

	} else {
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
	}

	PositionSmoothing::PositionSmoothingSetpoints setpoints;
	_position_smoothing.generateSetpoints(_position, target, feedforward_velocity, _deltatime, false, setpoints);

	_jerk_setpoint = setpoints.jerk;
	_acceleration_setpoint = setpoints.acceleration;
	_velocity_setpoint = setpoints.velocity;
	_position_setpoint = setpoints.position;
}

void FlightTaskTakeoff::_ekfResetHandlerPositionXY(const Vector2f &delta_xy)
{
	const Vector3f position = _position_smoothing.getCurrentPosition();
	_position_smoothing.forceSetPosition({position(0) + delta_xy(0), position(1) + delta_xy(1), NAN});
}

void FlightTaskTakeoff::_ekfResetHandlerVelocityXY(const Vector2f &delta_vxy)
{
	_ease_in_velocity_xy += delta_vxy;

	const Vector3f velocity = _position_smoothing.getCurrentVelocity();
	_position_smoothing.forceSetVelocity({velocity(0) + delta_vxy(0), velocity(1) + delta_vxy(1), NAN});
}

void FlightTaskTakeoff::_ekfResetHandlerPositionZ(float delta_z)
{
	_ground_z += delta_z;
	_position_smoothing.forceSetPosition({NAN, NAN, _position_smoothing.getCurrentPositionZ() + delta_z});
}

void FlightTaskTakeoff::_ekfResetHandlerVelocityZ(float delta_vz)
{
	_position_smoothing.forceSetVelocity({NAN, NAN, _position_smoothing.getCurrentVelocityZ() + delta_vz});
}

void FlightTaskTakeoff::_ekfResetHandlerHeading(float delta_psi)
{
	_yaw_hold = wrap_pi(_yaw_hold + delta_psi);
}

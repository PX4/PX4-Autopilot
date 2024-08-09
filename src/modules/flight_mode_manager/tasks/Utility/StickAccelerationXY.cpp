/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include "StickAccelerationXY.hpp"

#include <geo/geo.h>
#include "Sticks.hpp"

using namespace matrix;

StickAccelerationXY::StickAccelerationXY(ModuleParams *parent) :
	ModuleParams(parent)
{
	_brake_boost_filter.reset(1.f);
	resetPosition();
}

void StickAccelerationXY::resetPosition()
{
	_position_setpoint.setNaN();
}

void StickAccelerationXY::resetPosition(const matrix::Vector2f &position)
{
	_position_setpoint = position;
}

void StickAccelerationXY::resetVelocity(const matrix::Vector2f &velocity)
{
	if (velocity.isAllFinite()) {
		_velocity_setpoint = velocity;
	}
}

void StickAccelerationXY::resetAcceleration(const matrix::Vector2f &acceleration)
{
	if (acceleration.isAllFinite()) {
		_acceleration_slew_rate_x.setForcedValue(acceleration(0));
		_acceleration_slew_rate_y.setForcedValue(acceleration(1));
	}
}

void StickAccelerationXY::generateSetpoints(Vector2f stick_xy, const float yaw, const float yaw_sp, const Vector3f &pos,
		const matrix::Vector2f &vel_sp_feedback, const float dt)
{
	// maximum commanded velocity can be constrained dynamically
	const float velocity_sc = fminf(_param_mpc_vel_manual.get(), _velocity_constraint);
	Vector2f velocity_scale(velocity_sc, velocity_sc);
	// maximum commanded acceleration is scaled down with velocity
	const float acceleration_sc = _param_mpc_acc_hor.get() * (velocity_sc / _param_mpc_vel_manual.get());
	Vector2f acceleration_scale(acceleration_sc, acceleration_sc);

	acceleration_scale *= 2.f; // because of drag the average acceleration is half

	// Map stick input to acceleration
	Sticks::limitStickUnitLengthXY(stick_xy);

	if (_param_mpc_vel_man_side.get() >= 0.f) {
		stick_xy(1) *= _param_mpc_vel_man_side.get() / _param_mpc_vel_manual.get();
	}

	if ((_param_mpc_vel_man_back.get() >= 0.f) && (stick_xy(0) < 0.f)) {
		stick_xy(0) *= _param_mpc_vel_man_back.get() / _param_mpc_vel_manual.get();
	}

	Sticks::rotateIntoHeadingFrameXY(stick_xy, yaw, yaw_sp);
	_acceleration_setpoint = stick_xy.emult(acceleration_scale);

	if (_collision_prevention.is_active()) {
		matrix::Vector2f accel_setpoint_xy = _acceleration_setpoint;
		matrix::Vector2f vel_setpoint_xy = _velocity_setpoint;
		_collision_prevention.modifySetpoint(accel_setpoint_xy, vel_setpoint_xy);
		_acceleration_setpoint = accel_setpoint_xy;

	}

	// Add drag to limit speed and brake again
	Vector2f drag = calculateDrag(acceleration_scale.edivide(velocity_scale), dt, stick_xy, _velocity_setpoint);

	// Don't allow the drag to change the sign of the velocity, otherwise we might get into oscillations around 0, due
	// to discretization
	if (((_acceleration_setpoint.norm_squared() < FLT_EPSILON)
	     || (sign(_acceleration_setpoint_prev(0)) != sign(_acceleration_setpoint(0)))
	     || (sign(_acceleration_setpoint_prev(1)) != sign(_acceleration_setpoint(1))))
	    && (_velocity_setpoint.norm_squared() < (drag.norm_squared() * dt * dt))) {

		drag.setZero();
		_velocity_setpoint.setZero();
	}

	_acceleration_setpoint -= drag;

	applyJerkLimit(dt);
	applyTiltLimit(_acceleration_setpoint);

	// Generate velocity setpoint by forward integrating commanded acceleration
	_velocity_setpoint += _acceleration_setpoint * dt;

	lockPosition(pos, vel_sp_feedback, dt);
	_acceleration_setpoint_prev = _acceleration_setpoint;
}

void StickAccelerationXY::getSetpoints(Vector3f &pos_sp, Vector3f &vel_sp, Vector3f &acc_sp)
{
	pos_sp.xy() = _position_setpoint;
	vel_sp.xy() = _velocity_setpoint;
	acc_sp.xy() = _acceleration_setpoint;
}

void StickAccelerationXY::applyJerkLimit(const float dt)
{
	// Apply jerk limit - acceleration slew rate
	// Scale each jerk limit with the normalized projection of the acceleration
	// setpoint increment to produce a synchronized motion
	const Vector2f dir = Vector2f(_acceleration_setpoint - _acceleration_setpoint_prev).unit_or_zero();
	const float jerk_max_x = fabsf(dir(0)) * _param_mpc_jerk_max.get();
	const float jerk_max_y = fabsf(dir(1)) * _param_mpc_jerk_max.get();
	_acceleration_slew_rate_x.setSlewRate(jerk_max_x);
	_acceleration_slew_rate_y.setSlewRate(jerk_max_y);
	_acceleration_setpoint(0) = _acceleration_slew_rate_x.update(_acceleration_setpoint(0), dt);
	_acceleration_setpoint(1) = _acceleration_slew_rate_y.update(_acceleration_setpoint(1), dt);
}

Vector2f StickAccelerationXY::calculateDrag(Vector2f drag_coefficient, const float dt, const Vector2f &stick_xy,
		const Vector2f &vel_sp)
{
	_brake_boost_filter.setParameters(dt, .8f);

	if (stick_xy.norm_squared() < FLT_EPSILON) {
		_brake_boost_filter.update(math::max(2.f, sqrtf(_param_mpc_vel_manual.get())));

	} else {
		_brake_boost_filter.update(1.f);
	}

	drag_coefficient *= _brake_boost_filter.getState();

	// increase drag with squareroot function when velocity is lower than 1m/s
	const Vector2f velocity_with_sqrt_boost = vel_sp.unit_or_zero() * math::sqrt_linear(vel_sp.norm());

	// only apply the drag increase below 1m/s when actually braking such that speeds below 1m/s
	// are exactly reached but do so by blending it with the filter to avoid any discontinuity when switching
	const float brake_scale = math::interpolate(_brake_boost_filter.getState(), 1.f, 2.f, 0.f, 1.f);
	const Vector2f mixed_velocity = brake_scale * velocity_with_sqrt_boost + (1.f - brake_scale) * vel_sp;

	return drag_coefficient.emult(mixed_velocity);
}

void StickAccelerationXY::applyTiltLimit(Vector2f &acceleration)
{
	// fetch the tilt limit which is lower than the maximum during takeoff
	takeoff_status_s takeoff_status{};
	_takeoff_status_sub.copy(&takeoff_status);

	// Check if acceleration would exceed the tilt limit
	const float acc = acceleration.length();
	const float acc_tilt_max = tanf(takeoff_status.tilt_limit) * CONSTANTS_ONE_G;

	if (acc > acc_tilt_max) {
		acceleration *= acc_tilt_max / acc;
	}
}

void StickAccelerationXY::lockPosition(const Vector3f &pos, const matrix::Vector2f &vel_sp_feedback, const float dt)
{
	const bool moving = _velocity_setpoint.norm_squared() > FLT_EPSILON;
	const bool position_locked = Vector2f(_position_setpoint).isAllFinite();

	// lock position
	if (!moving && !position_locked) {
		_position_setpoint = pos.xy();
	}

	// open position loop
	if (moving && position_locked) {
		_position_setpoint.setNaN();

		// avoid velocity setpoint jump caused by ignoring remaining position error
		if (vel_sp_feedback.isAllFinite()) {
			_velocity_setpoint = vel_sp_feedback;
		}
	}
}

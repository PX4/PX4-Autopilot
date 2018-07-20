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
 * @file StraightLine.cpp
 */

#include "StraightLine.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

#define VEL_ZERO_THRESHOLD 0.001f
#define DECELERATION_MAX 8.0f

using namespace matrix;

StraightLine::StraightLine(ModuleParams *parent, const float &deltatime, const matrix::Vector3f &pos) :
	ModuleParams(parent),
	_deltatime(deltatime), _pos(pos)
{

}

void StraightLine::generateSetpoints(matrix::Vector3f &position_setpoint, matrix::Vector3f &velocity_setpoint)
{
	// Check if target position has been reached
	if (_is_target_reached || (_desired_speed_at_target < VEL_ZERO_THRESHOLD &&
				   (_pos - _target).length() < NAV_ACC_RAD.get())) {
		// Vehicle has reached target. Lock position
		position_setpoint = _target;
		velocity_setpoint = Vector3f(0.0f, 0.0f, 0.0f);
		_is_target_reached = true;

		return;
	}

	// unit vector in the direction of the straight line
	Vector3f u_orig_to_target = (_target - _origin).unit_or_zero();
	// vector from origin to current position
	Vector3f orig_to_pos = _pos - _origin;
	// current position projected perpendicularly onto desired line
	Vector3f closest_pt_on_line = _origin + u_orig_to_target * (orig_to_pos * u_orig_to_target);
	// previous velocity in the direction of the line
	float speed_sp_prev = math::max(velocity_setpoint * u_orig_to_target, 0.0f);

	// Calculate braking distance depending on speed, speed at target and deceleration (add 10% safety margin)
	float braking_distance = 1.1f * ((powf(_desired_speed, 2) - powf(_desired_speed_at_target, 2)) / (2.0f * _desired_deceleration));

	float dist_to_target = (_target - _pos).length(); // distance to target

	// Either accelerate or decelerate
	float speed_sp    = dist_to_target > braking_distance ? _desired_speed        :  _desired_speed_at_target;
	float max_acc_dec = dist_to_target > braking_distance ? _desired_acceleration : -_desired_deceleration;

	float acc_track = (speed_sp - speed_sp_prev) / _deltatime;

	if (fabs(acc_track) > fabs(max_acc_dec)) {
		// accelerate/decelerate with desired acceleration/deceleration towards target
		speed_sp = speed_sp_prev + max_acc_dec * _deltatime;
	}

	// constrain the velocity
	speed_sp = math::constrain(speed_sp, 0.0f, _desired_speed);

	// set the position and velocity setpoints
	position_setpoint = closest_pt_on_line;
	velocity_setpoint = u_orig_to_target * speed_sp;

}

float StraightLine::getMaxAcc()
{
	// unit vector in the direction of the straight line
	Vector3f u_orig_to_target = (_target - _origin).unit_or_zero();

	// calculate the maximal horizontal acceleration
	float divider = (sqrt(powf(u_orig_to_target(0), 2) + powf(u_orig_to_target(1), 2)));
	float max_acc_hor = MPC_ACC_HOR_MAX.get();

	if (divider > FLT_EPSILON) {
		max_acc_hor /= divider;

	} else {
		max_acc_hor *= 1000.0f;
	}

	// calculate the maximal vertical acceleration
	float max_acc_vert_original = u_orig_to_target(2) < 0 ? MPC_ACC_UP_MAX.get() : MPC_ACC_DOWN_MAX.get();
	float max_acc_vert = max_acc_vert_original;

	if (fabs(u_orig_to_target(2)) > FLT_EPSILON) {
		max_acc_vert /= fabs(u_orig_to_target(2));

	} else {
		max_acc_vert *= 1000.0f;
	}

	return math::min(max_acc_hor, max_acc_vert);
}

float StraightLine::getMaxVel()
{
	// unit vector in the direction of the straight line
	Vector3f u_orig_to_target = (_target - _origin).unit_or_zero();

	// calculate the maximal horizontal velocity
	float divider = (sqrt(powf(u_orig_to_target(0), 2) + powf(u_orig_to_target(1), 2)));
	float max_vel_hor = MPC_XY_VEL_MAX.get();

	if (divider > FLT_EPSILON) {
		max_vel_hor /= divider;

	} else {
		max_vel_hor *= 1000.0f;
	}

	// calculate the maximal vertical velocity
	float max_vel_vert_directional = u_orig_to_target(2) < 0 ? MPC_Z_VEL_MAX_UP.get() : MPC_Z_VEL_MAX_DN.get();
	float max_vel_vert = max_vel_vert_directional;

	if (fabs(u_orig_to_target(2)) > FLT_EPSILON) {
		max_vel_vert /= fabs(u_orig_to_target(2));

	} else {
		max_vel_vert *= 1000.0f;
	}

	return math::min(max_vel_hor, max_vel_vert);
}

void StraightLine::setAllDefaults()
{
	_desired_speed = getMaxVel();
	_desired_speed_at_target = 0.0f;
	_desired_acceleration = getMaxAcc();
	_desired_deceleration = DECELERATION_MAX;
}

void StraightLine::setTarget(const matrix::Vector3f &target)
{
	if (PX4_ISFINITE(target(0)) && PX4_ISFINITE(target(1)) && PX4_ISFINITE(target(2))) {
		_target = target;
		_is_target_reached = false;

		// set all parameters to their default value (depends on the direction)
		setAllDefaults();
	}
}

void StraightLine::setOrigin(const matrix::Vector3f &origin)
{
	if (PX4_ISFINITE(origin(0)) && PX4_ISFINITE(origin(1)) && PX4_ISFINITE(origin(2))) {
		_origin = origin;
	}
}

void StraightLine::setSpeed(const float &speed)
{
	if (speed > 0 && speed < getMaxVel()) {
		_desired_speed = speed;
	}
}

void StraightLine::setSpeedAtTarget(const float &speed_at_target)
{
	if (speed_at_target > 0 && speed_at_target < getMaxVel()) {
		_desired_speed_at_target = speed_at_target;
	}
}

void StraightLine::setAcceleration(const float &acc)
{
	if (acc > 0 && acc < getMaxAcc()) {
		_desired_acceleration = acc;
	}
}

void StraightLine::setDeceleration(const float &dec)
{
	if (dec > 0 && dec < DECELERATION_MAX) {
		_desired_deceleration = dec;
	}
}
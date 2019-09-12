/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 *    without spec{fic prior written permission.
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
 * @file ManualSmoothingZ.cpp
 */

#include "ManualSmoothingZ.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

ManualSmoothingZ::ManualSmoothingZ(ModuleParams *parent, const float &vel, const float &stick) :
	ModuleParams(parent),
	_vel(vel), _stick(stick), _vel_sp_prev(vel)
{
	_acc_state_dependent = _param_mpc_acc_up_max.get();
	_max_acceleration = _param_mpc_acc_up_max.get();
}

void
ManualSmoothingZ::smoothVelFromSticks(float &vel_sp, const float dt)
{
	updateAcceleration(vel_sp, dt);
	velocitySlewRate(vel_sp, dt);
	_vel_sp_prev = vel_sp;
}

void
ManualSmoothingZ::updateAcceleration(float &vel_sp, const float dt)
{
	// check for max acceleration
	setMaxAcceleration();

	// check if zero input stick
	const bool is_current_zero = (fabsf(_stick) <= FLT_EPSILON);

	// default is acceleration
	ManualIntentionZ intention = ManualIntentionZ::acceleration;

	// check zero input stick
	if (is_current_zero) {
		intention = ManualIntentionZ::brake;
	}


	// update intention
	if ((_intention != ManualIntentionZ::brake) && (intention == ManualIntentionZ::brake)) {

		// we start with lowest acceleration
		_acc_state_dependent = _param_mpc_acc_down_max.get();

		// reset slew-rate: this ensures that there
		// is no delay present when user demands to brake
		_vel_sp_prev = _vel;

	}

	switch (intention) {
	case ManualIntentionZ::brake: {

			// limit jerk when braking to zero
			float jerk = (_param_mpc_acc_up_max.get() - _acc_state_dependent) / dt;

			if (jerk > _param_mpc_jerk_max.get()) {
				_acc_state_dependent = _param_mpc_jerk_max.get() * dt + _acc_state_dependent;

			} else {
				_acc_state_dependent = _param_mpc_acc_up_max.get();
			}

			break;
		}

	case ManualIntentionZ::acceleration: {

			_acc_state_dependent = (getMaxAcceleration() - _param_mpc_acc_down_max.get())
					       * fabsf(_stick) + _param_mpc_acc_down_max.get();
			break;
		}
	}

	_intention = intention;
}

void
ManualSmoothingZ::setMaxAcceleration()
{
	if (_stick < -FLT_EPSILON) {
		// accelerating upward
		_max_acceleration =  _param_mpc_acc_up_max.get();

	} else if (_stick > FLT_EPSILON) {
		// accelerating downward
		_max_acceleration = _param_mpc_acc_down_max.get();

	} else {

		// want to brake
		if (fabsf(_vel_sp_prev) < FLT_EPSILON) {
			// at rest
			_max_acceleration = _param_mpc_acc_up_max.get();

		} else if (_vel_sp_prev < 0.0f) {
			// braking downward
			_max_acceleration = _param_mpc_acc_down_max.get();

		} else {
			// braking upward
			_max_acceleration = _param_mpc_acc_up_max.get();
		}
	}
}

void
ManualSmoothingZ::velocitySlewRate(float &vel_sp, const float dt)
{
	// limit vertical acceleration
	float acc = 0.f;

	if (dt > FLT_EPSILON) {
		acc = (vel_sp - _vel_sp_prev) / dt;
	}

	float max_acc = (acc < 0.0f) ? -_acc_state_dependent : _acc_state_dependent;

	if (fabsf(acc) > fabsf(max_acc)) {
		vel_sp = max_acc * dt + _vel_sp_prev;
	}
}

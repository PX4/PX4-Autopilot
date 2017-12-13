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
 * @file ManualSmoothingZ.hpp
 *
 * This Class is used for smoothing the velocity setpoints in Z-direction.
 */

#include "ManualSmoothingZ.hpp"
#include "uORB/topics/parameter_update.h"
#include <mathlib/mathlib.h>
#include <float.h>

ManualSmoothingZ::ManualSmoothingZ(const float &vel, const float &stick) :
	_vel(vel), _stick(stick)
{
	_acc_max_up_h = param_find("MPC_ACC_UP_MAX");
	_acc_max_down_h = param_find("MPC_ACC_DOWN_MAX");
	_jerk_max = param_find("MPC_JERK_MAX");

	/* Load the params the very first time */
	setParams();
}

/* in manual altitude control apply acceleration limit based on stick input
 * we consider two states
 * 1.) brake
 * 2.) accelerate */

void
ManualSmoothingZ::smoothVelFromSticks(float vel_sp[2], const float dt)
{
	updateParams();

	updateAcceleration(vel_sp, dt);

	velocitySlewRate(vel_sp, dt);

}

void
ManualSmoothingZ::updateParams()
{
	bool updated;
	parameter_update_s param_update;
	orb_check(_parameter_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_sub, &param_update);
		setParams();
	}
}

void
ManualSmoothingZ::setParams()
{
	param_get(_acc_max_up_h, &_acc_max_up);
	param_get(_acc_max_down_h, &_acc_max_down);
	param_get(_jerk_max_h, &_jerk_max);

}

void
ManualSmoothingZ::updateAcceleration(float vel_sp[2], const float dt)
{
	/* check if zero input stick */
	const bool is_current_zero = (fabsf(_stick) <= FLT_EPSILON);

	/* default is acceleration */
	Intention intention = Intention::acceleration;

	/* check zero input stick */
	if (is_current_zero) {
		intention = Intention::brake;
	}

	/*
	 * update intention
	 */
	if ((_intention != Intention::brake) && (intention == Intention::brake)) {

		/* we start with lowest acceleration */
		_acc_state_dependent = _acc_max_down;

		/* reset slewrate: this ensures that there
		 * is no delay present because of the slewrate
		 */

		vel_sp[1] = _vel;

	}

	switch (intention) {
	case Intention::brake: {

			/* limit jerk when braking to zero */
			float jerk = (_acc_max_up - _acc_state_dependent) / dt;

			if (jerk > _jerk_max) {
				_acc_state_dependent = _jerk_max * dt + _acc_state_dependent;

			} else {
				_acc_state_dependent = _acc_max_up;
			}

			break;
		}

	case Intention::acceleration: {

			_acc_state_dependent = (getMaxAcceleration(vel_sp) - _acc_max_down)
			_acc_state_dependent = (getMaxAcceleration() - _acc_max_down)
					       * fabsf(_stick) + _acc_max_down;
			break;
		}
	}

	_intention = intention;
}

float
ManualSmoothingZ::getMaxAcceleration(float vel_sp[2])
{
	/* Note: NED frame */

	if (_stick < 0.0f) {
		/* accelerating upward */
		return _acc_max_up;

	} else if (_stick > 0.0f) {
		/* accelerating downward */
		return _acc_max_down;

	} else {

		/* want to brake */

		if (fabsf(vel_sp[0] - vel_sp[1]) < FLT_EPSILON) {
			/* at rest */
			return _acc_max_up;

		} else if (vel_sp[0] < 0.0f ) {
			/* braking downward */
			return _acc_max_down;

		} else {
			/* braking upward */
			return _acc_max_up;
		}
	}
}

void
ManualSmoothingZ::velocitySlewRate(float vel_sp[2], const float dt)
{
	/* limit vertical acceleration */
	float acc = (vel_sp[0] - vel_sp[1]) / dt;
	float max_acc = (acc < 0.0f) ? -_acc_state_dependent : _acc_state_dependent;

	if (fabsf(acc) > fabsf(max_acc)) {
		vel_sp[0] = max_acc * dt + vel_sp[1];
	}
}

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
 * @file ManualSmoothingXY.cpp
 */

#include "ManualSmoothingXY.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include <px4_defines.h>

using namespace matrix;

ManualSmoothingXY::ManualSmoothingXY(ModuleParams *parent, const Vector2f &vel) :
	ModuleParams(parent), 	_vel_sp_prev(vel)

{
	_acc_state_dependent = _param_mpc_acc_hor.get();
	_jerk_state_dependent = _param_mpc_jerk_max.get();
}

void
ManualSmoothingXY::smoothVelocity(Vector2f &vel_sp, const Vector2f &vel, const float &yaw,
				  const float &yawrate_sp, const float dt)
{
	_updateAcceleration(vel_sp, vel, yaw, yawrate_sp, dt);
	_velocitySlewRate(vel_sp, dt);
	_vel_sp_prev = vel_sp;
}

void
ManualSmoothingXY::_updateAcceleration(Vector2f &vel_sp, const Vector2f &vel,  const float &yaw,
				       const float &yawrate_sp, const float dt)
{
	Intention intention = _getIntention(vel_sp, vel, yaw, yawrate_sp);

	// Adapt acceleration and jerk based on current state and
	// intention. Jerk is only used for braking.
	_setStateAcceleration(vel_sp, vel, intention, dt);
}

ManualSmoothingXY::Intention
ManualSmoothingXY::_getIntention(const Vector2f &vel_sp, const Vector2f &vel, const float &yaw,
				 const float &yawrate_sp)
{

	if (vel_sp.length() > FLT_EPSILON) {
		// Distinguish between acceleration, deceleration and direction change

		// Check if stick direction and current velocity are within 135.
		// If current setpoint and velocity are more than 135 apart, we assume
		// that the user demanded a direction change.
		// The detection is done in body frame.
		// Rotate velocity setpoint into body frame
		Vector2f vel_sp_heading = _getWorldToHeadingFrame(vel_sp, yaw);
		Vector2f vel_heading = _getWorldToHeadingFrame(vel, yaw);

		if (vel_sp_heading.length() > FLT_EPSILON) {
			vel_sp_heading.normalize();
		}

		if (vel_heading.length() > FLT_EPSILON) {
			vel_heading.normalize();
		}

		const bool is_aligned = (vel_sp_heading * vel_heading) > -0.707f;

		// In almost all cases we want to use acceleration.
		// Only use direction change if not aligned, no yawspeed demand, demand larger than 0.7 of max speed and velocity larger than 2m/s.
		// Only use deceleration if stick input is lower than previous setpoint, aligned and no yawspeed demand.
		bool yawspeed_demand =  fabsf(yawrate_sp) > 0.05f && PX4_ISFINITE(yawrate_sp);
		bool direction_change = !is_aligned && (vel_sp.length() > 0.7f * _vel_max) && !yawspeed_demand
					&& (vel.length() > 2.0f);
		bool deceleration = is_aligned && (vel_sp.length() < _vel_sp_prev.length()) && !yawspeed_demand;

		if (direction_change) {
			return Intention::direction_change;

		} else if (deceleration) {
			return Intention::deceleration;

		} else {
			return Intention::acceleration;
		}
	}

	return Intention::brake; //default is brake
}

void
ManualSmoothingXY::_setStateAcceleration(const Vector2f &vel_sp, const Vector2f &vel,
		const Intention &intention, const float dt)
{
	switch (intention) {
	case Intention::brake: {

			if (_intention == Intention::direction_change) {
				// Vehicle switched from direction change to brake.
				// Don't do any slwerate and just stop.
				_jerk_state_dependent = 1e6f;
				_vel_sp_prev = vel;

			} else if (intention != _intention) {
				// start the brake with lowest acceleration which
				// makes stopping smoother
				_acc_state_dependent = _param_mpc_dec_hor_slow.get();

				// Adjust jerk based on current velocity. This ensures
				// that the vehicle will stop much quicker at large speed but
				// very slow at low speed.
				_jerk_state_dependent = 1e6f; // default

				if (_param_mpc_jerk_max.get() > _param_mpc_jerk_min.get() && _param_mpc_jerk_min.get() > FLT_EPSILON) {

					_jerk_state_dependent = math::min((_param_mpc_jerk_max.get() - _param_mpc_jerk_min.get())
									  / _vel_max * vel.length() + _param_mpc_jerk_min.get(), _param_mpc_jerk_max.get());
				}

				// User wants to brake smoothly but does NOT want the vehicle to
				// continue to fly in the opposite direction. slewrate has to be reset
				// by setting previous velocity setpoint to current velocity. */
				_vel_sp_prev = vel;
			}

			/* limit jerk when braking to zero */
			float jerk = (_param_mpc_acc_hor_max.get() - _acc_state_dependent) / dt;

			if (jerk > _jerk_state_dependent) {
				_acc_state_dependent = _jerk_state_dependent * dt
						       + _acc_state_dependent;

			} else {
				_acc_state_dependent = _param_mpc_acc_hor_max.get();
			}

			break;
		}

	case Intention::direction_change: {
			// We allow for fast change by setting previous setpoint to current  setpoint.


			// Because previous setpoint is equal to current setpoint,
			// slewrate will have no effect. Nonetheless, just set
			// acceleration to maximum.
			_acc_state_dependent = _param_mpc_acc_hor_max.get();
			break;
		}

	case Intention::acceleration: {
			// Limit acceleration linearly based on velocity setpoint.
			_acc_state_dependent = (_param_mpc_acc_hor.get() - _param_mpc_dec_hor_slow.get())
					       / _vel_max * vel_sp.length() + _param_mpc_dec_hor_slow.get();
			break;
		}

	case Intention::deceleration: {
			_acc_state_dependent = _param_mpc_dec_hor_slow.get();
			break;
		}
	}

	// Update intention for next iteration.
	_intention = intention;
}

void
ManualSmoothingXY::_velocitySlewRate(Vector2f &vel_sp, const float dt)
{
	// Adjust velocity setpoint if demand exceeds acceleration. /
	Vector2f acc{};

	if (dt > FLT_EPSILON) {
		acc = (vel_sp - _vel_sp_prev) / dt;
	}

	if (acc.length() > _acc_state_dependent) {
		vel_sp = acc.normalized() * _acc_state_dependent  * dt + _vel_sp_prev;
	}
}

Vector2f
ManualSmoothingXY::_getWorldToHeadingFrame(const Vector2f &vec, const float &yaw)
{
	Quatf q_yaw = AxisAnglef(Vector3f(0.0f, 0.0f, 1.0f), yaw);
	Vector3f vec_heading = q_yaw.conjugate_inversed(Vector3f(vec(0), vec(1), 0.0f));
	return Vector2f(vec_heading);
}

Vector2f
ManualSmoothingXY::_getHeadingToWorldFrame(const Vector2f &vec, const float &yaw)
{
	Quatf q_yaw = AxisAnglef(Vector3f(0.0f, 0.0f, 1.0f), yaw);
	Vector3f vec_world = q_yaw.conjugate(Vector3f(vec(0), vec(1), 0.0f));
	return Vector2f(vec_world);
}

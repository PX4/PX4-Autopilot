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
 * @file FlightTaskManual.hpp
 *
 * Flight task for the normal, legacy, manual position controlled flight
 * where stick inputs map basically to the velocity setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "FlightTaskManual.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

int FlightTaskManual::activate()
{
	FlightTask::activate();
	_filter_roll_stick.set_cutoff_frequency(_rc_flt_smp_rate.get(), _rc_flt_cutoff.get());
	_filter_pitch_stick.set_cutoff_frequency(_rc_flt_smp_rate.get(), _rc_flt_cutoff.get());
	_hold_position = Vector3f(NAN, NAN, NAN);
	return 0;
}

int FlightTaskManual::disable()
{
	FlightTask::disable();
	return 0;
}

int FlightTaskManual::update()
{
	int ret = FlightTask::update();
	ret += _evaluate_sticks();

	/* prepare stick input */
	Vector2f stick_xy(_sticks.data()); /**< horizontal two dimensional stick input within a unit circle */

	const float stick_xy_norm = stick_xy.norm();

	/* saturate such that magnitude in xy is never larger than 1 */
	if (stick_xy_norm > 1.0f) {
		stick_xy /= stick_xy_norm;
	}

	/* rotate stick input to produce velocity setpoint in NED frame */
	Vector3f velocity_setpoint(stick_xy(0), stick_xy(1), _sticks(3));
	velocity_setpoint = Dcmf(Eulerf(0.0f, 0.0f, get_input_frame_yaw())) * velocity_setpoint;

	/* scale [0,1] length velocity vector to maximal manual speed (in m/s) */
	Vector3f vel_scale(_velocity_hor_manual.get(),
			   _velocity_hor_manual.get(),
			   (velocity_setpoint(2) > 0.0f) ? _z_vel_max_down.get() : _z_vel_max_up.get());
	velocity_setpoint = velocity_setpoint.emult(vel_scale);

	/* smooth out velocity setpoint by slewrate and return it */
	vel_sp_slewrate(velocity_setpoint, stick_xy, _sticks(3));
	_set_velocity_setpoint(velocity_setpoint);

	/* handle position and altitude hold */
	const bool stick_xy_zero = stick_xy_norm <= FLT_EPSILON;
	const bool stick_z_zero = fabsf(_sticks(3)) <= FLT_EPSILON;

	float velocity_xy_norm = Vector2f(_velocity.data()).norm();
	const bool stopped_xy = (_hold_max_xy.get() < FLT_EPSILON || velocity_xy_norm < _hold_max_xy.get());
	const bool stopped_z = (_hold_max_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _hold_max_z.get());

	if (stick_xy_zero && stopped_xy && !PX4_ISFINITE(_hold_position(0))) {
		_hold_position(0) = _position(0);
		_hold_position(1) = _position(1);

	} else if (!stick_xy_zero) {
		_hold_position(0) = NAN;
		_hold_position(1) = NAN;
	}

	if (stick_z_zero && stopped_z && !PX4_ISFINITE(_hold_position(2))) {
		_hold_position(2) = _position(2);

	} else if (!stick_z_zero) {
		_hold_position(2) = NAN;
	}

	_set_position_setpoint(_hold_position);
	return ret;
}

int FlightTaskManual::_evaluate_sticks()
{
	if (hrt_elapsed_time(&_sub_manual_control_setpoint.get().timestamp) < _timeout) {
		/* get data and scale correctly */
		_sticks(0) = _sub_manual_control_setpoint.get().x; /* NED x, "pitch" [-1,1] */
		_sticks(1) = _sub_manual_control_setpoint.get().y; /* NED y, "roll" [-1,1] */
		_sticks(2) = -(_sub_manual_control_setpoint.get().z - 0.5f) * 2.f; /* NED z, "thrust" resacaled from [0,1] to [-1,1] */
		_sticks(3) = _sub_manual_control_setpoint.get().r; /* "yaw" [-1,1] */

		/* apply expo and deadzone */
		_sticks(0) = math::expo_deadzone(_sticks(0), _xy_vel_man_expo.get(), _hold_dz.get());
		_sticks(1) = math::expo_deadzone(_sticks(1), _xy_vel_man_expo.get(), _hold_dz.get());
		_sticks(2) = math::expo_deadzone(_sticks(2), _z_vel_man_expo.get(), _hold_dz.get());

		return 0;

	} else {
		_sticks.zero(); /* default is all zero */
		return 1;
	}
}

void FlightTaskManual::reset_slewrate_xy()
{
	_vel_sp_prev(0) = _velocity(0);
	_vel_sp_prev(1) = _velocity(1);
}

void FlightTaskManual::vel_sp_slewrate(Vector3f &vel_sp, const Vector2f &stick_xy, const float &stick_z)
{
	Vector2f vel_sp_xy(vel_sp(0), vel_sp(1));
	const Vector2f vel_sp_prev_xy(_vel_sp_prev(0), _vel_sp_prev(1));
	const Vector2f vel_xy(_velocity(0), _velocity(1));
	const Vector2f acc_xy = (vel_sp_xy - vel_sp_prev_xy) / _deltatime;

	const float acc_xy_max = get_acceleration_xy(stick_xy);

	/* limit total horizontal acceleration */
	if (acc_xy.length() > acc_xy_max) {
		vel_sp_xy = acc_xy_max * acc_xy.normalized() * _deltatime + vel_sp_prev_xy;
		vel_sp(0) = vel_sp_xy(0);
		vel_sp(1) = vel_sp_xy(1);
	}

	/* limit vertical acceleration */
	const float acc_z = (vel_sp(2) - _vel_sp_prev(2)) / _deltatime;
	const float acc_z_max = math::sign(acc_z) * get_acceleration_z(stick_z);

	if (fabsf(acc_z) > fabsf(acc_z_max)) {
		vel_sp(2) = acc_z_max * _deltatime + _vel_sp_prev(2);
	}

	_vel_sp_prev = vel_sp;
}

float FlightTaskManual::get_acceleration_xy(const Vector2f &stick_xy)
{
	/*
	 * In manual mode we consider four states with different acceleration handling:
	 * 1. user wants to stop/brake (lets go of sticks)
	 * 2. user wants to quickly change direction (> 90 degree)
	 * 3. user wants to accelerate
	 * 4. user wants to decelerate
	 */
	float acceleration_state_dependent_xy = 0.f;
	const Vector2f vel_xy(_velocity(0), _velocity(1));

	/* check input stick for zero or direction */
	const float stick_xy_norm = stick_xy.norm();
	const bool stick_xy_zero = stick_xy_norm <= FLT_EPSILON;
	Vector2f stick_xy_unit = stick_xy;

	if (!stick_xy_zero) {
		stick_xy_unit.normalize();
	}

	const float stick_xy_prev_norm = _stick_input_xy_prev.norm();
	const bool stick_xy_prev_zero = stick_xy_prev_norm <= FLT_EPSILON;
	Vector2f stick_xy_prev_unit = _stick_input_xy_prev;

	if (!stick_xy_prev_zero) {
		stick_xy_prev_unit.normalize();
	}

	/* check if stick direction changed more than 60 degree angle cos(60) = 0.5 */
	const bool previous_stick_aligned = (stick_xy_unit * stick_xy_prev_unit) > 0.5f;

	/* check acceleration */
	const bool do_acceleration = stick_xy_prev_zero || (previous_stick_aligned &&
				     ((stick_xy_norm > stick_xy_prev_norm) || (fabsf(stick_xy_norm - 1.0f) < FLT_EPSILON)));

	const bool do_deceleration = (previous_stick_aligned && (stick_xy_norm <= stick_xy_prev_norm));

	const bool do_direction_change = !previous_stick_aligned;

	stick_user_intention intention_xy;

	if (stick_xy_zero) {
		/* we want to stop */
		intention_xy = brake;

	} else if (do_acceleration) {
		/* we do manual acceleration */
		intention_xy = acceleration;

	} else if (do_deceleration) {
		/* we do manual deceleration */
		intention_xy = deceleration;

	} else if (do_direction_change) {
		/* we have a direction change */
		intention_xy = direction_change;

	} else {
		/* catchall: acceleration */
		intention_xy = acceleration;
	}

	/*
	 * execute the user intention
	 */

	/* pilot starts to have brake intention */
	if ((_intention_xy_prev != brake) && (intention_xy  == brake)) {

		if (_jerk_hor_max.get() > _jerk_hor_min.get()) {
			/* allow jerk proportional to velocity */
			const float jerk_range = _jerk_hor_max.get() - _jerk_hor_min.get();
			const float velocity_ratio = vel_xy.norm() / _velocity_hor_manual.get();
			_manual_jerk_limit_xy = _jerk_hor_min.get() + jerk_range * velocity_ratio;

			/* start braking with lowest deceleration */
			acceleration_state_dependent_xy = _deceleration_hor_slow.get();

		} else {
			/* set the jerk limit large because it's disabled*/
			_manual_jerk_limit_xy = 1e6f;
			/* to brake we use max acceleration */
			acceleration_state_dependent_xy = _acceleration_hor_max.get();
		}

		reset_slewrate_xy();
	}

	/* allowed state transitions */
	switch (_intention_xy_prev) {
	case brake: {

			/* pilot does not want to brake anymore, initialize with lowest acceleration */
			if (intention_xy != brake) {
				_intention_xy_prev = acceleration;
				acceleration_state_dependent_xy = _deceleration_hor_slow.get();
			}

			break;
		}

	case direction_change: {
			/* only exit direction change if brake or desired heading aligned */
			const bool stick_vel_aligned = (vel_xy * stick_xy_unit > 0.0f);
			_manual_direction_change_hysteresis.set_state_and_update(!stick_vel_aligned);

			if (intention_xy == brake) {
				_intention_xy_prev = intention_xy;

			} else if (stick_vel_aligned) {
				_intention_xy_prev = acceleration;

			} else if (_manual_direction_change_hysteresis.get_state()) {

				/* TODO: find conditions which are always continuous
				 * only if stick input is large*/
				if (stick_xy_norm > 0.6f) {
					acceleration_state_dependent_xy = _acceleration_hor_max.get();
				}
			}

			break;
		}

	case acceleration: {
			_intention_xy_prev = intention_xy;

			if (_intention_xy_prev == direction_change) {
				reset_slewrate_xy();
			}

			break;
		}

	case deceleration: {
			_intention_xy_prev = intention_xy;

			if (_intention_xy_prev == direction_change) {
				reset_slewrate_xy();
			}

			break;
		}
	}

	/*
	 * calculate dynamic acceleration based on state
	*/

	switch (_intention_xy_prev) {
	case brake: {
			/* limit jerk when braking to zero */
			float jerk = (_acceleration_hor_max.get() - acceleration_state_dependent_xy) / _deltatime;

			if (jerk > _manual_jerk_limit_xy) {
				acceleration_state_dependent_xy += _manual_jerk_limit_xy * _deltatime;

			} else {
				acceleration_state_dependent_xy = _acceleration_hor_max.get();
			}

			break;
		}

	case direction_change: {
			/* limit acceleration linearly on stick input*/
			const float acceleration_range = _acceleration_hor_manual.get() - _deceleration_hor_slow.get();
			acceleration_state_dependent_xy = _deceleration_hor_slow.get() + acceleration_range * stick_xy_norm;
			break;
		}

	case acceleration: {
			/* limit acceleration linearly on stick input*/
			float acc_limit  = (_acceleration_hor_manual.get() - _deceleration_hor_slow.get()) * stick_xy_norm
					   + _deceleration_hor_slow.get();

			if (acceleration_state_dependent_xy > acc_limit) {
				acc_limit = acceleration_state_dependent_xy;
			}

			acceleration_state_dependent_xy = acc_limit;
			break;
		}

	case deceleration: {
			acceleration_state_dependent_xy = _deceleration_hor_slow.get();
			break;
		}

	}

	/* update previous stick input */
	_stick_input_xy_prev = Vector2f(_filter_pitch_stick.apply(stick_xy(0)), _filter_roll_stick.apply(stick_xy(1)));

	if (_stick_input_xy_prev.length() > 1.0f) {
		_stick_input_xy_prev = _stick_input_xy_prev.normalized();
	}

	return acceleration_state_dependent_xy;
}

float FlightTaskManual::get_acceleration_z(const float &stick_z)
{
	/* in manual altitude control apply acceleration limit based on stick input
	 * we consider two states
	 * 1.) brake
	 * 2.) accelerate */
	float acceleration_state_dependent_z = 0.f;

	/* default is acceleration */
	stick_user_intention intention = acceleration;

	if (fabsf(stick_z) <= FLT_EPSILON) {
		intention = brake;
	}

	/* set maximum acceleration depending on upwards or downwards flight */
	float max_acceleration = (stick_z <= 0.0f) ? _acceleration_z_max_up.get() : _acceleration_z_max_down.get();

	/*
	 * update user input
	 */
	if ((_user_intention_z != brake) && (intention == brake)) {

		/* we start with lowest acceleration */
		acceleration_state_dependent_z = _acceleration_z_max_down.get();

		/* reset slew rate */
		_vel_sp_prev(2) = _velocity(2);
		_user_intention_z = brake;
	}

	_user_intention_z = intention;

	/*
	 * apply acceleration depending on state
	 */
	if (_user_intention_z == brake) {

		/* limit jerk when braking to zero */
		float jerk = (_acceleration_z_max_up.get() - acceleration_state_dependent_z) / _deltatime;

		if (jerk > _manual_jerk_limit_z) {
			acceleration_state_dependent_z += _manual_jerk_limit_z * _deltatime;

		} else {
			acceleration_state_dependent_z = _acceleration_z_max_up.get();
		}
	}

	if (_user_intention_z == acceleration) {
		const float acceleration_range = max_acceleration - _acceleration_z_max_down.get();
		acceleration_state_dependent_z = _acceleration_z_max_down.get() + acceleration_range * fabsf(stick_z);
	}

	return acceleration_state_dependent_z;
}

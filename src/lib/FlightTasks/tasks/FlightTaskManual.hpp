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

#pragma once

#include "FlightTask.hpp"

class FlightTaskManual : public FlightTask
{
public:
	FlightTaskManual(SuperBlock *parent, const char *name) :
		FlightTask(parent, name),
		_xy_vel_man_expo(parent, "MPC_XY_MAN_EXPO", false),
		_z_vel_man_expo(parent, "MPC_Z_MAN_EXPO", false),
		_hold_dz(parent, "MPC_HOLD_DZ", false),
		_velocity_hor_manual(parent, "MPC_VEL_MANUAL", false),
		_z_vel_max_up(parent, "MPC_Z_VEL_MAX_UP", false),
		_z_vel_max_down(parent, "MPC_Z_VEL_MAX_DN", false),
		_hold_max_xy(parent, "MPC_HOLD_MAX_XY", false),
		_hold_max_z(parent, "MPC_HOLD_MAX_Z", false),
		_sub_control_state(ORB_ID(control_state), 0, 0, &getSubscriptions())
	{};
	virtual ~FlightTaskManual() {};

	/**
	 * Call once on the event where you switch to the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int activate()
	{
		FlightTask::activate();
		_hold_position = matrix::Vector3f(NAN, NAN, NAN);
		return 0;
	};

	/**
	 * Call once on the event of switching away from the task
	 * 	@return 0 on success, >0 on error otherwise
	 */
	virtual int disable()
	{
		FlightTask::disable();
		return 0;
	};

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int update()
	{
		FlightTask::update();

		/* prepare stick input */
		matrix::Vector2f sick_xy;
		sick_xy(0) = math::expo_deadzone(_sticks(0), _xy_vel_man_expo.get(), _hold_dz.get());
		sick_xy(1) = math::expo_deadzone(_sticks(1), _xy_vel_man_expo.get(), _hold_dz.get());
		float stick_z = -math::expo_deadzone(_sticks(2), _z_vel_man_expo.get(), _hold_dz.get());

		const float stick_xy_norm = sick_xy.norm();

		/* saturate such that magnitude in xy is never larger than 1 */
		if (stick_xy_norm > 1.0f) {
			sick_xy /= stick_xy_norm;
		}

		/* rotate stick input to produce velocity setpoint in NED frame */
		matrix::Vector3f velocity_setpoint(sick_xy(0), sick_xy(1), stick_z);
		velocity_setpoint = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, get_input_frame_yaw())) * velocity_setpoint;

		/* scale [0,1] length velocity vector to maximal manual speed (in m/s) */
		matrix::Vector3f vel_scale(_velocity_hor_manual.get(),
					   _velocity_hor_manual.get(),
					   (velocity_setpoint(2) > 0.0f) ? _z_vel_max_down.get() : _z_vel_max_up.get());
		velocity_setpoint = velocity_setpoint.emult(vel_scale);

		_set_velocity_setpoint(velocity_setpoint);

		/* handle position and altitude hold */
		const bool stick_xy_zero = stick_xy_norm <= FLT_EPSILON;
		const bool stick_z_zero = fabsf(stick_z) <= FLT_EPSILON;

		float velocity_xy_norm = matrix::Vector2f(_velocity._data).norm();
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
		return 0;
	};

protected:
	float get_input_frame_yaw()
	{
		return _yaw;
	};

private:
	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed */
	control::BlockParamFloat _z_vel_max_up; /**< maximal vertical velocity when flying upwards with the stick */
	control::BlockParamFloat _z_vel_max_down; /**< maximal vertical velocity when flying downwards with the stick */
	control::BlockParamFloat _hold_max_xy; /**< velocity threshold to switch into horizontal position hold */
	control::BlockParamFloat _hold_max_z; /**< velocity threshold to switch into vertical position hold */

	uORB::Subscription<control_state_s> _sub_control_state;

	matrix::Vector3f _hold_position; /**< position at which the vehicle stays while the input is zero velocity */

	void set_manual_acceleration_xy(matrix::Vector2f &stick_xy, const float dt)
	{

		/*
		 * In manual mode we consider four states with different acceleration handling:
		 * 1. user wants to stop
		 * 2. user wants to quickly change direction
		 * 3. user wants to accelerate
		 * 4. user wants to decelerate
		 */

		/* get normalized stick input vector */
		matrix::Vector2f stick_xy_norm = (stick_xy.length() > 0.0f) ? stick_xy.normalized() : stick_xy;
		matrix::Vector2f stick_xy_prev_norm = (_stick_input_xy_prev.length() > 0.0f) ? _stick_input_xy_prev.normalized() :
						      _stick_input_xy_prev;

		/* check if stick direction and current velocity are within 60angle */
		const bool is_aligned = (stick_xy_norm * stick_xy_prev_norm) > 0.5f;

		/* check if zero input stick */
		const bool is_prev_zero = (fabsf(_stick_input_xy_prev.length()) <= FLT_EPSILON);
		const bool is_current_zero = (fabsf(stick_xy.length()) <= FLT_EPSILON);

		/* check acceleration */
		const bool do_acceleration = is_prev_zero || (is_aligned &&
					     ((stick_xy.length() > _stick_input_xy_prev.length()) || (fabsf(stick_xy.length() - 1.0f) < FLT_EPSILON)));

		const bool do_deceleration = (is_aligned && (stick_xy.length() <= _stick_input_xy_prev.length()));

		const bool do_direction_change = !is_aligned;

		manual_stick_input intention;

		if (is_current_zero) {
			/* we want to stop */
			intention = brake;

		} else if (do_acceleration) {
			/* we do manual acceleration */
			intention = acceleration;

		} else if (do_deceleration) {
			/* we do manual deceleration */
			intention = deceleration;

		} else if (do_direction_change) {
			/* we have a direction change */
			intention = direction_change;

		} else {
			/* catchall: acceleration */
			intention = acceleration;
		}


		/*
		 * update user intention
		 */

		/* we always want to break starting with slow deceleration */
		if ((_user_intention_xy != brake) && (intention  == brake)) {

			if (_jerk_hor_max.get() > _jerk_hor_min.get()) {
				_manual_jerk_limit_xy = (_jerk_hor_max.get() - _jerk_hor_min.get()) / _velocity_hor_manual.get() *
							sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1)) + _jerk_hor_min.get();

				/* we start braking with lowest accleration */
				_acceleration_state_dependent_xy = _deceleration_hor_slow.get();

			} else {

				/* set the jerk limit large since we don't know it better*/
				_manual_jerk_limit_xy = 1000000.f;

				/* at brake we use max acceleration */
				_acceleration_state_dependent_xy = _acceleration_hor_max.get();

			}

			/* reset slew rate */
			_vel_sp_prev(0) = _vel(0);
			_vel_sp_prev(1) = _vel(1);

		}

		switch (_user_intention_xy) {
		case brake: {
				if (intention != brake) {
					_user_intention_xy = acceleration;
					/* we initialize with lowest acceleration */
					_acceleration_state_dependent_xy = _deceleration_hor_slow.get();
				}

				break;
			}

		case direction_change: {
				/* only exit direction change if brake or aligned */
				matrix::Vector2f vel_xy(_vel(0), _vel(1));
				matrix::Vector2f vel_xy_norm = (vel_xy.length() > 0.0f) ? vel_xy.normalized() : vel_xy;
				bool stick_vel_aligned = (vel_xy_norm * stick_xy_norm > 0.0f);

				/* update manual direction change hysteresis */
				_manual_direction_change_hysteresis.set_state_and_update(!stick_vel_aligned);


				/* exit direction change if one of the condition is met */
				if (intention == brake) {
					_user_intention_xy = intention;

				} else if (stick_vel_aligned) {
					_user_intention_xy = acceleration;

				} else if (_manual_direction_change_hysteresis.get_state()) {

					/* TODO: find conditions which are always continuous
					 * only if stick input is large*/
					if (stick_xy.length() > 0.6f) {
						_acceleration_state_dependent_xy = _acceleration_hor_max.get();
					}
				}

				break;
			}

		case acceleration: {
				_user_intention_xy = intention;

				if (_user_intention_xy == direction_change) {
					_vel_sp_prev(0) = _vel(0);
					_vel_sp_prev(1) = _vel(1);
				}

				break;
			}

		case deceleration: {
				_user_intention_xy = intention;

				if (_user_intention_xy == direction_change) {
					_vel_sp_prev(0) = _vel(0);
					_vel_sp_prev(1) = _vel(1);
				}

				break;
			}
		}

		/*
		 * apply acceleration based on state
		*/
		switch (_user_intention_xy) {
		case brake: {

				/* limit jerk when braking to zero */
				float jerk = (_acceleration_hor_max.get() - _acceleration_state_dependent_xy) / dt;

				if (jerk > _manual_jerk_limit_xy) {
					_acceleration_state_dependent_xy = _manual_jerk_limit_xy * dt + _acceleration_state_dependent_xy;

				} else {
					_acceleration_state_dependent_xy = _acceleration_hor_max.get();
				}

				break;
			}

		case direction_change: {

				/* limit acceleration linearly on stick input*/
				_acceleration_state_dependent_xy = (_acceleration_hor_manual.get() - _deceleration_hor_slow.get()) * stick_xy.length() +
								   _deceleration_hor_slow.get();
				break;
			}

		case acceleration: {
				/* limit acceleration linearly on stick input*/
				float acc_limit  = (_acceleration_hor_manual.get() - _deceleration_hor_slow.get()) * stick_xy.length()
						   + _deceleration_hor_slow.get();

				if (_acceleration_state_dependent_xy > acc_limit) {
					acc_limit = _acceleration_state_dependent_xy;
				}

				_acceleration_state_dependent_xy = acc_limit;
				break;
			}

		case deceleration: {
				_acceleration_state_dependent_xy = _deceleration_hor_slow.get();
				break;
			}

		default :
			warn_rate_limited("User intention not recognized");
			_acceleration_state_dependent_xy = _acceleration_hor_max.get();

		}

		/* update previous stick input */
		_stick_input_xy_prev = matrix::Vector2f(_filter_manual_pitch.apply(stick_xy(0)),
							_filter_manual_roll.apply(stick_xy(1)));


		if (_stick_input_xy_prev.length() > 1.0f) {
			_stick_input_xy_prev = _stick_input_xy_prev.normalized();
		}
	}

};

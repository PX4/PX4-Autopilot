/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include "output_predictor.h"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

void OutputPredictor::print_status()
{
	printf("output predictor: IMU dt: %.4f, EKF dt: %.4f\n", (double)_dt_update_states_avg, (double)_dt_correct_states_avg);

	printf("output predictor: tracking error, angular: %.6f rad, velocity: %.3f m/s, position: %.3f m\n",
	       (double)_output_tracking_error(0), (double)_output_tracking_error(1), (double)_output_tracking_error(2));

	printf("output buffer: %d/%d (%d Bytes)\n", _output_buffer.entries(), _output_buffer.get_length(),
	       _output_buffer.get_total_size());
}

void OutputPredictor::reset()
{
	_output_buffer.reset();

	_accel_bias.setZero();
	_gyro_bias.setZero();

	_time_last_update_states_us = 0;
	_time_last_correct_states_us = 0;

	_output_new = {};
	_R_to_earth_now.setIdentity();
	_vel_imu_rel_body_ned.zero();
	_vel_deriv.zero();

	_delta_angle_corr.zero();
	_vel_err_integ.zero();
	_pos_err_integ.zero();

	_output_tracking_error.zero();

	_output_filter_aligned = false;
}

void OutputPredictor::resetQuaternion(const uint64_t time_delayed_us, const Quatf &new_quat)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();
	const Quatf quat_change{(new_quat * output_delayed.quat_nominal.inversed()).normalized()};
	const Dcmf rot_change{quat_change};

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = (quat_change * _output_buffer[i].quat_nominal).normalized();

		// apply change to velocity
		_output_buffer[i].vel = rot_change * _output_buffer[i].vel;
		_output_buffer[i].vel_alternative = rot_change * _output_buffer[i].vel_alternative;
	}

	// apply the change in attitude quaternion to our newest quaternion estimate
	_output_new.quat_nominal = (quat_change * _output_new.quat_nominal).normalized();
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

	// apply change to velocity
	_output_new.vel = rot_change * _output_new.vel;
	_output_new.vel_alternative = rot_change * _output_new.vel_alternative;

	propagateVelocityUpdateToPosition();
}

void OutputPredictor::resetHorizontalVelocityTo(const uint64_t time_delayed_us, const Vector2f &new_horz_vel)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// horizontal velocity
	{
		const Vector2f delta_vxy = new_horz_vel - Vector2f(output_delayed.vel.xy());

		for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
			_output_buffer[index].vel.xy() += delta_vxy;
		}

		_output_new.vel.xy() += delta_vxy;
	}

	// horizontal velocity (alternative)
	{
		const Vector2f delta_vxy = new_horz_vel - Vector2f(output_delayed.vel_alternative.xy());

		for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
			_output_buffer[index].vel_alternative.xy() += delta_vxy;
		}

		_output_new.vel_alternative.xy() += delta_vxy;
	}

	// propagate position forward using the reset velocity
	propagateVelocityUpdateToPosition();
}

void OutputPredictor::resetVerticalVelocityTo(const uint64_t time_delayed_us, const float new_vert_vel)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// vertical velocity
	{
		const float delta_z = new_vert_vel - output_delayed.vel(2);

		for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
			_output_buffer[index].vel(2) += delta_z;
		}

		_output_new.vel(2) += delta_z;
	}

	// vertical velocity (alternative)
	{
		const float delta_z = new_vert_vel - output_delayed.vel_alternative(2);

		for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
			_output_buffer[index].vel_alternative(2) += delta_z;
		}

		_output_new.vel_alternative(2) += delta_z;
	}

	// propagate vertical position forward using the reset velocity
	propagateVelocityUpdateToPosition();
}

void OutputPredictor::resetHorizontalPositionTo(const uint64_t time_delayed_us, const Vector2f &new_horz_pos)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// horizontal position
	{
		const Vector2f delta_xy = new_horz_pos - output_delayed.pos.xy();

		for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
			_output_buffer[index].pos.xy() += delta_xy;
		}

		_output_new.pos.xy() += delta_xy;
	}

	// horizontal position (alternative)
	{
		const Vector2f delta_xy = new_horz_pos - output_delayed.vel_alternative_integ.xy();

		for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
			_output_buffer[index].vel_alternative_integ.xy() += delta_xy;
		}

		_output_new.vel_alternative_integ.xy() += delta_xy;
	}
}

void OutputPredictor::resetVerticalPositionTo(const uint64_t time_delayed_us, const float new_vert_pos)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// vertical position
	{
		const float delta_z = new_vert_pos - output_delayed.pos(2);

		// add the reset amount to the output observer buffered data
		for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
			_output_buffer[i].pos(2) += delta_z;
		}

		// apply the change in height / height rate to our newest height / height rate estimate
		// which have already been taken out from the output buffer
		_output_new.pos(2) += delta_z;
	}

	// vertical position (alternative)
	{
		const float delta_z = new_vert_pos - output_delayed.vel_alternative_integ(2);

		// add the reset amount to the output observer buffered data
		for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
			_output_buffer[i].vel_alternative_integ(2) += delta_z;
		}

		// add the reset amount to the output observer vertical position state
		_output_new.vel_alternative_integ(2) += delta_z;
	}
}

bool OutputPredictor::calculateOutputStates(const uint64_t time_us, const Vector3f &delta_angle,
		const float delta_angle_dt, const Vector3f &delta_velocity, const float delta_velocity_dt)
{
	if (time_us <= _time_last_update_states_us) {
		return false;
	}

	// Use full rate IMU data at the current time horizon
	if (_time_last_update_states_us != 0) {
		const float dt = math::constrain((time_us - _time_last_update_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_update_states_avg = 0.8f * _dt_update_states_avg + 0.2f * dt;
	}

	_time_last_update_states_us = time_us;

	// correct delta angle and delta velocity for bias offsets
	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle_bias_scaled = _gyro_bias * delta_angle_dt;
	const Vector3f delta_angle_corrected(delta_angle - delta_angle_bias_scaled + _delta_angle_corr);

	const Vector3f delta_vel_bias_scaled = _accel_bias * delta_velocity_dt;
	const Vector3f delta_velocity_corrected(delta_velocity - delta_vel_bias_scaled);

	_output_new.time_us = time_us;

	// rotate the previous INS quaternion by the delta quaternions
	const Quatf dq(AxisAnglef{delta_angle_corrected});
	_output_new.quat_nominal = (_output_new.quat_nominal * dq).normalized();
	_R_to_earth_now = Dcmf(_output_new.quat_nominal); // calculate the rotation matrix from body to earth frame

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_earth{_R_to_earth_now * delta_velocity_corrected};

	// correct for measured acceleration due to gravity
	delta_vel_earth(2) += CONSTANTS_ONE_G * delta_velocity_dt;

	// calculate the earth frame velocity derivatives
	if (delta_velocity_dt > 0.001f) {
		_vel_deriv = delta_vel_earth / delta_velocity_dt;
	}

	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last(_output_new.vel);

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.vel += delta_vel_earth;
	_output_new.vel_alternative += delta_vel_earth;

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (delta_velocity_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_new.vel_alternative_integ += delta_pos_NED;

	// accumulate the time for each update
	_output_new.dt += delta_velocity_dt;

	// correct velocity for IMU offset
	if (delta_angle_dt > 0.001f) {
		// calculate the average angular rate across the last IMU update
		const Vector3f ang_rate = delta_angle_corrected / delta_angle_dt;

		// calculate the velocity of the IMU relative to the body origin
		const Vector3f vel_imu_rel_body = ang_rate % _imu_pos_body;

		// rotate the relative velocity into earth frame
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}

	// update auxiliary yaw estimate
	const Vector3f unbiased_delta_angle = delta_angle - delta_angle_bias_scaled;
	const float spin_del_ang_D = unbiased_delta_angle.dot(Vector3f(_R_to_earth_now.row(2)));
	_unaided_yaw = matrix::wrap_pi(_unaided_yaw + spin_del_ang_D);

	return true;
}

void OutputPredictor::correctOutputStates(const uint64_t time_delayed_us,
		const Quatf &quat_state, const Vector3f &vel_state, const Vector3f &pos_state,
		const matrix::Vector3f &gyro_bias, const matrix::Vector3f &accel_bias)
{
	const uint64_t time_latest_us = _time_last_update_states_us;

	if (time_latest_us <= time_delayed_us) {
		return;
	}

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	if (_output_new.dt > 0.f) {
		_output_buffer.push(_output_new);
		_output_new.dt = 0.0f; // reset time delta to zero for the next accumulation of full rate IMU data
	}

	// store IMU bias for calculateOutputStates
	_gyro_bias = gyro_bias;
	_accel_bias = accel_bias;

	// calculate an average filter update time
	if ((_time_last_correct_states_us != 0) && (time_delayed_us > _time_last_correct_states_us)) {
		const float dt = math::constrain((time_delayed_us - _time_last_correct_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_correct_states_avg = 0.8f * _dt_correct_states_avg + 0.2f * dt;

	} else {
		if (!_output_filter_aligned) {
			resetQuaternion(time_delayed_us, quat_state);
			resetHorizontalVelocityTo(time_delayed_us, vel_state.xy());
			resetVerticalVelocityTo(time_delayed_us, vel_state(2));
			resetHorizontalPositionTo(time_delayed_us, pos_state.xy());
			resetVerticalPositionTo(time_delayed_us, pos_state(2));
		}

		_time_last_correct_states_us = time_delayed_us;
		return;
	}

	_time_last_correct_states_us = time_delayed_us;

	// get the oldest INS state data from the ring buffer
	// this data will be at the EKF fusion time horizon
	outputSample output_delayed;

	while (_output_buffer.pop_first_older_than(time_delayed_us, &output_delayed)) {

		if (output_delayed.time_us != time_delayed_us) {
			continue;
		}

		if (!_output_filter_aligned) {

			// calculate the quaternion rotation delta from the EKF to output observer states at the EKF fusion time horizon
			const Quatf q_delta((quat_state * output_delayed.quat_nominal.inversed()).normalized());
			const Dcmf rot_change{q_delta};

			// add the reset amount to the output observer buffered data
			for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {

				_output_buffer[i].quat_nominal = (q_delta * _output_buffer[i].quat_nominal).normalized();

				// apply change to velocity
				_output_buffer[i].vel = rot_change * _output_buffer[i].vel;
				_output_buffer[i].vel_alternative = rot_change * _output_buffer[i].vel_alternative;
			}

			// apply the change in attitude quaternion to our newest quaternion estimate
			_output_new.quat_nominal = (q_delta * _output_new.quat_nominal).normalized();


			// apply change to velocity
			_output_new.vel = rot_change * _output_new.vel;
			_output_new.vel_alternative = rot_change * _output_new.vel_alternative;

			propagateVelocityUpdateToPosition();


			// calculate the velocity and position deltas between the output and EKF at the EKF fusion time horizon
			output_delayed.vel = q_delta.rotateVectorInverse(output_delayed.vel);
			const Vector3f vel_err = vel_state - output_delayed.vel;

			output_delayed.vel_alternative = q_delta.rotateVectorInverse(output_delayed.vel_alternative);
			const Vector3f vel_alternative_err = vel_state - output_delayed.vel_alternative;

			for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
				// a constant velocity correction is applied
				_output_buffer[index].vel += vel_err;
				_output_buffer[index].vel_alternative += vel_alternative_err;
			}

			// manually correct next oldest state
			//  position is propagated forward using the corrected velocity and a trapezoidal integrator
			uint8_t index = _output_buffer.get_oldest_index();
			outputSample &next_oldest_state = _output_buffer[index];

			if ((next_oldest_state.time_us > output_delayed.time_us) && (next_oldest_state.dt > 0.f)) {

				next_oldest_state.pos = pos_state + (vel_state + next_oldest_state.vel) * 0.5f * next_oldest_state.dt;
				next_oldest_state.vel_alternative_integ = pos_state + (vel_state + next_oldest_state.vel_alternative) * 0.5f *
						next_oldest_state.dt;

				propagateVelocityUpdateToPosition();
			}

			_output_new = _output_buffer.get_newest();
			_output_new.dt = 0.0f;

			// reset tracking error, integral, etc
			_output_tracking_error.zero();
			_delta_angle_corr.zero();
			_vel_err_integ.zero();
			_pos_err_integ.zero();

			_output_filter_aligned = true;

		} else {

			/*
			* Loop through the output filter state history and apply the corrections to the velocity and position states.
			* This method is too expensive to use for the attitude states due to the quaternion operations required
			* but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
			* to be used and reduces tracking error relative to EKF states.
			*/

			// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
			const Quatf q_error((quat_state.inversed() * output_delayed.quat_nominal).normalized());

			// convert the quaternion delta to a delta angle
			const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

			const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

			// calculate a gain that provides tight tracking of the estimator attitude states and
			// adjust for changes in time delay to maintain consistent damping ratio of ~0.7

			const float time_delay = fmaxf((time_latest_us - time_delayed_us) * 1e-6f, _dt_update_states_avg);
			const float att_gain = 0.5f * _dt_update_states_avg / time_delay;

			// calculate a corrrection to the delta angle
			// that will cause the INS to track the EKF quaternions
			_delta_angle_corr = delta_ang_error * att_gain;
			_output_tracking_error(0) = delta_ang_error.norm();


			/*
			* Calculate corrections to be applied to vel and pos output state history.
			* The vel and pos state history are corrected individually so they track the EKF states at
			* the fusion time horizon. This option provides the most accurate tracking of EKF states.
			*/

			// calculate velocity tracking errors
			const Vector3f vel_err = vel_state - output_delayed.vel;
			_output_tracking_error(1) = vel_err.norm();
			// calculate a velocity correction that will be applied to the output state history
			const float vel_gain = _dt_correct_states_avg / math::constrain(_vel_tau, _dt_correct_states_avg,
					       10.f); // Complementary filter gains
			_vel_err_integ += vel_err;
			const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

			// calculate position tracking errors
			const Vector3f pos_err = pos_state - output_delayed.pos;
			_output_tracking_error(2) = pos_err.norm();
			// calculate a position correction that will be applied to the output state history
			const float pos_gain = _dt_correct_states_avg / math::constrain(_pos_tau, _dt_correct_states_avg,
					       10.f); // Complementary filter gains
			_pos_err_integ += pos_err;
			const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

			for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
				// a constant velocity correction is applied
				_output_buffer[index].vel += vel_correction;
				_output_buffer[index].pos += pos_correction;
			}


			/*
			* Calculate a correction to be applied to vel_alternative that casues vel_alternative_integ to track the EKF
			* position state at the fusion time horizon using an alternative algorithm to what
			* is used for the vel and pos state tracking. The algorithm applies a correction to the vel_alternative
			* state history and propagates vel_alternative_integ forward in time using the corrected vel_alternative history.
			* This provides an alternative velocity output that is closer to the first derivative
			* of the position but does degrade tracking relative to the EKF state.
			*/

			// vel alternative: calculate velocity and position tracking errors
			const Vector3f vel_alternative_err = (vel_state - output_delayed.vel_alternative);
			const Vector3f vel_alternative_integ_err = (pos_state - output_delayed.vel_alternative_integ);

			// calculate a velocity correction that will be applied to the output state history
			// using a PD feedback tuned to a 5% overshoot
			const Vector3f vel_alternative_correction = vel_alternative_integ_err * pos_gain + vel_alternative_err * vel_gain *
					1.1f;

			for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
				// a constant velocity correction is applied
				_output_buffer[index].vel_alternative += vel_alternative_correction;
				_output_buffer[index].vel_alternative_integ += vel_alternative_integ_err;
			}

			// recompute position by integrating velocity
			propagateVelocityUpdateToPosition();
		}
	}

	_output_new = _output_buffer.get_newest();
	_output_new.dt = 0.f;

	_R_to_earth_now = Dcmf(_output_new.quat_nominal);
}

void OutputPredictor::propagateVelocityUpdateToPosition()
{
	// propagate position forward using the reset velocity
	uint8_t index = _output_buffer.get_oldest_index();
	const uint8_t size = _output_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {

		const outputSample &current_state = _output_buffer[index];

		// next state
		const uint8_t index_next = (index + 1) % size;
		outputSample &next_state = _output_buffer[index_next];

		if ((next_state.time_us > current_state.time_us) && (next_state.dt > 0.f)) {
			// position is propagated forward using the corrected velocity and a trapezoidal integrator
			next_state.pos = current_state.pos + (current_state.vel + next_state.vel) * 0.5f * next_state.dt;
			next_state.vel_alternative_integ = current_state.vel_alternative_integ + (current_state.vel_alternative +
							   next_state.vel_alternative) * 0.5f * next_state.dt;
		}

		// advance the index
		index = (index + 1) % size;
	}

	if ((_output_new.time_us > _output_buffer.get_newest().time_us) && (_output_new.dt > 0.f)) {
		// position is propagated forward using the corrected velocity and a trapezoidal integrator
		_output_new.pos = _output_buffer.get_newest().pos
				  + (_output_buffer.get_newest().vel + _output_new.vel) * 0.5f * _output_new.dt;

		_output_new.vel_alternative_integ = _output_buffer.get_newest().vel_alternative_integ +
						    (_output_buffer.get_newest().vel_alternative + _output_new.vel_alternative) * 0.5f * _output_new.dt;

	} else {
		// update output state to corrected values
		_output_new = _output_buffer.get_newest();

		// reset time delta to zero for the next accumulation of full rate IMU data
		_output_new.dt = 0.0f;
	}
}

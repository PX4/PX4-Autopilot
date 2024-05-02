/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4. All rights reserved.
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
	printf("[output predictor] IMU dt: %.6f, EKF dt: %.6f\n",
	       (double)_dt_update_states_avg, (double)_dt_correct_states_avg);

	const matrix::Quatf q_att = _output_buffer.get_newest().quat_nominal;
	const matrix::Eulerf euler = q_att;

	printf("[output predictor] orientation: [%.4f, %.4f, %.4f, %.4f] (Euler [%.3f, %.3f, %.3f])\n",
	       (double)q_att(0), (double)q_att(1), (double)q_att(2), (double)q_att(3),
	       (double)euler.phi(), (double)euler.theta(), (double)euler.psi());

	printf("[output predictor] velocity: [%.3f, %.3f, %.3f]\n",
	       (double)_output_buffer.get_newest().vel(0), (double)_output_buffer.get_newest().vel(1),
	       (double)_output_buffer.get_newest().vel(2));

	printf("[output predictor] position: [%.3f, %.3f, %.3f]\n",
	       (double)_output_buffer.get_newest().pos(0), (double)_output_buffer.get_newest().pos(1),
	       (double)_output_buffer.get_newest().pos(2));

	printf("[output predictor] tracking error, angular: %.6f rad, velocity: %.4f m/s, position: %.4f m\n",
	       (double)_output_tracking_error(0), (double)_output_tracking_error(1), (double)_output_tracking_error(2));

	printf("[output predictor] output buffer: %d/%d (%d Bytes)\n",
	       _output_buffer.entries(), _output_buffer.get_length(), _output_buffer.get_total_size());

	printf("[output predictor] output vert buffer: %d/%d (%d Bytes)\n",
	       _output_vert_buffer.entries(), _output_vert_buffer.get_length(), _output_vert_buffer.get_total_size());
}

void OutputPredictor::alignOutputFilter(const Quatf &quat_state, const Vector3f &vel_state, const Vector3f &pos_state)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// calculate the quaternion rotation delta from the EKF to output observer states at the EKF fusion time horizon
	Quatf q_delta{quat_state * output_delayed.quat_nominal.inversed()};
	q_delta.normalize();

	// calculate the velocity and position deltas between the output and EKF at the EKF fusion time horizon
	const Vector3f vel_delta = vel_state - output_delayed.vel;
	const Vector3f pos_delta = pos_state - output_delayed.pos;

	// loop through the output filter state history and add the deltas
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = q_delta * _output_buffer[i].quat_nominal;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}

	_output_new = _output_buffer.get_newest();
}

void OutputPredictor::reset()
{
	// TODO: who resets the output buffer content?
	_output_new = {};
	_output_vert_new = {};

	_accel_bias.setZero();
	_gyro_bias.setZero();

	_time_last_update_states_us = 0;
	_time_last_correct_states_us = 0;

	_R_to_earth_now.setIdentity();
	_vel_imu_rel_body_ned.setZero();
	_vel_deriv.setZero();

	_delta_angle_corr.setZero();

	_vel_err_integ.setZero();
	_pos_err_integ.setZero();

	_output_tracking_error.setZero();

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index] = {};
	}

	for (uint8_t index = 0; index < _output_vert_buffer.get_length(); index++) {
		_output_vert_buffer[index] = {};
	}
}

void OutputPredictor::resetQuaternion(const Quatf &quat_change)
{
	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = quat_change * _output_buffer[i].quat_nominal;
	}

	// apply the change in attitude quaternion to our newest quaternion estimate
	// which was already taken out from the output buffer
	_output_new.quat_nominal = quat_change * _output_new.quat_nominal;
}

void OutputPredictor::resetHorizontalVelocityTo(const Vector2f &delta_horz_vel)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel.xy() += delta_horz_vel;
	}

	_output_new.vel.xy() += delta_horz_vel;
}

void OutputPredictor::resetVerticalVelocityTo(float delta_vert_vel)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel(2) += delta_vert_vel;
		_output_vert_buffer[index].vert_vel += delta_vert_vel;
	}

	_output_new.vel(2) += delta_vert_vel;
	_output_vert_new.vert_vel += delta_vert_vel;
}

void OutputPredictor::resetHorizontalPositionTo(const Vector2f &delta_horz_pos)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].pos.xy() += delta_horz_pos;
	}

	_output_new.pos.xy() += delta_horz_pos;
}

void OutputPredictor::resetVerticalPositionTo(const float new_vert_pos, const float vert_pos_change)
{
	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_new.pos(2) += vert_pos_change;

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos(2) += vert_pos_change;
		_output_vert_buffer[i].vert_vel_integ += vert_pos_change;
	}

	// add the reset amount to the output observer vertical position state
	_output_vert_new.vert_vel_integ = new_vert_pos;
}

void OutputPredictor::calculateOutputStates(const uint64_t time_us, const Vector3f &delta_angle,
		const float delta_angle_dt, const Vector3f &delta_velocity, const float delta_velocity_dt)
{
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
	_output_vert_new.time_us = time_us;

	const Quatf dq(AxisAnglef{delta_angle_corrected});

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// the quaternions must always be normalised after modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

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
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (delta_velocity_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// accumulate the time for each update
	_output_vert_new.dt += delta_velocity_dt;

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
}

void OutputPredictor::correctOutputStates(const uint64_t time_delayed_us,
		const Quatf &quat_state, const Vector3f &vel_state, const Vector3f &pos_state, const matrix::Vector3f &gyro_bias, const matrix::Vector3f &accel_bias)
{
	// calculate an average filter update time
	if (_time_last_correct_states_us != 0) {
		const float dt = math::constrain((time_delayed_us - _time_last_correct_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_correct_states_avg = 0.8f * _dt_correct_states_avg + 0.2f * dt;
	}

	_time_last_correct_states_us = time_delayed_us;

	// store IMU bias for calculateOutputStates
	_gyro_bias = gyro_bias;
	_accel_bias = accel_bias;

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	_output_buffer.push(_output_new);
	_output_vert_buffer.push(_output_vert_new);

	// get the oldest INS state data from the ring buffer
	// this data will be at the EKF fusion time horizon
	// TODO: there is no guarantee that data is at delayed fusion horizon
	//       Shouldnt we use pop_first_older_than?
	const outputSample &output_delayed = _output_buffer.get_oldest();
	const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

	// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
	const Quatf q_error((quat_state.inversed() * output_delayed.quat_nominal).normalized());

	// convert the quaternion delta to a delta angle
	const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

	const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

	// calculate a gain that provides tight tracking of the estimator attitude states and
	// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
	const uint64_t time_latest_us = _time_last_update_states_us;
	const float time_delay = fmaxf((time_latest_us - time_delayed_us) * 1e-6f, _dt_update_states_avg);
	const float att_gain = 0.5f * _dt_update_states_avg / time_delay;

	// calculate a corrrection to the delta angle
	// that will cause the INS to track the EKF quaternions
	_delta_angle_corr = delta_ang_error * att_gain;
	_output_tracking_error(0) = delta_ang_error.norm();

	/*
	* Loop through the output filter state history and apply the corrections to the velocity and position states.
	* This method is too expensive to use for the attitude states due to the quaternion operations required
	* but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
	* to be used and reduces tracking error relative to EKF states.
	*/

	// Complementary filter gains
	const float vel_gain = _dt_correct_states_avg / math::constrain(_vel_tau, _dt_correct_states_avg, 10.f);
	const float pos_gain = _dt_correct_states_avg / math::constrain(_pos_tau, _dt_correct_states_avg, 10.f);

	// calculate down velocity and position tracking errors
	const float vert_vel_err = (vel_state(2) - output_vert_delayed.vert_vel);
	const float vert_vel_integ_err = (pos_state(2) - output_vert_delayed.vert_vel_integ);

	// calculate a velocity correction that will be applied to the output state history
	// using a PD feedback tuned to a 5% overshoot
	const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

	applyCorrectionToVerticalOutputBuffer(vert_vel_correction);

	// calculate velocity and position tracking errors
	const Vector3f vel_err(vel_state - output_delayed.vel);
	const Vector3f pos_err(pos_state - output_delayed.pos);

	_output_tracking_error(1) = vel_err.norm();
	_output_tracking_error(2) = pos_err.norm();

	// calculate a velocity correction that will be applied to the output state history
	_vel_err_integ += vel_err;
	const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

	// calculate a position correction that will be applied to the output state history
	_pos_err_integ += pos_err;
	const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

	applyCorrectionToOutputBuffer(vel_correction, pos_correction);
}

void OutputPredictor::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
{
	// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
	// vert_vel states and propagate vert_vel_integ forward using the corrected vert_vel
	uint8_t index = _output_vert_buffer.get_oldest_index();

	const uint8_t size = _output_vert_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputVert &current_state = _output_vert_buffer[index];
		outputVert &next_state = _output_vert_buffer[index_next];

		// correct the velocity
		if (counter == 0) {
			current_state.vert_vel += vert_vel_correction;
		}

		next_state.vert_vel += vert_vel_correction;

		// position is propagated forward using the corrected velocity and a trapezoidal integrator
		next_state.vert_vel_integ = current_state.vert_vel_integ + (current_state.vert_vel + next_state.vert_vel) * 0.5f * next_state.dt;

		// advance the index
		index = (index + 1) % size;
	}

	// update output state to corrected values
	_output_vert_new = _output_vert_buffer.get_newest();

	// reset time delta to zero for the next accumulation of full rate IMU data
	_output_vert_new.dt = 0.0f;
}

void OutputPredictor::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
{
	// loop through the output filter state history and apply the corrections to the velocity and position states
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		// a constant velocity correction is applied
		_output_buffer[index].vel += vel_correction;

		// a constant position correction is applied
		_output_buffer[index].pos += pos_correction;
	}

	// update output state to corrected values
	_output_new = _output_buffer.get_newest();
}

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

//#define OUTPUT_PREDICTOR_DEBUG_VEROSE

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

	_accel_bias.zero();
	_gyro_bias.zero();

	_time_last_correct_states_us = 0;

	_output_new = {};

	_R_to_earth_now.setIdentity();
	_vel_imu_rel_body_ned.zero();
	_vel_deriv.zero();

	_delta_angle_corr.zero();
	_vel_err_integ.zero();
	_pos_err_integ.zero();

	_output_tracking_error.zero();

	_reset_quaternion = true;
	_reset_velocity_xy = true;
	_reset_velocity_z = true;
	_reset_position_xy = true;
	_reset_position_z = true;

	_output_filter_aligned = false;
}

void OutputPredictor::resetQuaternionTo(const uint64_t &time_delayed_us, const Quatf &new_quat)
{
	// find the output observer state corresponding to the reset time
	const outputSample &delayed_sample = findOutputSample(time_delayed_us);

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	const outputSample delayed_orig = delayed_sample;
	const outputSample newest_orig = _output_new;
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE

	const Quatf quat_change = (new_quat * delayed_sample.quat_nominal.inversed()).normalized();

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = (quat_change * _output_buffer[i].quat_nominal).normalized();
	}

	// apply change to latest
	_output_new.quat_nominal = (quat_change * _output_new.quat_nominal).normalized();
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);



	_delta_angle_corr.zero();
	_output_tracking_error(0) = 0.f;

	_reset_quaternion = false;


#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	printf("output predictor: reset q delayed (%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f)\n",
	       (double)matrix::Eulerf(delayed_orig.quat_nominal).phi(), (double)matrix::Eulerf(delayed_orig.quat_nominal).theta(),
	       (double)matrix::Eulerf(delayed_orig.quat_nominal).psi(),
	       (double)matrix::Eulerf(delayed_sample.quat_nominal).phi(), (double)matrix::Eulerf(delayed_sample.quat_nominal).theta(),
	       (double)matrix::Eulerf(delayed_sample.quat_nominal).psi()
	      );

	printf("output predictor: reset q newest (%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f)\n",
	       (double)matrix::Eulerf(newest_orig.quat_nominal).phi(), (double)matrix::Eulerf(newest_orig.quat_nominal).theta(),
	       (double)matrix::Eulerf(newest_orig.quat_nominal).psi(),
	       (double)matrix::Eulerf(_output_new.quat_nominal).phi(), (double)matrix::Eulerf(_output_new.quat_nominal).theta(),
	       (double)matrix::Eulerf(_output_new.quat_nominal).psi()
	      );
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE
}

void OutputPredictor::resetHorizontalVelocityTo(const uint64_t &time_delayed_us, const Vector2f &new_horz_vel)
{
	// find the output observer state corresponding to the reset time
	const outputSample &delayed_sample = findOutputSample(time_delayed_us);

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	const outputSample delayed_orig = delayed_sample;
	const outputSample newest_orig = _output_new;
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE

	const Vector2f delta_vxy = new_horz_vel - delayed_sample.vel.xy(); // horizontal velocity

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].vel.xy() += delta_vxy;
	}

	// apply change to latest velocity
	_output_new.vel.xy() += delta_vxy;



	_vel_err_integ(0) = 0.f;
	_vel_err_integ(1) = 0.f;
	_reset_velocity_xy = false;

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	printf("output predictor: reset v_xy delayed (%.1f,%.1f)->(%.1f,%.1f)\n",
	       (double)delayed_orig.vel(0), (double)delayed_orig.vel(1),
	       (double)delayed_sample.vel(0), (double)delayed_sample.vel(1)
	      );

	printf("output predictor: reset v_xy newest (%.1f,%.1f)->(%.1f,%.1f)\n",
	       (double)newest_orig.vel(0), (double)newest_orig.vel(1),
	       (double)_output_new.vel(0), (double)_output_new.vel(1)
	      );
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE
}

void OutputPredictor::resetVerticalVelocityTo(const uint64_t &time_delayed_us, const float new_vert_vel)
{
	// find the output observer state corresponding to the reset time
	const outputSample &delayed_sample = findOutputSample(time_delayed_us);

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	const outputSample delayed_orig = delayed_sample;
	const outputSample newest_orig = _output_new;
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE

	const float delta_vz = new_vert_vel - delayed_sample.vel(2); // vertical velocity
	const float delta_vz_alt = new_vert_vel - delayed_sample.vert_vel_alt; // vertical velocity (alternative)

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].vel(2) += delta_vz;
		_output_buffer[i].vert_vel_alt += delta_vz_alt;
	}

	// apply change to latest
	_output_new.vel(2) += delta_vz;
	_output_new.vert_vel_alt += delta_vz_alt;


	_vel_err_integ(2) = 0.f;
	_reset_velocity_z = false;

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	printf("output predictor: reset v_z delayed %.1f->%.1f\n", (double)delayed_orig.vel(2), (double)delayed_sample.vel(2));

	printf("output predictor: reset v_z newest %.1f->%.1f\n", (double)newest_orig.vel(2), (double)_output_new.vel(2));
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE
}

void OutputPredictor::resetHorizontalPositionTo(const uint64_t &time_delayed_us, const Vector2f &new_horz_pos)
{
	// find the output observer state corresponding to the reset time
	const outputSample &delayed_sample = findOutputSample(time_delayed_us);

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	const outputSample delayed_orig = delayed_sample;
	const outputSample newest_orig = _output_new;
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE

	const Vector2f delta_xy = new_horz_pos - delayed_sample.pos.xy(); // horizontal position

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos.xy() += delta_xy;
	}

	// apply change to latest
	_output_new.pos.xy() += delta_xy;


	_pos_err_integ(0) = 0.f;
	_pos_err_integ(1) = 0.f;
	_reset_position_xy = false;

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	printf("output predictor: reset xy delayed (%.1f,%.1f)->(%.1f,%.1f)\n",
	       (double)delayed_orig.pos(0), (double)delayed_orig.pos(1),
	       (double)delayed_sample.pos(0), (double)delayed_sample.pos(1)
	      );

	printf("output predictor: reset xy newest (%.1f,%.1f)->(%.1f,%.1f)\n",
	       (double)newest_orig.pos(0), (double)newest_orig.pos(1),
	       (double)_output_new.pos(0), (double)_output_new.pos(1)
	      );
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE
}

void OutputPredictor::resetVerticalPositionTo(const uint64_t &time_delayed_us, const float new_vert_pos)
{
	// find the output observer state corresponding to the reset time
	const outputSample &delayed_sample = findOutputSample(time_delayed_us);

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	const outputSample delayed_orig = delayed_sample;
	const outputSample newest_orig = _output_new;

	if (time_delayed_us != delayed_sample.time_us) {
		printf("output predictor: BAD RESET TIME!\n");
	}

#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE

	const float delta_z = new_vert_pos - delayed_sample.pos(2); // vertical position
	const float delta_z_alt = new_vert_pos - delayed_sample.vert_vel_integ; // vertical position (alternative)

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos(2) += delta_z;
		_output_buffer[i].vert_vel_integ += delta_z_alt;
	}

	// apply change to latest
	_output_new.pos(2) += delta_z;
	_output_new.vert_vel_integ += delta_z_alt;


	_pos_err_integ(2) = 0.f;
	_reset_position_z = false;

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
	printf("output predictor: reset z, delayed (%llu):%.1f->%.1f, newest (%llu):%.1f->%.1f\n",
	       time_delayed_us, (double)delayed_orig.pos(2), (double)delayed_sample.pos(2),
	       _output_new.time_us, (double)newest_orig.pos(2), (double)_output_new.pos(2)
	      );

	//printf("output predictor: %llu reset z newest %.1f->%.1f\n", _output_new.time_us, (double)newest_orig.pos(2), (double)_output_new.pos(2));
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE
}

bool OutputPredictor::calculateOutputStates(const uint64_t time_us, const Vector3f &delta_angle,
		const float delta_angle_dt, const Vector3f &delta_velocity, const float delta_velocity_dt)
{
	if (time_us <= _output_new.time_us) {
		reset();
		return false;
	}

	// Use full rate IMU data at the current time horizon
	if (_output_new.time_us != 0) {
		const float dt = math::constrain((time_us - _output_new.time_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_update_states_avg = 0.8f * _dt_update_states_avg + 0.2f * dt;
	}

	_output_new.time_us = time_us;

	// correct delta angle and delta velocity for bias offsets
	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle_bias_scaled = _gyro_bias * delta_angle_dt;
	const Vector3f delta_angle_corrected(delta_angle - delta_angle_bias_scaled + _delta_angle_corr);

	const Vector3f delta_vel_bias_scaled = _accel_bias * delta_velocity_dt;
	const Vector3f delta_velocity_corrected(delta_velocity - delta_vel_bias_scaled);

	const Quatf dq(AxisAnglef{delta_angle_corrected});

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.quat_nominal = (_output_new.quat_nominal * dq).normalized();
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
	_output_new.vert_vel_alt += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (delta_velocity_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_new.vert_vel_integ += delta_pos_NED(2);

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

bool OutputPredictor::correctOutputStates(const uint64_t &time_delayed_us,
		const Quatf &quat_state, const Vector3f &vel_state, const Vector3f &pos_state,
		const matrix::Vector3f &gyro_bias, const matrix::Vector3f &accel_bias)
{
	const uint64_t time_latest_us = _output_new.time_us;

	if (time_latest_us < time_delayed_us) {
		return false;
	}

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	if ((_output_new.time_us != 0) && (_output_new.dt > 0.f)) {
		_output_buffer.push(_output_new);
		_output_new.dt = 0.f; // reset time delta to zero for the next accumulation of full rate IMU data
	}

	// store IMU bias for calculateOutputStates
	_gyro_bias = gyro_bias;
	_accel_bias = accel_bias;

	// calculate an average filter update time
	if ((_time_last_correct_states_us != 0) && (time_delayed_us > _time_last_correct_states_us)) {
		const float dt = math::constrain((time_delayed_us - _time_last_correct_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_correct_states_avg = 0.8f * _dt_correct_states_avg + 0.2f * dt;
	}

	_time_last_correct_states_us = time_delayed_us;

	// get the output buffer state data at the EKF fusion time horizon
	const outputSample &output_delayed = findOutputSample(time_delayed_us);
	const bool delayed_index_found = (output_delayed.time_us == time_delayed_us);

	if (!_output_filter_aligned) {
		if (delayed_index_found) {
			resetQuaternionTo(time_delayed_us, quat_state);
			resetHorizontalVelocityTo(time_delayed_us, vel_state.xy());
			resetVerticalVelocityTo(time_delayed_us, vel_state(2));
			resetHorizontalPositionTo(time_delayed_us, pos_state.xy());
			resetVerticalPositionTo(time_delayed_us, pos_state(2));

			_output_filter_aligned = true;

#if defined(OUTPUT_PREDICTOR_DEBUG_VEROSE)
			printf("output predictor: ALIGNED, IMU dt: %.4f, EKF dt: %.4f\n",
			       (double)_dt_update_states_avg, (double)_dt_correct_states_avg);
#endif // OUTPUT_PREDICTOR_DEBUG_VEROSE
		}

		return false;
	}

	if (_reset_quaternion) {
		resetQuaternionTo(time_delayed_us, quat_state);

		_output_tracking_error(0) = 0.f;

	} else {
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
	}


	// velocity
	if (_reset_velocity_xy) {
		resetHorizontalVelocityTo(time_delayed_us, vel_state.xy());
	}

	if (_reset_velocity_z) {
		resetVerticalVelocityTo(time_delayed_us, vel_state(2));
	}

	// calculate a velocity correction and apply it to the output state history
	const Vector3f vel_err(vel_state - output_delayed.vel);
	_vel_err_integ += vel_err;
	const float vel_gain = _dt_correct_states_avg / math::constrain(_vel_tau, _dt_correct_states_avg, 10.f); // complementary filter gains
	const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].vel += vel_correction;
	}

	_output_tracking_error(1) = vel_err.norm(); // logging


	// position
	if (_reset_position_xy) {
		resetHorizontalPositionTo(time_delayed_us, pos_state.xy());
	}

	if (_reset_position_z) {
		resetVerticalPositionTo(time_delayed_us, pos_state(2));
	}

	// calculate a position correction and apply it  to the output state history
	const Vector3f pos_err(pos_state - output_delayed.pos);
	_pos_err_integ += pos_err;
	const float pos_gain = _dt_correct_states_avg / math::constrain(_pos_tau, _dt_correct_states_avg, 10.f); // complementary filter gains
	const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos += pos_correction;
	}

	_output_tracking_error(2) = pos_err.norm(); // logging


	// vertical position alternate
	// calculate down velocity and position tracking errors
	const float vert_vel_err = (vel_state(2) - output_delayed.vert_vel_alt);
	const float vert_vel_integ_err = (pos_state(2) - output_delayed.vert_vel_integ);

	// calculate a velocity correction that will be applied to the output state history
	// using a PD feedback tuned to a 5% overshoot
	const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

	applyCorrectionToVerticalOutputBuffer(vert_vel_correction);



	// update output state to corrected values and
	// reset time delta to zero for the next accumulation of full rate IMU data
	_output_new = _output_buffer.get_newest();
	_output_new.dt = 0.0f;

	return true;
}

void OutputPredictor::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
{
	// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
	// vert_vel states and propagate vert_vel_integ forward using the corrected vert_vel
	uint8_t index = _output_buffer.get_oldest_index();

	const uint8_t size = _output_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputSample &current_state = _output_buffer[index];
		outputSample &next_state = _output_buffer[index_next];

		// correct the velocity
		if (counter == 0) {
			current_state.vert_vel_alt += vert_vel_correction;
		}

		next_state.vert_vel_alt += vert_vel_correction;

		// position is propagated forward using the corrected velocity and a trapezoidal integrator
		next_state.vert_vel_integ = current_state.vert_vel_integ
					    + (current_state.vert_vel_alt + next_state.vert_vel_alt) * 0.5f * next_state.dt;

		// advance the index
		index = (index + 1) % size;
	}
}

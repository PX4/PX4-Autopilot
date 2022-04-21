/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	reset();
	return ret;
}

void Ekf::reset()
{
	_state.vel.setZero();
	_state.pos.setZero();
	_state.delta_ang_bias.setZero();
	_state.delta_vel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setIdentity();

	// TODO: who resets the output buffer content?
	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal.setIdentity();

	_delta_angle_corr.setZero();

	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_ang_rate_delayed_raw.zero();

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_prev_dvel_bias_var.zero();

	resetGpsDriftCheckFilters();
}

bool Ekf::update()
{
	bool updated = false;

	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		// perform state and covariance prediction for the main filter
		predictState();
		predictCovariance();

		// control fusion of observation data
		controlFusionModes();

		// run a separate filter for terrain estimation
		runTerrainEstimator();

		updated = true;
	}

	// the output observer always runs
	// Use full rate IMU data at the current time horizon
	calculateOutputStates(_newest_high_rate_imu_sample);

	return updated;
}

bool Ekf::initialiseFilter()
{
	// Filter accel for tilt initialization
	const imuSample &imu_init = _imu_buffer.get_newest();

	// protect against zero data
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	// Sum the magnetometer measurements
	if (_mag_buffer) {
		magSample mag_sample;

		if (_mag_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &mag_sample)) {
			if (mag_sample.time_us != 0) {
				if (_mag_counter == 0) {
					_mag_lpf.reset(mag_sample.mag);

				} else {
					_mag_lpf.update(mag_sample.mag);
				}

				_mag_counter++;
			}
		}
	}

	// accumulate enough height measurements to be confident in the quality of the data
	if (_baro_buffer && _baro_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
		if (_baro_sample_delayed.time_us != 0) {
			if (_baro_counter == 0) {
				_baro_hgt_offset = _baro_sample_delayed.hgt;

			} else {
				_baro_hgt_offset = 0.9f * _baro_hgt_offset + 0.1f * _baro_sample_delayed.hgt;
			}

			_baro_counter++;
		}
	}

	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		if (_mag_counter < _obs_buffer_length) {
			// not enough mag samples accumulated
			return false;
		}
	}

	if (_baro_counter < _obs_buffer_length) {
		// not enough baro samples accumulated
		return false;
	}

	// we use baro height initially and switch to GPS/range/EV finder later when it passes checks.
	setControlBaroHeight();

	if (!initialiseTilt()) {
		return false;
	}

	// calculate the initial magnetic field and yaw alignment
	// but do not mark the yaw alignement complete as it needs to be
	// reset once the leveling phase is done
	resetMagHeading(false, false);

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

	// update the yaw angle variance using the variance of the measurement
	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		// using magnetic heading tuning parameter
		increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
	}

	// Initialise the terrain estimator
	initHagl();

	// reset the essential fusion timeout counters
	_time_last_hgt_fuse = _time_last_imu;
	_time_last_hor_pos_fuse = _time_last_imu;
	_time_last_hor_vel_fuse = _time_last_imu;
	_time_last_hagl_fuse = _time_last_imu;
	_time_last_flow_terrain_fuse = _time_last_imu;
	_time_last_of_fuse = _time_last_imu;

	// reset the output predictor state history to match the EKF initial values
	alignOutputFilter();

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
	const Vector3f gravity_in_body = _accel_lpf.getState().normalized();
	const float pitch = asinf(gravity_in_body(0));
	const float roll = atan2f(-gravity_in_body(1), -gravity_in_body(2));

	_state.quat_nominal = Quatf{Eulerf{roll, pitch, 0.0f}};
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState()
{
	// apply imu bias corrections
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _state.delta_ang_bias;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f corrected_delta_vel = _imu_sample_delayed.delta_vel - _state.delta_vel_bias;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	const float alpha = 1.0f - _imu_sample_delayed.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f * (_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	const float filter_update_s = 1e-6f * _params.filter_update_interval_us;
	input = math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	// some calculations elsewhere in code require a raw angular rate vector so calculate here to avoid duplication
	// protect against possible small timesteps resulting from timing slip on previous frame that can drive spikes into the rate
	// due to insufficient averaging
	if (_imu_sample_delayed.delta_ang_dt > 0.25f * _dt_ekf_avg) {
		_ang_rate_delayed_raw = _imu_sample_delayed.delta_ang / _imu_sample_delayed.delta_ang_dt;
	}
}

/*
 * Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
 * Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
 * Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
 * current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian:
 * “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void Ekf::calculateOutputStates(const imuSample &imu)
{
	// Use full rate IMU data at the current time horizon

	// correct delta angles for bias offsets
	const float dt_scale_correction = _dt_imu_avg / _dt_ekf_avg;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle(imu.delta_ang - _state.delta_ang_bias * dt_scale_correction + _delta_angle_corr);

	// calculate a yaw change about the earth frame vertical
	const float spin_del_ang_D = delta_angle.dot(Vector3f(_R_to_earth_now.row(2)));
	_yaw_delta_ef += spin_del_ang_D;

	// Calculate filtered yaw rate to be used by the magnetometer fusion type selection logic
	// Note fixed coefficients are used to save operations. The exact time constant is not important.
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / imu.delta_ang_dt;


	_output_new.time_us = imu.time_us;
	_output_vert_new.time_us = imu.time_us;

	const Quatf dq(AxisAnglef{delta_angle});

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// the quaternions must always be normalised after modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

	// correct delta velocity for bias offsets
	const Vector3f delta_vel_body{imu.delta_vel - _state.delta_vel_bias * dt_scale_correction};

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_earth{_R_to_earth_now * delta_vel_body};

	// correct for measured acceleration due to gravity
	delta_vel_earth(2) += CONSTANTS_ONE_G * imu.delta_vel_dt;

	// calculate the earth frame velocity derivatives
	if (imu.delta_vel_dt > 1e-4f) {
		_vel_deriv = delta_vel_earth * (1.0f / imu.delta_vel_dt);
	}

	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last(_output_new.vel);

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.vel += delta_vel_earth;
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (imu.delta_vel_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// accumulate the time for each update
	_output_vert_new.dt += imu.delta_vel_dt;

	// correct velocity for IMU offset
	if (imu.delta_ang_dt > 1e-4f) {
		// calculate the average angular rate across the last IMU update
		const Vector3f ang_rate = imu.delta_ang * (1.0f / imu.delta_ang_dt);

		// calculate the velocity of the IMU relative to the body origin
		const Vector3f vel_imu_rel_body = ang_rate % _params.imu_pos_body;

		// rotate the relative velocity into earth frame
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	if (_imu_updated) {
		_output_buffer.push(_output_new);
		_output_vert_buffer.push(_output_vert_new);

		// get the oldest INS state data from the ring buffer
		// this data will be at the EKF fusion time horizon
		// TODO: there is no guarantee that data is at delayed fusion horizon
		//       Shouldnt we use pop_first_older_than?
		const outputSample &output_delayed = _output_buffer.get_oldest();
		const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		const Quatf q_error((_state.quat_nominal.inversed() * output_delayed.quat_nominal).normalized());

		// convert the quaternion delta to a delta angle
		const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

		const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		const float time_delay = fmaxf((imu.time_us - _imu_sample_delayed.time_us) * 1e-6f, _dt_imu_avg);
		const float att_gain = 0.5f * _dt_imu_avg / time_delay;

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
		const float vel_gain = _dt_ekf_avg / math::constrain(_params.vel_Tau, _dt_ekf_avg, 10.0f);
		const float pos_gain = _dt_ekf_avg / math::constrain(_params.pos_Tau, _dt_ekf_avg, 10.0f);

		// calculate down velocity and position tracking errors
		const float vert_vel_err = (_state.vel(2) - output_vert_delayed.vert_vel);
		const float vert_vel_integ_err = (_state.pos(2) - output_vert_delayed.vert_vel_integ);

		// calculate a velocity correction that will be applied to the output state history
		// using a PD feedback tuned to a 5% overshoot
		const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

		applyCorrectionToVerticalOutputBuffer(vert_vel_correction);

		// calculate velocity and position tracking errors
		const Vector3f vel_err(_state.vel - output_delayed.vel);
		const Vector3f pos_err(_state.pos - output_delayed.pos);

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
}

/*
* Calculate a correction to be applied to vert_vel that casues vert_vel_integ to track the EKF
* down position state at the fusion time horizon using an alternative algorithm to what
* is used for the vel and pos state tracking. The algorithm applies a correction to the vert_vel
* state history and propagates vert_vel_integ forward in time using the corrected vert_vel history.
* This provides an alternative vertical velocity output that is closer to the first derivative
* of the position but does degrade tracking relative to the EKF state.
*/
void Ekf::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
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

/*
* Calculate corrections to be applied to vel and pos output state history.
* The vel and pos state history are corrected individually so they track the EKF states at
* the fusion time horizon. This option provides the most accurate tracking of EKF states.
*/
void Ekf::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
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

/*
 * Predict the previous quaternion output state forward using the latest IMU delta angle data.
*/
Quatf Ekf::calculate_quaternion() const
{
	// Correct delta angle data for bias errors using bias state estimates from the EKF and also apply
	// corrections required to track the EKF quaternion states
	const Vector3f delta_angle{_newest_high_rate_imu_sample.delta_ang - _state.delta_ang_bias * (_dt_imu_avg / _dt_ekf_avg) + _delta_angle_corr};

	// increment the quaternions using the corrected delta angle vector
	// the quaternions must always be normalised after modification
	return Quatf{_output_new.quat_nominal * AxisAnglef{delta_angle}}.unit();
}

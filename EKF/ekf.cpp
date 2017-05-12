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

#include "../ecl.h"
#include "ekf.h"
#include "mathlib.h"

#ifndef __PX4_QURT
#if defined(__cplusplus) && !defined(__PX4_NUTTX)
#include <cmath>
#define ISFINITE(x) std::isfinite(x)
#else
#define ISFINITE(x) isfinite(x)
#endif
#endif

#if defined(__PX4_QURT)
// Missing math.h defines
#define ISFINITE(x) __builtin_isfinite(x)
#endif

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.accel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setZero();
	_state.quat_nominal(0) = 1.0f;

	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal.setZero();

	_delta_angle_corr.setZero();
	_imu_down_sampled.delta_ang.setZero();
	_imu_down_sampled.delta_vel.setZero();
	_imu_down_sampled.delta_ang_dt = 0.0f;
	_imu_down_sampled.delta_vel_dt = 0.0f;
	_imu_down_sampled.time_us = timestamp;

	_q_down_sampled(0) = 1.0f;
	_q_down_sampled(1) = 0.0f;
	_q_down_sampled(2) = 0.0f;
	_q_down_sampled(3) = 0.0f;

	_imu_updated = false;
	_NED_origin_initialised = false;
	_gps_speed_valid = false;

	_filter_initialised = false;
	_terrain_initialised = false;
	_sin_tilt_rng = sinf(_params.rng_sens_pitch);
	_cos_tilt_rng = cosf(_params.rng_sens_pitch);

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_dt_ekf_avg = 0.001f * (float)(FILTER_UPDATE_PERIOD_MS);

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_accel_mag_filt = 0.0f;
	_ang_rate_mag_filt = 0.0f;
	_prev_dvel_bias_var.zero();

	return ret;
}

bool Ekf::update()
{
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

	}

	// the output observer always runs
	calculateOutputStates();

	// check for NaN or inf on attitude states
	if (!ISFINITE(_state.quat_nominal(0)) || !ISFINITE(_output_new.quat_nominal(0))) {
		return false;
	}

	// We don't have valid data to output until tilt and yaw alignment is complete
	return _control_status.flags.tilt_align && _control_status.flags.yaw_align;
}

bool Ekf::initialiseFilter()
{
	// Keep accumulating measurements until we have a minimum of 10 samples for the required sensors

	// Sum the IMU delta angle measurements
	imuSample imu_init = _imu_buffer.get_newest();
	_delVel_sum += imu_init.delta_vel;

	// Sum the magnetometer measurements
	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		if ((_mag_counter == 0) && (_mag_sample_delayed.time_us != 0)) {
			// initialise the counter when we start getting data from the buffer
			_mag_counter = 1;

		} else if ((_mag_counter != 0) && (_mag_sample_delayed.time_us != 0)) {
			// increment the sample count and apply a LPF to the measurement
			_mag_counter ++;

			// don't start using data until we can be certain all bad initial data has been flushed
			if (_mag_counter == (uint8_t)(_obs_buffer_length + 1)) {
				// initialise filter states
				_mag_filt_state = _mag_sample_delayed.mag;

			} else if (_mag_counter > (uint8_t)(_obs_buffer_length + 1)) {
				// noise filter the data
				_mag_filt_state = _mag_filt_state * 0.9f + _mag_sample_delayed.mag * 0.1f;
			}
		}
	}

	// Count the number of external vision measurements received
	if (_ext_vision_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_ev_sample_delayed)) {
		if ((_ev_counter == 0) && (_ev_sample_delayed.time_us != 0)) {
			// initialise the counter
			_ev_counter = 1;

			// set the height fusion mode to use external vision data when we start getting valid data from the buffer
			if (_primary_hgt_source == VDIST_SENSOR_EV) {
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = true;
			}

		} else if ((_ev_counter != 0) && (_ev_sample_delayed.time_us != 0)) {
			// increment the sample count
			_ev_counter ++;
		}
	}

	// set the default height source from the adjustable parameter
	if (_hgt_counter == 0) {
		_primary_hgt_source = _params.vdist_sensor_type;
	}

	// accumulate enough height measurements to be confident in the qulaity of the data
	if (_primary_hgt_source == VDIST_SENSOR_RANGE) {
		if (_range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)) {
			if ((_hgt_counter == 0) && (_range_sample_delayed.time_us != 0)) {
				// initialise the counter height fusion method when we start getting data from the buffer
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = true;
				_control_status.flags.ev_hgt = false;
				_hgt_counter = 1;

			} else if ((_hgt_counter != 0) && (_range_sample_delayed.time_us != 0)) {
				// increment the sample count and apply a LPF to the measurement
				_hgt_counter ++;

				// don't start using data until we can be certain all bad initial data has been flushed
				if (_hgt_counter == (uint8_t)(_obs_buffer_length + 1)) {
					// initialise filter states
					_rng_filt_state = _range_sample_delayed.rng;

				} else if (_hgt_counter > (uint8_t)(_obs_buffer_length + 1)) {
					// noise filter the data
					_rng_filt_state = 0.9f * _rng_filt_state + 0.1f * _range_sample_delayed.rng;
				}
			}
		}

	} else if (_primary_hgt_source == VDIST_SENSOR_BARO || _primary_hgt_source == VDIST_SENSOR_GPS) {
		// if the user parameter specifies use of GPS for height we use baro height initially and switch to GPS
		// later when it passes checks.
		if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
			if ((_hgt_counter == 0) && (_baro_sample_delayed.time_us != 0)) {
				// initialise the counter and height fusion method when we start getting data from the buffer
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_hgt_counter = 1;

			} else if ((_hgt_counter != 0) && (_baro_sample_delayed.time_us != 0)) {
				// increment the sample count and apply a LPF to the measurement
				_hgt_counter ++;

				// don't start using data until we can be certain all bad initial data has been flushed
				if (_hgt_counter == (uint8_t)(_obs_buffer_length + 1)) {
					// initialise filter states
					_baro_hgt_offset = _baro_sample_delayed.hgt;

				} else if (_hgt_counter > (uint8_t)(_obs_buffer_length + 1)) {
					// noise filter the data
					_baro_hgt_offset = 0.9f * _baro_hgt_offset + 0.1f * _baro_sample_delayed.hgt;
				}
			}
		}

	} else if (_primary_hgt_source == VDIST_SENSOR_EV) {
		_hgt_counter = _ev_counter;

	} else {
		return false;
	}

	// check to see if we have enough measurements and return false if not
	bool hgt_count_fail = _hgt_counter <= 2 * _obs_buffer_length;
	bool mag_count_fail = _mag_counter <= 2 * _obs_buffer_length;
	bool ev_count_fail = ((_params.fusion_mode & MASK_USE_EVPOS) || (_params.fusion_mode & MASK_USE_EVYAW)) && (_ev_counter <= 2 * _obs_buffer_length);

	if (hgt_count_fail || mag_count_fail || ev_count_fail) {
		return false;

	} else {
		// reset variables that are shared with post alignment GPS checks
		_gps_drift_velD = 0.0f;
		_gps_alt_ref = 0.0f;

		// Zero all of the states
		_state.vel.setZero();
		_state.pos.setZero();
		_state.gyro_bias.setZero();
		_state.accel_bias.setZero();
		_state.mag_I.setZero();
		_state.mag_B.setZero();
		_state.wind_vel.setZero();

		// get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
		float pitch = 0.0f;
		float roll = 0.0f;

		if (_delVel_sum.norm() > 0.001f) {
			_delVel_sum.normalize();
			pitch = asinf(_delVel_sum(0));
			roll = atan2f(-_delVel_sum(1), -_delVel_sum(2));

		} else {
			return false;
		}

		// calculate initial tilt alignment
		matrix::Euler<float> euler_init(roll, pitch, 0.0f);
		_state.quat_nominal = Quaternion(euler_init);
		_output_new.quat_nominal = _state.quat_nominal;

		// update transformation matrix from body to world frame
		_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

		// calculate the averaged magnetometer reading
		Vector3f mag_init = _mag_filt_state;

		// calculate the initial magnetic field and yaw alignment
		_control_status.flags.yaw_align = resetMagHeading(mag_init);

		if (_control_status.flags.rng_hgt) {
			// if we are using the range finder as the primary source, then calculate the baro height at origin so  we can use baro as a backup
			// so it can be used as a backup ad set the initial height using the range finder
			baroSample baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt;
			_state.pos(2) = -math::max(_rng_filt_state * _R_rng_to_earth_2_2, _params.rng_gnd_clearance);
			ECL_INFO("EKF using range finder height - commencing alignment");

		} else if (_control_status.flags.ev_hgt) {
			// if we are using external vision data for height, then the vertical position state needs to be reset
			// because the initialisation position is not the zero datum
			resetHeight();

		}

		// initialise the state covariance matrix
		initialiseCovariance();

		// try to initialise the terrain estimator
		_terrain_initialised = initHagl();

		// reset the essential fusion timeout counters
		_time_last_hgt_fuse = _time_last_imu;
		_time_last_pos_fuse = _time_last_imu;
		_time_last_vel_fuse = _time_last_imu;
		_time_last_hagl_fuse = _time_last_imu;
		_time_last_of_fuse = _time_last_imu;

		// reset the output predictor state history to match the EKF initial values
		alignOutputFilter();

		return true;
	}
}

void Ekf::predictState()
{
	if (!_earth_rate_initialised) {
		if (_NED_origin_initialised) {
			calcEarthRateNED(_earth_rate_NED, _pos_ref.lat_rad);
			_earth_rate_initialised = true;
		}
	}

	// apply imu bias corrections
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _state.gyro_bias;
	Vector3f corrected_delta_vel = _imu_sample_delayed.delta_vel - _state.accel_bias;

	// correct delta angles for earth rotation rate
	corrected_delta_ang -= -_R_to_earth.transpose() * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;

	// convert the delta angle to a delta quaternion
	Quaternion dq;
	dq.from_axis_angle(corrected_delta_ang);

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = dq * _state.quat_nominal;

	// quaternions must be normalised whenever they are modified
	_state.quat_nominal.normalize();

	// save the previous value of velocity so we can use trapzoidal integration
	Vector3f vel_last = _state.vel;

	// update transformation matrix from body to world frame
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

	// calculate the increment in velocity using the current orientation
	_state.vel += _R_to_earth * corrected_delta_vel;

	// compensate for acceleration due to gravity
	_state.vel(2) += _gravity_mss * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f * (_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	input = math::constrain(input, 0.0005f * (float)(FILTER_UPDATE_PERIOD_MS), 0.002f * (float)(FILTER_UPDATE_PERIOD_MS));
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;
}

bool Ekf::collect_imu(imuSample &imu)
{
	// accumulate and downsample IMU data across a period FILTER_UPDATE_PERIOD_MS long

	// copy imu data to local variables
	_imu_sample_new.delta_ang	= imu.delta_ang;
	_imu_sample_new.delta_vel	= imu.delta_vel;
	_imu_sample_new.delta_ang_dt	= imu.delta_ang_dt;
	_imu_sample_new.delta_vel_dt	= imu.delta_vel_dt;
	_imu_sample_new.time_us		= imu.time_us;

	// accumulate the time deltas
	_imu_down_sampled.delta_ang_dt += imu.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu.delta_vel_dt;

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	Quaternion delta_q;
	delta_q.rotate(imu.delta_ang);
	_q_down_sampled =  _q_down_sampled * delta_q;
	_q_down_sampled.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	matrix::Dcm<float> delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (_imu_sample_new.delta_vel + delta_R * _imu_sample_new.delta_vel) * 0.5f;

	// if the target time delta between filter prediction steps has been exceeded
	// write the accumulated IMU data to the ring buffer
	float target_dt = (float)(FILTER_UPDATE_PERIOD_MS) / 1000;

	if (_imu_down_sampled.delta_ang_dt >= target_dt - _imu_collection_time_adj) {

		// accumulate the amount of time to advance the IMU collection time so that we meet the
		// average EKF update rate requirement
		_imu_collection_time_adj += 0.01f * (_imu_down_sampled.delta_ang_dt - target_dt);
		_imu_collection_time_adj = math::constrain(_imu_collection_time_adj, -0.5f * target_dt, 0.5f * target_dt);

		imu.delta_ang     = _q_down_sampled.to_axis_angle();
		imu.delta_vel     = _imu_down_sampled.delta_vel;
		imu.delta_ang_dt  = _imu_down_sampled.delta_ang_dt;
		imu.delta_vel_dt  = _imu_down_sampled.delta_vel_dt;

		_imu_down_sampled.delta_ang.setZero();
		_imu_down_sampled.delta_vel.setZero();
		_imu_down_sampled.delta_ang_dt = 0.0f;
		_imu_down_sampled.delta_vel_dt = 0.0f;
		_q_down_sampled(0) = 1.0f;
		_q_down_sampled(1) = _q_down_sampled(2) = _q_down_sampled(3) = 0.0f;

		return true;
	}

	return false;
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
void Ekf::calculateOutputStates()
{
	// use latest IMU data
	imuSample imu_new = _imu_sample_new;

	// correct delta angles for bias offsets and scale factors
	Vector3f delta_angle;
	float dt_scale_correction = _dt_imu_avg / _dt_ekf_avg;
	delta_angle(0) = _imu_sample_new.delta_ang(0) - _state.gyro_bias(0) * dt_scale_correction;
	delta_angle(1) = _imu_sample_new.delta_ang(1) - _state.gyro_bias(1) * dt_scale_correction;
	delta_angle(2) = _imu_sample_new.delta_ang(2) - _state.gyro_bias(2) * dt_scale_correction;

	// correct delta velocity for bias offsets
	Vector3f delta_vel = _imu_sample_new.delta_vel - _state.accel_bias * dt_scale_correction;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	delta_angle += _delta_angle_corr;

	// convert the delta angle to an equivalent delta quaternions
	Quaternion dq;
	dq.from_axis_angle(delta_angle);

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.time_us = imu_new.time_us;
	_output_new.quat_nominal = dq * _output_new.quat_nominal;

	// the quaternions must always be normalised afer modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = quat_to_invrotmat(_output_new.quat_nominal);

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_NED = _R_to_earth_now * delta_vel;

	// corrrect for measured accceleration due to gravity
	delta_vel_NED(2) += _gravity_mss * imu_new.delta_vel_dt;

	// save the previous velocity so we can use trapezidal integration
	Vector3f vel_last = _output_new.vel;

	// increment the INS velocity states by the measurement plus corrections
	_output_new.vel += delta_vel_NED;

	// use trapezoidal integration to calculate the INS position states
	_output_new.pos += (_output_new.vel + vel_last) * (imu_new.delta_vel_dt * 0.5f);

	// store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
	if (_imu_updated) {
		_output_buffer.push(_output_new);
		_imu_updated = false;

		// get the oldest INS state data from the ring buffer
		// this data will be at the EKF fusion time horizon
		_output_sample_delayed = _output_buffer.get_oldest();

		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		Quaternion quat_inv = _state.quat_nominal.inversed();
		Quaternion q_error =  _output_sample_delayed.quat_nominal * quat_inv;
		q_error.normalize();

		// convert the quaternion delta to a delta angle
		Vector3f delta_ang_error;
		float scalar;

		if (q_error(0) >= 0.0f) {
			scalar = -2.0f;

		} else {
			scalar = 2.0f;
		}

		delta_ang_error(0) = scalar * q_error(1);
		delta_ang_error(1) = scalar * q_error(2);
		delta_ang_error(2) = scalar * q_error(3);

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		float time_delay = 1e-6f * (float)(_imu_sample_new.time_us - _imu_sample_delayed.time_us);
		time_delay = fmaxf(time_delay, _dt_imu_avg);
		float att_gain = 0.5f * _dt_imu_avg / time_delay;

		// calculate a corrrection to the delta angle
		// that will cause the INS to track the EKF quaternions
		_delta_angle_corr = delta_ang_error * att_gain;

		// calculate velocity and position tracking errors
		Vector3f vel_err = (_state.vel - _output_sample_delayed.vel);
		Vector3f pos_err = (_state.pos - _output_sample_delayed.pos);

		// collect magnitude tracking error for diagnostics
		_output_tracking_error[0] = delta_ang_error.norm();
		_output_tracking_error[1] = vel_err.norm();
		_output_tracking_error[2] = pos_err.norm();

		// calculate a velocity correction that will be applied to the output state history
		float vel_gain = _dt_ekf_avg / math::constrain(_params.vel_Tau, _dt_ekf_avg, 10.0f);
		_vel_err_integ += vel_err;
		Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

		// calculate a position correction that will be applied to the output state history
		float pos_gain = _dt_ekf_avg / math::constrain(_params.pos_Tau, _dt_ekf_avg, 10.0f);
		_pos_err_integ += pos_err;
		Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

		// loop through the output filter state history and apply the corrections to the velocity and position states
		// this method is too expensive to use for the attitude states due to the quaternion operations required
		// but does not introduce a time delay in the 'correction loop' and allows smaller tracking time constants
		// to be used
		outputSample output_states = {};
		unsigned max_index = _output_buffer.get_length() - 1;

		for (unsigned index = 0; index <= max_index; index++) {
			output_states = _output_buffer.get_from_index(index);

			// a constant  velocity correction is applied
			output_states.vel += vel_correction;

			// a constant position correction is applied
			output_states.pos += pos_correction;

			// push the updated data to the buffer
			_output_buffer.push_to_index(index, output_states);
		}

		// update output state to corrected values
		_output_new = _output_buffer.get_newest();
	}
}

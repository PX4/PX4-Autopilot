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


Ekf::Ekf():
	_filter_initialised(false),
	_earth_rate_initialised(false),
	_fuse_height(false),
	_fuse_pos(false),
	_fuse_hor_vel(false),
	_fuse_vert_vel(false),
	_fuse_flow(false),
	_fuse_hagl_data(false),
	_time_last_fake_gps(0),
	_time_last_pos_fuse(0),
	_time_last_vel_fuse(0),
	_time_last_hgt_fuse(0),
	_time_last_of_fuse(0),
	_time_last_arsp_fuse(0),
	_last_disarmed_posD(0.0f),
	_last_dt_overrun(0.0f),
	_airspeed_innov(0.0f),
	_airspeed_innov_var(0.0f),
	_heading_innov(0.0f),
	_heading_innov_var(0.0f),
	_delta_time_of(0.0f),
	_mag_declination(0.0f),
	_gpsDriftVelN(0.0f),
	_gpsDriftVelE(0.0f),
	_gps_drift_velD(0.0f),
	_gps_velD_diff_filt(0.0f),
	_gps_velN_filt(0.0f),
	_gps_velE_filt(0.0f),
	_last_gps_fail_us(0),
	_last_gps_origin_time_us(0),
	_gps_alt_ref(0.0f),
	_hgt_counter(0),
	_rng_filt_state(0.0f),
	_mag_counter(0),
	_time_last_mag(0),
	_hgt_sensor_offset(0.0f),
	_terrain_vpos(0.0f),
	_terrain_var(1.e4f),
	_hagl_innov(0.0f),
	_hagl_innov_var(0.0f),
	_time_last_hagl_fuse(0),
	_baro_hgt_faulty(false),
	_gps_hgt_faulty(false),
	_rng_hgt_faulty(false),
	_baro_hgt_offset(0.0f),
	_vert_pos_reset_delta(0.0f),
	_time_vert_pos_reset(0),
	_vert_vel_reset_delta(0.0f),
	_time_vert_vel_reset(0),
	_time_bad_vert_accel(0)
{
	_state = {};
	_last_known_posNE.setZero();
	_earth_rate_NED.setZero();
	_R_to_earth = matrix::Dcm<float>();
	memset(_vel_pos_innov, 0, sizeof(_vel_pos_innov));
	memset(_mag_innov, 0, sizeof(_mag_innov));
	memset(_flow_innov, 0, sizeof(_flow_innov));
	memset(_vel_pos_innov_var, 0, sizeof(_vel_pos_innov_var));
	memset(_mag_innov_var, 0, sizeof(_mag_innov_var));
	memset(_flow_innov_var, 0, sizeof(_flow_innov_var));
	_delta_angle_corr.setZero();
	_delta_vel_corr.setZero();
	_vel_corr.setZero();
	_last_known_posNE.setZero();
	_imu_down_sampled = {};
	_q_down_sampled.setZero();
	_mag_filt_state = {};
	_delVel_sum = {};
	_flow_gyro_bias = {};
	_imu_del_ang_of = {};
	_gps_check_fail_status.value = 0;
}

Ekf::~Ekf()
{
}

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
	_output_new.quat_nominal = matrix::Quaternion<float>();

	_delta_angle_corr.setZero();
	_delta_vel_corr.setZero();
	_vel_corr.setZero();

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
	_mag_healthy = false;

	_filter_initialised = false;
	_terrain_initialised = false;

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_dt_ekf_avg = 0.001f * (float)(FILTER_UPDATE_PERRIOD_MS);

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

		// perform state and variance prediction for the terrain estimator
		if (!_terrain_initialised) {
			_terrain_initialised = initHagl();

		} else {
			predictHagl();
		}

		// control logic
		controlFusionModes();

		// measurement updates

		// Fuse magnetometer data using the selected fuson method and only if angular alignment is complete
		if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
			if (_control_status.flags.mag_3D && _control_status.flags.yaw_align) {
				fuseMag();

				if (_control_status.flags.mag_dec) {
					fuseDeclination();
				}

			} else if (_control_status.flags.mag_hdg && _control_status.flags.yaw_align) {
				// fusion of a Euler yaw angle from either a 321 or 312 rotation sequence
				fuseHeading();

			} else {
				// do no fusion at all
			}
		}

		// determine if range finder data has fallen behind the fusin time horizon fuse it if we are
		// not tilted too much to use it
		if (_range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)
		    && (_R_to_earth(2, 2) > 0.7071f)) {
			// correct the range data for position offset relative to the IMU
			Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sample_delayed.rng += pos_offset_earth(2) / _R_to_earth(2, 2);

			// if we have range data we always try to estimate terrain height
			_fuse_hagl_data = true;

			// only use range finder as a height observation in the main filter if specifically enabled
			if (_control_status.flags.rng_hgt) {
				_fuse_height = true;
			}

		} else if ((_time_last_imu - _time_last_hgt_fuse) > 2 * RNG_MAX_INTERVAL && _control_status.flags.rng_hgt) {
			// If we are supposed to be using range finder data as the primary height sensor, have missed or rejected measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air) {
				_range_sample_delayed.rng = _params.rng_gnd_clearance;
				_range_sample_delayed.time_us = _imu_sample_delayed.time_us;

			}

			_fuse_height = true;
		}

		// determine if baro data has fallen behind the fuson time horizon and fuse it in the main filter if enabled
		uint64_t last_baro_time_us = _baro_sample_delayed.time_us;
		if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
			if (_control_status.flags.baro_hgt) {
				_fuse_height = true;

			} else {
				// calculate a filtered offset between the baro origin and local NED origin if we are not using the baro  as a height reference
				float local_time_step = 1e-6f*(float)(_baro_sample_delayed.time_us - last_baro_time_us);
				local_time_step = math::constrain(local_time_step,0.0f,1.0f);
				last_baro_time_us = _baro_sample_delayed.time_us;
				float offset_rate_correction =  0.1f * (_baro_sample_delayed.hgt - _hgt_sensor_offset) + _state.pos(2) - _baro_hgt_offset;
				_baro_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
			}
		}

		// If we are using GPS aiding and data has fallen behind the fusion time horizon then fuse it
		if (_gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed)) {
			// Only use GPS data for position and velocity aiding if enabled
			if (_control_status.flags.gps) {
				_fuse_pos = true;
				_fuse_vert_vel = true;
				_fuse_hor_vel = true;

				// correct velocity for offset relative to IMU
				Vector3f ang_rate = _imu_sample_delayed.delta_ang * (1.0f/_imu_sample_delayed.delta_ang_dt);
				Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
				Vector3f vel_offset_body = cross_product(ang_rate,pos_offset_body);
				Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
				_gps_sample_delayed.vel -= vel_offset_earth;

				// correct position and height for offset relative to IMU
				Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
				_gps_sample_delayed.pos(0) -= pos_offset_earth(0);
				_gps_sample_delayed.pos(1) -= pos_offset_earth(1);
				_gps_sample_delayed.hgt += pos_offset_earth(2);

			}

			// only use gps height observation in the main filter if specifically enabled
			if (_control_status.flags.gps_hgt) {
				_fuse_height = true;
			}

		}

		// If we are using optical flow aiding and data has fallen behind the fusion time horizon, then fuse it
		if (_flow_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed)
		    && _control_status.flags.opt_flow && (_time_last_imu - _time_last_optflow) < 2e5
		    && (_R_to_earth(2, 2) > 0.7071f)) {
			_fuse_flow = true;
		}

		// if we aren't doing any aiding, fake GPS measurements at the last known position to constrain drift
		// Coincide fake measurements with baro data for efficiency with a minimum fusion rate of 5Hz
		if (!_control_status.flags.gps && !_control_status.flags.opt_flow
		    && ((_time_last_imu - _time_last_fake_gps > 2e5) || _fuse_height)) {
			_fuse_pos = true;
			_gps_sample_delayed.pos(0) = _last_known_posNE(0);
			_gps_sample_delayed.pos(1) = _last_known_posNE(1);
			_time_last_fake_gps = _time_last_imu;
		}

		// fuse available range finder data into a terrain height estimator if it has been initialised
		if (_fuse_hagl_data && _terrain_initialised) {
			fuseHagl();
			_fuse_hagl_data = false;
		}

		// Fuse available NED velocity and position data into the main filter
		if (_fuse_height || _fuse_pos || _fuse_hor_vel || _fuse_vert_vel) {
			fuseVelPosHeight();
			_fuse_hor_vel = _fuse_vert_vel = _fuse_pos = _fuse_height = false;
		}

		// Update optical flow bias estimates
		calcOptFlowBias();

		// Fuse optical flow LOS rate observations into the main filter
		if (_fuse_flow) {
			fuseOptFlow();
			_last_known_posNE(0) = _state.pos(0);
			_last_known_posNE(1) = _state.pos(1);
			_fuse_flow = false;
		}

		// TODO This is just to get the logic inside but we will only start fusion once we tested this again
		//if (_airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed)) {
		if (false) {
			fuseAirspeed();
		}
	}

	// the output observer always runs
	calculateOutputStates();

	// check for NaN or inf on attitude states
	if (!ISFINITE(_state.quat_nominal(0)) || !ISFINITE(_output_new.quat_nominal(0))) {
		return false;
	}

	// We don't have valid data to output until tilt and yaw alignment is complete
	if (_control_status.flags.tilt_align && _control_status.flags.yaw_align) {
		return true;

	} else {
		return false;
	}
}

bool Ekf::initialiseFilter(void)
{
	// Keep accumulating measurements until we have a minimum of 10 samples for the baro and magnetoemter

	// Sum the IMU delta angle measurements
	imuSample imu_init = _imu_buffer.get_newest();
	_delVel_sum += imu_init.delta_vel;

	// Sum the magnetometer measurements
	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		if (_mag_counter == 0 && _mag_sample_delayed.time_us !=0) {
			// initialise the filter states and counter when we start getting valid data from the buffer
			_mag_filt_state = _mag_sample_delayed.mag;
			_mag_counter = 1;
		} else if (_mag_counter != 0) {
			// increment the sample count and apply a LPF to the measurement
			_mag_counter ++;
			_mag_filt_state = _mag_filt_state * 0.9f + _mag_sample_delayed.mag * 0.1f;
		}
	}

	// set the default height source from the adjustable parameter
	if (_hgt_counter == 0) {
		_primary_hgt_source = _params.vdist_sensor_type;
	}

	if (_primary_hgt_source == VDIST_SENSOR_RANGE) {
		if (_range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)) {
			if (_hgt_counter == 0 && _range_sample_delayed.time_us != 0) {
				// initialise the filter states and counter when we start getting valid data from the buffer
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = true;
				_rng_filt_state = _range_sample_delayed.rng;
				_hgt_counter = 1;
			} else if (_hgt_counter != 0) {
				// increment the sample count and apply a LPF to the measurement
				_hgt_counter ++;
				_rng_filt_state = 0.9f * _rng_filt_state + 0.1f * _range_sample_delayed.rng;
			}
		}

	} else if (_primary_hgt_source == VDIST_SENSOR_BARO || _primary_hgt_source == VDIST_SENSOR_GPS) {
		// if the user parameter specifies use of GPS for height we use baro height initially and switch to GPS
		// later when it passes checks.
		if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
			if (_hgt_counter == 0 && _baro_sample_delayed.time_us != 0) {
				// initialise the filter states and counter when we start getting valid data from the buffer
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_baro_hgt_offset = _baro_sample_delayed.hgt;
				_hgt_counter = 1;
			} else if (_hgt_counter != 0) {
				// increment the sample count and apply a LPF to the measurement
				_hgt_counter ++;
				_baro_hgt_offset = 0.9f * _baro_hgt_offset + 0.1f * _baro_sample_delayed.hgt;
			}
		}

	} else {
		return false;
	}

	// check to see if we have enough measurements and return false if not
	if (_hgt_counter <= 10 || _mag_counter <= 10) {
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
		resetMagHeading(mag_init);

		// if we are using the range finder as the primary source, then calculate the baro height at origin so  we can use baro as a backup
		// so it can be used as a backup ad set the initial height using the range finder
		if (_control_status.flags.rng_hgt) {
			baroSample baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt;
			_state.pos(2) = -math::max(_rng_filt_state * _R_to_earth(2, 2),_params.rng_gnd_clearance);
		}

		// initialise the state covariance matrix
		initialiseCovariance();

		// initialise the terrain estimator
		initHagl();

		// reset the essential fusion timeout counters
		_time_last_hgt_fuse = _time_last_imu;
		_time_last_pos_fuse = _time_last_imu;
		_time_last_vel_fuse = _time_last_imu;
		_time_last_hagl_fuse = _time_last_imu;
		_time_last_of_fuse = _time_last_imu;

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

	// update the rotation matrix and calculate the increment in velocity using the current orientation
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);
	_state.vel += _R_to_earth * corrected_delta_vel;

	// compensate for acceleration due to gravity
	_state.vel(2) += _gravity_mss * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	// update transformation matrix from body to world frame
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f*(_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	input = math::constrain(input,0.0005f * (float)(FILTER_UPDATE_PERRIOD_MS),0.002f * (float)(FILTER_UPDATE_PERRIOD_MS));
	_dt_ekf_avg = 0.99f*_dt_ekf_avg + 0.005f*(_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);
}

bool Ekf::collect_imu(imuSample &imu)
{
	// accumulate and downsample IMU data across a period FILTER_UPDATE_PERRIOD_MS long

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
	float target_dt = (float)(FILTER_UPDATE_PERRIOD_MS) / 1000;
	if (_imu_down_sampled.delta_ang_dt >= target_dt - _last_dt_overrun) {

		// store the amount we have over-run the target update rate by
		_last_dt_overrun = _imu_down_sampled.delta_ang_dt - target_dt;

		imu.delta_ang     = _q_down_sampled.to_axis_angle();
		imu.delta_vel     = _imu_down_sampled.delta_vel;
		imu.delta_ang_dt  = _imu_down_sampled.delta_ang_dt;
		imu.delta_vel_dt  = _imu_down_sampled.delta_vel_dt;
		imu.time_us       = imu.time_us;

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
	float dt_scale_correction = _dt_imu_avg/_dt_ekf_avg;
	delta_angle(0) = _imu_sample_new.delta_ang(0) - _state.gyro_bias(0)*dt_scale_correction;
	delta_angle(1) = _imu_sample_new.delta_ang(1) - _state.gyro_bias(1)*dt_scale_correction;
	delta_angle(2) = _imu_sample_new.delta_ang(2) - _state.gyro_bias(2)*dt_scale_correction;

	// correct delta velocity for bias offsets
	Vector3f delta_vel = _imu_sample_new.delta_vel - _state.accel_bias*dt_scale_correction;

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
	// apply a delta velocity correction required to track the velocity states at the EKF fusion time horizon
	Vector3f delta_vel_NED = _R_to_earth_now * delta_vel + _delta_vel_corr;

	// corrrect for measured accceleration due to gravity
	delta_vel_NED(2) += _gravity_mss * imu_new.delta_vel_dt;

	// save the previous velocity so we can use trapezidal integration
	Vector3f vel_last = _output_new.vel;

	// increment the INS velocity states by the measurement plus corrections
	_output_new.vel += delta_vel_NED;

	// use trapezoidal integration to calculate the INS position states
	// apply a velocity correction required to track the position states at the EKF fusion time horizon
	_output_new.pos += (_output_new.vel + vel_last) * (imu_new.delta_vel_dt * 0.5f) + _vel_corr * imu_new.delta_vel_dt;

	// store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
	_output_buffer.push(_output_new);

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

	// calculate gains that provides tight tracking of the estimator states and
	// adjust for changes in time delay to mantain consistent overshoot
	float omega = 1e6f / (_imu_sample_new.time_us - _imu_sample_delayed.time_us);

	// calculate a corrrection to the delta angle
	// that will cause the INS to track the EKF quaternions
	_delta_angle_corr = delta_ang_error * imu_new.delta_ang_dt * omega * 0.5f;

	// calculate a correction to the delta velocity
	// that will cause the INS to track the EKF velocity
	_delta_vel_corr = (_state.vel - _output_sample_delayed.vel) * imu_new.delta_vel_dt * omega * 0.5f;

	// calculate a correction to the INS velocity
	// that will cause the INS to track the EKF position
	_vel_corr = (_state.pos - _output_sample_delayed.pos) * omega * 0.6f;
}

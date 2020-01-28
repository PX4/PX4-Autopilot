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
 * @file ekf_helper.cpp
 * Definition of ekf helper functions.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <ecl.h>
#include <mathlib/mathlib.h>
#include <cstdlib>

// Reset the velocity states. If we have a recent and valid
// gps measurement then use for velocity initialisation
bool Ekf::resetVelocity()
{
	// used to calculate the velocity change due to the reset
	Vector3f vel_before_reset = _state.vel;

	// reset EKF states
	if (_control_status.flags.gps && _gps_check_fail_status.value==0) {
		ECL_INFO_TIMESTAMPED("reset velocity to GPS");
		// this reset is only called if we have new gps data at the fusion time horizon
		_state.vel = _gps_sample_delayed.vel;

		// use GPS accuracy to reset variances
		P.uncorrelateCovarianceSetVariance<3>(4, sq(_gps_sample_delayed.sacc));

	} else if (_control_status.flags.opt_flow) {
		ECL_INFO_TIMESTAMPED("reset velocity to flow");
		// constrain height above ground to be above minimum possible
		float heightAboveGndEst = fmaxf((_terrain_vpos - _state.pos(2)), _params.rng_gnd_clearance);

		// calculate absolute distance from focal point to centre of frame assuming a flat earth
		float range = heightAboveGndEst / _R_rng_to_earth_2_2;

		if ((range - _params.rng_gnd_clearance) > 0.3f && _flow_sample_delayed.dt > 0.05f) {
			// we should have reliable OF measurements so
			// calculate X and Y body relative velocities from OF measurements
			Vector3f vel_optflow_body;
			vel_optflow_body(0) = - range * _flow_compensated_XY_rad(1) / _flow_sample_delayed.dt;
			vel_optflow_body(1) =   range * _flow_compensated_XY_rad(0) / _flow_sample_delayed.dt;
			vel_optflow_body(2) = 0.0f;

			// rotate from body to earth frame
			Vector3f vel_optflow_earth;
			vel_optflow_earth = _R_to_earth * vel_optflow_body;

			// take x and Y components
			_state.vel(0) = vel_optflow_earth(0);
			_state.vel(1) = vel_optflow_earth(1);

		} else {
			_state.vel(0) = 0.0f;
			_state.vel(1) = 0.0f;
		}

		// reset the horizontal velocity variance using the optical flow noise variance
		P.uncorrelateCovarianceSetVariance<2>(4, sq(range) * calcOptFlowMeasVar());

	} else if (_control_status.flags.ev_vel) {
		ECL_INFO_TIMESTAMPED("reset velocity to ev velocity");
		Vector3f _ev_vel = _ev_sample_delayed.vel;
		if(_params.fusion_mode & MASK_ROTATE_EV){
			_ev_vel = _R_ev_to_ekf *_ev_sample_delayed.vel;
		}
		_state.vel = _ev_vel;
		P.uncorrelateCovarianceSetVariance<3>(4, _ev_sample_delayed.velVar);
	} else {
		ECL_INFO_TIMESTAMPED("reset velocity to zero");
		// Used when falling back to non-aiding mode of operation
		_state.vel(0) = 0.0f;
		_state.vel(1) = 0.0f;
		P.uncorrelateCovarianceSetVariance<2>(4, 25.0f);
	}

	// calculate the change in velocity and apply to the output predictor state history
	const Vector3f velocity_change = _state.vel - vel_before_reset;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel += velocity_change;
	}

	// apply the change in velocity to our newest velocity estimate
	// which was already taken out from the output buffer
	_output_new.vel += velocity_change;

	// capture the reset event
	_state_reset_status.velNE_change(0) = velocity_change(0);
	_state_reset_status.velNE_change(1) = velocity_change(1);
	_state_reset_status.velD_change = velocity_change(2);
	_state_reset_status.velNE_counter++;
	_state_reset_status.velD_counter++;

	return true;
}

// Reset position states. If we have a recent and valid
// gps measurement then use for position initialisation
bool Ekf::resetPosition()
{
	// ECL_INFO_TIMESTAMPED("Reset Position");
	// used to calculate the position change due to the reset
	Vector2f posNE_before_reset;
	posNE_before_reset(0) = _state.pos(0);
	posNE_before_reset(1) = _state.pos(1);

	// let the next odometry update know that the previous value of states cannot be used to calculate the change in position
	_hpos_prev_available = false;

	if (_control_status.flags.gps) {
		ECL_INFO_TIMESTAMPED("reset position to GPS");

		// this reset is only called if we have new gps data at the fusion time horizon
		_state.pos(0) = _gps_sample_delayed.pos(0);
		_state.pos(1) = _gps_sample_delayed.pos(1);

		// use GPS accuracy to reset variances
		P.uncorrelateCovarianceSetVariance<2>(7, sq(_gps_sample_delayed.hacc));

	} else if (_control_status.flags.ev_pos) {
		ECL_INFO_TIMESTAMPED("reset position to ev position");

		// this reset is only called if we have new ev data at the fusion time horizon
		Vector3f _ev_pos = _ev_sample_delayed.pos;
		if(_params.fusion_mode & MASK_ROTATE_EV){
			_ev_pos = _R_ev_to_ekf *_ev_sample_delayed.pos;
		}
		_state.pos(0) = _ev_pos(0);
		_state.pos(1) = _ev_pos(1);

		// use EV accuracy to reset variances
		P.uncorrelateCovarianceSetVariance<2>(7, _ev_sample_delayed.posVar.slice<2, 1>(0, 0));

	} else if (_control_status.flags.opt_flow) {
		ECL_INFO_TIMESTAMPED("reset position to last known position");

		if (!_control_status.flags.in_air) {
			// we are likely starting OF for the first time so reset the horizontal position
			_state.pos(0) = 0.0f;
			_state.pos(1) = 0.0f;

		} else {
			// set to the last known position
			_state.pos(0) = _last_known_posNE(0);
			_state.pos(1) = _last_known_posNE(1);

		}

		// estimate is relative to initial position in this mode, so we start with zero error.
		P.uncorrelateCovarianceSetVariance<2>(7, 0.0f);

	} else {
		ECL_INFO_TIMESTAMPED("reset position to last known position");
		// Used when falling back to non-aiding mode of operation
		_state.pos(0) = _last_known_posNE(0);
		_state.pos(1) = _last_known_posNE(1);
		P.uncorrelateCovarianceSetVariance<2>(7, sq(_params.pos_noaid_noise));
	}

	// calculate the change in position and apply to the output predictor state history
	const Vector2f posNE_change{_state.pos(0) - posNE_before_reset(0), _state.pos(1) - posNE_before_reset(1)};

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].pos(0) += posNE_change(0);
		_output_buffer[index].pos(1) += posNE_change(1);
	}

	// apply the change in position to our newest position estimate
	// which was already taken out from the output buffer
	_output_new.pos(0) += posNE_change(0);
	_output_new.pos(1) += posNE_change(1);

	// capture the reset event
	_state_reset_status.posNE_change = posNE_change;
	_state_reset_status.posNE_counter++;

	return true;
}

// Reset height state using the last height measurement
void Ekf::resetHeight()
{
	// Get the most recent GPS data
	const gpsSample &gps_newest = _gps_buffer.get_newest();

	// store the current vertical position and velocity for reference so we can calculate and publish the reset amount
	float old_vert_pos = _state.pos(2);
	bool vert_pos_reset = false;
	float old_vert_vel = _state.vel(2);
	bool vert_vel_reset = false;

	// reset the vertical position
	if (_control_status.flags.rng_hgt) {

			float new_pos_down = _hgt_sensor_offset - _range_sample_delayed.rng * _R_rng_to_earth_2_2;

			// update the state and associated variance
			_state.pos(2) = new_pos_down;

			// the state variance is the same as the observation
			P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.range_noise));

			vert_pos_reset = true;

			// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
			const baroSample &baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

	} else if (_control_status.flags.baro_hgt) {
		// initialize vertical position with newest baro measurement
		const baroSample &baro_newest = _baro_buffer.get_newest();

		if (isRecent(baro_newest.time_us, 2 * BARO_MAX_INTERVAL)) {
			_state.pos(2) = _hgt_sensor_offset - baro_newest.hgt + _baro_hgt_offset;

			// the state variance is the same as the observation
			P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.baro_noise));

			vert_pos_reset = true;

		} else {
			// TODO: reset to last known baro based estimate
		}

	} else if (_control_status.flags.gps_hgt) {
		// initialize vertical position and velocity with newest gps measurement
		if (isRecent(gps_newest.time_us, 2 * GPS_MAX_INTERVAL)) {
			_state.pos(2) = _hgt_sensor_offset - gps_newest.hgt + _gps_alt_ref;

			// the state variance is the same as the observation
			P.uncorrelateCovarianceSetVariance<1>(9, sq(gps_newest.hacc));

			vert_pos_reset = true;

			// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
			const baroSample &baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

		} else {
			// TODO: reset to last known gps based estimate
		}

	} else if (_control_status.flags.ev_hgt) {
		// initialize vertical position with newest measurement
		const extVisionSample &ev_newest = _ext_vision_buffer.get_newest();

		// use the most recent data if it's time offset from the fusion time horizon is smaller
		int32_t dt_newest = ev_newest.time_us - _imu_sample_delayed.time_us;
		int32_t dt_delayed = _ev_sample_delayed.time_us - _imu_sample_delayed.time_us;

		vert_pos_reset = true;

		if (std::abs(dt_newest) < std::abs(dt_delayed)) {
			_state.pos(2) = ev_newest.pos(2);

		} else {
			_state.pos(2) = _ev_sample_delayed.pos(2);
		}

	}

	// reset the vertical velocity state
	if (_control_status.flags.gps && isRecent(gps_newest.time_us, 2 * GPS_MAX_INTERVAL)) {
		// If we are using GPS, then use it to reset the vertical velocity
		_state.vel(2) = gps_newest.vel(2);

		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(6, sq(1.5f * gps_newest.sacc));

	} else {
		// we don't know what the vertical velocity is, so set it to zero
		_state.vel(2) = 0.0f;

		// Set the variance to a value large enough to allow the state to converge quickly
		// that does not destabilise the filter
		P.uncorrelateCovarianceSetVariance<1>(6, 10.0f);

	}

	vert_vel_reset = true;

	// store the reset amount and time to be published
	if (vert_pos_reset) {
		_state_reset_status.posD_change = _state.pos(2) - old_vert_pos;
		_state_reset_status.posD_counter++;
	}

	if (vert_vel_reset) {
		_state_reset_status.velD_change = _state.vel(2) - old_vert_vel;
		_state_reset_status.velD_counter++;
	}

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	if (vert_pos_reset) {
		_output_new.pos(2) += _state_reset_status.posD_change;
	}

	if (vert_vel_reset) {
		_output_new.vel(2) += _state_reset_status.velD_change;
	}

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		if (vert_pos_reset) {
			_output_buffer[i].pos(2) += _state_reset_status.posD_change;
			_output_vert_buffer[i].vel_d_integ += _state_reset_status.posD_change;
		}

		if (vert_vel_reset) {
			_output_buffer[i].vel(2) += _state_reset_status.velD_change;
			_output_vert_buffer[i].vel_d += _state_reset_status.velD_change;
		}
	}

	// add the reset amount to the output observer vertical position state
	if (vert_pos_reset) {
		_output_vert_delayed.vel_d_integ = _state.pos(2);
		_output_vert_new.vel_d_integ = _state.pos(2);
	}

	if (vert_vel_reset) {
		_output_vert_delayed.vel_d = _state.vel(2);
		_output_vert_new.vel_d = _state.vel(2);
	}
}

// align output filter states to match EKF states at the fusion time horizon
void Ekf::alignOutputFilter()
{
	// calculate the quaternion rotation delta from the EKF to output observer states at the EKF fusion time horizon
	Quatf q_delta = _state.quat_nominal * _output_sample_delayed.quat_nominal.inversed();
	q_delta.normalize();

	// calculate the velocity and position deltas between the output and EKF at the EKF fusion time horizon
	const Vector3f vel_delta = _state.vel - _output_sample_delayed.vel;
	const Vector3f pos_delta = _state.pos - _output_sample_delayed.pos;

	// loop through the output filter state history and add the deltas
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = q_delta * _output_buffer[i].quat_nominal;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}

	_output_new.quat_nominal = q_delta * _output_new.quat_nominal;
	_output_new.quat_nominal.normalize();

	_output_sample_delayed.quat_nominal = q_delta * _output_sample_delayed.quat_nominal;
	_output_sample_delayed.quat_nominal.normalize();
}

// Do a forced re-alignment of the yaw angle to align with the horizontal velocity vector from the GPS.
// It is used to align the yaw angle after launch or takeoff for fixed wing vehicle only.
bool Ekf::realignYawGPS()
{
	const float gpsSpeed = sqrtf(sq(_gps_sample_delayed.vel(0)) + sq(_gps_sample_delayed.vel(1)));

	// Need at least 5 m/s of GPS horizontal speed and
	// ratio of velocity error to velocity < 0.15  for a reliable alignment
	if ((gpsSpeed > 5.0f) && (_gps_sample_delayed.sacc < (0.15f * gpsSpeed))) {
		// check for excessive horizontal GPS velocity innovations
		const bool badVelInnov = (_gps_vel_test_ratio(0) > 1.0f) && _control_status.flags.gps;

		// calculate GPS course over ground angle
		const float gpsCOG = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

		// calculate course yaw angle
		const float ekfCOG = atan2f(_state.vel(1), _state.vel(0));

		// Check the EKF and GPS course over ground for consistency
		const float courseYawError = gpsCOG - ekfCOG;

		// If the angles disagree and horizontal GPS velocity innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
		const bool badYawErr = fabsf(courseYawError) > 0.5f;
		const bool badMagYaw = (badYawErr && badVelInnov);

		if (badMagYaw) {
			_num_bad_flight_yaw_events ++;
		}

		// correct yaw angle using GPS ground course if compass yaw bad or yaw is previously not aligned
		if (badMagYaw || !_control_status.flags.yaw_align) {
			ECL_WARN_TIMESTAMPED("bad yaw corrected using GPS course");

			// declare the magnetometer as failed if a bad yaw has occurred more than once
			if (_control_status.flags.mag_aligned_in_flight && (_num_bad_flight_yaw_events >= 2) && !_control_status.flags.mag_fault) {
				ECL_WARN_TIMESTAMPED("stopping magnetometer use");
				_control_status.flags.mag_fault = true;
			}

			// save a copy of the quaternion state for later use in calculating the amount of reset change
			const Quatf quat_before_reset = _state.quat_nominal;

			// update transformation matrix from body to world frame using the current state estimate
			_R_to_earth = Dcmf(_state.quat_nominal);

			// get quaternion from existing filter states and calculate roll, pitch and yaw angles
			Eulerf euler321(_state.quat_nominal);

			// apply yaw correction
			if (!_control_status.flags.mag_aligned_in_flight) {
				// This is our first flight alignment so we can assume that the recent change in velocity has occurred due to a
				// forward direction takeoff or launch and therefore the inertial and GPS ground course discrepancy is due to yaw error
				euler321(2) += courseYawError;
				_control_status.flags.mag_aligned_in_flight = true;

			} else if (_control_status.flags.wind) {
				// we have previously aligned yaw in-flight and have wind estimates so set the yaw such that the vehicle nose is
				// aligned with the wind relative GPS velocity vector
				euler321(2) = atan2f((_gps_sample_delayed.vel(1) - _state.wind_vel(1)),
						     (_gps_sample_delayed.vel(0) - _state.wind_vel(0)));

			} else {
				// we don't have wind estimates, so align yaw to the GPS velocity vector
				euler321(2) = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

			}

			// calculate new filter quaternion states using corrected yaw angle
			_state.quat_nominal = Quatf(euler321);
			_R_to_earth = Dcmf(_state.quat_nominal);
			uncorrelateQuatStates();

			// If heading was bad, then we also need to reset the velocity and position states
			_velpos_reset_request = badMagYaw;

			// Use the last magnetometer measurements to reset the field states
			_state.mag_B.zero();
			_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

			// use the combined EKF and GPS speed variance to calculate a rough estimate of the yaw error after alignment
			float SpdErrorVariance = sq(_gps_sample_delayed.sacc) + P(4,4) + P(5,5);
			float sineYawError = math::constrain(sqrtf(SpdErrorVariance) / gpsSpeed, 0.0f, 1.0f);

			// adjust the quaternion covariances estimated yaw error
			increaseQuatYawErrVariance(sq(asinf(sineYawError)));

			// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
			clearMagCov();

			if (_control_status.flags.mag_3D) {
				for (uint8_t index = 16; index <= 21; index ++) {
					P(index,index) = sq(_params.mag_noise);
				}

				// save covariance data for re-use when auto-switching between heading and 3-axis fusion
				saveMagCovData();
			}

			// record the start time for the magnetic field alignment
			_flt_mag_align_start_time = _imu_sample_delayed.time_us;

			// calculate the amount that the quaternion has changed by
			_state_reset_status.quat_change = _state.quat_nominal * quat_before_reset.inversed();

			// add the reset amount to the output observer buffered data
			for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
				_output_buffer[i].quat_nominal = _state_reset_status.quat_change * _output_buffer[i].quat_nominal;
			}

			// apply the change in attitude quaternion to our newest quaternion estimate
			// which was already taken out from the output buffer
			_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;

			// capture the reset event
			_state_reset_status.quat_counter++;

			return true;

		} else {
			// align mag states only

			// calculate initial earth magnetic field states
			_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

			// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
			clearMagCov();

			if (_control_status.flags.mag_3D) {
				for (uint8_t index = 16; index <= 21; index ++) {
					P(index,index) = sq(_params.mag_noise);
				}

				// save covariance data for re-use when auto-switching between heading and 3-axis fusion
				saveMagCovData();
			}

			// record the start time for the magnetic field alignment
			_flt_mag_align_start_time = _imu_sample_delayed.time_us;

			return true;
		}

	} else {
		// attempt a normal alignment using the magnetometer
		return resetMagHeading(_mag_lpf.getState());

	}
}

// Reset heading and magnetic field states
bool Ekf::resetMagHeading(const Vector3f &mag_init, bool increase_yaw_var, bool update_buffer)
{
	// prevent a reset being performed more than once on the same frame
	if (_imu_sample_delayed.time_us == _flt_mag_align_start_time) {
		return true;
	}

	if (_params.mag_fusion_type >= MAG_FUSE_TYPE_NONE) {
		stopMagFusion();
		return false;
	}

	// save a copy of the quaternion state for later use in calculating the amount of reset change
	const Quatf quat_before_reset = _state.quat_nominal;
	Quatf quat_after_reset = _state.quat_nominal;

	// update transformation matrix from body to world frame using the current estimate
	_R_to_earth = Dcmf(_state.quat_nominal);

	// calculate the initial quaternion
	// determine if a 321 or 312 Euler sequence is best
	if (fabsf(_R_to_earth(2, 0)) < fabsf(_R_to_earth(2, 1))) {
		// use a 321 sequence

		// rotate the magnetometer measurement into earth frame
		Eulerf euler321(_state.quat_nominal);

		// Set the yaw angle to zero and calculate the rotation matrix from body to earth frame
		euler321(2) = 0.0f;

		// calculate the observed yaw angle
		if (_control_status.flags.ev_yaw) {
			// convert the observed quaternion to a rotation matrix
			const Dcmf R_to_earth_ev(_ev_sample_delayed.quat);	// transformation matrix from body to world frame
			// calculate the yaw angle for a 312 sequence
			euler321(2) = atan2f(R_to_earth_ev(1, 0), R_to_earth_ev(0, 0));

		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
			const Dcmf R_to_earth(euler321);
			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			const Vector3f mag_earth_pred = R_to_earth * mag_init;
			// the angle of the projection onto the horizontal gives the yaw angle
			euler321(2) = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_INDOOR && _mag_use_inhibit) {
			// we are operating without knowing the earth frame yaw angle
			return true;

		} else {
			// there is no yaw observation
			return false;
		}

		// calculate initial quaternion states for the ekf
		// we don't change the output attitude to avoid jumps
		quat_after_reset = Quatf(euler321);

	} else {
		// use a 312 sequence

		// Calculate the 312 sequence euler angles that rotate from earth to body frame
		// See http://www.atacolorado.com/eulersequences.doc
		Vector3f euler312;
		euler312(0) = atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1));  // first rotation (yaw)
		euler312(1) = asinf(_R_to_earth(2, 1)); // second rotation (roll)
		euler312(2) = atan2f(-_R_to_earth(2, 0), _R_to_earth(2, 2));  // third rotation (pitch)

		// Set the first rotation (yaw) to zero and calculate the rotation matrix from body to earth frame
		euler312(0) = 0.0f;

		// Calculate the body to earth frame rotation matrix from the euler angles using a 312 rotation sequence
		float c2 = cosf(euler312(2));
		float s2 = sinf(euler312(2));
		float s1 = sinf(euler312(1));
		float c1 = cosf(euler312(1));
		float s0 = sinf(euler312(0));
		float c0 = cosf(euler312(0));

		Dcmf R_to_earth;
		R_to_earth(0, 0) = c0 * c2 - s0 * s1 * s2;
		R_to_earth(1, 1) = c0 * c1;
		R_to_earth(2, 2) = c2 * c1;
		R_to_earth(0, 1) = -c1 * s0;
		R_to_earth(0, 2) = s2 * c0 + c2 * s1 * s0;
		R_to_earth(1, 0) = c2 * s0 + s2 * s1 * c0;
		R_to_earth(1, 2) = s0 * s2 - s1 * c0 * c2;
		R_to_earth(2, 0) = -s2 * c1;
		R_to_earth(2, 1) = s1;

		// calculate the observed yaw angle
		if (_control_status.flags.ev_yaw) {
			// convert the observed quaternion to a rotation matrix
			const Dcmf R_to_earth_ev(_ev_sample_delayed.quat);
			// calculate the yaw angle for a 312 sequence
			euler312(0) = atan2f(-R_to_earth_ev(0, 1), R_to_earth_ev(1, 1));

		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			const Vector3f mag_earth_pred = R_to_earth * mag_init;
			// the angle of the projection onto the horizontal gives the yaw angle
			euler312(0) = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_INDOOR && _mag_use_inhibit) {
			// we are operating without knowing the earth frame yaw angle
			return true;

		} else {
			// there is no yaw observation
			return false;
		}

		// re-calculate the rotation matrix using the updated yaw angle
		s0 = sinf(euler312(0));
		c0 = cosf(euler312(0));
		R_to_earth(0, 0) = c0 * c2 - s0 * s1 * s2;
		R_to_earth(1, 1) = c0 * c1;
		R_to_earth(2, 2) = c2 * c1;
		R_to_earth(0, 1) = -c1 * s0;
		R_to_earth(0, 2) = s2 * c0 + c2 * s1 * s0;
		R_to_earth(1, 0) = c2 * s0 + s2 * s1 * c0;
		R_to_earth(1, 2) = s0 * s2 - s1 * c0 * c2;
		R_to_earth(2, 0) = -s2 * c1;
		R_to_earth(2, 1) = s1;

		// calculate initial quaternion states for the ekf
		// we don't change the output attitude to avoid jumps
		quat_after_reset = Quatf(R_to_earth);
	}

	// set the earth magnetic field states using the updated rotation
	const Dcmf R_to_earth_after(quat_after_reset);
	_state.mag_I = R_to_earth_after * mag_init;

	// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
	clearMagCov();

	if (_control_status.flags.mag_3D) {
		for (uint8_t index = 16; index <= 21; index ++) {
			P(index,index) = sq(_params.mag_noise);
		}

		// save covariance data for re-use when auto-switching between heading and 3-axis fusion
		saveMagCovData();
	}

	// record the time for the magnetic field alignment event
	_flt_mag_align_start_time = _imu_sample_delayed.time_us;

	// calculate the amount that the quaternion has changed by
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// update quaternion states
	_state.quat_nominal = quat_after_reset;
	_R_to_earth = Dcmf(_state.quat_nominal);
	uncorrelateQuatStates();

	// record the state change
	_state_reset_status.quat_change = q_error;

	if (increase_yaw_var) {
		// update the yaw angle variance using the variance of the measurement
		if (_control_status.flags.ev_yaw) {
			// using error estimate from external vision data
			increaseQuatYawErrVariance(fmaxf(_ev_sample_delayed.angVar, sq(1.0e-2f)));
		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
			// using magnetic heading tuning parameter
			increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
		}
	}

	if (update_buffer) {
		// add the reset amount to the output observer buffered data
		for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
			_output_buffer[i].quat_nominal = _state_reset_status.quat_change * _output_buffer[i].quat_nominal;
		}

		// apply the change in attitude quaternion to our newest quaternion estimate
		// which was already taken out from the output buffer
		_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;
	}

	// capture the reset event
	_state_reset_status.quat_counter++;

	return true;
}

// Return the magnetic declination in radians to be used by the alignment and fusion processing
float Ekf::getMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_control_status.flags.mag_aligned_in_flight) {
		// Use value consistent with earth field state
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised) {
			return _mag_declination_gps;

		} else {
			return math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		return math::radians(_params.mag_declination_deg);
	}
}

void Ekf::constrainStates()
{
	for (int i = 0; i < 4; i++) {
		_state.quat_nominal(i) = math::constrain(_state.quat_nominal(i), -1.0f, 1.0f);
	}

	for (int i = 0; i < 3; i++) {
		_state.vel(i) = math::constrain(_state.vel(i), -1000.0f, 1000.0f);
	}

	for (int i = 0; i < 3; i++) {
		_state.pos(i) = math::constrain(_state.pos(i), -1.e6f, 1.e6f);
	}

	for (int i = 0; i < 3; i++) {
		_state.delta_ang_bias(i) = math::constrain(_state.delta_ang_bias(i), -math::radians(20.f) * _dt_ekf_avg, math::radians(20.f) * _dt_ekf_avg);
	}

	for (int i = 0; i < 3; i++) {
		_state.delta_vel_bias(i) = math::constrain(_state.delta_vel_bias(i), -_params.acc_bias_lim * _dt_ekf_avg, _params.acc_bias_lim * _dt_ekf_avg);
	}

	for (int i = 0; i < 3; i++) {
		_state.mag_I(i) = math::constrain(_state.mag_I(i), -1.0f, 1.0f);
	}

	for (int i = 0; i < 3; i++) {
		_state.mag_B(i) = math::constrain(_state.mag_B(i), -0.5f, 0.5f);
	}

	for (int i = 0; i < 2; i++) {
		_state.wind_vel(i) = math::constrain(_state.wind_vel(i), -100.0f, 100.0f);
	}
}

float Ekf::compensateBaroForDynamicPressure(const float baro_alt_uncompensated)
{
	// calculate static pressure error = Pmeas - Ptruth
	// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
	// negative X and Y directions. Used to correct baro data for positional errors
	const matrix::Dcmf R_to_body(_output_new.quat_nominal.inversed());

	// Calculate airspeed in body frame
	const Vector3f velocity_earth = _output_new.vel - _vel_imu_rel_body_ned;

	const Vector3f wind_velocity_earth(_state.wind_vel(0), _state.wind_vel(1), 0.0f);

	const Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;

	const Vector3f airspeed_body = R_to_body * airspeed_earth;

	const Vector3f K_pstatic_coef(airspeed_body(0) >= 0.0f ? _params.static_pressure_coef_xp : _params.static_pressure_coef_xn,
				      airspeed_body(1) >= 0.0f ? _params.static_pressure_coef_yp : _params.static_pressure_coef_yn,
				      _params.static_pressure_coef_z);

	const Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.max_correction_airspeed));

	const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

	// correct baro measurement using pressure error estimate and assuming sea level gravity
	return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
}

// calculate the earth rotation vector
Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

void Ekf::getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2],  float &vpos)
{
	hvel[0] = _gps_vel_innov(0);
	hvel[1] = _gps_vel_innov(1);
	vvel    = _gps_vel_innov(2);
	hpos[0] = _gps_pos_innov(0);
	hpos[1] = _gps_pos_innov(1);
	vpos    = _gps_pos_innov(2);
}

void Ekf::getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos)
{
	hvel[0] = _gps_vel_innov_var(0);
	hvel[1] = _gps_vel_innov_var(1);
	vvel    = _gps_vel_innov_var(2);
	hpos[0] = _gps_pos_innov_var(0);
	hpos[1] = _gps_pos_innov_var(1);
	vpos    = _gps_pos_innov_var(2);
}

void Ekf::getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos)
{
	hvel = _gps_vel_test_ratio(0);
	vvel = _gps_vel_test_ratio(1);
	hpos = _gps_pos_test_ratio(0);
	vpos = _gps_pos_test_ratio(1);
}

void Ekf::getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos)
{
	hvel[0] = _ev_vel_innov(0);
	hvel[1] = _ev_vel_innov(1);
	vvel    = _ev_vel_innov(2);
	hpos[0] = _ev_pos_innov(0);
	hpos[1] = _ev_pos_innov(1);
	vpos    = _ev_pos_innov(2);
}

void Ekf::getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos)
{
	hvel[0] = _ev_vel_innov_var(0);
	hvel[1] = _ev_vel_innov_var(1);
	vvel    = _ev_vel_innov_var(2);
	hpos[0] = _ev_pos_innov_var(0);
	hpos[1] = _ev_pos_innov_var(1);
	vpos    = _ev_pos_innov_var(2);
}

void Ekf::getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos)
{
	hvel = _ev_vel_test_ratio(0);
	vvel = _ev_vel_test_ratio(1);
	hpos = _ev_pos_test_ratio(0);
	vpos = _ev_pos_test_ratio(1);
}

void Ekf::getBaroHgtInnov(float &baro_hgt_innov)
{
	baro_hgt_innov = _baro_hgt_innov(2);
}

void Ekf::getBaroHgtInnovVar(float &baro_hgt_innov_var)
{
	baro_hgt_innov_var = _baro_hgt_innov_var(2);
}

void Ekf::getBaroHgtInnovRatio(float &baro_hgt_innov_ratio)
{
	baro_hgt_innov_ratio = _baro_hgt_test_ratio(1);
}

void Ekf::getRngHgtInnov(float &rng_hgt_innov)
{
	rng_hgt_innov = _rng_hgt_innov(2);
}

void Ekf::getRngHgtInnovVar(float &rng_hgt_innov_var)
{
	rng_hgt_innov_var = _rng_hgt_innov_var(2);
}

void Ekf::getRngHgtInnovRatio(float &rng_hgt_innov_ratio)
{
	rng_hgt_innov_ratio = _rng_hgt_test_ratio(1);
}

void Ekf::getAuxVelInnov(float aux_vel_innov[2])
{
	aux_vel_innov[0] = _aux_vel_innov(0);
	aux_vel_innov[1] = _aux_vel_innov(1);
}

void Ekf::getAuxVelInnovVar(float aux_vel_innov_var[2])
{
	aux_vel_innov_var[0] = _aux_vel_innov_var(0);
	aux_vel_innov_var[1] = _aux_vel_innov_var(1);
}

void Ekf::getAuxVelInnovRatio(float &aux_vel_innov_ratio)
{
	aux_vel_innov_ratio = _aux_vel_test_ratio(0);
}

void Ekf::getFlowInnov(float flow_innov[2])
{
	memcpy(flow_innov, _flow_innov, sizeof(_flow_innov));
}

void Ekf::getFlowInnovVar(float flow_innov_var[2])
{
	memcpy(flow_innov_var, _flow_innov_var, sizeof(_flow_innov_var));
}

void Ekf::getFlowInnovRatio(float &flow_innov_ratio)
{
	flow_innov_ratio = _optflow_test_ratio;
}

void Ekf::getHeadingInnov(float &heading_innov)
{
	heading_innov = _heading_innov;
}

void Ekf::getHeadingInnovVar(float &heading_innov_var)
{
	heading_innov_var = _heading_innov_var;
}

void Ekf::getHeadingInnovRatio(float &heading_innov_ratio)
{
	heading_innov_ratio = _yaw_test_ratio;
}

void Ekf::getMagInnov(float mag_innov[3])
{
	memcpy(mag_innov, _mag_innov, sizeof(_mag_innov));
}

void Ekf::getMagInnovVar(float mag_innov_var[3])
{
	memcpy(mag_innov_var, _mag_innov_var, sizeof(_mag_innov_var));
}

void Ekf::getMagInnovRatio(float &mag_innov_ratio)
{
	mag_innov_ratio = math::max(math::max(_mag_test_ratio[0], _mag_test_ratio[1]), _mag_test_ratio[2]);
}

void Ekf::getDragInnov(float drag_innov[2])
{
	memcpy(drag_innov, _drag_innov, sizeof(_drag_innov));
}

void Ekf::getDragInnovVar(float drag_innov_var[2])
{
	memcpy(drag_innov_var, _drag_innov_var, sizeof(_drag_innov_var));
}

void Ekf::getDragInnovRatio(float drag_innov_ratio[2])
{
	memcpy(drag_innov_ratio, &_drag_test_ratio, sizeof(_drag_test_ratio));
}

void Ekf::getAirspeedInnov(float &airspeed_innov)
{
	airspeed_innov = _airspeed_innov;
}

void Ekf::getAirspeedInnovVar(float &airspeed_innov_var)
{
	airspeed_innov_var = _airspeed_innov_var;
}

void Ekf::getAirspeedInnovRatio(float &airspeed_innov_ratio)
{
	airspeed_innov_ratio = _tas_test_ratio;
}

void Ekf::getBetaInnov(float &beta_innov)
{
	beta_innov = _beta_innov;
}

void Ekf::getBetaInnovVar(float &beta_innov_var)
{
	beta_innov_var = _beta_innov_var;
}

void Ekf::getBetaInnovRatio(float &beta_innov_ratio)
{
	beta_innov_ratio = _beta_test_ratio;
}

void Ekf::getHaglInnov(float &hagl_innov)
{
	hagl_innov = _hagl_innov;
}

void Ekf::getHaglInnovVar(float &hagl_innov_var)
{
	hagl_innov_var = _hagl_innov_var;
}

void Ekf::getHaglInnovRatio(float &hagl_innov_ratio)
{
	hagl_innov_ratio = _hagl_test_ratio;
}

// get GPS check status
void Ekf::get_gps_check_status(uint16_t *val)
{
	*val = _gps_check_fail_status.value;
}

// get the state vector at the delayed time horizon
void Ekf::get_state_delayed(float *state)
{
	for (int i = 0; i < 4; i++) {
		state[i] = _state.quat_nominal(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 4] = _state.vel(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 7] = _state.pos(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 10] = _state.delta_ang_bias(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 13] = _state.delta_vel_bias(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 16] = _state.mag_I(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 19] = _state.mag_B(i);
	}

	for (int i = 0; i < 2; i++) {
		state[i + 22] = _state.wind_vel(i);
	}
}

// get the accelerometer bias
void Ekf::get_accel_bias(float bias[3])
{
	float temp[3];
	temp[0] = _state.delta_vel_bias(0) / _dt_ekf_avg;
	temp[1] = _state.delta_vel_bias(1) / _dt_ekf_avg;
	temp[2] = _state.delta_vel_bias(2) / _dt_ekf_avg;
	memcpy(bias, temp, 3 * sizeof(float));
}

// get the gyroscope bias in rad/s
void Ekf::get_gyro_bias(float bias[3])
{
	float temp[3];
	temp[0] = _state.delta_ang_bias(0) / _dt_ekf_avg;
	temp[1] = _state.delta_ang_bias(1) / _dt_ekf_avg;
	temp[2] = _state.delta_ang_bias(2) / _dt_ekf_avg;
	memcpy(bias, temp, 3 * sizeof(float));
}

// get the position and height of the ekf origin in WGS-84 coordinates and time the origin was set
// return true if the origin is valid
bool Ekf::get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt)
{
	memcpy(origin_time, &_last_gps_origin_time_us, sizeof(uint64_t));
	memcpy(origin_pos, &_pos_ref, sizeof(map_projection_reference_s));
	memcpy(origin_alt, &_gps_alt_ref, sizeof(float));
	return _NED_origin_initialised;
}

// return an array containing the output predictor angular, velocity and position tracking
// error magnitudes (rad), (m/s), (m)
void Ekf::get_output_tracking_error(float error[3])
{
	memcpy(error, _output_tracking_error, 3 * sizeof(float));
}

/*
Returns  following IMU vibration metrics in the following array locations
0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
*/
void Ekf::get_imu_vibe_metrics(float vibe[3])
{
	memcpy(vibe, _vibe_metrics, 3 * sizeof(float));
}

/*
	First argument returns GPS drift  metrics in the following array locations
	0 : Horizontal position drift rate (m/s)
	1 : Vertical position drift rate (m/s)
	2 : Filtered horizontal velocity (m/s)
	Second argument returns true when IMU movement is blocking the drift calculation
	Function returns true if the metrics have been updated and not returned previously by this function
*/
bool Ekf::get_gps_drift_metrics(float drift[3], bool *blocked)
{
	memcpy(drift, _gps_drift_metrics, 3 * sizeof(float));
	*blocked = !_vehicle_at_rest;
	if (_gps_drift_updated) {
		_gps_drift_updated = false;
		return true;
	}
	return false;
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv)
{
	// report absolute accuracy taking into account the uncertainty in location of the origin
	// If not aiding, return 0 for horizontal position estimate as no estimate is available
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P(7,7) + P(8,8) + sq(_gps_origin_eph));

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning && (_control_status.flags.gps)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));
	}
	else if (_is_dead_reckoning && (_control_status.flags.ev_pos)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_ev_pos_innov(0)) + sq(_ev_pos_innov(1))));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9,9) + sq(_gps_origin_epv));
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv)
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P(7,7) + P(8,8));

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning && _control_status.flags.gps) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9,9));
}

// get the 1-sigma horizontal and vertical velocity uncertainty
void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv)
{
	float hvel_err = sqrtf(P(4,4) + P(5,5));

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning) {
		float vel_err_conservative = 0.0f;

		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max((_terrain_vpos - _state.pos(2)), gndclearance) * sqrtf(sq(_flow_innov[0]) + sq(_flow_innov[1]));
		}

		if (_control_status.flags.gps) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));
		}
		else if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_ev_pos_innov(0)) + sq(_ev_pos_innov(1))));
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_ev_vel_innov(0)) + sq(_ev_vel_innov(1))));
		}
		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(6,6));
}

/*
Returns the following vehicle control limits required by the estimator to keep within sensor limitations.
vxy_max : Maximum ground relative horizontal speed (meters/sec). NaN when limiting is not needed.
vz_max : Maximum ground relative vertical speed (meters/sec). NaN when limiting is not needed.
hagl_min : Minimum height above ground (meters). NaN when limiting is not needed.
hagl_max : Maximum height above ground (meters). NaN when limiting is not needed.
*/
void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max)
{
	// Calculate range finder limits
	const float rangefinder_hagl_min = _rng_valid_min_val;
	// Allow use of 75% of rangefinder maximum range to allow for angular motion
	const float rangefinder_hagl_max = 0.75f * _rng_valid_max_val;

	// Calculate optical flow limits
	// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
	const float flow_vxy_max = fmaxf(0.5f * _flow_max_rate * (_terrain_vpos - _state.pos(2)), 0.0f);
	const float flow_hagl_min = _flow_min_distance;
	const float flow_hagl_max = _flow_max_distance;

	// TODO : calculate visual odometry limits

	const bool relying_on_rangefinder = _control_status.flags.rng_hgt && !_params.range_aid;

	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	// Do not require limiting by default
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max = NAN;

	// Keep within range sensor limit when using rangefinder as primary height source
	if (relying_on_rangefinder) {
		*vxy_max = NAN;
		*vz_max = NAN;
		*hagl_min = rangefinder_hagl_min;
		*hagl_max = rangefinder_hagl_max;
	}

	// Keep within flow AND range sensor limits when exclusively using optical flow
	if (relying_on_optical_flow) {
		*vxy_max = flow_vxy_max;
		*vz_max = NAN;
		*hagl_min = fmaxf(rangefinder_hagl_min, flow_hagl_min);
		*hagl_max = fminf(rangefinder_hagl_max, flow_hagl_max);
	}

}

bool Ekf::reset_imu_bias()
{
	if (_imu_sample_delayed.time_us - _last_imu_bias_cov_reset_us < (uint64_t)10e6) {
		return false;

	}

	// Zero the delta angle and delta velocity bias states
	_state.delta_ang_bias.zero();
	_state.delta_vel_bias.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<3>(10, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
	P.uncorrelateCovarianceSetVariance<3>(13, sq(_params.switch_on_accel_bias * FILTER_UPDATE_PERIOD_S));
	_last_imu_bias_cov_reset_us = _imu_sample_delayed.time_us;

	// Set previous frame values
	_prev_dvel_bias_var = P.slice<3,3>(13,13).diag();

	return true;
}

// get EKF innovation consistency check status information comprising of:
// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
// Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
void Ekf::get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas, float &hagl, float &beta)
{
	// return the integer bitmask containing the consistency check pass/fail status
	status = _innov_check_fail_status.value;
	// return the largest magnetometer innovation test ratio
	mag = sqrtf(math::max(_yaw_test_ratio, math::max(math::max(_mag_test_ratio[0], _mag_test_ratio[1]), _mag_test_ratio[2])));
	// return the largest velocity innovation test ratio
	vel = math::max(sqrtf(math::max(_gps_vel_test_ratio(0), _gps_vel_test_ratio(1))),
			sqrtf(math::max(_ev_vel_test_ratio(0), _ev_vel_test_ratio(1))));
	// return the largest position innovation test ratio
	pos = math::max(sqrtf(_gps_pos_test_ratio(0)),sqrtf(_ev_pos_test_ratio(0)));

	// return the vertical position innovation test ratio
	hgt = sqrtf(_gps_pos_test_ratio(0));
	// return the airspeed fusion innovation test ratio
	tas = sqrtf(_tas_test_ratio);
	// return the terrain height innovation test ratio
	hagl = sqrtf(_hagl_test_ratio);
	// return the synthetic sideslip innovation test ratio
	beta = sqrtf(_beta_test_ratio);
}

// return a bitmask integer that describes which state estimates are valid
void Ekf::get_ekf_soln_status(uint16_t *status)
{
	ekf_solution_status soln_status;
	// TODO: Is this accurate enough?
	soln_status.flags.attitude = _control_status.flags.tilt_align && _control_status.flags.yaw_align && (_fault_status.value == 0);
	soln_status.flags.velocity_horiz = (isHorizontalAidingActive() || (_control_status.flags.fuse_beta && _control_status.flags.fuse_aspd)) && (_fault_status.value == 0);
	soln_status.flags.velocity_vert = (_control_status.flags.baro_hgt || _control_status.flags.ev_hgt || _control_status.flags.gps_hgt || _control_status.flags.rng_hgt) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_rel = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_abs = (_control_status.flags.gps || _control_status.flags.ev_pos) && (_fault_status.value == 0);
	soln_status.flags.pos_vert_abs = soln_status.flags.velocity_vert;
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
	soln_status.flags.const_pos_mode = !soln_status.flags.velocity_horiz;
	soln_status.flags.pred_pos_horiz_rel = soln_status.flags.pos_horiz_rel;
	soln_status.flags.pred_pos_horiz_abs = soln_status.flags.pos_horiz_abs;
	bool gps_vel_innov_bad = (_gps_vel_test_ratio(0) > 1.0f) || (_gps_vel_test_ratio(1) > 1.0f);
	bool gps_pos_innov_bad = (_gps_pos_test_ratio(0) > 1.0f);
	bool mag_innov_good = (_mag_test_ratio[0] < 1.0f) && (_mag_test_ratio[1] < 1.0f) && (_mag_test_ratio[2] < 1.0f) && (_yaw_test_ratio < 1.0f);
	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad) && mag_innov_good;
	soln_status.flags.accel_error = _bad_vert_accel_detected;
	*status = soln_status.value;
}

// fuse measurement
void Ekf::fuse(float *K, float innovation)
{
	for (unsigned i = 0; i < 4; i++) {
		_state.quat_nominal(i) = _state.quat_nominal(i) - K[i] * innovation;
	}

	_state.quat_nominal.normalize();

	for (unsigned i = 0; i < 3; i++) {
		_state.vel(i) = _state.vel(i) - K[i + 4] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.pos(i) = _state.pos(i) - K[i + 7] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.delta_ang_bias(i) = _state.delta_ang_bias(i) - K[i + 10] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.delta_vel_bias(i) = _state.delta_vel_bias(i) - K[i + 13] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.mag_I(i) = _state.mag_I(i) - K[i + 16] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.mag_B(i) = _state.mag_B(i) - K[i + 19] * innovation;
	}

	for (unsigned i = 0; i < 2; i++) {
		_state.wind_vel(i) = _state.wind_vel(i) - K[i + 22] * innovation;
	}
}



void Ekf::uncorrelateQuatStates()
{
	P.uncorrelateCovariance<4>(0);
}

bool Ekf::global_position_is_valid()
{
	// return true if the origin is set we are not doing unconstrained free inertial navigation
	// and have not started using synthetic position observations to constrain drift
	return (_NED_origin_initialised && !_deadreckon_time_exceeded && !_using_synthetic_position);
}

// return true if we are totally reliant on inertial dead-reckoning for position
void Ekf::update_deadreckoning_status()
{
	bool velPosAiding = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.ev_vel)
			    && (isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
				|| isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max)
				|| isRecent(_time_last_delpos_fuse, _params.no_aid_timeout_max));
	bool optFlowAiding = _control_status.flags.opt_flow && isRecent(_time_last_of_fuse, _params.no_aid_timeout_max);
	bool airDataAiding = _control_status.flags.wind &&
			     isRecent(_time_last_arsp_fuse, _params.no_aid_timeout_max) &&
			     isRecent(_time_last_beta_fuse, _params.no_aid_timeout_max);

	_is_wind_dead_reckoning = !velPosAiding && !optFlowAiding && airDataAiding;
	_is_dead_reckoning = !velPosAiding && !optFlowAiding && !airDataAiding;

	if (!_is_dead_reckoning) {
		_time_last_aiding = _time_last_imu - _params.no_aid_timeout_max;
	}

	// report if we have been deadreckoning for too long
	_deadreckon_time_exceeded = isTimedOut(_time_last_aiding, (uint64_t)_params.valid_timeout_max);
}

// calculate the inverse rotation matrix from a quaternion rotation
// this produces the inverse rotation to that produced by the math library quaternion to Dcmf operator
Matrix3f EstimatorInterface::quat_to_invrotmat(const Quatf &quat)
{
	float q00 = quat(0) * quat(0);
	float q11 = quat(1) * quat(1);
	float q22 = quat(2) * quat(2);
	float q33 = quat(3) * quat(3);
	float q01 = quat(0) * quat(1);
	float q02 = quat(0) * quat(2);
	float q03 = quat(0) * quat(3);
	float q12 = quat(1) * quat(2);
	float q13 = quat(1) * quat(3);
	float q23 = quat(2) * quat(3);

	Matrix3f dcm;
	dcm(0, 0) = q00 + q11 - q22 - q33;
	dcm(1, 1) = q00 - q11 + q22 - q33;
	dcm(2, 2) = q00 - q11 - q22 + q33;
	dcm(1, 0) = 2.0f * (q12 - q03);
	dcm(2, 0) = 2.0f * (q13 + q02);
	dcm(0, 1) = 2.0f * (q12 + q03);
	dcm(2, 1) = 2.0f * (q23 - q01);
	dcm(0, 2) = 2.0f * (q13 - q02);
	dcm(1, 2) = 2.0f * (q23 + q01);

	return dcm;
}

// calculate the variances for the rotation vector equivalent
Vector3f Ekf::calcRotVecVariances()
{
	Vector3f rot_var_vec;
	float q0, q1, q2, q3;

	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float t2 = q0*q0;
	float t3 = acosf(q0);
	float t4 = -t2+1.0f;
	float t5 = t2-1.0f;
	if ((t4 > 1e-9f) && (t5 < -1e-9f)) {
		float t6 = 1.0f/t5;
		float t7 = q1*t6*2.0f;
		float t8 = 1.0f/powf(t4,1.5f);
		float t9 = q0*q1*t3*t8*2.0f;
		float t10 = t7+t9;
		float t11 = 1.0f/sqrtf(t4);
		float t12 = q2*t6*2.0f;
		float t13 = q0*q2*t3*t8*2.0f;
		float t14 = t12+t13;
		float t15 = q3*t6*2.0f;
		float t16 = q0*q3*t3*t8*2.0f;
		float t17 = t15+t16;
		rot_var_vec(0) = t10*(P(0,0)*t10+P(1,0)*t3*t11*2.0f)+t3*t11*(P(0,1)*t10+P(1,1)*t3*t11*2.0f)*2.0f;
		rot_var_vec(1) = t14*(P(0,0)*t14+P(2,0)*t3*t11*2.0f)+t3*t11*(P(0,2)*t14+P(2,2)*t3*t11*2.0f)*2.0f;
		rot_var_vec(2) = t17*(P(0,0)*t17+P(3,0)*t3*t11*2.0f)+t3*t11*(P(0,3)*t17+P(3,3)*t3*t11*2.0f)*2.0f;
	} else {
		rot_var_vec = 4.0f * P.slice<3,3>(1,1).diag();
	}

	return rot_var_vec;
}

// initialise the quaternion covariances using rotation vector variances
void Ekf::initialiseQuatCovariances(Vector3f &rot_vec_var)
{
	// calculate an equivalent rotation vector from the quaternion
	float q0,q1,q2,q3;
	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float delta = 2.0f*acosf(q0);
	float scaler = (delta/sinf(delta*0.5f));
	float rotX = scaler*q1;
	float rotY = scaler*q2;
	float rotZ = scaler*q3;

	// autocode generated using matlab symbolic toolbox
	float t2 = rotX*rotX;
	float t4 = rotY*rotY;
	float t5 = rotZ*rotZ;
	float t6 = t2+t4+t5;
	if (t6 > 1e-9f) {
		float t7 = sqrtf(t6);
		float t8 = t7*0.5f;
		float t3 = sinf(t8);
		float t9 = t3*t3;
		float t10 = 1.0f/t6;
		float t11 = 1.0f/sqrtf(t6);
		float t12 = cosf(t8);
		float t13 = 1.0f/powf(t6,1.5f);
		float t14 = t3*t11;
		float t15 = rotX*rotY*t3*t13;
		float t16 = rotX*rotZ*t3*t13;
		float t17 = rotY*rotZ*t3*t13;
		float t18 = t2*t10*t12*0.5f;
		float t27 = t2*t3*t13;
		float t19 = t14+t18-t27;
		float t23 = rotX*rotY*t10*t12*0.5f;
		float t28 = t15-t23;
		float t20 = rotY*rot_vec_var(1)*t3*t11*t28*0.5f;
		float t25 = rotX*rotZ*t10*t12*0.5f;
		float t31 = t16-t25;
		float t21 = rotZ*rot_vec_var(2)*t3*t11*t31*0.5f;
		float t22 = t20+t21-rotX*rot_vec_var(0)*t3*t11*t19*0.5f;
		float t24 = t15-t23;
		float t26 = t16-t25;
		float t29 = t4*t10*t12*0.5f;
		float t34 = t3*t4*t13;
		float t30 = t14+t29-t34;
		float t32 = t5*t10*t12*0.5f;
		float t40 = t3*t5*t13;
		float t33 = t14+t32-t40;
		float t36 = rotY*rotZ*t10*t12*0.5f;
		float t39 = t17-t36;
		float t35 = rotZ*rot_vec_var(2)*t3*t11*t39*0.5f;
		float t37 = t15-t23;
		float t38 = t17-t36;
		float t41 = rot_vec_var(0)*(t15-t23)*(t16-t25);
		float t42 = t41-rot_vec_var(1)*t30*t39-rot_vec_var(2)*t33*t39;
		float t43 = t16-t25;
		float t44 = t17-t36;

		// zero all the quaternion covariances
		P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);


		// Update the quaternion internal covariances using auto-code generated using matlab symbolic toolbox
		P(0,0) = rot_vec_var(0)*t2*t9*t10*0.25f+rot_vec_var(1)*t4*t9*t10*0.25f+rot_vec_var(2)*t5*t9*t10*0.25f;
		P(0,1) = t22;
		P(0,2) = t35+rotX*rot_vec_var(0)*t3*t11*(t15-rotX*rotY*t10*t12*0.5f)*0.5f-rotY*rot_vec_var(1)*t3*t11*t30*0.5f;
		P(0,3) = rotX*rot_vec_var(0)*t3*t11*(t16-rotX*rotZ*t10*t12*0.5f)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-rotY*rotZ*t10*t12*0.5f)*0.5f-rotZ*rot_vec_var(2)*t3*t11*t33*0.5f;
		P(1,0) = t22;
		P(1,1) = rot_vec_var(0)*(t19*t19)+rot_vec_var(1)*(t24*t24)+rot_vec_var(2)*(t26*t26);
		P(1,2) = rot_vec_var(2)*(t16-t25)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(1,3) = rot_vec_var(1)*(t15-t23)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(2,0) = t35-rotY*rot_vec_var(1)*t3*t11*t30*0.5f+rotX*rot_vec_var(0)*t3*t11*(t15-t23)*0.5f;
		P(2,1) = rot_vec_var(2)*(t16-t25)*(t17-t36)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(2,2) = rot_vec_var(1)*(t30*t30)+rot_vec_var(0)*(t37*t37)+rot_vec_var(2)*(t38*t38);
		P(2,3) = t42;
		P(3,0) = rotZ*rot_vec_var(2)*t3*t11*t33*(-0.5f)+rotX*rot_vec_var(0)*t3*t11*(t16-t25)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-t36)*0.5f;
		P(3,1) = rot_vec_var(1)*(t15-t23)*(t17-t36)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(3,2) = t42;
		P(3,3) = rot_vec_var(2)*(t33*t33)+rot_vec_var(0)*(t43*t43)+rot_vec_var(1)*(t44*t44);

	} else {
		// the equations are badly conditioned so use a small angle approximation
		P.uncorrelateCovarianceSetVariance<1>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<3>(1, 0.25f * rot_vec_var);
	}
}

void Ekf::setControlBaroHeight()
{
	_control_status.flags.baro_hgt = true;

	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlRangeHeight()
{
	_control_status.flags.rng_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlGPSHeight()
{
	_control_status.flags.gps_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlEVHeight()
{
	_control_status.flags.ev_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
}

void Ekf::stopMagFusion()
{
	stopMag3DFusion();
	stopMagHdgFusion();
	clearMagCov();
}

void Ekf::stopMag3DFusion()
{
	// save covariance data for re-use if currently doing 3-axis fusion
	if (_control_status.flags.mag_3D) {
		saveMagCovData();
		_control_status.flags.mag_3D = false;
	}
}

void Ekf::stopMagHdgFusion()
{
	_control_status.flags.mag_hdg = false;
}

void Ekf::startMagHdgFusion()
{
	stopMag3DFusion();
	_control_status.flags.mag_hdg = true;
}

void Ekf::startMag3DFusion()
{
	if (!_control_status.flags.mag_3D) {
		stopMagHdgFusion();
		zeroMagCov();
		loadMagCovData();
		_control_status.flags.mag_3D = true;
	}
}

// update the rotation matrix which rotates EV measurements into the EKF's navigation frame
void Ekf::calcExtVisRotMat()
{
	// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
	const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
	_R_ev_to_ekf = Dcmf(q_error);
}

// return the quaternions for the rotation from External Vision system reference frame to the EKF reference frame
void Ekf::get_ev2ekf_quaternion(float *quat)
{
	const Quatf quat_ev2ekf(_R_ev_to_ekf);

	for (unsigned i = 0; i < 4; i++) {
		quat[i] = quat_ev2ekf(i);
	}
}

// Increase the yaw error variance of the quaternions
// Argument is additional yaw variance in rad**2
void Ekf::increaseQuatYawErrVariance(float yaw_variance)
{
	// See DeriveYawResetEquations.m for derivation which produces code fragments in C_code4.txt file
	// The auto-code was cleaned up and had terms multiplied by zero removed to give the following:

	// Intermediate variables
	float SG[3];
	SG[0] = sq(_state.quat_nominal(0)) - sq(_state.quat_nominal(1)) - sq(_state.quat_nominal(2)) + sq(_state.quat_nominal(3));
	SG[1] = 2*_state.quat_nominal(0)*_state.quat_nominal(2) - 2*_state.quat_nominal(1)*_state.quat_nominal(3);
	SG[2] = 2*_state.quat_nominal(0)*_state.quat_nominal(1) + 2*_state.quat_nominal(2)*_state.quat_nominal(3);

	float SQ[4];
	SQ[0] = 0.5f * ((_state.quat_nominal(1)*SG[0]) - (_state.quat_nominal(0)*SG[2]) + (_state.quat_nominal(3)*SG[1]));
	SQ[1] = 0.5f * ((_state.quat_nominal(0)*SG[1]) - (_state.quat_nominal(2)*SG[0]) + (_state.quat_nominal(3)*SG[2]));
	SQ[2] = 0.5f * ((_state.quat_nominal(3)*SG[0]) - (_state.quat_nominal(1)*SG[1]) + (_state.quat_nominal(2)*SG[2]));
	SQ[3] = 0.5f * ((_state.quat_nominal(0)*SG[0]) + (_state.quat_nominal(1)*SG[2]) + (_state.quat_nominal(2)*SG[1]));

	// Limit yaw variance increase to prevent a badly conditioned covariance matrix
	yaw_variance = fminf(yaw_variance, 1.0e-2f);

	// Add covariances for additonal yaw uncertainty to existing covariances.
	// This assumes that the additional yaw error is uncorrrelated to existing errors
	P(0,0) += yaw_variance*sq(SQ[2]);
	P(0,1) += yaw_variance*SQ[1]*SQ[2];
	P(1,1) += yaw_variance*sq(SQ[1]);
	P(0,2) += yaw_variance*SQ[0]*SQ[2];
	P(1,2) += yaw_variance*SQ[0]*SQ[1];
	P(2,2) += yaw_variance*sq(SQ[0]);
	P(0,3) -= yaw_variance*SQ[2]*SQ[3];
	P(1,3) -= yaw_variance*SQ[1]*SQ[3];
	P(2,3) -= yaw_variance*SQ[0]*SQ[3];
	P(3,3) += yaw_variance*sq(SQ[3]);
	P(1,0) += yaw_variance*SQ[1]*SQ[2];
	P(2,0) += yaw_variance*SQ[0]*SQ[2];
	P(2,1) += yaw_variance*SQ[0]*SQ[1];
	P(3,0) -= yaw_variance*SQ[2]*SQ[3];
	P(3,1) -= yaw_variance*SQ[1]*SQ[3];
	P(3,2) -= yaw_variance*SQ[0]*SQ[3];
}

// save covariance data for re-use when auto-switching between heading and 3-axis fusion
void Ekf::saveMagCovData()
{
	// save variances for the D earth axis and XYZ body axis field
	for (uint8_t index = 0; index <= 3; index ++) {
		_saved_mag_bf_variance[index] = P(index + 18,index + 18);
	}

	// save the NE axis covariance sub-matrix
	for (uint8_t row = 0; row <= 1; row ++) {
		for (uint8_t col = 0; col <= 1; col ++) {
			_saved_mag_ef_covmat[row][col] = P(row + 16,col + 16);
		}
	}
}

void Ekf::loadMagCovData()
{
	// re-instate variances for the D earth axis and XYZ body axis field
	for (uint8_t index = 0; index <= 3; index ++) {
		P(index + 18,index + 18) = _saved_mag_bf_variance[index];
	}
	// re-instate the NE axis covariance sub-matrix
	for (uint8_t row = 0; row <= 1; row ++) {
		for (uint8_t col = 0; col <= 1; col ++) {
			P(row + 16,col + 16) = _saved_mag_ef_covmat[row][col];
		}
	}
}

float Ekf::kahanSummation(float sum_previous, float input, float &accumulator) const
{
	float y = input - accumulator;
	float t = sum_previous + y;
	accumulator = (t - sum_previous) - y;
	return t;
}

void Ekf::stopGpsFusion()
{
	stopGpsPosFusion();
	stopGpsVelFusion();
	stopGpsYawFusion();
}

void Ekf::stopGpsPosFusion()
{
	_control_status.flags.gps = false;
	_control_status.flags.gps_hgt = false;
	_gps_pos_innov.setZero();
	_gps_pos_innov_var.setZero();
	_gps_pos_test_ratio.setZero();
}

void Ekf::stopGpsVelFusion()
{
	_gps_vel_innov.setZero();
	_gps_vel_innov_var.setZero();
	_gps_vel_test_ratio.setZero();
}

void Ekf::stopGpsYawFusion()
{
	_control_status.flags.gps_yaw = false;
}

void Ekf::stopEvFusion()
{
	stopEvPosFusion();
	stopEvVelFusion();
	stopEvYawFusion();
}

void Ekf::stopEvPosFusion()
{
	_control_status.flags.ev_pos = false;
	_ev_pos_innov.setZero();
	_ev_pos_innov_var.setZero();
	_ev_pos_test_ratio.setZero();
}

void Ekf::stopEvVelFusion()
{
	_control_status.flags.ev_vel = false;
	_ev_vel_innov.setZero();
	_ev_vel_innov_var.setZero();
	_ev_vel_test_ratio.setZero();
}

void Ekf::stopEvYawFusion()
{
	_control_status.flags.ev_yaw = false;
}

void Ekf::stopAuxVelFusion()
{
	_aux_vel_innov.setZero();
	_aux_vel_innov_var.setZero();
	_aux_vel_test_ratio.setZero();
}

void Ekf::stopFlowFusion()
{
	_control_status.flags.opt_flow = false;
	memset(_flow_innov,0.0f,sizeof(_flow_innov));
	memset(_flow_innov_var,0.0f,sizeof(_flow_innov_var));
	memset(&_optflow_test_ratio,0.0f,sizeof(_optflow_test_ratio));
}

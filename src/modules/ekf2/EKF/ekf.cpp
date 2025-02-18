/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
	if (!_initialised) {
		_initialised = initialise_interface(timestamp);
		reset();
	}

	return _initialised;
}

void Ekf::reset()
{
	ECL_INFO("reset");

	_state.quat_nominal.setIdentity();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.accel_bias.setZero();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_I.setZero();
	_state.mag_B.setZero();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_state.wind_vel.setZero();
#endif // CONFIG_EKF2_WIND
	//
#if defined(CONFIG_EKF2_TERRAIN)
	// assume a ground clearance
	_state.terrain = -_gpos.altitude() + _params.rng_gnd_clearance;
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);
	_range_sensor.setMaxFogDistance(_params.rng_fog);
#endif // CONFIG_EKF2_RANGE_FINDER

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

#if defined(CONFIG_EKF2_GNSS)
	resetGpsDriftCheckFilters();
	_gps_checks_passed = false;
#endif // CONFIG_EKF2_GNSS
	_local_origin_alt = NAN;

	_output_predictor.reset();

	// Ekf private fields
	_time_last_horizontal_aiding = 0;
	_time_last_v_pos_aiding = 0;
	_time_last_v_vel_aiding = 0;

	_time_last_hor_pos_fuse = 0;
	_time_last_hgt_fuse = 0;
	_time_last_hor_vel_fuse = 0;
	_time_last_ver_vel_fuse = 0;
	_time_last_heading_fuse = 0;
	_time_last_terrain_fuse = 0;

	_last_known_gpos.setZero();

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_counter = 0;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_mag_counter = 0;
#endif // CONFIG_EKF2_MAGNETOMETER

	_time_bad_vert_accel = 0;
	_time_good_vert_accel = 0;

	for (auto &clip_count : _clip_counter) {
		clip_count = 0;
	}

	_zero_velocity_update.reset();

	updateParameters();
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
		_imu_updated = false;

		// get the oldest IMU data from the buffer
		// TODO: explicitly pop at desired time horizon
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		// calculate an average filter update time
		//  filter and limit input between -50% and +100% of nominal value
		float input = 0.5f * (imu_sample_delayed.delta_vel_dt + imu_sample_delayed.delta_ang_dt);
		float filter_update_s = 1e-6f * _params.filter_update_interval_us;
		_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);

		updateIMUBiasInhibit(imu_sample_delayed);

		// perform state and covariance prediction for the main filter
		predictCovariance(imu_sample_delayed);
		predictState(imu_sample_delayed);

		// control fusion of observation data
		controlFusionModes(imu_sample_delayed);

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, _state.quat_nominal, _state.vel, _gpos,
						      _state.gyro_bias, _state.accel_bias);

		return true;
	}

	return false;
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

	if (!initialiseTilt()) {
		return false;
	}

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

	// reset the output predictor state history to match the EKF initial values
	_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _gpos);

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

	// get initial tilt estimate from delta velocity vector, assuming vehicle is static
	_state.quat_nominal = Quatf(_accel_lpf.getState(), Vector3f(0.f, 0.f, -1.f));
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState(const imuSample &imu_delayed)
{
	if (std::fabs(_gpos.latitude_rad() - _earth_rate_lat_ref_rad) > math::radians(1.0)) {
		_earth_rate_lat_ref_rad = _gpos.latitude_rad();
		_earth_rate_NED = calcEarthRateNED((float)_earth_rate_lat_ref_rad);
	}

	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity, Coriolis and transport rate
	const Vector3f gravity_acceleration(0.f, 0.f, CONSTANTS_ONE_G); // simplistic model
	const Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);
	const Vector3f transport_rate = -_gpos.computeAngularRateNavFrame(vel_last).cross(vel_last);
	_state.vel += (gravity_acceleration + coriolis_acceleration + transport_rate) * imu_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
	_state.pos(2) = -_gpos.altitude();

	// constrain states
	_state.vel = matrix::constrain(_state.vel, -_params.velocity_limit, _params.velocity_limit);

	// calculate a filtered horizontal acceleration this are used for manoeuvre detection elsewhere
	_accel_horiz_lpf.update(corrected_delta_vel_ef.xy() / imu_delayed.delta_vel_dt, imu_delayed.delta_vel_dt);
}

bool Ekf::resetGlobalPosToExternalObservation(const double latitude, const double longitude, const float altitude,
		const float eph,
		const float epv, uint64_t timestamp_observation)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	if (!_local_origin_lat_lon.isInitialized()) {
		if (!resetLatLonTo(latitude, longitude, sq(eph))) {
			return false;
		}

		initialiseAltitudeTo(altitude, sq(epv));

		return true;
	}

	Vector3f pos_correction;

	// apply a first order correction using velocity at the delayed time horizon and the delta time
	if ((timestamp_observation > 0) && isLocalHorizontalPositionValid()) {

		timestamp_observation = math::min(_time_latest_us, timestamp_observation);

		float dt_us;

		if (_time_delayed_us >= timestamp_observation) {
			dt_us = static_cast<float>(_time_delayed_us - timestamp_observation);

		} else {
			dt_us = -static_cast<float>(timestamp_observation - _time_delayed_us);
		}

		const float dt_s = dt_us * 1e-6f;
		pos_correction = _state.vel * dt_s;
	}

	LatLonAlt gpos(latitude, longitude, altitude);
	bool alt_valid = true;

	if (!checkAltitudeValidity(gpos.altitude())) {
		gpos.setAltitude(_gpos.altitude());
		alt_valid = false;
	}

	const LatLonAlt gpos_corrected = gpos + pos_correction;

	{
		const float obs_var = math::max(sq(eph), sq(0.01f));

		const Vector2f innov = (_gpos - gpos_corrected).xy();
		const Vector2f innov_var = Vector2f(getStateVariance<State::pos>()) + obs_var;

		const float sq_gate = sq(5.f); // magic hardcoded gate
		const float test_ratio = sq(innov(0)) / (sq_gate * innov_var(0)) + sq(innov(1)) / (sq_gate * innov_var(1));

		const bool innov_rejected = (test_ratio > 1.f);

		if (!_control_status.flags.in_air || (eph > 0.f && eph < 1.f) || innov_rejected) {
			// when on ground or accuracy chosen to be very low, we hard reset position
			// this allows the user to still send hard resets at any time
			ECL_INFO("reset position to external observation");
			_information_events.flags.reset_pos_to_ext_obs = true;

			resetHorizontalPositionTo(gpos_corrected.latitude_deg(), gpos_corrected.longitude_deg(), obs_var);
			_last_known_gpos.setLatLon(gpos_corrected);

		} else {
			ECL_INFO("fuse external observation as position measurement");
			fuseDirectStateMeasurement(innov(0), innov_var(0), obs_var, State::pos.idx + 0);
			fuseDirectStateMeasurement(innov(1), innov_var(1), obs_var, State::pos.idx + 1);

			// Use the reset counters to inform the controllers about a potential large position jump
			// TODO: compute the actual position change
			_state_reset_status.reset_count.posNE++;
			_state_reset_status.posNE_change.zero();

			_time_last_hor_pos_fuse = _time_delayed_us;
			_last_known_gpos.setLatLon(gpos_corrected);
		}
	}

	if (alt_valid) {
		const float obs_var = math::max(sq(epv), sq(0.01f));

		ECL_INFO("reset height to external observation");
		initialiseAltitudeTo(gpos_corrected.altitude(), obs_var);
		_last_known_gpos.setAltitude(gpos_corrected.altitude());
	}

	return true;
}

void Ekf::updateParameters()
{
	_params.gyro_noise = math::constrain(_params.gyro_noise, 0.f, 1.f);
	_params.accel_noise = math::constrain(_params.accel_noise, 0.f, 1.f);

	_params.gyro_bias_p_noise = math::constrain(_params.gyro_bias_p_noise, 0.f, 1.f);
	_params.accel_bias_p_noise = math::constrain(_params.accel_bias_p_noise, 0.f, 1.f);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_params.mage_p_noise = math::constrain(_params.mage_p_noise, 0.f, 1.f);
	_params.magb_p_noise = math::constrain(_params.magb_p_noise, 0.f, 1.f);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_params.wind_vel_nsd = math::constrain(_params.wind_vel_nsd, 0.f, 1.f);
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.updateParameters();
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
}

template<typename T>
static void printRingBuffer(const char *name, RingBuffer<T> *rb)
{
	if (rb) {
		printf("%s: %d/%d entries (%d/%d Bytes) (%zu Bytes per entry)\n",
		       name,
		       rb->entries(), rb->get_length(), rb->get_used_size(), rb->get_total_size(),
		       sizeof(T));
	}
}

void Ekf::print_status()
{
	printf("\nStates: (%.4f seconds ago)\n", (_time_latest_us - _time_delayed_us) * 1e-6);
	printf("Orientation (%d-%d): [%.3f, %.3f, %.3f, %.3f] (Euler [%.1f, %.1f, %.1f] deg) var: [%.1e, %.1e, %.1e]\n",
	       State::quat_nominal.idx, State::quat_nominal.idx + State::quat_nominal.dof - 1,
	       (double)_state.quat_nominal(0), (double)_state.quat_nominal(1), (double)_state.quat_nominal(2),
	       (double)_state.quat_nominal(3),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).phi()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).theta()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).psi()),
	       (double)getStateVariance<State::quat_nominal>()(0), (double)getStateVariance<State::quat_nominal>()(1),
	       (double)getStateVariance<State::quat_nominal>()(2)
	      );

	printf("Velocity (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::vel.idx, State::vel.idx + State::vel.dof - 1,
	       (double)_state.vel(0), (double)_state.vel(1), (double)_state.vel(2),
	       (double)getStateVariance<State::vel>()(0), (double)getStateVariance<State::vel>()(1),
	       (double)getStateVariance<State::vel>()(2)
	      );

	const Vector3f position = getPosition();
	printf("Position (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::pos.idx, State::pos.idx + State::pos.dof - 1,
	       (double)position(0), (double)position(1), (double) position(2),
	       (double)getStateVariance<State::pos>()(0), (double)getStateVariance<State::pos>()(1),
	       (double)getStateVariance<State::pos>()(2)
	      );

	printf("Gyro Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::gyro_bias.idx, State::gyro_bias.idx + State::gyro_bias.dof - 1,
	       (double)_state.gyro_bias(0), (double)_state.gyro_bias(1), (double)_state.gyro_bias(2),
	       (double)getStateVariance<State::gyro_bias>()(0), (double)getStateVariance<State::gyro_bias>()(1),
	       (double)getStateVariance<State::gyro_bias>()(2)
	      );

	printf("Accel Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::accel_bias.idx, State::accel_bias.idx + State::accel_bias.dof - 1,
	       (double)_state.accel_bias(0), (double)_state.accel_bias(1), (double)_state.accel_bias(2),
	       (double)getStateVariance<State::accel_bias>()(0), (double)getStateVariance<State::accel_bias>()(1),
	       (double)getStateVariance<State::accel_bias>()(2)
	      );

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printf("Magnetic Field (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_I.idx, State::mag_I.idx + State::mag_I.dof - 1,
	       (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
	       (double)getStateVariance<State::mag_I>()(0), (double)getStateVariance<State::mag_I>()(1),
	       (double)getStateVariance<State::mag_I>()(2)
	      );

	printf("Magnetic Bias (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_B.idx, State::mag_B.idx + State::mag_B.dof - 1,
	       (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2),
	       (double)getStateVariance<State::mag_B>()(0), (double)getStateVariance<State::mag_B>()(1),
	       (double)getStateVariance<State::mag_B>()(2)
	      );
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	printf("Wind velocity (%d-%d): [%.3f, %.3f] var: [%.1e, %.1e]\n",
	       State::wind_vel.idx, State::wind_vel.idx + State::wind_vel.dof - 1,
	       (double)_state.wind_vel(0), (double)_state.wind_vel(1),
	       (double)getStateVariance<State::wind_vel>()(0), (double)getStateVariance<State::wind_vel>()(1)
	      );
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	printf("Terrain position (%d): %.3f var: %.1e\n",
	       State::terrain.idx,
	       (double)_state.terrain,
	       (double)getStateVariance<State::terrain>()(0)
	      );
#endif // CONFIG_EKF2_TERRAIN

	printf("\nP:\n");
	P.print();

	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);
	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	printRingBuffer("IMU buffer", &_imu_buffer);
	printRingBuffer("system flag buffer", _system_flag_buffer);

#if defined(CONFIG_EKF2_AIRSPEED)
	printRingBuffer("airspeed buffer", _airspeed_buffer);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
	printRingBuffer("aux vel buffer", _auxvel_buffer);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
	printRingBuffer("baro buffer", _baro_buffer);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	printRingBuffer("drag buffer", _drag_buffer);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	printRingBuffer("ext vision buffer", _ext_vision_buffer);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	printRingBuffer("gps buffer", _gps_buffer);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printRingBuffer("mag buffer", _mag_buffer);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	printRingBuffer("flow buffer", _flow_buffer);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
	printRingBuffer("range buffer", _range_buffer);
#endif // CONFIG_EKF2_RANGE_FINDER


	_output_predictor.print_status();
}

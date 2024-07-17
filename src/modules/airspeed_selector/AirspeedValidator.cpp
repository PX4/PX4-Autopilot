/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file AirspeedValidator.cpp
 * Estimates airspeed scale error (from indicated to calibrated airspeed), performs
 * checks on airspeed measurement input and reports airspeed valid or invalid.
 */

#include "AirspeedValidator.hpp"

AirspeedValidator::AirspeedValidator()
{
	reset_CAS_scale_check(); //this resets all elements of the Vectors to NAN
}

void
AirspeedValidator::update_airspeed_validator(const airspeed_validator_update_data &input_data)
{
	// get indicated airspeed from input data (raw airspeed)
	_IAS = input_data.airspeed_indicated_raw;

	update_CAS_scale_validated(input_data.gnss_valid, input_data.ground_velocity, input_data.airspeed_true_raw);
	update_CAS_scale_applied();
	update_CAS_TAS(input_data.air_pressure_pa, input_data.air_temperature_celsius);
	update_wind_estimator(input_data.timestamp, input_data.airspeed_true_raw, input_data.gnss_valid,
			      input_data.ground_velocity, input_data.lpos_evh, input_data.lpos_evv, input_data.q_att);
	update_in_fixed_wing_flight(input_data.in_fixed_wing_flight);
	check_airspeed_data_stuck(input_data.timestamp);
	check_load_factor(input_data.accel_z);
	check_airspeed_innovation(input_data.timestamp, input_data.vel_test_ratio, input_data.hdg_test_ratio,
				  input_data.ground_velocity, input_data.gnss_valid);
	check_first_principle(input_data.timestamp, input_data.fixed_wing_tecs_throttle,
			      input_data.fixed_wing_tecs_throttle_trim, input_data.tecs_timestamp, input_data.q_att);
	update_airspeed_valid_status(input_data.timestamp);
}

void
AirspeedValidator::reset_airspeed_to_invalid(const uint64_t timestamp)
{
	_airspeed_valid = false;
	_time_checks_failed = timestamp;
}

void
AirspeedValidator::update_wind_estimator(const uint64_t time_now_usec, float airspeed_true_raw, bool gnss_valid,
		const matrix::Vector3f &vI, float lpos_evh, float lpos_evv, const Quatf &q_att)
{
	_wind_estimator.update(time_now_usec);

	if (gnss_valid && _in_fixed_wing_flight) {

		// airspeed fusion (with raw TAS)
		const float hor_vel_variance =  lpos_evh * lpos_evh;
		_wind_estimator.fuse_airspeed(time_now_usec, airspeed_true_raw, vI, hor_vel_variance, q_att);

		// sideslip fusion
		_wind_estimator.fuse_beta(time_now_usec, vI, hor_vel_variance, q_att);
	}
}

// this function returns the current states of the wind estimator to be published in the airspeed module
airspeed_wind_s
AirspeedValidator::get_wind_estimator_states(uint64_t timestamp)
{
	airspeed_wind_s wind_est = {};

	wind_est.timestamp = timestamp;
	wind_est.windspeed_north = _wind_estimator.get_wind()(0);
	wind_est.windspeed_east = _wind_estimator.get_wind()(1);
	wind_est.variance_north = _wind_estimator.get_wind_var()(0);
	wind_est.variance_east = _wind_estimator.get_wind_var()(1);
	wind_est.tas_innov = _wind_estimator.get_tas_innov();
	wind_est.tas_innov_var = _wind_estimator.get_tas_innov_var();
	wind_est.beta_innov = _wind_estimator.get_beta_innov();
	wind_est.beta_innov_var = _wind_estimator.get_beta_innov_var();
	wind_est.tas_scale_raw = _wind_estimator.get_tas_scale();
	wind_est.tas_scale_raw_var = _wind_estimator.get_tas_scale_var();
	wind_est.tas_scale_validated = _CAS_scale_validated;
	return wind_est;
}

void
AirspeedValidator::update_CAS_scale_validated(bool gnss_valid, const matrix::Vector3f &vI, float airspeed_true_raw)
{
	if (!_in_fixed_wing_flight || !gnss_valid) {
		return;
	}

	// reset every 100s as we assume that all the samples for current check are in similar wind conditions
	if (hrt_elapsed_time(&_begin_current_scale_check) > 100_s) {
		reset_CAS_scale_check();
	}

	const float course_over_ground_rad = matrix::wrap_2pi(atan2f(vI(0), vI(1)));
	const int segment_index = int(SCALE_CHECK_SAMPLES * course_over_ground_rad / (2.f * M_PI_F));

	_scale_check_groundspeed(segment_index) = vI.norm();
	_scale_check_TAS(segment_index) = airspeed_true_raw;

	// run check if all segments are filled
	if (_scale_check_groundspeed.isAllFinite()) {
		float ground_speed_sum = 0.f;
		float TAS_sum = 0.f;

		for (int i = 0; i < SCALE_CHECK_SAMPLES; i++) {
			ground_speed_sum += _scale_check_groundspeed(i);
			TAS_sum += _scale_check_TAS(i);
		}

		const float TAS_to_groundspeed_error_current = ground_speed_sum - TAS_sum * _CAS_scale_validated;
		const float TAS_to_groundspeed_error_new = ground_speed_sum - TAS_sum * _wind_estimator.get_tas_scale();

		// check passes if the average airspeed with the scale applied is closer to groundspeed than without
		if (fabsf(TAS_to_groundspeed_error_new) < fabsf(TAS_to_groundspeed_error_current)) {

			// constrain the scale update to max 0.05 at a time
			const float new_scale_constrained = math::constrain(_wind_estimator.get_tas_scale(), _CAS_scale_validated - 0.05f,
							    _CAS_scale_validated + 0.05f);

			_CAS_scale_validated = new_scale_constrained;
		}

		reset_CAS_scale_check();
	}
}

void
AirspeedValidator::reset_CAS_scale_check()
{

	_scale_check_groundspeed.setAll(NAN);
	_scale_check_TAS.setAll(NAN);

	_begin_current_scale_check = hrt_absolute_time();
}

void
AirspeedValidator::update_CAS_scale_applied()
{
	switch (_tas_scale_apply) {
	default:

	/* fallthrough */
	case 0:

	/* fallthrough */
	case 1:
		_CAS_scale_applied = _tas_scale_init;
		break;

	case 2:
		_CAS_scale_applied = _CAS_scale_validated;
		break;
	}
}

void
AirspeedValidator::update_CAS_TAS(float air_pressure_pa, float air_temperature_celsius)
{
	_CAS = calc_CAS_from_IAS(_IAS, _CAS_scale_applied);
	_TAS = calc_TAS_from_CAS(_CAS, air_pressure_pa, air_temperature_celsius);
}

void
AirspeedValidator::check_airspeed_data_stuck(uint64_t time_now)
{
	// Data stuck test: trigger when IAS is not changing for DATA_STUCK_TIMEOUT (2s) when in fixed-wing flight.
	// Only consider fixed-wing flight as some airspeed sensors have a very low resolution around 0m/s and
	// can output the exact same value for several seconds then.

	if (!_data_stuck_check_enabled) {
		_data_stuck_test_failed = false;
		return;
	}

	if (fabsf(_IAS - _IAS_prev) > FLT_EPSILON || _time_last_unequal_data == 0) {
		_time_last_unequal_data = time_now;
		_IAS_prev = _IAS;
	}

	_data_stuck_test_failed = hrt_elapsed_time(&_time_last_unequal_data) > DATA_STUCK_TIMEOUT && _in_fixed_wing_flight;
}

void
AirspeedValidator::check_airspeed_innovation(uint64_t time_now, float estimator_status_vel_test_ratio,
		float estimator_status_hdg_test_ratio, const matrix::Vector3f &vI, bool gnss_valid)
{
	// Check normalised innovation levels with requirement for continuous data and use of hysteresis
	// to prevent false triggering.

	if (_wind_estimator.get_wind_estimator_reset()) {
		_time_wind_estimator_initialized = time_now;
	}

	// reset states if check is disabled, we are not flying or wind estimator was just initialized/reset
	if (!_innovation_check_enabled || !_in_fixed_wing_flight || (time_now - _time_wind_estimator_initialized) < 5_s) {
		_innovations_check_failed = false;
		_aspd_innov_integ_state = 0.f;

	} else if (!gnss_valid || estimator_status_vel_test_ratio > 1.f || estimator_status_hdg_test_ratio > 1.f) {
		//nav velocity data is likely not good
		//don't run the test but don't reset the check if it had previously failed when nav velocity data was still likely good
		_aspd_innov_integ_state = 0.f;

	} else {
		// nav velocity data is likely good so airspeed innovations are able to be used
		const float dt_s = math::constrain((time_now - _time_last_aspd_innov_check) / 1e6f, 0.01f, 0.2f); // limit to [5,100] Hz
		matrix::Vector2f wind_2d(_wind_estimator.get_wind());
		matrix::Vector3f air_vel = vI - matrix::Vector3f {wind_2d(0), wind_2d(1), 0.f};
		const float tas_innov = fabsf(_TAS - air_vel.norm());

		if (tas_innov > _tas_innov_threshold) {
			_aspd_innov_integ_state += dt_s * (tas_innov - _tas_innov_threshold); // integrate exceedance

		} else {
			// reset integrator used to trigger and record pass if integrator check is disabled
			_aspd_innov_integ_state = 0.f;
		}

		_innovations_check_failed = _aspd_innov_integ_state > _tas_innov_integ_threshold;
	}

	_time_last_aspd_innov_check = time_now;
}


void
AirspeedValidator::check_load_factor(float accel_z)
{
	// Check if the airspeed reading is lower than physically possible given the load factor

	if (!_load_factor_check_enabled) {
		_load_factor_ratio = 0.5f;
		_load_factor_check_failed = false;
		return;
	}

	if (_in_fixed_wing_flight) {

		float max_lift_ratio = fmaxf(_CAS, 0.7f) / fmaxf(_airspeed_stall, 1.0f);
		max_lift_ratio *= max_lift_ratio;
		_load_factor_ratio = 0.95f * _load_factor_ratio + 0.05f * (fabsf(accel_z) / 9.81f) / max_lift_ratio;
		_load_factor_ratio = math::constrain(_load_factor_ratio, 0.25f, 2.0f);
		_load_factor_check_failed = (_load_factor_ratio > 1.1f);

	} else {
		_load_factor_ratio = 0.5f; // reset if not in fixed-wing flight (and not in takeoff condition)
	}
}

void
AirspeedValidator::check_first_principle(const uint64_t timestamp, const float throttle_fw, const float throttle_trim,
		const uint64_t tecs_timestamp, const Quatf &att_q)
{
	if (! _first_principle_check_enabled) {
		_first_principle_check_failed = false;
		_time_last_first_principle_check_passing = timestamp;
		return;
	}

	const float pitch = matrix::Eulerf(att_q).theta();
	const hrt_abstime tecs_dt = timestamp - tecs_timestamp; // return if TECS data is old (TECS not running)

	if (!_in_fixed_wing_flight || tecs_dt > 500_ms || !PX4_ISFINITE(_IAS) || !PX4_ISFINITE(throttle_fw)
	    || !PX4_ISFINITE(throttle_trim) || !PX4_ISFINITE(pitch)) {
		// do not do anything in that case
		return;
	}

	const float dt = static_cast<float>(timestamp - _time_last_first_principle_check) / 1_s;
	_time_last_first_principle_check = timestamp;

	// update filters
	if (dt < FLT_EPSILON || dt > 1.f) {
		// reset if dt is too large
		_IAS_derivative.reset(0.f);
		_throttle_filtered.reset(throttle_fw);
		_pitch_filtered.reset(pitch);
		_time_last_first_principle_check_passing = timestamp;

	} else {
		// update filters, with different time constant
		_IAS_derivative.setParameters(dt, 5.f);
		_throttle_filtered.setParameters(dt, 0.5f);
		_pitch_filtered.setParameters(dt, 1.5f);

		_IAS_derivative.update(_IAS);
		_throttle_filtered.update(throttle_fw);
		_pitch_filtered.update(pitch);
	}

	// declare high throttle if more than 5% above trim
	const float high_throttle_threshold = math::min(throttle_trim + kHighThrottleDelta, _param_throttle_max);
	const bool high_throttle = _throttle_filtered.getState() > high_throttle_threshold;
	const bool pitching_down = _pitch_filtered.getState() < _param_psp_off;

	// check if the airspeed derivative is too low given the throttle and pitch
	const bool check_failing = _IAS_derivative.getState() < kIASDerivateThreshold && high_throttle && pitching_down;

	if (!check_failing) {
		_time_last_first_principle_check_passing = timestamp;
		_first_principle_check_failed = false;
	}

	if (timestamp - _time_last_first_principle_check_passing > _aspd_fp_t_window * 1_s) {
		// only update the test_failed flag once the timeout since first principle check failing is over
		_first_principle_check_failed = check_failing;
	}
}

void
AirspeedValidator::update_airspeed_valid_status(const uint64_t timestamp)
{
	if (_data_stuck_test_failed || _innovations_check_failed || _load_factor_check_failed
	    || _first_principle_check_failed) {
		// at least one check (data stuck, innovation or load factor) failed, so record timestamp
		_time_checks_failed = timestamp;

	} else if (! _data_stuck_test_failed && !_innovations_check_failed
		   && !_load_factor_check_failed && !_first_principle_check_failed) {
		// all checks(data stuck, innovation and load factor) must pass to declare airspeed good
		_time_checks_passed = timestamp;
	}

	if (_airspeed_valid) {
		// A simultaneous load factor and innovaton check fail makes it more likely that a large
		// airspeed measurement fault has developed, so a fault should be declared immediately
		const bool both_checks_failed = (_innovations_check_failed && _load_factor_check_failed);

		// Because the innovation and load factor checks are subject to short duration false positives
		// a timeout period is applied.
		const bool single_check_fail_timeout = (timestamp - _time_checks_passed) > _checks_fail_delay * 1_s;

		if (both_checks_failed || single_check_fail_timeout || _data_stuck_test_failed) {

			_airspeed_valid = false;
		}

	} else if (_checks_clear_delay > 0.f && (timestamp - _time_checks_failed) > _checks_clear_delay * 1_s) {
		// re-enabling is only possible if the clear delay is positive
		_airspeed_valid = true;
	}
}

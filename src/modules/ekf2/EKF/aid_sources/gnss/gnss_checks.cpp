/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file gnss_checks.cpp
 * Perform pre-flight and in-flight GNSS quality checks
 */

#include "aid_sources/gnss/gnss_checks.hpp"

namespace estimator
{
namespace
{
constexpr uint16_t bit(uint8_t check) { return 1u << check; }

constexpr uint16_t kDriftChecks =
	bit(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_DRIFT) | bit(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_DRIFT) |
	bit(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR) | bit(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR);

// nsats and PDOP keep their last pre-flight result, so a check enabled in flight must not fail on it.
constexpr uint16_t kSimplifiedChecks =
	bit(estimator_status_s::GPS_CHECK_FAIL_GPS_FIX) | bit(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR) |
	bit(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR) | bit(estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR) |
	bit(estimator_status_s::GPS_CHECK_FAIL_SPOOFED) | bit(estimator_status_s::GPS_CHECK_FAIL_JAMMED);
}

bool GnssChecks::run(const gnssSample &gnss, uint64_t time_us)
{
	// assume failed first time through
	if (_time_last_fail_us == 0) {
		_time_last_fail_us = time_us;
	}

	// Run strict checks while disarmed on the ground
	if (!_control_status.flags.armed && !_control_status.flags.in_air) {
		_initial_checks_passed = false;
	}

	_passed = false;

	if (_initial_checks_passed) {
		clearDriftChecks();

		if (runSimplifiedChecks(gnss)) {
			_passed = isTimedOut(_time_last_fail_us, time_us, math::max((uint64_t)1e6, (uint64_t)_params.min_health_time_us / 10));

		} else {
			_time_last_fail_us = time_us;
		}

	} else {
		if (runInitialFixChecks(gnss)) {
			if (isTimedOut(_time_last_fail_us, time_us, (uint64_t)_params.min_health_time_us)) {
				_initial_checks_passed = true;
				_passed = true;
			}

		} else {
			_time_last_fail_us = time_us;
		}
	}

	lat_lon_prev.initReference(gnss.lat, gnss.lon, gnss.time_us);
	_alt_prev = gnss.alt;

	if (_passed) {
		_time_last_pass_us = time_us;
	}

	return _passed;
}

void GnssChecks::setFail(uint8_t check, bool failed)
{
	if (failed) {
		_fail_flags |= bit(check);

	} else {
		_fail_flags &= ~bit(check);
	}
}

bool GnssChecks::runSimplifiedChecks(const gnssSample &gnss)
{
	setFail(estimator_status_s::GPS_CHECK_FAIL_GPS_FIX, gnss.fix_type < 3);

	// Check the reported horizontal and vertical position accuracy
	setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR, gnss.hacc > 50.f);
	setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR, gnss.vacc > 50.f);

	// Check the reported speed accuracy
	setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR, gnss.sacc > 10.f);

	setFail(estimator_status_s::GPS_CHECK_FAIL_SPOOFED, gnss.spoofed);
	setFail(estimator_status_s::GPS_CHECK_FAIL_JAMMED, gnss.jammed);

	return enabledChecksPass(kSimplifiedChecks);
}

bool GnssChecks::runInitialFixChecks(const gnssSample &gnss)
{
	// Check the fix type
	setFail(estimator_status_s::GPS_CHECK_FAIL_GPS_FIX, gnss.fix_type < _params.ekf2_req_fix);

	// Check the number of satellites
	setFail(estimator_status_s::GPS_CHECK_FAIL_MIN_SAT_COUNT, gnss.nsats < _params.ekf2_req_nsats);

	// Check the position dilution of precision
	setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_PDOP, gnss.pdop > _params.ekf2_req_pdop);

	// Check the reported horizontal and vertical position accuracy
	setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR, gnss.hacc > _params.ekf2_req_eph);
	setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR, gnss.vacc > _params.ekf2_req_epv);

	// Check the reported speed accuracy
	setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR, gnss.sacc > _params.ekf2_req_sacc);

	setFail(estimator_status_s::GPS_CHECK_FAIL_SPOOFED, gnss.spoofed);
	setFail(estimator_status_s::GPS_CHECK_FAIL_JAMMED, gnss.jammed);

	runOnGroundGnssChecks(gnss);

	// force horizontal speed failure if above the limit
	if (gnss.vel.xy().longerThan(_params.ekf2_vel_lim)) {
		setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR, true);
	}

	// force vertical speed failure if above the limit
	if (fabsf(gnss.vel(2)) > _params.ekf2_vel_lim) {
		setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR, true);
	}

	return enabledChecksPass(UINT16_MAX);
}

void GnssChecks::runOnGroundGnssChecks(const gnssSample &gnss)
{
	if (_control_status.flags.in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		clearDriftChecks();
		return;
	}

	if (_control_status.flags.vehicle_at_rest) {
		// Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
		constexpr float filt_time_const = 10.0f;
		const float dt = math::constrain(float(int64_t(gnss.time_us) - int64_t(
				lat_lon_prev.getProjectionReferenceTimestamp()))
						 * 1e-6f, 0.001f, filt_time_const);
		const float filter_coef = dt / filt_time_const;

		// Calculate position movement since last measurement
		float delta_pos_n = 0.0f;
		float delta_pos_e = 0.0f;

		// calculate position movement since last fix
		if (lat_lon_prev.getProjectionReferenceTimestamp() > 0) {
			lat_lon_prev.project(gnss.lat, gnss.lon, delta_pos_n, delta_pos_e);

		} else {
			// no previous position has been set
			lat_lon_prev.initReference(gnss.lat, gnss.lon, gnss.time_us);
			_alt_prev = gnss.alt;
		}

		// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		const Vector3f vel_limit(_params.ekf2_req_hdrift, _params.ekf2_req_hdrift, _params.ekf2_req_vdrift);
		Vector3f delta_pos(delta_pos_n, delta_pos_e, (_alt_prev - gnss.alt));

		// Apply a low pass filter
		_lat_lon_alt_deriv_filt = delta_pos / dt * filter_coef + _lat_lon_alt_deriv_filt * (1.0f - filter_coef);

		// Apply anti-windup to the state instead of the input to avoid generating a bias on asymmetric signals
		_lat_lon_alt_deriv_filt = matrix::constrain(_lat_lon_alt_deriv_filt, -10.0f * vel_limit, 10.0f * vel_limit);

		// hdrift: calculate the horizontal drift speed and fail if too high
		_horizontal_position_drift_rate_m_s = Vector2f(_lat_lon_alt_deriv_filt.xy()).norm();
		setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_DRIFT, _horizontal_position_drift_rate_m_s > _params.ekf2_req_hdrift);

		// vdrift: fail if the vertical drift speed is too high
		_vertical_position_drift_rate_m_s = fabsf(_lat_lon_alt_deriv_filt(2));
		setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_DRIFT, _vertical_position_drift_rate_m_s > _params.ekf2_req_vdrift);

		// hspeed: check the magnitude of the filtered horizontal GNSS velocity
		const Vector2f vel_ne = matrix::constrain(Vector2f(gnss.vel.xy()),
					-10.0f * _params.ekf2_req_hdrift,
					10.0f * _params.ekf2_req_hdrift);
		_vel_ne_filt = vel_ne * filter_coef + _vel_ne_filt * (1.0f - filter_coef);
		_filtered_horizontal_velocity_m_s = _vel_ne_filt.norm();
		setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR, _filtered_horizontal_velocity_m_s > _params.ekf2_req_hdrift);

		// vspeed: check the magnitude of the filtered vertical GNSS velocity
		const float gnss_vz_limit = 10.f * _params.ekf2_req_vdrift;
		const float gnss_vz = math::constrain(gnss.vel(2), -gnss_vz_limit, gnss_vz_limit);
		_vel_d_filt = gnss_vz * filter_coef + _vel_d_filt * (1.f - filter_coef);

		setFail(estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR, fabsf(_vel_d_filt) > _params.ekf2_req_vdrift);

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		resetDriftFilters();
	}
}

void GnssChecks::clearDriftChecks()
{
	_fail_flags &= ~kDriftChecks;

	resetDriftFilters();
}

void GnssChecks::resetDriftFilters()
{
	_vel_ne_filt.setZero();
	_vel_d_filt = 0.f;

	_lat_lon_alt_deriv_filt.setZero();

	_horizontal_position_drift_rate_m_s = NAN;
	_vertical_position_drift_rate_m_s = NAN;
	_filtered_horizontal_velocity_m_s = NAN;
}
}; // namespace estimator

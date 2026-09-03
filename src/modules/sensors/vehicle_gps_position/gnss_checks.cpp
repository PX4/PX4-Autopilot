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
 * Perform GNSS quality checks
 */

#include "gnss_checks.hpp"


void GnssChecks::setParams(int32_t check_mask, int32_t req_nsats, float req_pdop, float req_eph, float req_epv,
			   float req_sacc, float req_hdrift, float req_vdrift, int32_t req_fix, float vel_lim,
			   uint32_t min_health_time_us)
{
	_params.check_mask 		= check_mask;
	_params.req_nsats 		= req_nsats;
	_params.req_pdop 		= req_pdop;
	_params.req_eph 		= req_eph;
	_params.req_epv 		= req_epv;
	_params.req_sacc 		= req_sacc;
	_params.req_hdrift 		= req_hdrift;
	_params.req_vdrift 		= req_vdrift;
	_params.req_fix 		= req_fix;
	_params.vel_lim 		= vel_lim;
	_params.min_health_time_us	= min_health_time_us;

}

bool GnssChecks::run(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest)
{
	// assume failed first time through
	if (_time_last_fail_us == 0) {
		_time_last_fail_us = gnss.time_us;
	}

	// Run strict checks while not flying yet
	if (!in_air) {
		_initial_checks_passed = false;
	}

	_passed = false;

	if (_initial_checks_passed) {
		if (runSimplifiedChecks(gnss)) {
			_passed = isTimedOut(_time_last_fail_us, gnss.time_us, math::max((uint64_t)1e6, (uint64_t)_params.min_health_time_us / 10));

		} else {
			_time_last_fail_us = gnss.time_us;
		}

	} else {
		if (runInitialFixChecks(gnss, in_air, vehicle_at_rest)) {
			if (isTimedOut(_time_last_fail_us, gnss.time_us, (uint64_t)_params.min_health_time_us)) {
				_initial_checks_passed = true;
				_passed = true;
			}

		} else {
			_time_last_fail_us = gnss.time_us;
		}
	}

	lat_lon_prev.initReference(gnss.lat, gnss.lon, gnss.time_us);
	_alt_prev = gnss.alt;

	if (_passed) {
		_time_last_pass_us = gnss.time_us;
	}

	return _passed;
}

bool GnssChecks::runSimplifiedChecks(const gnssChecksSample &gnss)
{
	_check_fail_status.flags.fix = (gnss.fix_type < 3);

	// Check the reported horizontal and vertical position accuracy
	_check_fail_status.flags.hacc = (gnss.hacc > 50.f);
	_check_fail_status.flags.vacc = (gnss.vacc > 50.f);

	// Check the reported speed accuracy
	_check_fail_status.flags.sacc = (gnss.sacc > 10.f);

	_check_fail_status.flags.spoofed = gnss.spoofed;
	_check_fail_status.flags.jammed = gnss.jammed;

	bool passed = true;

	if (
		(_check_fail_status.flags.fix     && isCheckEnabled(GnssChecksMask::kFix)) ||
		(_check_fail_status.flags.hacc    && isCheckEnabled(GnssChecksMask::kHacc)) ||
		(_check_fail_status.flags.vacc    && isCheckEnabled(GnssChecksMask::kVacc)) ||
		(_check_fail_status.flags.sacc    && isCheckEnabled(GnssChecksMask::kSacc)) ||
		(_check_fail_status.flags.spoofed && isCheckEnabled(GnssChecksMask::kSpoofed)) ||
		(_check_fail_status.flags.jammed  && isCheckEnabled(GnssChecksMask::kJammed))
	) {
		passed = false;
	}

	return passed;
}

bool GnssChecks::runInitialFixChecks(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest)
{
	// Check the fix type
	_check_fail_status.flags.fix = (gnss.fix_type < _params.req_fix);

	// Check the number of satellites
	_check_fail_status.flags.nsats = (gnss.nsats < _params.req_nsats);

	// Check the position dilution of precision
	_check_fail_status.flags.pdop = (gnss.pdop > _params.req_pdop);

	// Check the reported horizontal and vertical position accuracy
	_check_fail_status.flags.hacc = (gnss.hacc > _params.req_eph);
	_check_fail_status.flags.vacc = (gnss.vacc > _params.req_epv);

	// Check the reported speed accuracy
	_check_fail_status.flags.sacc = (gnss.sacc > _params.req_sacc);

	_check_fail_status.flags.spoofed = gnss.spoofed;
	_check_fail_status.flags.jammed = gnss.jammed;

	runOnGroundGnssChecks(gnss, in_air, vehicle_at_rest);

	// force horizontal speed failure if above the limit
	if (gnss.vel.xy().longerThan(_params.vel_lim)) {
		_check_fail_status.flags.hspeed = true;
	}

	// force vertical speed failure if above the limit
	if (fabsf(gnss.vel(2)) > _params.vel_lim) {
		_check_fail_status.flags.vspeed = true;
	}

	bool passed = true;

	// if any user selected checks have failed, record the fail time
	if (
		(_check_fail_status.flags.fix     && isCheckEnabled(GnssChecksMask::kFix)) ||
		(_check_fail_status.flags.nsats   && isCheckEnabled(GnssChecksMask::kNsats)) ||
		(_check_fail_status.flags.pdop    && isCheckEnabled(GnssChecksMask::kPdop)) ||
		(_check_fail_status.flags.hacc    && isCheckEnabled(GnssChecksMask::kHacc)) ||
		(_check_fail_status.flags.vacc    && isCheckEnabled(GnssChecksMask::kVacc)) ||
		(_check_fail_status.flags.sacc    && isCheckEnabled(GnssChecksMask::kSacc)) ||
		(_check_fail_status.flags.hdrift  && isCheckEnabled(GnssChecksMask::kHdrift)) ||
		(_check_fail_status.flags.vdrift  && isCheckEnabled(GnssChecksMask::kVdrift)) ||
		(_check_fail_status.flags.hspeed  && isCheckEnabled(GnssChecksMask::kHspd)) ||
		(_check_fail_status.flags.vspeed  && isCheckEnabled(GnssChecksMask::kVspd)) ||
		(_check_fail_status.flags.spoofed && isCheckEnabled(GnssChecksMask::kSpoofed)) ||
		(_check_fail_status.flags.jammed  && isCheckEnabled(GnssChecksMask::kJammed))
	) {
		passed = false;
	}

	return passed;
}

void GnssChecks::runOnGroundGnssChecks(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest)
{
	if (in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		_check_fail_status.flags.hdrift = false;
		_check_fail_status.flags.vdrift = false;
		_check_fail_status.flags.hspeed = false;
		_check_fail_status.flags.vspeed = false;

		resetDriftFilters();
		return;
	}

	if (vehicle_at_rest) {
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
		const matrix::Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
		matrix::Vector3f delta_pos(delta_pos_n, delta_pos_e, (_alt_prev - gnss.alt));

		// Apply a low pass filter
		_lat_lon_alt_deriv_filt = delta_pos / dt * filter_coef + _lat_lon_alt_deriv_filt * (1.0f - filter_coef);

		// Apply anti-windup to the state instead of the input to avoid generating a bias on asymmetric signals
		_lat_lon_alt_deriv_filt = matrix::constrain(_lat_lon_alt_deriv_filt, -10.0f * vel_limit, 10.0f * vel_limit);

		// hdrift: calculate the horizontal drift speed and fail if too high
		_horizontal_position_drift_rate_m_s = matrix::Vector2f(_lat_lon_alt_deriv_filt.xy()).norm();
		_check_fail_status.flags.hdrift = (_horizontal_position_drift_rate_m_s > _params.req_hdrift);

		// vdrift: fail if the vertical drift speed is too high
		_vertical_position_drift_rate_m_s = fabsf(_lat_lon_alt_deriv_filt(2));
		_check_fail_status.flags.vdrift = (_vertical_position_drift_rate_m_s > _params.req_vdrift);

		// hspeed: check the magnitude of the filtered horizontal GNSS velocity
		const matrix::Vector2f vel_ne = matrix::constrain(matrix::Vector2f(gnss.vel.xy()),
						-10.0f * _params.req_hdrift,
						10.0f * _params.req_hdrift);
		_vel_ne_filt = vel_ne * filter_coef + _vel_ne_filt * (1.0f - filter_coef);
		_filtered_horizontal_velocity_m_s = _vel_ne_filt.norm();
		_check_fail_status.flags.hspeed = (_filtered_horizontal_velocity_m_s > _params.req_hdrift);

		// vspeed: check the magnitude of the filtered vertical GNSS velocity
		const float gnss_vz_limit = 10.f * _params.req_vdrift;
		const float gnss_vz = math::constrain(gnss.vel(2), -gnss_vz_limit, gnss_vz_limit);
		_vel_d_filt = gnss_vz * filter_coef + _vel_d_filt * (1.f - filter_coef);

		_check_fail_status.flags.vspeed = (fabsf(_vel_d_filt) > _params.req_vdrift);

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		resetDriftFilters();
	}
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

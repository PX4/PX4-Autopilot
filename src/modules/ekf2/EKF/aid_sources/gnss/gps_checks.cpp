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
 * @file gps_checks.cpp
 * Perform pre-flight and in-flight GPS quality checks
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::isGnssCheckEnabled(GnssChecksMask check)
{
	return (_params.gps_check_mask & static_cast<int32_t>(check));
}

bool Ekf::runGnssChecks(const gnssSample &gnss)
{
	_gps_check_fail_status.flags.spoofed = gnss.spoofed;

	// Check the fix type
	_gps_check_fail_status.flags.fix = (gnss.fix_type < 3);

	// Check the number of satellites
	_gps_check_fail_status.flags.nsats = (gnss.nsats < _params.req_nsats);

	// Check the position dilution of precision
	_gps_check_fail_status.flags.pdop = (gnss.pdop > _params.req_pdop);

	// Check the reported horizontal and vertical position accuracy
	_gps_check_fail_status.flags.hacc = (gnss.hacc > _params.req_hacc);
	_gps_check_fail_status.flags.vacc = (gnss.vacc > _params.req_vacc);

	// Check the reported speed accuracy
	_gps_check_fail_status.flags.sacc = (gnss.sacc > _params.req_sacc);

	runOnGroundGnssChecks(gnss);

	// force horizontal speed failure if above the limit
	if (gnss.vel.xy().longerThan(_params.velocity_limit)) {
		_gps_check_fail_status.flags.hspeed = true;
	}

	// force vertical speed failure if above the limit
	if (fabsf(gnss.vel(2)) > _params.velocity_limit) {
		_gps_check_fail_status.flags.vspeed = true;
	}

	// save GPS fix for next time
	_gnss_pos_prev.initReference(gnss.lat, gnss.lon, gnss.time_us);
	_gnss_alt_prev = gnss.alt;

	// assume failed first time through
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_delayed_us;
	}

	// if any user selected checks have failed, record the fail time
	if (
		_gps_check_fail_status.flags.fix ||
		(_gps_check_fail_status.flags.nsats   && isGnssCheckEnabled(GnssChecksMask::kNsats)) ||
		(_gps_check_fail_status.flags.pdop    && isGnssCheckEnabled(GnssChecksMask::kPdop)) ||
		(_gps_check_fail_status.flags.hacc    && isGnssCheckEnabled(GnssChecksMask::kHacc)) ||
		(_gps_check_fail_status.flags.vacc    && isGnssCheckEnabled(GnssChecksMask::kVacc)) ||
		(_gps_check_fail_status.flags.sacc    && isGnssCheckEnabled(GnssChecksMask::kSacc)) ||
		(_gps_check_fail_status.flags.hdrift  && isGnssCheckEnabled(GnssChecksMask::kHdrift)) ||
		(_gps_check_fail_status.flags.vdrift  && isGnssCheckEnabled(GnssChecksMask::kVdrift)) ||
		(_gps_check_fail_status.flags.hspeed  && isGnssCheckEnabled(GnssChecksMask::kHspd)) ||
		(_gps_check_fail_status.flags.vspeed  && isGnssCheckEnabled(GnssChecksMask::kVspd)) ||
		(_gps_check_fail_status.flags.spoofed && isGnssCheckEnabled(GnssChecksMask::kSpoofed))
	) {
		_last_gps_fail_us = _time_delayed_us;
		return false;

	} else {
		_last_gps_pass_us = _time_delayed_us;
		return true;
	}
}

void Ekf::runOnGroundGnssChecks(const gnssSample &gnss)
{
	if (_control_status.flags.in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		_gps_check_fail_status.flags.hdrift = false;
		_gps_check_fail_status.flags.vdrift = false;
		_gps_check_fail_status.flags.hspeed = false;
		_gps_check_fail_status.flags.vspeed = false;

		resetGpsDriftCheckFilters();
		return;
	}

	if (_control_status.flags.vehicle_at_rest) {
		// Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
		constexpr float filt_time_const = 10.0f;
		const float dt = math::constrain(float(int64_t(gnss.time_us) - int64_t(
				_gnss_pos_prev.getProjectionReferenceTimestamp()))
						 * 1e-6f, 0.001f, filt_time_const);
		const float filter_coef = dt / filt_time_const;

		// Calculate position movement since last measurement
		float delta_pos_n = 0.0f;
		float delta_pos_e = 0.0f;

		// calculate position movement since last GPS fix
		if (_gnss_pos_prev.getProjectionReferenceTimestamp() > 0) {
			_gnss_pos_prev.project(gnss.lat, gnss.lon, delta_pos_n, delta_pos_e);

		} else {
			// no previous position has been set
			_gnss_pos_prev.initReference(gnss.lat, gnss.lon, gnss.time_us);
			_gnss_alt_prev = gnss.alt;
		}

		// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
		Vector3f delta_pos(delta_pos_n, delta_pos_e, (_gnss_alt_prev - gnss.alt));

		// Apply a low pass filter
		_gps_pos_deriv_filt = delta_pos / dt * filter_coef + _gps_pos_deriv_filt * (1.0f - filter_coef);

		// Apply anti-windup to the state instead of the input to avoid generating a bias on asymmetric signals
		_gps_pos_deriv_filt = matrix::constrain(_gps_pos_deriv_filt, -10.0f * vel_limit, 10.0f * vel_limit);

		// hdrift: calculate the horizontal drift speed and fail if too high
		_gps_horizontal_position_drift_rate_m_s = Vector2f(_gps_pos_deriv_filt.xy()).norm();
		_gps_check_fail_status.flags.hdrift = (_gps_horizontal_position_drift_rate_m_s > _params.req_hdrift);

		// vdrift: fail if the vertical drift speed is too high
		_gps_vertical_position_drift_rate_m_s = fabsf(_gps_pos_deriv_filt(2));
		_gps_check_fail_status.flags.vdrift = (_gps_vertical_position_drift_rate_m_s > _params.req_vdrift);

		// hspeed: check the magnitude of the filtered horizontal GNSS velocity
		const Vector2f gps_velNE = matrix::constrain(Vector2f(gnss.vel.xy()),
					   -10.0f * _params.req_hdrift,
					   10.0f * _params.req_hdrift);
		_gps_velNE_filt = gps_velNE * filter_coef + _gps_velNE_filt * (1.0f - filter_coef);
		_gps_filtered_horizontal_velocity_m_s = _gps_velNE_filt.norm();
		_gps_check_fail_status.flags.hspeed = (_gps_filtered_horizontal_velocity_m_s > _params.req_hdrift);

		// vspeed: check the magnitude of the filtered vertical GNSS velocity
		const float gnss_vz_limit = 10.f * _params.req_vdrift;
		const float gnss_vz = math::constrain(gnss.vel(2), -gnss_vz_limit, gnss_vz_limit);
		_gps_vel_d_filt = gnss_vz * filter_coef + _gps_vel_d_filt * (1.f - filter_coef);

		_gps_check_fail_status.flags.vspeed = (fabsf(_gps_vel_d_filt) > _params.req_vdrift);

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		resetGpsDriftCheckFilters();
	}
}

void Ekf::resetGpsDriftCheckFilters()
{
	_gps_velNE_filt.setZero();
	_gps_vel_d_filt = 0.f;

	_gps_pos_deriv_filt.setZero();

	_gps_horizontal_position_drift_rate_m_s = NAN;
	_gps_vertical_position_drift_rate_m_s = NAN;
	_gps_filtered_horizontal_velocity_m_s = NAN;
}

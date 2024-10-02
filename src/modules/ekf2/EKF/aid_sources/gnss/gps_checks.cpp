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

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS   (1<<0)
#define MASK_GPS_PDOP    (1<<1)
#define MASK_GPS_HACC    (1<<2)
#define MASK_GPS_VACC    (1<<3)
#define MASK_GPS_SACC    (1<<4)
#define MASK_GPS_HDRIFT  (1<<5)
#define MASK_GPS_VDRIFT  (1<<6)
#define MASK_GPS_HSPD    (1<<7)
#define MASK_GPS_VSPD    (1<<8)
#define MASK_GPS_SPOOFED (1<<9)

void Ekf::collect_gps(const gnssSample &gps)
{
	if (_filter_initialised && !_local_origin_lat_lon.isInitialized() && _gps_checks_passed) {
		_information_events.flags.gps_checks_passed = true;

	} else {
		// a rough 2D fix is sufficient to lookup earth spin rate
		const bool gps_rough_2d_fix = (gps.fix_type >= 2) && (gps.hacc < 1000);

		if (gps_rough_2d_fix && (_gps_checks_passed || !_local_origin_lat_lon.isInitialized())) {
			_earth_rate_NED = calcEarthRateNED((float)math::radians(gps.lat));
		}
	}
}

bool Ekf::runGnssChecks(const gnssSample &gps)
{
	_gps_check_fail_status.flags.spoofed = gps.spoofed;

	// Check the fix type
	_gps_check_fail_status.flags.fix = (gps.fix_type < 3);

	// Check the number of satellites
	_gps_check_fail_status.flags.nsats = (gps.nsats < _params.req_nsats);

	// Check the position dilution of precision
	_gps_check_fail_status.flags.pdop = (gps.pdop > _params.req_pdop);

	// Check the reported horizontal and vertical position accuracy
	_gps_check_fail_status.flags.hacc = (gps.hacc > _params.req_hacc);
	_gps_check_fail_status.flags.vacc = (gps.vacc > _params.req_vacc);

	// Check the reported speed accuracy
	_gps_check_fail_status.flags.sacc = (gps.sacc > _params.req_sacc);

	// Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
	constexpr float filt_time_const = 10.0f;
	const float dt = math::constrain(float(int64_t(gps.time_us) - int64_t(_gps_pos_prev.getProjectionReferenceTimestamp()))
					 * 1e-6f, 0.001f, filt_time_const);
	const float filter_coef = dt / filt_time_const;

	// The following checks are only valid when the vehicle is at rest
	const double lat = gps.lat;
	const double lon = gps.lon;

	if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// Calculate position movement since last measurement
		float delta_pos_n = 0.0f;
		float delta_pos_e = 0.0f;

		// calculate position movement since last GPS fix
		if (_gps_pos_prev.getProjectionReferenceTimestamp() > 0) {
			_gps_pos_prev.project(lat, lon, delta_pos_n, delta_pos_e);

		} else {
			// no previous position has been set
			_gps_pos_prev.initReference(lat, lon, gps.time_us);
			_gps_alt_prev = gps.alt;
		}

		// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
		Vector3f pos_derived(delta_pos_n, delta_pos_e, (_gps_alt_prev - gps.alt));
		pos_derived = matrix::constrain(pos_derived / dt, -10.0f * vel_limit, 10.0f * vel_limit);

		// Apply a low pass filter
		_gps_pos_deriv_filt = pos_derived * filter_coef + _gps_pos_deriv_filt * (1.0f - filter_coef);

		// hdrift: calculate the horizontal drift speed and fail if too high
		_gps_horizontal_position_drift_rate_m_s = Vector2f(_gps_pos_deriv_filt.xy()).norm();
		_gps_check_fail_status.flags.hdrift = (_gps_horizontal_position_drift_rate_m_s > _params.req_hdrift);

		// vdrift: fail if the vertical drift speed is too high
		_gps_vertical_position_drift_rate_m_s = fabsf(_gps_pos_deriv_filt(2));
		_gps_check_fail_status.flags.vdrift = (_gps_vertical_position_drift_rate_m_s > _params.req_vdrift);

		// hspeed: check the magnitude of the filtered horizontal GNSS velocity
		const Vector2f gps_velNE = matrix::constrain(Vector2f(gps.vel.xy()),
					   -10.0f * _params.req_hdrift,
					   10.0f * _params.req_hdrift);
		_gps_velNE_filt = gps_velNE * filter_coef + _gps_velNE_filt * (1.0f - filter_coef);
		_gps_filtered_horizontal_velocity_m_s = _gps_velNE_filt.norm();
		_gps_check_fail_status.flags.hspeed = (_gps_filtered_horizontal_velocity_m_s > _params.req_hdrift);

		// vspeed: check the magnitude of the filtered vertical GNSS velocity
		const float gnss_vz_limit = 10.f * _params.req_vdrift;
		const float gnss_vz = math::constrain(gps.vel(2), -gnss_vz_limit, gnss_vz_limit);
		_gps_vel_d_filt = gnss_vz * filter_coef + _gps_vel_d_filt * (1.f - filter_coef);

		_gps_check_fail_status.flags.vspeed = (fabsf(_gps_vel_d_filt) > _params.req_vdrift);

	} else if (_control_status.flags.in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		_gps_check_fail_status.flags.hdrift = false;
		_gps_check_fail_status.flags.vdrift = false;
		_gps_check_fail_status.flags.hspeed = false;
		_gps_check_fail_status.flags.vspeed = false;

		resetGpsDriftCheckFilters();

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		resetGpsDriftCheckFilters();
	}

	// force horizontal speed failure if above the limit
	if (gps.vel.xy().longerThan(_params.velocity_limit)) {
		_gps_check_fail_status.flags.hspeed = true;
	}

	// force vertical speed failure if above the limit
	if (fabsf(gps.vel(2)) > _params.velocity_limit) {
		_gps_check_fail_status.flags.vspeed = true;
	}

	// save GPS fix for next time
	_gps_pos_prev.initReference(lat, lon, gps.time_us);
	_gps_alt_prev = gps.alt;

	// assume failed first time through
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_delayed_us;
	}

	// if any user selected checks have failed, record the fail time
	if (
		_gps_check_fail_status.flags.fix ||
		(_gps_check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
		(_gps_check_fail_status.flags.pdop    && (_params.gps_check_mask & MASK_GPS_PDOP)) ||
		(_gps_check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
		(_gps_check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
		(_gps_check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
		(_gps_check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
		(_gps_check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
		(_gps_check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) ||
		(_gps_check_fail_status.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD)) ||
		(_gps_check_fail_status.flags.spoofed && (_params.gps_check_mask & MASK_GPS_SPOOFED))
	) {
		_last_gps_fail_us = _time_delayed_us;
		return false;

	} else {
		_last_gps_pass_us = _time_delayed_us;
		return true;
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

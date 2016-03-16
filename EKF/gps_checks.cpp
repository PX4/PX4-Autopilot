/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * @file gps_checks.cpp
 * Perform pre-flight and in-flight GPS quality checks
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include "mathlib.h"
#include "geo.h"
// GPS pre-flight check bit locations
#define MASK_GPS_NSATS  (1<<0)
#define MASK_GPS_GDOP   (1<<1)
#define MASK_GPS_HACC   (1<<2)
#define MASK_GPS_VACC   (1<<3)
#define MASK_GPS_SACC   (1<<4)
#define MASK_GPS_HDRIFT (1<<5)
#define MASK_GPS_VDRIFT (1<<6)
#define MASK_GPS_HSPD   (1<<7)
#define MASK_GPS_VSPD   (1<<8)

bool Ekf::collect_gps(uint64_t time_usec, struct gps_message *gps)
{
	// If we have defined the WGS-84 position of the NED origin, run gps quality checks until they pass, then define the origins WGS-84 position using the last GPS fix
	if (!_NED_origin_initialised) {
		// we have good GPS data so can now set the origin's WGS-84 position
		if (gps_is_good(gps) && !_NED_origin_initialised) {
			printf("EKF gps is good - setting origin\n");
			// Set the origin's WGS-84 position to the last gps fix
			double lat = gps->lat / 1.0e7;
			double lon = gps->lon / 1.0e7;
			map_projection_init_timestamped(&_pos_ref, lat, lon, _time_last_imu);

			// if we are already doing aiding, corect for the change in posiiton since the EKF started navigating
			if (_control_status.flags.opt_flow || _control_status.flags.gps) {
				double est_lat, est_lon;
				map_projection_reproject(&_pos_ref, -_state.pos(0), -_state.pos(1), &est_lat, &est_lon);
				map_projection_init_timestamped(&_pos_ref, est_lat, est_lon, _time_last_imu);
			}

			// Take the current GPS height and subtract the filter height above origin to estimate the GPS height of the origin
			_gps_alt_ref = 1e-3f * (float)gps->alt + _state.pos(2);
			_NED_origin_initialised = true;
			_last_gps_origin_time_us = _time_last_imu;
			// set the magnetic declination returned by the geo library using the current GPS position
			_mag_declination_gps = math::radians(get_mag_declination(lat, lon));
			// save the horizontal and vertical position uncertainty of the origin
			_gps_origin_eph = gps->eph;
			_gps_origin_epv = gps->epv;

			// if the user has selected GPS as the primary height source, switch across to using it
			if (_primary_hgt_source == VDIST_SENSOR_GPS) {
				printf("EKF switching to GPS height\n");
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = true;
				_control_status.flags.rng_hgt = false;
				// zero the sensor offset
				_hgt_sensor_offset = 0.0f;
			}
		}
	}

	// start collecting GPS if there is a 3D fix and the NED origin has been set
	if (_NED_origin_initialised && gps->fix_type >= 3) {
		return true;

	} else {
		return false;
	}

	return false;
}

/*
 * Return true if the GPS solution quality is adequate to set an origin for the EKF
 * and start GPS aiding.
 * All activated checks must pass for 10 seconds.
 * Checks are activated using the EKF2_GPS_CHECK bitmask parameter
 * Checks are adjusted using the EKF2_REQ_* parameters
*/
bool Ekf::gps_is_good(struct gps_message *gps)
{
	// Check the fix type
	_gps_check_fail_status.flags.fix = (gps->fix_type < 3);

	// Check the number of satellites
	_gps_check_fail_status.flags.nsats = (gps->nsats < _params.req_nsats);

	// Check the geometric dilution of precision
	_gps_check_fail_status.flags.gdop = (gps->gdop > _params.req_gdop);

	// Check the reported horizontal position accuracy
	_gps_check_fail_status.flags.hacc = (gps->eph > _params.req_hacc);

	// Check the reported vertical position accuracy
	_gps_check_fail_status.flags.vacc = (gps->epv > _params.req_vacc);

	// Check the reported speed accuracy
	_gps_check_fail_status.flags.sacc = (gps->sacc > _params.req_sacc);

	// Calculate position movement since last measurement
	float delta_posN = 0.0f;
	float delta_PosE = 0.0f;
	double lat = gps->lat * 1.0e-7;
	double lon = gps->lon * 1.0e-7;

	if (_pos_ref.init_done) {
		map_projection_project(&_pos_ref, lat, lon, &delta_posN, &delta_PosE);

	} else {
		map_projection_init_timestamped(&_pos_ref, lat, lon, _time_last_imu);
		_gps_alt_ref = 1e-3f * (float)gps->alt;
	}

	// Calculate time lapsed since last update, limit to prevent numerical errors and calculate the lowpass filter coefficient
	const float filt_time_const = 10.0f;
	float dt = fminf(fmaxf(float(_time_last_imu - _last_gps_origin_time_us) * 1e-6f, 0.001f), filt_time_const);
	float filter_coef = dt / filt_time_const;

	// Calculate the horizontal drift velocity components and limit to 10x the threshold
	float vel_limit = 10.0f * _params.req_hdrift;
	float velN = fminf(fmaxf(delta_posN / dt, -vel_limit), vel_limit);
	float velE = fminf(fmaxf(delta_PosE / dt, -vel_limit), vel_limit);

	// Apply a low pass filter
	_gpsDriftVelN = velN * filter_coef + _gpsDriftVelN * (1.0f - filter_coef);
	_gpsDriftVelE = velE * filter_coef + _gpsDriftVelE * (1.0f - filter_coef);

	// Calculate the horizontal drift speed and fail if too high
	// This check can only be used if the vehicle is stationary during alignment
	if (!_control_status.flags.armed) {
		float drift_speed = sqrtf(_gpsDriftVelN * _gpsDriftVelN + _gpsDriftVelE * _gpsDriftVelE);
		_gps_check_fail_status.flags.hdrift = (drift_speed > _params.req_hdrift);

	} else {
		_gps_check_fail_status.flags.hdrift = false;
	}

	// Save current position as the reference for next time
	map_projection_init_timestamped(&_pos_ref, lat, lon, _time_last_imu);
	_last_gps_origin_time_us = _time_last_imu;

	// Calculate the vertical drift velocity and limit to 10x the threshold
	vel_limit = 10.0f * _params.req_vdrift;
	float velD = fminf(fmaxf((_gps_alt_ref - 1e-3f * (float)gps->alt) / dt, -vel_limit), vel_limit);

	// Save the current height as the reference for next time
	_gps_alt_ref = 1e-3f * (float)gps->alt;

	// Apply a low pass filter to the vertical velocity
	_gps_drift_velD = velD * filter_coef + _gps_drift_velD * (1.0f - filter_coef);

	// Fail if the vertical drift speed is too high
	// This check can only be used if the vehicle is stationary during alignment
	if (!_control_status.flags.armed) {
		_gps_check_fail_status.flags.vdrift = (fabsf(_gps_drift_velD) > _params.req_vdrift);

	} else {
		_gps_check_fail_status.flags.vdrift = false;
	}

	// Check the magnitude of the filtered horizontal GPS velocity
	// This check can only be used if the vehicle is stationary during alignment
	if (!_control_status.flags.armed) {
		vel_limit = 10.0f * _params.req_hdrift;
		float velN = fminf(fmaxf(gps->vel_ned[0], -vel_limit), vel_limit);
		float velE = fminf(fmaxf(gps->vel_ned[1], -vel_limit), vel_limit);
		_gps_velN_filt = velN * filter_coef + _gps_velN_filt * (1.0f - filter_coef);
		_gps_velE_filt  = velE * filter_coef + _gps_velE_filt  * (1.0f - filter_coef);
		float horiz_speed = sqrtf(_gps_velN_filt * _gps_velN_filt + _gps_velE_filt * _gps_velE_filt);
		_gps_check_fail_status.flags.hspeed = (horiz_speed > _params.req_hdrift);

	} else {
		_gps_check_fail_status.flags.hspeed = false;
	}

	// Check  the filtered difference between GPS and EKF vertical velocity
	vel_limit = 10.0f * _params.req_vdrift;
	float vertVel = fminf(fmaxf((gps->vel_ned[2] - _state.vel(2)), -vel_limit), vel_limit);
	_gps_velD_diff_filt = vertVel * filter_coef + _gps_velD_diff_filt * (1.0f - filter_coef);
	_gps_check_fail_status.flags.vspeed = (fabsf(_gps_velD_diff_filt) > _params.req_vdrift);

	// assume failed first time through
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_last_imu;
	}

	// if any user selected checks have failed, record the fail time
	if (
		_gps_check_fail_status.flags.fix ||
		(_gps_check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
		(_gps_check_fail_status.flags.gdop    && (_params.gps_check_mask & MASK_GPS_GDOP)) ||
		(_gps_check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
		(_gps_check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
		(_gps_check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
		(_gps_check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
		(_gps_check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
		(_gps_check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) ||
		(_gps_check_fail_status.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD))
	) {
		_last_gps_fail_us = _time_last_imu;
	}

	// continuous period without fail of 10 seconds required to return a healthy status
	if (_time_last_imu - _last_gps_fail_us > 1e7) {
		return true;
	}

	return false;
}


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
 * @file ekf_helper.cpp
 * Definition of ekf helper functions.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <cstdlib>

bool Ekf::isHeightResetRequired() const
{
	// check if height is continuously failing because of accel errors
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max);

	return (continuous_bad_accel_hgt || hgt_fusion_timeout);
}

Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

void Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	origin_time = _local_origin_lat_lon.getProjectionReferenceTimestamp();
	latitude = _local_origin_lat_lon.getProjectionReferenceLat();
	longitude = _local_origin_lat_lon.getProjectionReferenceLon();
	origin_alt  = getEkfGlobalOriginAltitude();
}

bool Ekf::checkLatLonValidity(const double latitude, const double longitude)
{
	const bool lat_valid = (PX4_ISFINITE(latitude) && (abs(latitude) <= 90));
	const bool lon_valid = (PX4_ISFINITE(longitude) && (abs(longitude) <= 180));

	return (lat_valid && lon_valid);
}

bool Ekf::checkAltitudeValidity(const float altitude)
{
	// sanity check valid altitude anywhere between the Mariana Trench and edge of Space
	return (PX4_ISFINITE(altitude) && ((altitude > -12'000.f) && (altitude < 100'000.f)));
}

bool Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude, const float hpos_var,
			     const float vpos_var)
{
	if (!setLatLonOrigin(latitude, longitude, hpos_var)) {
		return false;
	}

	// altitude is optional
	setAltOrigin(altitude, vpos_var);

	return true;
}

bool Ekf::setLatLonOrigin(const double latitude, const double longitude, const float hpos_var)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	if (!_local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid()) {
		// Already navigating in a local frame, use the origin to initialize global position
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		double new_latitude;
		double new_longitude;
		_local_origin_lat_lon.reproject(pos_prev(0), pos_prev(1), new_latitude, new_longitude);
		resetHorizontalPositionTo(new_latitude, new_longitude, hpos_var);

	} else {
		// Simply move the origin and compute the change in local position
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		const Vector2f pos_new = getLocalHorizontalPosition();
		const Vector2f delta_pos = pos_new - pos_prev;
		updateHorizontalPositionResetStatus(delta_pos);
	}

	return true;
}

bool Ekf::setAltOrigin(const float altitude, const float vpos_var)
{
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	ECL_INFO("EKF origin altitude %.1fm -> %.1fm", (double)_local_origin_alt,
		 (double)altitude);

	if (!PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid()) {
		const float local_alt_prev = _gpos.altitude();
		_local_origin_alt = altitude;
		resetHeightTo(local_alt_prev + _local_origin_alt);

	} else {
		const float delta_origin_alt = altitude - _local_origin_alt;
		_local_origin_alt = altitude;
		updateVerticalPositionResetStatus(-delta_origin_alt);

#if defined(CONFIG_EKF2_TERRAIN)
		updateTerrainResetStatus(-delta_origin_alt);
#endif // CONFIG_EKF2_TERRAIN
	}

	return true;
}

bool Ekf::resetGlobalPositionTo(const double latitude, const double longitude, const float altitude,
				const float hpos_var, const float vpos_var)
{
	if (!resetLatLonTo(latitude, longitude, hpos_var)) {
		return false;
	}

	// altitude is optional
	resetAltitudeTo(altitude, vpos_var);

	return true;
}

bool Ekf::resetLatLonTo(const double latitude, const double longitude, const float hpos_var)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	Vector2f pos_prev;

	if (!_local_origin_lat_lon.isInitialized()) {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		pos_prev = zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg());

		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);

		// if we are already doing aiding, correct for the change in position since the EKF started navigating
		if (isLocalHorizontalPositionValid()) {
			double est_lat;
			double est_lon;
			_local_origin_lat_lon.reproject(-pos_prev(0), -pos_prev(1), est_lat, est_lon);
			_local_origin_lat_lon.initReference(est_lat, est_lon, _time_delayed_us);
		}

		ECL_INFO("Origin set to lat=%.6f, lon=%.6f",
			 _local_origin_lat_lon.getProjectionReferenceLat(), _local_origin_lat_lon.getProjectionReferenceLon());

	} else {
		pos_prev = _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());
	}

	_gpos.setLatLonDeg(latitude, longitude);
	_output_predictor.resetLatLonTo(latitude, longitude);

	const Vector2f delta_horz_pos = getLocalHorizontalPosition() - pos_prev;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - delta_horz_pos);
#endif // CONFIG_EKF2_EXTERNAL_VISION

	updateHorizontalPositionResetStatus(delta_horz_pos);

	if (PX4_ISFINITE(hpos_var)) {
		P.uncorrelateCovarianceSetVariance<2>(State::pos.idx, math::max(sq(0.01f), hpos_var));
	}

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;

	return true;
}

bool Ekf::resetAltitudeTo(const float altitude, const float vpos_var)
{
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	if (!PX4_ISFINITE(_local_origin_alt)) {
		const float local_alt_prev = _gpos.altitude();

		if (isLocalVerticalPositionValid()) {
			_local_origin_alt = altitude - local_alt_prev;

		} else {
			_local_origin_alt = altitude;
		}

		ECL_INFO("Origin alt=%.3f", (double)_local_origin_alt);
	}

	resetHeightTo(altitude, vpos_var);

	return true;
}

void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	if (global_origin_valid()) {
		get_ekf_lpos_accuracy(ekf_eph, ekf_epv);

	} else {
		*ekf_eph = INFINITY;
		*ekf_epv = INFINITY;
	}
}

void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P.trace<2>(State::pos.idx));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_horizontal_deadreckon_time_exceeded) {
#if defined(CONFIG_EKF2_GNSS)

		if (_control_status.flags.gps) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(State::pos.idx + 2, State::pos.idx + 2));
}

void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const
{
	float hvel_err = sqrtf(P.trace<2>(State::vel.idx));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_horizontal_deadreckon_time_exceeded) {
		float vel_err_conservative = 0.0f;

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max(getHagl(), gndclearance) * Vector2f(_aid_src_optical_flow.innovation).norm();
		}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)

		if (_control_status.flags.gps) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_vel.innovation).norm());
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(State::vel.idx + 2, State::vel.idx + 2));
}

void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const
{
	// Do not require limiting by default
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max = NAN;

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// Calculate range finder limits
	const float rangefinder_hagl_min = _range_sensor.getValidMinVal();

	// Allow use of 90% of rangefinder maximum range to allow for angular motion
	const float rangefinder_hagl_max = 0.9f * _range_sensor.getValidMaxVal();

	// TODO : calculate visual odometry limits
	const bool relying_on_rangefinder = isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.rng_hgt);

	// Keep within range sensor limit when using rangefinder as primary height source
	if (relying_on_rangefinder) {
		*hagl_min = rangefinder_hagl_min;
		*hagl_max = rangefinder_hagl_max;
	}

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Keep within flow AND range sensor limits when exclusively using optical flow
	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	if (relying_on_optical_flow) {
		// Calculate optical flow limits
		float flow_hagl_min = _flow_min_distance;
		float flow_hagl_max = _flow_max_distance;

		// only limit optical flow height is dependent on range finder or terrain estimate invalid (precaution)
		if ((!_control_status.flags.opt_flow_terrain && _control_status.flags.rng_terrain)
		    || !isTerrainEstimateValid()
		   ) {
			flow_hagl_min = math::max(flow_hagl_min, rangefinder_hagl_min);
			flow_hagl_max = math::min(flow_hagl_max, rangefinder_hagl_max);
		}

		const float flow_constrained_height = math::constrain(getHagl(), flow_hagl_min, flow_hagl_max);

		// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
		const float flow_vxy_max = 0.5f * _flow_max_rate * flow_constrained_height;

		*vxy_max = flow_vxy_max;
		*hagl_min = flow_hagl_min;
		*hagl_max = flow_hagl_max;
	}

# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_RANGE_FINDER
}

void Ekf::resetGyroBias()
{
	// Zero the gyro bias states
	_state.gyro_bias.zero();

	resetGyroBiasCov();
}

void Ekf::resetAccelBias()
{
	// Zero the accel bias states
	_state.accel_bias.zero();

	resetAccelBiasCov();
}

float Ekf::getHeadingInnovationTestRatio() const
{
	// return the largest heading innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		for (auto &test_ratio_filtered : _aid_src_mag.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_yaw.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_yaw.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getHorizontalVelocityInnovationTestRatio() const
{
	// return the largest velocity innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps) {
		for (int i = 0; i < 2; i++) { // only xy
			test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[i]));
		}
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_vel) {
		for (int i = 0; i < 2; i++) { // only xy
			test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[i]));
		}
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		for (auto &test_ratio_filtered : _aid_src_optical_flow.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getVerticalVelocityInnovationTestRatio() const
{
	// return the largest velocity innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[2]));
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_vel) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[2]));
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getHorizontalPositionInnovationTestRatio() const
{
	// return the largest position innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps) {
		for (auto &test_ratio_filtered : _aid_src_gnss_pos.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_pos) {
		for (auto &test_ratio_filtered : _aid_src_ev_pos.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

	if (_control_status.flags.aux_gpos) {
		test_ratio = math::max(test_ratio, fabsf(_aux_global_position.test_ratio_filtered()));
	}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getVerticalPositionInnovationTestRatio() const
{
	// return the combined vertical position innovation test ratio
	float hgt_sum = 0.f;
	int n_hgt_sources = 0;

#if defined(CONFIG_EKF2_BAROMETER)

	if (_control_status.flags.baro_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_baro_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_gnss_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_ev_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (n_hgt_sources > 0) {
		return math::max(hgt_sum / static_cast<float>(n_hgt_sources), FLT_MIN);
	}

	return NAN;
}

float Ekf::getAirspeedInnovationTestRatio() const
{
#if defined(CONFIG_EKF2_AIRSPEED)

	if (_control_status.flags.fuse_aspd) {
		// return the airspeed fusion innovation test ratio
		return sqrtf(fabsf(_aid_src_airspeed.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_AIRSPEED

	return NAN;
}

float Ekf::getSyntheticSideslipInnovationTestRatio() const
{
#if defined(CONFIG_EKF2_SIDESLIP)

	if (_control_status.flags.fuse_beta) {
		// return the synthetic sideslip innovation test ratio
		return sqrtf(fabsf(_aid_src_sideslip.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_SIDESLIP

	return NAN;
}

float Ekf::getHeightAboveGroundInnovationTestRatio() const
{
	// return the combined HAGL innovation test ratio
	float hagl_sum = 0.f;
	int n_hagl_sources = 0;

#if defined(CONFIG_EKF2_TERRAIN)

# if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_control_status.flags.opt_flow_terrain) {
		hagl_sum += sqrtf(math::max(fabsf(_aid_src_optical_flow.test_ratio_filtered[0]),
					    _aid_src_optical_flow.test_ratio_filtered[1]));
		n_hagl_sources++;
	}

# endif // CONFIG_EKF2_OPTICAL_FLOW

# if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_terrain) {
		hagl_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered));
		n_hagl_sources++;
	}

# endif // CONFIG_EKF2_RANGE_FINDER

#endif // CONFIG_EKF2_TERRAIN

	if (n_hagl_sources > 0) {
		return math::max(hagl_sum / static_cast<float>(n_hagl_sources), FLT_MIN);
	}

	return NAN;
}

uint16_t Ekf::get_ekf_soln_status() const
{
	// LEGACY Mavlink bitmask containing state of estimator solution (see Mavlink ESTIMATOR_STATUS_FLAGS)
	union ekf_solution_status_u {
		struct {
			uint16_t attitude           : 1;
			uint16_t velocity_horiz     : 1;
			uint16_t velocity_vert      : 1;
			uint16_t pos_horiz_rel      : 1;
			uint16_t pos_horiz_abs      : 1;
			uint16_t pos_vert_abs       : 1;
			uint16_t pos_vert_agl       : 1;
			uint16_t const_pos_mode     : 1;
			uint16_t pred_pos_horiz_rel : 1;
			uint16_t pred_pos_horiz_abs : 1;
			uint16_t gps_glitch         : 1;
			uint16_t accel_error        : 1;
		} flags;
		uint16_t value;
	} soln_status{};

	// 1	ESTIMATOR_ATTITUDE	True if the attitude estimate is good
	soln_status.flags.attitude = attitude_valid();

	// 2	ESTIMATOR_VELOCITY_HORIZ	True if the horizontal velocity estimate is good
	soln_status.flags.velocity_horiz = isLocalHorizontalPositionValid();

	// 4	ESTIMATOR_VELOCITY_VERT	True if the vertical velocity estimate is good
	soln_status.flags.velocity_vert = isLocalVerticalVelocityValid() || isLocalVerticalPositionValid();

	// 8	ESTIMATOR_POS_HORIZ_REL	True if the horizontal position (relative) estimate is good
	soln_status.flags.pos_horiz_rel = isLocalHorizontalPositionValid();

	// 16	ESTIMATOR_POS_HORIZ_ABS	True if the horizontal position (absolute) estimate is good
	soln_status.flags.pos_horiz_abs = isGlobalHorizontalPositionValid();

	// 32	ESTIMATOR_POS_VERT_ABS	True if the vertical position (absolute) estimate is good
	soln_status.flags.pos_vert_abs = isVerticalAidingActive();

	// 64	ESTIMATOR_POS_VERT_AGL	True if the vertical position (above ground) estimate is good
#if defined(CONFIG_EKF2_TERRAIN)
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
#endif // CONFIG_EKF2_TERRAIN

	// 128	ESTIMATOR_CONST_POS_MODE	True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
	soln_status.flags.const_pos_mode = _control_status.flags.fake_pos || _control_status.flags.valid_fake_pos
					   || _control_status.flags.vehicle_at_rest;

	// 256	ESTIMATOR_PRED_POS_HORIZ_REL	True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
	soln_status.flags.pred_pos_horiz_rel = isHorizontalAidingActive();

	// 512	ESTIMATOR_PRED_POS_HORIZ_ABS	True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
	soln_status.flags.pred_pos_horiz_abs = _control_status.flags.gps || _control_status.flags.aux_gpos;

	// 1024	ESTIMATOR_GPS_GLITCH	True if the EKF has detected a GPS glitch
#if defined(CONFIG_EKF2_GNSS)
	const bool gps_vel_innov_bad = Vector3f(_aid_src_gnss_vel.test_ratio).max() > 1.f;
	const bool gps_pos_innov_bad = Vector2f(_aid_src_gnss_pos.test_ratio).max() > 1.f;
	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad);
#endif // CONFIG_EKF2_GNSS

	// 2048	ESTIMATOR_ACCEL_ERROR	True if the EKF has detected bad accelerometer data
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical || _fault_status.flags.bad_acc_clipping;

	return soln_status.value;
}

void Ekf::fuse(const VectorState &K, float innovation)
{
	// quat_nominal
	Quatf delta_quat(matrix::AxisAnglef(K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx,
					    0) * (-1.f * innovation)));
	_state.quat_nominal = delta_quat * _state.quat_nominal;
	_state.quat_nominal.normalize();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// vel
	_state.vel = matrix::constrain(_state.vel - K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation, -1.e3f, 1.e3f);

	// pos
	const Vector3f pos_correction = K.slice<State::pos.dof, 1>(State::pos.idx, 0) * (-innovation);

	// Accumulate position in global coordinates
	_gpos += pos_correction;
	_state.pos.zero();
	// Also store altitude in the state vector as this is used for optical flow fusion
	_state.pos(2) = -_gpos.altitude();

	// gyro_bias
	_state.gyro_bias = matrix::constrain(_state.gyro_bias - K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx,
					     0) * innovation,
					     -getGyroBiasLimit(), getGyroBiasLimit());

	// accel_bias
	_state.accel_bias = matrix::constrain(_state.accel_bias - K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx,
					      0) * innovation,
					      -getAccelBiasLimit(), getAccelBiasLimit());

#if defined(CONFIG_EKF2_MAGNETOMETER)

	// mag_I, mag_B
	if (_control_status.flags.mag) {
		_state.mag_I = matrix::constrain(_state.mag_I - K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation, -1.f,
						 1.f);
		_state.mag_B = matrix::constrain(_state.mag_B - K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation,
						 -getMagBiasLimit(), getMagBiasLimit());
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	// wind_vel
	if (_control_status.flags.wind) {
		_state.wind_vel = matrix::constrain(_state.wind_vel - K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx,
						    0) * innovation, -1.e2f, 1.e2f);
	}

#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	_state.terrain = math::constrain(_state.terrain - K(State::terrain.idx) * innovation, -1e4f, 1e4f);
#endif // CONFIG_EKF2_TERRAIN
}

void Ekf::updateDeadReckoningStatus()
{
	updateHorizontalDeadReckoningstatus();
	updateVerticalDeadReckoningStatus();
}

void Ekf::updateHorizontalDeadReckoningstatus()
{
	bool inertial_dead_reckoning = true;
	bool aiding_expected_in_air = false;

	// velocity aiding active
	if ((_control_status.flags.gps || _control_status.flags.ev_vel)
	    && isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

	// position aiding active
	if ((_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.aux_gpos)
	    && isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	// optical flow active
	if (_control_status.flags.opt_flow
	    && isRecent(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;

	} else {
		if (!_control_status.flags.in_air && (_params.flow_ctrl == 1)
		    && isRecent(_aid_src_optical_flow.timestamp_sample, _params.no_aid_timeout_max)
		   ) {
			// currently landed, but optical flow aiding should be possible once in air
			aiding_expected_in_air = true;
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)

	// air data aiding active
	if ((_control_status.flags.fuse_aspd && isRecent(_aid_src_airspeed.time_last_fuse, _params.no_aid_timeout_max))
	    && (_control_status.flags.fuse_beta && isRecent(_aid_src_sideslip.time_last_fuse, _params.no_aid_timeout_max))
	   ) {
		// wind_dead_reckoning: no other aiding but air data
		_control_status.flags.wind_dead_reckoning = inertial_dead_reckoning;

		// air data aiding is active, we're not inertial dead reckoning
		inertial_dead_reckoning = false;

	} else {
		_control_status.flags.wind_dead_reckoning = false;

		if (!_control_status.flags.in_air && _control_status.flags.fixed_wing
		    && (_params.beta_fusion_enabled == 1)
		    && (_params.arsp_thr > 0.f) && isRecent(_aid_src_airspeed.timestamp_sample, _params.no_aid_timeout_max)
		   ) {
			// currently landed, but air data aiding should be possible once in air
			aiding_expected_in_air = true;
		}
	}

#endif // CONFIG_EKF2_AIRSPEED

	// zero velocity update
	if (isRecent(_zero_velocity_update.time_last_fuse(), _params.no_aid_timeout_max)) {
		// only respect as a valid aiding source now if we expect to have another valid source once in air
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	if (_control_status.flags.valid_fake_pos && isRecent(_aid_src_fake_pos.time_last_fuse, _params.no_aid_timeout_max)) {
		// only respect as a valid aiding source now if we expect to have another valid source once in air
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	if (inertial_dead_reckoning) {
		if (isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.valid_timeout_max)) {
			// deadreckon time exceeded
			if (!_horizontal_deadreckon_time_exceeded) {
				ECL_WARN("horizontal dead reckon time exceeded");
				_horizontal_deadreckon_time_exceeded = true;
			}
		}

	} else {
		if (_time_delayed_us > _params.no_aid_timeout_max) {
			_time_last_horizontal_aiding = _time_delayed_us - _params.no_aid_timeout_max;
		}

		_horizontal_deadreckon_time_exceeded = false;

	}

	_control_status.flags.inertial_dead_reckoning = inertial_dead_reckoning;
}

void Ekf::updateVerticalDeadReckoningStatus()
{
	if (isVerticalPositionAidingActive()) {
		_time_last_v_pos_aiding = _time_last_hgt_fuse;
		_vertical_position_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_pos_aiding, (uint64_t)_params.valid_timeout_max)) {
		_vertical_position_deadreckon_time_exceeded = true;
	}

	if (isVerticalVelocityAidingActive()) {
		_time_last_v_vel_aiding = _time_last_ver_vel_fuse;
		_vertical_velocity_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_vel_aiding, (uint64_t)_params.valid_timeout_max)
		   && _vertical_position_deadreckon_time_exceeded) {

		_vertical_velocity_deadreckon_time_exceeded = true;
	}
}

Vector3f Ekf::getRotVarBody() const
{
	const matrix::SquareMatrix3f rot_cov_body = getStateCovariance<State::quat_nominal>();
	return matrix::SquareMatrix3f(_R_to_earth.T() * rot_cov_body * _R_to_earth).diag();
}

Vector3f Ekf::getRotVarNed() const
{
	const matrix::SquareMatrix3f rot_cov_ned = getStateCovariance<State::quat_nominal>();
	return rot_cov_ned.diag();
}

float Ekf::getYawVar() const
{
	return getRotVarNed()(2);
}

float Ekf::getTiltVariance() const
{
	const Vector3f rot_var_ned = getRotVarNed();
	return rot_var_ned(0) + rot_var_ned(1);
}

#if defined(CONFIG_EKF2_BAROMETER)
void Ekf::updateGroundEffect()
{
	if (_control_status.flags.in_air && !_control_status.flags.fixed_wing) {
#if defined(CONFIG_EKF2_TERRAIN)

		if (isTerrainEstimateValid()) {
			// automatically set ground effect if terrain is valid
			float height = getHagl();
			_control_status.flags.gnd_effect = (height < _params.gnd_effect_max_hgt);

		} else
#endif // CONFIG_EKF2_TERRAIN
			if (_control_status.flags.gnd_effect) {
				// Turn off ground effect compensation if it times out
				if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {
					_control_status.flags.gnd_effect = false;
				}
			}

	} else {
		_control_status.flags.gnd_effect = false;
	}
}
#endif // CONFIG_EKF2_BAROMETER


void Ekf::updateIMUBiasInhibit(const imuSample &imu_delayed)
{
	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	{
		const Vector3f gyro_corrected = imu_delayed.delta_ang / imu_delayed.delta_ang_dt - _state.gyro_bias;

		const float alpha = math::constrain((imu_delayed.delta_ang_dt / _params.acc_bias_learn_tc), 0.f, 1.f);
		const float beta = 1.f - alpha;

		_ang_rate_magnitude_filt = fmaxf(gyro_corrected.norm(), beta * _ang_rate_magnitude_filt);
	}

	{
		const Vector3f accel_corrected = imu_delayed.delta_vel / imu_delayed.delta_vel_dt - _state.accel_bias;

		const float alpha = math::constrain((imu_delayed.delta_vel_dt / _params.acc_bias_learn_tc), 0.f, 1.f);
		const float beta = 1.f - alpha;

		_accel_magnitude_filt = fmaxf(accel_corrected.norm(), beta * _accel_magnitude_filt);
	}


	const bool is_manoeuvre_level_high = (_ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim)
					     || (_accel_magnitude_filt > _params.acc_bias_learn_acc_lim);


	// gyro bias inhibit
	const bool do_inhibit_all_gyro_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias));

	for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
		bool is_bias_observable = true; // TODO: gyro bias conditions
		_gyro_bias_inhibit[index] = do_inhibit_all_gyro_axes || !is_bias_observable;
	}

	// accel bias inhibit
	const bool do_inhibit_all_accel_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::AccelBias))
					       || is_manoeuvre_level_high
					       || _fault_status.flags.bad_acc_vertical;

	for (unsigned index = 0; index < State::accel_bias.dof; index++) {
		bool is_bias_observable = true;

		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos) {
			// when using fake position (but not fake height) only consider an accel bias observable if aligned with the gravity vector
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f); // cos 15 degrees ~= 0.966
		}

		_accel_bias_inhibit[index] = do_inhibit_all_accel_axes || imu_delayed.delta_vel_clipping[index] || !is_bias_observable;
	}
}

void Ekf::fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index)
{
	VectorState K;  // Kalman gain vector for any single observation - sequential fusion is used.

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		K(row) = P(row, state_index) / innov_var;
	}

	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// This is extremely expensive to compute. Use for debugging purposes only.
	auto A = matrix::eye<float, State::size>();
	VectorState H;
	H(state_index) = 1.f;
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
	VectorState PH = P.row(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetric if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	// P (or "P_temp") is not symmetric so we must take the column
	PH = P.col(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	constrainStateVariances();

	// apply the state corrections
	fuse(K, innov);
}

bool Ekf::measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation)
{
	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// This is extremely expensive to compute. Use for debugging purposes only.
	auto A = matrix::eye<float, State::size>();
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
	VectorState PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetrical if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	constrainStateVariances();

	// apply the state corrections
	fuse(K, innovation);
	return true;
}

void Ekf::updateAidSourceStatus(estimator_aid_source1d_s &status, const uint64_t &timestamp_sample,
				const float &observation, const float &observation_variance,
				const float &innovation, const float &innovation_variance,
				float innovation_gate) const
{
	bool innovation_rejected = false;

	const float test_ratio = sq(innovation) / (sq(innovation_gate) * innovation_variance);

	if ((status.timestamp_sample > 0) && (timestamp_sample > status.timestamp_sample)) {

		const float dt_s = math::constrain((timestamp_sample - status.timestamp_sample) * 1e-6f, 0.001f, 1.f);

		static constexpr float tau = 0.5f;
		const float alpha = math::constrain(dt_s / (dt_s + tau), 0.f, 1.f);

		// test_ratio_filtered
		if (PX4_ISFINITE(status.test_ratio_filtered)) {
			status.test_ratio_filtered += alpha * (matrix::sign(innovation) * test_ratio - status.test_ratio_filtered);

		} else {
			// otherwise, init the filtered test ratio
			status.test_ratio_filtered = test_ratio;
		}

		// innovation_filtered
		if (PX4_ISFINITE(status.innovation_filtered)) {
			status.innovation_filtered += alpha * (innovation - status.innovation_filtered);

		} else {
			// otherwise, init the filtered innovation
			status.innovation_filtered = innovation;
		}


		// limit extremes in filtered values
		static constexpr float kNormalizedInnovationLimit = 2.f;
		static constexpr float kTestRatioLimit = sq(kNormalizedInnovationLimit);

		if (test_ratio > kTestRatioLimit) {

			status.test_ratio_filtered = math::constrain(status.test_ratio_filtered, -kTestRatioLimit, kTestRatioLimit);

			const float innov_limit = kNormalizedInnovationLimit * innovation_gate * sqrtf(innovation_variance);
			status.innovation_filtered = math::constrain(status.innovation_filtered, -innov_limit, innov_limit);
		}

	} else {
		// invalid timestamp_sample, reset
		status.test_ratio_filtered = test_ratio;
		status.innovation_filtered = innovation;
	}

	status.test_ratio = test_ratio;

	status.observation = observation;
	status.observation_variance = observation_variance;

	status.innovation = innovation;
	status.innovation_variance = innovation_variance;

	if ((test_ratio > 1.f)
	    || !PX4_ISFINITE(test_ratio)
	    || !PX4_ISFINITE(status.innovation)
	    || !PX4_ISFINITE(status.innovation_variance)
	   ) {
		innovation_rejected = true;
	}

	status.timestamp_sample = timestamp_sample;

	// if any of the innovations are rejected, then the overall innovation is rejected
	status.innovation_rejected = innovation_rejected;

	// reset
	status.fused = false;
}

void Ekf::clearInhibitedStateKalmanGains(VectorState &K) const
{
	for (unsigned i = 0; i < State::gyro_bias.dof; i++) {
		if (_gyro_bias_inhibit[i]) {
			K(State::gyro_bias.idx + i) = 0.f;
		}
	}

	for (unsigned i = 0; i < State::accel_bias.dof; i++) {
		if (_accel_bias_inhibit[i]) {
			K(State::accel_bias.idx + i) = 0.f;
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_I.dof; i++) {
			K(State::mag_I.idx + i) = 0.f;
		}
	}

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_B.dof; i++) {
			K(State::mag_B.idx + i) = 0.f;
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	if (!_control_status.flags.wind) {
		for (unsigned i = 0; i < State::wind_vel.dof; i++) {
			K(State::wind_vel.idx + i) = 0.f;
		}
	}

#endif // CONFIG_EKF2_WIND
}

float Ekf::getHeadingInnov() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.innovation).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.innovation;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.innovation;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}

float Ekf::getHeadingInnovVar() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.innovation_variance).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.innovation_variance;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.innovation_variance;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}

float Ekf::getHeadingInnovRatio() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.test_ratio).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.test_ratio;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.test_ratio;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}

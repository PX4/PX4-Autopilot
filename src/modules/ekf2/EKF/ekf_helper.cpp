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
#include "python/ekf_derivation/generated/quat_var_to_rot_var.h"
#include "python/ekf_derivation/generated/rot_var_ned_to_lower_triangular_quat_cov.h"

#include <mathlib/mathlib.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <cstdlib>

void Ekf::resetHorizontalVelocityToZero()
{
	_information_events.flags.reset_vel_to_zero = true;
	ECL_INFO("reset velocity to zero");
	// Used when falling back to non-aiding mode of operation
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f}, 25.f);
}

void Ekf::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	if (PX4_ISFINITE(new_horz_vel_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx, math::max(sq(0.01f), new_horz_vel_var(0)));
	}

	if (PX4_ISFINITE(new_horz_vel_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));
	}

	_output_predictor.resetHorizontalVelocityTo(delta_horz_vel);

	// record the state change
	if (_state_reset_status.reset_count.velNE == _state_reset_count_prev.velNE) {
		_state_reset_status.velNE_change = delta_horz_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velNE_change += delta_horz_vel;
	}

	_state_reset_status.reset_count.velNE++;

	// Reset the timout timer
	_time_last_hor_vel_fuse = _time_delayed_us;
}

void Ekf::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	if (PX4_ISFINITE(new_vert_vel_var)) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 2, math::max(sq(0.01f), new_vert_vel_var));
	}

	_output_predictor.resetVerticalVelocityTo(delta_vert_vel);

	// record the state change
	if (_state_reset_status.reset_count.velD == _state_reset_count_prev.velD) {
		_state_reset_status.velD_change = delta_vert_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velD_change += delta_vert_vel;
	}

	_state_reset_status.reset_count.velD++;

	// Reset the timout timer
	_time_last_ver_vel_fuse = _time_delayed_us;
}

void Ekf::resetHorizontalPositionToLastKnown()
{
	ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));

	_information_events.flags.reset_pos_to_last_known = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalPositionTo(_last_known_pos.xy(), sq(_params.pos_noaid_noise));
}

void Ekf::resetHorizontalPositionTo(const Vector2f &new_horz_pos, const Vector2f &new_horz_pos_var)
{
	const Vector2f delta_horz_pos{new_horz_pos - Vector2f{_state.pos}};
	_state.pos.xy() = new_horz_pos;

	if (PX4_ISFINITE(new_horz_pos_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, math::max(sq(0.01f), new_horz_pos_var(0)));
	}

	if (PX4_ISFINITE(new_horz_pos_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, math::max(sq(0.01f), new_horz_pos_var(1)));
	}

	_output_predictor.resetHorizontalPositionTo(delta_horz_pos);

	// record the state change
	if (_state_reset_status.reset_count.posNE == _state_reset_count_prev.posNE) {
		_state_reset_status.posNE_change = delta_horz_pos;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posNE_change += delta_horz_pos;
	}

	_state_reset_status.reset_count.posNE++;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - _state_reset_status.posNE_change);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	//_gps_pos_b_est.setBias(_gps_pos_b_est.getBias() + _state_reset_status.posNE_change);

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;
}

bool Ekf::isHeightResetRequired() const
{
	// check if height is continuously failing because of accel errors
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max);

	return (continuous_bad_accel_hgt || hgt_fusion_timeout);
}

void Ekf::resetVerticalPositionTo(const float new_vert_pos, float new_vert_pos_var)
{
	const float old_vert_pos = _state.pos(2);
	_state.pos(2) = new_vert_pos;

	if (PX4_ISFINITE(new_vert_pos_var)) {
		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, math::max(sq(0.01f), new_vert_pos_var));
	}

	const float delta_z = new_vert_pos - old_vert_pos;

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_predictor.resetVerticalPositionTo(new_vert_pos, delta_z);

	// record the state change
	if (_state_reset_status.reset_count.posD == _state_reset_count_prev.posD) {
		_state_reset_status.posD_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posD_change += delta_z;
	}

	_state_reset_status.reset_count.posD++;

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_b_est.setBias(_baro_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - delta_z);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + delta_z);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_TERRAIN)
	terrainHandleVerticalPositionReset(delta_z);
#endif

	// Reset the timout timer
	_time_last_hgt_fuse = _time_delayed_us;
}

void Ekf::resetVerticalVelocityToZero()
{
	// we don't know what the vertical velocity is, so set it to zero
	// Set the variance to a value large enough to allow the state to converge quickly
	// that does not destabilise the filter
	resetVerticalVelocityTo(0.0f, 10.f);
}

void Ekf::constrainStates()
{
	_state.quat_nominal = matrix::constrain(_state.quat_nominal, -1.0f, 1.0f);
	_state.vel = matrix::constrain(_state.vel, -1000.0f, 1000.0f);
	_state.pos = matrix::constrain(_state.pos, -1.e6f, 1.e6f);

	const float gyro_bias_limit = getGyroBiasLimit();
	_state.gyro_bias = matrix::constrain(_state.gyro_bias, -gyro_bias_limit, gyro_bias_limit);

	const float accel_bias_limit = getAccelBiasLimit();
	_state.accel_bias = matrix::constrain(_state.accel_bias, -accel_bias_limit, accel_bias_limit);

	_state.mag_I = matrix::constrain(_state.mag_I, -1.0f, 1.0f);
#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_B = matrix::constrain(_state.mag_B, -getMagBiasLimit(), getMagBiasLimit());
#endif // CONFIG_EKF2_MAGNETOMETER

	_state.wind_vel = matrix::constrain(_state.wind_vel, -100.0f, 100.0f);
}

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
float Ekf::compensateBaroForDynamicPressure(const float baro_alt_uncompensated) const
{
	if (_control_status.flags.wind && local_position_is_valid()) {
		// calculate static pressure error = Pmeas - Ptruth
		// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
		// negative X and Y directions. Used to correct baro data for positional errors

		// Calculate airspeed in body frame
		const Vector3f vel_imu_rel_body_ned = _R_to_earth * (_ang_rate_delayed_raw % _params.imu_pos_body);
		const Vector3f velocity_earth = _state.vel - vel_imu_rel_body_ned;

		const Vector3f wind_velocity_earth(_state.wind_vel(0), _state.wind_vel(1), 0.0f);

		const Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;

		const Vector3f airspeed_body = _state.quat_nominal.rotateVectorInverse(airspeed_earth);

		const Vector3f K_pstatic_coef(
			airspeed_body(0) >= 0.f ? _params.static_pressure_coef_xp : _params.static_pressure_coef_xn,
			airspeed_body(1) >= 0.f ? _params.static_pressure_coef_yp : _params.static_pressure_coef_yn,
			_params.static_pressure_coef_z);

		const Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.max_correction_airspeed));

		const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

		// correct baro measurement using pressure error estimate and assuming sea level gravity
		return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
	}

	// otherwise return the uncorrected baro measurement
	return baro_alt_uncompensated;
}
#endif // CONFIG_EKF2_BARO_COMPENSATION

// calculate the earth rotation vector
Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

void Ekf::getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2],  float &vpos) const
{
	hvel[0] = _aid_src_gnss_vel.innovation[0];
	hvel[1] = _aid_src_gnss_vel.innovation[1];
	vvel    = _aid_src_gnss_vel.innovation[2];

	hpos[0] = _aid_src_gnss_pos.innovation[0];
	hpos[1] = _aid_src_gnss_pos.innovation[1];
	vpos    = _aid_src_gnss_hgt.innovation;
}

void Ekf::getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos)  const
{
	hvel[0] = _aid_src_gnss_vel.innovation_variance[0];
	hvel[1] = _aid_src_gnss_vel.innovation_variance[1];
	vvel    = _aid_src_gnss_vel.innovation_variance[2];

	hpos[0] = _aid_src_gnss_pos.innovation_variance[0];
	hpos[1] = _aid_src_gnss_pos.innovation_variance[1];
	vpos    = _aid_src_gnss_hgt.innovation_variance;
}

void Ekf::getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = fmaxf(_aid_src_gnss_vel.test_ratio[0], _aid_src_gnss_vel.test_ratio[1]);
	vvel = _aid_src_gnss_vel.test_ratio[2];

	hpos = fmaxf(_aid_src_gnss_pos.test_ratio[0], _aid_src_gnss_pos.test_ratio[1]);
	vpos = _aid_src_gnss_hgt.test_ratio;
}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void Ekf::getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _aid_src_ev_vel.innovation[0];
	hvel[1] = _aid_src_ev_vel.innovation[1];
	vvel    = _aid_src_ev_vel.innovation[2];

	hpos[0] = _aid_src_ev_pos.innovation[0];
	hpos[1] = _aid_src_ev_pos.innovation[1];
	vpos    = _aid_src_ev_hgt.innovation;
}

void Ekf::getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _aid_src_ev_vel.innovation_variance[0];
	hvel[1] = _aid_src_ev_vel.innovation_variance[1];
	vvel    = _aid_src_ev_vel.innovation_variance[2];

	hpos[0] = _aid_src_ev_pos.innovation_variance[0];
	hpos[1] = _aid_src_ev_pos.innovation_variance[1];
	vpos    = _aid_src_ev_hgt.innovation_variance;
}

void Ekf::getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = fmaxf(_aid_src_ev_vel.test_ratio[0], _aid_src_ev_vel.test_ratio[1]);
	vvel = _aid_src_ev_vel.test_ratio[2];

	hpos = fmaxf(_aid_src_ev_pos.test_ratio[0], _aid_src_ev_pos.test_ratio[1]);
	vpos = _aid_src_ev_hgt.test_ratio;
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
void Ekf::getAuxVelInnov(float aux_vel_innov[2]) const
{
	aux_vel_innov[0] = _aid_src_aux_vel.innovation[0];
	aux_vel_innov[1] = _aid_src_aux_vel.innovation[1];
}

void Ekf::getAuxVelInnovVar(float aux_vel_innov_var[2]) const
{
	aux_vel_innov_var[0] = _aid_src_aux_vel.innovation_variance[0];
	aux_vel_innov_var[1] = _aid_src_aux_vel.innovation_variance[1];
}
#endif // CONFIG_EKF2_AUXVEL

// get the state vector at the delayed time horizon
matrix::Vector<float, 24> Ekf::getStateAtFusionHorizonAsVector() const
{
	matrix::Vector<float, 24> state;
	state.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx, 0) = _state.quat_nominal;
	state.slice<State::vel.dof, 1>(State::vel.idx, 0) = _state.vel;
	state.slice<State::pos.dof, 1>(State::pos.idx, 0) = _state.pos;
	state.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx, 0) = _state.gyro_bias;
	state.slice<State::accel_bias.dof, 1>(State::accel_bias.idx, 0) = _state.accel_bias;
	state.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) = _state.mag_I;
	state.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) = _state.mag_B;
	state.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = _state.wind_vel;
	return state;
}

bool Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	origin_time = _pos_ref.getProjectionReferenceTimestamp();
	latitude = _pos_ref.getProjectionReferenceLat();
	longitude = _pos_ref.getProjectionReferenceLon();
	origin_alt  = getEkfGlobalOriginAltitude();
	return _NED_origin_initialised;
}

bool Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude)
{
	// sanity check valid latitude/longitude and altitude anywhere between the Mariana Trench and edge of Space
	if (PX4_ISFINITE(latitude) && (abs(latitude) <= 90)
	&& PX4_ISFINITE(longitude) && (abs(longitude) <= 180)
	&& PX4_ISFINITE(altitude) && (altitude > -12'000.f) && (altitude < 100'000.f)
	) {
		bool current_pos_available = false;
		double current_lat = static_cast<double>(NAN);
		double current_lon = static_cast<double>(NAN);

		// if we are already doing aiding, correct for the change in position since the EKF started navigating
		if (_pos_ref.isInitialized() && isHorizontalAidingActive()) {
			_pos_ref.reproject(_state.pos(0), _state.pos(1), current_lat, current_lon);
			current_pos_available = true;
		}

		const float gps_alt_ref_prev = getEkfGlobalOriginAltitude();

		// reinitialize map projection to latitude, longitude, altitude, and reset position
		_pos_ref.initReference(latitude, longitude, _time_delayed_us);
		_gps_alt_ref = altitude;

		const float mag_declination_gps = get_mag_declination_radians(latitude, longitude);
		const float mag_inclination_gps = get_mag_inclination_radians(latitude, longitude);
		const float mag_strength_gps = get_mag_strength_gauss(latitude, longitude);

		if (PX4_ISFINITE(mag_declination_gps) && PX4_ISFINITE(mag_inclination_gps) && PX4_ISFINITE(mag_strength_gps)) {
			_mag_declination_gps = mag_declination_gps;
			_mag_inclination_gps = mag_inclination_gps;
			_mag_strength_gps = mag_strength_gps;

			_wmm_gps_time_last_set = _time_delayed_us;
		}

		// We don't know the uncertainty of the origin
		_gpos_origin_eph = 0.f;
		_gpos_origin_epv = 0.f;

		_NED_origin_initialised = true;

		// minimum change in position or height that triggers a reset
		static constexpr float MIN_RESET_DIST_M = 0.01f;

		if (current_pos_available) {
			// reset horizontal position
			Vector2f position = _pos_ref.project(current_lat, current_lon);

			if (Vector2f(position - Vector2f(_state.pos)).longerThan(MIN_RESET_DIST_M)) {
				resetHorizontalPositionTo(position);
			}
		}

		// reset vertical position (if there's any change)
		if (fabsf(altitude - gps_alt_ref_prev) > MIN_RESET_DIST_M) {
			// determine current z
			float current_alt = -_state.pos(2) + gps_alt_ref_prev;

			const float gps_hgt_bias = _gps_hgt_b_est.getBias();
			resetVerticalPositionTo(_gps_alt_ref - current_alt);

			// preserve GPS height bias
			_gps_hgt_b_est.setBias(gps_hgt_bias);
		}

		return true;
	}

	return false;
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// report absolute accuracy taking into account the uncertainty in location of the origin
	// If not aiding, return 0 for horizontal position estimate as no estimate is available
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P.trace<2>(State::pos.idx) + sq(_gpos_origin_eph));

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_control_status.flags.inertial_dead_reckoning) {
		if (_control_status.flags.gps) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(State::pos.idx + 2, State::pos.idx + 2) + sq(_gpos_origin_epv));
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P.trace<2>(State::pos.idx));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_horizontal_deadreckon_time_exceeded) {
		if (_control_status.flags.gps) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(State::pos.idx + 2, State::pos.idx + 2));
}

// get the 1-sigma horizontal and vertical velocity uncertainty
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
			vel_err_conservative = math::max((_terrain_vpos - _state.pos(2)), gndclearance) * Vector2f(_aid_src_optical_flow.innovation).norm();
		}
#endif // CONFIG_EKF2_OPTICAL_FLOW

		if (_control_status.flags.gps) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

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

/*
Returns the following vehicle control limits required by the estimator to keep within sensor limitations.
vxy_max : Maximum ground relative horizontal speed (meters/sec). NaN when limiting is not needed.
vz_max : Maximum ground relative vertical speed (meters/sec). NaN when limiting is not needed.
hagl_min : Minimum height above ground (meters). NaN when limiting is not needed.
hagl_max : Maximum height above ground (meters). NaN when limiting is not needed.
*/
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

	// Allow use of 75% of rangefinder maximum range to allow for angular motion
	const float rangefinder_hagl_max = 0.75f * _range_sensor.getValidMaxVal();

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
		const float flow_hagl_min = fmaxf(rangefinder_hagl_min, _flow_min_distance);
		const float flow_hagl_max = fminf(rangefinder_hagl_max, _flow_max_distance);

		const float flow_constrained_height = math::constrain(_terrain_vpos - _state.pos(2), flow_hagl_min, flow_hagl_max);

		// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
		const float flow_vxy_max = 0.5f * _flow_max_rate * flow_constrained_height;

		*vxy_max = flow_vxy_max;
		*hagl_min = flow_hagl_min;
		*hagl_max = flow_hagl_max;
	}
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_RANGE_FINDER
}

void Ekf::resetImuBias()
{
	resetGyroBias();
	resetAccelBias();
}

void Ekf::resetGyroBias()
{
	// Zero the gyro bias states
	_state.gyro_bias.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<State::gyro_bias.dof>(State::gyro_bias.idx, sq(_params.switch_on_gyro_bias));

	// Set previous frame values
	_prev_gyro_bias_var = getStateVariance<State::gyro_bias>();
}

void Ekf::resetAccelBias()
{
	// Zero the accel bias states
	_state.accel_bias.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<State::accel_bias.dof>(State::accel_bias.idx, sq(_params.switch_on_accel_bias));

	// Set previous frame values
	_prev_accel_bias_var = getStateVariance<State::accel_bias>();
}

// get EKF innovation consistency check status information comprising of:
// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
// Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
void Ekf::get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas,
				     float &hagl, float &beta) const
{
	// return the integer bitmask containing the consistency check pass/fail status
	status = _innov_check_fail_status.value;

	// return the largest magnetometer innovation test ratio
	mag = 0.f;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (_control_status.flags.mag_hdg) {
		mag = math::max(mag, sqrtf(_aid_src_mag_heading.test_ratio));
	}

	if (_control_status.flags.mag_3D) {
		mag = math::max(mag, sqrtf(Vector3f(_aid_src_mag.test_ratio).max()));
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)
	if (_control_status.flags.gps_yaw) {
		mag = math::max(mag, sqrtf(_aid_src_gnss_yaw.test_ratio));
	}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_yaw) {
		mag = math::max(mag, sqrtf(_aid_src_ev_yaw.test_ratio));
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// return the largest velocity and position innovation test ratio
	vel = NAN;
	pos = NAN;

	if (_control_status.flags.gps) {
		float gps_vel = sqrtf(Vector3f(_aid_src_gnss_vel.test_ratio).max());
		vel = math::max(gps_vel, FLT_MIN);

		float gps_pos = sqrtf(Vector2f(_aid_src_gnss_pos.test_ratio).max());
		pos = math::max(gps_pos, FLT_MIN);
	}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_vel) {
		float ev_vel = sqrtf(Vector3f(_aid_src_ev_vel.test_ratio).max());
		vel = math::max(vel, ev_vel, FLT_MIN);
	}

	if (_control_status.flags.ev_pos) {
		float ev_pos = sqrtf(Vector2f(_aid_src_ev_pos.test_ratio).max());
		pos = math::max(pos, ev_pos, FLT_MIN);
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		float of_vel = sqrtf(Vector2f(_aid_src_optical_flow.test_ratio).max());
		vel = math::max(of_vel, FLT_MIN);
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// return the combined vertical position innovation test ratio
	float hgt_sum = 0.f;
	int n_hgt_sources = 0;

#if defined(CONFIG_EKF2_BAROMETER)
	if (_control_status.flags.baro_hgt) {
		hgt_sum += sqrtf(_aid_src_baro_hgt.test_ratio);
		n_hgt_sources++;
	}
#endif // CONFIG_EKF2_BAROMETER

	if (_control_status.flags.gps_hgt) {
		hgt_sum += sqrtf(_aid_src_gnss_hgt.test_ratio);
		n_hgt_sources++;
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_control_status.flags.rng_hgt) {
		hgt_sum += sqrtf(_aid_src_rng_hgt.test_ratio);
		n_hgt_sources++;
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_hgt) {
		hgt_sum += sqrtf(_aid_src_ev_hgt.test_ratio);
		n_hgt_sources++;
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (n_hgt_sources > 0) {
		hgt = math::max(hgt_sum / static_cast<float>(n_hgt_sources), FLT_MIN);

	} else {
		hgt = NAN;
	}

#if defined(CONFIG_EKF2_AIRSPEED)
	// return the airspeed fusion innovation test ratio
	tas = sqrtf(_aid_src_airspeed.test_ratio);
#endif // CONFIG_EKF2_AIRSPEED

	hagl = NAN;
#if defined(CONFIG_EKF2_TERRAIN)
# if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_hagl_sensor_status.flags.range_finder) {
		// return the terrain height innovation test ratio
		hagl = sqrtf(_aid_src_terrain_range_finder.test_ratio);
	}
#endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	if (_hagl_sensor_status.flags.flow) {
		// return the terrain height innovation test ratio
		hagl = sqrtf(math::max(_aid_src_terrain_optical_flow.test_ratio[0], _aid_src_terrain_optical_flow.test_ratio[1]));
	}
# endif // CONFIG_EKF2_OPTICAL_FLOW
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_SIDESLIP)
	// return the synthetic sideslip innovation test ratio
	beta = sqrtf(_aid_src_sideslip.test_ratio);
#endif // CONFIG_EKF2_SIDESLIP
}

// return a bitmask integer that describes which state estimates are valid
void Ekf::get_ekf_soln_status(uint16_t *status) const
{
	ekf_solution_status_u soln_status{};
	// TODO: Is this accurate enough?
	soln_status.flags.attitude = _control_status.flags.tilt_align && _control_status.flags.yaw_align && (_fault_status.value == 0);
	soln_status.flags.velocity_horiz = (isHorizontalAidingActive() || (_control_status.flags.fuse_beta && _control_status.flags.fuse_aspd)) && (_fault_status.value == 0);
	soln_status.flags.velocity_vert = (_control_status.flags.baro_hgt || _control_status.flags.ev_hgt || _control_status.flags.gps_hgt || _control_status.flags.rng_hgt) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_rel = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_abs = (_control_status.flags.gps || _control_status.flags.ev_pos) && (_fault_status.value == 0);
	soln_status.flags.pos_vert_abs = soln_status.flags.velocity_vert;
#if defined(CONFIG_EKF2_TERRAIN)
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
#endif // CONFIG_EKF2_TERRAIN
	soln_status.flags.const_pos_mode = !soln_status.flags.velocity_horiz;
	soln_status.flags.pred_pos_horiz_rel = soln_status.flags.pos_horiz_rel;
	soln_status.flags.pred_pos_horiz_abs = soln_status.flags.pos_horiz_abs;

	bool mag_innov_good = true;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (_control_status.flags.mag_hdg) {
		if (_aid_src_mag_heading.test_ratio < 1.f) {
			mag_innov_good = false;
		}

	} else if (_control_status.flags.mag_3D) {
		if (Vector3f(_aid_src_mag.test_ratio).max() < 1.f) {
			mag_innov_good = false;
		}
	}
#endif // CONFIG_EKF2_MAGNETOMETER

	const bool gps_vel_innov_bad = Vector3f(_aid_src_gnss_vel.test_ratio).max() > 1.f;
	const bool gps_pos_innov_bad = Vector2f(_aid_src_gnss_pos.test_ratio).max() > 1.f;

	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad) && mag_innov_good;
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical;
	*status = soln_status.value;
}

void Ekf::fuse(const VectorState &K, float innovation)
{
	_state.quat_nominal -= K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx, 0) * innovation;
	_state.quat_nominal.normalize();
	_R_to_earth = Dcmf(_state.quat_nominal);

	_state.vel -= K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation;
	_state.pos -= K.slice<State::pos.dof, 1>(State::pos.idx, 0) * innovation;
	_state.gyro_bias -= K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx, 0) * innovation;
	_state.accel_bias -= K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx, 0) * innovation;
	_state.mag_I -= K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation;
	_state.mag_B -= K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation;
	_state.wind_vel -= K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) * innovation;
}

void Ekf::uncorrelateQuatFromOtherStates()
{
	P.uncorrelateCovarianceBlock<State::quat_nominal.dof>(State::quat_nominal.idx);
}

void Ekf::updateDeadReckoningStatus()
{
	updateHorizontalDeadReckoningstatus();
	updateVerticalDeadReckoningStatus();
}

void Ekf::updateHorizontalDeadReckoningstatus()
{
	const bool velPosAiding = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.ev_vel)
				  && (isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
				      || isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max));

	bool optFlowAiding = false;
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	optFlowAiding = _control_status.flags.opt_flow && isRecent(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max);
#endif // CONFIG_EKF2_OPTICAL_FLOW

	bool airDataAiding = false;

#if defined(CONFIG_EKF2_AIRSPEED)
	airDataAiding = _control_status.flags.wind &&
				   isRecent(_aid_src_airspeed.time_last_fuse, _params.no_aid_timeout_max) &&
				   isRecent(_aid_src_sideslip.time_last_fuse, _params.no_aid_timeout_max);

	_control_status.flags.wind_dead_reckoning = !velPosAiding && !optFlowAiding && airDataAiding;
#else
	_control_status.flags.wind_dead_reckoning = false;
#endif // CONFIG_EKF2_AIRSPEED

	_control_status.flags.inertial_dead_reckoning = !velPosAiding && !optFlowAiding && !airDataAiding;

	if (!_control_status.flags.inertial_dead_reckoning) {
		if (_time_delayed_us > _params.no_aid_timeout_max) {
			_time_last_horizontal_aiding = _time_delayed_us - _params.no_aid_timeout_max;
		}
	}

	// report if we have been deadreckoning for too long, initial state is deadreckoning until aiding is present
	bool deadreckon_time_exceeded = isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.valid_timeout_max);

	if (!_horizontal_deadreckon_time_exceeded && deadreckon_time_exceeded) {
		// deadreckon time now exceeded
		ECL_WARN("dead reckon time exceeded");
	}

	_horizontal_deadreckon_time_exceeded = deadreckon_time_exceeded;
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

// calculate the variances for the rotation vector equivalent
Vector3f Ekf::calcRotVecVariances() const
{
	Vector3f rot_var;
	sym::QuatVarToRotVar(getStateAtFusionHorizonAsVector(), P, FLT_EPSILON, &rot_var);
	return rot_var;
}

float Ekf::getYawVar() const
{
	VectorState H_YAW;
	float yaw_var = 0.f;
	computeYawInnovVarAndH(0.f, yaw_var, H_YAW);

	return yaw_var;
}

#if defined(CONFIG_EKF2_BAROMETER)
void Ekf::updateGroundEffect()
{
	if (_control_status.flags.in_air && !_control_status.flags.fixed_wing) {
#if defined(CONFIG_EKF2_TERRAIN)
		if (isTerrainEstimateValid()) {
			// automatically set ground effect if terrain is valid
			float height = _terrain_vpos - _state.pos(2);
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

void Ekf::resetQuatStateYaw(float yaw, float yaw_variance)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	const Quatf quat_before_reset = _state.quat_nominal;

	// save a copy of covariance in NED frame to restore it after the quat reset
	const matrix::SquareMatrix3f rot_cov = diag(calcRotVecVariances());
	Vector3f rot_var_ned_before_reset = matrix::SquareMatrix3f(_R_to_earth * rot_cov * _R_to_earth.T()).diag();

	// update the yaw angle variance
	if (PX4_ISFINITE(yaw_variance) && (yaw_variance > FLT_EPSILON)) {
		rot_var_ned_before_reset(2) = yaw_variance;
	}

	// update transformation matrix from body to world frame using the current estimate
	// update the rotation matrix using the new yaw value
	_R_to_earth = updateYawInRotMat(yaw, Dcmf(_state.quat_nominal));

	// calculate the amount that the quaternion has changed by
	const Quatf quat_after_reset(_R_to_earth);
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// update quaternion states
	_state.quat_nominal = quat_after_reset;
	uncorrelateQuatFromOtherStates();

	// restore covariance
	resetQuatCov(rot_var_ned_before_reset);

	// add the reset amount to the output observer buffered data
	_output_predictor.resetQuaternion(q_error);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// update EV attitude error filter
	if (_ev_q_error_initialized) {
		const Quatf ev_q_error_updated = (q_error * _ev_q_error_filt.getState()).normalized();
		_ev_q_error_filt.reset(ev_q_error_updated);
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// record the state change
	if (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) {
		_state_reset_status.quat_change = q_error;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.quat_change = q_error * _state_reset_status.quat_change;
		_state_reset_status.quat_change.normalize();
	}

	_state_reset_status.reset_count.quat++;

	_time_last_heading_fuse = _time_delayed_us;
}

bool Ekf::resetYawToEKFGSF()
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	// don't allow reset if there's just been a yaw reset
	const bool yaw_alignment_changed = (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);
	const bool quat_reset = (_state_reset_status.reset_count.quat != _state_reset_count_prev.quat);

	if (yaw_alignment_changed || quat_reset) {
		return false;
	}

	ECL_INFO("yaw estimator reset heading %.3f -> %.3f rad",
		 (double)getEulerYaw(_R_to_earth), (double)_yawEstimator.getYaw());

	resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

	_control_status.flags.yaw_align = true;
	_information_events.flags.yaw_aligned_to_imu_gps = true;

	return true;
}

bool Ekf::isYawEmergencyEstimateAvailable() const
{
	// don't allow reet using the EKF-GSF estimate until the filter has started fusing velocity
	// data and the yaw estimate has converged
	if (!_yawEstimator.isActive()) {
		return false;
	}

	return _yawEstimator.getYawVar() < sq(_params.EKFGSF_yaw_err_max);
}

bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight);
}

void Ekf::resetGpsDriftCheckFilters()
{
	_gps_velNE_filt.setZero();
	_gps_pos_deriv_filt.setZero();

	_gps_horizontal_position_drift_rate_m_s = NAN;
	_gps_vertical_position_drift_rate_m_s = NAN;
	_gps_filtered_horizontal_velocity_m_s = NAN;
}

void Ekf::resetWind()
{
#if defined(CONFIG_EKF2_AIRSPEED)
	if (_control_status.flags.fuse_aspd && isRecent(_airspeed_sample_delayed.time_us, 1e6)) {
		resetWindUsingAirspeed(_airspeed_sample_delayed);
		return;
	}
#endif // CONFIG_EKF2_AIRSPEED

	resetWindToZero();
}

void Ekf::resetWindToZero()
{
	ECL_INFO("reset wind to zero");

	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();

	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, _params.initial_wind_uncertainty);
}

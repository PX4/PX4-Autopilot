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
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/predict_covariance.h"
#include "utils.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

// Sets initial values for the covariance matrix
// Do not call before quaternion states have been initialised
void Ekf::initialiseCovariance()
{
	P.zero();

	_delta_angle_bias_var_accum.setZero();
	_delta_vel_bias_var_accum.setZero();

	const float dt = _dt_ekf_avg;

	resetQuatCov();

	// velocity
	P(4,4) = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P(5,5) = P(4,4);
	P(6,6) = sq(1.5f) * P(4,4);

	// position
	P(7,7) = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	P(8,8) = P(7,7);

	if (_control_status.flags.rng_hgt) {
		P(9,9) = sq(fmaxf(_params.range_noise, 0.01f));

	} else if (_control_status.flags.gps_hgt) {
		P(9,9) = sq(fmaxf(1.5f * _params.gps_pos_noise, 0.01f));

	} else {
		P(9,9) = sq(fmaxf(_params.baro_noise, 0.01f));
	}

	// gyro bias
	P.uncorrelateCovarianceSetVariance<3>(10, sq(_params.switch_on_gyro_bias * dt));

	// accel bias
	P.uncorrelateCovarianceSetVariance<3>(13, sq(_params.switch_on_accel_bias * dt));

	// mag I
	P.uncorrelateCovarianceSetVariance<3>(16, sq(_params.mag_noise));

	// mag B
	P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

	// wind
	P.uncorrelateCovarianceSetVariance<2>(22, sq(_params.initial_wind_uncertainty));
}

void Ekf::predictCovariance()
{
	// Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	const float dt = _dt_ekf_avg;
	const float dt_inv = 1.f / dt;

	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	const float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	const float beta = 1.0f - alpha;
	_ang_rate_magnitude_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_ang.norm(), beta * _ang_rate_magnitude_filt);
	_accel_magnitude_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_vel.norm(), beta * _accel_magnitude_filt);
	_accel_vec_filt = alpha * dt_inv * _imu_sample_delayed.delta_vel + beta * _accel_vec_filt;

	const bool is_manoeuvre_level_high = _ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim
					     || _accel_magnitude_filt > _params.acc_bias_learn_acc_lim;

	const bool do_inhibit_all_axes = (_params.fusion_mode & SensorFusionMask::INHIBIT_ACC_BIAS)
					 || is_manoeuvre_level_high
					 || _fault_status.flags.bad_acc_vertical;

	for (unsigned stateIndex = 13; stateIndex <= 15; stateIndex++) {
		const unsigned index = stateIndex - 13;

		bool is_bias_observable = true;

		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos) {
			// when using fake position (but not fake height) only consider an accel bias observable if aligned with the gravity vector
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f); // cos 15 degrees ~= 0.966
		}

		const bool do_inhibit_axis = do_inhibit_all_axes || _imu_sample_delayed.delta_vel_clipping[index] || !is_bias_observable;

		const bool was_inhibited = _state_inhibited[stateIndex];
		_state_inhibited.set(stateIndex, do_inhibit_axis);

		if (!was_inhibited && _state_inhibited[stateIndex]) {
			ECL_DEBUG("accel %d now inhibited", index);
		} else if (was_inhibited && !_state_inhibited[stateIndex]) {
			ECL_DEBUG("accel %d no longer inhibited", index);
		}
	}

	// compute noise variance for stationary processes
	Vector24f process_noise{};

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	// delta angle bias states
	//  convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	const float d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.f, 1.f);
	process_noise.slice<3, 1>(10, 0) = sq(d_ang_bias_sig);

	// delta_velocity bias states
	//  convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	const float d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.f, 1.f);
	process_noise.slice<3, 1>(13, 0) = sq(d_vel_bias_sig);

	// mag states
	if (_control_status.flags.mag_3D) {
		// earth frame magnetic field states (16,17,18)
		//  Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
		if ((P(16, 16) + P(17, 17) + P(18, 18)) < 0.1f) {
			float mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.f, 1.f);
			process_noise.slice<3, 1>(16, 0) = sq(mag_I_sig);
		}

		// body frame magnetic field states (19,20,21)
		//  Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
		if ((P(19, 19) + P(20, 20) + P(21, 21)) < 0.1f) {
			float mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.f, 1.f);
			process_noise.slice<3, 1>(19, 0) = sq(mag_B_sig);
		}

		for (int i = 16; i <= 21; i++) {
			_state_inhibited.set(i, false);
		}

	} else {
		for (int i = 16; i <= 21; i++) {
			_state_inhibited.set(i, true);
		}
	}

	// wind velocity states (22, 23)
	//  Calculate low pass filtered height rate
	const float alpha_height_rate_lpf = 0.1f * dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;

	if (_control_status.flags.wind) {
		// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
		if ((P(22,22) + P(23,23)) < sq(_params.initial_wind_uncertainty)) {
			float wind_vel_nsd_scaled = math::constrain(_params.wind_vel_nsd, 0.f, 1.f) * (1.f + _params.wind_vel_nsd_scaler * fabsf(_height_rate_lpf));
			process_noise.slice<2, 1>(22, 0) = sq(wind_vel_nsd_scaled) * dt;
		}

		for (int i = 22; i <= 23; i++) {
			_state_inhibited.set(i, false);
		}
	} else {
		for (int i = 22; i <= 23; i++) {
			_state_inhibited.set(i, true);
		}
	}

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	const float d_ang_var = sq(dt * gyro_noise);

	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);

	Vector3f d_vel_var;

	for (int i = 0; i <= 2; i++) {
		if (_fault_status.flags.bad_acc_vertical || _imu_sample_delayed.delta_vel_clipping[i]) {
			// Increase accelerometer process noise if bad accel data is detected
			d_vel_var(i) = sq(dt * BADACC_BIAS_PNOISE);

		} else {
			d_vel_var(i) = sq(dt * accel_noise);
		}
	}

	// predict the covariance
	SquareMatrix24f nextP;

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	sym::PredictCovariance(getStateAtFusionHorizonAsVector(), P, _imu_sample_delayed.delta_vel, d_vel_var, _imu_sample_delayed.delta_ang, d_ang_var, dt, &nextP);


	// delta angle process noise contribution can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	for (unsigned i = 10; i <= 12; i++) {
		const int index = i - 10;

		if (!_state_inhibited[i]) {
			nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_angle_bias_var_accum(index));

		} else {
			_delta_angle_bias_var_accum(index) = 0.f;
		}
	}

	// delta velocity process noise contribution can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	for (int i = 13; i <= 15; i++) {
		const int index = i - 13;

		if (!_state_inhibited[i]) {
			nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_vel_bias_var_accum(index));

		} else {
			_delta_vel_bias_var_accum(index) = 0.f;
		}
	}

	// add process noise that is not from the IMU
	for (unsigned i = 16; i <= 23; i++) {
		if (!_state_inhibited[i]) {
			nextP(i, i) += process_noise(i);
		}
	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P(7, 7) + P(8, 8)) > 1e4f) {
		for (uint8_t i = 7; i <= 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 0; row < _k_num_states; row++) {
		if (!_state_inhibited[row]) {
			for (unsigned column = 0; column < row; column++) {
				P(row, column) = P(column, row) = nextP(column, row);
			}

			P(row, row) = nextP(row, row);
		}
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors(false);
}

void Ekf::fixCovarianceErrors(bool force_symmetry)
{
	// the following states are optional and are deactivated when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

	// accelerometer bias states
	if (!accelBiasInhibited()) {
		// Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
		const float minSafeStateVar = 1e-9f;
		float maxStateVar = minSafeStateVar;
		bool resetRequired = false;

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_state_inhibited[stateIndex]) {
				// Skip the check for the inhibited axis
				continue;
			}

			if (P(stateIndex, stateIndex) > maxStateVar) {
				maxStateVar = P(stateIndex, stateIndex);

			} else if (P(stateIndex, stateIndex) < minSafeStateVar) {
				resetRequired = true;
			}
		}

		// To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
		// not exceed 100 and the minimum variance must not fall below the target minimum
		// Also limit variance to a maximum equivalent to a 0.1g uncertainty
		const float minStateVarTarget = 5E-8f;
		float minAllowedStateVar = fmaxf(0.01f * maxStateVar, minStateVarTarget);

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_state_inhibited[stateIndex]) {
				// Skip the check for the inhibited axis
				continue;
			}

			P(stateIndex, stateIndex) = math::constrain(P(stateIndex, stateIndex), minAllowedStateVar,
						    sq(0.1f * CONSTANTS_ONE_G * _dt_ekf_avg));
		}

		// If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
		if (resetRequired) {
			ECL_ERR("accel bias reset required (fixCovarianceErrors)");
			P.uncorrelateCovariance<3>(13);
		}

		// Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
		// calculate accel bias term aligned with the gravity vector
		const float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		const float down_dvel_bias = _state.delta_vel_bias.dot(Vector3f(_R_to_earth.row(2)));

		// check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = false;
		if (fabsf(down_dvel_bias) > dVel_bias_lim) {

			bool bad_vz_gps = _control_status.flags.gps    && (down_dvel_bias * _aid_src_gnss_vel.innovation[2] < 0.0f);
			bool bad_vz_ev  = _control_status.flags.ev_vel && (down_dvel_bias * _aid_src_ev_vel.innovation[2] < 0.0f);

			if (bad_vz_gps || bad_vz_ev) {
				bool bad_z_baro = _control_status.flags.baro_hgt && (down_dvel_bias * _aid_src_baro_hgt.innovation < 0.0f);
				bool bad_z_gps  = _control_status.flags.gps_hgt  && (down_dvel_bias * _aid_src_gnss_hgt.innovation < 0.0f);
				bool bad_z_rng  = _control_status.flags.rng_hgt  && (down_dvel_bias * _aid_src_rng_hgt.innovation  < 0.0f);
				bool bad_z_ev   = _control_status.flags.ev_hgt   && (down_dvel_bias * _aid_src_ev_hgt.innovation   < 0.0f);

				if (bad_z_baro || bad_z_gps || bad_z_rng || bad_z_ev) {
					bad_acc_bias = true;
				}
			}
		}

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _imu_sample_delayed.time_us;

		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (isTimedOut(_time_acc_bias_check, (uint64_t)7e6)) {

			P.uncorrelateCovariance<3>(13);

			_time_acc_bias_check = _imu_sample_delayed.time_us;
			_fault_status.flags.bad_acc_bias = false;
			_warning_events.flags.invalid_accel_bias_cov_reset = true;
			ECL_WARN("invalid accel bias - covariance reset");
		}

	}

	// uncorrelate all inhibited states
	for (unsigned i = 0; i < _k_num_states; i++) {
		if (_state_inhibited[i]) {
			P.uncorrelateCovariance<1>(i);
		}
	}

	// constrain variances
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)

	// quaternion states
	for (int i = 0; i <= 3; i++) {
		P(i, i) = math::constrain(P(i, i), 0.f, 1.f);
	}

	// NED velocity states
	for (int i = 4; i <= 6; i++) {
		P(i, i) = math::constrain(P(i, i), 1e-6f, 1e6f);
	}

	// NED position states
	for (int i = 7; i <= 9; i++) {
		P(i, i) = math::constrain(P(i, i), 1e-6f, 1e6f);
	}

	// gyro bias states
	for (int i = 10; i <= 12; i++) {
		P(i, i) = math::constrain(P(i, i), 0.f, 1.f);
	}

	// accel bias states
	for (int i = 13; i <= 15; i++) {
		P(i, i) = math::constrain(P(i, i), 0.f, 1.f);
	}

	// earth mag field
	for (int i = 16; i <= 18; i++) {
		P(i, i) = math::constrain(P(i, i), 0.f, 1.f);
	}

	// body mag field
	for (int i = 19; i <= 21; i++) {
		P(i, i) = math::constrain(P(i, i), 0.f, 1.f);
	}

	// wind velocity
	for (int i = 22; i <= 23; i++) {
		P(i, i) = math::constrain(P(i, i), 0.f, 1e6f);
	}


	// force symmetry on all state covariances
	if (force_symmetry) {
		P.makeRowColSymmetric<_k_num_states>(0);
	}
}

// if the covariance correction will result in a negative variance, then
// the covariance matrix is unhealthy and must be corrected
bool Ekf::checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP)
{
	bool healthy = true;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i, i) < KHP(i, i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);
			healthy = false;
		}
	}

	return healthy;
}

void Ekf::resetMagRelatedCovariances()
{
	resetQuatCov();
	resetMagCov();
}

void Ekf::resetQuatCov()
{
	zeroQuatCov();

	// define the initial angle uncertainty as variances for a rotation vector
	Vector3f rot_vec_var;
	rot_vec_var.setAll(sq(_params.initial_tilt_err));

	initialiseQuatCovariances(rot_vec_var);
}

void Ekf::zeroQuatCov()
{
	P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
	P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);
}

void Ekf::resetMagCov()
{
	// reset the corresponding rows and columns in the covariance matrix and
	// set the variances on the magnetic field states to the measurement variance
	_mag_decl_cov_reset = false;

	P.uncorrelateCovarianceSetVariance<3>(16, sq(_params.mag_noise));
	P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));
}

void Ekf::resetZDeltaAngBiasCov()
{
	const float init_delta_ang_bias_var = sq(_params.switch_on_gyro_bias * _dt_ekf_avg);

	P.uncorrelateCovarianceSetVariance<1>(12, init_delta_ang_bias_var);
}

void Ekf::resetWindCovarianceUsingAirspeed()
{
	// Derived using EKF/matlab/scripts/Inertial Nav EKF/wind_cov.py
	// TODO: explicitly include the sideslip angle in the derivation
	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) * math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));
	constexpr float initial_sideslip_uncertainty = math::radians(15.0f);
	const float initial_wind_var_body_y = sq(_airspeed_sample_delayed.true_airspeed * sinf(initial_sideslip_uncertainty));
	constexpr float R_yaw = sq(math::radians(10.0f));

	const float cos_yaw = cosf(euler_yaw);
	const float sin_yaw = sinf(euler_yaw);

	// rotate wind velocity into earth frame aligned with vehicle yaw
	const float Wx = _state.wind_vel(0) * cos_yaw + _state.wind_vel(1) * sin_yaw;
	const float Wy = -_state.wind_vel(0) * sin_yaw + _state.wind_vel(1) * cos_yaw;

	// it is safer to remove all existing correlations to other states at this time
	P.uncorrelateCovarianceSetVariance<2>(22, 0.0f);

	P(22, 22) = R_TAS * sq(cos_yaw) + R_yaw * sq(-Wx * sin_yaw - Wy * cos_yaw) + initial_wind_var_body_y * sq(sin_yaw);
	P(22, 23) = R_TAS * sin_yaw * cos_yaw + R_yaw * (-Wx * sin_yaw - Wy * cos_yaw) * (Wx * cos_yaw - Wy * sin_yaw) -
		    initial_wind_var_body_y * sin_yaw * cos_yaw;
	P(23, 22) = P(22, 23);
	P(23, 23) = R_TAS * sq(sin_yaw) + R_yaw * sq(Wx * cos_yaw - Wy * sin_yaw) + initial_wind_var_body_y * sq(cos_yaw);

	// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
	P(22, 22) += P(4, 4);
	P(23, 23) += P(5, 5);
}

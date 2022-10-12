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
 * @file heading_fusion.cpp
 * Magnetometer fusion methods.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_mag_innov_innov_var_and_hx.h"
#include "python/ekf_derivation/generated/compute_mag_y_innov_var_and_h.h"
#include "python/ekf_derivation/generated/compute_mag_z_innov_var_and_h.h"

#include <mathlib/mathlib.h>

bool Ekf::fuseMag(const Vector3f &mag, estimator_aid_source_3d_s &aid_src_mag, bool update_all_states)
{
	// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	const float R_MAG = sq(fmaxf(_params.mag_noise, 0.0f));

	// calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gains
	const char* numerical_error_covariance_reset_string = "numerical error - covariance reset";
	Vector3f mag_innov;
	Vector3f innov_var;

	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3,16,17,18,19,20,21> Hfusion;
	Vector24f H;
	const Vector24f state_vector = getStateAtFusionHorizonAsVector();
	sym::ComputeMagInnovInnovVarAndHx(state_vector, P, mag, R_MAG, FLT_EPSILON, &mag_innov, &innov_var, &H);
	Hfusion = H;

	innov_var.copyTo(aid_src_mag.innovation_variance);

	if (aid_src_mag.innovation_variance[0] < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_x = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magX %s", numerical_error_covariance_reset_string);
		return false;
	}

	_fault_status.flags.bad_mag_x = false;

	// check innovation variances for being badly conditioned
	if (aid_src_mag.innovation_variance[1] < R_MAG) {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_y = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magY %s", numerical_error_covariance_reset_string);
		return false;
	}

	_fault_status.flags.bad_mag_y = false;

	if (aid_src_mag.innovation_variance[2] < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_z = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
		return false;
	}

	_fault_status.flags.bad_mag_z = false;

	// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (_control_status.flags.synthetic_mag_z) {
		mag_innov(2) = 0.0f;
	}

	for (int i = 0; i < 3; i++) {
		aid_src_mag.observation[i] = mag(i) - _state.mag_B(i);
		aid_src_mag.observation_variance[i] = R_MAG;
		aid_src_mag.innovation[i] = mag_innov(i);
	}

	aid_src_mag.fusion_enabled = _control_status.flags.mag_3D && update_all_states;

	// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (_control_status.flags.synthetic_mag_z) {
		aid_src_mag.innovation[2] = 0.0f;
	}

	const float innov_gate = math::max(_params.mag_innov_gate, 1.f);
	setEstimatorAidStatusTestRatio(aid_src_mag, innov_gate);

	// Perform an innovation consistency check and report the result
	_innov_check_fail_status.flags.reject_mag_x = (aid_src_mag.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_mag_y = (aid_src_mag.test_ratio[1] > 1.f);
	_innov_check_fail_status.flags.reject_mag_z = (aid_src_mag.test_ratio[2] > 1.f);

	// if any axis fails, abort the mag fusion
	if (aid_src_mag.innovation_rejected) {
		return false;
	}

	bool fused[3] {false, false, false};

	// update the states and covariance using sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagYInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src_mag.innovation_variance[index], &H);
			Hfusion = H;

			// recalculate innovation using the updated state
			aid_src_mag.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);

			if (aid_src_mag.innovation_variance[index] < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_y = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magY %s", numerical_error_covariance_reset_string);
				return false;
			}

		} else if (index == 2) {
			// we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (_control_status.flags.synthetic_mag_z) {
				continue;
			}

			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagZInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src_mag.innovation_variance[index], &H);
			Hfusion = H;

			// recalculate innovation using the updated state
			aid_src_mag.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);

			if (aid_src_mag.innovation_variance[index] < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_z = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
				return false;
			}
		}

		Vector24f Kfusion = P * Hfusion / aid_src_mag.innovation_variance[index];

		if (!update_all_states) {
			for (unsigned row = 0; row <= 15; row++) {
				Kfusion(row) = 0.f;
			}

			for (unsigned row = 22; row <= 23; row++) {
				Kfusion(row) = 0.f;
			}
		}

		if (measurementUpdate(Kfusion, Hfusion, aid_src_mag.innovation[index])) {
			fused[index] = true;
			limitDeclination();

		} else {
			fused[index] = false;
		}
	}

	_fault_status.flags.bad_mag_x = !fused[0];
	_fault_status.flags.bad_mag_y = !fused[1];
	_fault_status.flags.bad_mag_z = !fused[2];

	if (fused[0] && fused[1] && fused[2]) {
		aid_src_mag.fused = true;
		aid_src_mag.time_last_fuse = _imu_sample_delayed.time_us;
		return true;
	}

	aid_src_mag.fused = false;
	return false;
}

// update quaternion states and covariances using the yaw innovation and yaw observation variance
bool Ekf::fuseYaw(const float innovation, const float variance, estimator_aid_source_1d_s& aid_src_status)
{
	aid_src_status.innovation = innovation;

	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	bool canUseA = false;
	bool canUseB = false;

	float SA0;
	float SA1;
	float SA2;
	float SA3;
	float SA4;
	float SA5_inv;

	float SB0;
	float SB1;
	float SB2;
	float SB3;
	float SB4;
	float SB5_inv;

	const bool fuse_yaw_321 = shouldUse321RotationSequence(_R_to_earth);

	if (fuse_yaw_321) {
		// calculate 321 yaw observation matrix
		SA0 = 2*q3;
		SA1 = 2*q2;
		SA2 = SA0*q0 + SA1*q1;
		SA3 = sq(q0) + sq(q1) - sq(q2) - sq(q3);

		SB0 = 2*q0;
		SB1 = 2*q1;
		SB2 = SB0*q3 + SB1*q2;
		SB4 = sq(q0) + sq(q1) - sq(q2) - sq(q3);

	} else {
		// calculate 312 yaw observation matrix
		SA0 = 2*q3;
		SA1 = 2*q2;
		SA2 = SA0*q0 - SA1*q1;
		SA3 = sq(q0) - sq(q1) + sq(q2) - sq(q3);

		SB0 = 2*q0;
		SB1 = 2*q1;
		SB2 = -SB0*q3 + SB1*q2;
		SB4 = -sq(q0) + sq(q1) - sq(q2) + sq(q3);
	}

	if (sq(SA3) > 1e-6f) {
		SA4 = 1.f/sq(SA3);
		SA5_inv = sq(SA2)*SA4 + 1.f;
		canUseA = fabsf(SA5_inv) > 1e-6f;
	}

	if (sq(SB2) > 1e-6f) {
		SB3 = 1.f/sq(SB2);
		SB5_inv = SB3*sq(SB4) + 1.f;
		canUseB = fabsf(SB5_inv) > 1e-6f;
	}

	Vector4f H_YAW;

	if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
		const float SA5 = 1.f/SA5_inv;
		const float SA6 = 1.f/SA3;
		const float SA7 = SA2*SA4;
		const float SA8 = 2*SA7;
		const float SA9 = 2*SA6;

		if (fuse_yaw_321) {
			// calculate 321 yaw observation matrix
			H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
			H_YAW(1) = SA5*(SA1*SA6 - SA8*q1);
			H_YAW(2) = SA5*(SA1*SA7 + SA9*q1);
			H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);

		} else {
			// calculate 312 yaw observation matrix
			H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
			H_YAW(1) = SA5*(-SA1*SA6 + SA8*q1);
			H_YAW(2) = SA5*(-SA1*SA7 - SA9*q1);
			H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);
		}

	} else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
		const float SB5 = 1.f/SB5_inv;
		const float SB6 = 1.f/SB2;
		const float SB7 = SB3*SB4;
		const float SB8 = 2*SB7;
		const float SB9 = 2*SB6;

		if (fuse_yaw_321) {
			// calculate 321 yaw observation matrix
			H_YAW(0) = -SB5*(SB0*SB6 - SB8*q3);
			H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
			H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
			H_YAW(3) = -SB5*(-SB0*SB7 - SB9*q3);

		} else {
			// calculate 312 yaw observation matrix
			H_YAW(0) = -SB5*(-SB0*SB6 + SB8*q3);
			H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
			H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
			H_YAW(3) = -SB5*(SB0*SB7 + SB9*q3);
		}

	} else {
		return false;
	}

	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
	// calculate the innovation variance
	aid_src_status.innovation_variance = variance;

	for (unsigned row = 0; row <= 3; row++) {
		float tmp = 0.0f;

		for (uint8_t col = 0; col <= 3; col++) {
			tmp += P(row, col) * H_YAW(col);
		}

		aid_src_status.innovation_variance += H_YAW(row) * tmp;
	}

	float heading_innov_var_inv = 0.f;

	// check if the innovation variance calculation is badly conditioned
	if (aid_src_status.innovation_variance >= variance) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;
		heading_innov_var_inv = 1.f / aid_src_status.innovation_variance;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("yaw fusion numerical error - covariance reset");
		return false;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;

	for (uint8_t row = 0; row <= 15; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * H_YAW(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	if (_control_status.flags.wind) {
		for (uint8_t row = 22; row <= 23; row++) {
			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion(row) += P(row, col) * H_YAW(col);
			}

			Kfusion(row) *= heading_innov_var_inv;
		}
	}

	// define the innovation gate size
	float gate_sigma = math::max(_params.heading_innov_gate, 1.f);

	// innovation test ratio
	setEstimatorAidStatusTestRatio(aid_src_status, gate_sigma);

	// set the magnetometer unhealthy if the test fails
	if (aid_src_status.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (!_control_status.flags.in_air
		&& isTimedOut(_time_last_in_air, (uint64_t)5e6)
		&& isTimedOut(aid_src_status.time_last_fuse, (uint64_t)1e6)
		) {
			// constrain the innovation to the maximum set by the gate
			// we need to delay this forced fusion to avoid starting it
			// immediately after touchdown, when the drone is still armed
			float gate_limit = sqrtf((sq(gate_sigma) * aid_src_status.innovation_variance));
			aid_src_status.innovation = math::constrain(aid_src_status.innovation, -gate_limit, gate_limit);

			// also reset the yaw gyro variance to converge faster and avoid
			// being stuck on a previous bad estimate
			resetZDeltaAngBiasCov();

		} else {
			return false;
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
	}

	// copy observation jacobian
	SparseVector24f<0,1,2,3> Hfusion;
	Hfusion.at<0>() = H_YAW(0);
	Hfusion.at<1>() = H_YAW(1);
	Hfusion.at<2>() = H_YAW(2);
	Hfusion.at<3>() = H_YAW(3);

	if (measurementUpdate(Kfusion, Hfusion, aid_src_status.innovation)) {

		_time_last_heading_fuse = _imu_sample_delayed.time_us;

		aid_src_status.time_last_fuse = _imu_sample_delayed.time_us;
		aid_src_status.fused = true;

		_fault_status.flags.bad_hdg = false;

		return true;
	}

	// otherwise
	aid_src_status.fused = false;
	_fault_status.flags.bad_hdg = true;
	return false;
}

bool Ekf::fuseDeclination(float decl_sigma)
{
	// assign intermediate state variables
	const float magN = _state.mag_I(0);
	const float magE = _state.mag_I(1);

	// minimum North field strength before calculation becomes badly conditioned (T)
	constexpr float N_field_min = 0.001f;

	// observation variance (rad**2)
	const float R_DECL = sq(decl_sigma);

	// Calculate intermediate variables
	if (fabsf(magN) < sq(N_field_min)) {
		// calculation is badly conditioned close to +-90 deg declination
		return false;
	}

	const float HK0 = ecl::powf(magN, -2);
	const float HK1 = HK0*ecl::powf(magE, 2) + 1.0F;
	const float HK2 = 1.0F/HK1;
	const float HK3 = 1.0F/magN;
	const float HK4 = HK2*HK3;
	const float HK5 = HK3*magE;
	const float HK6 = HK5*P(16,17) - P(17,17);
	const float HK7 = ecl::powf(HK1, -2);
	const float HK8 = HK5*P(16,16) - P(16,17);
	const float innovation_variance = -HK0*HK6*HK7 + HK7*HK8*magE/ecl::powf(magN, 3) + R_DECL;
	float HK9;

	if (innovation_variance > R_DECL) {
		HK9 = HK4/innovation_variance;
	} else {
		// variance calculation is badly conditioned
		return false;
	}

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
	// Note Hfusion indices do not match state indices
	SparseVector24f<16,17> Hfusion;
	Hfusion.at<16>() = -HK0*HK2*magE;
	Hfusion.at<17>() = HK4;

	// Calculate the Kalman gains
	Vector24f Kfusion;

	for (unsigned row = 0; row <= 15; row++) {
		Kfusion(row) = -HK9*(HK5*P(row,16) - P(row,17));
	}

	Kfusion(16) = -HK8*HK9;
	Kfusion(17) = -HK6*HK9;

	for (unsigned row = 18; row <= 23; row++) {
		Kfusion(row) = -HK9*(HK5*P(16,row) - P(17,row));
	}

	const float innovation = math::constrain(atan2f(magE, magN) - getMagDeclination(), -0.5f, 0.5f);

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, innovation);

	_fault_status.flags.bad_mag_decl = !is_fused;

	if (is_fused) {
		limitDeclination();
	}

	return is_fused;
}

void Ekf::limitDeclination()
{
	// get a reference value for the earth field declinaton and minimum plausible horizontal field strength
	// set to 50% of the horizontal strength from geo tables if location is known
	float decl_reference;
	float h_field_min = 0.001f;

	if (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps)) {
			decl_reference = _mag_declination_gps;
			h_field_min = fmaxf(h_field_min, 0.5f * _mag_strength_gps * cosf(_mag_inclination_gps));

		} else {
			decl_reference = math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		decl_reference = math::radians(_params.mag_declination_deg);
	}

	// do not allow the horizontal field length to collapse - this will make the declination fusion badly conditioned
	// and can result in a reversal of the NE field states which the filter cannot recover from
	// apply a circular limit
	float h_field = sqrtf(_state.mag_I(0) * _state.mag_I(0) + _state.mag_I(1) * _state.mag_I(1));

	if (h_field < h_field_min) {
		if (h_field > 0.001f * h_field_min) {
			const float h_scaler = h_field_min / h_field;
			_state.mag_I(0) *= h_scaler;
			_state.mag_I(1) *= h_scaler;

		} else {
			// too small to scale radially so set to expected value
			const float mag_declination = getMagDeclination();
			_state.mag_I(0) = 2.0f * h_field_min * cosf(mag_declination);
			_state.mag_I(1) = 2.0f * h_field_min * sinf(mag_declination);
		}

		h_field = h_field_min;
	}

	// do not allow the declination estimate to vary too much relative to the reference value
	constexpr float decl_tolerance = 0.5f;
	const float decl_max = decl_reference + decl_tolerance;
	const float decl_min = decl_reference - decl_tolerance;
	const float decl_estimate = atan2f(_state.mag_I(1), _state.mag_I(0));

	if (decl_estimate > decl_max)  {
		_state.mag_I(0) = h_field * cosf(decl_max);
		_state.mag_I(1) = h_field * sinf(decl_max);

	} else if (decl_estimate < decl_min)  {
		_state.mag_I(0) = h_field * cosf(decl_min);
		_state.mag_I(1) = h_field * sinf(decl_min);
	}
}

float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted)
{
	// theoretical magnitude of the magnetometer Z component value given X and Y sensor measurement and our knowledge
	// of the earth magnetic field vector at the current location
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetomer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs;
}

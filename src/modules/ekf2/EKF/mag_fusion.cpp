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

#include <mathlib/mathlib.h>

void Ekf::fuseMag(const Vector3f &mag)
{
	using D = matrix::Dual<float, 10>;
	using matrix::Vector3;

	// rotate magnetometer earth field state into body frame
	const matrix::Dcm<D> R_to_earth(matrix::Quaternion<D> (D(_state.quat_nominal(0), 0),
			D(_state.quat_nominal(1), 1),
			D(_state.quat_nominal(2), 2),
			D(_state.quat_nominal(3), 3)));

	const Vector3<D> mag_I_body = R_to_earth.transpose() * Vector3<D>(D(_state.mag_I(0), 4), D(_state.mag_I(1), 5), D(_state.mag_I(2), 6));

	// compute magnetometer innovations
	const Vector3<D> mag_pred = mag_I_body + Vector3<D>(D(_state.mag_B(0), 7), D(_state.mag_B(1), 8), D(_state.mag_B(2), 9));

	_mag_innov(0) = mag_pred(0).value - mag(0);
	_mag_innov(1) = mag_pred(1).value - mag(1);

	if (!_control_status.flags.synthetic_mag_z) {
		_mag_innov(2) = mag_pred(2).value - mag(2);

	} else {
		// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
		_mag_innov(2) = 0.0f;
	}

	// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	const float R_MAG = sq(fmaxf(_params.mag_noise, 0.0f));

	const char* numerical_error_covariance_reset_string = "numerical error - covariance reset";

	// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
	// before they are used to constrain heading drift
	const bool update_all_states = ((_imu_sample_delayed.time_us - _flt_mag_align_start_time) > (uint64_t)5e6);

	// Update the states and covariance using sequential fusion of the magnetometer components
	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3,16,17,18,19,20,21> Hfusion[3];
	bool all_innovation_checks_passed = true;

	for (uint8_t index = 0; index <= 2; index++) {
		// Observation Jacobians
		Hfusion[index].at<0>() = mag_pred(index).derivative(0);
		Hfusion[index].at<1>() = mag_pred(index).derivative(1);
		Hfusion[index].at<2>() = mag_pred(index).derivative(2);
		Hfusion[index].at<3>() = mag_pred(index).derivative(3);
		Hfusion[index].at<16>() = mag_pred(index).derivative(4);
		Hfusion[index].at<17>() = mag_pred(index).derivative(5);
		Hfusion[index].at<18>() = mag_pred(index).derivative(6);
		Hfusion[index].at<19>() = mag_pred(index).derivative(7);
		Hfusion[index].at<20>() = mag_pred(index).derivative(8);
		Hfusion[index].at<21>() = mag_pred(index).derivative(9);

		_mag_innov_var(index) = Hfusion[index].dot(P * Hfusion[index]) + R_MAG;

		if (_mag_innov_var(index) < R_MAG) {
			// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
			switch (index) {
			case 0:
				_fault_status.flags.bad_mag_x = true;
				ECL_ERR("magX %s", numerical_error_covariance_reset_string);
				break;
			case 1:
				_fault_status.flags.bad_mag_y = true;
				ECL_ERR("magY %s", numerical_error_covariance_reset_string);
				break;
			case 2:
				_fault_status.flags.bad_mag_z = true;
				ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
				break;
			}

			// we need to re-initialise covariances and abort this fusion step
			resetMagRelatedCovariances();
			return;
		}

		// Perform an innovation consistency check and report the result
		_mag_test_ratio(index) = sq(_mag_innov(index)) / (sq(math::max(_params.mag_innov_gate, 1.0f)) * _mag_innov_var(index));

		if (_mag_test_ratio(index) > 1.0f) {
			all_innovation_checks_passed = false;
			_innov_check_fail_status.value |= (1 << (index + 3));

		} else {
			_innov_check_fail_status.value &= ~(1 << (index + 3));
		}
	}

	// if any axis fails, abort the mag fusion
	if (!all_innovation_checks_passed) {
		return;
	}

	for (uint8_t index = 0; index <= 2; index++) {
		// recalculate innovation variance becasue covariances have changed due to previous fusion
		const auto PHt = P * Hfusion[index];
		_mag_innov_var(index) = Hfusion[index].dot(PHt) + R_MAG;

		Vector24f Kfusion = P * Hfusion[index] / _mag_innov_var(index);

		if (!update_all_states) {
			for (unsigned i = 0; i <= 15; i++) {
				Kfusion(i) = 0.f;
			}

			for (unsigned i = 22; i <= 23; i++) {
				Kfusion(i) = 0.f;
			}
		}

		const bool is_fused = measurementUpdate(Kfusion, Hfusion[index], _mag_innov(index));

		switch (index) {
		case 0:
			_fault_status.flags.bad_mag_x = !is_fused;
			break;

		case 1:
			_fault_status.flags.bad_mag_y = !is_fused;
			break;

		case 2:
			_fault_status.flags.bad_mag_z = !is_fused;
			break;
		}

		if (is_fused) {
			limitDeclination();
		}
	}
}

bool Ekf::fuseYaw321(float yaw, float yaw_variance, bool zero_innovation)
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	const float R_YAW = fmaxf(yaw_variance, 1.0e-4f);
	const float measurement = wrap_pi(yaw);

	// calculate 321 yaw observation matrix
	// choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	bool canUseA = false;
	const float SA0 = 2*q3;
	const float SA1 = 2*q2;
	const float SA2 = SA0*q0 + SA1*q1;
	const float SA3 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
	float SA4, SA5_inv;

	if (sq(SA3) > 1e-6f) {
		SA4 = 1.0F/sq(SA3);
		SA5_inv = sq(SA2)*SA4 + 1;
		canUseA = fabsf(SA5_inv) > 1e-6f;
	}

	bool canUseB = false;
	const float SB0 = 2*q0;
	const float SB1 = 2*q1;
	const float SB2 = SB0*q3 + SB1*q2;
	const float SB4 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
	float SB3, SB5_inv;

	if (sq(SB2) > 1e-6f) {
		SB3 = 1.0F/sq(SB2);
		SB5_inv = SB3*sq(SB4) + 1;
		canUseB = fabsf(SB5_inv) > 1e-6f;
	}

	Vector4f H_YAW;

	if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
		const float SA5 = 1.0F/SA5_inv;
		const float SA6 = 1.0F/SA3;
		const float SA7 = SA2*SA4;
		const float SA8 = 2*SA7;
		const float SA9 = 2*SA6;

		H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
		H_YAW(1) = SA5*(SA1*SA6 - SA8*q1);
		H_YAW(2) = SA5*(SA1*SA7 + SA9*q1);
		H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);
	} else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
		const float SB5 = 1.0F/SB5_inv;
		const float SB6 = 1.0F/SB2;
		const float SB7 = SB3*SB4;
		const float SB8 = 2*SB7;
		const float SB9 = 2*SB6;

		H_YAW(0) = -SB5*(SB0*SB6 - SB8*q3);
		H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
		H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
		H_YAW(3) = -SB5*(-SB0*SB7 - SB9*q3);
	} else {
		return false;
	}

	// calculate the yaw innovation and wrap to the interval between +-pi
	float innovation;

	if (zero_innovation) {
		innovation = 0.0f;

	} else {
		innovation = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - measurement);
	}

	// define the innovation gate size
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);

	// Update the quaternion states and covariance matrix
	return updateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

bool Ekf::fuseYaw312(float yaw, float yaw_variance, bool zero_innovation)
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	const float R_YAW = fmaxf(yaw_variance, 1.0e-4f);
	const float measurement = wrap_pi(yaw);

	// calculate 312 yaw observation matrix
	// choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	bool canUseA = false;
	const float SA0 = 2*q3;
	const float SA1 = 2*q2;
	const float SA2 = SA0*q0 - SA1*q1;
	const float SA3 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
	float SA4, SA5_inv;

	if (sq(SA3) > 1e-6f) {
		SA4 = 1.0F/sq(SA3);
		SA5_inv = sq(SA2)*SA4 + 1;
		canUseA = fabsf(SA5_inv) > 1e-6f;
	}

	bool canUseB = false;
	const float SB0 = 2*q0;
	const float SB1 = 2*q1;
	const float SB2 = -SB0*q3 + SB1*q2;
	const float SB4 = -sq(q0) + sq(q1) - sq(q2) + sq(q3);
	float SB3, SB5_inv;

	if (sq(SB2) > 1e-6f) {
		SB3 = 1.0F/sq(SB2);
		SB5_inv = SB3*sq(SB4) + 1;
		canUseB = fabsf(SB5_inv) > 1e-6f;
	}

	Vector4f H_YAW;

	if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
		const float SA5 = 1.0F/SA5_inv;
		const float SA6 = 1.0F/SA3;
		const float SA7 = SA2*SA4;
		const float SA8 = 2*SA7;
		const float SA9 = 2*SA6;

		H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
		H_YAW(1) = SA5*(-SA1*SA6 + SA8*q1);
		H_YAW(2) = SA5*(-SA1*SA7 - SA9*q1);
		H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);
	} else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
		const float SB5 = 1.0F/SB5_inv;
		const float SB6 = 1.0F/SB2;
		const float SB7 = SB3*SB4;
		const float SB8 = 2*SB7;
		const float SB9 = 2*SB6;

		H_YAW(0) = -SB5*(-SB0*SB6 + SB8*q3);
		H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
		H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
		H_YAW(3) = -SB5*(SB0*SB7 + SB9*q3);
	} else {
		return false;
	}

	float innovation;

	if (zero_innovation) {
		innovation = 0.0f;

	} else {
		// calculate the the innovation and wrap to the interval between +-pi
		innovation = wrap_pi(atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1)) - measurement);
	}

	// define the innovation gate size
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);

	// Update the quaternion states and covariance matrix
	return updateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

// update quaternion states and covariances using the yaw innovation, yaw observation variance and yaw Jacobian
bool Ekf::updateQuaternion(const float innovation, const float variance, const float gate_sigma,
			   const Vector4f &yaw_jacobian)
{
	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
	// calculate the innovation variance
	_heading_innov_var = variance;

	for (unsigned row = 0; row <= 3; row++) {
		float tmp = 0.0f;

		for (uint8_t col = 0; col <= 3; col++) {
			tmp += P(row, col) * yaw_jacobian(col);
		}

		_heading_innov_var += yaw_jacobian(row) * tmp;
	}

	float heading_innov_var_inv;

	// check if the innovation variance calculation is badly conditioned
	if (_heading_innov_var >= variance) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;
		heading_innov_var_inv = 1.0f / _heading_innov_var;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("mag yaw fusion numerical error - covariance reset");
		return false;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;

	for (uint8_t row = 0; row <= 15; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * yaw_jacobian(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	if (_control_status.flags.wind) {
		for (uint8_t row = 22; row <= 23; row++) {
			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion(row) += P(row, col) * yaw_jacobian(col);
			}

			Kfusion(row) *= heading_innov_var_inv;
		}
	}

	// innovation test ratio
	_yaw_test_ratio = sq(innovation) / (sq(gate_sigma) * _heading_innov_var);

	// we are no longer using 3-axis fusion so set the reported test levels to zero
	_mag_test_ratio.setZero();

	// set the magnetometer unhealthy if the test fails
	if (_yaw_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (!_control_status.flags.in_air && isTimedOut(_time_last_in_air, (uint64_t)5e6)) {
			// constrain the innovation to the maximum set by the gate
			// we need to delay this forced fusion to avoid starting it
			// immediately after touchdown, when the drone is still armed
			float gate_limit = sqrtf((sq(gate_sigma) * _heading_innov_var));
			_heading_innov = math::constrain(innovation, -gate_limit, gate_limit);

			// also reset the yaw gyro variance to converge faster and avoid
			// being stuck on a previous bad estimate
			resetZDeltaAngBiasCov();

		} else {
			return false;
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
		_heading_innov = innovation;
	}

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	SquareMatrix24f KHP;
	float KH[4];

	for (unsigned row = 0; row < _k_num_states; row++) {

		KH[0] = Kfusion(row) * yaw_jacobian(0);
		KH[1] = Kfusion(row) * yaw_jacobian(1);
		KH[2] = Kfusion(row) * yaw_jacobian(2);
		KH[3] = Kfusion(row) * yaw_jacobian(3);

		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[0] * P(0, column);
			tmp += KH[1] * P(1, column);
			tmp += KH[2] * P(2, column);
			tmp += KH[3] * P(3, column);
			KHP(row, column) = tmp;
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	_fault_status.flags.bad_hdg = !healthy;

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, _heading_innov);

		return true;
	}

	return false;
}

void Ekf::fuseHeading(float measured_hdg, float obs_var)
{
	// observation variance
	float R_YAW = PX4_ISFINITE(obs_var) ? obs_var : 0.01f;

	// update transformation matrix from body to world frame using the current state estimate
	const float predicted_hdg = getEulerYaw(_R_to_earth);

	if (!PX4_ISFINITE(measured_hdg)) {
		measured_hdg = predicted_hdg;
	}

	// handle special case where yaw measurement is unavailable
	bool fuse_zero_innov = false;

	if (_is_yaw_fusion_inhibited) {
		// The yaw measurement cannot be trusted but we need to fuse something to prevent a badly
		// conditioned covariance matrix developing over time.
		if (!_control_status.flags.vehicle_at_rest) {
			// Vehicle is not at rest so fuse a zero innovation if necessary to prevent
			// unconstrained quaternion variance growth and record the predicted heading
			// to use as an observation when movement ceases.
			// TODO a better way of determining when this is necessary
			const float sumQuatVar = P(0, 0) + P(1, 1) + P(2, 2) + P(3, 3);

			if (sumQuatVar > _params.quat_max_variance) {
				fuse_zero_innov = true;
				R_YAW = 0.25f;
			}

			_last_static_yaw = predicted_hdg;

		} else {
			// Vehicle is at rest so use the last moving prediction as an observation
			// to prevent the heading from drifting and to enable yaw gyro bias learning
			// before takeoff.
			if (!PX4_ISFINITE(_last_static_yaw)) {
				_last_static_yaw = predicted_hdg;
			}

			measured_hdg = _last_static_yaw;
		}

	} else {
		_last_static_yaw = predicted_hdg;
	}

	if (shouldUse321RotationSequence(_R_to_earth)) {
		fuseYaw321(measured_hdg, R_YAW, fuse_zero_innov);

	} else {
		fuseYaw312(measured_hdg, R_YAW, fuse_zero_innov);
	}
}

void Ekf::fuseDeclination(float decl_sigma)
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
		return;
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
		return;
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
}

void Ekf::limitDeclination()
{
	// get a reference value for the earth field declinaton and minimum plausible horizontal field strength
	// set to 50% of the horizontal strength from geo tables if location is known
	float decl_reference;
	float h_field_min = 0.001f;

	if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
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

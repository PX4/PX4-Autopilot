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
#include "python/ekf_derivation/generated/compute_yaw_321_innov_var_and_h.h"
#include "python/ekf_derivation/generated/compute_yaw_312_innov_var_and_h.h"
#include "python/ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h"

#include <mathlib/mathlib.h>

bool Ekf::fuseMag(const Vector3f &mag, estimator_aid_source3d_s &aid_src_mag, bool update_all_states)
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
				fused[2] = true;
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

		if (measurementUpdate(Kfusion, aid_src_mag.innovation_variance[index], aid_src_mag.innovation[index])) {
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
		aid_src_mag.time_last_fuse = _time_delayed_us;

		if (update_all_states) {
			_time_last_heading_fuse = _time_delayed_us;
		}

		return true;
	}

	aid_src_mag.fused = false;
	return false;
}

// update quaternion states and covariances using the yaw innovation and yaw observation variance
bool Ekf::fuseYaw(estimator_aid_source1d_s& aid_src_status)
{
	Vector24f H_YAW;
	computeYawInnovVarAndH(aid_src_status.observation_variance, aid_src_status.innovation_variance, H_YAW);

	return fuseYaw(aid_src_status, H_YAW);
}

bool Ekf::fuseYaw(estimator_aid_source1d_s& aid_src_status, const Vector24f &H_YAW)
{
	// define the innovation gate size
	float gate_sigma = math::max(_params.heading_innov_gate, 1.f);

	// innovation test ratio
	setEstimatorAidStatusTestRatio(aid_src_status, gate_sigma);

	if (aid_src_status.fusion_enabled) {

		// check if the innovation variance calculation is badly conditioned
		if (aid_src_status.innovation_variance >= aid_src_status.observation_variance) {
			// the innovation variance contribution from the state covariances is not negative, no fault
			_fault_status.flags.bad_hdg = false;

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
		const float heading_innov_var_inv = 1.f / aid_src_status.innovation_variance;

		for (uint8_t row = 0; row < _k_num_states; row++) {
			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion(row) += P(row, col) * H_YAW(col);
			}

			Kfusion(row) *= heading_innov_var_inv;
		}

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

		if (measurementUpdate(Kfusion, aid_src_status.innovation_variance, aid_src_status.innovation)) {

			_time_last_heading_fuse = _time_delayed_us;

			aid_src_status.time_last_fuse = _time_delayed_us;
			aid_src_status.fused = true;

			_fault_status.flags.bad_hdg = false;

			return true;

		} else {
			_fault_status.flags.bad_hdg = true;
		}
	}

	// otherwise
	aid_src_status.fused = false;
	return false;
}

void Ekf::computeYawInnovVarAndH(float variance, float &innovation_variance, Vector24f &H_YAW) const
{
	if (shouldUse321RotationSequence(_R_to_earth)) {
		sym::ComputeYaw321InnovVarAndH(getStateAtFusionHorizonAsVector(), P, variance, FLT_EPSILON, &innovation_variance, &H_YAW);

	} else {
		sym::ComputeYaw312InnovVarAndH(getStateAtFusionHorizonAsVector(), P, variance, FLT_EPSILON, &innovation_variance, &H_YAW);
	}
}

bool Ekf::fuseDeclination(float decl_sigma)
{
	// observation variance (rad**2)
	const float R_DECL = sq(decl_sigma);

	Vector24f H;
	float decl_pred;
	float innovation_variance;

	sym::ComputeMagDeclinationPredInnovVarAndH(getStateAtFusionHorizonAsVector(), P, R_DECL, FLT_EPSILON, &decl_pred, &innovation_variance, &H);

	const float innovation = wrap_pi(decl_pred - getMagDeclination());

	if (innovation_variance < R_DECL) {
		// variance calculation is badly conditioned
		return false;
	}

	SparseVector24f<16,17> Hfusion(H);

	// Calculate the Kalman gains
	Vector24f Kfusion = P * Hfusion / innovation_variance;

	const bool is_fused = measurementUpdate(Kfusion, innovation_variance, innovation);

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

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetometer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs;
}

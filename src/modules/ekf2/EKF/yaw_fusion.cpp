/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ekf.h"

#include <ekf_derivation/generated/compute_yaw_innov_var_and_h.h>

#include <mathlib/mathlib.h>

// update quaternion states and covariances using the yaw innovation and yaw observation variance
bool Ekf::fuseYaw(estimator_aid_source1d_s &aid_src_status)
{
	VectorState H_YAW;
	sym::ComputeYawInnovVarAndH(_state.vector(), P, aid_src_status.observation_variance, &aid_src_status.innovation_variance, &H_YAW);

	return fuseYaw(aid_src_status, H_YAW);
}

bool Ekf::fuseYaw(estimator_aid_source1d_s &aid_src_status, const VectorState &H_YAW)
{
	// define the innovation gate size
	float gate_sigma = math::max(_params.heading_innov_gate, 1.f);

	// innovation test ratio
	setEstimatorAidStatusTestRatio(aid_src_status, gate_sigma);

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
	VectorState Kfusion;
	const float heading_innov_var_inv = 1.f / aid_src_status.innovation_variance;

	for (uint8_t row = 0; row < State::size; row++) {
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
			resetGyroBiasZCov();

		} else {
			return false;
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
	}

	if (measurementUpdate(Kfusion, H_YAW, aid_src_status.observation_variance, aid_src_status.innovation)) {

		_time_last_heading_fuse = _time_delayed_us;

		aid_src_status.time_last_fuse = _time_delayed_us;
		aid_src_status.fused = true;

		_fault_status.flags.bad_hdg = false;

		return true;

	} else {
		_fault_status.flags.bad_hdg = true;
	}

	// otherwise
	aid_src_status.fused = false;
	return false;
}

void Ekf::computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const
{
	sym::ComputeYawInnovVarAndH(_state.vector(), P, variance, &innovation_variance, &H_YAW);
}

void Ekf::resetQuatStateYaw(float yaw, float yaw_variance)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	const Quatf quat_before_reset = _state.quat_nominal;

	// update the yaw angle variance
	if (PX4_ISFINITE(yaw_variance) && (yaw_variance > FLT_EPSILON)) {
		P.uncorrelateCovarianceSetVariance<1>(2, yaw_variance);
	}

	// update transformation matrix from body to world frame using the current estimate
	// update the rotation matrix using the new yaw value
	_R_to_earth = updateYawInRotMat(yaw, Dcmf(_state.quat_nominal));

	// calculate the amount that the quaternion has changed by
	const Quatf quat_after_reset(_R_to_earth);
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// update quaternion states
	_state.quat_nominal = quat_after_reset;

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

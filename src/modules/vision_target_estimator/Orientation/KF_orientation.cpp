/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file KF_orientation.cpp
 * @brief Filter to estimate the orientation of static and moving targets. State: [yaw, yaw_rate]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_orientation.h"

#include <cmath>
#include <float.h>

#include <px4_platform_common/defines.h>
#include <vte_orientation_derivation/generated/getTransitionMatrix.h>
#include <vte_orientation_derivation/generated/predictState.h>
#include <vte_orientation_derivation/generated/predictCov.h>

namespace vision_target_estimator
{

using VectorState = KF_orientation::VectorState;
using SquareMatrixState = KF_orientation::SquareMatrixState;

void KF_orientation::predict(float dt)
{
	if (!PX4_ISFINITE(dt) || (dt < 0.f)) {
		return;
	}

	VectorState state_next;
	SquareMatrixState cov_next;

	predictState(dt, EmptyInput{}, _state, _state_covariance, state_next, cov_next);

	_state = state_next;
	_state_covariance = cov_next;
}

SquareMatrixState KF_orientation::getTransitionMatrix(float dt) const
{
	return sym::Gettransitionmatrix(dt);
}

void KF_orientation::predictState(float dt, const EmptyInput &input,
				  const VectorState &x_in,
				  const SquareMatrixState &P_in,
				  VectorState &x_out,
				  SquareMatrixState &P_out)
{
	(void)input;

	x_out = sym::Predictstate(dt, x_in);
	x_out(KF_orientation::kYaw) = matrix::wrap_pi(x_out(KF_orientation::kYaw));

	// Keep invalid values from injecting covariance and use generated covariance propagation.
	const float yaw_acc_var = (PX4_ISFINITE(_yaw_acc_var) && (_yaw_acc_var > 0.f)) ? _yaw_acc_var : 0.f;
	P_out = sym::Predictcov(dt, yaw_acc_var, P_in);
}

void KF_orientation::computeInnovation(const VectorState &state,
				       const SquareMatrixState &cov,
				       const ScalarMeas &meas,
				       float &innov, float &innov_var) const
{

	innov = meas.val - (meas.H.transpose() * state)(0, 0);
	innov_var = (meas.H.transpose() * cov * meas.H)(0, 0) + meas.unc;

	// Wrap innovations for yaw measurements
	const bool is_yaw_meas = (fabsf(meas.H(KF_orientation::kYaw)) > 0.f) && (fabsf(meas.H(KF_orientation::kYawRate)) < FLT_EPSILON);

	if (is_yaw_meas) {
		innov = matrix::wrap_pi(innov);
	}
}

void KF_orientation::pushHistory(const uint64_t time_us)
{
	_oosm.push(time_us, _state, _state_covariance, EmptyInput{});
}

void KF_orientation::resetHistory()
{
	_oosm.reset();
}

FusionResult KF_orientation::fuseScalarAtTime(const ScalarMeas &meas, uint64_t now_us, float nis_threshold)
{
	return _oosm.fuse(*this, meas, now_us, nis_threshold, _state, _state_covariance, EmptyInput{});
}

void KF_orientation::applyCorrection(VectorState &state,
				     SquareMatrixState &cov,
				     const VectorState &K,
				     float innov, float S)
{
	state = state + K * innov;
	state(KF_orientation::kYaw) = matrix::wrap_pi(state(KF_orientation::kYaw));

	for (size_t row = 0; row < KF_orientation::kSize; row++) {
		for (size_t col = 0; col < KF_orientation::kSize; col++) {
			cov(row, col) -= K(row) * K(col) * S;
		}
	}

	for (size_t i = 0; i < KF_orientation::kSize; i++) {
		cov(i, i) = fmaxf(cov(i, i), kMinVar);
	}
}

} // namespace vision_target_estimator

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
 * @file KF_position.cpp
 * @brief Filter to estimate the pose of moving targets. State: [pos_rel, vel_uav, bias, acc_target, vel_target]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_position.h"

#include <cmath>

#include <px4_platform_common/defines.h>
#include <vtest_derivation/generated/predictState.h>
#include <vtest_derivation/generated/predictCov.h>
#include <vtest_derivation/generated/computeInnovCov.h>
#include <vtest_derivation/generated/applyCorrection.h>
#include <vtest_derivation/generated/getTransitionMatrix.h>

namespace vision_target_estimator
{

void KF_position::predict(float dt, float acc_uav)
{

	if (!PX4_ISFINITE(dt) || (dt < 0.f) || !PX4_ISFINITE(acc_uav)) {
		return;
	}

	_last_acc = acc_uav;

	VectorState state_next;
	SquareMatrixState cov_next;

	predictState(dt, acc_uav, _state, _state_covariance, state_next, cov_next);

	_state = state_next;
	_state_covariance = cov_next;
}

KF_position::SquareMatrixState KF_position::getTransitionMatrix(float dt) const
{
	return sym::Gettransitionmatrix(dt);
}

void KF_position::predictState(float dt, float acc,
			       const VectorState &x_in,
			       const SquareMatrixState &P_in,
			       VectorState &x_out,
			       SquareMatrixState &P_out)
{
	x_out = sym::Predictstate(dt, x_in, acc);
	P_out = sym::Predictcov(dt, _input_var, _bias_var, _acc_var, P_in);
}

void KF_position::computeInnovation(const VectorState &state,
				    const SquareMatrixState &cov,
				    const ScalarMeas &meas,
				    float &innov, float &innov_var) const
{
	innov = meas.val - (meas.H.transpose() * state)(0, 0);
	innov_var = sym::Computeinnovcov(meas.unc, cov, meas.H.transpose());
}

void KF_position::pushHistory(const uint64_t time_us)
{
	_oosm.push(time_us, _state, _state_covariance, _last_acc);
}

void KF_position::resetHistory()
{
	_oosm.reset();
}

FusionResult KF_position::fuseScalarAtTime(const ScalarMeas &meas, uint64_t now_us, float nis_threshold)
{
	return _oosm.fuse(*this, meas, now_us, nis_threshold, _state, _state_covariance, _last_acc);
}

void KF_position::applyCorrection(VectorState &state, SquareMatrixState &cov, const VectorState &K, float innov, float S)
{
	VectorState state_new;
	SquareMatrixState cov_new;

	sym::Applycorrection(state, cov, K, innov, S, &state_new, &cov_new);

	state = state_new;
	cov = cov_new;

	// Clamp diagonal assuming a small epsilon for stability:
	for (size_t i = 0; i < kStateSize; i++) {
		cov(i, i) = fmaxf(cov(i, i), kMinVar);
	}
}

} // namespace vision_target_estimator

/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/defines.h>

namespace vision_target_estimator
{

void KF_orientation::predict(float dt)
{
	matrix::Vector<float, State::size> state_next;
	matrix::SquareMatrix<float, State::size> cov_next;

	predictState(dt, EmptyInput{}, _state, _state_covariance, state_next, cov_next);

	_state = state_next;
	_state_covariance = cov_next;
}

void KF_orientation::getTransitionMatrix(float dt, matrix::SquareMatrix<float, State::size> &phi) const
{
	float data[State::size * State::size] = {
		1.f, dt,
		0.f, 1.f
	};

	phi = matrix::SquareMatrix<float, State::size>(data);
}

void KF_orientation::predictState(float dt, const EmptyInput &input,
				  const matrix::Vector<float, State::size> &x_in,
				  const matrix::SquareMatrix<float, State::size> &P_in,
				  matrix::Vector<float, State::size> &x_out,
				  matrix::SquareMatrix<float, State::size> &P_out)
{
	(void)input;

	matrix::SquareMatrix<float, State::size> phi;
	getTransitionMatrix(dt, phi);

	x_out = phi * x_in;
	x_out(State::yaw) = matrix::wrap_pi(x_out(State::yaw));
	P_out = phi * P_in * phi.transpose();

	// Add process noise assuming a white-noise yaw acceleration model
	// Q = q * [[dt^3/3, dt^2/2],
	//          [dt^2/2, dt]]
	if (PX4_ISFINITE(_yaw_acc_var) && (_yaw_acc_var > 0.f)) {
		const float dt2 = dt * dt;
		const float dt3 = dt2 * dt;

		matrix::SquareMatrix<float, State::size> Q{};
		Q(State::yaw, State::yaw) = (dt3 / 3.f) * _yaw_acc_var;
		Q(State::yaw, State::yaw_rate) = (dt2 / 2.f) * _yaw_acc_var;
		Q(State::yaw_rate, State::yaw) = Q(State::yaw, State::yaw_rate);
		Q(State::yaw_rate, State::yaw_rate) = dt * _yaw_acc_var;

		P_out = P_out + Q;
	}
}

void KF_orientation::computeInnovation(const matrix::Vector<float, State::size> &state,
				       const matrix::SquareMatrix<float, State::size> &cov,
				       const ScalarMeas &meas,
				       float &innov, float &innov_var) const
{
	innov = matrix::wrap_pi(meas.val - (meas.H.transpose() * state)(0, 0));
	innov_var = (meas.H.transpose() * cov * meas.H)(0, 0) + meas.unc;
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

void KF_orientation::applyCorrection(matrix::Vector<float, State::size> &state,
				     matrix::SquareMatrix<float, State::size> &cov,
				     const matrix::Vector<float, State::size> &K,
				     float innov, float S)
{
	state = state + K * innov;
	state(State::yaw) = matrix::wrap_pi(state(State::yaw));

	for (int row = 0; row < State::size; row++) {
		for (int col = 0; col < State::size; col++) {
			cov(row, col) -= K(row) * K(col) * S;
		}
	}

	static constexpr float kMinVar = 1e-9f;

	for (int i = 0; i < State::size; i++) {
		cov(i, i) = fmaxf(cov(i, i), kMinVar);
	}
}

} // namespace vision_target_estimator

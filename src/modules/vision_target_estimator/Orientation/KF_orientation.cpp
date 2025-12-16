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

void KF_orientation::predictState(float dt)
{
	matrix::SquareMatrix<float, State::size> phi = getTransitionMatrix(dt);
	_state = phi * _state;
	_state(State::yaw) = matrix::wrap_pi(_state(State::yaw));
}

void KF_orientation::predictCov(float dt)
{
	matrix::SquareMatrix<float, State::size> phi = getTransitionMatrix(dt);
	_state_covariance = phi * _state_covariance * phi.transpose();
}

void KF_orientation::pushHistory(const uint64_t time_us)
{
	_history[_history_head].time_us = time_us;
	_history[_history_head].state = _state;
	_history[_history_head].cov = _state_covariance;

	_history_head = (_history_head + 1) % kHistorySize;

	if (_history_head == 0) {
		_history_valid = true;
	}
}

void KF_orientation::resetHistory()
{
	_history_head = 0;
	_history_valid = false;

	for (auto &sample : _history) {
		sample.time_us = 0;
	}
}

FusionResult KF_orientation::fuseScalarAtTime(const ScalarMeas &meas, uint64_t now_us, float nis_threshold)
{
	FusionResult res{};

	static constexpr uint64_t kOosmMinTimeUs = 20_ms;
	static constexpr uint64_t kOosmMaxTimeUs = 500_ms;
	const uint64_t time_diff = (now_us >= meas.time_us) ? (now_us - meas.time_us) : (meas.time_us - now_us);

	// No need for OOSM
	if (time_diff < kOosmMinTimeUs) {
		matrix::Vector<float, State::size> K;

		if (computeFusionGain(_state, _state_covariance, meas, nis_threshold, res, K)) {
			applyCorrection(_state, _state_covariance, K, res.innov, res.innov_var);
			res.status = FusionStatus::FUSED_CURRENT;
		}

		return res;
	}

	if (!_history_valid && _history_head == 0) {
		res.status = FusionStatus::REJECT_EMPTY;
		return res;
	}

	// Newest sample is always just before the head
	const int newest_idx = (_history_head == 0) ? (kHistorySize - 1) : (_history_head - 1);
	// Oldest sample is at head (if full) or at 0 (if filling)
	const int oldest_idx = _history_valid ? _history_head : 0;

	const uint64_t newest_time_us = _history[newest_idx].time_us;
	const uint64_t oldest_time_us = _history[oldest_idx].time_us;

	// Reset history if:
	// - newest history sample is too old (stale)
	// - the caller's "now" is older than the newest history sample (time discontinuity).
	const uint64_t time_since_newest = (now_us >= newest_time_us) ? (now_us - newest_time_us) : 0;

	if (time_since_newest > kOosmMaxTimeUs || now_us < newest_time_us) {
		resetHistory();
		res.status = FusionStatus::REJECT_STALE;
		return res;
	}

	// Reject fusion if the measurement time is in the future beyond a small tolerance.
	if (meas.time_us > now_us + kOosmMinTimeUs) {
		res.status = FusionStatus::REJECT_TOO_NEW;
		return res;
	}

	// Reject fusion if the measurement is too old.
	if (time_diff > kOosmMaxTimeUs || meas.time_us < oldest_time_us) {
		res.status = FusionStatus::REJECT_TOO_OLD;
		return res;
	}

	// Most measurements are recent. Search backwards from newest.
	int floor_idx = -1;
	int curr_idx = newest_idx;

	for (int i = 0; i < kHistorySize; i++) {
		if (_history[curr_idx].time_us <= meas.time_us) {
			floor_idx = curr_idx;
			break;
		}

		curr_idx = (curr_idx == 0) ? (kHistorySize - 1) : (curr_idx - 1);

		// If we wrapped all the way to newest (should be caught by oldest_time check above)
		if (curr_idx == newest_idx) { break; }
	}

	if (floor_idx == -1) {
		res.status = FusionStatus::REJECT_TOO_OLD;
		return res;
	}

	// Measurement Update at t_meas (predict from floor sample to the exact measurement timestamp).
	const StateSample &sample_floor = _history[floor_idx];
	const uint64_t t_floor_us = sample_floor.time_us;

	// Predict from floor forward to the exact measurement time.
	const float dt_meas = (meas.time_us - t_floor_us) * 1e-6f;

	matrix::Vector<float, State::size> x_meas_pred = sample_floor.state;
	matrix::SquareMatrix<float, State::size> P_meas_pred = sample_floor.cov;

	if (dt_meas > 0.f) {
		const matrix::SquareMatrix<float, State::size> Phi = getTransitionMatrix(dt_meas);
		x_meas_pred = Phi * sample_floor.state;
		x_meas_pred(State::yaw) = matrix::wrap_pi(x_meas_pred(State::yaw));
		P_meas_pred = Phi * sample_floor.cov * Phi.transpose();
	}

	matrix::Vector<float, State::size> K_meas;

	if (!computeFusionGain(x_meas_pred, P_meas_pred, meas, nis_threshold, res, K_meas)) {
		return res; // res contains the failure reason (NIS or COV)
	}

	matrix::SquareMatrix<float, State::size> Phi_step; // from t_prev to t_current
	matrix::SquareMatrix<float, State::size> Phi_cumulative; // from t_meas till t_prev
	Phi_cumulative.setIdentity();

	// If the fusion time lands exactly on a history sample, update it directly.
	if (meas.time_us == t_floor_us) {
		StateSample &sample_at_meas = _history[floor_idx];
		applyCorrection(sample_at_meas.state, sample_at_meas.cov, K_meas, res.innov, res.innov_var);
	}

	// Update History Buffer (samples strictly after t_meas)
	uint64_t prev_time_us = meas.time_us;
	int idx = (floor_idx + 1) % kHistorySize;
	uint8_t history_steps = 0;

	while (idx != _history_head) {
		StateSample &sample = _history[idx];

		if (sample.time_us <= meas.time_us) {
			idx = (idx + 1) % kHistorySize;
			continue;
		}

		const uint64_t curr_time_us = sample.time_us;
		const float dt = (curr_time_us - prev_time_us) * 1e-6f;

		Phi_step = getTransitionMatrix(dt);
		Phi_cumulative = Phi_step * Phi_cumulative;

		const matrix::Vector<float, State::size> K_proj = Phi_cumulative * K_meas;
		applyCorrection(sample.state, sample.cov, K_proj, res.innov, res.innov_var);
		history_steps++;

		prev_time_us = curr_time_us;
		idx = (idx + 1) % kHistorySize;
	}

	// Update Live State (Consistent with History Update)
	if (now_us > prev_time_us) {
		const float dt_now = (now_us - prev_time_us) * 1e-6f;
		Phi_step = getTransitionMatrix(dt_now);
		Phi_cumulative = Phi_step * Phi_cumulative;
	}

	const matrix::Vector<float, State::size> K_proj_now = Phi_cumulative * K_meas;
	applyCorrection(_state, _state_covariance, K_proj_now, res.innov, res.innov_var);

	res.history_steps = history_steps;
	res.status = FusionStatus::FUSED_OOSM;
	return res;
}

bool KF_orientation::computeFusionGain(const matrix::Vector<float, State::size> &state,
				       const matrix::SquareMatrix<float, State::size> &cov, const ScalarMeas &meas, float nis_threshold,
				       FusionResult &out_res,
				       matrix::Vector<float, State::size> &out_K)
{
	const float innov = matrix::wrap_pi(meas.val - (meas.H.transpose() * state)(0, 0));
	const float innov_cov = (meas.H.transpose() * cov * meas.H)(0, 0) + meas.unc;

	out_res.innov = innov;
	out_res.innov_var = innov_cov;

	if (!PX4_ISFINITE(innov_cov) || innov_cov < 1e-6f) {
		out_res.status = FusionStatus::REJECT_COV;
		return false;
	}

	const float beta = math::sq(innov) / innov_cov;

	if (nis_threshold > 0.f) {
		out_res.test_ratio = beta / nis_threshold;

	} else {
		out_res.test_ratio = -1.f;
	}

	if ((nis_threshold > 0.f) && (beta > nis_threshold)) {
		out_res.status = FusionStatus::REJECT_NIS;
		return false;
	}

	out_K = cov * meas.H / innov_cov;
	return true;
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

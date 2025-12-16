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
 * @file KF_position.cpp
 * @brief Filter to estimate the pose of moving targets. State: [pos_rel, vel_uav, bias, acc_target, vel_target]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_position.h"

#include <px4_platform_common/defines.h>

#include <vtest_derivation/generated/predictState.h>
#include <vtest_derivation/generated/predictCov.h>
#include <vtest_derivation/generated/computeInnovCov.h>
#include <vtest_derivation/generated/applyCorrection.h>
#include <vtest_derivation/generated/getTransitionMatrix.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

namespace vision_target_estimator
{

void KF_position::predictState(float dt, float acc_uav)
{
	_last_acc = acc_uav;
	matrix::Vector<float, vtest::State::size> state_updated;
	sym::Predictstate(dt, _state, acc_uav, &state_updated);
	_state = state_updated;
}

void KF_position::predictCov(float dt)
{
	matrix::Matrix<float, vtest::State::size, vtest::State::size> cov_updated;
	sym::Predictcov(dt, _input_var, _bias_var, _acc_var, _state_covariance, &cov_updated);
	_state_covariance = cov_updated;
}

void KF_position::pushHistory(const uint64_t time_us)
{
	_history[_history_head].time_us = time_us;
	_history[_history_head].state = _state;
	_history[_history_head].cov = _state_covariance;
	_history[_history_head].acc = _last_acc;

	_history_head = (_history_head + 1) % kHistorySize;

	if (_history_head == 0) {
		_history_valid = true;
	}
}

void KF_position::resetHistory()
{
	_history_head = 0;
	_history_valid = false;

	for (auto &sample : _history) {
		sample.time_us = 0;
		sample.acc = 0.f;
	}
}

bool KF_position::fuseScalarAtTime(uint64_t meas_time_us, uint64_t now_us, float meas, float meas_unc,
				   const matrix::Vector<float, vtest::State::size> &H, float nis_threshold,
				   float &out_innov, float &out_innov_var)
{
	out_innov = 0.f;
	out_innov_var = 0.f;

	static constexpr uint64_t kOosmMinTimeUs = 20_ms;
	const uint64_t time_diff = (now_us >= meas_time_us) ? (now_us - meas_time_us) : (meas_time_us - now_us);

	static constexpr float kMinMeasVar = 0.1f;
	meas_unc = fmaxf(meas_unc, kMinMeasVar);

	// No need for OOSM
	if (time_diff < kOosmMinTimeUs) {
		matrix::Vector<float, vtest::State::size> K;

		// Use helper to compute K and Innov based on current state
		if (computeFusionGain(_state, _state_covariance, meas, meas_unc, H, nis_threshold,
				      out_innov, out_innov_var, K)) {

			applyCorrection(_state, _state_covariance, K, out_innov, out_innov_var);
			return true;
		}

		return false;
	}

	if (!_history_valid && _history_head == 0) {
		return false; // Empty history
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
	static constexpr uint64_t kOosmMaxTimeUs = 500_ms;
	const uint64_t time_since_newest = (now_us >= newest_time_us) ? (now_us - newest_time_us) : 0;

	if (time_since_newest > kOosmMaxTimeUs || now_us < newest_time_us) {
		// TODO: this needs to be logged somehow e.g. reset counter
		resetHistory();
		return false;
	}

	// Reject fusion if:
	// - measurement is too old
	// - no OOSM possible because the measurement time is older than the oldest available history sample.
	// - the measurement time is in the future beyond a small tolerance.
	if (time_diff > kOosmMaxTimeUs ||
	    meas_time_us < oldest_time_us ||
	    meas_time_us > now_us + kOosmMinTimeUs) {
		return false;
	}

	// Most measurements are recent. Search backwards from newest.
	int floor_idx = -1;
	int curr_idx = newest_idx;

	for (int i = 0; i < kHistorySize; i++) {
		if (_history[curr_idx].time_us <= meas_time_us) {
			floor_idx = curr_idx;
			break;
		}

		curr_idx = (curr_idx == 0) ? (kHistorySize - 1) : (curr_idx - 1);

		// If we wrapped all the way to newest (should be caught by oldest_time check above)
		if (curr_idx == newest_idx) { break; }
	}

	if (floor_idx == -1) {
		return false;
	}

	// Measurement Update at t_meas (predict from floor sample to the exact measurement timestamp).
	const StateSample &sample_floor = _history[floor_idx];
	const uint64_t t_floor_us = sample_floor.time_us;

	// Determine the input used to predict within the interval containing the measurement.
	// We store the acceleration used to predict *to* each history sample, so the interval
	// (t_k -> t_{k+1}) uses the input stored in sample k+1. If the floor sample is the newest
	// sample, fall back to the last input used for the live prediction step.
	float acc_interval = _last_acc;

	if (t_floor_us != newest_time_us) {
		const int next_idx = (floor_idx + 1) % kHistorySize;
		acc_interval = _history[next_idx].acc;
	}

	// Predict from floor forward to the exact measurement time.
	const float dt_meas = (meas_time_us - t_floor_us) * 1e-6f;

	matrix::Vector<float, vtest::State::size> x_meas_pred = sample_floor.state;
	matrix::SquareMatrix<float, vtest::State::size> P_meas_pred = sample_floor.cov;

	if (dt_meas > 0.f) {
		sym::Predictstate(dt_meas, sample_floor.state, acc_interval, &x_meas_pred);
		sym::Predictcov(dt_meas, _input_var, _bias_var, _acc_var, sample_floor.cov, &P_meas_pred);
	}

	matrix::Vector<float, vtest::State::size> K_meas;

	if (!computeFusionGain(x_meas_pred, P_meas_pred, meas, meas_unc, H, nis_threshold,
			       out_innov, out_innov_var, K_meas)) {
		return false;
	}

	// Project correction forward (OOSM approximation).
	// Keep the linearization points consistent with the predictions used:
	// - For the sub-interval (t_meas -> t_next) we linearize at x_meas_pred and use the input stored in t_next.
	// - For subsequent intervals we linearize at the original history state at the interval start.

	matrix::SquareMatrix<float, vtest::State::size> Phi_step; // from t_prev to t_current
	matrix::SquareMatrix<float, vtest::State::size> Phi_cumulative; // from t_meas till t_prev
	Phi_cumulative.setIdentity();

	// Temp variables for SymForce output
	matrix::Vector<float, vtest::State::size> state_new;
	matrix::SquareMatrix<float, vtest::State::size> cov_new;

	// If the fusion time lands exactly on a history sample, update it directly.
	if (meas_time_us == t_floor_us) {
		StateSample &sample_at_meas = _history[floor_idx];
		applyCorrection(sample_at_meas.state, sample_at_meas.cov, K_meas, out_innov, out_innov_var);
	}

	// Update History Buffer (samples strictly after t_meas)
	matrix::Vector<float, vtest::State::size> prev_state_linearization = x_meas_pred;
	uint64_t prev_time_us = meas_time_us;
	int idx = (floor_idx + 1) % kHistorySize;

	while (idx != _history_head) {
		StateSample &sample = _history[idx];

		if (sample.time_us <= meas_time_us) {
			idx = (idx + 1) % kHistorySize;
			continue;
		}

		const uint64_t curr_time_us = sample.time_us;
		const float dt = (curr_time_us - prev_time_us) * 1e-6f;

		// Keep linearization consistent: use the state at the interval start.
		sym::Gettransitionmatrix(dt, prev_state_linearization, sample.acc, &Phi_step);
		Phi_cumulative = Phi_step * Phi_cumulative;

		// Project the correction vector.
		// Note: We only use Phi (and not G*acc) because we are projecting the error state (delta).
		// The input (acc) is common to both the reference and corrected trajectories and cancels out.
		const matrix::Vector<float, vtest::State::size> K_proj = Phi_cumulative * K_meas;
		const matrix::Vector<float, vtest::State::size> curr_state_linearization = sample.state;

		applyCorrection(sample.state, sample.cov, K_proj, out_innov, out_innov_var);

		prev_state_linearization = curr_state_linearization;
		prev_time_us = curr_time_us;

		idx = (idx + 1) % kHistorySize;
	}

	// Update Live State (Consistent with History Update)
	if (now_us > prev_time_us) {
		const float dt_now = (now_us - prev_time_us) * 1e-6f;
		sym::Gettransitionmatrix(dt_now, prev_state_linearization, _last_acc, &Phi_step);
		Phi_cumulative = Phi_step * Phi_cumulative;
	}

	const matrix::Vector<float, vtest::State::size> K_proj_now = Phi_cumulative * K_meas;
	applyCorrection(_state, _state_covariance, K_proj_now, out_innov, out_innov_var);

	return true;
}

bool KF_position::computeFusionGain(const matrix::Vector<float, vtest::State::size> &state,
				    const matrix::SquareMatrix<float, vtest::State::size> &cov,
				    float meas, float meas_unc,
				    const matrix::Vector<float, vtest::State::size> &H,
				    float nis_threshold,
				    float &out_innov, float &out_innov_var,
				    matrix::Vector<float, vtest::State::size> &out_K)
{
	const float innov = meas - (H.transpose() * state)(0, 0);

	float innov_cov;
	sym::Computeinnovcov(meas_unc, cov, H.transpose(), &innov_cov);

	if (!PX4_ISFINITE(innov_cov) || innov_cov < 1e-6f) {
		return false;
	}

	out_innov = innov;
	out_innov_var = innov_cov;

	// NIS Check
	const float beta = (innov * innov) / innov_cov;

	if ((nis_threshold > 0.f) && (beta > nis_threshold)) {
		return false;
	}

	// Compute Kalman Gain (K)
	out_K = cov * H / innov_cov;

	return true;
}

void KF_position::applyCorrection(matrix::Vector<float, vtest::State::size> &state,
				  matrix::SquareMatrix<float, vtest::State::size> &cov,
				  const matrix::Vector<float, vtest::State::size> &K,
				  float innov, float S)
{
	matrix::Vector<float, vtest::State::size> state_new;
	matrix::SquareMatrix<float, vtest::State::size> cov_new;

	sym::Applycorrection(state, cov, K, innov, S, &state_new, &cov_new);

	state = state_new;
	cov = cov_new;

	// Clamp diagonal assuming a small epsilon for stability:
	static constexpr float kMinVar = 1e-9f;

	for (int i = 0; i < vtest::State::size; i++) {
		cov(i, i) = fmaxf(cov(i, i), kMinVar);
	}
}

} // namespace vision_target_estimator

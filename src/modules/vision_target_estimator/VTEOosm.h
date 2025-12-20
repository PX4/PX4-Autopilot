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
 * @file VTEOosm.h
 * @brief Generic Out-Of-Sequence Measurement (OOSM) Manager for VTE filters.
 *
 * Implements a templated Ring Buffer and the "Projected Correction" algorithm
 * to fuse delayed measurements without replaying the full filter history.
 */

#pragma once

#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include "common.h"
#include <lib/ringbuffer/TimestampedRingBuffer.hpp>

namespace vision_target_estimator
{

// Tag type for filters that do not store control inputs (e.g. Orientation filter)
struct EmptyInput {};

/**
 * @class OOSMManager
 * @brief Manages history and performs OOSM fusion.
 *
 * @tparam FilterT      The filter class (Must implement the Interface defined below).
 * @tparam StateDim     Dimension of the state vector.
 * @tparam InputT       Type of control input stored (e.g. float for acc, or EmptyInput).
 * @tparam HistorySize  Size of the static ring buffer.
 */
template <typename FilterT, int StateDim, typename InputT, int HistorySize = 25>
class OOSMManager
{
public:
	using StateVec = matrix::Vector<float, StateDim>;
	using StateCov = matrix::SquareMatrix<float, StateDim>;

	struct Sample {
		uint64_t time_us{0};
		StateVec state{};
		StateCov cov{};
		InputT input{}; // Input used to reach this state from the previous timestamp
	};

	static_assert(HistorySize > 0, "HistorySize must be > 0");
	static_assert(HistorySize <= 255, "HistorySize must fit in uint8_t");
	static constexpr uint8_t kHistorySize = static_cast<uint8_t>(HistorySize);
	static constexpr uint64_t kOosmMinTimeUs = 20_ms;
	static constexpr uint64_t kOosmMaxTimeUs = 500_ms;

	OOSMManager() = default;

	void reset()
	{
		if (_history.valid()) {
			_history.reset();
		}
	}

	/**
	 * @brief Add a post-update state snapshot to history.
	 * @param input The control input that was used to propagate *to* this state (from prev).
	 */
	void push(uint64_t time_us, const StateVec &state, const StateCov &cov, const InputT &input)
	{
		if (!_history.valid()) {
			return;
		}

		Sample sample{};
		sample.time_us = time_us;
		sample.state = state;
		sample.cov = cov;
		sample.input = input;

		_history.push(sample);
	}

	/**
	 * @brief Fuse a measurement that might be delayed (OOSM).
	 *
	 * @param filter        Reference to the filter (provides physics/math methods).
	 * @param meas          The scalar measurement.
	 * @param now_us        Current system time.
	 * @param nis_threshold NIS threshold for rejection.
	 * @param curr_state    [In/Out] Current filter state (will be corrected).
	 * @param curr_cov      [In/Out] Current filter covariance.
	 * @param curr_input    The input currently being used for the *next* prediction (fallback).
	 */
	template <typename ScalarMeasT>
	FusionResult fuse(FilterT &filter, const ScalarMeasT &meas, uint64_t now_us, float nis_threshold,
			  StateVec &curr_state, StateCov &curr_cov, const InputT &curr_input)
	{
		FusionResult res{};

		const uint64_t time_diff = (now_us >= meas.time_us) ? (now_us - meas.time_us) : (meas.time_us - now_us);

		// No OOSM, current Measurement (or very recent)
		if (time_diff < kOosmMinTimeUs) {
			StateVec K;

			if (processMeasurement(filter, curr_state, curr_cov, meas, nis_threshold, res, K)) {
				filter.applyCorrection(curr_state, curr_cov, K, res.innov, res.innov_var);
				res.status = FusionStatus::FUSED_CURRENT;
			}

			return res;
		}

		// OOSM checks
		if (!_history.valid() || (_history.entries() == 0)) {
			res.status = FusionStatus::REJECT_EMPTY;
			return res;
		}

		const uint8_t newest_idx = _history.get_newest_index();
		const uint64_t newest_time_us = _history.get_newest().time_us;
		const uint64_t oldest_time_us = _history.get_oldest().time_us;

		// Check buffer consistency (reset if the "live" time jumped backwards or too far forward)
		if (now_us < newest_time_us || (now_us - newest_time_us) > kOosmMaxTimeUs) {
			reset();
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

		// Find the floor sample (sample immediately before or at meas time)
		bool floor_found = false;
		uint8_t floor_idx = newest_idx;
		uint8_t curr_idx = newest_idx;

		const int history_count = _history.entries();

		for (int i = 0; i < history_count; i++) {
			if (_history[curr_idx].time_us <= meas.time_us) {
				floor_idx = curr_idx;
				floor_found = true;
				break;
			}

			curr_idx = _history.prev(curr_idx);
		}

		if (!floor_found) {
			res.status = FusionStatus::REJECT_TOO_OLD;
			return res;
		}

		// Predict from floor to exact measurement time
		const Sample &sample_floor = _history[floor_idx];
		const uint64_t t_floor_us = sample_floor.time_us;

		// Determine input for the interval [t_floor, t_meas].
		// The history stores the input used to reach a sample.
		// So the input *during* the interval starting at `floor` is stored in `floor+1`.
		// If floor is the newest, we use the current live input.
		InputT input_interval = curr_input;

		if (floor_idx != newest_idx) {
			const uint8_t next_idx = _history.next(floor_idx);
			input_interval = _history[next_idx].input;
		}

		const float dt_meas = (meas.time_us - t_floor_us) * 1e-6f;

		// Temp variables for prediction at t_meas
		StateVec x_meas = sample_floor.state;
		StateCov P_meas = sample_floor.cov;

		if (dt_meas > 0.f) {
			// Filter must implement: predictState(dt, input, x_in, P_in, x_out, P_out)
			filter.predictState(dt_meas, input_interval, sample_floor.state, sample_floor.cov, x_meas, P_meas);
		}

		// Compute Gain K at t_meas
		StateVec K_meas;

		if (!processMeasurement(filter, x_meas, P_meas, meas, nis_threshold, res, K_meas)) {
			return res; // REJECT_NIS or REJECT_COV
		}

		// Apply & Project Correction
		// If t_meas matches t_floor exactly, correct the floor sample directly
		if (meas.time_us == t_floor_us) {
			filter.applyCorrection(_history[floor_idx].state, _history[floor_idx].cov, K_meas, res.innov, res.innov_var);
		}

		StateCov Phi_step;
		StateCov Phi_cumulative;
		Phi_cumulative.setIdentity();

		uint64_t prev_time_us = meas.time_us;
		const uint8_t end_idx = _history.next(newest_idx);
		uint8_t idx = _history.next(floor_idx);
		uint8_t steps = 0;

		// Iterate from meas time up to newest sample
		while (idx != end_idx) {
			Sample &sample = _history[idx];

			// Skip if this sample is older than measurement (shouldn't happen given finding logic, but safety)
			if (sample.time_us <= meas.time_us) {
				idx = _history.next(idx);
				continue;
			}

			const float dt = (sample.time_us - prev_time_us) * 1e-6f;

			// Get transition matrix for the error state dynamics
			filter.getTransitionMatrix(dt, Phi_step);
			Phi_cumulative = Phi_step * Phi_cumulative;

			// Project K: K_k = Phi * K_{k-1}
			const StateVec K_proj = Phi_cumulative * K_meas;

			// Apply correction to history
			filter.applyCorrection(sample.state, sample.cov, K_proj, res.innov, res.innov_var);

			prev_time_us = sample.time_us;
			idx = _history.next(idx);
			steps++;
		}

		// Update Live State
		if (now_us > prev_time_us) {
			const float dt_now = (now_us - prev_time_us) * 1e-6f;
			filter.getTransitionMatrix(dt_now, Phi_step);
			Phi_cumulative = Phi_step * Phi_cumulative;
		}

		const StateVec K_final = Phi_cumulative * K_meas;
		filter.applyCorrection(curr_state, curr_cov, K_final, res.innov, res.innov_var);

		res.status = FusionStatus::FUSED_OOSM;
		res.history_steps = steps;

		return res;
	}

private:
	template <typename ScalarMeasT>
	bool processMeasurement(FilterT &filter, const StateVec &state, const StateCov &cov,
				const ScalarMeasT &meas, float nis_threshold,
				FusionResult &out_res, StateVec &out_K)
	{
		float innov = 0.f;
		float innov_var = 0.f;

		filter.computeInnovation(state, cov, meas, innov, innov_var);

		out_res.innov = innov;
		out_res.innov_var = innov_var;

		if (!PX4_ISFINITE(innov_var) || (innov_var < 1e-6f)) {
			out_res.status = FusionStatus::REJECT_COV;
			out_res.test_ratio = -1.f;
			return false;
		}

		const float beta = (innov * innov) / innov_var;

		if (nis_threshold > 0.f) {
			out_res.test_ratio = beta / nis_threshold;

			if (beta > nis_threshold) {
				out_res.status = FusionStatus::REJECT_NIS;
				return false;
			}

		} else {
			out_res.test_ratio = -1.f;
		}

		out_K = (cov * meas.H) / innov_var;
		return true;
	}

	TimestampedRingBuffer<Sample, kHistorySize> _history;
};

} // namespace vision_target_estimator

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
 * @file VTEOosm.h
 * @brief Generic Out-Of-Sequence Measurement (OOSM) manager for VTE filters.
 *
 * Implements the "projected correction" OOSM algorithm backed by a fixed-size
 * ring buffer of (time, state, covariance, input) snapshots. When a delayed
 * scalar measurement arrives:
 *   1. Locate the floor snapshot (newest snapshot with time <= meas.time_us).
 *   2. Predict the floor forward to meas.time_us to get (x_meas, P_meas).
 *   3. Compute the Kalman gain K at meas.time_us from (x_meas, P_meas).
 *   4. Gate with NIS; reject if the normalized innovation squared exceeds the threshold.
 *   5. Apply the correction locally to obtain the posterior at meas.time_us.
 *   6. Write the posterior back into the timeline (replace the floor if times match,
 *      otherwise insert a new snapshot). This lets a subsequent delayed measurement
 *      falling in the same interval replay from the corrected state instead of the
 *      stale pre-measurement floor.
 *   7. Propagate K forward with the transition matrix Phi and apply the projected
 *      correction K_i = Phi * K to every later history sample and to the live state.
 *
 * See docs/en/advanced_features/vision_target_estimator_advanced.md
 * (section "OOSM Implementation") for the derivation and approximation
 * assumptions.
 */

#pragma once

#include <px4_platform_common/defines.h>

#include <lib/ringbuffer/TimestampedRingBuffer.hpp>
#include <matrix/SquareMatrix.hpp>
#include <matrix/Vector.hpp>

#include "common.h"

namespace vision_target_estimator
{

// Tag type for filters that do not store control inputs (e.g. the orientation filter).
struct EmptyInput {};

/**
 * @class OOSMManager
 * @brief Maintains the snapshot history and fuses possibly-delayed scalar measurements.
 *
 * @tparam FilterT      Owning filter type. Must expose the methods listed in
 *                      the "FilterT concept" block below.
 * @tparam StateDim     Dimension of the state vector.
 * @tparam InputT       Control input stored per snapshot (e.g. acceleration for
 *                      the position filter, or EmptyInput for orientation).
 * @tparam HistorySize  Capacity of the ring buffer (max 255).
 *
 * FilterT concept (the OOSM manager never touches the filter's internal state
 * directly, it drives it exclusively through this interface):
 *   - void predictState:
 *       Propagate (x, P) over dt using control input u.
 *
 *   - StateCov getTransitionMatrix:
 *       Phi used to project a correction forward (u cancels out in the
 *       correction, so only Phi is needed here).
 *
 *   - void computeInnovation:
 *       Compute innovation y = z - H x and variance S = H P H^T + R.
 *
 *   - void applyCorrection:
 *       Update (x, P) with an already-computed gain vector K.
 *
 * ScalarMeasT concept: must expose `uint64_t time_us` and a vector `H` of
 * size StateDim, plus whatever the filter's computeInnovation expects.
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
		InputT input{}; // Control input applied from the previous snapshot up to this one.
	};

	static_assert(HistorySize > 0, "HistorySize must be > 0");
	static_assert(HistorySize <= 255, "HistorySize must fit in uint8_t");
	static constexpr uint8_t kHistorySize = static_cast<uint8_t>(HistorySize);
	static constexpr uint64_t kOosmMinTimeUs = vision_target_estimator::kOosmMinTimeUs;
	static constexpr uint64_t kOosmMaxTimeUs = vision_target_estimator::kOosmMaxTimeUs;

	/**
	 * @brief Flat oldest-to-newest view of the ring buffer with one extra slot.
	 *
	 * The OOSM path works on this layout rather than on the ring buffer directly so that
	 * the corrected posterior at t_meas can be inserted between two snapshots without
	 * mutating the ring mid-algorithm. The +1 slot guarantees room for that insertion
	 * even when history is already full.
	 *
	 * Invariants:
	 *   - count <= kCapacity (== HistorySize + 1)
	 *   - samples[0..count) are sorted by time_us (strictly increasing)
	 */
	struct LinearTimeline {
		static constexpr size_t kCapacity = static_cast<size_t>(HistorySize) + 1u;
		Sample samples[kCapacity] {};
		size_t count{0};
	};

	OOSMManager() = default;

	void reset()
	{
		if (_history.valid()) {
			_history.reset();
		}
	}

	/**
	 * @brief Add a post-update state snapshot to history.
	 * @param input Control input applied to propagate from the previous snapshot to this one.
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
	 * @brief Fuse a scalar measurement that may be delayed.
	 *
	 * Fresh measurements take the fast path and update the live state directly.
	 * Delayed measurements are fused at their own timestamp and the correction
	 * is projected forward through history and into the live state.
	 *
	 * @param filter        Filter providing prediction / correction primitives.
	 * @param meas          The scalar measurement (must provide `time_us` and `H`).
	 * @param now_us        Current system time.
	 * @param nis_threshold NIS (chi-square) threshold; <= 0 disables gating.
	 * @param curr_state    [In/Out] Live filter state, updated with the projected correction.
	 * @param curr_cov      [In/Out] Live filter covariance.
	 * @param curr_input    Control input currently active (used when the floor sample is the
	 *                      newest in history, since no "next" snapshot exists to describe the
	 *                      input over [t_floor, t_meas]).
	 */
	template <typename ScalarMeasT>
	FusionResult fuse(FilterT &filter, const ScalarMeasT &meas, uint64_t now_us,
			  float nis_threshold, StateVec &curr_state,
			  StateCov &curr_cov, const InputT &curr_input)
	{
		FusionResult res{};

		const uint64_t time_diff = (now_us >= meas.time_us)
					   ? (now_us - meas.time_us)
					   : (meas.time_us - now_us);

		// Fast path: measurement is effectively current, fuse it directly into the live state.
		if (time_diff < kOosmMinTimeUs) {
			StateVec K;

			if (processMeasurement(filter, curr_state, curr_cov, meas, nis_threshold, res, K)) {
				filter.applyCorrection(curr_state, curr_cov, K, res.innov, res.innov_var);
				res.status = FusionStatus::STATUS_FUSED_CURRENT;
			}

			return res;
		}

		// OOSM path: reject anything we cannot safely replay.
		if (!_history.valid() || (_history.entries() == 0)) {
			res.status = FusionStatus::STATUS_REJECT_EMPTY;
			return res;
		}

		const uint64_t newest_time_us = _history.get_newest().time_us;
		const uint64_t oldest_time_us = _history.get_oldest().time_us;

		// Live time moved backwards or jumped too far forward relative to history: wipe and recover.
		if (now_us < newest_time_us || (now_us - newest_time_us) > kOosmMaxTimeUs) {
			reset();
			res.status = FusionStatus::STATUS_REJECT_STALE;
			return res;
		}

		// Measurement time is in the future beyond the small tolerance used for the fast path.
		if (meas.time_us > now_us + kOosmMinTimeUs) {
			res.status = FusionStatus::STATUS_REJECT_TOO_NEW;
			return res;
		}

		// Measurement predates the buffer window: we cannot reconstruct a floor for it.
		if (time_diff > kOosmMaxTimeUs || meas.time_us < oldest_time_us) {
			res.status = FusionStatus::STATUS_REJECT_TOO_OLD;
			return res;
		}

		// Unroll history into a flat oldest->newest view so we can insert the corrected posterior
		// without mutating the ring buffer mid-algorithm.
		LinearTimeline linear_timeline{};
		copyHistoryToLinearTimeline(linear_timeline);

		// Floor: newest snapshot with time <= meas.time_us. Scan from newest backwards so we stop early.
		bool floor_found = false;
		size_t floor_pos = 0;

		for (size_t i = linear_timeline.count; i-- > 0;) {
			if (linear_timeline.samples[i].time_us <= meas.time_us) {
				floor_pos = i;
				floor_found = true;
				break;
			}
		}

		if (!floor_found) {
			res.status = FusionStatus::STATUS_REJECT_TOO_OLD;
			return res;
		}

		const Sample &sample_floor = linear_timeline.samples[floor_pos];
		const uint64_t t_floor_us = sample_floor.time_us;

		// Pick the control input that was active over [t_floor, t_meas]. Each snapshot stores the
		// input that drove the filter *to* that snapshot, so the active input during the interval
		// starting at `floor` is the one attached to the next snapshot. If the floor is already the
		// newest sample, no "next" snapshot exists and we fall back to the live input.
		InputT input_interval = curr_input;

		if (floor_pos + 1 < linear_timeline.count) {
			input_interval = linear_timeline.samples[floor_pos + 1].input;
		}

		const float dt_meas = (meas.time_us - t_floor_us) * kMicrosecondsToSeconds;

		// Predict the floor forward to the exact measurement time. dt_meas == 0 means the floor
		// snapshot is already at meas time, so the copy below is the correct state/cov at t_meas.
		StateVec x_meas = sample_floor.state;
		StateCov P_meas = sample_floor.cov;

		if (dt_meas > 0.f) {
			filter.predictState(dt_meas, input_interval, sample_floor.state, sample_floor.cov, x_meas, P_meas);
		}

		// Compute K at t_meas and gate on NIS / covariance validity.
		StateVec K_meas;

		if (!processMeasurement(filter, x_meas, P_meas, meas, nis_threshold, res, K_meas)) {
			return res; // STATUS_REJECT_NIS or STATUS_REJECT_COV
		}

		// Apply the correction at t_meas locally. We will also write it back into the timeline so
		// that any later delayed measurement falling in the same interval can use this corrected
		// posterior as its floor (otherwise it would replay from the stale pre-measurement sample).
		filter.applyCorrection(x_meas, P_meas, K_meas, res.innov, res.innov_var);

		// Write the corrected posterior into the timeline. If meas lands exactly on the floor
		// timestamp we overwrite it, otherwise we insert a new snapshot right after the floor.
		const size_t corrected_pos =
			insertCorrectedPosteriorInLinearTimeline(linear_timeline, floor_pos, meas.time_us,
					x_meas, P_meas, input_interval);

		// Projected correction: walk forward from t_meas, accumulate Phi across each step, and apply
		// K_i = Phi_cumulative * K_meas to every later snapshot. The control-input term cancels out
		// in the error dynamics, so only Phi is needed here (see docs).
		StateCov Phi_step;
		StateCov Phi_cumulative;
		Phi_cumulative.setIdentity();

		uint64_t prev_time_us = meas.time_us;
		uint8_t steps = 0;

		// Iterate from t_meas up to the newest stored sample.
		for (size_t i = corrected_pos + 1; i < linear_timeline.count; ++i) {
			Sample &sample = linear_timeline.samples[i];

			const float dt = (sample.time_us - prev_time_us) * kMicrosecondsToSeconds;

			// Get transition matrix for the error state dynamics
			Phi_step = filter.getTransitionMatrix(dt);
			Phi_cumulative = Phi_step * Phi_cumulative;

			// Project K: K_k = Phi * K_{k-1}
			const StateVec K_proj = Phi_cumulative * K_meas;

			// Apply correction to history
			filter.applyCorrection(sample.state, sample.cov, K_proj, res.innov, res.innov_var);

			prev_time_us = sample.time_us;
			steps++;
		}

		// Project the remaining interval [newest_history, now] and
		// apply the final correction to the live state.
		if (now_us > prev_time_us) {
			const float dt_now = (now_us - prev_time_us) * kMicrosecondsToSeconds;
			Phi_step = filter.getTransitionMatrix(dt_now);
			Phi_cumulative = Phi_step * Phi_cumulative;
		}

		const StateVec K_final = Phi_cumulative * K_meas;
		filter.applyCorrection(curr_state, curr_cov, K_final, res.innov, res.innov_var);

		// Commit the modified timeline back into the ring buffer so the next OOSM call sees the
		// corrected history. If we inserted a sample and overflowed capacity, the oldest entry is
		// dropped.
		rebuildHistoryFromLinearTimeline(linear_timeline);

		res.status = FusionStatus::STATUS_FUSED_OOSM;
		res.history_steps = steps;

		return res;
	}

private:
	// Test-only access: the companion struct is defined in the unit test translation unit and reaches
	// the private helpers without needing any #ifdef or production-side wrappers.
	template <typename> friend struct OOSMManagerTestAccess;

	// Run NIS / covariance gating and compute the Kalman gain K for the given (state, cov, meas).
	// Returns true iff the measurement is accepted; out_res is populated with innov/innov_var/test_ratio.
	template <typename ScalarMeasT>
	bool processMeasurement(FilterT &filter, const StateVec &state,
				const StateCov &cov, const ScalarMeasT &meas,
				float nis_threshold, FusionResult &out_res,
				StateVec &out_K)
	{
		float innov = 0.f;
		float innov_var = 0.f;

		filter.computeInnovation(state, cov, meas, innov, innov_var);

		out_res.innov = innov;
		out_res.innov_var = innov_var;

		// Reject degenerate or non-finite innovation variance before inverting it.
		if (!PX4_ISFINITE(innov_var) || (innov_var < kMinInnovationVariance)) {
			out_res.status = FusionStatus::STATUS_REJECT_COV;
			out_res.test_ratio = -1.f;
			return false;
		}

		const float beta = (innov * innov) / innov_var;

		if (nis_threshold > 0.f) {
			out_res.test_ratio = beta / nis_threshold;

			if (beta > nis_threshold) {
				out_res.status = FusionStatus::STATUS_REJECT_NIS;
				return false;
			}

		} else {
			out_res.test_ratio = -1.f;
		}

		out_K = (cov * meas.H) / innov_var;
		return true;
	}

	/**
	 * @brief Copy the ring buffer into a flat oldest-to-newest array.
	 *
	 * Walking the ring via _history.next() keeps the traversal canonical and avoids any
	 * manual modular arithmetic on uint8_t indices (which can wrap before the bounds check
	 * when HistorySize approaches 255).
	 *
	 * Post-condition: out.count == _history.entries(), with out.count <= kHistorySize < kCapacity,
	 * so no out-of-bounds writes are possible by construction.
	 */
	void copyHistoryToLinearTimeline(LinearTimeline &out)
	{
		out.count = 0;

		if (!_history.valid()) {
			return;
		}

		const int history_count = _history.entries();

		if (history_count == 0) {
			return;
		}

		uint8_t idx = _history.get_oldest_index();

		for (int i = 0; i < history_count; ++i) {
			out.samples[out.count] = _history[idx];
			out.count++;
			idx = _history.next(idx);
		}
	}

	/**
	 * @brief Insert the corrected posterior at t_meas into the timeline.
	 *
	 * When meas time matches the floor exactly we overwrite the floor's state/cov in place,
	 * otherwise we shift the tail right by one slot and insert a fresh snapshot after the floor.
	 * The +1 capacity slot in LinearTimeline guarantees room for the insertion even when history
	 * was already at capacity.
	 *
	 * @return Position of the corrected posterior in the timeline.
	 */
	size_t insertCorrectedPosteriorInLinearTimeline(LinearTimeline &tl, size_t floor_pos,
			uint64_t meas_time_us,
			const StateVec &x_posterior,
			const StateCov &P_posterior,
			const InputT &input_interval)
	{
		if (tl.samples[floor_pos].time_us == meas_time_us) {
			tl.samples[floor_pos].state = x_posterior;
			tl.samples[floor_pos].cov = P_posterior;
			return floor_pos;
		}

		const size_t corrected_pos = floor_pos + 1;

		// Shift [corrected_pos .. count) one slot to the right to make room.
		// tl.count <= kHistorySize at entry (copy never exceeds it), so writing tl.samples[count]
		// stays within the kCapacity = HistorySize + 1 slots.
		for (size_t i = tl.count; i > corrected_pos; --i) {
			tl.samples[i] = tl.samples[i - 1];
		}

		tl.samples[corrected_pos].time_us = meas_time_us;
		tl.samples[corrected_pos].state = x_posterior;
		tl.samples[corrected_pos].cov = P_posterior;
		tl.samples[corrected_pos].input = input_interval;
		tl.count++;
		return corrected_pos;
	}

	/**
	 * @brief Push the (possibly augmented) timeline back into the ring buffer.
	 *
	 * If an insertion pushed the timeline past ring-buffer capacity, the oldest sample is dropped.
	 */
	void rebuildHistoryFromLinearTimeline(const LinearTimeline &tl)
	{
		_history.reset();

		const size_t start = (tl.count > kHistorySize) ? (tl.count - kHistorySize) : 0;

		for (size_t i = start; i < tl.count; ++i) {
			_history.push(tl.samples[i]);
		}
	}

	TimestampedRingBuffer<Sample, kHistorySize> _history;
};

} // namespace vision_target_estimator

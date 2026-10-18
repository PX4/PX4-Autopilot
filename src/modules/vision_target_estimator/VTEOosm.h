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
 * Keeps post-update snapshots and accepted scalar observations in fixed-size buffers.
 * A delayed observation is fused at its capture time, then subsequent predictions and
 * accepted observations are replayed in time order with recomputed Kalman gains.
 * This preserves the information from newer observations when arrivals cross in time.
 *
 * For a delayed scalar observation:
 *   1. Find the newest snapshot at or before its timestamp.
 *   2. Predict to that time, replaying any accepted observations between the snapshot and it.
 *   3. Compute the innovation and gain, and reject the new observation if it fails the NIS gate.
 *   4. Apply the ordinary Kalman correction at the observation time.
 *   5. Replay later predictions and accepted observations, recomputing gains without re-gating.
 *   6. Commit the updated snapshots and live state only when replay succeeds. Retain the new
 *      observation for future replay; no extra prediction snapshot is needed.
 *
 * See docs/en/advanced_features/vision_target_estimator_advanced.md
 * (section "OOSM Implementation") for the replay policy and storage bounds.
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
 * @tparam MaxObsPerSnapshot  Most scalar observations fused between two snapshots, one per aid
 *                      source and cycle (up to four for the position filter). Sizes the log.
 *
 * FilterT concept (the OOSM manager never touches the filter's internal state
 * directly, it drives it exclusively through this interface):
 *   - void predictState:
 *       Propagate (x, P) over dt using control input u.
 *
 *   - void computeInnovation:
 *       Compute innovation y = z - H x and variance S = H P H^T + R.
 *
 *   - void applyCorrection:
 *       Update (x, P) with an already-computed gain vector K.
 *
 * ScalarMeasT concept: default-constructible with members time_us, val, unc and H.
 */
template <typename FilterT, int StateDim, typename InputT, int HistorySize = 25, int MaxObsPerSnapshot = 4>
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
	static_assert(MaxObsPerSnapshot > 0, "MaxObsPerSnapshot must be > 0");
	static constexpr uint8_t kHistorySize = static_cast<uint8_t>(HistorySize);

	// Retained observations were fused within the last HistorySize cycles, so the log
	// only fills if the filter fuses more than MaxObsPerSnapshot per cycle.
	static constexpr size_t kMeasurementCapacity = static_cast<size_t>(MaxObsPerSnapshot) * HistorySize;
	static constexpr uint64_t kOosmMinTimeUs = vision_target_estimator::kOosmMinTimeUs;
	static constexpr uint64_t kOosmMaxTimeUs = vision_target_estimator::kOosmMaxTimeUs;

	/**
	 * @brief Copy of the snapshots used to replay without changing history until fusion succeeds.
	 *
	 * Invariants:
	 *   - count <= kCapacity (== HistorySize)
	 *   - samples[0..count) are sorted by time_us (strictly increasing)
	 */
	struct LinearTimeline {
		static constexpr size_t kCapacity = static_cast<size_t>(HistorySize);
		Sample samples[kCapacity] {};
		size_t count{0};
	};

	OOSMManager() = default;

	void reset()
	{
		_measurement_count = 0;

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
		pruneMeasurements();
	}

	/**
	 * @brief Fuse a scalar measurement that may be delayed.
	 *
	 * Fresh measurements take the fast path and update the live state directly.
	 * Delayed measurements are fused at their own timestamp, then subsequent accepted
	 * observations are replayed through history and into the live state.
	 *
	 * @param filter        Filter providing prediction / correction primitives.
	 * @param meas          The scalar measurement (see the ScalarMeasT concept).
	 * @param now_us        Current system time.
	 * @param nis_threshold NIS (chi-square) threshold; <= 0 disables gating.
	 * @param curr_state    [In/Out] Live filter state, updated by chronological replay.
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
				// The fast path treats near-current samples as observations at now.
				rememberMeasurement(meas, now_us, now_us, curr_state, curr_cov, curr_input);
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
		if (meas.time_us >= now_us + kOosmMinTimeUs) {
			res.status = FusionStatus::STATUS_REJECT_TOO_NEW;
			return res;
		}

		// Measurement predates the buffer window: we cannot reconstruct a floor for it.
		if (time_diff > kOosmMaxTimeUs || meas.time_us < oldest_time_us) {
			res.status = FusionStatus::STATUS_REJECT_TOO_OLD;
			return res;
		}

		// Replay into a copy so a rejected measurement leaves history unchanged.
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

		Sample &sample_floor = linear_timeline.samples[floor_pos];
		const uint64_t t_floor_us = sample_floor.time_us;

		// Pick the control input that was active over [t_floor, t_meas]. Each snapshot stores the
		// input that drove the filter *to* that snapshot, so the active input during the interval
		// starting at `floor` is the one attached to the next snapshot. If the floor is already the
		// newest sample, no "next" snapshot exists and we fall back to the live input.
		InputT input_interval = curr_input;

		if (floor_pos + 1 < linear_timeline.count) {
			input_interval = linear_timeline.samples[floor_pos + 1].input;
		}

		StateVec x_meas = sample_floor.state;
		StateCov P_meas = sample_floor.cov;
		uint64_t replay_time = t_floor_us;
		size_t measurement_pos = 0;

		if (!replayUntil<ScalarMeasT>(filter, meas.time_us, input_interval, replay_time, measurement_pos, x_meas, P_meas)) {
			res.status = FusionStatus::STATUS_REJECT_COV;
			return res;
		}

		// Compute K at t_meas and gate on NIS / covariance validity.
		StateVec K_meas;

		if (!processMeasurement(filter, x_meas, P_meas, meas, nis_threshold, res, K_meas)) {
			return res; // STATUS_REJECT_NIS or STATUS_REJECT_COV
		}

		filter.applyCorrection(x_meas, P_meas, K_meas, res.innov, res.innov_var);

		// Update an existing snapshot at this time. Between snapshots, the observation log
		// retains this correction for future replays without consuming a history slot.
		if (meas.time_us == t_floor_us) {
			sample_floor.state = x_meas;
			sample_floor.cov = P_meas;
		}

		// Recompute each later posterior instead of subtracting a historical covariance
		// correction from a posterior already reduced by a newer measurement.
		uint8_t steps = 0;

		for (size_t i = floor_pos + 1; i < linear_timeline.count; ++i) {
			Sample &sample = linear_timeline.samples[i];

			if (!replayUntil<ScalarMeasT>(filter, sample.time_us, sample.input, replay_time, measurement_pos, x_meas, P_meas)) {
				res.status = FusionStatus::STATUS_REJECT_COV;
				return res;
			}

			sample.state = x_meas;
			sample.cov = P_meas;
			steps++;
		}

		if (!replayUntil<ScalarMeasT>(filter, now_us, curr_input, replay_time, measurement_pos, x_meas, P_meas)) {
			res.status = FusionStatus::STATUS_REJECT_COV;
			return res;
		}

		curr_state = x_meas;
		curr_cov = P_meas;
		rebuildHistoryFromLinearTimeline(linear_timeline);
		rememberMeasurement(meas, meas.time_us, now_us, curr_state, curr_cov, curr_input);

		res.status = FusionStatus::STATUS_FUSED_OOSM;
		res.history_steps = steps;

		return res;
	}

private:
	// Test-only access: the companion struct is defined in the unit test translation unit and reaches
	// the private helpers without needing any #ifdef or production-side wrappers.
	template <typename> friend struct OOSMManagerTestAccess;

	struct StoredMeasurement {
		uint64_t time_us;
		float val;
		float unc;
		StateVec H;
	};
	StoredMeasurement _measurements[kMeasurementCapacity] {};
	size_t _measurement_count{0};

	void pruneMeasurements()
	{
		if (_history.entries() == 0) {
			return;
		}

		size_t obsolete = 0;

		while (obsolete < _measurement_count && _measurements[obsolete].time_us <= _history.get_oldest().time_us) {
			++obsolete;
		}

		if (obsolete == 0) {
			return;
		}

		for (size_t i = obsolete; i < _measurement_count; ++i) {
			_measurements[i - obsolete] = _measurements[i];
		}

		_measurement_count -= obsolete;
	}

	template <typename ScalarMeasT>
	void rememberMeasurement(const ScalarMeasT &meas, uint64_t effective_time, uint64_t now_us,
				 const StateVec &state, const StateCov &cov, const InputT &input)
	{
		pruneMeasurements();

		if (_measurement_count == kMeasurementCapacity) {
			// Restart from the live posterior rather than drop an update a later replay needs.
			reset();
			push(now_us, state, cov, input);
			return;
		}

		size_t pos = _measurement_count;

		while (pos > 0 && _measurements[pos - 1].time_us > effective_time) {
			_measurements[pos] = _measurements[pos - 1];
			--pos;
		}

		_measurements[pos] = {effective_time, meas.val, meas.unc, meas.H};
		++_measurement_count;
	}

	// The starting posterior already contains all measurements at or before time_us.
	// Previously accepted observations stay accepted: recompute their gains, without
	// retrospectively changing the gate decisions reported when they arrived.
	template <typename ScalarMeasT>
	bool replayUntil(FilterT &filter, uint64_t end_time, const InputT &input,
			 uint64_t &time_us, size_t &measurement_pos, StateVec &state, StateCov &cov)
	{
		const uint64_t start_time = time_us;

		while (measurement_pos < _measurement_count) {
			const StoredMeasurement &stored = _measurements[measurement_pos];

			if (stored.time_us <= start_time) {
				++measurement_pos;
				continue;
			}

			if (stored.time_us > end_time) {
				break;
			}

			predictTo(filter, stored.time_us, input, time_us, state, cov);
			ScalarMeasT meas{};
			meas.time_us = stored.time_us;
			meas.val = stored.val;
			meas.unc = stored.unc;
			meas.H = stored.H;
			StateVec K;
			FusionResult result{};

			if (!processMeasurement(filter, state, cov, meas, 0.f, result, K)) {
				return false;
			}

			filter.applyCorrection(state, cov, K, result.innov, result.innov_var);
			++measurement_pos;
		}

		predictTo(filter, end_time, input, time_us, state, cov);
		return true;
	}

	void predictTo(FilterT &filter, uint64_t end_time, const InputT &input,
		       uint64_t &time_us, StateVec &state, StateCov &cov)
	{
		if (end_time > time_us) {
			StateVec next_state;
			StateCov next_cov;
			filter.predictState((end_time - time_us) * kMicrosecondsToSeconds, input,
					    state, cov, next_state, next_cov);
			state = next_state;
			cov = next_cov;
			time_us = end_time;
		}
	}

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
	 * Post-condition: out.count == _history.entries(), with out.count <= kCapacity,
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

	// Replace history only after the entire replay succeeds.
	void rebuildHistoryFromLinearTimeline(const LinearTimeline &tl)
	{
		_history.reset();

		for (size_t i = 0; i < tl.count; ++i) {
			_history.push(tl.samples[i]);
		}
	}

	TimestampedRingBuffer<Sample, kHistorySize> _history;
};

} // namespace vision_target_estimator

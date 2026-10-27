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
 * Keeps post-update snapshots and gated scalar observations in fixed-size buffers.
 * A delayed observation is fused at its capture time, then subsequent predictions and
 * stored observations are replayed in time order with recomputed Kalman gains.
 * This preserves the information from newer observations when arrivals cross in time.
 *
 * For a delayed scalar observation:
 *   1. Find the newest snapshot at or before its timestamp.
 *   2. Predict to that time, replaying the stored observations between the snapshot and it.
 *   3. Compute the innovation and gain, and reject the new observation if it fails the NIS gate.
 *   4. Apply the ordinary Kalman correction at the observation time.
 *   5. Replay later predictions and stored observations, recomputing gains and gating each one
 *      again, so a stored observation can change from accepted to rejected or back.
 *   6. Rewrite the later snapshots in place and replace the live state. Retain the new
 *      observation, accepted or rejected, for future replays; no extra prediction snapshot is
 *      needed. If a stored observation fails during replay, clear the history so it cannot
 *      block later replays.
 *
 * Delayed observations from the same cycle share one replay: it stops at each of them in turn,
 * oldest first, so the window is replayed once per cycle rather than once per observation.
 *
 * See docs/en/advanced_features/vision_target_estimator_advanced.md
 * (section "OOSM Implementation") for the replay policy and storage bounds.
 */

#pragma once

#include <cmath>

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
 * @tparam MaxStoredObs Capacity of the observation log, which also bounds the stored observations
 *                      one replay gates again. When full, the oldest observation is dropped.
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
 * ScalarMeasT concept: default-constructible with members time_us, val, unc, H and source.
 */
template <typename FilterT, int StateDim, typename InputT, int HistorySize = 25, int MaxStoredObs = 4 * HistorySize>
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
	static_assert(MaxStoredObs > 0, "MaxStoredObs must be > 0");
	static constexpr uint8_t kHistorySize = static_cast<uint8_t>(HistorySize);
	static constexpr size_t kMeasurementCapacity = static_cast<size_t>(MaxStoredObs);
	static constexpr uint64_t kOosmMinTimeUs = vision_target_estimator::kOosmMinTimeUs;
	static constexpr uint64_t kOosmMaxTimeUs = vision_target_estimator::kOosmMaxTimeUs;
	static_assert(kOosmMinTimeUs <= INT16_MAX, "Fast-path capture-time offset must fit in int16_t");
	// Most measurements fused in one pass; a larger batch is fused in chunks of this size.
	static constexpr size_t kMaxBatchSize = 8;

	OOSMManager() = default;

	FusionTimes getFusionTimes() const { return _fusion_times; }

	// A state reset clears freshness by default. History-only recovery passes the committed times.
	void reset(FusionTimes times = {})
	{
		_measurement_count = 0;
		_min_floor_time_us = 0;
		_baseline_times = times;
		_fusion_times = times;
		_history.reset();
	}

	/**
	 * @brief Add a post-update state snapshot to history.
	 * @param input Control input applied to propagate from the previous snapshot to this one.
	 */
	void push(uint64_t time_us, const StateVec &state, const StateCov &cov, const InputT &input)
	{
		Sample sample{};
		sample.time_us = time_us;
		sample.state = state;
		sample.cov = cov;
		sample.input = input;

		_history.push(sample);
		pruneMeasurements();
	}

	/**
	 * @brief Fuse a scalar measurement that may be delayed (a batch of one, see fuseBatch()).
	 */
	template <typename ScalarMeasT>
	FusionResult fuse(FilterT &filter, const ScalarMeasT &meas, uint64_t now_us,
			  float nis_threshold, StateVec &curr_state,
			  StateCov &curr_cov, const InputT &curr_input)
	{
		FusionResult res{};
		fuseBatch(filter, &meas, 1, now_us, nis_threshold, curr_state, curr_cov, curr_input, &res);
		return res;
	}

	/**
	 * @brief Fuse the scalar measurements of one cycle, any of which may be delayed.
	 *
	 * Delayed measurements are fused at their own timestamps, oldest first, in a single replay
	 * through history and into the live state. Fresh measurements then take the fast path and
	 * update the live state directly. With successful replay and sufficient observation-log capacity,
	 * the final state and covariance match individual fusions in that order, up to float rounding.
	 * Replay diagnostics use the shared trajectory, including later accepted batch measurements.
	 * A stored-observation failure rejects the whole delayed batch and clears history without
	 * changing the live state; fresh measurements can still fuse afterwards.
	 *
	 * Batches larger than kMaxBatchSize are processed in input-order chunks. Timestamp ordering
	 * and delayed-before-fresh ordering apply within each chunk, not across the whole input.
	 *
	 * @param filter        Filter providing prediction / correction primitives.
	 * @param meas          The scalar measurements (see the ScalarMeasT concept).
	 * @param count         Number of measurements.
	 * @param now_us        Current system time.
	 * @param nis_threshold NIS (chi-square) threshold; <= 0 disables gating.
	 * @param curr_state    [In/Out] Live filter state, updated by chronological replay.
	 * @param curr_cov      [In/Out] Live filter covariance.
	 * @param curr_input    Control input currently active (used after the newest snapshot,
	 *                      since no "next" snapshot exists to describe the input there).
	 * @param results       [Out] One result per measurement, in input order.
	 */
	template <typename ScalarMeasT>
	void fuseBatch(FilterT &filter, const ScalarMeasT *meas, size_t count, uint64_t now_us, float nis_threshold,
		       StateVec &curr_state, StateCov &curr_cov, const InputT &curr_input, FusionResult *results)
	{
		for (size_t first = 0; first < count; first += kMaxBatchSize) {
			const size_t chunk = ((count - first) < kMaxBatchSize) ? (count - first) : kMaxBatchSize;
			fuseChunk(filter, &meas[first], chunk, now_us, nis_threshold, curr_state, curr_cov, curr_input, &results[first]);
		}
	}

private:
	// Test-only access: the companion struct is defined in the unit test translation unit and reaches
	// the private helpers without needing any #ifdef or production-side wrappers.
	template <typename> friend struct OOSMManagerTestAccess;

	template <typename ScalarMeasT>
	void fuseChunk(FilterT &filter, const ScalarMeasT *meas, size_t count, uint64_t now_us, float nis_threshold,
		       StateVec &curr_state, StateCov &curr_cov, const InputT &curr_input, FusionResult *results)
	{
		// Delayed measurements in time order, keeping the input order for equal times.
		uint8_t order[kMaxBatchSize];
		size_t delayed_count = 0;

		for (size_t i = 0; i < count; ++i) {
			results[i] = FusionResult{};

			if (isCurrent(meas[i].time_us, now_us)) {
				continue;
			}

			size_t j = delayed_count++;

			while ((j > 0) && (meas[order[j - 1]].time_us > meas[i].time_us)) {
				order[j] = order[j - 1];
				--j;
			}

			order[j] = static_cast<uint8_t>(i);
		}

		// Keep the delayed measurements that history can replay.
		size_t replay_count = 0;

		for (size_t k = 0; k < delayed_count; ++k) {
			if (checkDelayed(meas[order[k]], now_us, results[order[k]])) {
				order[replay_count++] = order[k];
			}
		}

		if (replay_count > 0) {
			replayBatch(filter, meas, order, replay_count, now_us, nis_threshold, curr_state, curr_cov, curr_input,
				    results);
		}

		for (size_t i = 0; i < count; ++i) {
			if (isCurrent(meas[i].time_us, now_us)) {
				fuseCurrent(filter, meas[i], now_us, nis_threshold, curr_state, curr_cov, curr_input, results[i]);
			}
		}
	}

	struct StoredMeasurement {
		uint64_t time_us;
		float val;
		float unc;
		StateVec H;
		FusionSource source;
		bool accepted; // Latest gate decision
		// Only the fast path retimes observations: the offset is strictly less than 20 ms.
		int16_t capture_offset_us;

		uint64_t captureTime() const
		{
			return (capture_offset_us >= 0) ? time_us + static_cast<uint64_t>(capture_offset_us)
			       : time_us - static_cast<uint64_t>(-capture_offset_us);
		}
	};
	StoredMeasurement _measurements[kMeasurementCapacity] {};
	size_t _measurement_count{0};
	uint64_t _min_floor_time_us{0}; // Replays start at or after it: older snapshots lack a dropped observation
	FusionTimes _baseline_times{}; // Accepted observations already incorporated in the oldest usable snapshot.
	FusionTimes _fusion_times{}; // Committed live posterior, unchanged while replay is speculative.

	// Oldest snapshot a replay can start from.
	uint64_t oldestFloorTime()
	{
		const uint8_t newest_idx = _history.get_newest_index();
		uint8_t idx = _history.get_oldest_index();

		while ((idx != newest_idx) && (_history[idx].time_us < _min_floor_time_us)) {
			idx = _history.next(idx);
		}

		return _history[idx].time_us;
	}

	void pruneMeasurements()
	{
		if (_history.entries() == 0) {
			return;
		}

		// Observations at or before the oldest usable snapshot are contained in it.
		const uint64_t floor_time = oldestFloorTime();
		size_t obsolete = 0;

		while (obsolete < _measurement_count && _measurements[obsolete].time_us <= floor_time) {
			const StoredMeasurement &stored = _measurements[obsolete];

			if (stored.accepted) {
				_baseline_times.record(stored.source, stored.captureTime());
			}

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
				 const StateVec &state, const StateCov &cov, const InputT &input, bool accepted)
	{
		pruneMeasurements();

		if (_measurement_count == kMeasurementCapacity) {
			if ((_history.entries() > 0) && (_measurements[0].time_us <= _history.get_newest().time_us)) {
				// Drop the oldest observation. Replays then start from snapshots that contain it.
				_min_floor_time_us = _measurements[0].time_us;
				pruneMeasurements();

			} else {
				// No snapshot contains it yet: restart from the live posterior instead.
				reset(_fusion_times);
				push(now_us, state, cov, input);
				return;
			}
		}

		size_t pos = _measurement_count;

		while (pos > 0 && _measurements[pos - 1].time_us > effective_time) {
			_measurements[pos] = _measurements[pos - 1];
			--pos;
		}

		_measurements[pos] = {effective_time, meas.val, meas.unc, meas.H, meas.source, accepted,
				      static_cast<int16_t>(signedTimeDiffUs(meas.time_us, effective_time))
				     };
		++_measurement_count;
	}

	// Replay position: posterior at time_us, with stored observations before measurement_pos replayed
	// or in the starting snapshot. changed marks a replay that differs from history and must be committed.
	struct ReplayCursor {
		StateVec state;
		StateCov cov;
		uint64_t time_us;
		size_t measurement_pos;
		bool changed;
	};

	// Fast path: the measurement is effectively current.
	static bool isCurrent(uint64_t time_us, uint64_t now_us)
	{
		const uint64_t time_diff = (now_us >= time_us) ? (now_us - time_us) : (time_us - now_us);
		return time_diff < kOosmMinTimeUs;
	}

	// Reject a delayed measurement that history cannot replay.
	template <typename ScalarMeasT>
	bool checkDelayed(const ScalarMeasT &meas, uint64_t now_us, FusionResult &res)
	{
		if (_history.entries() == 0) {
			res.status = FusionStatus::STATUS_REJECT_EMPTY;
			return false;
		}

		const uint64_t newest_time_us = _history.get_newest().time_us;

		// Live time moved backwards or jumped too far forward relative to history: wipe and recover.
		if (now_us < newest_time_us || (now_us - newest_time_us) > kOosmMaxTimeUs) {
			reset(_fusion_times);
			res.status = FusionStatus::STATUS_REJECT_STALE;
			return false;
		}

		// Measurement time is in the future beyond the small tolerance used for the fast path.
		if (meas.time_us >= now_us + kOosmMinTimeUs) {
			res.status = FusionStatus::STATUS_REJECT_TOO_NEW;
			return false;
		}

		// Measurement predates the replay window: we cannot reconstruct a floor for it.
		if ((now_us - meas.time_us) > kOosmMaxTimeUs || meas.time_us < oldestFloorTime()) {
			res.status = FusionStatus::STATUS_REJECT_TOO_OLD;
			return false;
		}

		return true;
	}

	template <typename ScalarMeasT>
	void fuseCurrent(FilterT &filter, const ScalarMeasT &meas, uint64_t now_us, float nis_threshold,
			 StateVec &curr_state, StateCov &curr_cov, const InputT &curr_input, FusionResult &res)
	{
		StateVec K;

		// The fast path treats near-current samples as observations at now. Rejected ones are kept
		// too, so a later replay can gate them again.
		if (processMeasurement(filter, curr_state, curr_cov, meas, nis_threshold, res, K)) {
			filter.applyCorrection(curr_state, curr_cov, K, res.innov, res.innov_var);
			res.status = FusionStatus::STATUS_FUSED_CURRENT;
			_fusion_times.record(meas.source, meas.time_us);
			rememberMeasurement(meas, now_us, now_us, curr_state, curr_cov, curr_input, true);

		} else if (res.status == FusionStatus::STATUS_REJECT_NIS) {
			rememberMeasurement(meas, now_us, now_us, curr_state, curr_cov, curr_input, false);
		}
	}

	// Newest snapshot at or before time_us. checkDelayed() ensures the oldest usable one qualifies.
	uint8_t floorIndex(uint64_t time_us)
	{
		const uint8_t oldest_idx = _history.get_oldest_index();
		uint8_t idx = _history.get_newest_index();

		while ((idx != oldest_idx) && (_history[idx].time_us > time_us)) {
			idx = _history.prev(idx);
		}

		return idx;
	}

	// Each snapshot stores the input that drove the filter *to* it, so the input active after a
	// snapshot is the one attached to the next. After the newest snapshot, the live input applies.
	InputT inputAfter(uint8_t idx, const InputT &curr_input)
	{
		return (idx != _history.get_newest_index()) ? _history[_history.next(idx)].input : curr_input;
	}

	// Fuse the sorted delayed measurements in one replay. Until something changes (an accepted measurement
	// or a revised decision), each starts from its own floor; after that, history is rewritten in place.
	template <typename ScalarMeasT>
	void replayBatch(FilterT &filter, const ScalarMeasT *meas, const uint8_t *order, size_t count, uint64_t now_us,
			 float nis_threshold, StateVec &curr_state, StateCov &curr_cov, const InputT &curr_input,
			 FusionResult *results)
	{
		const uint8_t newest_idx = _history.get_newest_index();
		ReplayCursor cursor{};
		uint8_t idx = 0;
		size_t k = 0;
		bool ok = true;

		while (ok && ((k < count) || (cursor.changed && (idx != newest_idx)))) {
			// Recompute later posteriors rather than subtract an old correction from a newer one.
			// A snapshot at a measurement's time is rewritten after that measurement.
			if (cursor.changed && (idx != newest_idx)
			    && ((k == count) || (_history[_history.next(idx)].time_us < meas[order[k]].time_us))) {
				idx = _history.next(idx);
				Sample &sample = _history[idx];
				ok = replayStored<ScalarMeasT>(filter, sample.time_us, sample.input, nis_threshold, cursor, order, k, results);

				if (!ok) {
					break;
				}

				predictTo(filter, sample.time_us, sample.input, cursor);
				sample.state = cursor.state;
				sample.cov = cursor.cov;

				for (size_t j = 0; j < k; ++j) {
					FusionResult &res = results[order[j]];

					if ((res.status == FusionStatus::STATUS_FUSED_OOSM) && (meas[order[j]].time_us < sample.time_us)) {
						++res.history_steps;
					}
				}

				continue;
			}

			if (!cursor.changed) {
				idx = floorIndex(meas[order[k]].time_us);
				cursor = {_history[idx].state, _history[idx].cov, _history[idx].time_us, 0, false};
			}

			ok = fuseDelayed(filter, meas, order, k, inputAfter(idx, curr_input), nis_threshold, idx, cursor, results);

			if (ok) {
				++k;
			}
		}

		if (ok && cursor.changed) {
			ok = replayStored<ScalarMeasT>(filter, now_us, curr_input, nis_threshold, cursor, order, count, results);
		}

		if (!ok) {
			failBatch(order, count, k, results);
			return;
		}

		if (cursor.changed) {
			predictTo(filter, now_us, curr_input, cursor);
			curr_state = cursor.state;
			curr_cov = cursor.cov;
		}

		// Rebuild freshness from the decisions that survived replay, including rejected-to-accepted
		// transitions. Capture times are distinct from the fast path's effective fusion time.
		_fusion_times = _baseline_times;

		for (size_t j = 0; j < _measurement_count; ++j) {
			const StoredMeasurement &stored = _measurements[j];

			if (stored.accepted) {
				_fusion_times.record(stored.source, stored.captureTime());
			}
		}

		// Commit all new timestamps before insertion can restart a full observation log.
		for (size_t j = 0; j < count; ++j) {
			if (results[order[j]].status == FusionStatus::STATUS_FUSED_OOSM) {
				_fusion_times.record(meas[order[j]].source, meas[order[j]].time_us);
			}
		}

		for (size_t j = 0; j < count; ++j) {
			const uint8_t status = results[order[j]].status;

			if ((status == FusionStatus::STATUS_FUSED_OOSM) || (status == FusionStatus::STATUS_REJECT_NIS)) {
				rememberMeasurement(meas[order[j]], meas[order[j]].time_us, now_us, curr_state, curr_cov, curr_input,
						    status == FusionStatus::STATUS_FUSED_OOSM);
			}
		}
	}

	// Fuse order[k] at its time, after snapshot idx. A rejected measurement adds nothing: the cursor only
	// moves to its time. Returns false if a stored observation fails on the way.
	template <typename ScalarMeasT>
	bool fuseDelayed(FilterT &filter, const ScalarMeasT *meas, const uint8_t *order, size_t k, const InputT &input,
			 float nis_threshold, uint8_t idx, ReplayCursor &cursor, FusionResult *results)
	{
		const ScalarMeasT &delayed = meas[order[k]];
		FusionResult &res = results[order[k]];

		if (!replayStored<ScalarMeasT>(filter, delayed.time_us, input, nis_threshold, cursor, order, k, results)) {
			return false;
		}

		predictTo(filter, delayed.time_us, input, cursor);
		StateVec K;

		if (processMeasurement(filter, cursor.state, cursor.cov, delayed, nis_threshold, res, K)) {
			filter.applyCorrection(cursor.state, cursor.cov, K, res.innov, res.innov_var);
			res.status = FusionStatus::STATUS_FUSED_OOSM;
			cursor.changed = true;

			// Update an existing snapshot at this time. Between snapshots, the observation log
			// retains this correction for future replays without consuming a history slot.
			if (_history[idx].time_us == delayed.time_us) {
				_history[idx].state = cursor.state;
				_history[idx].cov = cursor.cov;
			}
		}

		return true;
	}

	// A stored observation failed during replay: clear history so it cannot block later replays.
	// Delayed measurements not already rejected on their own report the failure.
	void failBatch(const uint8_t *order, size_t count, size_t processed, FusionResult *results)
	{
		reset(_fusion_times);

		for (size_t j = 0; j < count; ++j) {
			FusionResult &res = results[order[j]];

			if ((j >= processed) || (res.status == FusionStatus::STATUS_FUSED_OOSM)) {
				res.status = FusionStatus::STATUS_REJECT_COV;
				res.history_steps = 0;
				res.replayed_count = 0;
				res.replay_revised_count = 0;
				res.replay_test_ratio_max = -1.f;
			}
		}
	}

	// Gate and apply the stored observations up to end_time, without the final prediction. A revised
	// decision marks the replay as changed; each counts for the fused delayed order[0..fused_count).
	template <typename ScalarMeasT>
	bool replayStored(FilterT &filter, uint64_t end_time, const InputT &input, float nis_threshold,
			  ReplayCursor &cursor, const uint8_t *order, size_t fused_count, FusionResult *results)
	{
		const uint64_t start_time = cursor.time_us;

		while (cursor.measurement_pos < _measurement_count) {
			StoredMeasurement &stored = _measurements[cursor.measurement_pos];

			if (stored.time_us <= start_time) {
				++cursor.measurement_pos;
				continue;
			}

			if (stored.time_us > end_time) {
				break;
			}

			predictTo(filter, stored.time_us, input, cursor);
			ScalarMeasT meas{};
			meas.time_us = stored.time_us;
			meas.val = stored.val;
			meas.unc = stored.unc;
			meas.H = stored.H;
			StateVec K;
			FusionResult result{};
			const bool accepted = processMeasurement(filter, cursor.state, cursor.cov, meas, nis_threshold, result, K);

			if (!accepted && (result.status != FusionStatus::STATUS_REJECT_NIS)) {
				return false;
			}

			const bool revised = (accepted != stored.accepted);

			for (size_t j = 0; j < fused_count; ++j) {
				FusionResult &res = results[order[j]];

				if (res.status == FusionStatus::STATUS_FUSED_OOSM) {
					if (res.replayed_count < UINT8_MAX) {
						++res.replayed_count;
					}

					if (revised && (res.replay_revised_count < UINT8_MAX)) {
						++res.replay_revised_count;
					}

					// Only previously accepted observations: above 1 means one was rejected now.
					if (stored.accepted) {
						res.replay_test_ratio_max = fmaxf(res.replay_test_ratio_max, result.test_ratio);
					}
				}
			}

			if (revised) {
				stored.accepted = accepted;
				cursor.changed = true;
			}

			if (accepted) {
				filter.applyCorrection(cursor.state, cursor.cov, K, result.innov, result.innov_var);
			}

			++cursor.measurement_pos;
		}

		return true;
	}

	void predictTo(FilterT &filter, uint64_t end_time, const InputT &input, ReplayCursor &cursor)
	{
		if (end_time > cursor.time_us) {
			StateVec next_state;
			StateCov next_cov;
			filter.predictState((end_time - cursor.time_us) * kMicrosecondsToSeconds, input,
					    cursor.state, cursor.cov, next_state, next_cov);
			cursor.state = next_state;
			cursor.cov = next_cov;
			cursor.time_us = end_time;
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

	TimestampedRingBuffer<Sample, kHistorySize> _history;
};

} // namespace vision_target_estimator

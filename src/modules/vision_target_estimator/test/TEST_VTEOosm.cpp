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
 * @file TEST_VTEOosm.cpp
 * @brief Unit test VTEOosm.h
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "VTEOosm.h"
#include "common.h"

namespace vte = vision_target_estimator;

namespace vision_target_estimator
{

// Access into OOSMManager's private helpers for unit tests.
template <typename Manager>
struct OOSMManagerTestAccess {
	// The i-th snapshot, counting from the oldest.
	static const typename Manager::Sample &historySample(Manager &m, int i)
	{
		uint8_t idx = m._history.get_oldest_index();

		for (int k = 0; k < i; ++k) {
			idx = m._history.next(idx);
		}

		return m._history[idx];
	}

	static int historyEntries(const Manager &m) { return m._history.entries(); }
	static uint64_t historyOldestTime(const Manager &m) { return m._history.get_oldest().time_us; }
	static uint64_t historyNewestTime(const Manager &m) { return m._history.get_newest().time_us; }
};

} // namespace vision_target_estimator

namespace
{
using namespace time_literals;

static constexpr int kStateDim = 1;
static constexpr float kTolerance = 1e-4f;
static constexpr int kStateDim2 = 2;
using StateVec2 = matrix::Vector<float, kStateDim2>;
using StateCov2 = matrix::SquareMatrix<float, kStateDim2>;

struct ScalarMeas {
	uint64_t time_us{0};
	float val{0.f};
	float unc{0.f};
	matrix::Vector<float, kStateDim> H{};
	vte::FusionSource source{vte::FusionSource::Primary};
};

struct ScalarMeas2 {
	uint64_t time_us{0};
	float val{0.f};
	float unc{0.f};
	StateVec2 H{};
	vte::FusionSource source{vte::FusionSource::Primary};
};

class TestFilter
{
public:
	float last_predict_input{NAN};
	int predict_count{0};
	uint64_t fail_time{0}; // Observations at this time get an invalid innovation variance.
	matrix::Vector<float, kStateDim> last_H{};
	std::vector<float> applied_k;
	std::vector<float> applied_state_before;
	std::vector<float> applied_state_after;

	void resetTracking()
	{
		last_predict_input = NAN;
		predict_count = 0;
		fail_time = 0;
		last_H.setZero();
		applied_k.clear();
		applied_state_before.clear();
		applied_state_after.clear();
	}

	// Scalar constant-velocity model with small process-noise growth.
	void predictState(float dt, float input, const matrix::Vector<float, kStateDim> &x_in,
			  const matrix::SquareMatrix<float, kStateDim> &P_in,
			  matrix::Vector<float, kStateDim> &x_out,
			  matrix::SquareMatrix<float, kStateDim> &P_out)
	{
		last_predict_input = input;
		++predict_count;

		x_out = x_in;
		x_out(0) += input * dt;

		P_out = P_in;
		P_out(0, 0) += 0.1f * dt;
	}

	void computeInnovation(const matrix::Vector<float, kStateDim> &state,
			       const matrix::SquareMatrix<float, kStateDim> &cov,
			       const ScalarMeas &meas, float &innov, float &innov_var)
	{
		last_H = meas.H;
		innov = meas.val - (meas.H.transpose() * state)(0, 0);
		innov_var = (meas.time_us == fail_time) ? NAN : (meas.H.transpose() * cov * meas.H)(0, 0) + meas.unc;
	}

	void applyCorrection(matrix::Vector<float, kStateDim> &state,
			     matrix::SquareMatrix<float, kStateDim> &cov,
			     const matrix::Vector<float, kStateDim> &K, float innov, float innov_var)
	{
		(void)innov_var;
		applied_k.push_back(K(0));
		applied_state_before.push_back(state(0));
		state(0) += K(0) * innov;
		applied_state_after.push_back(state(0));
		cov(0, 0) = (1.f - K(0) * last_H(0)) * cov(0, 0);
	}
};

class TestFilter2d
{
public:
	float last_predict_input{NAN};
	StateVec2 last_H{};
	std::vector<StateVec2> applied_k;
	std::vector<StateVec2> applied_state_before;
	std::vector<StateVec2> applied_state_after;

	void resetTracking()
	{
		last_predict_input = NAN;
		applied_k.clear();
		applied_state_before.clear();
		applied_state_after.clear();
	}

	void predictState(float dt, float input, const StateVec2 &x_in,
			  const StateCov2 &P_in,
			  StateVec2 &x_out, StateCov2 &P_out)
	{
		last_predict_input = input;

		StateCov2 phi{};
		phi.setIdentity();
		phi(0, 1) = dt;
		StateVec2 control{};
		control(0) = 0.5f * input * dt * dt;
		control(1) = input * dt;

		x_out = phi * x_in + control;
		P_out = phi * P_in * phi.transpose();
	}

	void computeInnovation(const StateVec2 &state, const StateCov2 &cov,
			       const ScalarMeas2 &meas, float &innov, float &innov_var)
	{
		last_H = meas.H;
		innov = meas.val - (meas.H.transpose() * state)(0, 0);
		innov_var = (meas.H.transpose() * cov * meas.H)(0, 0) + meas.unc;
	}

	void applyCorrection(StateVec2 &state, StateCov2 &cov,
			     const StateVec2 &K, float innov, float innov_var)
	{
		applied_k.push_back(K);
		applied_state_before.push_back(state);
		state = state + K * innov;
		applied_state_after.push_back(state);

		const matrix::Matrix<float, 2, 1> &K_mat = K;

		cov = cov - (K_mat * K.transpose()) * innov_var;
	}
};

class VTEOosmTest : public ::testing::Test
{
protected:
	using Manager = vte::OOSMManager<TestFilter, kStateDim, float, 8>;

	Manager oosm{};
	TestFilter filter{};
	float state{0.f};
	float cov{0.f};
	float curr_input{0.f};

	void SetUp() override
	{
		state = 0.f;
		cov = 4.f;
		curr_input = 0.f;
		filter.resetTracking();
	}

	static matrix::Vector<float, kStateDim> makeVec(float value)
	{
		matrix::Vector<float, kStateDim> v{};
		v(0) = value;
		return v;
	}

	static matrix::SquareMatrix<float, kStateDim> makeCov(float variance)
	{
		matrix::SquareMatrix<float, kStateDim> P{};
		P.setZero();
		P(0, 0) = variance;
		return P;
	}

	static ScalarMeas makeMeas(uint64_t time_us, float val, float unc, float h_val = 1.f)
	{
		ScalarMeas meas{};
		meas.time_us = time_us;
		meas.val = val;
		meas.unc = unc;
		meas.H(0) = h_val;
		return meas;
	}

	void pushSample(uint64_t time_us, float state_val, float cov_val, float input)
	{
		oosm.push(time_us, makeVec(state_val), makeCov(cov_val), input);
	}

	vte::FusionResult fuseMeas(const ScalarMeas &meas, uint64_t now_us, float nis_threshold)
	{
		auto state_vec = makeVec(state);
		auto cov_mat = makeCov(cov);

		const vte::FusionResult res = oosm.fuse(filter, meas, now_us, nis_threshold, state_vec, cov_mat, curr_input);

		state = state_vec(0);
		cov = cov_mat(0, 0);
		return res;
	}
};

class VTEOosmTest2d : public ::testing::Test
{
protected:
	using Manager = vte::OOSMManager<TestFilter2d, kStateDim2, float, 8>;

	Manager oosm{};
	TestFilter2d filter{};
	StateVec2 state{};
	StateCov2 cov{};
	float curr_input{0.f};

	void SetUp() override
	{
		state = makeVec(0.f, 1.f);
		cov = makeCov(4.f, 1.f, 0.5f);
		curr_input = 0.f;
		filter.resetTracking();
	}

	static StateVec2 makeVec(float pos, float vel)
	{
		StateVec2 v{};
		v(0) = pos;
		v(1) = vel;
		return v;
	}

	static StateCov2 makeCov(float p00, float p11, float p01)
	{
		StateCov2 P{};
		P.setZero();
		P(0, 0) = p00;
		P(1, 1) = p11;
		P(0, 1) = p01;
		P(1, 0) = p01;
		return P;
	}

	static ScalarMeas2 makeMeas(uint64_t time_us, float val, float unc, float h_pos = 1.f, float h_vel = 0.f)
	{
		ScalarMeas2 meas{};
		meas.time_us = time_us;
		meas.val = val;
		meas.unc = unc;
		meas.H = makeVec(h_pos, h_vel);
		return meas;
	}

	void pushSample(uint64_t time_us, const StateVec2 &state_in,
			const StateCov2 &cov_in, float input)
	{
		oosm.push(time_us, state_in, cov_in, input);
	}

	vte::FusionResult fuseMeas(const ScalarMeas2 &meas, uint64_t now_us, float nis_threshold)
	{
		StateVec2 state_vec = state;
		StateCov2 cov_mat = cov;

		const vte::FusionResult res = oosm.fuse(filter, meas, now_us, nis_threshold, state_vec, cov_mat, curr_input);

		state = state_vec;
		cov = cov_mat;
		return res;
	}

};

// WHY: The OOSM manager must use the fast path for fresh measurements.
// WHAT: Ensure a current measurement updates state/covariance and returns FUSED_CURRENT.
TEST_F(VTEOosmTest, FusesCurrentMeasurement)
{
	// GIVEN: a fresh measurement that is newer than the OOSM cutoff.
	const uint64_t now_us = 1_s;
	const uint64_t current_offset = Manager::kOosmMinTimeUs - 1_ms;
	const ScalarMeas meas = makeMeas(now_us - current_offset, 2.f, 1.f);

	// WHEN: the measurement is fused.
	const vte::FusionResult res = fuseMeas(meas, now_us, 10.f);

	// THEN: the current-fusion path updates the live state once.
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(state, 1.6f, kTolerance);
	EXPECT_NEAR(cov, 0.8f, kTolerance);
	EXPECT_EQ(filter.applied_k.size(), 1u);
}

// A full log whose oldest observation no snapshot contains yet cannot drop it: history restarts
// from the live posterior instead, rather than replaying with a missing observation.
TEST_F(VTEOosmTest, MeasurementCapacityRestartsHistoryFromLivePosterior)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	pushSample(1_s, state, cov, curr_input);
	const uint64_t now = 1050_ms;
	constexpr int measurement_count = static_cast<int>(Manager::kMeasurementCapacity) + 1;

	for (int i = 0; i < measurement_count; ++i) {
		ASSERT_EQ(fuseMeas(makeMeas(now, 0.f, 1.f), now, 0.f).status,
			  vte::FusionStatus::STATUS_FUSED_CURRENT);
	}

	EXPECT_NEAR(cov, 4.f / (1.f + 4.f * measurement_count), kTolerance);
	EXPECT_EQ(Access::historyEntries(oosm), 1);
	EXPECT_EQ(Access::historyOldestTime(oosm), now);
	EXPECT_EQ(oosm.getFusionTimes().last_fused, now);
	EXPECT_EQ(fuseMeas(makeMeas(1020_ms, 0.f, 1.f), now, 0.f).status,
		  vte::FusionStatus::STATUS_REJECT_TOO_OLD);

	oosm.reset();
	EXPECT_EQ(oosm.getFusionTimes().last_fused, 0u);
	pushSample(now, state, cov, curr_input);
	const float variance_before = cov;
	ASSERT_EQ(fuseMeas(makeMeas(now, 0.f, 1.f), now + 30_ms, 0.f).status,
		  vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_NEAR(cov, variance_before / (variance_before + 1.f) + 0.003f, kTolerance);
}

// Once an accepted observation leaves the replay window, its capture time remains the
// freshness baseline; an unrelated velocity update must not replace it.
TEST_F(VTEOosmTest, RetainsFreshnessAfterObservationIsPruned)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	constexpr uint64_t start = 1_s;
	constexpr uint64_t step = 20_ms;
	pushSample(start, state, cov, curr_input);

	for (int cycle = 1; cycle <= Manager::kHistorySize; ++cycle) {
		const uint64_t now = start + cycle * step;
		cov += 0.002f;

		if (cycle == 1) {
			ScalarMeas relative = makeMeas(now - 5_ms, 0.f, 0.1f);
			relative.source = vte::FusionSource::Relative;
			ASSERT_EQ(fuseMeas(relative, now, 10.f).status, vte::FusionStatus::STATUS_FUSED_CURRENT);
		}

		pushSample(now, state, cov, curr_input);
	}

	ASSERT_EQ(Access::historyOldestTime(oosm), start + step);
	ScalarMeas velocity = makeMeas(start + step + 10_ms, 0.f, 0.1f);
	velocity.source = vte::FusionSource::Untracked;
	ASSERT_EQ(fuseMeas(velocity, start + Manager::kHistorySize * step, 10.f).status,
		  vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(oosm.getFusionTimes().last_fused, start + step - 5_ms);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, start + step - 5_ms);

	oosm.reset();
	EXPECT_EQ(oosm.getFusionTimes().last_fused, 0u);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, 0u);
}

// A log with one slot per snapshot must hold one fused observation per cycle indefinitely,
// so the orientation filter never drops one in steady state.
TEST_F(VTEOosmTest, OneObservationPerCycleFitsMeasurementCapacity)
{
	using OneObsManager = vte::OOSMManager<TestFilter, kStateDim, float, 8, 8>;
	using Access = vte::OOSMManagerTestAccess<OneObsManager>;
	OneObsManager one_obs{};
	auto x = makeVec(0.f);
	auto P = makeCov(4.f);
	constexpr uint64_t start = 1_s;
	constexpr uint64_t step = 20_ms;
	uint64_t now = start;
	one_obs.push(now, x, P, curr_input);

	for (int cycle = 1; cycle <= 3 * OneObsManager::kHistorySize; ++cycle) {
		now = start + cycle * step;
		ASSERT_EQ(one_obs.fuse(filter, makeMeas(now, 0.f, 1.f), now, 0.f, x, P, curr_input).status,
			  vte::FusionStatus::STATUS_FUSED_CURRENT);
		one_obs.push(now, x, P, curr_input);

		// A restart would move the oldest snapshot to the current cycle; a drop would reject the oldest below.
		const int retained = std::min(cycle + 1, static_cast<int>(OneObsManager::kHistorySize));
		ASSERT_EQ(Access::historyOldestTime(one_obs), now - (retained - 1) * step);
	}

	EXPECT_EQ(one_obs.fuse(filter, makeMeas(Access::historyOldestTime(one_obs), 0.f, 1.f), now + step, 0.f,
			       x, P, curr_input).status, vte::FusionStatus::STATUS_FUSED_OOSM);
}

// A full log drops its oldest observation instead of restarting history: replays then start from
// snapshots that contain it, so only delayed samples older than those are rejected.
TEST_F(VTEOosmTest, DropsOldestObservationWhenLogIsFull)
{
	constexpr int kObservationCapacity = 4;
	constexpr int kCycles = 10;
	constexpr int kFloorCycle = kCycles - kObservationCapacity;
	using SmallLogManager = vte::OOSMManager<TestFilter, kStateDim, float, 8, kObservationCapacity>;
	using Access = vte::OOSMManagerTestAccess<SmallLogManager>;
	SmallLogManager small_log{};
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr uint64_t now = t0 + (kCycles + 1) * step;
	constexpr uint64_t floor_time = t0 + kFloorCycle * step;
	auto x = makeVec(0.f);
	auto P = makeCov(1.f);
	small_log.push(t0, x, P, 0.f);

	// Fill the log, advance its floor, and wrap the snapshot ring.
	for (int cycle = 1; cycle <= kCycles; ++cycle) {
		const uint64_t t = t0 + cycle * step;
		filter.predictState(step * 1e-6f, 0.f, x, P, x, P);
		ScalarMeas meas = makeMeas(t - 5_ms, 0.1f * cycle, 0.5f);
		meas.source = vte::FusionSource::Untracked;

		if (cycle == kFloorCycle) {
			// This relative observation leaves the log because of capacity, before its snapshot expires.
			meas.source = vte::FusionSource::Relative;
		}

		ASSERT_EQ(small_log.fuse(filter, meas, t, 0.f, x, P, 0.f).status,
			  vte::FusionStatus::STATUS_FUSED_CURRENT);
		small_log.push(t, x, P, 0.f);
	}

	EXPECT_EQ(Access::historyEntries(small_log), SmallLogManager::kHistorySize);
	EXPECT_LT(Access::historyOldestTime(small_log), floor_time);
	EXPECT_EQ(small_log.getFusionTimes().last_relative_fused, floor_time - 5_ms);
	filter.predictState(step * 1e-6f, 0.f, x, P, x, P);

	// Only snapshots that contain all dropped observations remain valid replay floors.
	const ScalarMeas too_old = makeMeas(floor_time - 1_us, 0.f, 0.5f);
	ScalarMeas delayed = makeMeas(floor_time, 0.f, 0.5f);
	delayed.source = vte::FusionSource::Untracked;
	EXPECT_EQ(small_log.fuse(filter, too_old, now, 0.f, x, P, 0.f).status, vte::FusionStatus::STATUS_REJECT_TOO_OLD);
	ASSERT_EQ(small_log.fuse(filter, delayed, now, 0.f, x, P, 0.f).status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(small_log.getFusionTimes().last_fused, floor_time - 5_ms);
	EXPECT_EQ(small_log.getFusionTimes().last_relative_fused, floor_time - 5_ms);

	TestFilter truth_filter;
	auto x_truth = makeVec(0.f);
	auto P_truth = makeCov(1.f);
	uint64_t truth_time = t0;
	const auto fuse_truth = [&](const ScalarMeas & m, uint64_t t) {
		truth_filter.predictState((t - truth_time) * 1e-6f, 0.f, x_truth, P_truth, x_truth, P_truth);
		truth_time = t;
		float innov = 0.f;
		float innov_var = 0.f;
		truth_filter.computeInnovation(x_truth, P_truth, m, innov, innov_var);
		truth_filter.applyCorrection(x_truth, P_truth, (P_truth * m.H) / innov_var, innov, innov_var);
	};

	for (int cycle = 1; cycle <= kCycles; ++cycle) {
		if (cycle == kFloorCycle + 1) {
			fuse_truth(delayed, delayed.time_us);
		}

		const uint64_t t = t0 + cycle * step;
		fuse_truth(makeMeas(t - 5_ms, 0.1f * cycle, 0.5f), t);
	}

	truth_filter.predictState((now - truth_time) * 1e-6f, 0.f, x_truth, P_truth, x_truth, P_truth);
	EXPECT_NEAR(x(0), x_truth(0), kTolerance);
	EXPECT_NEAR(P(0, 0), P_truth(0, 0), kTolerance);

	// A reset also clears the capacity floor, even when starting a new timeline at an earlier time.
	small_log.reset();
	small_log.push(t0, x, P, 0.f);
	filter.predictState(step * 1e-6f, 0.f, x, P, x, P);
	small_log.push(t0 + step, x, P, 0.f);
	filter.predictState(step * 1e-6f, 0.f, x, P, x, P);
	EXPECT_EQ(small_log.fuse(filter, makeMeas(t0, 0.f, 0.5f), t0 + 2 * step, 0.f, x, P, 0.f).status,
		  vte::FusionStatus::STATUS_FUSED_OOSM);
}

// WHY: Near-zero innovation variance should never result in a correction.
// WHAT: Verify REJECT_COV and that the live state stays unchanged.
TEST_F(VTEOosmTest, RejectsInvalidInnovationVariance)
{
	// GIVEN: a zero-variance prior and a zero-uncertainty measurement.
	const uint64_t now_us = 2_s;
	state = 1.f;
	cov = 0.f;
	const uint64_t current_offset = Manager::kOosmMinTimeUs - 1_ms;
	const ScalarMeas meas = makeMeas(now_us - current_offset, 1.f, 0.f);

	// WHEN: the measurement is fused.
	const vte::FusionResult res = fuseMeas(meas, now_us, 10.f);

	// THEN: the correction is rejected before any gain is applied.
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_COV);
	EXPECT_EQ(filter.applied_k.size(), 0u);
	EXPECT_NEAR(state, 1.f, kTolerance);
	EXPECT_NEAR(cov, 0.f, kTolerance);
}

// WHY: NIS gating is the primary protection against outlier measurements.
// WHAT: Ensure large innovation triggers REJECT_NIS without changing state.
TEST_F(VTEOosmTest, RejectsNisForCurrentMeasurement)
{
	// GIVEN: a current measurement with a large residual and a small innovation variance.
	const uint64_t now_us = 3_s;
	state = 0.f;
	cov = 0.1f;
	const uint64_t current_offset = Manager::kOosmMinTimeUs - 1_ms;
	const ScalarMeas meas = makeMeas(now_us - current_offset, 10.f, 0.1f);

	// WHEN: the measurement is fused with a tight NIS threshold.
	const vte::FusionResult res = fuseMeas(meas, now_us, 1.f);

	// THEN: the measurement is rejected and the live state is unchanged.
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_NIS);
	EXPECT_EQ(filter.applied_k.size(), 0u);
	EXPECT_NEAR(state, 0.f, kTolerance);
	EXPECT_NEAR(cov, 0.1f, kTolerance);
}

// WHY: OOSM fusion must protect against empty or inconsistent history.
// WHAT: Validate REJECT_EMPTY and REJECT_STALE behavior with history reset.
TEST_F(VTEOosmTest, RejectsOosmHistoryGuards)
{
	// GIVEN: a delayed measurement, first with no history and then with stale history timing.
	const uint64_t meas_time_us = 900_ms;
	const ScalarMeas meas = makeMeas(meas_time_us, 0.f, 1.f);

	// WHEN: we fuse without history, then with a now timestamp older than the newest stored sample.
	const vte::FusionResult empty_res = fuseMeas(meas, 1_s, 10.f);
	EXPECT_EQ(empty_res.status, vte::FusionStatus::STATUS_REJECT_EMPTY);

	pushSample(980_ms, 0.5f, 4.f, 1.f);

	const vte::FusionResult stale_res = fuseMeas(meas, 800_ms, 10.f);
	EXPECT_EQ(stale_res.status, vte::FusionStatus::STATUS_REJECT_STALE);

	// THEN: the stale path resets history and the next attempt is empty again.
	const vte::FusionResult post_reset_res = fuseMeas(meas, 1_s, 10.f);
	EXPECT_EQ(post_reset_res.status, vte::FusionStatus::STATUS_REJECT_EMPTY);
}

// WHY: OOSM fusion must reject timestamps outside the valid window.
// WHAT: Check REJECT_TOO_NEW and REJECT_TOO_OLD for valid history.
TEST_F(VTEOosmTest, RejectsOosmOutOfRangeTimestamps)
{
	// GIVEN: a populated history window and delayed measurements just outside both ends.
	pushSample(100_ms, 0.f, 4.f, 0.f);
	pushSample(200_ms, 1.f, 4.f, 0.f);
	pushSample(250_ms, 2.f, 4.f, 0.f);

	const uint64_t now_us = 300_ms;
	const uint64_t too_new_time = now_us + Manager::kOosmMinTimeUs + 1_us;
	const ScalarMeas meas_too_new = makeMeas(too_new_time, 0.f, 1.f);

	// WHEN: the timestamps lie just beyond the allowed future/past bounds.
	const vte::FusionResult too_new_res = fuseMeas(meas_too_new, now_us, 10.f);
	EXPECT_EQ(too_new_res.status, vte::FusionStatus::STATUS_REJECT_TOO_NEW);

	const uint64_t oldest_time = 100_ms;
	const uint64_t too_old_time = oldest_time - 10_ms;
	const ScalarMeas meas_too_old = makeMeas(too_old_time, 0.f, 1.f);
	const vte::FusionResult too_old_res = fuseMeas(meas_too_old, now_us, 10.f);

	// THEN: both samples are rejected for being out of range.
	EXPECT_EQ(too_old_res.status, vte::FusionStatus::STATUS_REJECT_TOO_OLD);
}

// A sample exactly at the future tolerance must not advance the live state past now.
TEST_F(VTEOosmTest, RejectsExactFutureBoundaryWithoutChangingStateOrHistory)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	pushSample(1_s, 0.f, 4.f, 1.f);
	state = 0.1f;
	cov = 4.01f;
	curr_input = 1.f;
	const uint64_t now = 1100_ms;
	pushSample(now, state, cov, curr_input);
	const ScalarMeas meas = makeMeas(now + Manager::kOosmMinTimeUs, 0.12f, 1.f);

	EXPECT_EQ(fuseMeas(meas, now, 0.f).status, vte::FusionStatus::STATUS_REJECT_TOO_NEW);
	EXPECT_FLOAT_EQ(state, 0.1f);
	EXPECT_FLOAT_EQ(cov, 4.01f);
	EXPECT_TRUE(filter.applied_k.empty());
	EXPECT_EQ(Access::historyEntries(oosm), 2);
	EXPECT_EQ(Access::historyOldestTime(oosm), 1_s);
	EXPECT_EQ(Access::historyNewestTime(oosm), now);
}

// WHY: When the floor sample is the newest, OOSM should use current input.
// WHAT: Provide a measurement between newest sample and now and verify curr_input was used.
TEST_F(VTEOosmTest, UsesCurrentInputWhenFloorIsNewest)
{
	// GIVEN: history whose newest sample is still older than the delayed measurement.
	const uint64_t t0 = 1_s;
	pushSample(t0, 0.f, 4.f, 1.f);

	curr_input = 5.f;

	const ScalarMeas meas = makeMeas(t0 + 30_ms, 1.f, 1.f);

	// WHEN: the delayed measurement falls between the newest sample and now.
	const vte::FusionResult res = fuseMeas(meas, t0 + 60_ms, 10.f);

	// THEN: the replay applies the live input from t0, not the input stored with the floor sample.
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_OOSM);

	TestFilter truth_filter;
	auto x_truth = makeVec(0.f);
	auto P_truth = makeCov(4.f);
	truth_filter.predictState(0.03f, curr_input, x_truth, P_truth, x_truth, P_truth);
	float innov = 0.f;
	float innov_var = 0.f;
	truth_filter.computeInnovation(x_truth, P_truth, meas, innov, innov_var);
	const matrix::Vector<float, kStateDim> K = (P_truth * meas.H) / innov_var;
	truth_filter.applyCorrection(x_truth, P_truth, K, innov, innov_var);
	truth_filter.predictState(0.03f, curr_input, x_truth, P_truth, x_truth, P_truth);

	EXPECT_NEAR(state, x_truth(0), kTolerance);
	EXPECT_NEAR(cov, P_truth(0, 0), kTolerance);
}

// A fast-path observation is fused at now, so a later replay must apply it at that time
// rather than at its own timestamp, which can be up to 20 ms earlier.
TEST_F(VTEOosmTest, ReplaysFastPathObservationAtFusionTime)
{
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t t1 = t0 + 20_ms;
	constexpr uint64_t now = t1 + 20_ms;
	constexpr float input = 10.f;
	auto x = makeVec(0.f);
	auto P = makeCov(4.f);
	oosm.push(t0, x, P, input);

	// Cycle at t1: a sample captured 10 ms earlier takes the fast path.
	filter.predictState(0.02f, input, x, P, x, P);
	state = x(0);
	cov = P(0, 0);
	curr_input = input;
	const ScalarMeas fast = makeMeas(t1 - 10_ms, 1.f, 0.5f);
	ASSERT_EQ(fuseMeas(fast, t1, 0.f).status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	x = makeVec(state);
	P = makeCov(cov);
	oosm.push(t1, x, P, input);

	// Next cycle: an older sample arrives and replays the fast one.
	filter.predictState(0.02f, input, x, P, x, P);
	state = x(0);
	cov = P(0, 0);
	const ScalarMeas late = makeMeas(t0 + 5_ms, 0.2f, 0.5f);
	ASSERT_EQ(fuseMeas(late, now, 0.f).status, vte::FusionStatus::STATUS_FUSED_OOSM);

	TestFilter truth_filter;
	auto x_truth = makeVec(0.f);
	auto P_truth = makeCov(4.f);
	const auto fuse_truth = [&](const ScalarMeas & m) {
		float innov = 0.f;
		float innov_var = 0.f;
		truth_filter.computeInnovation(x_truth, P_truth, m, innov, innov_var);
		const matrix::Vector<float, kStateDim> K = (P_truth * m.H) / innov_var;
		truth_filter.applyCorrection(x_truth, P_truth, K, innov, innov_var);
	};
	truth_filter.predictState(0.005f, input, x_truth, P_truth, x_truth, P_truth);
	fuse_truth(late);
	truth_filter.predictState(0.015f, input, x_truth, P_truth, x_truth, P_truth);
	fuse_truth(fast);
	truth_filter.predictState(0.02f, input, x_truth, P_truth, x_truth, P_truth);

	EXPECT_NEAR(state, x_truth(0), kTolerance);
	EXPECT_NEAR(cov, P_truth(0, 0), kTolerance);
}

// WHY: Large time gaps between history and now should reset the buffer.
// WHAT: Force now to exceed the max OOSM window and verify REJECT_STALE with reset.
TEST_F(VTEOosmTest, RejectsStaleWhenNowExceedsHistoryWindow)
{
	// GIVEN: history that is older than the maximum supported OOSM window.
	const uint64_t t0 = 1_s;
	pushSample(t0, 0.f, 4.f, 0.f);

	const ScalarMeas meas = makeMeas(t0 + 50_ms, 1.f, 1.f);
	const uint64_t stale_now = t0 + Manager::kOosmMaxTimeUs + 1_ms;

	// WHEN: we fuse with a now timestamp outside the allowed live/history span.
	const vte::FusionResult stale_res = fuseMeas(meas, stale_now, 10.f);
	EXPECT_EQ(stale_res.status, vte::FusionStatus::STATUS_REJECT_STALE);

	// THEN: history is cleared and the next call sees an empty buffer.
	const vte::FusionResult empty_res = fuseMeas(meas, stale_now, 10.f);
	EXPECT_EQ(empty_res.status, vte::FusionStatus::STATUS_REJECT_EMPTY);
}

// WHY: Ensure OOSM (delayed fusion) yields a result equivalent to
// fusing the measurement sequentially at the correct time.
// WHAT: Run a filter in which the measurement comes on time and compare it with
// a filter with the same measurement but delayed
TEST_F(VTEOosmTest, FusesOosmBetweenSamplesSimplified)
{
	// GIVEN: a scalar linear system with history at t0, t1, and t2 plus a delayed sample at t_meas.

	static constexpr uint64_t t0 = 100_ms;
	static constexpr uint64_t t_meas = 150_ms;
	static constexpr uint64_t t1 = 200_ms;
	static constexpr uint64_t t2 = 300_ms;

	static constexpr float kInput = 2.f;
	static constexpr float kDtStep = 0.1f; // 100ms steps for history

	// Initial Conditions
	const auto x0 = makeVec(0.f);
	const auto P0 = makeCov(1.f);
	const ScalarMeas meas = makeMeas(t_meas, 10.f, 1.f);

	// Build history: t0 -> t1 -> t2
	// We simulate the filter running normally without the measurement first.
	matrix::Vector<float, kStateDim> x1, x2;
	matrix::SquareMatrix<float, kStateDim> P1, P2;

	// t0 -> t1
	filter.predictState(kDtStep, kInput, x0, P0, x1, P1);
	// t1 -> t2
	filter.predictState(kDtStep, kInput, x1, P1, x2, P2);

	// Setup OOSM Manager
	oosm.reset();
	filter.resetTracking(); // Reset the helper's internal tracking logs

	oosm.push(t0, x0, P0, kInput);
	oosm.push(t1, x1, P1, kInput);
	oosm.push(t2, x2, P2, kInput);

	// Now is t2
	state = x2(0);
	cov = P2(0, 0);

	// WHEN: we fuse the delayed sample through OOSM and compare it with chronological truth.
	const vte::FusionResult oosm_res = fuseMeas(meas, t2, 100.f);

	EXPECT_EQ(oosm_res.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(oosm_res.history_steps, 2); // Should update t1 and t2 (t0 is floor, not updated)

	const float val_oosm_state = state;
	const float val_oosm_cov = cov;

	// The truth path fuses the same sample exactly at t_meas.
	TestFilter truth_filter;
	matrix::Vector<float, kStateDim> x_truth = x0;
	matrix::SquareMatrix<float, kStateDim> P_truth = P0;

	// Predict t0 -> t_meas (100ms -> 150ms = 0.05s)
	float dt_1 = (t_meas - t0) * 1e-6f;
	truth_filter.predictState(dt_1, kInput, x_truth, P_truth, x_truth, P_truth);

	// Fuse Measurement exactly at t_meas
	float innov, innov_var;
	truth_filter.computeInnovation(x_truth, P_truth, meas, innov, innov_var);

	// Calculate K and Apply (Manually calling apply to mimic update step)
	// K = PH' / S
	matrix::Vector<float, kStateDim> K = (P_truth * meas.H) / innov_var;
	truth_filter.applyCorrection(x_truth, P_truth, K, innov, innov_var);

	// Predict t_meas -> t2 (150ms -> 300ms = 0.15s)
	float dt_2 = (t2 - t_meas) * 1e-6f;
	truth_filter.predictState(dt_2, kInput, x_truth, P_truth, x_truth, P_truth);

	// THEN: state and covariance match chronological replay.
	EXPECT_NEAR(val_oosm_state, x_truth(0), kTolerance) << "State mismatch: OOSM vs Truth";

	EXPECT_NEAR(val_oosm_cov, P_truth(0, 0), kTolerance);
}

// WHY: A second delayed measurement in the same history interval must replay the first accepted observation.
// WHAT: Fuse delayed samples at 150 ms and 175 ms into history at 100/200/300 ms and compare with chronological truth.
TEST_F(VTEOosmTest, FusesSequentialDelayedMeasurementsInSameInterval)
{
	// GIVEN: history sampled at 100/200/300 ms and two delayed measurements in the same interval.
	static constexpr uint64_t t0 = 100_ms;
	static constexpr uint64_t t_meas_1 = 150_ms;
	static constexpr uint64_t t_meas_2 = 175_ms;
	static constexpr uint64_t t1 = 200_ms;
	static constexpr uint64_t t2 = 300_ms;
	static constexpr float kInput = 2.f;

	const auto x0 = makeVec(0.f);
	const auto P0 = makeCov(1.f);
	const ScalarMeas meas_1 = makeMeas(t_meas_1, 10.f, 1.f);
	const ScalarMeas meas_2 = makeMeas(t_meas_2, 8.f, 1.f);

	matrix::Vector<float, kStateDim> x1{};
	matrix::Vector<float, kStateDim> x2{};
	matrix::SquareMatrix<float, kStateDim> P1{};
	matrix::SquareMatrix<float, kStateDim> P2{};

	filter.predictState((t1 - t0) * 1e-6f, kInput, x0, P0, x1, P1);
	filter.predictState((t2 - t1) * 1e-6f, kInput, x1, P1, x2, P2);

	oosm.reset();
	filter.resetTracking();
	oosm.push(t0, x0, P0, kInput);
	oosm.push(t1, x1, P1, kInput);
	oosm.push(t2, x2, P2, kInput);

	state = x2(0);
	cov = P2(0, 0);
	curr_input = kInput;

	// WHEN: both delayed samples are fused in arrival order at t2.
	const vte::FusionResult res_1 = fuseMeas(meas_1, t2, 100.f);
	const vte::FusionResult res_2 = fuseMeas(meas_2, t2, 100.f);

	// THEN: each delayed fusion succeeds and the second replay still updates the later stored samples.
	EXPECT_EQ(res_1.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res_2.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res_2.history_steps, 2);

	TestFilter truth_filter;
	matrix::Vector<float, kStateDim> x_truth = x0;
	matrix::SquareMatrix<float, kStateDim> P_truth = P0;

	auto fuseTruth = [&](const ScalarMeas & meas) {
		float innov = 0.f;
		float innov_var = 0.f;
		truth_filter.computeInnovation(x_truth, P_truth, meas, innov, innov_var);
		const matrix::Vector<float, kStateDim> K = (P_truth * meas.H) / innov_var;
		truth_filter.applyCorrection(x_truth, P_truth, K, innov, innov_var);
	};

	truth_filter.predictState((t_meas_1 - t0) * 1e-6f, kInput, x_truth, P_truth, x_truth, P_truth);
	fuseTruth(meas_1);
	truth_filter.predictState((t_meas_2 - t_meas_1) * 1e-6f, kInput, x_truth, P_truth, x_truth, P_truth);
	fuseTruth(meas_2);
	truth_filter.predictState((t2 - t_meas_2) * 1e-6f, kInput, x_truth, P_truth, x_truth, P_truth);

	EXPECT_NEAR(state, x_truth(0), kTolerance);
	EXPECT_NEAR(cov, P_truth(0, 0), kTolerance);
}

// Replaying crossed arrivals between snapshots must preserve the full prediction history.
TEST_F(VTEOosmTest, ReplaysBetweenSnapshotsWithoutShorteningHistory)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	constexpr uint64_t start = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr uint64_t now = start + (Manager::kHistorySize - 1) * step;
	constexpr float input = 2.f;
	auto x = makeVec(0.f);
	auto P = makeCov(4.f);
	oosm.push(start, x, P, input);

	for (int i = 1; i < Manager::kHistorySize; ++i) {
		filter.predictState(step * 1e-6f, input, x, P, x, P);
		oosm.push(start + i * step, x, P, input);
	}

	state = x(0);
	cov = P(0, 0);
	curr_input = input;
	const ScalarMeas earlier = makeMeas(start + 65_ms, 0.5f, 0.1f);
	const ScalarMeas later = makeMeas(start + 75_ms, 0.6f, 0.2f);
	ASSERT_EQ(fuseMeas(later, now, 0.f).status, vte::FusionStatus::STATUS_FUSED_OOSM);
	ASSERT_EQ(fuseMeas(earlier, now, 0.f).status, vte::FusionStatus::STATUS_FUSED_OOSM);

	TestFilter truth_filter;
	auto x_truth = makeVec(0.f);
	auto P_truth = makeCov(4.f);
	uint64_t truth_time = start;

	for (const ScalarMeas &meas : {earlier, later}) {
		truth_filter.predictState((meas.time_us - truth_time) * 1e-6f, input,
					  x_truth, P_truth, x_truth, P_truth);
		float innov = 0.f;
		float innov_var = 0.f;
		truth_filter.computeInnovation(x_truth, P_truth, meas, innov, innov_var);
		const auto K = (P_truth * meas.H) / innov_var;
		truth_filter.applyCorrection(x_truth, P_truth, K, innov, innov_var);
		truth_time = meas.time_us;
	}

	truth_filter.predictState((now - truth_time) * 1e-6f, input, x_truth, P_truth, x_truth, P_truth);
	EXPECT_NEAR(state, x_truth(0), kTolerance);
	EXPECT_NEAR(cov, P_truth(0, 0), kTolerance);

	ASSERT_EQ(Access::historyEntries(oosm), Manager::kHistorySize);

	for (int i = 0; i < Manager::kHistorySize; ++i) {
		EXPECT_EQ(Access::historySample(oosm, i).time_us, start + i * step);
	}
}

// WHY: Delayed samples after the newest snapshot must replay prior accepted observations using the live input.
// WHAT: Fuse delayed samples at 130 ms and 145 ms when history only contains 100 ms, then compare with chronological truth at 170 ms.
TEST_F(VTEOosmTest, FusesSequentialDelayedMeasurementsAfterNewestHistorySample)
{
	// GIVEN: only the 100 ms sample is stored, while both delayed measurements fall between that sample and now.
	static constexpr uint64_t t0 = 100_ms;
	static constexpr uint64_t t_meas_1 = 130_ms;
	static constexpr uint64_t t_meas_2 = 145_ms;
	static constexpr uint64_t now_us = 170_ms;
	static constexpr float kInput = 3.f;

	const auto x0 = makeVec(0.f);
	const auto P0 = makeCov(1.f);
	const ScalarMeas meas_1 = makeMeas(t_meas_1, 5.f, 1.f);
	const ScalarMeas meas_2 = makeMeas(t_meas_2, 4.f, 1.f);

	matrix::Vector<float, kStateDim> x_now{};
	matrix::SquareMatrix<float, kStateDim> P_now{};
	filter.predictState((now_us - t0) * 1e-6f, kInput, x0, P0, x_now, P_now);

	oosm.reset();
	filter.resetTracking();
	oosm.push(t0, x0, P0, kInput);

	state = x_now(0);
	cov = P_now(0, 0);
	curr_input = kInput;

	// WHEN: both delayed samples are fused before the next live sample is pushed.
	const vte::FusionResult res_1 = fuseMeas(meas_1, now_us, 100.f);
	const vte::FusionResult res_2 = fuseMeas(meas_2, now_us, 100.f);

	// THEN: both delayed fusions succeed using the live input branch and still match chronological truth.
	EXPECT_EQ(res_1.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res_2.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res_2.history_steps, 0);
	EXPECT_FLOAT_EQ(filter.last_predict_input, curr_input);

	TestFilter truth_filter;
	matrix::Vector<float, kStateDim> x_truth = x0;
	matrix::SquareMatrix<float, kStateDim> P_truth = P0;

	auto fuseTruth = [&](const ScalarMeas & meas) {
		float innov = 0.f;
		float innov_var = 0.f;
		truth_filter.computeInnovation(x_truth, P_truth, meas, innov, innov_var);
		const matrix::Vector<float, kStateDim> K = (P_truth * meas.H) / innov_var;
		truth_filter.applyCorrection(x_truth, P_truth, K, innov, innov_var);
	};

	truth_filter.predictState((t_meas_1 - t0) * 1e-6f, kInput, x_truth, P_truth, x_truth, P_truth);
	fuseTruth(meas_1);
	truth_filter.predictState((t_meas_2 - t_meas_1) * 1e-6f, kInput, x_truth, P_truth, x_truth, P_truth);
	fuseTruth(meas_2);
	truth_filter.predictState((now_us - t_meas_2) * 1e-6f, kInput, x_truth, P_truth, x_truth, P_truth);

	EXPECT_NEAR(state, x_truth(0), kTolerance);
	EXPECT_NEAR(cov, P_truth(0, 0), kTolerance);
}

// WHY: 2D OOSM must replay predictions using the full transition matrix.
// WHAT: Compare delayed fusion against chronological replay with non-zero control input.
TEST_F(VTEOosmTest2d, FusesOosm2dTruthComparison)
{
	// GIVEN: a 2D linear system with delayed position measurement and stored history.

	static constexpr uint64_t t0 = 100_ms;
	static constexpr uint64_t t_meas = 150_ms;
	static constexpr uint64_t t1 = 200_ms;
	static constexpr uint64_t t2 = 300_ms;

	static constexpr float kInput = 0.5f;
	static constexpr float kDtStep = 0.1f;

	// Initial Conditions
	const StateVec2 x0 = makeVec(0.f, 5.f);
	const StateCov2 P0 = makeCov(1.f, 1.f, 0.f);

	// Measurement: Position only (H=[1,0]), Value=0.2
	const ScalarMeas2 meas = makeMeas(t_meas, 0.2f, 1.f, 1.f, 0.f);

	matrix::Vector<float, kStateDim2> x1, x2;
	matrix::SquareMatrix<float, kStateDim2> P1, P2;

	// Predict t0 -> t1 -> t2
	filter.predictState(kDtStep, kInput, x0, P0, x1, P1);
	filter.predictState(kDtStep, kInput, x1, P1, x2, P2);

	oosm.reset();
	filter.resetTracking();
	oosm.push(t0, x0, P0, kInput);
	oosm.push(t1, x1, P1, kInput);
	oosm.push(t2, x2, P2, kInput);

	// Current State is t2
	state = x2;
	cov = P2;
	curr_input = kInput;

	// WHEN: we fuse the delayed sample through OOSM and through perfect chronological replay.
	const vte::FusionResult oosm_res = fuseMeas(meas, t2, 100.f);
	EXPECT_EQ(oosm_res.status, vte::FusionStatus::STATUS_FUSED_OOSM);

	const StateVec2 oosm_state = state;
	const StateCov2 oosm_cov = cov;

	// We run a separate filter instance in perfect chronological order.
	TestFilter2d truth_filter;
	StateVec2 x_truth = x0;
	StateCov2 P_truth = P0;

	// Predict t0 -> t_meas
	float dt_1 = (t_meas - t0) * 1e-6f;
	truth_filter.predictState(dt_1, kInput, x_truth, P_truth, x_truth, P_truth);

	// Fuse at t_meas
	float innov, innov_var;
	truth_filter.computeInnovation(x_truth, P_truth, meas, innov, innov_var);
	StateVec2 K = P_truth * meas.H / innov_var; // Simple K calculation for this scope

	// Manual correction
	const matrix::Matrix<float, 2, 1> &K_mat = K;
	x_truth = x_truth + K * innov;
	P_truth = P_truth - (K_mat * K.transpose()) * innov_var;

	// Predict t_meas -> t2
	float dt_2 = (t2 - t_meas) * 1e-6f;
	truth_filter.predictState(dt_2, kInput, x_truth, P_truth, x_truth, P_truth);

	// THEN: both state components and covariance remain close to the truth model.
	EXPECT_NEAR(oosm_state(0), x_truth(0), kTolerance) << "Position Mismatch";
	EXPECT_NEAR(oosm_state(1), x_truth(1), kTolerance) << "Velocity Mismatch";

	EXPECT_NEAR(oosm_cov(0, 0), P_truth(0, 0), kTolerance) << "Cov(0,0) Mismatch";
	EXPECT_NEAR(oosm_cov(1, 1), P_truth(1, 1), kTolerance) << "Cov(1,1) Mismatch";
}

// WHY: The OOSM manager relies on a ring buffer. Indices must wrap correctly.
// WHAT: Push enough samples to wrap the buffer (Size=8), then perform OOSM
// on a sample that is technically "older" in memory index but "newer" in time
// than the wrapped limit, ensuring the iterator logic holds.
TEST_F(VTEOosmTest, HandlesRingBufferWrapAroundAndBoundaries)
{
	// GIVEN: a history buffer that wraps around and then a second history with 1 ms spacing.

	// Standard Wrap-Around (Wide Timestamps)
	// Fill buffer (0 to 7)
	for (int i = 0; i < 8; ++i) {
		pushSample(100_ms * (i + 1), (float)i, 1.f, 0.f);
	}

	// Buffer: [100 ms, 200 ms, ..., 800 ms]
	// Push 9th sample (wraps to index 0, overwrites 100 ms)
	pushSample(900_ms, 9.f, 1.f, 0.f);

	// State:
	// Newest: 900 ms (Index 0)
	// Oldest: 200 ms (Index 1)
	// Now:    950 ms
	// Max OOSM Lag: kOosmMaxTimeUs -> Cutoff: (Now - kOosmMaxTimeUs)
	const uint64_t now_us = 950_ms;
	const uint64_t max_oosm_lag = Manager::kOosmMaxTimeUs;
	const uint64_t cutoff_time = now_us - max_oosm_lag;

	// Verify Valid Fusion in the middle
	// This forces the search to traverse from Index 0 backwards to Index 4
	// And the correction loop to traverse Index 4 -> 5 -> 6 -> 7 -> 0
	const uint64_t mid_offset = 100_ms;
	const ScalarMeas meas_mid = makeMeas(cutoff_time + mid_offset, 5.5f, 1.f);

	// WHEN: we fuse delayed samples in the middle of the wrapped buffer and at the time-window edges.
	EXPECT_EQ(fuseMeas(meas_mid, now_us, 10.f).status,
		  vte::FusionStatus::STATUS_FUSED_OOSM) << "Failed mid-buffer fusion";

	// Verify Time Window Boundaries (Max OOSM Timeout)

	// Case A: Just inside the window (cutoff time) -> Should Fuse (floor is 400 ms)
	// Note: now - cutoff_time = kOosmMaxTimeUs. Code uses `if (diff > max)`, so equal is OK.
	const ScalarMeas meas_edge_valid = makeMeas(cutoff_time, 4.5f, 1.f);
	EXPECT_EQ(fuseMeas(meas_edge_valid, now_us, 10.f).status,
		  vte::FusionStatus::STATUS_FUSED_OOSM) << "Failed valid edge case (cutoff time)";

	// Case B: Just outside the window (cutoff - 10 us) -> Reject Too Old
	const uint64_t edge_invalid_time = cutoff_time - 10_us;
	const ScalarMeas meas_edge_invalid = makeMeas(edge_invalid_time, 4.5f, 1.f);
	EXPECT_EQ(fuseMeas(meas_edge_invalid, now_us, 10.f).status,
		  vte::FusionStatus::STATUS_REJECT_TOO_OLD) << "Failed invalid edge case (cutoff - 10 us)";

	// Tight Timestamp Verification
	// We reset and use tiny time steps to prove that data is rejected
	// because it fell off the ring buffer, NOT because of the max OOSM timeout.
	oosm.reset();
	filter.resetTracking();
	state = 0.f;
	cov = 4.f;

	// We need time_diff > kOosmMinTimeUs to force OOSM logic.
	// We use 1ms steps.
	// Buffer: [10ms, 11ms, ..., 17ms] (Size 8)
	const uint64_t base_t = 10_ms;
	const uint64_t step_t = 1_ms;

	for (int i = 0; i < 8; ++i) {
		pushSample(base_t + step_t *i, (float)i, 1.f, 0.f);
	}

	// Buffer: [10 ms, 11 ms, ..., 17 ms]

	// Push 9th sample (wraps to index 0, overwrites 10 ms)
	// Timestamp: 18 ms
	pushSample(base_t + step_t * 8, 9.f, 1.f, 0.f);

	// Buffer State:
	// Newest: 18 ms
	// Oldest: 11 ms (Index 1)

	// Set 'now' such that (now - meas) > kOosmMinTimeUs to avoid FUSED_CURRENT.
	const uint64_t extra_margin = 10_ms;
	const uint64_t tight_now = base_t + Manager::kOosmMinTimeUs + extra_margin;

	// Case C: Try to fuse time=base_t + 500 us.
	// Time diff = tight_now - meas_overwritten_time (> kOosmMinTimeUs).
	// History Check: 10.5 ms < Oldest (11 ms).
	// MUST return REJECT_TOO_OLD (buffer overwritten).
	const uint64_t meas_overwritten_time = base_t + 500_us;
	const ScalarMeas meas_overwritten = makeMeas(meas_overwritten_time, 0.f, 1.f);
	vte::FusionResult res_over = fuseMeas(meas_overwritten, tight_now, 10.f);

	EXPECT_EQ(res_over.status, vte::FusionStatus::STATUS_REJECT_TOO_OLD)
			<< "Expected REJECT_TOO_OLD for overwritten sample. Got: " << (int)res_over.status;

	// Case D: Verify boundary of new oldest (11 ms)
	const uint64_t oldest_time = base_t + step_t;
	const ScalarMeas meas_oldest_valid = makeMeas(oldest_time, 0.f, 1.f);
	vte::FusionResult res_valid = fuseMeas(meas_oldest_valid, tight_now, 10.f);

	// THEN: wrapped-but-valid samples still fuse, while overwritten samples are rejected as too old.
	EXPECT_EQ(res_valid.status, vte::FusionStatus::STATUS_FUSED_OOSM)
			<< "Expected FUSED_OOSM for oldest valid sample. Got: " << (int)res_valid.status;
}

// WHY: An exact-time update must remain in the oldest posterior after its observation is retired.
// WHAT: Fuse twice at the oldest snapshot and verify both updates contribute exactly once.
TEST_F(VTEOosmTest, FusesOosmExactTimeMatch)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	pushSample(100_ms, 0.f, 4.f, 0.f);
	pushSample(200_ms, 0.f, 4.01f, 0.f);
	cov = 4.015f;

	ASSERT_EQ(fuseMeas(makeMeas(100_ms, 0.5f, 1.f), 250_ms, 0.f).status,
		  vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_NEAR(state, 0.4f, kTolerance);
	EXPECT_NEAR(cov, 0.815f, kTolerance);

	ASSERT_EQ(fuseMeas(makeMeas(100_ms, 1.5f, 1.f), 250_ms, 0.f).status,
		  vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_NEAR(state, 8.f / 9.f, kTolerance);
	EXPECT_NEAR(cov, 4.f / 9.f + 0.015f, kTolerance);

	ASSERT_EQ(Access::historyEntries(oosm), 2);
	const Manager::Sample &oldest = Access::historySample(oosm, 0);
	EXPECT_EQ(oldest.time_us, 100_ms);
	EXPECT_NEAR(oldest.state(0), 8.f / 9.f, kTolerance);
	EXPECT_NEAR(oldest.cov(0, 0), 4.f / 9.f, kTolerance);
}

// WHY: A delayed measurement might look valid now, but was an outlier at the time it occurred.
// WHAT: Provide a measurement that contradicts the history state at t_meas.
TEST_F(VTEOosmTest, RejectsNisDuringOosmPrediction)
{
	// GIVEN: a delayed measurement whose residual is large at its own timestamp.
	pushSample(100_ms, 0.f, 0.1f, 0.f);
	pushSample(200_ms, 0.f, 0.1f, 0.f);

	// State at 150 ms is ~0.0. Meas is 10.0. Variance small. Should fail NIS.
	const ScalarMeas meas = makeMeas(150_ms, 10.f, 0.1f);

	// WHEN: the OOSM logic evaluates that historical innovation.
	const vte::FusionResult res = fuseMeas(meas, 250_ms, 1.f);

	// THEN: the delayed sample is rejected before any history is modified.
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_NIS);
	// Ensure no history corrections were applied
	EXPECT_EQ(filter.applied_k.size(), 0u);
}

// A late observation that contradicts stored ones gets them rejected on replay, as if all had arrived
// in time order, and reports how many decisions changed and how strongly the stored ones now disagree.
TEST_F(VTEOosmTest, RegatesStoredObservationsAfterDelayedSample)
{
	constexpr uint64_t start = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr float kThreshold = 10.f;
	auto x = makeVec(0.f);
	auto P = makeCov(4.f);
	oosm.push(start, x, P, curr_input);

	// Two fast-path observations agree with the prior and are stored at their cycle times.
	for (uint64_t cycle = 1; cycle <= 2; ++cycle) {
		const uint64_t now = start + cycle * step;
		filter.predictState(step * 1e-6f, curr_input, x, P, x, P);
		state = x(0);
		cov = P(0, 0);
		ScalarMeas relative = makeMeas(now - 5_ms, 0.f, 0.01f);
		relative.source = vte::FusionSource::Relative;
		ASSERT_EQ(fuseMeas(relative, now, kThreshold).status,
			  vte::FusionStatus::STATUS_FUSED_CURRENT);
		x = makeVec(state);
		P = makeCov(cov);
		oosm.push(now, x, P, curr_input);
	}

	EXPECT_EQ(oosm.getFusionTimes().last_fused, start + 2 * step - 5_ms);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, start + 2 * step - 5_ms);

	// A late observation passes its own gate but contradicts both stored ones.
	const uint64_t now = start + 3 * step;
	filter.predictState(step * 1e-6f, curr_input, x, P, x, P);
	state = x(0);
	cov = P(0, 0);
	const size_t corrections_before = filter.applied_k.size();
	const ScalarMeas late = makeMeas(start + 10_ms, 5.f, 0.01f);
	const vte::FusionResult res = fuseMeas(late, now, kThreshold);

	ASSERT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res.replayed_count, 2);
	EXPECT_EQ(res.replay_revised_count, 2);
	EXPECT_EQ(filter.applied_k.size() - corrections_before, 1u);
	EXPECT_EQ(oosm.getFusionTimes().last_fused, late.time_us);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, 0u);

	// Chronological fusion with the same gate.
	TestFilter truth_filter;
	auto x_truth = makeVec(0.f);
	auto P_truth = makeCov(4.f);
	const auto gate_truth = [&](const ScalarMeas & m) {
		float innov = 0.f;
		float innov_var = 0.f;
		truth_filter.computeInnovation(x_truth, P_truth, m, innov, innov_var);
		const float ratio = innov * innov / (innov_var * kThreshold);

		if (ratio <= 1.f) {
			truth_filter.applyCorrection(x_truth, P_truth, (P_truth * m.H) / innov_var, innov, innov_var);
		}

		return ratio;
	};
	truth_filter.predictState(0.01f, curr_input, x_truth, P_truth, x_truth, P_truth);
	gate_truth(late);
	truth_filter.predictState(0.01f, curr_input, x_truth, P_truth, x_truth, P_truth);
	const float ratio_1 = gate_truth(makeMeas(start + step, 0.f, 0.01f));
	truth_filter.predictState(0.02f, curr_input, x_truth, P_truth, x_truth, P_truth);
	const float ratio_2 = gate_truth(makeMeas(start + 2 * step, 0.f, 0.01f));
	truth_filter.predictState(0.02f, curr_input, x_truth, P_truth, x_truth, P_truth);

	EXPECT_GT(ratio_1, 1.f);
	EXPECT_GT(ratio_2, 1.f);
	const float max_ratio = fmaxf(ratio_1, ratio_2);
	EXPECT_NEAR(res.replay_test_ratio_max, max_ratio, 1e-3f * max_ratio);
	EXPECT_NEAR(state, x_truth(0), kTolerance);
	EXPECT_NEAR(cov, P_truth(0, 0), kTolerance);
}

// With the gate repeated on replay, the arrival order does not matter: delayed observations,
// outliers included, give the same state as fusing them in time order with the same gate.
TEST_F(VTEOosmTest, ResultDoesNotDependOnArrivalOrder)
{
	using LongManager = vte::OOSMManager<TestFilter, kStateDim, float, 25>;
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr float input = 0.5f;
	constexpr float kThreshold = 10.f;
	constexpr int kObsCount = 10;
	constexpr int kCycles = 22;
	std::mt19937 rng(1234);
	const auto uniform = [&rng](float lo, float hi) { return lo + (hi - lo) * static_cast<float>(rng() / 4294967296.0); };
	int revised_total = 0;

	struct Arrival {
		ScalarMeas meas;
		uint64_t time_us;
	};

	for (int trial = 0; trial < 200; ++trial) {
		Arrival arrivals[kObsCount];

		// Capture within 20-200 ms after t0, arrival 20-200 ms later; every fifth one is an outlier.
		for (int i = 0; i < kObsCount; ++i) {
			const uint64_t capture = t0 + 20_ms + static_cast<uint64_t>(uniform(0.f, 180000.f));
			const float value = (i % 5 == 0) ? ((uniform(0.f, 1.f) < 0.5f) ? 3.f : -3.f) : uniform(-0.15f, 0.15f);
			arrivals[i] = {makeMeas(capture, value, 0.05f), capture + 20_ms + static_cast<uint64_t>(uniform(0.f, 180000.f))};
		}

		LongManager run{};
		TestFilter run_filter{};
		auto x = makeVec(0.f);
		auto P = makeCov(1.f);
		run.push(t0, x, P, input);
		uint64_t now = t0;

		for (int cycle = 1; cycle <= kCycles; ++cycle) {
			now = t0 + cycle * step;
			run_filter.predictState(step * 1e-6f, input, x, P, x, P);
			ScalarMeas batch[kObsCount];
			vte::FusionResult res[kObsCount];
			size_t count = 0;

			for (const Arrival &arrival : arrivals) {
				if ((arrival.time_us > now - step) && (arrival.time_us <= now)) {
					batch[count++] = arrival.meas;
				}
			}

			run.fuseBatch(run_filter, batch, count, now, kThreshold, x, P, input, res);

			for (size_t i = 0; i < count; ++i) {
				ASSERT_TRUE((res[i].status == vte::FusionStatus::STATUS_FUSED_OOSM)
					    || (res[i].status == vte::FusionStatus::STATUS_REJECT_NIS)) << "trial " << trial;
				revised_total += res[i].replay_revised_count;
			}

			run.push(now, x, P, input);
		}

		std::sort(arrivals, arrivals + kObsCount, [](const Arrival & a, const Arrival & b) { return a.meas.time_us < b.meas.time_us; });
		TestFilter truth_filter;
		auto x_truth = makeVec(0.f);
		auto P_truth = makeCov(1.f);
		uint64_t truth_time = t0;

		for (const Arrival &arrival : arrivals) {
			truth_filter.predictState((arrival.meas.time_us - truth_time) * 1e-6f, input, x_truth, P_truth, x_truth, P_truth);
			truth_time = arrival.meas.time_us;
			float innov = 0.f;
			float innov_var = 0.f;
			truth_filter.computeInnovation(x_truth, P_truth, arrival.meas, innov, innov_var);

			if (innov * innov <= kThreshold * innov_var) {
				truth_filter.applyCorrection(x_truth, P_truth, (P_truth * arrival.meas.H) / innov_var, innov, innov_var);
			}
		}

		truth_filter.predictState((now - truth_time) * 1e-6f, input, x_truth, P_truth, x_truth, P_truth);
		EXPECT_NEAR(x(0), x_truth(0), kTolerance) << "trial " << trial;
		EXPECT_NEAR(P(0, 0), P_truth(0, 0), kTolerance) << "trial " << trial;
	}

	// Some arrival orders changed decisions that a gate frozen at arrival would have kept.
	EXPECT_GT(revised_total, 0);
}

// A decision revised on the way to a rejected late sample still reaches the live state.
TEST_F(VTEOosmTest, CommitsRevisedDecisionBeforeRejectedSample)
{
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t now = t0 + 40_ms;
	pushSample(t0, 0.f, 1.f, 0.f);
	pushSample(t0 + 20_ms, 0.f, 1.002f, 0.f);
	cov = 1.004f;

	// Accepted with a wide gate (test ratio about 0.03).
	ASSERT_EQ(fuseMeas(makeMeas(t0 + 5_ms, 0.6f, 0.05f), now, 10.f).status, vte::FusionStatus::STATUS_FUSED_OOSM);
	ASSERT_GT(state, 0.5f);

	// With a narrower gate, the stored observation is rejected on the way to the late outlier.
	const vte::FusionResult res = fuseMeas(makeMeas(t0 + 10_ms, 50.f, 0.05f), now, 0.1f);
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_NIS);
	EXPECT_NEAR(state, 0.f, kTolerance);
	EXPECT_NEAR(cov, 1.004f, kTolerance);
}

// A fast-path observation rejected on arrival is kept, so a later replay can accept it.
TEST_F(VTEOosmTest, RegatesRejectedFastPathObservation)
{
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t t1 = t0 + 20_ms;
	constexpr uint64_t now = t1 + 20_ms;
	pushSample(t0, 0.f, 0.01f, 0.f);
	cov = 0.012f;
	ScalarMeas relative = makeMeas(t1 - 5_ms, 1.f, 0.01f);
	relative.source = vte::FusionSource::Relative;
	ASSERT_EQ(fuseMeas(relative, t1, 10.f).status, vte::FusionStatus::STATUS_REJECT_NIS);
	EXPECT_EQ(oosm.getFusionTimes().last_fused, 0u);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, 0u);
	pushSample(t1, state, cov, 0.f);
	cov += 0.002f;

	// A wider gate on the next replay accepts it.
	const vte::FusionResult res = fuseMeas(makeMeas(t0 + 5_ms, 0.f, 0.01f), now, 100.f);
	ASSERT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res.replayed_count, 1);
	EXPECT_EQ(res.replay_revised_count, 1);
	EXPECT_FLOAT_EQ(res.replay_test_ratio_max, -1.f);
	EXPECT_GT(state, 0.3f);
	EXPECT_EQ(oosm.getFusionTimes().last_fused, relative.time_us);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, relative.time_us);

	ScalarMeas velocity = makeMeas(now, state, 0.1f);
	velocity.source = vte::FusionSource::Untracked;
	ASSERT_EQ(fuseMeas(velocity, now, 100.f).status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_EQ(oosm.getFusionTimes().last_fused, relative.time_us);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, relative.time_us);
}

// A stored observation that fails during replay must not stay in the log and block every later replay.
TEST_F(VTEOosmTest, ClearsHistoryWhenReplayFails)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t t1 = t0 + 20_ms;

	// Exercise failure in the snapshot loop and in the final replay to now.
	for (bool snapshot_at_t1 : {true, false}) {
		SCOPED_TRACE(snapshot_at_t1);
		oosm.reset();
		state = 0.f;
		pushSample(t0, state, 4.f, 0.f);
		pushSample(t1 - 1_us, state, 4.002f, 0.f);

		// A noiseless observation is accepted against the wide live prior and stored at t1.
		cov = 4.002f;
		ASSERT_EQ(fuseMeas(makeMeas(t1, 0.f, 0.f), t1, 0.f).status, vte::FusionStatus::STATUS_FUSED_CURRENT);

		if (snapshot_at_t1) {
			pushSample(t1, state, cov, 0.f);
		}

		// The late sample rewrites the t1 - 1 us snapshot before the stored observation fails.
		const uint64_t now = t1 + 30_ms;
		cov += 0.003f;
		const float state_before = state;
		const float cov_before = cov;

		EXPECT_EQ(fuseMeas(makeMeas(t1 - 2_us, 1.f, 0.f), now, 0.f).status, vte::FusionStatus::STATUS_REJECT_COV);
		EXPECT_FLOAT_EQ(state, state_before);
		EXPECT_FLOAT_EQ(cov, cov_before);
		EXPECT_EQ(Access::historyEntries(oosm), 0);
		EXPECT_EQ(fuseMeas(makeMeas(t1 + 5_ms, 0.f, 1.f), now, 0.f).status, vte::FusionStatus::STATUS_REJECT_EMPTY);

		// Fresh history allows delayed fusion to recover.
		pushSample(now, state, cov, 0.f);
		cov += 0.003f;
		EXPECT_EQ(fuseMeas(makeMeas(now + 5_ms, 0.2f, 1.f), now + 30_ms, 0.f).status,
			  vte::FusionStatus::STATUS_FUSED_OOSM);
		EXPECT_TRUE(std::isfinite(state));
		EXPECT_TRUE(std::isfinite(cov));
	}
}

// Observations between the restored snapshot and the late one only rebuild its prior, so they are not reported.
TEST_F(VTEOosmTest, DoesNotReportReplaysBeforeDelayedSample)
{
	pushSample(100_ms, 0.f, 4.f, 0.f);
	pushSample(200_ms, 0.f, 4.f, 0.f);

	ASSERT_EQ(fuseMeas(makeMeas(130_ms, 0.f, 1.f), 250_ms, 10.f).status, vte::FusionStatus::STATUS_FUSED_OOSM);
	const vte::FusionResult res = fuseMeas(makeMeas(160_ms, 0.f, 1.f), 250_ms, 10.f);

	ASSERT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res.replayed_count, 0);
	EXPECT_FLOAT_EQ(res.replay_test_ratio_max, -1.f);
}

// Replay rewrites the snapshots after a late sample in place, also across the ring-buffer wrap,
// while a rejected late sample leaves every snapshot untouched.
TEST_F(VTEOosmTest, RewritesWrappedHistoryInPlace)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	constexpr uint64_t start = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr int pushed = Manager::kHistorySize + 3;
	auto x = makeVec(0.f);
	auto P = makeCov(4.f);
	oosm.push(start, x, P, 0.f);

	for (int i = 1; i < pushed; ++i) {
		const float input = static_cast<float>(i);
		filter.predictState(step * 1e-6f, input, x, P, x, P);
		oosm.push(start + i * step, x, P, input);
	}

	const uint64_t now = start + pushed * step;
	curr_input = -2.f;
	filter.predictState(step * 1e-6f, curr_input, x, P, x, P);
	state = x(0);
	cov = P(0, 0);

	ASSERT_EQ(Access::historyEntries(oosm), Manager::kHistorySize);
	std::vector<Manager::Sample> before;

	for (int i = 0; i < Manager::kHistorySize; ++i) {
		before.push_back(Access::historySample(oosm, i));
	}

	// Between the third and fourth retained snapshots.
	const uint64_t t_meas = before[2].time_us + 5_ms;
	ASSERT_EQ(fuseMeas(makeMeas(t_meas, 100.f, 0.01f), now, 1.f).status, vte::FusionStatus::STATUS_REJECT_NIS);

	for (int i = 0; i < Manager::kHistorySize; ++i) {
		const Manager::Sample &sample = Access::historySample(oosm, i);
		EXPECT_EQ(sample.time_us, before[i].time_us);
		EXPECT_FLOAT_EQ(sample.state(0), before[i].state(0));
		EXPECT_FLOAT_EQ(sample.cov(0, 0), before[i].cov(0, 0));
		EXPECT_FLOAT_EQ(sample.input, before[i].input);
	}

	// Same gate as the outlier, which is stored and stays rejected on replay.
	const ScalarMeas meas = makeMeas(t_meas, 0.5f, 0.1f);
	const vte::FusionResult res = fuseMeas(meas, now, 1.f);
	ASSERT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_EQ(res.history_steps, Manager::kHistorySize - 3);
	EXPECT_EQ(res.replay_revised_count, 0);

	TestFilter truth_filter;
	auto x_truth = makeVec(before[2].state(0));
	auto P_truth = makeCov(before[2].cov(0, 0));
	truth_filter.predictState((t_meas - before[2].time_us) * 1e-6f, before[3].input, x_truth, P_truth, x_truth, P_truth);
	float innov = 0.f;
	float innov_var = 0.f;
	truth_filter.computeInnovation(x_truth, P_truth, meas, innov, innov_var);
	truth_filter.applyCorrection(x_truth, P_truth, (P_truth * meas.H) / innov_var, innov, innov_var);
	uint64_t truth_time = t_meas;

	for (int i = 0; i < Manager::kHistorySize; ++i) {
		const Manager::Sample &sample = Access::historySample(oosm, i);
		EXPECT_EQ(sample.time_us, before[i].time_us);
		EXPECT_FLOAT_EQ(sample.input, before[i].input);

		if (sample.time_us < t_meas) {
			EXPECT_FLOAT_EQ(sample.state(0), before[i].state(0));
			EXPECT_FLOAT_EQ(sample.cov(0, 0), before[i].cov(0, 0));
			continue;
		}

		truth_filter.predictState((sample.time_us - truth_time) * 1e-6f, before[i].input, x_truth, P_truth, x_truth, P_truth);
		truth_time = sample.time_us;
		EXPECT_NEAR(sample.state(0), x_truth(0), kTolerance) << "snapshot " << i;
		EXPECT_NEAR(sample.cov(0, 0), P_truth(0, 0), kTolerance) << "snapshot " << i;
	}

	truth_filter.predictState((now - truth_time) * 1e-6f, curr_input, x_truth, P_truth, x_truth, P_truth);
	EXPECT_NEAR(state, x_truth(0), kTolerance);
	EXPECT_NEAR(cov, P_truth(0, 0), kTolerance);
}

// Delayed samples of one cycle share one replay, with the same result as fusing them one at a time,
// oldest first: an outlier between them is still rejected and a sample on a snapshot time still
// rewrites that snapshot. Fewer predictions and corrections are needed.
TEST_F(VTEOosmTest, BatchMatchesSequentialFusion)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	constexpr uint64_t start = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr uint64_t now = start + Manager::kHistorySize * step;
	constexpr float input = 0.5f;
	constexpr float kThreshold = 10.f;

	struct Run {
		Manager oosm{};
		TestFilter filter{};
		matrix::Vector<float, kStateDim> x{};
		matrix::SquareMatrix<float, kStateDim> P{};
	};

	// History with a fast-path observation stored in every cycle.
	const auto build = [&](Run & run) {
		run.x = makeVec(0.f);
		run.P = makeCov(1.f);
		run.oosm.push(start, run.x, run.P, input);

		for (int cycle = 1; cycle < Manager::kHistorySize; ++cycle) {
			const uint64_t t = start + cycle * step;
			run.filter.predictState(step * 1e-6f, input, run.x, run.P, run.x, run.P);
			ASSERT_EQ(run.oosm.fuse(run.filter, makeMeas(t - 5_ms, 0.01f * cycle, 0.05f), t, kThreshold,
						run.x, run.P, input).status, vte::FusionStatus::STATUS_FUSED_CURRENT);
			run.oosm.push(t, run.x, run.P, input);
		}

		run.filter.predictState(step * 1e-6f, input, run.x, run.P, run.x, run.P);
		run.filter.resetTracking();
	};

	const ScalarMeas fresh = makeMeas(now - 5_ms, 0.1f, 0.05f);
	const ScalarMeas late_on_snapshot = makeMeas(start + 5 * step, 0.08f, 0.05f);
	const ScalarMeas outlier = makeMeas(start + 3 * step + 3_ms, 50.f, 0.05f);
	const ScalarMeas late = makeMeas(start + 2 * step + 7_ms, 0.03f, 0.05f);

	Run batch;
	build(batch);
	const ScalarMeas batch_meas[] {fresh, late_on_snapshot, outlier, late};
	vte::FusionResult batch_res[4];
	batch.oosm.fuseBatch(batch.filter, batch_meas, 4, now, kThreshold, batch.x, batch.P, input, batch_res);

	Run seq;
	build(seq);
	vte::FusionResult seq_res[4];

	for (int i : {3, 2, 1, 0}) {
		seq_res[i] = seq.oosm.fuse(seq.filter, batch_meas[i], now, kThreshold, seq.x, seq.P, input);
	}

	const uint8_t expected_status[] {vte::FusionStatus::STATUS_FUSED_CURRENT, vte::FusionStatus::STATUS_FUSED_OOSM,
					 vte::FusionStatus::STATUS_REJECT_NIS, vte::FusionStatus::STATUS_FUSED_OOSM
					};

	for (int i = 0; i < 4; ++i) {
		EXPECT_EQ(batch_res[i].status, expected_status[i]) << "measurement " << i;
		EXPECT_EQ(batch_res[i].status, seq_res[i].status) << "measurement " << i;
		EXPECT_FLOAT_EQ(batch_res[i].innov, seq_res[i].innov) << "measurement " << i;
		EXPECT_FLOAT_EQ(batch_res[i].innov_var, seq_res[i].innov_var) << "measurement " << i;
		EXPECT_FLOAT_EQ(batch_res[i].test_ratio, seq_res[i].test_ratio) << "measurement " << i;
		EXPECT_EQ(batch_res[i].history_steps, seq_res[i].history_steps) << "measurement " << i;
		EXPECT_EQ(batch_res[i].replayed_count, seq_res[i].replayed_count) << "measurement " << i;
	}

	// The newer sample's replay diagnostics do not depend on how the older one was fused.
	EXPECT_FLOAT_EQ(batch_res[1].replay_test_ratio_max, seq_res[1].replay_test_ratio_max);
	EXPECT_EQ(batch_res[3].history_steps, Manager::kHistorySize - 3);
	EXPECT_EQ(batch_res[1].history_steps, Manager::kHistorySize - 6);

	EXPECT_FLOAT_EQ(batch.x(0), seq.x(0));
	EXPECT_FLOAT_EQ(batch.P(0, 0), seq.P(0, 0));

	for (int i = 0; i < Manager::kHistorySize; ++i) {
		const Manager::Sample &batch_sample = Access::historySample(batch.oosm, i);
		const Manager::Sample &seq_sample = Access::historySample(seq.oosm, i);
		EXPECT_EQ(batch_sample.time_us, seq_sample.time_us);
		EXPECT_FLOAT_EQ(batch_sample.state(0), seq_sample.state(0)) << "snapshot " << i;
		EXPECT_FLOAT_EQ(batch_sample.cov(0, 0), seq_sample.cov(0, 0)) << "snapshot " << i;
	}

	// One replay from the oldest floor instead of one per delayed sample.
	EXPECT_EQ(batch.filter.predict_count, 8);
	EXPECT_EQ(batch.filter.applied_k.size(), 8u);
	EXPECT_EQ(seq.filter.predict_count, 11);
	EXPECT_EQ(seq.filter.applied_k.size(), 10u);

	// The stored log matches as well: a later replay gives the same result.
	const ScalarMeas next = makeMeas(start + 4 * step + 2_ms, 0.05f, 0.05f);
	const vte::FusionResult batch_next = batch.oosm.fuse(batch.filter, next, now, kThreshold, batch.x, batch.P, input);
	const vte::FusionResult seq_next = seq.oosm.fuse(seq.filter, next, now, kThreshold, seq.x, seq.P, input);
	EXPECT_EQ(batch_next.replayed_count, seq_next.replayed_count);
	EXPECT_FLOAT_EQ(batch.x(0), seq.x(0));
	EXPECT_FLOAT_EQ(batch.P(0, 0), seq.P(0, 0));
}

// Sequential fusion stores the rejected outlier, so the later sample's replay splits its prediction
// there while the batch does not: the two then match up to float rounding.
TEST_F(VTEOosmTest, BatchMatchesSequentialFusionAcrossRejectedSample)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr uint64_t now = t0 + 4 * step;
	constexpr float input = 0.5f;
	constexpr float kThreshold = 10.f;
	Manager seq{};
	auto x = makeVec(0.f);
	auto P = makeCov(1.f);
	oosm.push(t0, x, P, input);
	seq.push(t0, x, P, input);

	for (int i = 1; i < 4; ++i) {
		filter.predictState(step * 1e-6f, input, x, P, x, P);
		oosm.push(t0 + i * step, x, P, input);
		seq.push(t0 + i * step, x, P, input);
	}

	filter.predictState(step * 1e-6f, input, x, P, x, P);
	auto x_seq = x;
	auto P_seq = P;

	// The outlier lies between the second sample and its floor at t0 + step.
	const ScalarMeas batch_meas[] {makeMeas(t0 + step + 11_ms, 0.03f, 0.05f), makeMeas(t0 + step + 3_ms, 50.f, 0.05f),
				       makeMeas(t0 + 7_ms, 0.01f, 0.05f)
				      };
	vte::FusionResult batch_res[3];
	oosm.fuseBatch(filter, batch_meas, 3, now, kThreshold, x, P, input, batch_res);

	vte::FusionResult seq_res[3];

	for (int i : {2, 1, 0}) {
		seq_res[i] = seq.fuse(filter, batch_meas[i], now, kThreshold, x_seq, P_seq, input);
	}

	const uint8_t expected_status[] {vte::FusionStatus::STATUS_FUSED_OOSM, vte::FusionStatus::STATUS_REJECT_NIS,
					 vte::FusionStatus::STATUS_FUSED_OOSM
					};

	for (int i = 0; i < 3; ++i) {
		EXPECT_EQ(batch_res[i].status, expected_status[i]) << "measurement " << i;
		EXPECT_EQ(seq_res[i].status, expected_status[i]) << "measurement " << i;
		EXPECT_NEAR(batch_res[i].innov, seq_res[i].innov, 1e-5f) << "measurement " << i;
	}

	EXPECT_NEAR(x(0), x_seq(0), 1e-5f);
	EXPECT_NEAR(P(0, 0), P_seq(0, 0), 1e-5f);

	for (int i = 0; i < 4; ++i) {
		EXPECT_NEAR(Access::historySample(oosm, i).state(0), Access::historySample(seq, i).state(0), 1e-5f);
		EXPECT_NEAR(Access::historySample(oosm, i).cov(0, 0), Access::historySample(seq, i).cov(0, 0), 1e-5f);
	}
}

// A stored observation that fails during a batch replay rejects the delayed samples and clears
// history, while fresh samples of the same cycle still fuse.
TEST_F(VTEOosmTest, BatchReplayFailureKeepsFreshSamples)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	constexpr uint64_t t0 = 1_s;
	constexpr uint64_t t1 = t0 + 20_ms;
	pushSample(t0, 0.f, 4.f, 0.f);
	cov = 4.002f;
	ASSERT_EQ(fuseMeas(makeMeas(t1, 0.f, 0.f), t1, 0.f).status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	pushSample(t1, state, cov, 0.f);

	const uint64_t now = t1 + 30_ms;
	cov += 0.003f;
	auto x = makeVec(state);
	auto P = makeCov(cov);
	const ScalarMeas meas[] {makeMeas(now - 5_ms, 0.f, 1.f), makeMeas(t1 - 1_us, 0.f, 0.f)};
	vte::FusionResult res[2];
	oosm.fuseBatch(filter, meas, 2, now, 0.f, x, P, curr_input, res);

	EXPECT_EQ(res[0].status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_EQ(res[1].status, vte::FusionStatus::STATUS_REJECT_COV);
	EXPECT_EQ(Access::historyEntries(oosm), 0);
	EXPECT_NEAR(P(0, 0), cov / (cov + 1.f), kTolerance);
}

// A stored observation that fails before a delayed sample's gate rejects that sample too and
// clears history, while an outlier rejected on its own keeps its status.
TEST_F(VTEOosmTest, BatchReplayFailureBeforeGate)
{
	using Access = vte::OOSMManagerTestAccess<Manager>;
	pushSample(1000_ms, 0.f, 1.f, 0.f);
	pushSample(1020_ms, 0.f, 1.002f, 0.f);
	cov = 1.004f;
	const uint64_t now = 1040_ms;
	ASSERT_EQ(fuseMeas(makeMeas(1005_ms, 0.f, 0.1f), now, 0.f).status, vte::FusionStatus::STATUS_FUSED_OOSM);
	ScalarMeas relative = makeMeas(1003_ms, 0.5f, 0.1f);
	relative.source = vte::FusionSource::Relative;
	ASSERT_EQ(fuseMeas(relative, now, 0.01f).status, vte::FusionStatus::STATUS_REJECT_NIS);
	ASSERT_EQ(oosm.getFusionTimes().last_relative_fused, 0u);

	// Replay provisionally accepts the relative observation, then fails on the next one.
	filter.fail_time = 1005_ms;
	const size_t corrections_before = filter.applied_k.size();
	auto x = makeVec(state);
	auto P = makeCov(cov);
	const ScalarMeas meas[] {makeMeas(1002_ms, 50.f, 0.01f), makeMeas(1010_ms, 0.f, 0.1f)};
	vte::FusionResult res[2];
	oosm.fuseBatch(filter, meas, 2, now, 1.f, x, P, curr_input, res);

	EXPECT_EQ(res[0].status, vte::FusionStatus::STATUS_REJECT_NIS);
	EXPECT_EQ(res[1].status, vte::FusionStatus::STATUS_REJECT_COV);
	EXPECT_EQ(Access::historyEntries(oosm), 0);
	EXPECT_EQ(filter.applied_k.size() - corrections_before, 1u);
	EXPECT_FLOAT_EQ(x(0), state);
	EXPECT_FLOAT_EQ(P(0, 0), cov);
	EXPECT_EQ(oosm.getFusionTimes().last_fused, 1005_ms);
	EXPECT_EQ(oosm.getFusionTimes().last_relative_fused, 0u);
}

// More measurements than one pass handles are fused in chunks, and delayed ones still land at their
// own times: without gating, the result matches fusing them in time order.
TEST_F(VTEOosmTest, FusesOversizedBatchInChunks)
{
	constexpr size_t count = Manager::kMaxBatchSize + 1;
	constexpr uint64_t start = 1_s;
	constexpr uint64_t step = 20_ms;
	constexpr uint64_t now = start + Manager::kHistorySize * step;
	constexpr float input = 0.5f;
	auto x = makeVec(0.f);
	auto P = makeCov(1.f);
	oosm.push(start, x, P, input);

	for (int i = 1; i < Manager::kHistorySize; ++i) {
		filter.predictState(step * 1e-6f, input, x, P, x, P);
		oosm.push(start + i * step, x, P, input);
	}

	filter.predictState(step * 1e-6f, input, x, P, x, P);

	// Newest first, so both chunks need sorting and the second one replays the first.
	ScalarMeas meas[count];

	for (size_t i = 0; i < count; ++i) {
		meas[i] = makeMeas(now - 30_ms - i * 15_ms, 0.1f * i, 0.5f);
	}

	vte::FusionResult res[count];
	oosm.fuseBatch(filter, meas, count, now, 0.f, x, P, input, res);

	TestFilter truth_filter;
	auto x_truth = makeVec(0.f);
	auto P_truth = makeCov(1.f);
	uint64_t truth_time = start;

	for (size_t i = count; i-- > 0;) {
		EXPECT_EQ(res[i].status, vte::FusionStatus::STATUS_FUSED_OOSM) << "measurement " << i;
		truth_filter.predictState((meas[i].time_us - truth_time) * 1e-6f, input, x_truth, P_truth, x_truth, P_truth);
		truth_time = meas[i].time_us;
		float innov = 0.f;
		float innov_var = 0.f;
		truth_filter.computeInnovation(x_truth, P_truth, meas[i], innov, innov_var);
		truth_filter.applyCorrection(x_truth, P_truth, (P_truth * meas[i].H) / innov_var, innov, innov_var);
	}

	truth_filter.predictState((now - truth_time) * 1e-6f, input, x_truth, P_truth, x_truth, P_truth);
	EXPECT_NEAR(x(0), x_truth(0), kTolerance);
	EXPECT_NEAR(P(0, 0), P_truth(0, 0), kTolerance);
}

} // namespace

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
 * @file VTEOosmTest.cpp
 * @brief Unit test VTEOosm.h
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "VTEOosm.h"
#include "common.h"

namespace vte = vision_target_estimator;

namespace
{

static constexpr int kStateDim = 1;
static constexpr float kTolerance = 1e-4f;

using StateVec = matrix::Vector<float, kStateDim>;
using StateCov = matrix::SquareMatrix<float, kStateDim>;

struct ScalarMeas {
	uint64_t time_us{0};
	float val{0.f};
	float unc{0.f};
	StateVec H{};
};

class TestFilter
{
public:
	float last_predict_input{NAN};
	StateVec last_H{};
	std::vector<float> applied_k;
	std::vector<float> applied_state_before;
	std::vector<float> applied_state_after;

	void resetTracking()
	{
		last_predict_input = NAN;
		applied_k.clear();
		applied_state_before.clear();
		applied_state_after.clear();
	}

	// Description:
	// Implements a simple linear dynamic model for testing: x_k+1 = x_k + input * dt.
	// This corresponds to a Transition Matrix (Phi) of 1.0 and a Control Matrix (B) of dt.
	// We add a small Process Noise (Q) to P_out to simulate uncertainty growth
	void predictState(float dt, float input, const StateVec &x_in, const StateCov &P_in,
			  StateVec &x_out, StateCov &P_out)
	{
		last_predict_input = input;

		// x_{k+1} = F * x_k + B * u
		// Linear model: x_new = x_old + input * dt
		x_out = x_in;
		x_out(0) += input * dt;

		// P_{k+1} = F * P_k * F^T + Q
		// We use F=1 (Identity), so P_new = P_old + Q
		// Adding process noise (Q = 0.1 * dt) ensures covariance grows over time
		P_out = P_in;
		P_out(0, 0) += 0.1f * dt;
	}

	void getTransitionMatrix(float dt, StateCov &phi) const
	{
		// Consistent with predictState: Phi is Identity (1.0)
		// The error dynamics match the state dynamics for linear systems.
		phi.setIdentity();
	}

	void computeInnovation(const StateVec &state, const StateCov &cov, const ScalarMeas &meas,
			       float &innov, float &innov_var)
	{
		last_H = meas.H;
		innov = meas.val - (meas.H.transpose() * state)(0, 0);
		innov_var = (meas.H.transpose() * cov * meas.H)(0, 0) + meas.unc;
	}

	void applyCorrection(StateVec &state, StateCov &cov, const StateVec &K, float innov, float innov_var)
	{
		(void)innov_var;
		applied_k.push_back(K(0));
		applied_state_before.push_back(state(0));
		state(0) += K(0) * innov;
		applied_state_after.push_back(state(0));
		cov(0, 0) = (1.f - K(0) * last_H(0)) * cov(0, 0);
	}
};

class VTEOosmTest : public ::testing::Test
{
protected:
	using Manager = vte::OOSMManager<TestFilter, kStateDim, float, 8>;

	Manager oosm{};
	TestFilter filter{};
	StateVec state{};
	StateCov cov{};
	float curr_input{0.f};

	void SetUp() override
	{
		state = makeVec(0.f);
		cov = makeCov(4.f);
		curr_input = 0.f;
		filter.resetTracking();
	}

	static StateVec makeVec(float value)
	{
		StateVec v{};
		v(0) = value;
		return v;
	}

	static StateCov makeCov(float variance)
	{
		StateCov P{};
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
		meas.H = makeVec(h_val);
		return meas;
	}

	void pushSample(uint64_t time_us, float state_val, float cov_val, float input)
	{
		oosm.push(time_us, makeVec(state_val), makeCov(cov_val), input);
	}
};

TEST_F(VTEOosmTest, FusesCurrentMeasurement)
{
	// WHY: The OOSM manager must use the fast path for fresh measurements.
	// WHAT: Ensure a current measurement updates state/covariance and returns FUSED_CURRENT.
	const uint64_t now_us = 1'000'000;
	const ScalarMeas meas = makeMeas(now_us - 10'000, 2.f, 1.f);

	const vte::FusionResult res = oosm.fuse(filter, meas, now_us, 10.f, state, cov, curr_input);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_NEAR(state(0), 1.6f, kTolerance);
	EXPECT_NEAR(cov(0, 0), 0.8f, kTolerance);
	EXPECT_EQ(filter.applied_k.size(), 1u);
}

TEST_F(VTEOosmTest, RejectsInvalidInnovationVariance)
{
	// WHY: Near-zero innovation variance should never result in a correction.
	// WHAT: Verify REJECT_COV and that the live state stays unchanged.
	const uint64_t now_us = 2'000'000;
	state = makeVec(1.f);
	cov = makeCov(0.f);
	const ScalarMeas meas = makeMeas(now_us - 5'000, 1.f, 0.f);

	const vte::FusionResult res = oosm.fuse(filter, meas, now_us, 10.f, state, cov, curr_input);

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_COV);
	EXPECT_EQ(filter.applied_k.size(), 0u);
	EXPECT_NEAR(state(0), 1.f, kTolerance);
	EXPECT_NEAR(cov(0, 0), 0.f, kTolerance);
}

TEST_F(VTEOosmTest, RejectsNisForCurrentMeasurement)
{
	// WHY: NIS gating is the primary protection against outlier measurements.
	// WHAT: Ensure large innovation triggers REJECT_NIS without changing state.
	const uint64_t now_us = 3'000'000;
	state = makeVec(0.f);
	cov = makeCov(0.1f);
	const ScalarMeas meas = makeMeas(now_us - 1'000, 10.f, 0.1f);

	const vte::FusionResult res = oosm.fuse(filter, meas, now_us, 1.f, state, cov, curr_input);

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_NIS);
	EXPECT_EQ(filter.applied_k.size(), 0u);
	EXPECT_NEAR(state(0), 0.f, kTolerance);
	EXPECT_NEAR(cov(0, 0), 0.1f, kTolerance);
}

TEST_F(VTEOosmTest, RejectsOosmHistoryGuards)
{
	// WHY: OOSM fusion must protect against empty or inconsistent history.
	// WHAT: Validate REJECT_EMPTY and REJECT_STALE behavior with history reset.
	const uint64_t meas_time_us = 900'000;
	const ScalarMeas meas = makeMeas(meas_time_us, 0.f, 1.f);

	const vte::FusionResult empty_res = oosm.fuse(filter, meas, 1'000'000, 10.f, state, cov, curr_input);
	EXPECT_EQ(empty_res.status, vte::FusionStatus::REJECT_EMPTY);

	pushSample(980'000, 0.5f, 4.f, 1.f);

	const vte::FusionResult stale_res = oosm.fuse(filter, meas, 800'000, 10.f, state, cov, curr_input);
	EXPECT_EQ(stale_res.status, vte::FusionStatus::REJECT_STALE);

	const vte::FusionResult post_reset_res = oosm.fuse(filter, meas, 1'000'000, 10.f, state, cov, curr_input);
	EXPECT_EQ(post_reset_res.status, vte::FusionStatus::REJECT_EMPTY);
}

TEST_F(VTEOosmTest, RejectsOosmOutOfRangeTimestamps)
{
	// WHY: OOSM fusion must reject timestamps outside the valid window.
	// WHAT: Check REJECT_TOO_NEW and REJECT_TOO_OLD for valid history.
	pushSample(100'000, 0.f, 4.f, 0.f);
	pushSample(200'000, 1.f, 4.f, 0.f);
	pushSample(250'000, 2.f, 4.f, 0.f);

	const uint64_t now_us = 300'000;
	const ScalarMeas meas_too_new = makeMeas(now_us + 30'000, 0.f, 1.f);
	const vte::FusionResult too_new_res = oosm.fuse(filter, meas_too_new, now_us, 10.f, state, cov, curr_input);
	EXPECT_EQ(too_new_res.status, vte::FusionStatus::REJECT_TOO_NEW);

	const ScalarMeas meas_too_old = makeMeas(90'000, 0.f, 1.f);
	const vte::FusionResult too_old_res = oosm.fuse(filter, meas_too_old, now_us, 10.f, state, cov, curr_input);
	EXPECT_EQ(too_old_res.status, vte::FusionStatus::REJECT_TOO_OLD);
}

TEST_F(VTEOosmTest, FusesOosmAndProjectsCorrection)
{
	// WHY: The projected correction path is core to delayed measurement handling.
	// WHAT: Verify projected gains update history and live state with expected values.
	pushSample(100'000, 0.f, 4.f, 0.f);
	pushSample(200'000, 1.f, 4.f, 2.f);
	pushSample(300'000, 2.f, 4.f, 2.f);

	state = makeVec(2.5f);
	cov = makeCov(4.f);
	curr_input = 9.f;

	const ScalarMeas meas = makeMeas(150'000, 1.f, 1.f);
	// Meas (150k) is fused. Floor is 100k. dt=0.05.
	// Predict P to meas: P_meas = P_floor(4.0) + Q(0.1*0.05) = 4.005.
	// K_meas = P H' / (H P H' + R) = 4.005 * 1 / (1 * 4.005 * 1 + 1) = 4.005 / 5.005 = 0.8001998...
	// Phi = 1.0. K_proj = K_meas.

	const vte::FusionResult res = oosm.fuse(filter, meas, 350'000, 10.f, state, cov, curr_input);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_OOSM);
	EXPECT_EQ(res.history_steps, 2);
	ASSERT_EQ(filter.applied_k.size(), 3u);
	EXPECT_FLOAT_EQ(filter.last_predict_input, 2.f); // Input from floor sample (0.f was at 100k, next input is 2.f at 200k)

	// We calculate expected K based on P at measurement time
	const float P_meas = 4.f + 0.1f * 0.05f; // P_in + Q*dt
	const float S = P_meas + 1.f;
	const float k_meas = P_meas / S; // ~0.8002

	// Since Phi is identity, K projects constantly (Phi * K = 1 * K)
	const float k_proj = k_meas;

	EXPECT_NEAR(filter.applied_k[0], k_proj, kTolerance); // Correction at 200k
	EXPECT_NEAR(filter.applied_k[1], k_proj, kTolerance); // Correction at 300k
	EXPECT_NEAR(filter.applied_k[2], k_proj, kTolerance); // Correction at Live (350k)

	// State Before Correction check
	EXPECT_NEAR(filter.applied_state_before[0], 1.f, kTolerance);
	EXPECT_NEAR(filter.applied_state_before[1], 2.f, kTolerance);
	EXPECT_NEAR(filter.applied_state_before[2], 2.5f, kTolerance);

	const float innov = 1.f - (0.f + 2.f * 0.05f); // Meas - (Pos_floor + vel*dt) = 1.0 - 0.1 = 0.9

	// State After Correction check
	EXPECT_NEAR(filter.applied_state_after[0], 1.f + k_proj * innov, kTolerance);
	EXPECT_NEAR(filter.applied_state_after[1], 2.f + k_proj * innov, kTolerance);
	EXPECT_NEAR(filter.applied_state_after[2], 2.5f + k_proj * innov, kTolerance);

	EXPECT_NEAR(state(0), 2.5f + k_proj * innov, kTolerance);
	EXPECT_NEAR(cov(0, 0), (1.f - k_proj) * 4.f, kTolerance); // P_live updated with K
}

TEST_F(VTEOosmTest, HandlesRingBufferWrapAroundAndBoundaries)
{
	// WHY: The OOSM manager relies on a ring buffer. Indices must wrap correctly.
	// WHAT: Push enough samples to wrap the buffer (Size=8), then perform OOSM
	// on a sample that is technically "older" in memory index but "newer" in time
	// than the wrapped limit, ensuring the iterator logic holds.

	// Standard Wrap-Around (Wide Timestamps)
	// Fill buffer (0 to 7)
	for (int i = 0; i < 8; ++i) {
		pushSample(100'000 * (i + 1), (float)i, 1.f, 0.f);
	}

	// Buffer: [100k, 200k, ..., 800k]
	// Push 9th sample (wraps to index 0, overwrites 100k)
	pushSample(900'000, 9.f, 1.f, 0.f);

	// State:
	// Newest: 900k (Index 0)
	// Oldest: 200k (Index 1)
	// Now:    950k
	// Max OOSM Lag: 500ms -> Cutoff: 450k

	// Verify Valid Fusion in the middle
	// This forces the search to traverse from Index 0 backwards to Index 4
	// And the correction loop to traverse Index 4 -> 5 -> 6 -> 7 -> 0
	const ScalarMeas meas_mid = makeMeas(550'000, 5.5f, 1.f);
	EXPECT_EQ(oosm.fuse(filter, meas_mid, 950'000, 10.f, state, cov, curr_input).status,
		  vte::FusionStatus::FUSED_OOSM) << "Failed mid-buffer fusion";

	// Verify Time Window Boundaries (The 500ms Timeout)

	// Case A: Just inside the window (450k) -> Should Fuse (floor is 400k)
	// Note: 950k - 450k = 500k. Code uses `if (diff > max)`, so 500 == 500 is OK.
	const ScalarMeas meas_edge_valid = makeMeas(450'000, 4.5f, 1.f);
	EXPECT_EQ(oosm.fuse(filter, meas_edge_valid, 950'000, 10.f, state, cov, curr_input).status,
		  vte::FusionStatus::FUSED_OOSM) << "Failed valid edge case (450k)";

	// Case B: Just outside the window (449.999k) -> Reject Too Old
	const ScalarMeas meas_edge_invalid = makeMeas(449'990, 4.5f, 1.f);
	EXPECT_EQ(oosm.fuse(filter, meas_edge_invalid, 950'000, 10.f, state, cov, curr_input).status,
		  vte::FusionStatus::REJECT_TOO_OLD) << "Failed invalid edge case (449.99k)";


	// Tight Timestamp Verification
	// We reset and use tiny time steps to prove that data is rejected
	// because it fell off the ring buffer, NOT because of the 500ms timeout.
	oosm.reset();
	filter.resetTracking();
	state = makeVec(0.f);
	cov = makeCov(4.f);

	// We need time_diff > kOosmMinTimeUs (20ms) to force OOSM logic.
	// We use 1ms steps.
	// Buffer: [10ms, 11ms, ..., 17ms] (Size 8)
	const uint64_t base_t = 10'000;
	const uint64_t step_t = 1'000;

	for (int i = 0; i < 8; ++i) {
		pushSample(base_t + step_t *i, (float)i, 1.f, 0.f);
	}

	// Buffer: [10000, 11000, ..., 17000]

	// Push 9th sample (wraps to index 0, overwrites 10000)
	// Timestamp: 18000
	pushSample(base_t + step_t * 8, 9.f, 1.f, 0.f);

	// Buffer State:
	// Newest: 18000
	// Oldest: 11000 (Index 1)

	// Set 'now' such that (now - meas) > 20ms to avoid FUSED_CURRENT
	// Let's set now to 50ms (50000).
	// Lag to oldest (11000) is 39ms (valid < 500ms).
	uint64_t tight_now = 50'000;

	// Case C: Try to fuse time=10500 (10.5ms).
	// Time diff = 50000 - 10500 = 39.5ms (> 20ms MinOosm).
	// History Check: 10500 < Oldest(11000).
	// MUST return REJECT_TOO_OLD (buffer overwritten).
	const ScalarMeas meas_overwritten = makeMeas(10'500, 0.f, 1.f);
	vte::FusionResult res_over = oosm.fuse(filter, meas_overwritten, tight_now, 10.f, state, cov, curr_input);

	EXPECT_EQ(res_over.status, vte::FusionStatus::REJECT_TOO_OLD)
			<< "Expected REJECT_TOO_OLD for overwritten sample. Got: " << (int)res_over.status;

	// Case D: Verify boundary of new oldest (11000)
	const ScalarMeas meas_oldest_valid = makeMeas(11'000, 0.f, 1.f);
	vte::FusionResult res_valid = oosm.fuse(filter, meas_oldest_valid, tight_now, 10.f, state, cov, curr_input);

	EXPECT_EQ(res_valid.status, vte::FusionStatus::FUSED_OOSM)
			<< "Expected FUSED_OOSM for oldest valid sample. Got: " << (int)res_valid.status;
}

TEST_F(VTEOosmTest, FusesOosmExactTimeMatch)
{
	// WHY: OOSM logic has a specific optimization when t_meas == t_history.
	// WHAT: Verify fusion works when measurement time aligns exactly with a sample.
	pushSample(100'000, 0.f, 4.f, 0.f);
	pushSample(200'000, 2.f, 4.f, 0.f);

	// Measurement matches the sample at 100k exactly
	const ScalarMeas meas = makeMeas(100'000, 0.5f, 1.f);
	const vte::FusionResult res = oosm.fuse(filter, meas, 250'000, 10.f, state, cov, curr_input);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_OOSM);
	// Should update 100k (floor) and project to 200k, then Live.
	EXPECT_GT(filter.applied_k.size(), 0u);
}

TEST_F(VTEOosmTest, RejectsNisDuringOosmPrediction)
{
	// WHY: A delayed measurement might look valid now, but was an outlier at the time it occurred.
	// WHAT: Provide a measurement that contradicts the history state at t_meas.
	pushSample(100'000, 0.f, 0.1f, 0.f);
	pushSample(200'000, 0.f, 0.1f, 0.f);

	// State at 150k is ~0.0. Meas is 10.0. Variance small. Should fail NIS.
	const ScalarMeas meas = makeMeas(150'000, 10.f, 0.1f);
	const vte::FusionResult res = oosm.fuse(filter, meas, 250'000, 1.f, state, cov, curr_input);

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_NIS);
	// Ensure no history corrections were applied
	EXPECT_EQ(filter.applied_k.size(), 0u);
}

} // namespace

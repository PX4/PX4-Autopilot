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
 * @file TEST_VTE_VTEOosm.cpp
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
};

struct ScalarMeas2 {
	uint64_t time_us{0};
	float val{0.f};
	float unc{0.f};
	StateVec2 H{};
};

class TestFilter
{
public:
	float last_predict_input{NAN};
	matrix::Vector<float, kStateDim> last_H{};
	std::vector<float> applied_k;
	std::vector<float> applied_state_before;
	std::vector<float> applied_state_after;

	void resetTracking()
	{
		last_predict_input = NAN;
		last_H.setZero();
		applied_k.clear();
		applied_state_before.clear();
		applied_state_after.clear();
	}

	// Description:
	// Implements a simple linear dynamic model for testing: x_k+1 = x_k + input * dt.
	// This corresponds to a Transition Matrix (Phi) of 1.0 and a Control Matrix (B) of dt.
	// We add a small Process Noise (Q) to P_out to simulate uncertainty growth
	void predictState(float dt, float input, const matrix::Vector<float, kStateDim> &x_in,
			  const matrix::SquareMatrix<float, kStateDim> &P_in,
			  matrix::Vector<float, kStateDim> &x_out,
			  matrix::SquareMatrix<float, kStateDim> &P_out)
	{
		last_predict_input = input;

		// Linear model: x_{k+1} = F * x_k + B * u
		x_out = x_in;
		x_out(0) += input * dt;

		// P_{k+1} = F * P_k * F^T + Q
		// We use F=1 (Identity), so P_new = P_old + Q
		// Adding process noise (Q = 0.1 * dt) ensures covariance grows over time
		P_out = P_in;
		P_out(0, 0) += 0.1f * dt;
	}

	void getTransitionMatrix(float dt, matrix::SquareMatrix<float, kStateDim> &phi) const
	{
		// Consistent with predictState
		phi.setIdentity();
	}

	void computeInnovation(const matrix::Vector<float, kStateDim> &state,
			       const matrix::SquareMatrix<float, kStateDim> &cov,
			       const ScalarMeas &meas, float &innov, float &innov_var)
	{
		last_H = meas.H;
		innov = meas.val - (meas.H.transpose() * state)(0, 0);
		innov_var = (meas.H.transpose() * cov * meas.H)(0, 0) + meas.unc;
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
		getTransitionMatrix(dt, phi);

		x_out = phi * x_in;
		P_out = phi * P_in * phi.transpose();
	}

	void getTransitionMatrix(float dt, StateCov2 &phi) const
	{
		phi.setIdentity();
		phi(0, 1) = dt;
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

		matrix::Matrix<float, 2, 1> K_mat = K;

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

TEST_F(VTEOosmTest, FusesCurrentMeasurement)
{
	// WHY: The OOSM manager must use the fast path for fresh measurements.
	// WHAT: Ensure a current measurement updates state/covariance and returns FUSED_CURRENT.
	const uint64_t now_us = 1_s;
	const ScalarMeas meas = makeMeas(now_us - 10_ms, 2.f, 1.f);

	const vte::FusionResult res = fuseMeas(meas, now_us, 10.f);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_NEAR(state, 1.6f, kTolerance);
	EXPECT_NEAR(cov, 0.8f, kTolerance);
	EXPECT_EQ(filter.applied_k.size(), 1u);
}

TEST_F(VTEOosmTest, RejectsInvalidInnovationVariance)
{
	// WHY: Near-zero innovation variance should never result in a correction.
	// WHAT: Verify REJECT_COV and that the live state stays unchanged.
	const uint64_t now_us = 2_s;
	state = 1.f;
	cov = 0.f;
	const ScalarMeas meas = makeMeas(now_us - 5_ms, 1.f, 0.f);

	const vte::FusionResult res = fuseMeas(meas, now_us, 10.f);

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_COV);
	EXPECT_EQ(filter.applied_k.size(), 0u);
	EXPECT_NEAR(state, 1.f, kTolerance);
	EXPECT_NEAR(cov, 0.f, kTolerance);
}

TEST_F(VTEOosmTest, RejectsNisForCurrentMeasurement)
{
	// WHY: NIS gating is the primary protection against outlier measurements.
	// WHAT: Ensure large innovation triggers REJECT_NIS without changing state.
	const uint64_t now_us = 3_s;
	state = 0.f;
	cov = 0.1f;
	const ScalarMeas meas = makeMeas(now_us - 1_ms, 10.f, 0.1f);

	const vte::FusionResult res = fuseMeas(meas, now_us, 1.f);

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_NIS);
	EXPECT_EQ(filter.applied_k.size(), 0u);
	EXPECT_NEAR(state, 0.f, kTolerance);
	EXPECT_NEAR(cov, 0.1f, kTolerance);
}

TEST_F(VTEOosmTest, RejectsOosmHistoryGuards)
{
	// WHY: OOSM fusion must protect against empty or inconsistent history.
	// WHAT: Validate REJECT_EMPTY and REJECT_STALE behavior with history reset.
	const uint64_t meas_time_us = 900_ms;
	const ScalarMeas meas = makeMeas(meas_time_us, 0.f, 1.f);

	const vte::FusionResult empty_res = fuseMeas(meas, 1_s, 10.f);
	EXPECT_EQ(empty_res.status, vte::FusionStatus::REJECT_EMPTY);

	pushSample(980_ms, 0.5f, 4.f, 1.f);

	const vte::FusionResult stale_res = fuseMeas(meas, 800_ms, 10.f);
	EXPECT_EQ(stale_res.status, vte::FusionStatus::REJECT_STALE);

	const vte::FusionResult post_reset_res = fuseMeas(meas, 1_s, 10.f);
	EXPECT_EQ(post_reset_res.status, vte::FusionStatus::REJECT_EMPTY);
}

TEST_F(VTEOosmTest, RejectsOosmOutOfRangeTimestamps)
{
	// WHY: OOSM fusion must reject timestamps outside the valid window.
	// WHAT: Check REJECT_TOO_NEW and REJECT_TOO_OLD for valid history.
	pushSample(100_ms, 0.f, 4.f, 0.f);
	pushSample(200_ms, 1.f, 4.f, 0.f);
	pushSample(250_ms, 2.f, 4.f, 0.f);

	const uint64_t now_us = 300_ms;
	const ScalarMeas meas_too_new = makeMeas(now_us + 30_ms, 0.f, 1.f);
	const vte::FusionResult too_new_res = fuseMeas(meas_too_new, now_us, 10.f);
	EXPECT_EQ(too_new_res.status, vte::FusionStatus::REJECT_TOO_NEW);

	const ScalarMeas meas_too_old = makeMeas(90_ms, 0.f, 1.f);
	const vte::FusionResult too_old_res = fuseMeas(meas_too_old, now_us, 10.f);
	EXPECT_EQ(too_old_res.status, vte::FusionStatus::REJECT_TOO_OLD);
}

TEST_F(VTEOosmTest, UsesCurrentInputWhenFloorIsNewest)
{
	// WHY: When the floor sample is the newest, OOSM should use current input.
	// WHAT: Provide a measurement between newest sample and now and verify curr_input was used.
	const uint64_t t0 = 1_s;
	pushSample(t0, 0.f, 4.f, 1.f);

	curr_input = 5.f;

	const ScalarMeas meas = makeMeas(t0 + 30_ms, 1.f, 1.f);
	const vte::FusionResult res = fuseMeas(meas, t0 + 60_ms, 10.f);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_OOSM);
	EXPECT_FLOAT_EQ(filter.last_predict_input, curr_input);
}

TEST_F(VTEOosmTest, RejectsStaleWhenNowExceedsHistoryWindow)
{
	// WHY: Large time gaps between history and now should reset the buffer.
	// WHAT: Force now to exceed the max OOSM window and verify REJECT_STALE with reset.
	const uint64_t t0 = 1_s;
	pushSample(t0, 0.f, 4.f, 0.f);

	const ScalarMeas meas = makeMeas(t0 + 50_ms, 1.f, 1.f);
	const vte::FusionResult stale_res = fuseMeas(meas, t0 + 700_ms, 10.f);
	EXPECT_EQ(stale_res.status, vte::FusionStatus::REJECT_STALE);

	const vte::FusionResult empty_res = fuseMeas(meas, t0 + 700_ms, 10.f);
	EXPECT_EQ(empty_res.status, vte::FusionStatus::REJECT_EMPTY);
}

TEST_F(VTEOosmTest, FusesOosmBetweenSamples_Simplified)
{
	// WHY: Ensure OOSM (delayed fusion) yields a result equivalent to
	// fusing the measurement sequentially at the correct time.
	// WHAT: Run a filter in which the measurement comes on time and compare it with
	// a filter with the same measurement but delayed

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

	// The "OOSM" Path (Real World Scenario), Fuse Delayed Measurement
	const vte::FusionResult oosm_res = fuseMeas(meas, t2, 100.f);

	EXPECT_EQ(oosm_res.status, vte::FusionStatus::FUSED_OOSM);
	EXPECT_EQ(oosm_res.history_steps, 2); // Should update t1 and t2 (t0 is floor, not updated)

	const float val_oosm_state = state;
	const float val_oosm_cov = cov;

	// We run a separate filter instance in perfect chronological order.
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

	// State: For linear systems, OOSM state projection is exact.
	EXPECT_NEAR(val_oosm_state, x_truth(0), kTolerance) << "State mismatch: OOSM vs Truth";

	// Covariance: Will differ slightly.
	// OOSM uses an approximation for covariance propagation (Projected P).
	// Truth uses exact sequential P propagation.
	// We increase tolerance to ~1-2% of the value to account for this approximation.
	EXPECT_NEAR(val_oosm_cov, P_truth(0, 0), 0.01f) << "Covariance mismatch exceeds approximation tolerance";
}

TEST_F(VTEOosmTest2d, FusesOosm2d_TruthComparison)
{
	// WHY: Verify that 2D OOSM (with matrix projection) matches sequential fusion.
	// WHAT: Run a filter in which the measurement comes on time and compare it with
	// a filter with the same measurement but delayed

	static constexpr uint64_t t0 = 100_ms;
	static constexpr uint64_t t_meas = 150_ms;
	static constexpr uint64_t t1 = 200_ms;
	static constexpr uint64_t t2 = 300_ms;

	static constexpr float kInput = 0.5f; // Non-zero input to test B*u dynamics
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

	// Fuse delayed measurement (OOSM)
	const vte::FusionResult oosm_res = fuseMeas(meas, t2, 100.f);
	EXPECT_EQ(oosm_res.status, vte::FusionStatus::FUSED_OOSM);

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
	matrix::Matrix<float, 2, 1> K_mat = K;
	x_truth = x_truth + K * innov;
	P_truth = P_truth - (K_mat * K.transpose()) * innov_var;

	// Predict t_meas -> t2
	float dt_2 = (t2 - t_meas) * 1e-6f;
	truth_filter.predictState(dt_2, kInput, x_truth, P_truth, x_truth, P_truth);

	// State must match closely (checking both Pos and Vel)
	EXPECT_NEAR(oosm_state(0), x_truth(0), kTolerance) << "Position Mismatch";
	EXPECT_NEAR(oosm_state(1), x_truth(1), kTolerance) << "Velocity Mismatch";

	// Covariance approx tolerance (~1-2%)
	EXPECT_NEAR(oosm_cov(0, 0), P_truth(0, 0), 0.05f) << "Cov(0,0) Mismatch";
	EXPECT_NEAR(oosm_cov(1, 1), P_truth(1, 1), 0.05f) << "Cov(1,1) Mismatch";
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
		pushSample(100_ms * (i + 1), (float)i, 1.f, 0.f);
	}

	// Buffer: [100 ms, 200 ms, ..., 800 ms]
	// Push 9th sample (wraps to index 0, overwrites 100 ms)
	pushSample(900_ms, 9.f, 1.f, 0.f);

	// State:
	// Newest: 900 ms (Index 0)
	// Oldest: 200 ms (Index 1)
	// Now:    950 ms
	// Max OOSM Lag: 500 ms -> Cutoff: 450 ms

	// Verify Valid Fusion in the middle
	// This forces the search to traverse from Index 0 backwards to Index 4
	// And the correction loop to traverse Index 4 -> 5 -> 6 -> 7 -> 0
	const ScalarMeas meas_mid = makeMeas(550_ms, 5.5f, 1.f);
	EXPECT_EQ(fuseMeas(meas_mid, 950_ms, 10.f).status,
		  vte::FusionStatus::FUSED_OOSM) << "Failed mid-buffer fusion";

	// Verify Time Window Boundaries (The 500ms Timeout)

	// Case A: Just inside the window (450 ms) -> Should Fuse (floor is 400 ms)
	// Note: 950 ms - 450 ms = 500 ms. Code uses `if (diff > max)`, so 500 == 500 is OK.
	const ScalarMeas meas_edge_valid = makeMeas(450_ms, 4.5f, 1.f);
	EXPECT_EQ(fuseMeas(meas_edge_valid, 950_ms, 10.f).status,
		  vte::FusionStatus::FUSED_OOSM) << "Failed valid edge case (450 ms)";

	// Case B: Just outside the window (449.99 ms) -> Reject Too Old
	const ScalarMeas meas_edge_invalid = makeMeas(449990_us, 4.5f, 1.f);
	EXPECT_EQ(fuseMeas(meas_edge_invalid, 950_ms, 10.f).status,
		  vte::FusionStatus::REJECT_TOO_OLD) << "Failed invalid edge case (449.99 ms)";

	// Tight Timestamp Verification
	// We reset and use tiny time steps to prove that data is rejected
	// because it fell off the ring buffer, NOT because of the 500ms timeout.
	oosm.reset();
	filter.resetTracking();
	state = 0.f;
	cov = 4.f;

	// We need time_diff > kOosmMinTimeUs (20ms) to force OOSM logic.
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

	// Set 'now' such that (now - meas) > 20ms to avoid FUSED_CURRENT
	// Let's set now to 50 ms.
	// Lag to oldest (11 ms) is 39 ms (valid < 500 ms).
	uint64_t tight_now = 50_ms;

	// Case C: Try to fuse time=10.5 ms.
	// Time diff = 50 ms - 10.5 ms = 39.5 ms (> 20 ms MinOosm).
	// History Check: 10.5 ms < Oldest (11 ms).
	// MUST return REJECT_TOO_OLD (buffer overwritten).
	const uint64_t meas_overwritten_time = 10_ms + 500_us;
	const ScalarMeas meas_overwritten = makeMeas(meas_overwritten_time, 0.f, 1.f);
	vte::FusionResult res_over = fuseMeas(meas_overwritten, tight_now, 10.f);

	EXPECT_EQ(res_over.status, vte::FusionStatus::REJECT_TOO_OLD)
			<< "Expected REJECT_TOO_OLD for overwritten sample. Got: " << (int)res_over.status;

	// Case D: Verify boundary of new oldest (11 ms)
	const ScalarMeas meas_oldest_valid = makeMeas(11_ms, 0.f, 1.f);
	vte::FusionResult res_valid = fuseMeas(meas_oldest_valid, tight_now, 10.f);

	EXPECT_EQ(res_valid.status, vte::FusionStatus::FUSED_OOSM)
			<< "Expected FUSED_OOSM for oldest valid sample. Got: " << (int)res_valid.status;
}

TEST_F(VTEOosmTest, FusesOosmExactTimeMatch)
{
	// WHY: OOSM logic has a specific optimization when t_meas == t_history.
	// WHAT: Verify fusion works when measurement time aligns exactly with a sample.
	pushSample(100_ms, 0.f, 4.f, 0.f);
	pushSample(200_ms, 2.f, 4.f, 0.f);

	// Measurement matches the sample at 100 ms exactly
	const ScalarMeas meas = makeMeas(100_ms, 0.5f, 1.f);
	const vte::FusionResult res = fuseMeas(meas, 250_ms, 10.f);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_OOSM);
	// Should update 100 ms (floor) and project to 200 ms, then Live.
	EXPECT_GT(filter.applied_k.size(), 0u);
}

TEST_F(VTEOosmTest, RejectsNisDuringOosmPrediction)
{
	// WHY: A delayed measurement might look valid now, but was an outlier at the time it occurred.
	// WHAT: Provide a measurement that contradicts the history state at t_meas.
	pushSample(100_ms, 0.f, 0.1f, 0.f);
	pushSample(200_ms, 0.f, 0.1f, 0.f);

	// State at 150 ms is ~0.0. Meas is 10.0. Variance small. Should fail NIS.
	const ScalarMeas meas = makeMeas(150_ms, 10.f, 0.1f);
	const vte::FusionResult res = fuseMeas(meas, 250_ms, 1.f);

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_NIS);
	// Ensure no history corrections were applied
	EXPECT_EQ(filter.applied_k.size(), 0u);
}

} // namespace

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

	// Scalar constant-velocity model with small process-noise growth.
	void predictState(float dt, float input, const matrix::Vector<float, kStateDim> &x_in,
			  const matrix::SquareMatrix<float, kStateDim> &P_in,
			  matrix::Vector<float, kStateDim> &x_out,
			  matrix::SquareMatrix<float, kStateDim> &P_out)
	{
		last_predict_input = input;

		x_out = x_in;
		x_out(0) += input * dt;

		P_out = P_in;
		P_out(0, 0) += 0.1f * dt;
	}

	matrix::SquareMatrix<float, kStateDim> getTransitionMatrix(float dt) const
	{
		(void)dt;
		matrix::SquareMatrix<float, kStateDim> phi{};
		phi.setIdentity();
		return phi;
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

		const StateCov2 phi = getTransitionMatrix(dt);
		StateVec2 control{};
		control(0) = 0.5f * input * dt * dt;
		control(1) = input * dt;

		x_out = phi * x_in + control;
		P_out = phi * P_in * phi.transpose();
	}

	StateCov2 getTransitionMatrix(float dt) const
	{
		StateCov2 phi{};
		phi.setIdentity();
		phi(0, 1) = dt;
		return phi;
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

	// THEN: state matches the chronological result and covariance stays within approximation tolerance.
	EXPECT_NEAR(val_oosm_state, x_truth(0), kTolerance) << "State mismatch: OOSM vs Truth";

	// Covariance: Will differ slightly.
	// OOSM uses an approximation for covariance propagation (Projected P).
	// Truth uses exact sequential P propagation.
	// We increase tolerance to ~1-2% of the value to account for this approximation.
	EXPECT_NEAR(val_oosm_cov, P_truth(0, 0), 0.01f) << "Covariance mismatch exceeds approximation tolerance";
}

// WHY: A second delayed measurement in the same history interval must use the first delayed posterior as its floor.
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
	EXPECT_NEAR(cov, P_truth(0, 0), 0.01f);
}

// WHY: Delayed samples that both land after the newest stored sample still need an inserted posterior for the next fusion.
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
	EXPECT_NEAR(cov, P_truth(0, 0), 0.01f);
}

// WHY: 2D OOSM must project corrections through the full transition matrix.
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

	// Covariance approx tolerance (~1-2%)
	EXPECT_NEAR(oosm_cov(0, 0), P_truth(0, 0), 0.05f) << "Cov(0,0) Mismatch";
	EXPECT_NEAR(oosm_cov(1, 1), P_truth(1, 1), 0.05f) << "Cov(1,1) Mismatch";
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

	// THEN: the delayed sample is rejected before any projected correction is applied.
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_NIS);
	// Ensure no history corrections were applied
	EXPECT_EQ(filter.applied_k.size(), 0u);
}

} // namespace

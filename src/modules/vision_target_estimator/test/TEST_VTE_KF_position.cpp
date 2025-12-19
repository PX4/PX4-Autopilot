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
 * @file TEST_VTE_KF_position.cpp
 * @brief Unit test KF_position.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <matrix/math.hpp>

#include "Position/KF_position.h"

namespace vte = vision_target_estimator;

namespace
{
static constexpr float kTolerance = 1e-4f;
static constexpr float kMinVar = 1e-9f;

using StateVec = matrix::Vector<float, vtest::State::size>;

StateVec makeState(float pos_rel, float vel_uav, float bias)
{
	StateVec state{};
	state(vtest::State::pos_rel) = pos_rel;
	state(vtest::State::vel_uav) = vel_uav;
	state(vtest::State::bias) = bias;
	return state;
}

StateVec makeDiag(float d0, float d1, float d2)
{
	StateVec diag{};
	diag(0) = d0;
	diag(1) = d1;
	diag(2) = d2;
	return diag;
}

vte::ScalarMeas makeMeas(uint64_t time_us, float val, float unc, const StateVec &H)
{
	vte::ScalarMeas meas{};
	meas.time_us = time_us;
	meas.val = val;
	meas.unc = unc;
	meas.H = H;
	return meas;
}

StateVec makeH(uint8_t index)
{
	StateVec H{};
	H(index) = 1.f;
	return H;
}

} // namespace

TEST(KFPositionTest, PredictUpdatesStateAndCovariance)
{
	// WHY: State propagation is the backbone of the position filter.
	// WHAT: Verify predict() matches the generated model for state and variance.
	if (vtest::State::size != 3) {
		GTEST_SKIP() << "Static VTEST state expected (size 3).";
	}

	vte::KF_position filter;
	filter.setState(makeState(1.f, 2.f, 3.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 2.f, 3.f));
	filter.setInputVar(0.4f);
	filter.setBiasVar(0.1f);

	const float dt = 0.5f;
	const float acc = 1.5f;

	filter.predict(dt, acc);

	const StateVec state = filter.getState();
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	const float expected_pos = 1.f - 2.f * dt - 0.5f * acc * dt * dt;
	const float expected_vel = 2.f + acc * dt;
	const float expected_bias = 3.f;

	EXPECT_NEAR(state(vtest::State::pos_rel), expected_pos, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), expected_vel, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), expected_bias, kTolerance);

	const float dt2 = dt * dt;
	const float dt4 = dt2 * dt2;
	const float expected_p00 = 1.f + 2.f * dt2 + 0.25f * dt4 * 0.4f;
	const float expected_p11 = 2.f + dt2 * 0.4f;
	const float expected_p22 = 3.f + 0.1f * dt;

	EXPECT_NEAR(cov_diag(vtest::State::pos_rel), expected_p00, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::vel_uav), expected_p11, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::bias), expected_p22, kTolerance);
}

TEST(KFPositionTest, InnovationMatchesH)
{
	// WHY: Innovation consistency ensures measurement mapping is correct.
	// WHAT: Check innov and innov_var against an analytic H*x and HPH^T + R.
	if (vtest::State::size != 3) {
		GTEST_SKIP() << "Static VTEST state expected (size 3).";
	}

	vte::KF_position filter;
	filter.setState(makeState(1.f, 2.f, 3.f));
	filter.setStateCovarianceDiag(makeDiag(4.f, 1.f, 1.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	const vte::ScalarMeas meas = makeMeas(1'000'000, 2.5f, 0.5f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 1'000'010, 1e6f);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_NEAR(res.innov, 1.5f, kTolerance);
	EXPECT_NEAR(res.innov_var, 4.5f, kTolerance);
}

TEST(KFPositionTest, FusesCurrentMeasurement)
{
	// WHY: The fast-path (non-OOSM) fusion is used for most measurements.
	// WHAT: Verify current fusion updates state and reduces covariance as expected.
	if (vtest::State::size != 3) {
		GTEST_SKIP() << "Static VTEST state expected (size 3).";
	}

	vte::KF_position filter;
	filter.setState(makeState(0.f, 0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(4.f, 1.f, 1.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	const vte::ScalarMeas meas = makeMeas(2'000'000, 5.f, 1.f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 2'000'010, 100.f);
	const StateVec state = filter.getState();
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_NEAR(state(vtest::State::pos_rel), 4.f, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::pos_rel), 0.8f, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::vel_uav), 1.f, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::bias), 1.f, kTolerance);
}

TEST(KFPositionTest, RejectsInvalidInnovationVariance)
{
	// WHY: Invalid innovation variance must stop the correction to avoid NaNs.
	// WHAT: Ensure REJECT_COV and unchanged state when S is near zero.
	if (vtest::State::size != 3) {
		GTEST_SKIP() << "Static VTEST state expected (size 3).";
	}

	vte::KF_position filter;
	filter.setState(makeState(1.f, 2.f, 3.f));
	filter.setStateCovarianceDiag(makeDiag(0.f, 0.f, 0.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	const vte::ScalarMeas meas = makeMeas(3'000'000, 1.f, 0.f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 3'000'010, 100.f);
	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_COV);
	EXPECT_NEAR(state(vtest::State::pos_rel), 1.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), 2.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), 3.f, kTolerance);
}

TEST(KFPositionTest, FusesOosmMeasurementWithHistory)
{
	// WHY: OOSM fusion must handle delayed measurements consistently.
	// WHAT: Provide history and fuse a delayed measurement to trigger FUSED_OOSM.
	if (vtest::State::size != 3) {
		GTEST_SKIP() << "Static VTEST state expected (size 3).";
	}

	vte::KF_position filter;
	filter.setState(makeState(0.f, 0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 1.f, 1.f));
	filter.setInputVar(0.2f);
	filter.setBiasVar(0.1f);

	const uint64_t t0 = 1'000'000;

	filter.predict(0.02f, 1.f);
	filter.pushHistory(t0);
	filter.predict(0.02f, 1.f);
	filter.pushHistory(t0 + 20'000);
	filter.predict(0.02f, 1.f);
	filter.pushHistory(t0 + 40'000);

	const StateVec state_before = filter.getState();
	const StateVec H = makeH(vtest::State::pos_rel);
	const vte::ScalarMeas meas = makeMeas(t0 + 10'000, 5.f, 0.5f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, t0 + 60'000, 100.f);
	const StateVec state_after = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_OOSM);
	EXPECT_GT(res.history_steps, 0u);
	EXPECT_GT(fabsf(state_after(vtest::State::pos_rel) - state_before(vtest::State::pos_rel)), 1e-4f);
}

TEST(KFPositionTest, ClampCovarianceToMin)
{
	// WHY: Very small covariances can destabilize future updates.
	// WHAT: Verify covariance diagonals are clamped to a minimum after correction.
	if (vtest::State::size != 3) {
		GTEST_SKIP() << "Static VTEST state expected (size 3).";
	}

	vte::KF_position filter;
	filter.setState(makeState(0.f, 0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1e-12f, 1e-12f, 1e-12f));

	const StateVec H = makeH(vtest::State::pos_rel);
	// Use a near-zero innovation to avoid NIS rejection while exercising the clamp.
	const vte::ScalarMeas meas = makeMeas(4'000'000, 0.f, 1e-4f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 4'000'010, 100.f);
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_GE(cov_diag(vtest::State::pos_rel), kMinVar);
	EXPECT_GE(cov_diag(vtest::State::vel_uav), kMinVar);
	EXPECT_GE(cov_diag(vtest::State::bias), kMinVar);
}

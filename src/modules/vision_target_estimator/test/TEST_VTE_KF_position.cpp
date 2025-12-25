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
#include <drivers/drv_hrt.h>

#include "Position/KF_position.h"

namespace vte = vision_target_estimator;

namespace
{
using namespace time_literals;

static constexpr float kTolerance = 1e-4f;
static constexpr hrt_abstime kNowOffset = 10_us;

using StateVec = matrix::Vector<float, vtest::State::size>;

#if !defined(CONFIG_VTEST_MOVING)
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
#endif

#if defined(CONFIG_VTEST_MOVING)
StateVec makeStateMoving(float pos_rel, float vel_uav, float bias, float acc_target, float vel_target)
{
	StateVec state{};
	state(vtest::State::pos_rel) = pos_rel;
	state(vtest::State::vel_uav) = vel_uav;
	state(vtest::State::bias) = bias;
	state(vtest::State::acc_target) = acc_target;
	state(vtest::State::vel_target) = vel_target;
	return state;
}

StateVec makeDiagMoving(float d0, float d1, float d2, float d3, float d4)
{
	StateVec diag{};
	diag(0) = d0;
	diag(1) = d1;
	diag(2) = d2;
	diag(3) = d3;
	diag(4) = d4;
	return diag;
}
#endif

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

#if !defined(CONFIG_VTEST_MOVING)

TEST(KFPositionTest, PredictUpdatesStateAndCovariance)
{
	// WHY: State propagation is the backbone of the position filter.
	// WHAT: Verify predict() matches the generated model for state and variance.

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

	vte::KF_position filter;
	filter.setState(makeState(1.f, 2.f, 3.f));
	filter.setStateCovarianceDiag(makeDiag(4.f, 1.f, 1.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 1_s;
	const vte::ScalarMeas meas = makeMeas(kMeasTime, 2.5f, 0.5f, H);

	const StateVec state_before = filter.getState();
	const StateVec cov_diag_before = filter.getStateCovarianceDiag();
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 1e6f);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	const float expected_innov = meas.val - state_before(vtest::State::pos_rel);
	const float expected_innov_var = cov_diag_before(vtest::State::pos_rel) + meas.unc;
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
	EXPECT_NEAR(res.innov_var, expected_innov_var, kTolerance);
}

TEST(KFPositionTest, FusesCurrentMeasurement)
{
	// WHY: The fast-path (non-OOSM) fusion is used for most measurements.
	// WHAT: Verify current fusion updates state and reduces covariance as expected.

	vte::KF_position filter;
	filter.setState(makeState(0.f, 0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(4.f, 1.f, 1.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 2_s;
	const hrt_abstime now = kMeasTime + kNowOffset;
	const vte::ScalarMeas meas = makeMeas(kMeasTime, 5.f, 1.f, H);

	const float state_pos = filter.getState()(vtest::State::pos_rel);
	const float p00 = filter.getStateCovarianceDiag()(vtest::State::pos_rel);
	const float innov = meas.val - state_pos;
	const float innov_var = p00 + meas.unc;
	const float k_pos = p00 / innov_var;
	const float expected_pos = k_pos * innov;
	const float expected_p00 = (1.f - k_pos) * p00;

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, now, 100.f);
	const StateVec state = filter.getState();
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_NEAR(state(vtest::State::pos_rel), expected_pos, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::pos_rel), expected_p00, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::vel_uav), 1.f, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::bias), 1.f, kTolerance);

	const hrt_abstime future_meas_time = now + 10_ms;
	const vte::ScalarMeas future_meas = makeMeas(future_meas_time, 6.f, 1.f, H);
	const vte::FusionResult future_res = filter.fuseScalarAtTime(future_meas, now, 100.f);
	EXPECT_EQ(future_res.status, vte::FusionStatus::FUSED_CURRENT);
}

TEST(KFPositionTest, RejectsInvalidInnovationVariance)
{
	// WHY: Invalid innovation variance must stop the correction to avoid NaNs.
	// WHAT: Ensure REJECT_COV and unchanged state when S is near zero.

	vte::KF_position filter;
	filter.setState(makeState(1.f, 2.f, 3.f));
	filter.setStateCovarianceDiag(makeDiag(0.f, 0.f, 0.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 3_s;
	const vte::ScalarMeas meas = makeMeas(kMeasTime, 1.f, 0.f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);
	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_COV);
	EXPECT_NEAR(state(vtest::State::pos_rel), 1.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), 2.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), 3.f, kTolerance);
}

TEST(KFPositionTest, RejectsOutlierNis)
{
	// WHY: NIS gating protects against large outliers.
	// WHAT: Provide a large innovation with small variance and expect REJECT_NIS.
	vte::KF_position filter;
	filter.setState(makeState(0.f, 0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(0.01f, 0.01f, 0.01f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 3500_ms;
	const vte::ScalarMeas meas = makeMeas(kMeasTime, 10.f, 0.01f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 0.1f);
	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_NIS);
	EXPECT_NEAR(state(vtest::State::pos_rel), 0.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), 0.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), 0.f, kTolerance);
}

TEST(KFPositionTest, FusesMeasurementWithBiasInH)
{
	// WHY: GNSS with bias must use H(pos_rel)=1 and H(bias)=1.
	// WHAT: Verify innovation and corrections for a joint position+bias observation.
	vte::KF_position filter;
	filter.setState(makeState(1.f, 0.f, 2.f));
	filter.setStateCovarianceDiag(makeDiag(2.f, 1.f, 3.f));

	StateVec H{};
	H(vtest::State::pos_rel) = 1.f;
	H(vtest::State::bias) = 1.f;

	static constexpr hrt_abstime kMeasTime = 3600_ms;
	const vte::ScalarMeas meas = makeMeas(kMeasTime, 10.f, 1.f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);
	const StateVec state = filter.getState();

	const float expected_innov = 10.f - (1.f + 2.f);
	const float S = 2.f + 3.f + 1.f;
	const float k_pos = 2.f / S;
	const float k_bias = 3.f / S;

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
	EXPECT_NEAR(state(vtest::State::pos_rel), 1.f + k_pos * expected_innov, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), 2.f + k_bias * expected_innov, kTolerance);
}

TEST(KFPositionTest, FusesOosmMeasurementWithHistory)
{
	// WHY: OOSM fusion must handle delayed measurements consistently.
	// WHAT: Provide history and fuse a delayed measurement to trigger FUSED_OOSM.

	vte::KF_position filter;
	filter.setState(makeState(0.f, 0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 1.f, 1.f));
	filter.setInputVar(0.2f);
	filter.setBiasVar(0.1f);

	static constexpr hrt_abstime kT0 = 1_s;
	static constexpr hrt_abstime kHistoryStep = 20_ms;
	static constexpr hrt_abstime kMeasOffset = 10_ms;
	static constexpr hrt_abstime kNowOffsetLocal = 60_ms;

	filter.predict(0.02f, 1.f);
	filter.pushHistory(kT0);
	filter.predict(0.02f, 1.f);
	filter.pushHistory(kT0 + kHistoryStep);
	filter.predict(0.02f, 1.f);
	filter.pushHistory(kT0 + kHistoryStep * 2);

	const StateVec state_before = filter.getState();
	const StateVec H = makeH(vtest::State::pos_rel);
	const vte::ScalarMeas meas = makeMeas(kT0 + kMeasOffset, 5.f, 0.5f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kT0 + kNowOffsetLocal, 100.f);
	const StateVec state_after = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_OOSM);
	EXPECT_GT(res.history_steps, 0u);
	EXPECT_GT(fabsf(state_after(vtest::State::pos_rel) - state_before(vtest::State::pos_rel)), 1e-4f);
}

TEST(KFPositionTest, ClampCovarianceToMin)
{
	// WHY: Very small covariances can destabilize future updates.
	// WHAT: Verify covariance diagonals are clamped to a minimum after correction.

	vte::KF_position filter;
	filter.setState(makeState(0.f, 0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1e-12f, 1e-12f, 1e-12f));

	const StateVec H = makeH(vtest::State::pos_rel);
	// Use a near-zero innovation to avoid NIS rejection while exercising the clamp.
	static constexpr hrt_abstime kMeasTime = 4_s;
	const vte::ScalarMeas meas = makeMeas(kMeasTime, 0.f, 1e-4f, H);

	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_GE(cov_diag(vtest::State::pos_rel), vte::KF_position::kMinVar);
	EXPECT_GE(cov_diag(vtest::State::vel_uav), vte::KF_position::kMinVar);
	EXPECT_GE(cov_diag(vtest::State::bias), vte::KF_position::kMinVar);
}

#endif // !defined(CONFIG_VTEST_MOVING)

#if defined(CONFIG_VTEST_MOVING)
TEST(KFPositionTest, PredictMovingTargetAddsTargetTerms)
{
	// WHY: Moving-target model adds target velocity/acceleration to relative position.
	// WHAT: Validate state propagation and acc_target random-walk covariance.
	vte::KF_position filter;
	filter.setState(makeStateMoving(1.f, 2.f, 3.f, 0.4f, -0.5f));
	filter.setStateCovarianceDiag(makeDiagMoving(0.f, 0.f, 0.f, 0.f, 0.f));
	filter.setInputVar(0.f);
	filter.setBiasVar(0.f);
	filter.setTargetAccVar(0.2f);

	const float dt = 0.5f;
	const float acc_uav = 1.2f;

	filter.predict(dt, acc_uav);

	const StateVec state = filter.getState();
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	const float dt2 = dt * dt;
	const float expected_pos = 1.f - 2.f * dt - 0.5f * acc_uav * dt2 + (-0.5f) * dt + 0.5f * dt2 * 0.4f;
	const float expected_vel = 2.f + acc_uav * dt;
	const float expected_bias = 3.f;
	const float expected_acc = 0.4f;
	const float expected_vel_target = -0.5f + dt * 0.4f;

	EXPECT_NEAR(state(vtest::State::pos_rel), expected_pos, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), expected_vel, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), expected_bias, kTolerance);
	EXPECT_NEAR(state(vtest::State::acc_target), expected_acc, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_target), expected_vel_target, kTolerance);

	EXPECT_NEAR(cov_diag(vtest::State::acc_target), 0.2f * dt, kTolerance);
}
#endif // defined(CONFIG_VTEST_MOVING)

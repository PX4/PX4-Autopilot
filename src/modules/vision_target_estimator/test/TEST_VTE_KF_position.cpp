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
 * @file TEST_VTE_KF_position.cpp
 * @brief Unit test KF_position.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <matrix/math.hpp>
#include <drivers/drv_hrt.h>

#include "Position/KF_position.h"

namespace vte = vision_target_estimator;

namespace
{
using namespace time_literals;

static constexpr float kTolerance = 1e-4f;
static constexpr float kTightTolerance = 1e-7f;
#if !defined(CONFIG_VTEST_MOVING)
static constexpr hrt_abstime kNowOffset = 10_us;
#endif // !defined(CONFIG_VTEST_MOVING)

using StateVec = vte::KF_position::VectorState;
using StateCov = vte::KF_position::SquareMatrixState;
using ScalarMeas = vte::KF_position::ScalarMeas;

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

void setFilterState(vte::KF_position &filter, const StateVec &state, const StateVec &cov_diag)
{
	filter.setState(state);
	filter.setStateCovarianceDiag(cov_diag);
}

void expectSymmetric(const StateCov &cov, float tolerance = kTolerance)
{
	for (size_t row = 0; row < vte::KF_position::kStateSize; row++) {
		for (size_t col = 0; col < vte::KF_position::kStateSize; col++) {
			EXPECT_NEAR(cov(row, col), cov(col, row), tolerance);
		}
	}
}

void expectStateAndCovUnchanged(const vte::KF_position &filter, const StateVec &state_before,
				const StateCov &cov_before)
{
	const StateVec state_after = filter.getState();
	const StateCov cov_after = filter.getStateCovariance();

	for (size_t i = 0; i < vte::KF_position::kStateSize; i++) {
		EXPECT_NEAR(state_after(i), state_before(i), kTolerance);

		for (size_t j = 0; j < vte::KF_position::kStateSize; j++) {
			EXPECT_NEAR(cov_after(i, j), cov_before(i, j), kTolerance);
		}
	}
}

#if !defined(CONFIG_VTEST_MOVING)
ScalarMeas makeMeas(uint64_t time_us, float val, float unc, const StateVec &H)
{
	ScalarMeas meas{};
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

hrt_abstime makeFusionNow(hrt_abstime meas_time)
{
	return meas_time + kNowOffset;
}
#endif // !defined(CONFIG_VTEST_MOVING)

} // namespace

// WHY: Invalid predict inputs must not corrupt state or covariance.
// WHAT: Exercise the negative-dt, non-finite-dt, and non-finite-acceleration guards.
TEST(KFPositionTest, PredictRejectsInvalidInputs)
{
	StateVec state{};
	StateVec cov_diag{};

	for (size_t i = 0; i < vte::KF_position::kStateSize; i++) {
		state(i) = 1.f + static_cast<float>(i);
		cov_diag(i) = 0.1f * (1.f + static_cast<float>(i));
	}

	vte::KF_position filter;
	setFilterState(filter, state, cov_diag);

	const StateVec state_before = filter.getState();
	const StateCov cov_before = filter.getStateCovariance();

	filter.predict(-0.01f, 1.f);
	expectStateAndCovUnchanged(filter, state_before, cov_before);

	filter.predict(NAN, 1.f);
	expectStateAndCovUnchanged(filter, state_before, cov_before);

	filter.predict(0.01f, NAN);
	expectStateAndCovUnchanged(filter, state_before, cov_before);
}

#if !defined(CONFIG_VTEST_MOVING)

// WHY:   State propagation and process-noise integration are the backbone of the
//        position filter. A regression here corrupts every downstream fusion.
// WHAT:  Predict one step from a non-zero prior and check both the propagated state
//        and the full Q-augmented covariance against the analytic discrete model.
TEST(KFPositionTest, PredictUpdatesStateAndCovariance)
{
	// Inputs (named so a failing line points at a labelled quantity, not a literal).
	constexpr float kInitPosRel  = 1.f;   // m
	constexpr float kInitVelUav  = 2.f;   // m/s
	constexpr float kInitBias    = 3.f;   // m
	constexpr float kPriorVarPos = 1.f;   // m^2 (diagonal prior)
	constexpr float kPriorVarVel = 2.f;   // (m/s)^2
	constexpr float kPriorVarBias = 3.f;  // m^2
	constexpr float kQAcc  = 0.4f;        // UAV-acceleration PSD, m^2/s^3
	constexpr float kQBias = 0.1f;        // bias random-walk PSD, m^2/s
	constexpr float kDt    = 0.5f;        // s
	constexpr float kAccUav = 1.5f;       // m/s^2 (deterministic input)

	vte::KF_position filter;
	setFilterState(filter, makeState(kInitPosRel, kInitVelUav, kInitBias),
		       makeDiag(kPriorVarPos, kPriorVarVel, kPriorVarBias));
	filter.setInputVar(kQAcc);
	filter.setBiasVar(kQBias);

	filter.predict(kDt, kAccUav);

	// Reference state: discrete prediction model (static target)
	//   r_{k+1}     = r_k - dt * v_uav_k - 0.5 * dt^2 * acc_uav
	//   v_uav_{k+1} = v_uav_k + dt * acc_uav
	//   b_{k+1}     = b_k
	const float dt2 = kDt * kDt;
	const float dt3 = dt2 * kDt;
	const float expected_pos_rel = kInitPosRel - kDt * kInitVelUav - 0.5f * dt2 * kAccUav;
	const float expected_vel_uav = kInitVelUav + kDt * kAccUav;
	const float expected_bias    = kInitBias;

	const StateVec state = filter.getState();
	EXPECT_NEAR(state(vtest::State::pos_rel), expected_pos_rel, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), expected_vel_uav, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias),    expected_bias,    kTolerance);

	// Reference covariance: P_new = Phi * P_prior * Phi^T + Q.
	// For diagonal P_prior and the static Phi (only off-diagonal entry is Phi[r,v_uav] = -dt):
	//   (Phi P Phi^T)[r,   r]   = var(r) + dt^2 * var(v_uav)
	//   (Phi P Phi^T)[r,   v_uav] = -dt * var(v_uav)
	//   (Phi P Phi^T)[v_uav,v_uav] = var(v_uav)
	//   (Phi P Phi^T)[b,   b]   = var(b)
	// Q from continuous white UAV-acceleration noise + bias random walk:
	//   Q[r,     r]     = q_acc * dt^3 / 3
	//   Q[r,     v_uav] = -q_acc * dt^2 / 2
	//   Q[v_uav, v_uav] = q_acc * dt
	//   Q[b,     b]     = q_bias * dt
	const float expected_p_r_r       = kPriorVarPos + dt2 * kPriorVarVel + kQAcc * dt3 / 3.f;
	const float expected_p_r_v       = -kDt * kPriorVarVel               - kQAcc * dt2 / 2.f;
	const float expected_p_v_v       = kPriorVarVel                       + kQAcc * kDt;
	const float expected_p_b_b       = kPriorVarBias                      + kQBias * kDt;

	const StateCov cov = filter.getStateCovariance();
	EXPECT_NEAR(cov(vtest::State::pos_rel, vtest::State::pos_rel), expected_p_r_r, kTolerance);
	EXPECT_NEAR(cov(vtest::State::pos_rel, vtest::State::vel_uav), expected_p_r_v, kTolerance);
	EXPECT_NEAR(cov(vtest::State::vel_uav, vtest::State::pos_rel), expected_p_r_v, kTolerance);
	EXPECT_NEAR(cov(vtest::State::vel_uav, vtest::State::vel_uav), expected_p_v_v, kTolerance);
	EXPECT_NEAR(cov(vtest::State::bias,    vtest::State::bias),    expected_p_b_b, kTolerance);
	expectSymmetric(cov);
}

// WHY:   Continuous-time process noise should not depend on how the predict
//        interval is sliced. dt-invariance is what makes the spectral-density
//        parameterization transferable across update rates.
// WHAT:  One predict step of dt and two predict steps of dt/2 from the same zero
//        prior must produce the same posterior covariance.
TEST(KFPositionTest, PredictCovarianceIsSplitStepInvariant)
{
	constexpr float kQAcc   = 0.4f;
	constexpr float kQBias  = 0.1f;
	constexpr float kFullDt = 0.02f;
	constexpr float kHalfDt = 0.01f;

	const StateVec zero_state = makeState(0.f, 0.f, 0.f);
	const StateVec zero_cov   = makeDiag(0.f, 0.f, 0.f);

	vte::KF_position single_step;
	vte::KF_position split_step;
	setFilterState(single_step, zero_state, zero_cov);
	setFilterState(split_step,  zero_state, zero_cov);
	single_step.setInputVar(kQAcc);
	single_step.setBiasVar(kQBias);
	split_step.setInputVar(kQAcc);
	split_step.setBiasVar(kQBias);

	single_step.predict(kFullDt, 0.f);
	split_step.predict(kHalfDt, 0.f);
	split_step.predict(kHalfDt, 0.f);

	const StateCov cov_single = single_step.getStateCovariance();
	const StateCov cov_split  = split_step.getStateCovariance();

	for (size_t row = 0; row < vte::KF_position::kStateSize; row++) {
		for (size_t col = 0; col < vte::KF_position::kStateSize; col++) {
			EXPECT_NEAR(cov_single(row, col), cov_split(row, col), kTightTolerance)
					<< "Split-step mismatch at (" << row << ", " << col << ")";
		}
	}
}

// WHY: Innovation consistency ensures measurement mapping is correct.
// WHAT: Check innov and innov_var against an analytic H*x and HPH^T + R.
TEST(KFPositionTest, InnovationMatchesH)
{
	// GIVEN: a filter with a position measurement model and a finite prior covariance.
	vte::KF_position filter;
	setFilterState(filter, makeState(1.f, 2.f, 3.f), makeDiag(4.f, 1.f, 1.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 1_s;
	const ScalarMeas meas = makeMeas(kMeasTime, 2.5f, 0.5f, H);

	const StateVec state_before = filter.getState();
	const StateVec cov_diag_before = filter.getStateCovarianceDiag();

	// WHEN: we fuse the measurement with a permissive NIS threshold.
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, makeFusionNow(kMeasTime), 1e6f);

	// THEN: the reported innovation terms equal the analytic residual and variance.
	const float expected_innov = meas.val - state_before(vtest::State::pos_rel);
	const float expected_innov_var = cov_diag_before(vtest::State::pos_rel) + meas.unc;

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
	EXPECT_NEAR(res.innov_var, expected_innov_var, kTolerance);
}

// WHY: The fast-path (non-OOSM) fusion is used for most measurements.
// WHAT: Verify current fusion updates state and reduces covariance as expected.
TEST(KFPositionTest, FusesCurrentMeasurement)
{
	// GIVEN: a filter with uncertainty on relative position and a matching position measurement.
	vte::KF_position filter;
	setFilterState(filter, makeState(0.f, 0.f, 0.f), makeDiag(4.f, 1.f, 1.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 2_s;
	const hrt_abstime now = makeFusionNow(kMeasTime);
	const ScalarMeas meas = makeMeas(kMeasTime, 5.f, 1.f, H);

	const float state_pos = filter.getState()(vtest::State::pos_rel);
	const float p00 = filter.getStateCovarianceDiag()(vtest::State::pos_rel);
	const float innov = meas.val - state_pos;
	const float innov_var = p00 + meas.unc;
	const float k_pos = p00 / innov_var;
	const float expected_pos = state_pos + k_pos * innov;
	const float expected_p00 = (1.f - k_pos) * p00;

	// WHEN: we fuse the current measurement and then one stamped slightly in the future.
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, now, 100.f);
	const StateVec state = filter.getState();
	const StateVec cov_diag = filter.getStateCovarianceDiag();
	const hrt_abstime future_meas_time = now + 10_ms;
	const ScalarMeas future_meas = makeMeas(future_meas_time, 6.f, 1.f, H);
	const vte::FusionResult future_res = filter.fuseScalarAtTime(future_meas, now, 100.f);

	// THEN: only the observed state is corrected and future stamps still use the current path.
	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(state(vtest::State::pos_rel), expected_pos, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::pos_rel), expected_p00, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::vel_uav), 1.f, kTolerance);
	EXPECT_NEAR(cov_diag(vtest::State::bias), 1.f, kTolerance);
	EXPECT_EQ(future_res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
}

// WHY: Invalid innovation variance must stop the correction to avoid NaNs.
// WHAT: Ensure REJECT_COV and unchanged state when S is near zero.
TEST(KFPositionTest, RejectsInvalidInnovationVariance)
{
	// GIVEN: a measurement with zero uncertainty applied to a zero-covariance prior.
	vte::KF_position filter;
	setFilterState(filter, makeState(1.f, 2.f, 3.f), makeDiag(0.f, 0.f, 0.f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 3_s;
	const ScalarMeas meas = makeMeas(kMeasTime, 1.f, 0.f, H);

	// WHEN: we attempt to fuse the measurement.
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, makeFusionNow(kMeasTime), 100.f);

	// THEN: the filter rejects the correction and preserves the full state.
	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_COV);
	EXPECT_NEAR(state(vtest::State::pos_rel), 1.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), 2.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), 3.f, kTolerance);
}

// WHY: NIS gating protects against large outliers.
// WHAT: Provide a large innovation with small variance and expect REJECT_NIS.
TEST(KFPositionTest, RejectsOutlierNis)
{
	// GIVEN: a tightly constrained prior and a large position residual.
	vte::KF_position filter;
	setFilterState(filter, makeState(0.f, 0.f, 0.f), makeDiag(0.01f, 0.01f, 0.01f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 3500_ms;
	const ScalarMeas meas = makeMeas(kMeasTime, 10.f, 0.01f, H);

	// WHEN: we fuse with a tight NIS threshold.
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, makeFusionNow(kMeasTime), 0.1f);

	// THEN: the measurement is rejected and the state remains untouched.
	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_NIS);
	EXPECT_NEAR(state(vtest::State::pos_rel), 0.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav), 0.f, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), 0.f, kTolerance);
}

// WHY: GNSS with bias must use H(pos_rel)=1 and H(bias)=1.
// WHAT: Verify innovation and corrections for a joint position+bias observation.
TEST(KFPositionTest, FusesMeasurementWithBiasInH)
{
	// GIVEN: a filter whose position and bias both contribute to the scalar measurement.
	vte::KF_position filter;
	setFilterState(filter, makeState(1.f, 0.f, 2.f), makeDiag(2.f, 1.f, 3.f));

	StateVec H{};
	H(vtest::State::pos_rel) = 1.f;
	H(vtest::State::bias) = 1.f;

	static constexpr hrt_abstime kMeasTime = 3600_ms;
	const ScalarMeas meas = makeMeas(kMeasTime, 10.f, 1.f, H);

	// WHEN: we fuse the combined position-plus-bias observation.
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, makeFusionNow(kMeasTime), 100.f);

	// THEN: the innovation is shared across both observed states according to their variance.
	const StateVec state = filter.getState();

	const float expected_innov = 10.f - (1.f + 2.f);
	const float innov_var = 2.f + 3.f + 1.f;
	const float k_pos = 2.f / innov_var;
	const float k_bias = 3.f / innov_var;

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
	EXPECT_NEAR(state(vtest::State::pos_rel), 1.f + k_pos * expected_innov, kTolerance);
	EXPECT_NEAR(state(vtest::State::bias), 2.f + k_bias * expected_innov, kTolerance);
}

// WHY: OOSM fusion must handle delayed measurements consistently.
// WHAT: Provide history and fuse a delayed measurement to trigger FUSED_OOSM.
TEST(KFPositionTest, FusesOosmMeasurementWithHistory)
{
	// GIVEN: a filter with three history snapshots spanning the delayed measurement time.
	vte::KF_position filter;
	setFilterState(filter, makeState(0.f, 0.f, 0.f), makeDiag(1.f, 1.f, 1.f));
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
	const ScalarMeas meas = makeMeas(kT0 + kMeasOffset, 5.f, 0.5f, H);

	// WHEN: we fuse the delayed measurement after newer samples are already in history.
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kT0 + kNowOffsetLocal, 100.f);

	// THEN: the OOSM path is used and the delayed correction changes the current state.
	const StateVec state_after = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_OOSM);
	EXPECT_GT(res.history_steps, 0u);
	EXPECT_GT(fabsf(state_after(vtest::State::pos_rel) - state_before(vtest::State::pos_rel)), 1e-4f);
}

// WHY: Very small covariances can destabilize future updates.
// WHAT: Verify covariance diagonals are clamped to a minimum after correction.
TEST(KFPositionTest, ClampCovarianceToMin)
{
	// GIVEN: a filter with near-zero covariance and a near-zero innovation measurement.
	vte::KF_position filter;
	setFilterState(filter, makeState(0.f, 0.f, 0.f), makeDiag(1e-12f, 1e-12f, 1e-12f));

	const StateVec H = makeH(vtest::State::pos_rel);
	static constexpr hrt_abstime kMeasTime = 4_s;
	const ScalarMeas meas = makeMeas(kMeasTime, 0.f, 1e-4f, H);

	// WHEN: we fuse without triggering NIS rejection.
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, makeFusionNow(kMeasTime), 100.f);

	// THEN: every diagonal entry is clamped to at least the configured minimum variance.
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_GE(cov_diag(vtest::State::pos_rel), vte::KF_position::kMinVar);
	EXPECT_GE(cov_diag(vtest::State::vel_uav), vte::KF_position::kMinVar);
	EXPECT_GE(cov_diag(vtest::State::bias), vte::KF_position::kMinVar);
}

#endif // !defined(CONFIG_VTEST_MOVING)

#if defined(CONFIG_VTEST_MOVING)
// WHY:   In moving mode, target velocity and acceleration couple into pos_rel,
//        and white jerk on acc_target injects an integrated noise block. Both
//        must be present and correct.
// WHAT:  Predict one step with only target-jerk PSD enabled (UAV and bias PSDs
//        zeroed). State propagation must include the target-motion coupling; the
//        resulting covariance must equal the closed-form constant-jerk Q block
//        because the prior covariance is zero.
TEST(KFPositionTest, PredictMovingTargetAddsTargetTerms)
{
	// Initial state: non-zero so the propagation coupling is observable.
	constexpr float kInitPosRel    = 1.f;     // m
	constexpr float kInitVelUav    = 2.f;     // m/s
	constexpr float kInitBias      = 3.f;     // m
	constexpr float kInitAccTarget = 0.4f;    // m/s^2
	constexpr float kInitVelTarget = -0.5f;   // m/s
	// Process noise: only target jerk is active so the resulting Q isolates that path.
	constexpr float kQJerk  = 0.2f;           // target jerk PSD, (m/s^2)^2/s
	constexpr float kDt     = 0.5f;           // s
	constexpr float kAccUav = 1.2f;           // m/s^2

	vte::KF_position filter;
	setFilterState(filter,
		       makeStateMoving(kInitPosRel, kInitVelUav, kInitBias, kInitAccTarget, kInitVelTarget),
		       makeDiagMoving(0.f, 0.f, 0.f, 0.f, 0.f));   // zero prior, so cov(after) == Q
	filter.setInputVar(0.f);
	filter.setBiasVar(0.f);
	filter.setTargetAccVar(kQJerk);

	filter.predict(kDt, kAccUav);

	// Reference state: moving-target discrete prediction model
	//   r_{k+1}        = r - dt * (v_uav - v_target) - 0.5 * dt^2 * (acc_uav - acc_target)
	//   v_uav_{k+1}    = v_uav + dt * acc_uav
	//   v_target_{k+1} = v_target + dt * acc_target
	//   acc_target, b unchanged
	const float dt2 = kDt * kDt;
	const float dt3 = dt2 * kDt;
	const float dt4 = dt2 * dt2;
	const float dt5 = dt4 * kDt;
	const float expected_pos_rel    = kInitPosRel
					  - kDt * (kInitVelUav - kInitVelTarget)
					  - 0.5f * dt2 * (kAccUav - kInitAccTarget);
	const float expected_vel_uav    = kInitVelUav + kDt * kAccUav;
	const float expected_vel_target = kInitVelTarget + kDt * kInitAccTarget;
	const float expected_acc_target = kInitAccTarget;
	const float expected_bias       = kInitBias;

	const StateVec state = filter.getState();
	EXPECT_NEAR(state(vtest::State::pos_rel),    expected_pos_rel,    kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_uav),    expected_vel_uav,    kTolerance);
	EXPECT_NEAR(state(vtest::State::bias),       expected_bias,       kTolerance);
	EXPECT_NEAR(state(vtest::State::acc_target), expected_acc_target, kTolerance);
	EXPECT_NEAR(state(vtest::State::vel_target), expected_vel_target, kTolerance);

	// Reference Q from continuous white jerk on acc_target, integrated through the
	// constant-jerk model. Prior cov is zero, so cov(after predict) == Q exactly.
	//   Q[r,        r]          = q * dt^5 / 20
	//   Q[r,        v_target]   = q * dt^4 / 8
	//   Q[r,        acc_target] = q * dt^3 / 6
	//   Q[v_target, v_target]   = q * dt^3 / 3
	//   Q[v_target, acc_target] = q * dt^2 / 2
	//   Q[acc_target,acc_target]= q * dt
	const float expected_q_r_r   = kQJerk * dt5 / 20.f;
	const float expected_q_r_vt  = kQJerk * dt4 / 8.f;
	const float expected_q_r_at  = kQJerk * dt3 / 6.f;
	const float expected_q_vt_vt = kQJerk * dt3 / 3.f;
	const float expected_q_vt_at = kQJerk * dt2 / 2.f;
	const float expected_q_at_at = kQJerk * kDt;

	const StateCov cov = filter.getStateCovariance();
	EXPECT_NEAR(cov(vtest::State::pos_rel,    vtest::State::pos_rel),    expected_q_r_r,   kTolerance);
	EXPECT_NEAR(cov(vtest::State::pos_rel,    vtest::State::vel_target), expected_q_r_vt,  kTolerance);
	EXPECT_NEAR(cov(vtest::State::pos_rel,    vtest::State::acc_target), expected_q_r_at,  kTolerance);
	EXPECT_NEAR(cov(vtest::State::vel_target, vtest::State::vel_target), expected_q_vt_vt, kTolerance);
	EXPECT_NEAR(cov(vtest::State::vel_target, vtest::State::acc_target), expected_q_vt_at, kTolerance);
	EXPECT_NEAR(cov(vtest::State::acc_target, vtest::State::acc_target), expected_q_at_at, kTolerance);
	expectSymmetric(cov);
}

// WHY:   Moving mode shares the same continuous white-UAV-acceleration model as
//        static mode. Confirms the larger state vector does not perturb the
//        (pos_rel, vel_uav) Q block, and that the target sub-block stays at zero
//        when only UAV-input noise is active.
// WHAT:  Predict one step with only UAV-input PSD enabled (bias and jerk PSDs
//        zeroed). cov(pos_rel, vel_uav) must match the static-mode integrated Q;
//        cov(acc_target, acc_target) and cov(vel_target, vel_target) must be zero.
TEST(KFPositionTest, PredictMovingTargetAddsUavInputNoise)
{
	constexpr float kQAcc = 0.4f;   // UAV-acceleration PSD, m^2/s^3
	constexpr float kDt   = 0.5f;

	vte::KF_position filter;
	setFilterState(filter,
		       makeStateMoving(0.f, 0.f, 0.f, 0.f, 0.f),
		       makeDiagMoving(0.f, 0.f, 0.f, 0.f, 0.f));   // zero prior, so cov(after) == Q
	filter.setInputVar(kQAcc);
	filter.setBiasVar(0.f);
	filter.setTargetAccVar(0.f);

	filter.predict(kDt, 0.f);

	// Reference Q from continuous white UAV-acceleration noise (same closed-form
	// block as in the static test):
	//   Q[r,     r]     = q * dt^3 / 3
	//   Q[r,     v_uav] = -q * dt^2 / 2
	//   Q[v_uav, v_uav] = q * dt
	// Target sub-block must remain identically zero because no jerk noise was injected.
	const float dt2 = kDt * kDt;
	const float dt3 = dt2 * kDt;
	const float expected_q_r_r = kQAcc * dt3 / 3.f;
	const float expected_q_r_v = -kQAcc * dt2 / 2.f;
	const float expected_q_v_v = kQAcc * kDt;

	const StateCov cov = filter.getStateCovariance();
	EXPECT_NEAR(cov(vtest::State::pos_rel,    vtest::State::pos_rel),    expected_q_r_r, kTolerance);
	EXPECT_NEAR(cov(vtest::State::pos_rel,    vtest::State::vel_uav),    expected_q_r_v, kTolerance);
	EXPECT_NEAR(cov(vtest::State::vel_uav,    vtest::State::pos_rel),    expected_q_r_v, kTolerance);
	EXPECT_NEAR(cov(vtest::State::vel_uav,    vtest::State::vel_uav),    expected_q_v_v, kTolerance);
	EXPECT_NEAR(cov(vtest::State::acc_target, vtest::State::acc_target), 0.f, kTolerance);
	EXPECT_NEAR(cov(vtest::State::vel_target, vtest::State::vel_target), 0.f, kTolerance);
	expectSymmetric(cov);
}
#endif // defined(CONFIG_VTEST_MOVING)

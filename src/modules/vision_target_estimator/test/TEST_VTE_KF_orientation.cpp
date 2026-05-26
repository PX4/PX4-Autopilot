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
 * @file TEST_VTE_KF_orientation.cpp
 * @brief Unit test KF_orientation.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <matrix/math.hpp>
#include <drivers/drv_hrt.h>

#include "Orientation/KF_orientation.h"

namespace vte = vision_target_estimator;

namespace
{
using namespace time_literals;

static constexpr float kTolerance = 1e-4f;
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kYawNearPi = 3.13f;
static constexpr hrt_abstime kNowOffset = 10_us;

using StateVec = vte::KF_orientation::VectorState;
using StateCov = vte::KF_orientation::SquareMatrixState;
using ScalarMeas = vte::KF_orientation::ScalarMeas;

StateVec makeState(float yaw, float yaw_rate)
{
	StateVec state{};
	state(vte::KF_orientation::kYaw) = yaw;
	state(vte::KF_orientation::kYawRate) = yaw_rate;
	return state;
}

StateVec makeDiag(float d0, float d1)
{
	StateVec diag{};
	diag(0) = d0;
	diag(1) = d1;
	return diag;
}

ScalarMeas makeMeas(uint64_t time_us, float val, float unc, const StateVec &H)
{
	ScalarMeas meas{};
	meas.time_us = time_us;
	meas.val = val;
	meas.unc = unc;
	meas.H = H;
	return meas;
}

StateVec makeH(size_t index)
{
	StateVec H{};
	H(index) = 1.f;
	return H;
}

void expectStateAndCovUnchanged(const vte::KF_orientation &filter, const StateVec &state_before,
				const StateCov &cov_before)
{
	const StateVec state_after = filter.getState();
	const StateCov cov_after = filter.getStateCovariance();

	for (size_t i = 0; i < vte::KF_orientation::kSize; i++) {
		EXPECT_NEAR(state_after(i), state_before(i), kTolerance);

		for (size_t j = 0; j < vte::KF_orientation::kSize; j++) {
			EXPECT_NEAR(cov_after(i, j), cov_before(i, j), kTolerance);
		}
	}
}

} // namespace

// WHY: Yaw must remain in [-pi, pi] to avoid discontinuities.
// WHAT: Predict across the wrap and ensure yaw is normalized.
TEST(KFOrientationTest, PredictWrapsYaw)
{
	// GIVEN: a yaw near +pi with a positive yaw-rate.
	vte::KF_orientation filter;
	const float yaw = 3.1f;
	const float yaw_rate = 1.0f;
	const float dt = 1.0f;
	filter.setState(makeState(yaw, yaw_rate));
	filter.setStateCovarianceDiag(makeDiag(0.f, 0.f));

	// WHEN: we advance the filter by one second (crossing pi).
	filter.predict(dt);

	// THEN: the predicted yaw is wrapped back into [-pi, pi] and yaw-rate is unchanged.
	const StateVec state = filter.getState();
	const float expected_yaw = matrix::wrap_pi(yaw + yaw_rate * dt);

	EXPECT_NEAR(state(vte::KF_orientation::kYaw), expected_yaw, kTolerance);
	EXPECT_NEAR(state(vte::KF_orientation::kYawRate), yaw_rate, kTolerance);
}

// WHY: Process noise must be injected to model yaw acceleration uncertainty.
// WHAT: Verify Q terms are added when yaw_acc_var > 0.
TEST(KFOrientationTest, PredictAddsProcessNoise)
{
	// GIVEN: a zero-initialized covariance and a positive yaw-acceleration variance.
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(0.f, 0.f));
	filter.setYawAccVar(0.4f);

	// WHEN: we predict for dt = 2s.
	filter.predict(2.0f);

	// THEN: the diagonal terms match the white-noise yaw-acceleration integration.
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	const float expected_p00 = (8.f / 3.f) * 0.4f;
	const float expected_p11 = 2.0f * 0.4f;

	EXPECT_NEAR(cov_diag(vte::KF_orientation::kYaw), expected_p00, kTolerance);
	EXPECT_NEAR(cov_diag(vte::KF_orientation::kYawRate), expected_p11, kTolerance);
}

// WHY: The white-noise yaw acceleration model introduces cross-covariance terms.
// WHAT: Ensure off-diagonal terms match dt^2/2 * var and covariance remains symmetric.
TEST(KFOrientationTest, PredictAddsOffDiagonalCovariance)
{
	// GIVEN: a zero covariance with a positive yaw-acc variance.
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(0.f, 0.f));
	filter.setYawAccVar(0.4f);

	// WHEN: we predict by dt = 0.5s.
	const float dt = 0.5f;
	filter.predict(dt);

	// THEN: yaw/yaw-rate cross terms equal dt^2/2 * var and are symmetric.
	const auto cov = filter.getStateCovariance();
	const float expected_off_diag = 0.5f * dt * dt * 0.4f;

	EXPECT_NEAR(cov(vte::KF_orientation::kYaw, vte::KF_orientation::kYawRate), expected_off_diag, kTolerance);
	EXPECT_NEAR(cov(vte::KF_orientation::kYawRate, vte::KF_orientation::kYaw), expected_off_diag, kTolerance);
	EXPECT_NEAR(cov(vte::KF_orientation::kYaw, vte::KF_orientation::kYawRate), cov(vte::KF_orientation::kYawRate, vte::KF_orientation::kYaw),
		    kTolerance);
}

// WHY: Invalid noise parameters must not corrupt the covariance.
// WHAT: Confirm negative yaw_acc_var bypasses the Q addition.
TEST(KFOrientationTest, PredictIgnoresInvalidYawAccVar)
{
	// GIVEN: a valid prior covariance and a negative (invalid) yaw-acc variance.
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 2.f));
	filter.setYawAccVar(-1.f);

	// WHEN: we predict for dt = 1s.
	filter.predict(1.0f);

	// THEN: only the deterministic propagation is applied (no Q injection).
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_NEAR(cov_diag(vte::KF_orientation::kYaw), 3.f, kTolerance);
	EXPECT_NEAR(cov_diag(vte::KF_orientation::kYawRate), 2.f, kTolerance);
}

// WHY: Zero dt should not change state or covariance.
// WHAT: Predict with dt=0 and ensure state/cov stay unchanged.
TEST(KFOrientationTest, PredictZeroDtNoChange)
{
	// GIVEN: a filter with an arbitrary state/covariance snapshot.
	vte::KF_orientation filter;
	filter.setState(makeState(1.2f, -0.5f));
	filter.setStateCovarianceDiag(makeDiag(0.3f, 0.2f));
	filter.setYawAccVar(0.4f);

	const StateVec state_before = filter.getState();
	const StateCov cov_before = filter.getStateCovariance();

	// WHEN: we call predict with dt = 0.
	filter.predict(0.f);

	// THEN: the filter is a no-op: state and covariance are preserved.
	expectStateAndCovUnchanged(filter, state_before, cov_before);
}

// WHY: Invalid predict inputs must not corrupt state or covariance.
// WHAT: Exercise the negative-dt and non-finite-dt guards.
TEST(KFOrientationTest, PredictRejectsInvalidDt)
{
	vte::KF_orientation filter;
	filter.setState(makeState(-1.0f, 0.7f));
	filter.setStateCovarianceDiag(makeDiag(0.6f, 0.4f));
	filter.setYawAccVar(0.4f);

	const StateVec state_before = filter.getState();
	const StateCov cov_before = filter.getStateCovariance();

	filter.predict(-0.1f);
	expectStateAndCovUnchanged(filter, state_before, cov_before);

	filter.predict(NAN);
	expectStateAndCovUnchanged(filter, state_before, cov_before);
}

// WHY: Innovations across the wrap should be small, not ~2pi.
// WHAT: Check innovation for +pi to -pi transitions is wrapped.
TEST(KFOrientationTest, InnovationWrapsAcrossPi)
{
	vte::KF_orientation filter;
	filter.setState(makeState(kYawNearPi, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 1.f));

	const StateVec H = makeH(vte::KF_orientation::kYaw);
	static constexpr hrt_abstime kMeasTime = 1_s;
	const vte::KF_orientation::ScalarMeas meas = makeMeas(kMeasTime, -kYawNearPi, 0.1f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);

	const float expected_innov = matrix::wrap_pi(-kYawNearPi - kYawNearPi);

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
}

// WHY: Only yaw innovations should be wrapped; yaw-rate is linear.
// WHAT: Ensure yaw-rate innovation uses the raw difference.
TEST(KFOrientationTest, InnovationDoesNotWrapYawRateMeasurement)
{
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, kYawNearPi));
	filter.setStateCovarianceDiag(makeDiag(1.f, 1.f));

	const StateVec H = makeH(vte::KF_orientation::kYawRate);
	static constexpr hrt_abstime kMeasTime = 1500_ms;
	const vte::KF_orientation::ScalarMeas meas = makeMeas(kMeasTime, -kYawNearPi, 0.1f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);

	const float expected_innov = -kYawNearPi - kYawNearPi;

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
}

// WHY: NIS gating is critical to rejecting yaw outliers.
// WHAT: Provide a large innovation and expect REJECT_NIS.
TEST(KFOrientationTest, RejectsOutlierNis)
{
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(0.01f, 0.01f));

	const StateVec H = makeH(vte::KF_orientation::kYaw);
	static constexpr hrt_abstime kMeasTime = 2_s;
	const vte::KF_orientation::ScalarMeas meas = makeMeas(kMeasTime, kPi, 0.01f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 0.1f);

	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_REJECT_NIS);
	EXPECT_NEAR(state(vte::KF_orientation::kYaw), 0.f, kTolerance);
	EXPECT_NEAR(state(vte::KF_orientation::kYawRate), 0.f, kTolerance);
}

// WHY: Very small variances can destabilize future updates.
// WHAT: Ensure correction enforces a minimum covariance floor.
TEST(KFOrientationTest, ClampCovarianceToMin)
{
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1e-12f, 1e-12f));

	const StateVec H = makeH(vte::KF_orientation::kYaw);
	// Keep innovation small to avoid NIS rejection while exercising the clamp.
	static constexpr hrt_abstime kMeasTime = 3_s;
	const vte::KF_orientation::ScalarMeas meas = makeMeas(kMeasTime, 0.f, 1e-4f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_GE(cov_diag(vte::KF_orientation::kYaw), vte::KF_orientation::kMinVar);
	EXPECT_GE(cov_diag(vte::KF_orientation::kYawRate), vte::KF_orientation::kMinVar);
}

// WHY: Correction should normalize yaw after applying the gain.
// WHAT: Force a correction that would exceed pi without wrapping.
TEST(KFOrientationTest, CorrectionWrapsYaw)
{
	vte::KF_orientation filter;
	filter.setState(makeState(kYawNearPi, 0.f));
	filter.setStateCovarianceDiag(makeDiag(0.5f, 0.2f));

	const StateVec H = makeH(vte::KF_orientation::kYaw);
	static constexpr hrt_abstime kMeasTime = 4_s;
	const vte::KF_orientation::ScalarMeas meas = makeMeas(kMeasTime, -kYawNearPi, 0.2f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);
	const StateVec state = filter.getState();

	const float expected_innov = matrix::wrap_pi(-kYawNearPi - kYawNearPi);
	const float expected_k = 0.5f / (0.5f + 0.2f);
	const float expected_yaw = matrix::wrap_pi(kYawNearPi + expected_k * expected_innov);

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
	EXPECT_NEAR(state(vte::KF_orientation::kYaw), expected_yaw, kTolerance);
	EXPECT_LE(state(vte::KF_orientation::kYaw), kPi);
	EXPECT_GE(state(vte::KF_orientation::kYaw), -kPi);
}

// WHY: Corrections must keep covariance symmetric and within the Cauchy-Schwarz bound.
// WHAT: Predict to seed cross-covariance, fuse a yaw measurement, and verify symmetry
//       plus |cov(i,j)| <= sqrt(var(i) * var(j)).
TEST(KFOrientationTest, CorrectionMaintainsSymmetricCovariance)
{
	vte::KF_orientation filter;
	filter.setState(makeState(0.2f, 0.1f));
	filter.setStateCovarianceDiag(makeDiag(0.6f, 0.4f));
	filter.setYawAccVar(0.3f);

	// Predict so the yaw/yaw-rate cross-covariance is non-zero before fusion.
	filter.predict(0.2f);

	const StateVec H = makeH(vte::KF_orientation::kYaw);
	static constexpr hrt_abstime kMeasTime = 5_s;
	const vte::KF_orientation::ScalarMeas meas = makeMeas(kMeasTime, 0.25f, 0.1f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, kMeasTime + kNowOffset, 100.f);
	const StateCov cov_after = filter.getStateCovariance();

	EXPECT_EQ(res.status, vte::FusionStatus::STATUS_FUSED_CURRENT);
	EXPECT_NEAR(cov_after(vte::KF_orientation::kYaw, vte::KF_orientation::kYawRate),
		    cov_after(vte::KF_orientation::kYawRate, vte::KF_orientation::kYaw), kTolerance);

	const float cs_bound = sqrtf(cov_after(vte::KF_orientation::kYaw, vte::KF_orientation::kYaw)
				     * cov_after(vte::KF_orientation::kYawRate, vte::KF_orientation::kYawRate)) + kTolerance;
	EXPECT_LE(fabsf(cov_after(vte::KF_orientation::kYaw, vte::KF_orientation::kYawRate)), cs_bound);
}

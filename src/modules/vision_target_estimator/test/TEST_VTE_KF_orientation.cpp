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
 * @file TEST_VTE_KF_orientation.cpp
 * @brief Unit test KF_orientation.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <matrix/math.hpp>

#include "Orientation/KF_orientation.h"

namespace vte = vision_target_estimator;

namespace
{
static constexpr float kTolerance = 1e-4f;
static constexpr float kMinVar = 1e-9f;
static constexpr float kPi = 3.14159265358979323846f;

using StateVec = matrix::Vector<float, vte::State::size>;

StateVec makeState(float yaw, float yaw_rate)
{
	StateVec state{};
	state(vte::State::yaw) = yaw;
	state(vte::State::yaw_rate) = yaw_rate;
	return state;
}

StateVec makeDiag(float d0, float d1)
{
	StateVec diag{};
	diag(0) = d0;
	diag(1) = d1;
	return diag;
}

vte::KF_orientation::ScalarMeas makeMeas(uint64_t time_us, float val, float unc, const StateVec &H)
{
	vte::KF_orientation::ScalarMeas meas{};
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

TEST(KFOrientationTest, PredictWrapsYaw)
{
	// WHY: Yaw must remain in [-pi, pi] to avoid discontinuities.
	// WHAT: Predict across the wrap and ensure yaw is normalized.
	vte::KF_orientation filter;
	filter.setState(makeState(3.1f, 1.0f));
	filter.setStateCovarianceDiag(makeDiag(0.f, 0.f));

	filter.predict(1.0f);

	const StateVec state = filter.getState();
	const float expected_yaw = matrix::wrap_pi(4.1f);

	EXPECT_NEAR(state(vte::State::yaw), expected_yaw, kTolerance);
	EXPECT_NEAR(state(vte::State::yaw_rate), 1.0f, kTolerance);
}

TEST(KFOrientationTest, PredictAddsProcessNoise)
{
	// WHY: Process noise must be injected to model yaw acceleration uncertainty.
	// WHAT: Verify Q terms are added when yaw_acc_var > 0.
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(0.f, 0.f));
	filter.setYawAccVar(0.4f);

	filter.predict(2.0f);

	const StateVec cov_diag = filter.getStateCovarianceDiag();

	const float expected_p00 = (8.f / 3.f) * 0.4f;
	const float expected_p11 = 2.0f * 0.4f;

	EXPECT_NEAR(cov_diag(vte::State::yaw), expected_p00, kTolerance);
	EXPECT_NEAR(cov_diag(vte::State::yaw_rate), expected_p11, kTolerance);
}

TEST(KFOrientationTest, PredictIgnoresInvalidYawAccVar)
{
	// WHY: Invalid noise parameters must not corrupt the covariance.
	// WHAT: Confirm negative yaw_acc_var bypasses the Q addition.
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 2.f));
	filter.setYawAccVar(-1.f);

	filter.predict(1.0f);

	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_NEAR(cov_diag(vte::State::yaw), 3.f, kTolerance);
	EXPECT_NEAR(cov_diag(vte::State::yaw_rate), 2.f, kTolerance);
}

TEST(KFOrientationTest, InnovationWrapsAcrossPi)
{
	// WHY: Innovations across the wrap should be small, not ~2pi.
	// WHAT: Check innovation for +pi to -pi transitions is wrapped.
	vte::KF_orientation filter;
	filter.setState(makeState(3.13f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 1.f));

	const StateVec H = makeH(vte::State::yaw);
	const vte::KF_orientation::ScalarMeas meas = makeMeas(1'000'000, -3.13f, 0.1f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 1'000'010, 100.f);

	const float expected_innov = matrix::wrap_pi(-3.13f - 3.13f);

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_NEAR(res.innov, expected_innov, kTolerance);
}

TEST(KFOrientationTest, RejectsOutlierNis)
{
	// WHY: NIS gating is critical to rejecting yaw outliers.
	// WHAT: Provide a large innovation and expect REJECT_NIS.
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(0.01f, 0.01f));

	const StateVec H = makeH(vte::State::yaw);
	const vte::KF_orientation::ScalarMeas meas = makeMeas(2'000'000, 3.14f, 0.01f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 2'000'010, 0.1f);

	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::REJECT_NIS);
	EXPECT_NEAR(state(vte::State::yaw), 0.f, kTolerance);
	EXPECT_NEAR(state(vte::State::yaw_rate), 0.f, kTolerance);
}

TEST(KFOrientationTest, ClampCovarianceToMin)
{
	// WHY: Very small variances can destabilize future updates.
	// WHAT: Ensure correction enforces a minimum covariance floor.
	vte::KF_orientation filter;
	filter.setState(makeState(0.f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1e-12f, 1e-12f));

	const StateVec H = makeH(vte::State::yaw);
	// Keep innovation small to avoid NIS rejection while exercising the clamp.
	const vte::KF_orientation::ScalarMeas meas = makeMeas(3'000'000, 0.f, 1e-4f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 3'000'010, 100.f);
	const StateVec cov_diag = filter.getStateCovarianceDiag();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_GE(cov_diag(vte::State::yaw), kMinVar);
	EXPECT_GE(cov_diag(vte::State::yaw_rate), kMinVar);
}

TEST(KFOrientationTest, CorrectionWrapsYaw)
{
	// WHY: Correction should normalize yaw after applying the gain.
	// WHAT: Force a correction that would exceed pi without wrapping.
	vte::KF_orientation filter;
	filter.setState(makeState(3.0f, 0.f));
	filter.setStateCovarianceDiag(makeDiag(1.f, 1.f));

	const StateVec H = makeH(vte::State::yaw);
	const vte::KF_orientation::ScalarMeas meas = makeMeas(4'000'000, 3.14f, 0.1f, H);
	const vte::FusionResult res = filter.fuseScalarAtTime(meas, 4'000'010, 100.f);
	const StateVec state = filter.getState();

	EXPECT_EQ(res.status, vte::FusionStatus::FUSED_CURRENT);
	EXPECT_LE(state(vte::State::yaw), kPi);
	EXPECT_GE(state(vte::State::yaw), -kPi);
}

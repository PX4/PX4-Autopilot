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
 * @file TEST_VTEOrientation.cpp
 * @brief Unit test VTEOrientation.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <matrix/math.hpp>
#include <memory>
#include <string>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fiducial_marker_yaw_report.h>
#include <uORB/topics/vte_orientation.h>
#include <uORB/topics/vte_aid_source1d.h>
#include <uORB/uORBManager.hpp>

#include "Orientation/VTEOrientation.h"
#include "VTETestHelper.hpp"

namespace vte = vision_target_estimator;

namespace
{
using namespace time_literals;

static constexpr float kTolerance = 1e-4f;
static constexpr float kPi = static_cast<float>(M_PI);
static constexpr float kTwoPi = 2.f * kPi;
static constexpr hrt_abstime kStepUs = 1_ms;

// Default EV noise parameter and the corresponding variance (params are configured as std-dev).
static constexpr float kDefaultEvYawNoise = 0.5f;
static constexpr float kDefaultEvYawVar = kDefaultEvYawNoise * kDefaultEvYawNoise;
// Typical reported variance of a clean yaw measurement; smaller than the floor so the floor applies.
static constexpr float kMeasVar = 0.01f;
// Initial state uncertainty used by the test parameter VTE_YAW_UNC_IN.
static constexpr float kInitialYawUnc = 0.2f;
// NIS threshold permissive enough to accept any realistic innovation.
static constexpr float kPermissiveNis = 100.f;
// NIS threshold low enough to reject everything but tiny innovations.
static constexpr float kRestrictiveNis = 0.2f;
// The estimator rejects prediction gaps longer than 100 ms — see kMaxPredictionDeltaTimeUs.
static constexpr hrt_abstime kStaleGapUs = 120_ms;

class VTEOrientationTestable : public vte::VTEOrientation
{
public:
	void updateParamsPublic() { updateParams(); }
};

} // namespace

class VTEOrientationTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite() { uORB::Manager::initialize(); }

	void SetUp() override
	{
		param_control_autosave(false);
		vte_test::useFakeTime();

		setParamFloat("VTE_YAW_UNC_IN", kInitialYawUnc);
		setParamFloat("VTE_EVA_NOISE", kDefaultEvYawNoise);
		setParamFloat("VTE_YAW_NIS_THRE", vte::kDefaultNisThreshold);

		_vte = std::make_unique<VTEOrientationTestable>();
		ASSERT_TRUE(_vte->init());
		_vte->updateParamsPublic();

		_vision_pub = std::make_unique<uORB::Publication<fiducial_marker_yaw_report_s>>(ORB_ID(fiducial_marker_yaw_report));
		_aid_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source1d_s>>(ORB_ID(vte_aid_ev_yaw));
		_state_sub = std::make_unique<uORB::SubscriptionData<vte_orientation_s>>(ORB_ID(vte_orientation));

		vte_test::flushSubscription(_aid_sub);
		vte_test::flushSubscription(_state_sub);
	}

	void TearDown() override
	{
		_param_guard.restore();
		param_control_autosave(true);
		_state_sub.reset();
		_aid_sub.reset();
		_vision_pub.reset();

		_vte.reset();
		vte_test::useRealTime();
		// Keep the uORB manager alive; parameters retain uORB handles across tests.
	}

	void setParamFloat(const char *name, float value)
	{
		ASSERT_TRUE(_param_guard.setFloat(name, value));
	}

	void publishVisionYaw(float yaw, float yaw_var, hrt_abstime timestamp)
	{
		ASSERT_TRUE(vte_test::publishVisionYaw(*_vision_pub, yaw, yaw_var, timestamp));
	}

	void enableVisionFusion()
	{
		vte::SensorFusionMaskU mask{};
		mask.flags.use_vision_pos = 1;
		_vte->setVteAidMask(mask.value);
	}

	/**
	 * Publish a valid yaw sample timestamped kStepUs in the future and drive one update tick.
	 * Returns the published timestamp so tests can reference it (e.g. for OOSM replays).
	 */
	hrt_abstime pushValidYawSample(float yaw, float yaw_var = kMeasVar)
	{
		const hrt_abstime sample_time = vte_test::advanceMicroseconds(kStepUs);
		publishVisionYaw(yaw, yaw_var, sample_time);
		_vte->update();
		return sample_time;
	}

	std::unique_ptr<VTEOrientationTestable> _vte;
	std::unique_ptr<uORB::Publication<fiducial_marker_yaw_report_s>> _vision_pub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source1d_s>> _aid_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_orientation_s>> _state_sub;

	vte_test::ParamGuard _param_guard{};
};

// WHY: The estimator must initialize from the first valid yaw measurement.
// WHAT: Publish a valid yaw and verify state and innovation messages.
TEST_F(VTEOrientationTest, InitFromVisionYawPublishesOrientation)
{
	// GIVEN: vision fusion is enabled and the first measurement has finite yaw and variance.
	enableVisionFusion();

	// WHEN: the estimator processes the first yaw observation.
	const float yaw_meas = 0.5f;
	pushValidYawSample(yaw_meas);

	// THEN: the state initializes from that sample and publishes a fused innovation report.
	ASSERT_TRUE(_state_sub->update());
	const auto state = _state_sub->get();
	EXPECT_NEAR(state.yaw, yaw_meas, kTolerance);
	EXPECT_NEAR(state.yaw_rate, 0.f, kTolerance);
	// Reported variance is clamped to the configured floor (EVA_NOISE^2) before fusion.
	const float meas_var = fmaxf(kMeasVar, kDefaultEvYawVar);
	const float expected_cov = (kInitialYawUnc * meas_var) / (kInitialYawUnc + meas_var);
	EXPECT_NEAR(state.cov_yaw, expected_cov, kTolerance);

	ASSERT_TRUE(_aid_sub->update());
	const auto aid = _aid_sub->get();
	EXPECT_NEAR(aid.observation_variance, kDefaultEvYawVar, kTolerance);
	EXPECT_EQ(aid.fusion_status, static_cast<uint8_t>(vte::FusionStatus::STATUS_FUSED_CURRENT));
}

// WHY: Fusion should be disabled when the aid mask is empty.
// WHAT: Publish a valid yaw without enabling vision fusion and expect no
// output.
TEST_F(VTEOrientationTest, DoesNotFuseWhenVisionDisabled)
{
	// GIVEN: a valid yaw report but no enabled aid source.
	publishVisionYaw(0.2f, kMeasVar, vte_test::advanceMicroseconds(kStepUs));

	// WHEN: the estimator updates.
	_vte->update();

	// THEN: no state or innovation message is published.
	EXPECT_FALSE(_aid_sub->update());
	EXPECT_FALSE(_state_sub->update());
}

// Parametrized rejection tests: each row describes one way a sample can be invalid and must not fuse.
struct BadVisionSample {
	const char *label;
	float yaw;
	float yaw_var;
	bool stale; // true: timestamp older than the measurement-recent timeout
};

class VTEOrientationBadSampleTest : public VTEOrientationTest,
	public ::testing::WithParamInterface<BadVisionSample> {};

// WHY: Corrupt or stale yaw measurements must not initialize or fuse the filter.
// WHAT: Publish an invalid sample and ensure no outputs are produced.
TEST_P(VTEOrientationBadSampleTest, RejectsBadSample)
{
	const auto param = GetParam();

	// GIVEN: vision fusion is enabled with a tight recency timeout used only by the stale variant.
	enableVisionFusion();
	static constexpr hrt_abstime kMeasTimeoutUs = 1_ms;
	_vte->setMeasRecentTimeout(kMeasTimeoutUs);

	const hrt_abstime timestamp = param.stale
				      ? vte_test::nowUs() - (kMeasTimeoutUs + 1_ms)
				      : vte_test::advanceMicroseconds(kStepUs);
	publishVisionYaw(param.yaw, param.yaw_var, timestamp);

	// WHEN: the estimator processes the sample.
	_vte->update();

	// THEN: the sample is ignored.
	EXPECT_FALSE(_aid_sub->update()) << param.label;
	EXPECT_FALSE(_state_sub->update()) << param.label;
}

INSTANTIATE_TEST_SUITE_P(
	InvalidSamples,
	VTEOrientationBadSampleTest,
	::testing::Values(
		BadVisionSample{"NanYaw", NAN, kMeasVar, false},
		BadVisionSample{"NanVariance", 0.1f, NAN, false},
		BadVisionSample{"StaleTimestamp", 0.1f, kMeasVar, true}
	),
[](const testing::TestParamInfo<BadVisionSample> &p) { return std::string(p.param.label); });

// WHY: Measurement variance must never drop below the configured floor.
// WHAT: Provide a tiny variance and verify the floor is applied.
TEST_F(VTEOrientationTest, YawNoiseFloor)
{
	// GIVEN: vision fusion is enabled and the configured floor is above the reported variance.
	enableVisionFusion();

	static constexpr float kEvNoise = 0.2f;
	setParamFloat("VTE_EVA_NOISE", kEvNoise);
	_vte->updateParamsPublic();

	// WHEN: a sample with zero variance is fused.
	pushValidYawSample(0.3f, /* yaw_var = */ 0.0f);

	// THEN: the innovation log uses the configured floor instead of zero variance.
	ASSERT_TRUE(_aid_sub->update());
	const auto aid = _aid_sub->get();
	EXPECT_NEAR(aid.observation_variance, kEvNoise * kEvNoise, kTolerance);
}

// WHY: NIS gating protects against yaw outliers.
// WHAT: Initialize at yaw 0, then fuse a far measurement and expect
// rejection.
TEST_F(VTEOrientationTest, RejectsOutlierNis)
{
	// GIVEN: an initialized estimator with a very small NIS threshold.
	enableVisionFusion();
	setParamFloat("VTE_YAW_NIS_THRE", kRestrictiveNis);
	_vte->updateParamsPublic();

	pushValidYawSample(0.f);
	ASSERT_TRUE(_state_sub->update());
	vte_test::flushSubscription(_aid_sub);

	// WHEN: a far-away yaw sample is fused.
	pushValidYawSample(kPi);

	// THEN: the innovation is rejected and the state stays near the initialized yaw.
	ASSERT_TRUE(_aid_sub->update());
	EXPECT_EQ(_aid_sub->get().fusion_status,
		  static_cast<uint8_t>(vte::FusionStatus::STATUS_REJECT_NIS));

	ASSERT_TRUE(_state_sub->update());
	EXPECT_NEAR(_state_sub->get().yaw, 0.f, 0.1f);
}

// WHY: Wrap handling should not average 0 and 2pi to pi.
// WHAT: Toggle measurements near 0 and 2pi and verify yaw stays near zero.
TEST_F(VTEOrientationTest, WrapsAcrossZeroAndTwoPi)
{
	// GIVEN: an initialized estimator with two measurements straddling the wrap boundary.
	enableVisionFusion();

	const float yaw_near_zero = 0.01f;
	const float yaw_near_two_pi = kTwoPi - 0.013f;

	pushValidYawSample(yaw_near_zero);

	// WHEN: both measurements are fused in sequence.
	pushValidYawSample(yaw_near_two_pi);

	// THEN: the estimate stays close to zero instead of jumping toward pi.
	ASSERT_TRUE(_state_sub->update());
	const float expected = matrix::wrap_pi(yaw_near_two_pi);
	EXPECT_NEAR(_state_sub->get().yaw, expected, 0.1f);
	EXPECT_LT(fabsf(_state_sub->get().yaw), 0.2f);
}

// WHY: Initial yaw uncertainty is only meant to seed a new estimator instance.
// WHAT: Change VTE_YAW_UNC_IN while the filter is running and verify the active estimator is not reset.
TEST_F(VTEOrientationTest, DoesNotResetOnYawUncChangeWhileRunning)
{
	// GIVEN: an already running estimator with a fused yaw state.
	enableVisionFusion();

	pushValidYawSample(0.2f);
	ASSERT_TRUE(_state_sub->update());
	const float cov_before = _state_sub->get().cov_yaw;

	// WHEN: the initial-uncertainty parameter changes while the estimator keeps running.
	setParamFloat("VTE_YAW_UNC_IN", 10.f * kInitialYawUnc);
	_vte->updateParamsPublic();
	pushValidYawSample(0.2f);

	// THEN: the live state keeps converging instead of being reinitialized (covariance must keep shrinking).
	ASSERT_TRUE(_state_sub->update());
	const auto state = _state_sub->get();
	EXPECT_NEAR(state.yaw, 0.2f, 0.1f);
	EXPECT_LT(state.cov_yaw, cov_before);
}

// WHY: Delayed yaw measurements should follow the OOSM fusion path.
// WHAT: Populate history, then fuse a delayed measurement and expect FUSED_OOSM.
TEST_F(VTEOrientationTest, FusesOosmYawMeasurement)
{
	// GIVEN: an estimator with enough history for a delayed yaw sample.
	enableVisionFusion();
	setParamFloat("VTE_YAW_NIS_THRE", kPermissiveNis);
	_vte->updateParamsPublic();

	// Seed the OOSM history with a few recent samples and capture the first predict timestamp.
	hrt_abstime first_predict_time = 0;

	for (int i = 0; i < 3; ++i) {
		pushValidYawSample(0.1f);

		if (_aid_sub->update() && (first_predict_time == 0)) {
			first_predict_time = _aid_sub->get().time_last_predict;
		}

		(void)_state_sub->update();
	}

	ASSERT_GT(first_predict_time, 0u);

	// Publish a sample stamped back at the first predict time, after advancing the clock.
	vte_test::advanceMicroseconds(30_ms);
	publishVisionYaw(0.12f, kMeasVar, first_predict_time);

	// WHEN: a measurement arrives delayed but still inside the OOSM window.
	_vte->update();

	// THEN: the OOSM path is taken and reports replayed history steps.
	ASSERT_TRUE(_aid_sub->update());
	const auto aid = _aid_sub->get();
	EXPECT_EQ(aid.fusion_status, static_cast<uint8_t>(vte::FusionStatus::STATUS_FUSED_OOSM));
	EXPECT_GT(aid.history_steps, 0u);
}

// WHY: A long scheduling gap should reset the filter instead of predicting across stale time.
// WHAT: Initialize once, wait past the allowed prediction gap, then verify a fresh measurement re-initializes cleanly.
TEST_F(VTEOrientationTest, ResetsFilterOnStalePredictionGap)
{
	// GIVEN: an initialized estimator that then experiences a gap longer than the allowed prediction horizon.
	enableVisionFusion();

	pushValidYawSample(0.1f);
	ASSERT_TRUE(_state_sub->update());
	EXPECT_NEAR(_state_sub->get().yaw, 0.1f, kTolerance);

	vte_test::flushSubscription(_aid_sub);
	vte_test::flushSubscription(_state_sub);

	vte_test::advanceMicroseconds(kStaleGapUs);

	// WHEN: the estimator updates after the stale gap and then sees a fresh measurement.
	_vte->update();
	EXPECT_FALSE(_aid_sub->update());
	EXPECT_FALSE(_state_sub->update());

	pushValidYawSample(-0.25f);

	// THEN: the stale state is dropped and the fresh sample initializes a new current fusion.
	ASSERT_TRUE(_aid_sub->update());
	EXPECT_EQ(_aid_sub->get().fusion_status,
		  static_cast<uint8_t>(vte::FusionStatus::STATUS_FUSED_CURRENT));

	ASSERT_TRUE(_state_sub->update());
	const auto state = _state_sub->get();
	EXPECT_NEAR(state.yaw, -0.25f, kTolerance);
	EXPECT_NEAR(state.yaw_rate, 0.f, kTolerance);
}

// WHY: Orientation validity must reflect the configured timeout.
// WHAT: Set timeout to zero and confirm orientation_valid is false.
TEST_F(VTEOrientationTest, TargetValidityTimeout)
{
	// GIVEN: an estimator with zero target-valid timeout.
	enableVisionFusion();
	_vte->setTargetValidTimeout(0);

	// WHEN: a valid sample is fused.
	pushValidYawSample(0.1f);

	// THEN: the published state is immediately marked invalid.
	ASSERT_TRUE(_state_sub->update());
	EXPECT_FALSE(_state_sub->get().orientation_valid);
}

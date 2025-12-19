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
 * @file TEST_VTE_VTEOrientation.cpp
 * @brief Unit test VTEOrientation.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <parameters/param.h>
#include <uORB/uORBManager.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fiducial_marker_yaw_report.h>
#include <uORB/topics/vte_aid_source1d.h>
#include <uORB/topics/vision_target_est_orientation.h>

#include "Orientation/VTEOrientation.h"

namespace vte = vision_target_estimator;

namespace
{
static constexpr float kTolerance = 1e-4f;
static constexpr hrt_abstime kTimestampLagUs = 5'000;

class VTEOrientationTestable : public vte::VTEOrientation
{
public:
	void updateParamsPublic() { updateParams(); }
};

} // namespace

class VTEOrientationTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		uORB::Manager::initialize();
		param_control_autosave(false);

		setParamFloat("VTE_YAW_UNC_IN", 0.2f);
		setParamFloat("VTE_EVA_NOISE", 0.5f);
		setParamFloat("VTE_YAW_NIS_THRE", 3.84f);
		setParamInt("VTE_EV_NOISE_MD", 0);

		_vte = std::make_unique<VTEOrientationTestable>();
		ASSERT_TRUE(_vte->init());
		_vte->updateParamsPublic();

		_vision_pub = std::make_unique<uORB::Publication<fiducial_marker_yaw_report_s>>(ORB_ID(fiducial_marker_yaw_report));
		_aid_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source1d_s>>(ORB_ID(vte_aid_ev_yaw));
		_state_sub = std::make_unique<uORB::SubscriptionData<vision_target_est_orientation_s>>(ORB_ID(vision_target_est_orientation));

		resetTimestamp();
	}

	void TearDown() override
	{
		_state_sub.reset();
		_aid_sub.reset();
		_vision_pub.reset();
		_vte.reset();
		// Keep the uORB manager alive; parameters retain uORB handles across tests.
	}

	void setParamFloat(const char *name, float value)
	{
		param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID);
		ASSERT_EQ(param_set(handle, &value), 0);
	}

	void setParamInt(const char *name, int32_t value)
	{
		param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID);
		ASSERT_EQ(param_set(handle, &value), 0);
	}

	void resetTimestamp()
	{
		_timestamp = hrt_absolute_time() - kTimestampLagUs;
	}

	hrt_abstime stepTime(hrt_abstime delta_us)
	{
		_timestamp += delta_us;
		const hrt_abstime now = hrt_absolute_time();

		if (_timestamp >= now) {
			_timestamp = now - 1;
		}

		return _timestamp;
	}

	void publishVisionYaw(float yaw, float yaw_var, hrt_abstime timestamp)
	{
		fiducial_marker_yaw_report_s msg{};
		msg.timestamp = timestamp;
		msg.timestamp_sample = timestamp;
		msg.yaw_ned = yaw;
		msg.yaw_var_ned = yaw_var;
		ASSERT_TRUE(_vision_pub->publish(msg));
	}

	void enableVisionFusion()
	{
		vte::SensorFusionMaskU mask{};
		mask.flags.use_vision_pos = 1;
		_vte->set_vte_aid_mask(mask.value);
	}

	std::unique_ptr<VTEOrientationTestable> _vte;
	std::unique_ptr<uORB::Publication<fiducial_marker_yaw_report_s>> _vision_pub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source1d_s>> _aid_sub;
	std::unique_ptr<uORB::SubscriptionData<vision_target_est_orientation_s>> _state_sub;

	hrt_abstime _timestamp{0};
};

TEST_F(VTEOrientationTest, InitFromVisionYawPublishesOrientation)
{
	// WHY: The estimator must initialize from the first valid yaw measurement.
	// WHAT: Publish a valid yaw and verify state and innovation messages.
	enableVisionFusion();

	const float yaw_meas = 0.5f;
	publishVisionYaw(yaw_meas, 0.01f, stepTime(1'000));

	_vte->update();

	ASSERT_TRUE(_state_sub->update());
	const auto state = _state_sub->get();
	EXPECT_NEAR(state.yaw, yaw_meas, kTolerance);
	EXPECT_NEAR(state.yaw_rate, 0.f, kTolerance);
	const float meas_var = fmaxf(0.01f, 0.5f * 0.5f);
	const float expected_cov = (0.2f * meas_var) / (0.2f + meas_var);
	EXPECT_NEAR(state.cov_yaw, expected_cov, kTolerance);

	ASSERT_TRUE(_aid_sub->update());
	const auto aid = _aid_sub->get();
	EXPECT_GE(aid.observation_variance, 0.25f);
	EXPECT_EQ(aid.fusion_status, static_cast<uint8_t>(vte::FusionStatus::FUSED_CURRENT));
}

TEST_F(VTEOrientationTest, RejectsInvalidYaw)
{
	// WHY: Corrupt yaw measurements should not initialize or fuse.
	// WHAT: Publish NaN yaw and ensure no outputs are produced.
	enableVisionFusion();

	publishVisionYaw(NAN, 0.01f, stepTime(1'000));
	_vte->update();

	EXPECT_FALSE(_aid_sub->update());
	EXPECT_FALSE(_state_sub->update());
}

TEST_F(VTEOrientationTest, RejectsStaleYaw)
{
	// WHY: Stale yaw data can destabilize the filter.
	// WHAT: Set a tiny timeout and verify stale data is ignored.
	enableVisionFusion();

	_vte->set_meas_recent_timeout(1'000);
	const hrt_abstime stale_time = hrt_absolute_time() - 2'000;
	publishVisionYaw(0.1f, 0.01f, stale_time);

	_vte->update();

	EXPECT_FALSE(_aid_sub->update());
	EXPECT_FALSE(_state_sub->update());
}

TEST_F(VTEOrientationTest, YawNoiseFloor)
{
	// WHY: Measurement variance must never drop below the configured floor.
	// WHAT: Provide a tiny variance and verify the floor is applied.
	enableVisionFusion();

	setParamFloat("VTE_EVA_NOISE", 0.2f);
	_vte->updateParamsPublic();

	publishVisionYaw(0.3f, 0.0f, stepTime(1'000));
	_vte->update();

	ASSERT_TRUE(_aid_sub->update());
	const auto aid = _aid_sub->get();
	EXPECT_GE(aid.observation_variance, 0.04f);
}

TEST_F(VTEOrientationTest, RangeScaledYawNoise)
{
	// WHY: Range-based noise scaling should increase variance with distance.
	// WHAT: Enable range model and verify expected variance scaling.
	enableVisionFusion();

	setParamFloat("VTE_EVA_NOISE", 0.2f);
	setParamInt("VTE_EV_NOISE_MD", 1);
	_vte->updateParamsPublic();

	_vte->set_range_sensor(2.f, true, stepTime(1'000));
	publishVisionYaw(0.2f, 0.0f, stepTime(1'000));
	_vte->update();

	ASSERT_TRUE(_aid_sub->update());
	const auto aid = _aid_sub->get();

	const float expected_var = 0.04f * 4.f;
	EXPECT_NEAR(aid.observation_variance, expected_var, kTolerance);
}

TEST_F(VTEOrientationTest, RejectsOutlierNis)
{
	// WHY: NIS gating protects against yaw outliers.
	// WHAT: Initialize at yaw 0, then fuse a far measurement and expect rejection.
	enableVisionFusion();

	setParamFloat("VTE_YAW_NIS_THRE", 0.2f);
	_vte->updateParamsPublic();

	publishVisionYaw(0.f, 0.01f, stepTime(1'000));
	_vte->update();
	ASSERT_TRUE(_state_sub->update());

	publishVisionYaw(3.14f, 0.01f, stepTime(1'000));
	_vte->update();

	ASSERT_TRUE(_aid_sub->update());
	const auto aid = _aid_sub->get();
	EXPECT_EQ(aid.fusion_status, static_cast<uint8_t>(vte::FusionStatus::REJECT_NIS));

	ASSERT_TRUE(_state_sub->update());
	const auto state = _state_sub->get();
	EXPECT_NEAR(state.yaw, 0.f, 0.1f);
}

TEST_F(VTEOrientationTest, TargetValidityTimeout)
{
	// WHY: Orientation validity must reflect the configured timeout.
	// WHAT: Set timeout to zero and confirm orientation_valid is false.
	enableVisionFusion();
	_vte->set_target_valid_timeout(0);

	publishVisionYaw(0.1f, 0.01f, stepTime(1'000));
	_vte->update();

	ASSERT_TRUE(_state_sub->update());
	const auto state = _state_sub->get();
	EXPECT_FALSE(state.orientation_valid);
}

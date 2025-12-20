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
 * @file TEST_VTEPosition.cpp
 * @brief Unit test VTEPosition.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <memory>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <parameters/param.h>
#include <uORB/uORBManager.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/target_gnss.h>
#include <uORB/topics/vte_aid_source3d.h>
#include <uORB/topics/vision_target_est_position.h>
#include <matrix/Quaternion.hpp>

#include "Position/VTEPosition.h"
#include "VTETestHelper.hpp"

namespace vte = vision_target_estimator;

namespace
{
using namespace time_literals;

static constexpr float kTolerance = 1e-3f;
static constexpr hrt_abstime kStepUs = 1_ms;
static constexpr float kDefaultEvPosNoise = 0.2f;
static constexpr float kDefaultGpsPosNoise = 0.1f;
static constexpr float kDefaultGpsVelNoise = 0.2f;
static constexpr float kDefaultEvPosVar = kDefaultEvPosNoise * kDefaultEvPosNoise;
static constexpr float kDefaultGpsPosVar = kDefaultGpsPosNoise * kDefaultGpsPosNoise;
static constexpr float kDefaultGpsVelVar = kDefaultGpsVelNoise * kDefaultGpsVelNoise;
static constexpr double kUavLat = 47.3977419;
static constexpr double kUavLon = 8.5455938;
static constexpr float kUavAltM = 100.f;
static constexpr float kTargetAltM = 90.f;
static const matrix::Vector3f kSmallVisionCov{0.01f, 0.01f, 0.01f};
static const matrix::Vector3f kZeroVec{0.f, 0.f, 0.f};

enum class FusionMaskOption {
	VisionPos,
	MissionPos,
	TargetGpsPos,
	UavGpsVel,
	TargetGpsVel
};

class VTEPositionTestable : public vte::VTEPosition
{
public:
	void updateParamsPublic() { updateParams(); }
};

} // namespace

class VTEPositionTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();
	}

	void SetUp() override
	{
		param_control_autosave(false);

		setParamFloat("VTE_POS_UNC_IN", 1.f);
		setParamFloat("VTE_VEL_UNC_IN", 1.f);
		setParamFloat("VTE_BIA_UNC_IN", 0.5f);
		setParamFloat("VTE_EVP_NOISE", kDefaultEvPosNoise);
		setParamFloat("VTE_GPS_P_NOISE", kDefaultGpsPosNoise);
		setParamFloat("VTE_GPS_V_NOISE", kDefaultGpsVelNoise);
		setParamFloat("VTE_POS_NIS_THRE", 3.84f);
		setParamInt("VTE_EV_NOISE_MD", 0);

		_vte = std::make_unique<VTEPositionTestable>();
		ASSERT_TRUE(_vte->init());
		_vte->updateParamsPublic();

		_vision_pub = std::make_unique<uORB::Publication<fiducial_marker_pos_report_s>>(ORB_ID(fiducial_marker_pos_report));
		_uav_gps_pub = std::make_unique<uORB::Publication<sensor_gps_s>>(ORB_ID(vehicle_gps_position));
		_target_gps_pub = std::make_unique<uORB::Publication<target_gnss_s>>(ORB_ID(target_gnss));

		_aid_fiducial_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_fiducial_marker));
		_aid_gps_mission_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_pos_mission));
		_aid_gps_target_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_pos_target));
		_aid_gps_vel_uav_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_vel_uav));
		_aid_gps_vel_target_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_vel_target));
		_vte_state_sub = std::make_unique<uORB::SubscriptionData<vision_target_est_position_s>>(ORB_ID(vision_target_est_position));
		_target_pose_sub = std::make_unique<uORB::SubscriptionData<landing_target_pose_s>>(ORB_ID(landing_target_pose));

		vte_test::flushSubscription(_aid_fiducial_sub);
		vte_test::flushSubscription(_aid_gps_mission_sub);
		vte_test::flushSubscription(_aid_gps_target_sub);
		vte_test::flushSubscription(_aid_gps_vel_uav_sub);
		vte_test::flushSubscription(_aid_gps_vel_target_sub);
		vte_test::flushSubscription(_vte_state_sub);
		vte_test::flushSubscription(_target_pose_sub);

	}

	void TearDown() override
	{
		_param_guard.restore();
		param_control_autosave(true);
		_target_pose_sub.reset();
		_vte_state_sub.reset();
		_aid_gps_vel_target_sub.reset();
		_aid_gps_vel_uav_sub.reset();
		_aid_gps_target_sub.reset();
		_aid_gps_mission_sub.reset();
		_aid_fiducial_sub.reset();
		_target_gps_pub.reset();
		_uav_gps_pub.reset();
		_vision_pub.reset();

		_vte.reset();
		// Keep the uORB manager alive; parameters retain uORB handles across tests.
	}

	void setParamFloat(const char *name, float value)
	{
		ASSERT_TRUE(_param_guard.setFloat(name, value));
	}

	void setParamInt(const char *name, int32_t value)
	{
		ASSERT_TRUE(_param_guard.setInt(name, value));
	}

	hrt_abstime advanceMicroseconds(hrt_abstime delta_us)
	{
		if (delta_us > 0) {
			px4_usleep(static_cast<useconds_t>(delta_us));
		}

		return hrt_absolute_time();
	}

	hrt_abstime advanceStep()
	{
		return advanceMicroseconds(kStepUs);
	}

	hrt_abstime nowUs() const
	{
		return hrt_absolute_time();
	}

	void enableMask(std::initializer_list<FusionMaskOption> sources)
	{
		vte::SensorFusionMaskU mask{};

		for (const FusionMaskOption source : sources) {
			switch (source) {
			case FusionMaskOption::VisionPos:
				mask.flags.use_vision_pos = 1;
				break;

			case FusionMaskOption::MissionPos:
				mask.flags.use_mission_pos = 1;
				break;

			case FusionMaskOption::TargetGpsPos:
				mask.flags.use_target_gps_pos = 1;
				break;

			case FusionMaskOption::UavGpsVel:
				mask.flags.use_uav_gps_vel = 1;
				break;

			case FusionMaskOption::TargetGpsVel:
				mask.flags.use_target_gps_vel = 1;
				break;
			}
		}

		_vte->set_vte_aid_mask(mask.value);
	}

	void publishVisionPos(const matrix::Vector3f &rel_pos, const matrix::Quaternionf &q,
			      const matrix::Vector3f &cov, hrt_abstime timestamp)
	{
		ASSERT_TRUE(vte_test::publishVisionPos(*_vision_pub, rel_pos, q, cov, timestamp));
	}

	void publishUavGps(double lat, double lon, float alt, float eph, float epv,
			   const matrix::Vector3f &vel_ned, float vel_var, bool vel_valid, hrt_abstime timestamp)
	{
		ASSERT_TRUE(vte_test::publishUavGps(*_uav_gps_pub, lat, lon, alt, eph, epv, vel_ned, vel_var, vel_valid, timestamp));
	}

	void publishTargetGnss(double lat, double lon, float alt, float eph, float epv,
			       hrt_abstime timestamp, bool abs_pos_updated, bool vel_ned_updated = false,
			       const matrix::Vector3f &vel_ned = matrix::Vector3f{}, float vel_acc = NAN)
	{
		ASSERT_TRUE(vte_test::publishTargetGnss(*_target_gps_pub, lat, lon, alt, eph, epv, timestamp,
							abs_pos_updated, vel_ned_updated, vel_ned, vel_acc));
	}

	void setLocalVelocity(const matrix::Vector3f &vel_ned, hrt_abstime timestamp)
	{
		_vte->set_local_velocity(vel_ned, true, timestamp);
	}

	void setLocalPosition(const matrix::Vector3f &pos_ned, hrt_abstime timestamp)
	{
		_vte->set_local_position(pos_ned, true, timestamp);
	}

	std::unique_ptr<VTEPositionTestable> _vte;
	std::unique_ptr<uORB::Publication<fiducial_marker_pos_report_s>> _vision_pub;
	std::unique_ptr<uORB::Publication<sensor_gps_s>> _uav_gps_pub;
	std::unique_ptr<uORB::Publication<target_gnss_s>> _target_gps_pub;

	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_fiducial_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_mission_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_target_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_vel_uav_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_vel_target_sub;
	std::unique_ptr<uORB::SubscriptionData<vision_target_est_position_s>> _vte_state_sub;
	std::unique_ptr<uORB::SubscriptionData<landing_target_pose_s>> _target_pose_sub;

	vte_test::ParamGuard _param_guard{};
};

TEST_F(VTEPositionTest, InitRequiresVelocityEstimate)
{
	// WHY: Estimator must not initialize without a velocity estimate.
	// WHAT: Provide a position measurement only and expect no output.
	enableMask({FusionMaskOption::VisionPos});

	const matrix::Vector3f rel_pos(1.f, 2.f, -3.f);
	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kZeroVec, vision_time);

	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_vte_state_sub->update());
	EXPECT_FALSE(_aid_fiducial_sub->update());
}

TEST_F(VTEPositionTest, InitWithVisionAndLocalVelocity)
{
	// WHY: Vision + velocity should be sufficient to start the filter.
	// WHAT: Verify initial state matches the vision position and local velocity.
	enableMask({FusionMaskOption::VisionPos});

	const matrix::Vector3f rel_pos(4.f, -2.f, 1.f);
	const matrix::Vector3f vel_ned(1.f, 2.f, 3.f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kZeroVec, vision_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();

	EXPECT_NEAR(state.rel_pos[0], rel_pos(0), kTolerance);
	EXPECT_NEAR(state.rel_pos[1], rel_pos(1), kTolerance);
	EXPECT_NEAR(state.rel_pos[2], rel_pos(2), kTolerance);
	EXPECT_NEAR(state.vel_uav[0], vel_ned(0), kTolerance);
	EXPECT_NEAR(state.vel_uav[1], vel_ned(1), kTolerance);
	EXPECT_NEAR(state.vel_uav[2], vel_ned(2), kTolerance);

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();
	EXPECT_NEAR(aid.observation_variance[0], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[1], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[2], kDefaultEvPosVar, kTolerance);
}

TEST_F(VTEPositionTest, InitialPositionUsesVisionFirst)
{
	// WHY: Vision should take priority over GNSS for initialization.
	// WHAT: Provide vision and GPS with different values and check vision innovation near zero.
	enableMask({FusionMaskOption::VisionPos, FusionMaskOption::TargetGpsPos});

	const matrix::Vector3f vel_ned(0.1f, 0.2f, 0.3f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	setParamFloat("VTE_POS_NIS_THRE", 100.f);
	_vte->updateParamsPublic();

	static constexpr hrt_abstime kBaseAgeUs = 10_ms;
	static constexpr hrt_abstime kGpsLagUs = 4_ms;
	const hrt_abstime base_time = nowUs() - kBaseAgeUs;
	const hrt_abstime vision_time = base_time;
	const hrt_abstime gps_time = base_time + kGpsLagUs;

	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.5f, 0.5f, vel_ned, 0.1f, true, gps_time);
	publishTargetGnss(kUavLat, kUavLon, kTargetAltM, 0.5f, 0.5f, gps_time, true);

	const matrix::Vector3f rel_pos(5.f, 6.f, 7.f);
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();
	EXPECT_NEAR(aid.innovation[0], 0.f, 0.5f);
	EXPECT_NEAR(aid.innovation[1], 0.f, 0.5f);
	EXPECT_NEAR(aid.innovation[2], 0.f, 0.5f);

	ASSERT_TRUE(_aid_gps_target_sub->update());
	const auto aid_gps = _aid_gps_target_sub->get();
	EXPECT_EQ(aid_gps.fusion_status[0], static_cast<uint8_t>(vte::FusionStatus::FUSED_CURRENT));
}

TEST_F(VTEPositionTest, RejectsOutlierNis)
{
	// WHY: NIS gating should reject large innovations.
	// WHAT: Initialize with vision, then feed an outlier and expect REJECT_NIS.
	enableMask({FusionMaskOption::VisionPos});

	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), vel_time);
	const hrt_abstime init_time = advanceStep();
	publishVisionPos(matrix::Vector3f(0.f, 0.f, 0.f), vte_test::identityQuat(),
			 kSmallVisionCov, init_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state_before = _vte_state_sub->get();

	setParamFloat("VTE_POS_NIS_THRE", 0.11f);
	_vte->updateParamsPublic();

	const hrt_abstime outlier_time = advanceStep();
	publishVisionPos(matrix::Vector3f(100.f, -100.f, 50.f), vte_test::identityQuat(), kSmallVisionCov, outlier_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();
	EXPECT_EQ(aid.fusion_status[0], static_cast<uint8_t>(vte::FusionStatus::REJECT_NIS));

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state_after = _vte_state_sub->get();
	EXPECT_NEAR(state_after.rel_pos[0], state_before.rel_pos[0], 0.1f);
	EXPECT_NEAR(state_after.rel_pos[1], state_before.rel_pos[1], 0.1f);
	EXPECT_NEAR(state_after.rel_pos[2], state_before.rel_pos[2], 0.1f);
}

TEST_F(VTEPositionTest, RejectsInvalidTargetGnssData)
{
	// WHY: Invalid GNSS data should be rejected before fusion.
	// WHAT: Feed out-of-range, sentinel, and altitude-invalid measurements and expect no aid update.
	enableMask({FusionMaskOption::TargetGpsPos});

	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), vel_time);

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);

	auto run_case = [&](double lat, double lon, float alt) {
		const hrt_abstime uav_time = advanceStep();
		publishUavGps(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, uav_time);
		const hrt_abstime target_time = advanceStep();
		publishTargetGnss(lat, lon, alt, 0.2f, 0.2f, target_time, true);
		_vte->update(matrix::Vector3f{});
		EXPECT_FALSE(_aid_gps_target_sub->update());
		vte_test::flushSubscription(_aid_gps_target_sub);
	};

	run_case(91.0, kUavLon, kTargetAltM);
	run_case(0.0, 0.0, kTargetAltM);
	run_case(kUavLat, kUavLon, 20000.f);
}

TEST_F(VTEPositionTest, FusesMeasurementsInTimestampOrder)
{
	// WHY: Fusions must occur in timestamp order to keep OOSM consistent.
	// WHAT: Apply non-zero vision then GNSS updates and verify ordering in the innovations.
	enableMask({FusionMaskOption::VisionPos, FusionMaskOption::MissionPos});

	static constexpr float kPosNoise = 1.f;
	static constexpr float kPosVar = kPosNoise * kPosNoise;
	setParamFloat("VTE_EVP_NOISE", kPosNoise);
	setParamFloat("VTE_GPS_P_NOISE", kPosNoise);
	setParamFloat("VTE_BIA_UNC_IN", 0.f);
	setParamFloat("VTE_POS_NIS_THRE", 100.f);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(0.f, 0.f, 0.f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	static constexpr double kMetersPerDegLat = 1.11111e5;
	static constexpr double kTargetNorthMeters = 20.0;

	const hrt_abstime init_time = advanceStep();
	_vte->set_mission_position(kUavLat, kUavLon, kUavAltM);
	publishUavGps(kUavLat, kUavLon, kUavAltM, kPosNoise, kPosNoise, vel_ned, 0.1f, true, init_time);
	publishVisionPos(matrix::Vector3f{}, vte_test::identityQuat(),
			 matrix::Vector3f(kPosVar, kPosVar, kPosVar), init_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	vte_test::flushSubscription(_aid_fiducial_sub);
	vte_test::flushSubscription(_aid_gps_mission_sub);
	vte_test::flushSubscription(_vte_state_sub);

	static constexpr hrt_abstime kBaseAgeUs = 10_ms;
	static constexpr hrt_abstime kGpsLagUs = 5_ms;
	const hrt_abstime base_time = nowUs() - kBaseAgeUs;
	const hrt_abstime vision_time = base_time;
	const hrt_abstime gps_time = base_time + kGpsLagUs;

	const double mission_lat = kUavLat + (kTargetNorthMeters / kMetersPerDegLat);
	_vte->set_mission_position(mission_lat, kUavLon, kUavAltM);

	publishUavGps(kUavLat, kUavLon, kUavAltM, kPosNoise, kPosNoise, vel_ned, 0.1f, true, gps_time);

	const matrix::Vector3f rel_pos(10.f, 0.f, 0.f);
	publishVisionPos(rel_pos, vte_test::identityQuat(),
			 matrix::Vector3f(kPosVar, kPosVar, kPosVar), vision_time);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	ASSERT_TRUE(_aid_gps_mission_sub->update());

	const auto aid_vision = _aid_fiducial_sub->get();
	const auto aid_gps = _aid_gps_mission_sub->get();

	EXPECT_GT(fabsf(aid_vision.innovation[0]), 1.f);
	EXPECT_NEAR(aid_vision.observation[0], rel_pos(0), 0.1f);
	EXPECT_NEAR(aid_gps.observation[0], kTargetNorthMeters, 0.6f);

	const float pred_state_vision = aid_vision.observation[0] - aid_vision.innovation[0];
	const float p_vision = aid_vision.innovation_variance[0] - aid_vision.observation_variance[0];
	const float k_vision = p_vision / aid_vision.innovation_variance[0];
	const float state_after_vision = pred_state_vision + k_vision * aid_vision.innovation[0];

	const float pred_state_gps = aid_gps.observation[0] - aid_gps.innovation[0];
	EXPECT_NEAR(pred_state_gps, state_after_vision, 0.5f);
}

TEST_F(VTEPositionTest, BiasSetWhenNonGpsArrives)
{
	// WHY: Bias is observable only after a non-GNSS position measurement arrives.
	// WHAT: Initialize with GNSS, then fuse vision and check bias update.
	enableMask({FusionMaskOption::VisionPos, FusionMaskOption::MissionPos});

	const matrix::Vector3f vel_ned(0.5f, -0.5f, 0.1f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime init_gps_time = advanceStep();
	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, init_gps_time);
	_vte->set_mission_position(kUavLat, kUavLon, kTargetAltM);

	_vte->update(matrix::Vector3f{});

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	const hrt_abstime gps_time = advanceStep();
	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, gps_time);
	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();
	EXPECT_NEAR(state.bias[0], -1.f, 0.1f);
	EXPECT_NEAR(state.bias[1], -2.f, 0.1f);
	EXPECT_NEAR(state.bias[2], 7.f, 0.1f);
	EXPECT_NEAR(state.cov_bias[0], 0.5f, kTolerance);
	EXPECT_NEAR(state.cov_bias[1], 0.5f, kTolerance);
	EXPECT_NEAR(state.cov_bias[2], 0.5f, kTolerance);
}

TEST_F(VTEPositionTest, VisionNoiseFloorAndRotation)
{
	// WHY: Vision measurements must be rotated to NED and respect noise floors.
	// WHAT: Rotate a vector by 180deg yaw and check floor in the innovation message.
	enableMask({FusionMaskOption::VisionPos});

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	matrix::Quaternionf q;
	q(0) = 0.f;
	q(1) = 0.f;
	q(2) = 0.f;
	q(3) = 1.f;

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(rel_pos, q, kZeroVec, vision_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();

	EXPECT_NEAR(aid.observation[0], -1.f, kTolerance);
	EXPECT_NEAR(aid.observation[1], -2.f, kTolerance);
	EXPECT_NEAR(aid.observation[2], 3.f, kTolerance);
	EXPECT_NEAR(aid.observation_variance[0], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[1], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[2], kDefaultEvPosVar, kTolerance);
}

TEST_F(VTEPositionTest, RejectsVisionWithInvalidCovarianceWhenNoiseModeOff)
{
	// WHY: Vision covariance must be finite when using message-provided noise.
	// WHAT: Publish NaN covariance and ensure the measurement is rejected.
	enableMask({FusionMaskOption::VisionPos});

	setParamInt("VTE_EV_NOISE_MD", 0);
	_vte->updateParamsPublic();

	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), vel_time);

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	const matrix::Vector3f cov(NAN, NAN, NAN);
	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(rel_pos, vte_test::identityQuat(), cov, vision_time);

	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_aid_fiducial_sub->update());
	EXPECT_FALSE(_vte_state_sub->update());
}

TEST_F(VTEPositionTest, RejectsVisionWithInvalidQuaternion)
{
	// WHY: Vision quaternion must be finite for rotation into NED.
	// WHAT: Publish NaN quaternion and ensure the measurement is rejected.
	enableMask({FusionMaskOption::VisionPos});

	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), vel_time);

	matrix::Quaternionf q = vte_test::identityQuat();
	q(0) = NAN;

	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(matrix::Vector3f(1.f, 0.f, 0.f), q, kSmallVisionCov, vision_time);
	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_aid_fiducial_sub->update());
	EXPECT_FALSE(_vte_state_sub->update());
}

TEST_F(VTEPositionTest, RejectsVisionWithInvalidPosition)
{
	// WHY: Position measurements must be finite to compute innovations.
	// WHAT: Publish NaN position and verify the update is rejected.
	enableMask({FusionMaskOption::VisionPos});

	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), vel_time);

	matrix::Vector3f rel_pos = matrix::Vector3f(1.f, 2.f, 3.f);
	rel_pos(1) = NAN;

	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time);
	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_aid_fiducial_sub->update());
	EXPECT_FALSE(_vte_state_sub->update());
}

TEST_F(VTEPositionTest, MissionGpsRelativePositionAndOffset)
{
	// WHY: Mission GNSS measurements must apply offset and variance floors.
	// WHAT: Publish GPS without offset, then apply an offset and verify observation changes.
	enableMask({FusionMaskOption::MissionPos});

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime t0 = advanceStep();
	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, vel_ned, 0.1f, true, t0);
	_vte->set_mission_position(kUavLat, kUavLon, kTargetAltM);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_mission_sub->update());
	const auto aid_no_offset = _aid_gps_mission_sub->get();

	_vte->set_gps_pos_offset(matrix::Vector3f(1.f, 2.f, 3.f), true);
	const hrt_abstime t1 = advanceStep();
	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, vel_ned, 0.1f, true, t1);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_mission_sub->update());
	const auto aid_offset = _aid_gps_mission_sub->get();

	EXPECT_NEAR(aid_offset.observation[0], aid_no_offset.observation[0] + 1.f, kTolerance);
	EXPECT_NEAR(aid_offset.observation[1], aid_no_offset.observation[1] + 2.f, kTolerance);
	EXPECT_NEAR(aid_offset.observation[2], aid_no_offset.observation[2] + 3.f, kTolerance);
	EXPECT_NEAR(aid_offset.observation_variance[0], kDefaultGpsPosVar, kTolerance);
	EXPECT_NEAR(aid_offset.observation_variance[1], kDefaultGpsPosVar, kTolerance);
	EXPECT_NEAR(aid_offset.observation_variance[2], kDefaultGpsPosVar, kTolerance);
}

TEST_F(VTEPositionTest, MissionGpsOffsetTimeoutRejectsMeasurement)
{
	// WHY: GPS offset should expire if it becomes stale.
	// WHAT: Force an offset timeout and ensure the mission GPS measurement is rejected.
	enableMask({FusionMaskOption::MissionPos});

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);
	_vte->set_meas_updated_timeout(1_ms);

	_vte->set_gps_pos_offset(matrix::Vector3f(1.f, 2.f, 3.f), true);
	_vte->set_mission_position(kUavLat, kUavLon, kTargetAltM);

	const hrt_abstime gps_time = advanceStep();
	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, vel_ned, 0.1f, true, gps_time);
	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_aid_gps_mission_sub->update());
}

TEST_F(VTEPositionTest, UavGpsVelocityFusionAndSign)
{
	// WHY: UAV GPS velocity fusion should update state and sign-converted outputs.
	// WHAT: Fuse GPS velocity and verify landing_target_pose uses negative sign.
	enableMask({FusionMaskOption::VisionPos, FusionMaskOption::UavGpsVel});

	const matrix::Vector3f init_vel(0.f, 0.f, 0.f);
	const hrt_abstime init_vel_time = advanceStep();
	setLocalVelocity(init_vel, init_vel_time);
	const hrt_abstime init_pos_time = advanceStep();
	setLocalPosition(matrix::Vector3f{}, init_pos_time);

	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(matrix::Vector3f(2.f, 0.f, -1.f), vte_test::identityQuat(), kSmallVisionCov, vision_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state_before = _vte_state_sub->get();

	const matrix::Vector3f vel_ned(0.8f, -1.2f, 0.4f);
	const hrt_abstime gps_time = advanceStep();
	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, gps_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_vel_uav_sub->update());
	const auto aid = _aid_gps_vel_uav_sub->get();
	EXPECT_NEAR(aid.observation_variance[0], kDefaultGpsVelVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[1], kDefaultGpsVelVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[2], kDefaultGpsVelVar, kTolerance);

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();
	ASSERT_TRUE(_target_pose_sub->update());
	const auto pose = _target_pose_sub->get();

	EXPECT_GT(fabsf(state.vel_uav[0] - state_before.vel_uav[0]), 0.1f);
	EXPECT_GT(fabsf(state.vel_uav[1] - state_before.vel_uav[1]), 0.1f);
	EXPECT_GT(fabsf(state.vel_uav[2] - state_before.vel_uav[2]), 0.1f);

	EXPECT_NEAR(pose.vx_rel, -state.vel_uav[0], 0.5f);
	EXPECT_NEAR(pose.vy_rel, -state.vel_uav[1], 0.5f);
	EXPECT_NEAR(pose.vz_rel, -state.vel_uav[2], 0.5f);

#if defined(CONFIG_VTEST_MOVING)
	EXPECT_FALSE(pose.is_static);
#else
	EXPECT_TRUE(pose.is_static);
#endif
}

TEST_F(VTEPositionTest, TargetGpsInterpolationVariance)
{
	// WHY: GNSS time misalignment should increase measurement variance.
	// WHAT: Publish delayed UAV GPS and verify propagation uncertainty is included.
	enableMask({FusionMaskOption::TargetGpsPos});

	static constexpr float kGpsPosNoise = 0.02f;
	setParamFloat("VTE_GPS_P_NOISE", kGpsPosNoise);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(10.f, 0.f, 0.f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime now = nowUs();
	const hrt_abstime uav_time = now - 300_ms;
	const hrt_abstime target_time = now - 100_ms;
	publishUavGps(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, vel_ned, 1.0f, true, uav_time);
	publishTargetGnss(kUavLat, kUavLon, kTargetAltM, 0.01f, 0.01f, target_time, true);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_target_sub->update());
	const auto aid = _aid_gps_target_sub->get();
	const float base_var = 2.f * kGpsPosNoise * kGpsPosNoise;
	const float expected_shift = -2.f; // 10 m/s north * 0.2 s => 2 m south

	EXPECT_NEAR(aid.observation[0], expected_shift, 0.05f);
	EXPECT_GT(aid.observation_variance[0], base_var);
	EXPECT_GT(aid.observation_variance[1], base_var);
	EXPECT_GT(aid.observation_variance[2], base_var);
}

TEST_F(VTEPositionTest, StaleMeasurementsIgnored)
{
	// WHY: Old measurements must be dropped to avoid stale fusion.
	// WHAT: Set a tiny timeout and ensure stale vision data is ignored.
	enableMask({FusionMaskOption::VisionPos});

	_vte->set_meas_recent_timeout(1_ms);

	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), vel_time);

	const hrt_abstime stale_time = nowUs() - 2_ms;
	publishVisionPos(matrix::Vector3f(1.f, 1.f, 1.f), vte_test::identityQuat(),
			 kSmallVisionCov, stale_time);

	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_aid_fiducial_sub->update());
	EXPECT_FALSE(_vte_state_sub->update());
}

#if defined(CONFIG_VTEST_MOVING)
TEST_F(VTEPositionTest, TargetVelocityFusionMoving)
{
	// WHY: Moving-target mode must fuse target velocity measurements.
	// WHAT: Provide target GNSS velocity and verify fusion output.
	enableMask({FusionMaskOption::VisionPos, FusionMaskOption::TargetGpsVel});

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	const hrt_abstime vel_time = advanceStep();
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime vision_time = advanceStep();
	publishVisionPos(matrix::Vector3f(0.f, 0.f, 0.f), vte_test::identityQuat(),
			 kSmallVisionCov, vision_time);
	_vte->update(matrix::Vector3f{});

	const matrix::Vector3f target_vel(1.f, 2.f, 3.f);
	const hrt_abstime target_time = advanceStep();
	publishTargetGnss(kUavLat, kUavLon, kTargetAltM, 0.5f, 0.5f, target_time, true, true, target_vel, 0.1f);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_vel_target_sub->update());
	const auto aid = _aid_gps_vel_target_sub->get();
	EXPECT_EQ(aid.fusion_status[0], static_cast<uint8_t>(vte::FusionStatus::FUSED_CURRENT));
	EXPECT_NEAR(aid.observation_variance[0], kDefaultGpsVelVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[1], kDefaultGpsVelVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[2], kDefaultGpsVelVar, kTolerance);
}
#endif

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
#include <lib/geo/geo.h>
#include <matrix/Quaternion.hpp>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/target_gnss.h>
#include <uORB/topics/vehicle_gnss.h>
#include <uORB/topics/vte_position.h>
#include <uORB/topics/vte_aid_source3d.h>
#include <uORB/topics/vte_bias_init_status.h>
#include <uORB/uORBManager.hpp>

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
static constexpr float kDefaultBiasUnc = 0.5f;
static constexpr float kDefaultEvPosVar = kDefaultEvPosNoise * kDefaultEvPosNoise;
static constexpr float kDefaultGpsPosVar = kDefaultGpsPosNoise * kDefaultGpsPosNoise;
static constexpr float kDefaultGpsVelVar = kDefaultGpsVelNoise * kDefaultGpsVelNoise;
static constexpr double kUavLat = 47.3977419;
static constexpr double kUavLon = 8.5455938;
static constexpr float kUavAltM = 100.f;
static constexpr float kTargetAltM = 90.f;

// NIS threshold high enough to accept any realistic innovation (used when a test is
// not exercising the gating logic itself).
static constexpr float kPermissiveNisThreshold = 100.f;
// NIS threshold tight enough to reject everything but zero innovations.
static constexpr float kRestrictiveNisThreshold = 0.11f;
// Keep-alive period between synthetic update ticks while vision is withheld.
static constexpr hrt_abstime kKeepAliveStepUs = 50_ms;

static const matrix::Vector3f kSmallVisionCov{0.01f, 0.01f, 0.01f};
static const matrix::Vector3f kZeroVec{0.f, 0.f, 0.f};

enum class FusionMaskOption {
	kVisionPos,
	kMissionPos,
	kTargetGpsPos,
	kUavGpsVel,
	kTargetGpsVel
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
	static void SetUpTestSuite() { uORB::Manager::initialize(); }

	void SetUp() override
	{
		param_control_autosave(false);
		vte_test::useFakeTime();

		setParamFloat("VTE_POS_UNC_IN", 1.f);
		setParamFloat("VTE_VEL_UNC_IN", 1.f);
		setParamFloat("VTE_BIA_UNC_IN", kDefaultBiasUnc);
		setParamFloat("VTE_EVP_NOISE", kDefaultEvPosNoise);
		setParamFloat("VTE_GPS_P_NOISE", kDefaultGpsPosNoise);
		setParamFloat("VTE_GPS_V_NOISE", kDefaultGpsVelNoise);
		setParamFloat("VTE_POS_NIS_THRE", 3.84f);
		setParamFloat("VTE_BIA_AVG_THR", 0.3f);
		setParamFloat("VTE_BIA_AVG_TOUT", 1.0f);

		_vte = std::make_unique<VTEPositionTestable>();
		ASSERT_TRUE(_vte->init());
		_vte->updateParamsPublic();

		_vision_pub = std::make_unique<uORB::Publication<fiducial_marker_pos_report_s>>(ORB_ID(fiducial_marker_pos_report));
		_uav_gnss_pub = std::make_unique<uORB::Publication<vehicle_gnss_s>>(ORB_ID(vehicle_gnss));
		_target_gps_pub = std::make_unique<uORB::Publication<target_gnss_s>>(ORB_ID(target_gnss));

		// Discard input samples retained from the previous test.
		_vision_pub->publish({});
		_uav_gnss_pub->publish({});
		_target_gps_pub->publish({});

		_aid_fiducial_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_fiducial_marker));
		_aid_gps_mission_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_pos_mission));
		_aid_gps_target_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_pos_target));
		_aid_gps_vel_uav_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_vel_uav));
		_aid_gps_vel_target_sub = std::make_unique<uORB::SubscriptionData<vte_aid_source3d_s>>(ORB_ID(vte_aid_gps_vel_target));
		_bias_init_status_sub = std::make_unique<uORB::SubscriptionData<vte_bias_init_status_s>>(ORB_ID(vte_bias_init_status));
		_vte_state_sub = std::make_unique<uORB::SubscriptionData<vte_position_s>>(ORB_ID(vte_position));
		_target_pose_sub = std::make_unique<uORB::SubscriptionData<landing_target_pose_s>>(ORB_ID(landing_target_pose));

		vte_test::flushSubscription(_aid_fiducial_sub);
		vte_test::flushSubscription(_aid_gps_mission_sub);
		vte_test::flushSubscription(_aid_gps_target_sub);
		vte_test::flushSubscription(_aid_gps_vel_uav_sub);
		vte_test::flushSubscription(_aid_gps_vel_target_sub);
		vte_test::flushSubscription(_bias_init_status_sub);
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
		_bias_init_status_sub.reset();
		_target_gps_pub.reset();
		_uav_gnss_pub.reset();
		_vision_pub.reset();

		_vte.reset();
		vte_test::useRealTime();
		// Keep the uORB manager alive; parameters retain uORB handles across tests.
	}

	void setParamFloat(const char *name, float value)
	{
		ASSERT_TRUE(_param_guard.setFloat(name, value));
	}

	void enableMask(std::initializer_list<FusionMaskOption> sources)
	{
		vte::SensorFusionMaskU mask{};

		for (const FusionMaskOption source : sources) {
			switch (source) {
			case FusionMaskOption::kVisionPos:
				mask.flags.use_vision_pos = 1;
				break;

			case FusionMaskOption::kMissionPos:
				mask.flags.use_mission_pos = 1;
				break;

			case FusionMaskOption::kTargetGpsPos:
				mask.flags.use_target_gps_pos = 1;
				break;

			case FusionMaskOption::kUavGpsVel:
				mask.flags.use_uav_gps_vel = 1;
				break;

			case FusionMaskOption::kTargetGpsVel:
				mask.flags.use_target_gps_vel = 1;
				break;
			}
		}

		_vte->setVteAidMask(mask.value);
	}

	void publishVisionPos(const matrix::Vector3f &rel_pos, const matrix::Quaternionf &q,
			      const matrix::Vector3f &cov, hrt_abstime timestamp)
	{
		ASSERT_TRUE(vte_test::publishVisionPos(*_vision_pub, rel_pos, q, cov, timestamp));
	}

	void publishUavGnss(double lat, double lon, float alt, float eph, float epv,
			    const matrix::Vector3f &vel_ned, float vel_var, bool vel_valid, hrt_abstime timestamp)
	{
		ASSERT_TRUE(vte_test::publishUavGnss(*_uav_gnss_pub, lat, lon, alt, eph, epv, vel_ned, vel_var, vel_valid, timestamp));
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
		_vte->setLocalVelocity(vel_ned, true, timestamp);
	}

	void setLocalPosition(const matrix::Vector3f &pos_ned, hrt_abstime timestamp)
	{
		_vte->setLocalPosition(pos_ned, true, timestamp);
	}

	void publishNominalUavGnss(const matrix::Vector3f &vel_ned, hrt_abstime timestamp)
	{
		publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, timestamp);
	}

	void initializeFromMissionGnss(const matrix::Vector3f &vel_ned, double mission_lat = kUavLat,
				       double mission_lon = kUavLon, float mission_alt = kTargetAltM)
	{
		const hrt_abstime gps_time = vte_test::advanceMicroseconds(kStepUs);
		publishNominalUavGnss(vel_ned, gps_time);
		_vte->setMissionPosition(mission_lat, mission_lon, mission_alt);
		_vte->update(matrix::Vector3f{});
		ASSERT_TRUE(_vte_state_sub->update());
	}

	void flushVisionAndMissionOutputs()
	{
		vte_test::flushSubscription(_aid_fiducial_sub);
		vte_test::flushSubscription(_aid_gps_mission_sub);
		vte_test::flushSubscription(_bias_init_status_sub);
		vte_test::flushSubscription(_vte_state_sub);
	}

	void flushMissionOutputs()
	{
		vte_test::flushSubscription(_vte_state_sub);
		vte_test::flushSubscription(_aid_gps_mission_sub);
	}

	void expectBiasNear(const vte_position_s &state, const matrix::Vector3f &expected_bias,
			    float tolerance)
	{
		EXPECT_NEAR(state.bias[0], expected_bias(0), tolerance);
		EXPECT_NEAR(state.bias[1], expected_bias(1), tolerance);
		EXPECT_NEAR(state.bias[2], expected_bias(2), tolerance);
	}

	/**
	 * Advance the filter by roughly @p duration_us while keeping the UAV GPS stream alive
	 * so bias averaging keeps ticking.
	 */
	void advanceEstimatorWithGnss(hrt_abstime duration_us, const matrix::Vector3f &vel_ned)
	{
		hrt_abstime elapsed = 0;

		while (elapsed < duration_us) {
			const hrt_abstime remaining = duration_us - elapsed;
			const hrt_abstime step = (remaining < kKeepAliveStepUs) ? remaining : kKeepAliveStepUs;
			const hrt_abstime now = vte_test::advanceMicroseconds(step);

			publishNominalUavGnss(vel_ned, now);
			_vte->update(matrix::Vector3f{});
			flushMissionOutputs();
			elapsed += step;
		}
	}

	std::unique_ptr<VTEPositionTestable> _vte;
	std::unique_ptr<uORB::Publication<fiducial_marker_pos_report_s>> _vision_pub;
	std::unique_ptr<uORB::Publication<vehicle_gnss_s>> _uav_gnss_pub;
	std::unique_ptr<uORB::Publication<target_gnss_s>> _target_gps_pub;

	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_fiducial_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_mission_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_target_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_vel_uav_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_aid_source3d_s>> _aid_gps_vel_target_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_bias_init_status_s>> _bias_init_status_sub;
	std::unique_ptr<uORB::SubscriptionData<vte_position_s>> _vte_state_sub;
	std::unique_ptr<uORB::SubscriptionData<landing_target_pose_s>> _target_pose_sub;

	vte_test::ParamGuard _param_guard{};
};

// WHY: Vision + velocity should be sufficient to start the filter.
// WHAT: Verify initial state matches the vision position and local velocity.
TEST_F(VTEPositionTest, InitWithVisionAndLocalVelocity)
{
	// GIVEN: both the required vision position and local velocity inputs are available.
	enableMask({FusionMaskOption::kVisionPos});

	const matrix::Vector3f rel_pos(4.f, -2.f, 1.f);
	const matrix::Vector3f vel_ned(1.f, 2.f, 3.f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime vision_time = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(rel_pos, vte_test::identityQuat(), kZeroVec, vision_time);

	// WHEN: the estimator performs its first update.
	_vte->update(matrix::Vector3f{});

	// THEN: the state initializes directly from those measurements.
	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();

	EXPECT_NEAR(state.rel_pos[0], rel_pos(0), kTolerance);
	EXPECT_NEAR(state.rel_pos[1], rel_pos(1), kTolerance);
	EXPECT_NEAR(state.rel_pos[2], rel_pos(2), kTolerance);
	EXPECT_TRUE(state.rel_pos_valid);
	EXPECT_TRUE(state.rel_vel_valid);
	EXPECT_NEAR(state.vel_uav[0], vel_ned(0), kTolerance);
	EXPECT_NEAR(state.vel_uav[1], vel_ned(1), kTolerance);
	EXPECT_NEAR(state.vel_uav[2], vel_ned(2), kTolerance);

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();
	EXPECT_NEAR(aid.observation_variance[0], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[1], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[2], kDefaultEvPosVar, kTolerance);
}

// WHY: The first vision sample after GNSS can be noisy and should not set the GNSS bias immediately.
// WHAT: Initialize from GNSS, provide one vision sample to start averaging,
// then require the raw bias to stay close to the LPF for a minimum averaging time before activating the bias.
TEST_F(VTEPositionTest, BiasAveragingDelaysVisionFusionUntilStable)
{
	// GIVEN: mission GNSS initializes first and bias averaging requires the raw bias to settle around the LPF.
	enableMask({FusionMaskOption::kVisionPos, FusionMaskOption::kMissionPos});

	setParamFloat("VTE_BIA_AVG_THR", 0.05f);
	setParamFloat("VTE_BIA_AVG_TOUT", 2.0f);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(0.5f, -0.5f, 0.1f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	initializeFromMissionGnss(vel_ned);
	flushVisionAndMissionOutputs();

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	const hrt_abstime gps_time_1 = vte_test::advanceMicroseconds(kStepUs);
	publishNominalUavGnss(vel_ned, gps_time_1);
	const hrt_abstime vision_time_1 = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time_1);

	// WHEN: the first vision sample starts bias averaging.
	_vte->update(matrix::Vector3f{});

	// THEN: the raw bias is tracked, but the active state keeps the pre-bias GNSS-relative position.
	ASSERT_TRUE(_vte_state_sub->update());
	ASSERT_TRUE(_bias_init_status_sub->update());
	const auto initial_bias_status = _bias_init_status_sub->get();
	const auto pending_state = _vte_state_sub->get();
	EXPECT_FALSE(_aid_fiducial_sub->update());
	const float altitude_bias = (kUavAltM - kTargetAltM) - rel_pos(2);
	const matrix::Vector3f expected_bias(-rel_pos(0), -rel_pos(1), altitude_bias);
	EXPECT_NEAR(initial_bias_status.raw_bias[0], expected_bias(0), kTolerance);
	EXPECT_NEAR(initial_bias_status.raw_bias[1], expected_bias(1), kTolerance);
	EXPECT_NEAR(initial_bias_status.raw_bias[2], expected_bias(2), kTolerance);
	EXPECT_NEAR(initial_bias_status.filtered_bias[0], expected_bias(0), kTolerance);
	EXPECT_NEAR(initial_bias_status.filtered_bias[1], expected_bias(1), kTolerance);
	EXPECT_NEAR(initial_bias_status.filtered_bias[2], expected_bias(2), kTolerance);
	EXPECT_NEAR(initial_bias_status.delta_norm, 0.f, kTolerance);
	expectBiasNear(pending_state, kZeroVec, kTolerance);
	EXPECT_NEAR(pending_state.rel_pos[0], 0.f, 0.1f);
	EXPECT_NEAR(pending_state.rel_pos[1], 0.f, 0.1f);
	EXPECT_NEAR(pending_state.rel_pos[2], kUavAltM - kTargetAltM, 0.1f);

	// Each of the advance blocks below drives 200 ms of keep-alive GPS without refreshing
	// local velocity. This exercises the min-time / stable-delta parts of the averaging path.
	static constexpr hrt_abstime kAdvanceStepUs = 4 * kKeepAliveStepUs;

	advanceEstimatorWithGnss(kAdvanceStepUs, vel_ned);
	const hrt_abstime vision_time_2 = vte_test::nowUs();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time_2);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto still_pending_state = _vte_state_sub->get();
	EXPECT_FALSE(_aid_fiducial_sub->update());
	expectBiasNear(still_pending_state, kZeroVec, kTolerance);
	vte_test::flushSubscription(_vte_state_sub);

	advanceEstimatorWithGnss(kAdvanceStepUs, vel_ned);
	const hrt_abstime vision_time_3 = vte_test::nowUs();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time_3);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto min_time_not_reached_state = _vte_state_sub->get();
	EXPECT_FALSE(_aid_fiducial_sub->update());
	expectBiasNear(min_time_not_reached_state, kZeroVec, kTolerance);
	vte_test::flushSubscription(_vte_state_sub);

	advanceEstimatorWithGnss(kAdvanceStepUs, vel_ned);
	const hrt_abstime vision_time_4 = vte_test::nowUs();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time_4);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto still_waiting_for_lpf_settle_state = _vte_state_sub->get();
	EXPECT_FALSE(_aid_fiducial_sub->update());
	expectBiasNear(still_waiting_for_lpf_settle_state, kZeroVec, kTolerance);
	vte_test::flushSubscription(_vte_state_sub);

	advanceEstimatorWithGnss(kAdvanceStepUs, vel_ned);
	const hrt_abstime vision_time_5 = vte_test::nowUs();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time_5);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto still_waiting_for_final_lpf_delta_state = _vte_state_sub->get();
	EXPECT_FALSE(_aid_fiducial_sub->update());
	expectBiasNear(still_waiting_for_final_lpf_delta_state, kZeroVec, kTolerance);
	vte_test::flushSubscription(_vte_state_sub);

	advanceEstimatorWithGnss(kAdvanceStepUs, vel_ned);
	const hrt_abstime vision_time_6 = vte_test::nowUs();
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, vision_time_6);

	// WHEN: the minimum time and raw-to-LPF delta requirements are finally both satisfied.
	_vte->update(matrix::Vector3f{});

	// THEN: bias averaging completes and vision starts fusing with the initialized bias.
	ASSERT_TRUE(_aid_fiducial_sub->update());
	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();
	expectBiasNear(state, expected_bias, 0.1f);
	EXPECT_NEAR(state.cov_bias[0], kDefaultBiasUnc, kTolerance);
	EXPECT_NEAR(state.cov_bias[1], kDefaultBiasUnc, kTolerance);
	EXPECT_NEAR(state.cov_bias[2], kDefaultBiasUnc, kTolerance);
}

// WHY: When the vision timestamp is newer than a still-valid GNSS-relative sample, the bias logic should
// propagate that GNSS measurement with the UAV velocity estimate before comparing it to vision.
// WHAT: Keep the second vision sample inside the GNSS validity window, force a timeout exit, and verify
// the resulting bias matches the velocity-compensated GNSS/vision difference.
TEST_F(VTEPositionTest, BiasAveragingPropagatesValidGnssUsingVelocity)
{
	// GIVEN: mission GNSS initializes the filter and bias averaging waits long enough to compare a propagated GNSS sample.
	enableMask({FusionMaskOption::kVisionPos, FusionMaskOption::kMissionPos});

	setParamFloat("VTE_BIA_AVG_THR", 0.05f);
	setParamFloat("VTE_BIA_AVG_TOUT", 0.06f);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(5.0f, -3.0f, 1.5f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	initializeFromMissionGnss(vel_ned);
	flushVisionAndMissionOutputs();

	const matrix::Vector3f initial_gnss_rel(0.f, 0.f, kUavAltM - kTargetAltM);
	const matrix::Vector3f expected_bias(0.6f, -0.4f, 1.2f);

	const hrt_abstime vision_time_1 = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(initial_gnss_rel - expected_bias, vte_test::identityQuat(), kSmallVisionCov, vision_time_1);

	// WHEN: the first vision sample begins bias averaging.
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	EXPECT_FALSE(_aid_fiducial_sub->update());
	vte_test::flushSubscription(_bias_init_status_sub);
	vte_test::flushSubscription(_vte_state_sub);

	const hrt_abstime vision_time_2 = vte_test::advanceMicroseconds(60_ms);
	setLocalVelocity(vel_ned, vision_time_2);
	const matrix::Vector3f propagated_gnss = initial_gnss_rel - vel_ned * 0.06f;
	publishVisionPos(propagated_gnss - expected_bias, vte_test::identityQuat(), kSmallVisionCov, vision_time_2);

	// WHEN: the next vision sample arrives while the earlier GNSS-relative sample is still valid.
	_vte->update(matrix::Vector3f{});

	// THEN: the bias is computed from the velocity-propagated GNSS sample rather than the stale raw position.
	ASSERT_TRUE(_bias_init_status_sub->update());
	const auto propagated_bias_status = _bias_init_status_sub->get();
	ASSERT_TRUE(_aid_fiducial_sub->update());
	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();
	EXPECT_NEAR(propagated_bias_status.raw_bias[0], expected_bias(0), 0.05f);
	EXPECT_NEAR(propagated_bias_status.raw_bias[1], expected_bias(1), 0.05f);
	EXPECT_NEAR(propagated_bias_status.raw_bias[2], expected_bias(2), 0.05f);
	EXPECT_LT(propagated_bias_status.delta_norm, 0.05f);
	expectBiasNear(state, expected_bias, 0.05f);
}

// WHY: The vision sample can be older than the latest GNSS sample.
// WHAT: Publish a newer mission/GNSS relative position from a moved UAV and verify the bias setup
// back-propagates that GNSS sample before initializing the bias, even if the state then advances
// to the newer GNSS observation in the same update.
TEST_F(VTEPositionTest, BiasInitializationBackPropagatesNewerGnssSampleToOlderVision)
{
	// GIVEN: vision is timestamped earlier than the GNSS-relative mission sample.
	enableMask({FusionMaskOption::kVisionPos, FusionMaskOption::kMissionPos});

	setParamFloat("VTE_BIA_AVG_TOUT", 0.f);
	setParamFloat("VTE_POS_NIS_THRE", kPermissiveNisThreshold);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(5.f, -3.f, 0.f);
	_vte->setMissionPosition(kUavLat, kUavLon, kUavAltM);

	const hrt_abstime vision_time = vte_test::advanceMicroseconds(kStepUs);
	const hrt_abstime gps_time = vte_test::advanceMicroseconds(50_ms);

	double moved_lat = kUavLat;
	double moved_lon = kUavLon;
	add_vector_to_global_position(kUavLat, kUavLon, vel_ned(0) * 0.05f, vel_ned(1) * 0.05f, &moved_lat, &moved_lon);

	setLocalVelocity(vel_ned, gps_time);
	publishUavGnss(moved_lat, moved_lon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, gps_time);
	publishVisionPos(kZeroVec, vte_test::identityQuat(), kSmallVisionCov, vision_time);

	// WHEN: both samples are processed together.
	_vte->update(matrix::Vector3f{});

	// THEN: bias initialization back-propagates GNSS to the older vision time, leaving zero bias here.
	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();

	expectBiasNear(state, kZeroVec, 0.1f);
}

// WHY: Bias averaging must not wait forever when GNSS/vision agreement stays poor.
// WHAT: Force a tiny timeout with inconsistent vision samples and expect the LPF bias to activate once the timeout expires.
TEST_F(VTEPositionTest, BiasAveragingUsesTimeoutWhenSamplesDoNotStabilize)
{
	// GIVEN: mission GNSS initializes first, but the incoming vision samples never stabilize.
	enableMask({FusionMaskOption::kVisionPos, FusionMaskOption::kMissionPos});

	setParamFloat("VTE_BIA_AVG_THR", 0.001f);
	setParamFloat("VTE_BIA_AVG_TOUT", 0.002f);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(0.5f, -0.5f, 0.1f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	initializeFromMissionGnss(vel_ned);
	flushVisionAndMissionOutputs();

	const matrix::Vector3f rel_pos_first(1.f, 0.f, 3.f);
	const hrt_abstime gps_time_1 = vte_test::advanceMicroseconds(kStepUs);
	publishNominalUavGnss(vel_ned, gps_time_1);
	const hrt_abstime vision_time_1 = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(rel_pos_first, vte_test::identityQuat(), kSmallVisionCov, vision_time_1);

	// WHEN: the first vision sample starts averaging.
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	EXPECT_FALSE(_aid_fiducial_sub->update());
	vte_test::flushSubscription(_vte_state_sub);

	vte_test::advanceMicroseconds(3_ms);
	setParamFloat("VTE_POS_NIS_THRE", kRestrictiveNisThreshold);
	_vte->updateParamsPublic();

	const matrix::Vector3f rel_pos_second(4.f, -2.f, 6.f);
	const hrt_abstime gps_time_2 = vte_test::advanceMicroseconds(kStepUs);
	publishNominalUavGnss(vel_ned, gps_time_2);
	const hrt_abstime vision_time_2 = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(rel_pos_second, vte_test::identityQuat(), kSmallVisionCov, vision_time_2);

	// WHEN: the timeout has expired before the next inconsistent sample arrives.
	_vte->update(matrix::Vector3f{});

	// THEN: the filtered bias activates instead of waiting for stability that never comes.
	ASSERT_TRUE(_aid_fiducial_sub->update());
	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();
	const float altitude_bias_first = (kUavAltM - kTargetAltM) - rel_pos_first(2);
	const matrix::Vector3f expected_bias_first(-rel_pos_first(0), -rel_pos_first(1), altitude_bias_first);
	const matrix::Vector3f expected_gnss_rel(0.f, 0.f, kUavAltM - kTargetAltM);

	expectBiasNear(state, expected_bias_first, 0.3f);
	EXPECT_NEAR(state.rel_pos[0] + state.bias[0], expected_gnss_rel(0), 0.3f);
	EXPECT_NEAR(state.rel_pos[1] + state.bias[1], expected_gnss_rel(1), 0.3f);
	EXPECT_NEAR(state.rel_pos[2] + state.bias[2], expected_gnss_rel(2), 0.3f);
	EXPECT_NEAR(state.rel_pos[0], rel_pos_first(0), 0.3f);
	EXPECT_NEAR(state.rel_pos[1], rel_pos_first(1), 0.3f);
	EXPECT_NEAR(state.rel_pos[2], rel_pos_first(2), 0.3f);
}

// WHY: Once the GNSS/vision bias has been set, temporary vision loss must not restart bias initialization.
// WHAT: Initialize the bias immediately, fuse GNSS alone for one cycle, then restore vision and verify
// that the same bias stays active without re-entering the initialization path.
TEST_F(VTEPositionTest, VisionReturnAfterTemporaryLossDoesNotReinitializeBias)
{
	enableMask({FusionMaskOption::kVisionPos, FusionMaskOption::kMissionPos});

	setParamFloat("VTE_BIA_AVG_TOUT", 0.f);
	setParamFloat("VTE_POS_NIS_THRE", kPermissiveNisThreshold);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(0.5f, -0.5f, 0.1f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	_vte->setMissionPosition(kUavLat, kUavLon, kTargetAltM);

	const hrt_abstime init_gps_time = vte_test::advanceMicroseconds(kStepUs);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, init_gps_time);
	const hrt_abstime init_vision_time = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, init_vision_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	ASSERT_TRUE(_aid_gps_mission_sub->update());
	ASSERT_TRUE(_vte_state_sub->update());
	EXPECT_FALSE(_bias_init_status_sub->update());
	const auto initial_state = _vte_state_sub->get();
	const float altitude_bias = (kUavAltM - kTargetAltM) - rel_pos(2);
	const matrix::Vector3f expected_bias(-rel_pos(0), -rel_pos(1), altitude_bias);
	expectBiasNear(initial_state, expected_bias, 0.1f);

	vte_test::flushSubscription(_aid_fiducial_sub);
	vte_test::flushSubscription(_aid_gps_mission_sub);
	vte_test::flushSubscription(_bias_init_status_sub);
	vte_test::flushSubscription(_vte_state_sub);

	const hrt_abstime gnss_only_time = vte_test::advanceMicroseconds(kStepUs);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, gnss_only_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_mission_sub->update());
	ASSERT_TRUE(_vte_state_sub->update());
	EXPECT_FALSE(_aid_fiducial_sub->update());
	EXPECT_FALSE(_bias_init_status_sub->update());
	const auto gnss_only_state = _vte_state_sub->get();
	EXPECT_GT(fabsf(gnss_only_state.bias[0]), 0.5f);
	EXPECT_GT(fabsf(gnss_only_state.bias[1]), 1.0f);
	EXPECT_GT(fabsf(gnss_only_state.bias[2]), 0.5f);

	vte_test::flushSubscription(_aid_fiducial_sub);
	vte_test::flushSubscription(_aid_gps_mission_sub);
	vte_test::flushSubscription(_bias_init_status_sub);
	vte_test::flushSubscription(_vte_state_sub);

	const hrt_abstime return_gps_time = vte_test::advanceMicroseconds(kStepUs);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, return_gps_time);
	const hrt_abstime return_vision_time = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(rel_pos, vte_test::identityQuat(), kSmallVisionCov, return_vision_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	ASSERT_TRUE(_aid_gps_mission_sub->update());
	ASSERT_TRUE(_vte_state_sub->update());
	EXPECT_FALSE(_bias_init_status_sub->update());
	const auto state = _vte_state_sub->get();
	EXPECT_NEAR(state.bias[0], gnss_only_state.bias[0], 0.5f);
	EXPECT_NEAR(state.bias[1], gnss_only_state.bias[1], 0.5f);
	EXPECT_NEAR(state.bias[2], gnss_only_state.bias[2], 0.5f);
}

// WHY: Vision measurements must be rotated to NED and respect noise floors.
// WHAT: Rotate a vector by 180deg yaw and check floor in the innovation message.
TEST_F(VTEPositionTest, VisionNoiseFloorAndRotation)
{
	enableMask({FusionMaskOption::kVisionPos});

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	const matrix::Quaternionf q{0.f, 0.f, 0.f, 1.f};

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	const hrt_abstime vision_time = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(rel_pos, q, kZeroVec, vision_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();

	const matrix::Vector3f expected_obs = q.rotateVector(rel_pos);
	EXPECT_NEAR(aid.observation[0], expected_obs(0), kTolerance);
	EXPECT_NEAR(aid.observation[1], expected_obs(1), kTolerance);
	EXPECT_NEAR(aid.observation[2], expected_obs(2), kTolerance);
	EXPECT_NEAR(aid.observation_variance[0], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[1], kDefaultEvPosVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[2], kDefaultEvPosVar, kTolerance);
}

// WHY: Mission GNSS measurements must apply offset and variance floors.
// WHAT: Publish GPS without offset, then apply an offset and verify
// observation changes.
TEST_F(VTEPositionTest, MissionGpsRelativePositionAndOffset)
{
	enableMask({FusionMaskOption::kMissionPos});

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime t0 = vte_test::advanceMicroseconds(kStepUs);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, vel_ned, 0.1f, true, t0);
	_vte->setMissionPosition(kUavLat, kUavLon, kTargetAltM);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_mission_sub->update());
	const auto aid_no_offset = _aid_gps_mission_sub->get();

	const matrix::Vector3f gps_offset(1.f, 2.f, 3.f);
	_vte->setGpsPosOffset(gps_offset, true);
	const hrt_abstime t1 = vte_test::advanceMicroseconds(kStepUs);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, vel_ned, 0.1f, true, t1);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_mission_sub->update());
	const auto aid_offset = _aid_gps_mission_sub->get();

	EXPECT_NEAR(aid_offset.observation[0], aid_no_offset.observation[0] + gps_offset(0), kTolerance);
	EXPECT_NEAR(aid_offset.observation[1], aid_no_offset.observation[1] + gps_offset(1), kTolerance);
	EXPECT_NEAR(aid_offset.observation[2], aid_no_offset.observation[2] + gps_offset(2), kTolerance);
	EXPECT_NEAR(aid_offset.observation_variance[0], kDefaultGpsPosVar, kTolerance);
	EXPECT_NEAR(aid_offset.observation_variance[1], kDefaultGpsPosVar, kTolerance);
	EXPECT_NEAR(aid_offset.observation_variance[2], kDefaultGpsPosVar, kTolerance);
}

// WHY: UAV GPS velocity fusion should update state and sign-converted outputs.
// WHAT: Fuse GPS velocity and verify landing_target_pose uses negative sign.
TEST_F(VTEPositionTest, UavGpsVelocityFusionAndSign)
{
	enableMask({FusionMaskOption::kVisionPos, FusionMaskOption::kUavGpsVel});

	const matrix::Vector3f init_vel(0.f, 0.f, 0.f);
	const hrt_abstime init_vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(init_vel, init_vel_time);
	const hrt_abstime init_pos_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalPosition(matrix::Vector3f{}, init_pos_time);

	const hrt_abstime vision_time = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(matrix::Vector3f(2.f, 0.f, -1.f), vte_test::identityQuat(),
			 kSmallVisionCov, vision_time);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state_before = _vte_state_sub->get();

	const matrix::Vector3f vel_ned(0.8f, -1.2f, 0.4f);
	const hrt_abstime gps_time = vte_test::advanceMicroseconds(kStepUs);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, gps_time);
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
	EXPECT_TRUE(pose.rel_pos_valid);
	EXPECT_TRUE(pose.rel_vel_valid);

#if defined(CONFIG_VTEST_MOVING)
	EXPECT_FALSE(pose.is_static);
	EXPECT_FALSE(pose.rel_vel_ekf2_valid);
#else
	EXPECT_TRUE(pose.is_static);
	EXPECT_TRUE(pose.rel_vel_ekf2_valid);
#endif
}

// WHY: EKF2 auxiliary-velocity aiding must require a recent relative
// measurement, while guidance can still use a finite relative velocity estimate.
// WHAT: Initialize from target GNSS only and verify the velocity-valid flag
// remains true but the EKF2-specific aiding flag stays false.
TEST_F(VTEPositionTest, GpsOnlyRelativeVelocityDoesNotEnableEkf2Aiding)
{
	enableMask({FusionMaskOption::kTargetGpsPos});

	const matrix::Vector3f vel_ned(0.3f, -0.2f, 0.1f);
	const hrt_abstime local_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, local_time);
	setLocalPosition(matrix::Vector3f{}, local_time);

	const hrt_abstime gps_time = vte_test::advanceMicroseconds(kStepUs);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.2f, 0.2f, vel_ned, 0.1f, true, gps_time);
	publishTargetGnss(kUavLat, kUavLon, kTargetAltM, 0.2f, 0.2f, gps_time, true);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_target_pose_sub->update());
	const auto pose = _target_pose_sub->get();
	EXPECT_TRUE(pose.rel_pos_valid);
	EXPECT_TRUE(pose.rel_vel_valid);
	EXPECT_FALSE(pose.rel_vel_ekf2_valid);
}

// WHY: GNSS time misalignment should increase measurement variance.
// WHAT: Publish delayed UAV GPS and verify propagation uncertainty is included.
TEST_F(VTEPositionTest, TargetGpsInterpolationVariance)
{
	enableMask({FusionMaskOption::kTargetGpsPos});

	static constexpr float kGpsPosNoise = 0.02f;
	setParamFloat("VTE_GPS_P_NOISE", kGpsPosNoise);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(10.f, 0.f, 0.f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime now = vte_test::nowUs();
	const hrt_abstime uav_time = now - 300_ms;
	const hrt_abstime target_time = now - 100_ms;
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, vel_ned, 1.0f, true, uav_time);
	publishTargetGnss(kUavLat, kUavLon, kTargetAltM, 0.01f, 0.01f, target_time, true);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_target_sub->update());
	const auto aid = _aid_gps_target_sub->get();
	const float base_var = 2.f * kGpsPosNoise * kGpsPosNoise;
	const float time_diff_s = (target_time - uav_time) * 1e-6f;
	const float expected_shift = -vel_ned(0) * time_diff_s;

	EXPECT_NEAR(aid.observation[0], expected_shift, 0.05f);
	EXPECT_GT(aid.observation_variance[0], base_var);
	EXPECT_GT(aid.observation_variance[1], base_var);
	EXPECT_GT(aid.observation_variance[2], base_var);
}

// WHY: GNSS latency compensation should use vehicle CoM velocity rather than
// antenna velocity.
// WHAT: Apply a lever-arm velocity correction and verify
// the propagated target observation reflects it.
TEST_F(VTEPositionTest, TargetGpsInterpolationUsesLeverArmCorrectedVelocity)
{
	enableMask({FusionMaskOption::kTargetGpsPos});
	_vte->setMeasUpdatedTimeout(400_ms);

	const matrix::Vector3f vel_com_ned(7.f, 0.f, 0.f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_com_ned, vel_time);

	_vte->setGpsPosOffset(matrix::Vector3f(0.5f, 0.f, 0.f), true);
	_vte->setVelOffset(matrix::Vector3f(3.f, 0.f, 0.f));

	const hrt_abstime now = vte_test::nowUs();
	const hrt_abstime uav_time = now - 300_ms;
	const hrt_abstime target_time = now - 100_ms;
	const matrix::Vector3f antenna_vel_ned(10.f, 0.f, 0.f);
	publishUavGnss(kUavLat, kUavLon, kUavAltM, 0.01f, 0.01f, antenna_vel_ned, 1.0f, true,
		       uav_time);
	publishTargetGnss(kUavLat, kUavLon, kTargetAltM, 0.01f, 0.01f, target_time, true);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_target_sub->update());
	const auto aid = _aid_gps_target_sub->get();
	const float time_diff_s = (target_time - uav_time) * 1e-6f;
	const float expected_shift = -vel_com_ned(0) * time_diff_s + 0.5f;

	EXPECT_NEAR(aid.observation[0], expected_shift, 0.05f);
}

// WHY: A long scheduling gap should reset the filter instead of predicting across stale time.
// WHAT: Initialize once, wait past the allowed prediction gap, then verify a fresh measurement re-initializes cleanly.
TEST_F(VTEPositionTest, ResetsFilterOnStalePredictionGap)
{
	const matrix::Vector3f initial_rel_pos(1.f, 0.5f, -2.f);
	const matrix::Vector3f fresh_rel_pos(-0.5f, 1.25f, -1.5f);

	enableMask({FusionMaskOption::kVisionPos});
	_vte->setMeasUpdatedTimeout(500_ms);
	setLocalVelocity(matrix::Vector3f{}, vte_test::advanceMicroseconds(kStepUs));

	publishVisionPos(initial_rel_pos, vte_test::identityQuat(), kSmallVisionCov,
			 vte_test::advanceMicroseconds(kStepUs));
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_vte_state_sub->update());
	const auto initial_state = _vte_state_sub->get();
	EXPECT_NEAR(initial_state.rel_pos[0], initial_rel_pos(0), kTolerance);
	EXPECT_NEAR(initial_state.rel_pos[1], initial_rel_pos(1), kTolerance);
	EXPECT_NEAR(initial_state.rel_pos[2], initial_rel_pos(2), kTolerance);

	vte_test::flushSubscription(_aid_fiducial_sub);
	vte_test::flushSubscription(_vte_state_sub);

	vte_test::advanceMicroseconds(120_ms);
	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_aid_fiducial_sub->update());
	EXPECT_FALSE(_vte_state_sub->update());

	publishVisionPos(fresh_rel_pos, vte_test::identityQuat(), kSmallVisionCov,
			 vte_test::advanceMicroseconds(kStepUs));
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();
	EXPECT_EQ(aid.fusion_status[0], static_cast<uint8_t>(vte::FusionStatus::STATUS_FUSED_CURRENT));

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();
	EXPECT_NEAR(state.rel_pos[0], fresh_rel_pos(0), kTolerance);
	EXPECT_NEAR(state.rel_pos[1], fresh_rel_pos(1), kTolerance);
	EXPECT_NEAR(state.rel_pos[2], fresh_rel_pos(2), kTolerance);
	EXPECT_NEAR(state.vel_uav[0], 0.f, kTolerance);
	EXPECT_NEAR(state.vel_uav[1], 0.f, kTolerance);
	EXPECT_NEAR(state.vel_uav[2], 0.f, kTolerance);
}

#if defined(CONFIG_VTEST_MOVING)
// WHY: Moving-target mode must fuse target velocity measurements.
// WHAT: Provide target GNSS velocity and verify fusion output.
TEST_F(VTEPositionTest, TargetVelocityFusionMoving)
{
	enableMask({FusionMaskOption::kVisionPos, FusionMaskOption::kTargetGpsVel});

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	const hrt_abstime vel_time = vte_test::advanceMicroseconds(kStepUs);
	setLocalVelocity(vel_ned, vel_time);

	const hrt_abstime vision_time = vte_test::advanceMicroseconds(kStepUs);
	publishVisionPos(matrix::Vector3f(0.f, 0.f, 0.f), vte_test::identityQuat(),
			 kSmallVisionCov, vision_time);
	_vte->update(matrix::Vector3f{});

	const matrix::Vector3f target_vel(1.f, 2.f, 3.f);
	const hrt_abstime target_time = vte_test::advanceMicroseconds(kStepUs);
	publishTargetGnss(kUavLat, kUavLon, kTargetAltM, 0.5f, 0.5f, target_time,
			  true, true, target_vel, 0.1f);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_vel_target_sub->update());
	const auto aid = _aid_gps_vel_target_sub->get();
	EXPECT_EQ(aid.fusion_status[0], static_cast<uint8_t>(vte::FusionStatus::STATUS_FUSED_CURRENT));
	EXPECT_NEAR(aid.observation_variance[0], kDefaultGpsVelVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[1], kDefaultGpsVelVar, kTolerance);
	EXPECT_NEAR(aid.observation_variance[2], kDefaultGpsVelVar, kTolerance);
}
#endif

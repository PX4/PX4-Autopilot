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
 * @file TEST_VTE_VTEPosition.cpp
 * @brief Unit test VTEPosition.cpp
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
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/target_gnss.h>
#include <uORB/topics/vte_aid_source3d.h>
#include <uORB/topics/vision_target_est_position.h>
#include <matrix/Quaternion.hpp>

#include "Position/VTEPosition.h"

namespace vte = vision_target_estimator;

namespace
{
static constexpr float kTolerance = 1e-3f;
static constexpr hrt_abstime kTimestampLagUs = 10'000;

matrix::Quaternionf identityQuat()
{
	matrix::Quaternionf q;
	q(0) = 1.f;
	q(1) = 0.f;
	q(2) = 0.f;
	q(3) = 0.f;
	return q;
}

class VTEPositionTestable : public vte::VTEPosition
{
public:
	void updateParamsPublic() { updateParams(); }
};

} // namespace

class VTEPositionTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		uORB::Manager::initialize();
		param_control_autosave(false);

		setParamFloat("VTE_POS_UNC_IN", 1.f);
		setParamFloat("VTE_VEL_UNC_IN", 1.f);
		setParamFloat("VTE_BIA_UNC_IN", 0.5f);
		setParamFloat("VTE_EVP_NOISE", 0.2f);
		setParamFloat("VTE_GPS_P_NOISE", 0.1f);
		setParamFloat("VTE_GPS_V_NOISE", 0.2f);
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
		_vte_state_sub = std::make_unique<uORB::SubscriptionData<vision_target_est_position_s>>(ORB_ID(vision_target_est_position));
		_target_pose_sub = std::make_unique<uORB::SubscriptionData<landing_target_pose_s>>(ORB_ID(landing_target_pose));

		resetTimestamp();
	}

	void TearDown() override
	{
		_target_pose_sub.reset();
		_vte_state_sub.reset();
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

	void enableMask(bool vision, bool mission, bool target_pos, bool uav_vel)
	{
		vte::SensorFusionMaskU mask{};
		mask.flags.use_vision_pos = vision;
		mask.flags.use_mission_pos = mission;
		mask.flags.use_target_gps_pos = target_pos;
		mask.flags.use_uav_gps_vel = uav_vel;
		_vte->set_vte_aid_mask(mask.value);
	}

	void publishVisionPos(const matrix::Vector3f &rel_pos, const matrix::Quaternionf &q,
			      const matrix::Vector3f &cov, hrt_abstime timestamp)
	{
		fiducial_marker_pos_report_s msg{};
		msg.timestamp = timestamp;
		msg.timestamp_sample = timestamp;
		msg.rel_pos[0] = rel_pos(0);
		msg.rel_pos[1] = rel_pos(1);
		msg.rel_pos[2] = rel_pos(2);
		msg.cov_rel_pos[0] = cov(0);
		msg.cov_rel_pos[1] = cov(1);
		msg.cov_rel_pos[2] = cov(2);
		q.copyTo(msg.q);
		ASSERT_TRUE(_vision_pub->publish(msg));
	}

	void publishUavGps(double lat, double lon, float alt, float eph, float epv,
			   const matrix::Vector3f &vel_ned, float vel_var, bool vel_valid, hrt_abstime timestamp)
	{
		sensor_gps_s msg{};
		msg.timestamp = timestamp;
		msg.timestamp_sample = timestamp;
		msg.latitude_deg = lat;
		msg.longitude_deg = lon;
		msg.altitude_msl_m = alt;
		msg.eph = eph;
		msg.epv = epv;
		msg.vel_n_m_s = vel_ned(0);
		msg.vel_e_m_s = vel_ned(1);
		msg.vel_d_m_s = vel_ned(2);
		msg.s_variance_m_s = vel_var;
		msg.vel_ned_valid = vel_valid;
		ASSERT_TRUE(_uav_gps_pub->publish(msg));
	}

	void publishTargetGnss(double lat, double lon, float alt, float eph, float epv,
			       hrt_abstime timestamp, bool abs_pos_updated)
	{
		target_gnss_s msg{};
		msg.timestamp = timestamp;
		msg.latitude_deg = lat;
		msg.longitude_deg = lon;
		msg.altitude_msl_m = alt;
		msg.eph = eph;
		msg.epv = epv;
		msg.abs_pos_updated = abs_pos_updated;
		ASSERT_TRUE(_target_gps_pub->publish(msg));
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
	std::unique_ptr<uORB::SubscriptionData<vision_target_est_position_s>> _vte_state_sub;
	std::unique_ptr<uORB::SubscriptionData<landing_target_pose_s>> _target_pose_sub;

	hrt_abstime _timestamp{0};
};

TEST_F(VTEPositionTest, InitRequiresVelocityEstimate)
{
	// WHY: Estimator must not initialize without a velocity estimate.
	// WHAT: Provide a position measurement only and expect no output.
	enableMask(true, false, false, false);

	const matrix::Vector3f rel_pos(1.f, 2.f, -3.f);
	publishVisionPos(rel_pos, identityQuat(), matrix::Vector3f(0.0f, 0.0f, 0.0f), stepTime(1'000));

	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_vte_state_sub->update());
	EXPECT_FALSE(_aid_fiducial_sub->update());
}

TEST_F(VTEPositionTest, InitWithVisionAndLocalVelocity)
{
	// WHY: Vision + velocity should be sufficient to start the filter.
	// WHAT: Verify initial state matches the vision position and local velocity.
	enableMask(true, false, false, false);

	const matrix::Vector3f rel_pos(4.f, -2.f, 1.f);
	const matrix::Vector3f vel_ned(1.f, 2.f, 3.f);
	setLocalVelocity(vel_ned, stepTime(1'000));

	publishVisionPos(rel_pos, identityQuat(), matrix::Vector3f(0.0f, 0.0f, 0.0f), stepTime(1'000));
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
	EXPECT_GE(aid.observation_variance[0], 0.04f);
	EXPECT_GE(aid.observation_variance[1], 0.04f);
	EXPECT_GE(aid.observation_variance[2], 0.04f);
}

TEST_F(VTEPositionTest, InitialPositionUsesVisionFirst)
{
	// WHY: Vision should take priority over GNSS for initialization.
	// WHAT: Provide vision and GPS with different values and check vision innovation near zero.
	enableMask(true, false, true, false);

	const matrix::Vector3f vel_ned(0.1f, 0.2f, 0.3f);
	setLocalVelocity(vel_ned, stepTime(1'000));

	const hrt_abstime base_time = hrt_absolute_time() - 10'000;
	const hrt_abstime vision_time = base_time;
	const hrt_abstime gps_time = base_time + 4'000;

	const double lat = 47.3977419;
	const double lon = 8.5455938;
	publishUavGps(lat, lon, 100.f, 0.5f, 0.5f, vel_ned, 0.1f, true, gps_time);
	publishTargetGnss(lat, lon, 90.f, 0.5f, 0.5f, gps_time, true);

	const matrix::Vector3f rel_pos(5.f, 6.f, 7.f);
	publishVisionPos(rel_pos, identityQuat(), matrix::Vector3f(0.01f, 0.01f, 0.01f), vision_time);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();
	EXPECT_NEAR(aid.innovation[0], 0.f, 0.5f);
	EXPECT_NEAR(aid.innovation[1], 0.f, 0.5f);
	EXPECT_NEAR(aid.innovation[2], 0.f, 0.5f);
}

TEST_F(VTEPositionTest, BiasSetWhenNonGpsArrives)
{
	// WHY: Bias is observable only after a non-GNSS position measurement arrives.
	// WHAT: Initialize with GNSS, then fuse vision and check bias update.
	enableMask(true, true, false, false);

	const matrix::Vector3f vel_ned(0.5f, -0.5f, 0.1f);
	setLocalVelocity(vel_ned, stepTime(1'000));

	const double lat = 47.3977419;
	const double lon = 8.5455938;
	publishUavGps(lat, lon, 100.f, 0.2f, 0.2f, vel_ned, 0.1f, true, stepTime(1'000));
	_vte->set_mission_position(lat, lon, 90.f);

	_vte->update(matrix::Vector3f{});

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	publishUavGps(lat, lon, 100.f, 0.2f, 0.2f, vel_ned, 0.1f, true, stepTime(1'000));
	publishVisionPos(rel_pos, identityQuat(), matrix::Vector3f(0.01f, 0.01f, 0.01f), stepTime(1'000));

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
	enableMask(true, false, false, false);

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	setLocalVelocity(vel_ned, stepTime(1'000));

	matrix::Quaternionf q;
	q(0) = 0.f;
	q(1) = 0.f;
	q(2) = 0.f;
	q(3) = 1.f;

	const matrix::Vector3f rel_pos(1.f, 2.f, 3.f);
	publishVisionPos(rel_pos, q, matrix::Vector3f(0.0f, 0.0f, 0.0f), stepTime(1'000));
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_fiducial_sub->update());
	const auto aid = _aid_fiducial_sub->get();

	EXPECT_NEAR(aid.observation[0], -1.f, kTolerance);
	EXPECT_NEAR(aid.observation[1], -2.f, kTolerance);
	EXPECT_NEAR(aid.observation[2], 3.f, kTolerance);
	EXPECT_GE(aid.observation_variance[0], 0.04f);
	EXPECT_GE(aid.observation_variance[1], 0.04f);
	EXPECT_GE(aid.observation_variance[2], 0.04f);
}

TEST_F(VTEPositionTest, MissionGpsRelativePositionAndOffset)
{
	// WHY: Mission GNSS measurements must apply offset and variance floors.
	// WHAT: Publish GPS and mission position with offset and verify observation output.
	enableMask(false, true, false, false);

	const matrix::Vector3f vel_ned(0.1f, 0.1f, 0.1f);
	setLocalVelocity(vel_ned, stepTime(1'000));

	const double lat = 47.3977419;
	const double lon = 8.5455938;
	publishUavGps(lat, lon, 100.f, 0.01f, 0.01f, vel_ned, 0.1f, true, stepTime(1'000));
	_vte->set_mission_position(lat, lon, 90.f);

	_vte->set_gps_pos_offset(matrix::Vector3f(1.f, 2.f, 3.f), true);
	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_mission_sub->update());
	const auto aid = _aid_gps_mission_sub->get();
	EXPECT_NEAR(aid.observation[0], 1.f, kTolerance);
	EXPECT_NEAR(aid.observation[1], 2.f, kTolerance);
	EXPECT_NEAR(aid.observation[2], 13.f, kTolerance);
	EXPECT_GE(aid.observation_variance[0], 0.01f);
	EXPECT_GE(aid.observation_variance[1], 0.01f);
	EXPECT_GE(aid.observation_variance[2], 0.01f);
}

TEST_F(VTEPositionTest, UavGpsVelocityFusionAndSign)
{
	// WHY: UAV GPS velocity fusion should update state and sign-converted outputs.
	// WHAT: Fuse GPS velocity and verify landing_target_pose uses negative sign.
	enableMask(true, false, false, true);

	const matrix::Vector3f vel_ned(1.f, -2.f, 0.5f);
	setLocalVelocity(vel_ned, stepTime(1'000));
	setLocalPosition(matrix::Vector3f{}, stepTime(1'000));

	publishUavGps(47.3977419, 8.5455938, 100.f, 0.2f, 0.2f, vel_ned, 0.1f, true, stepTime(1'000));
	publishVisionPos(matrix::Vector3f(2.f, 0.f, -1.f), identityQuat(),
			 matrix::Vector3f(0.01f, 0.01f, 0.01f), stepTime(1'000));

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_vel_uav_sub->update());
	const auto aid = _aid_gps_vel_uav_sub->get();
	EXPECT_GE(aid.observation_variance[0], 0.04f);
	EXPECT_GE(aid.observation_variance[1], 0.04f);
	EXPECT_GE(aid.observation_variance[2], 0.04f);

	ASSERT_TRUE(_vte_state_sub->update());
	const auto state = _vte_state_sub->get();
	ASSERT_TRUE(_target_pose_sub->update());
	const auto pose = _target_pose_sub->get();

	EXPECT_NEAR(pose.vx_rel, -state.vel_uav[0], 0.5f);
	EXPECT_NEAR(pose.vy_rel, -state.vel_uav[1], 0.5f);
	EXPECT_NEAR(pose.vz_rel, -state.vel_uav[2], 0.5f);
}

TEST_F(VTEPositionTest, TargetGpsInterpolationVariance)
{
	// WHY: GNSS time misalignment should increase measurement variance.
	// WHAT: Publish delayed UAV GPS and verify propagation uncertainty is included.
	enableMask(false, false, true, false);

	setParamFloat("VTE_GPS_P_NOISE", 0.02f);
	_vte->updateParamsPublic();

	const matrix::Vector3f vel_ned(1.f, 0.f, 0.f);
	setLocalVelocity(vel_ned, stepTime(1'000));

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime uav_time = now - 300'000;
	const hrt_abstime target_time = now - 100'000;
	publishUavGps(47.3977419, 8.5455938, 100.f, 0.01f, 0.01f, vel_ned, 1.0f, true, uav_time);
	publishTargetGnss(47.3977419, 8.5455938, 90.f, 0.01f, 0.01f, target_time, true);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_target_sub->update());
	const auto aid = _aid_gps_target_sub->get();
	const float base_var = 2.f * 0.0004f;

	EXPECT_GT(aid.observation_variance[0], base_var);
	EXPECT_GT(aid.observation_variance[1], base_var);
	EXPECT_GT(aid.observation_variance[2], base_var);
}

TEST_F(VTEPositionTest, StaleMeasurementsIgnored)
{
	// WHY: Old measurements must be dropped to avoid stale fusion.
	// WHAT: Set a tiny timeout and ensure stale vision data is ignored.
	enableMask(true, false, false, false);

	_vte->set_meas_recent_timeout(1'000);

	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), stepTime(1'000));

	const hrt_abstime stale_time = hrt_absolute_time() - 2'000;
	publishVisionPos(matrix::Vector3f(1.f, 1.f, 1.f), identityQuat(),
			 matrix::Vector3f(0.01f, 0.01f, 0.01f), stale_time);

	_vte->update(matrix::Vector3f{});

	EXPECT_FALSE(_aid_fiducial_sub->update());
	EXPECT_FALSE(_vte_state_sub->update());
}

#if defined(CONFIG_VTEST_MOVING)
TEST_F(VTEPositionTest, TargetVelocityFusionMoving)
{
	// WHY: Moving-target mode must fuse target velocity measurements.
	// WHAT: Provide target GNSS velocity and verify fusion output.
	enableMask(false, false, false, false);

	vte::SensorFusionMaskU mask{};
	mask.flags.use_target_gps_vel = 1;
	_vte->set_vte_aid_mask(mask.value);

	setLocalVelocity(matrix::Vector3f(0.1f, 0.1f, 0.1f), stepTime(1'000));

	target_gnss_s msg{};
	msg.timestamp = stepTime(1'000);
	msg.vel_ned_updated = true;
	msg.vel_n_m_s = 1.f;
	msg.vel_e_m_s = 2.f;
	msg.vel_d_m_s = 3.f;
	msg.s_acc_m_s = 0.1f;
	_target_gps_pub->publish(msg);

	_vte->update(matrix::Vector3f{});

	ASSERT_TRUE(_aid_gps_target_sub->update());
	const auto aid = _aid_gps_target_sub->get();
	EXPECT_EQ(aid.fusion_status[0], static_cast<uint8_t>(vte::FusionStatus::FUSED_CURRENT));
}
#endif

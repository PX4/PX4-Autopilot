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
 * @file TEST_VTE_VisionTargetEst.cpp
 * @brief Unit test VisionTargetEst.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <memory>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/uORBManager.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vision_target_est_input.h>
#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
#include <uORB/topics/prec_land_status.h>
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
#include <matrix/Quaternion.hpp>
#include <matrix/Vector.hpp>

#include "VisionTargetEst.h"
#include "VTETestHelper.hpp"

namespace vte = vision_target_estimator;

namespace
{
using namespace time_literals;

static constexpr float kTolerance = 1e-4f;
static constexpr float kGravity = 9.80665f;
static constexpr hrt_abstime kStepUs = 1_ms;

} // namespace

class VisionTargetEstTestable : public vte::VisionTargetEst
{
public:
	using vte::VisionTargetEst::kAccUpdatedTimeoutUs;
	using vte::VisionTargetEst::kEstRestartTimeUs;
	using vte::VisionTargetEst::kMinAccDownsampleTimeoutUs;
	using vte::VisionTargetEst::kPosUpdatePeriodUs;
	using vte::VisionTargetEst::kYawUpdatePeriodUs;
	using vte::VisionTargetEst::LocalPose;
	using vte::VisionTargetEst::adjustAidMask;
	using vte::VisionTargetEst::allEstStoppedDueToTimeout;
	using vte::VisionTargetEst::computeGpsVelocityOffset;
	using vte::VisionTargetEst::findLandSetpoint;
	using vte::VisionTargetEst::isCurrentTaskComplete;
	using vte::VisionTargetEst::isNewTaskAvailable;
	using vte::VisionTargetEst::pollEstimatorInput;
	using vte::VisionTargetEst::publishVteInput;
	using vte::VisionTargetEst::startPosEst;
	using vte::VisionTargetEst::startYawEst;
	using vte::VisionTargetEst::stopPosEst;
	using vte::VisionTargetEst::stopYawEst;
	using vte::VisionTargetEst::updateEstimators;
	using vte::VisionTargetEst::updateParams;
	using vte::VisionTargetEst::updatePosEst;
	using vte::VisionTargetEst::updateTaskTopics;
	using vte::VisionTargetEst::updateWhenIntervalElapsed;
	using vte::VisionTargetEst::_acc_sample_count;
	using vte::VisionTargetEst::_acc_sample_warn_last;
	using vte::VisionTargetEst::_current_task;
	using vte::VisionTargetEst::_gps_pos_is_offset;
	using vte::VisionTargetEst::_gps_pos_offset_xyz;
	using vte::VisionTargetEst::_is_in_prec_land;
	using vte::VisionTargetEst::_last_acc_reset;
	using vte::VisionTargetEst::_last_update_pos;
	using vte::VisionTargetEst::_orientation_estimator_running;
	using vte::VisionTargetEst::_pos_sp_triplet_buffer;
	using vte::VisionTargetEst::_position_estimator_running;
	using vte::VisionTargetEst::_vehicle_acc_body;
	using vte::VisionTargetEst::_vehicle_acc_ned_sum;
	using vte::VisionTargetEst::_vehicle_acceleration_sub;
	using vte::VisionTargetEst::_vehicle_angular_velocity_sub;
	using vte::VisionTargetEst::_vehicle_attitude_sub;
	using vte::VisionTargetEst::_vehicle_land_detected_sub;
	using vte::VisionTargetEst::_vehicle_local_position_sub;
	using vte::VisionTargetEst::_vte_orientation_enabled;
	using vte::VisionTargetEst::_vte_orientation_stop_time;
	using vte::VisionTargetEst::_vte_position;
	using vte::VisionTargetEst::_vte_position_enabled;
	using vte::VisionTargetEst::_vte_position_stop_time;
	using vte::VisionTargetEst::_vte_task_mask;
	using vte::VisionTargetEst::_pos_sp_triplet_sub;
#if !defined(CONSTRAINED_FLASH)
	using vte::VisionTargetEst::_prec_land_status_sub;
#endif
};

class VisionTargetEstTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();
	}

	void SetUp() override
	{
		param_control_autosave(false);

		setParamInt("VTE_POS_EN", 1);
		setParamInt("VTE_YAW_EN", 1);
		setParamInt("VTE_TASK_MASK", 0);
		setParamInt("VTE_AID_MASK", 0);
		setParamFloat("VTE_BTOUT", 3.f);
		setParamFloat("VTE_TGT_TOUT", 2.f);
		setParamFloat("VTE_M_REC_TOUT", 1.f);
		setParamFloat("VTE_M_UPD_TOUT", 0.1f);

		_vte = std::make_unique<VisionTargetEstTestable>();

		_attitude_pub = std::make_unique<uORB::Publication<vehicle_attitude_s>>(ORB_ID(vehicle_attitude));
		_accel_pub = std::make_unique<uORB::Publication<vehicle_acceleration_s>>(ORB_ID(vehicle_acceleration));
		_ang_vel_pub = std::make_unique<uORB::Publication<vehicle_angular_velocity_s>>(ORB_ID(vehicle_angular_velocity));
		_pos_sp_triplet_pub = std::make_unique<uORB::Publication<position_setpoint_triplet_s>>(ORB_ID(position_setpoint_triplet));
		_land_detected_pub = std::make_unique<uORB::Publication<vehicle_land_detected_s>>(ORB_ID(vehicle_land_detected));
#if !defined(CONSTRAINED_FLASH)
		_prec_land_status_pub = std::make_unique<uORB::Publication<prec_land_status_s>>(ORB_ID(prec_land_status));
#endif

		_vte_input_sub = std::make_unique<uORB::SubscriptionData<vision_target_est_input_s>>(ORB_ID(vision_target_est_input));

		flushInternalSubscriptions();
		vte_test::flushSubscription(_vte_input_sub);
	}

	void TearDown() override
	{
		_param_guard.restore();
		param_control_autosave(true);

		_vte_input_sub.reset();
#if !defined(CONSTRAINED_FLASH)
		_prec_land_status_pub.reset();
#endif
		_land_detected_pub.reset();
		_pos_sp_triplet_pub.reset();
		_ang_vel_pub.reset();
		_accel_pub.reset();
		_attitude_pub.reset();

		_vte.reset();
	}

	void setParamFloat(const char *name, float value)
	{
		ASSERT_TRUE(_param_guard.setFloat(name, value));
	}

	void setParamInt(const char *name, int32_t value)
	{
		ASSERT_TRUE(_param_guard.setInt(name, value));
	}

	void publishAttitude(const matrix::Quaternionf &q, hrt_abstime timestamp)
	{
		vehicle_attitude_s msg{};
		msg.timestamp = timestamp;
		q.copyTo(msg.q);
		ASSERT_TRUE(_attitude_pub->publish(msg));
	}

	void publishAcceleration(const matrix::Vector3f &accel, hrt_abstime timestamp)
	{
		vehicle_acceleration_s msg{};
		msg.timestamp = timestamp;
		msg.xyz[0] = accel(0);
		msg.xyz[1] = accel(1);
		msg.xyz[2] = accel(2);
		ASSERT_TRUE(_accel_pub->publish(msg));
	}

	void publishAngularVelocity(const matrix::Vector3f &ang_vel, hrt_abstime timestamp)
	{
		vehicle_angular_velocity_s msg{};
		msg.timestamp = timestamp;
		msg.xyz[0] = ang_vel(0);
		msg.xyz[1] = ang_vel(1);
		msg.xyz[2] = ang_vel(2);
		ASSERT_TRUE(_ang_vel_pub->publish(msg));
	}

	void publishLandDetected(bool landed, hrt_abstime timestamp)
	{
		vehicle_land_detected_s msg{};
		msg.timestamp = timestamp;
		msg.landed = landed;
		ASSERT_TRUE(_land_detected_pub->publish(msg));
	}

	void publishPositionSetpointTriplet(const position_setpoint_triplet_s &triplet)
	{
		ASSERT_TRUE(_pos_sp_triplet_pub->publish(triplet));
	}

#if !defined(CONSTRAINED_FLASH)
	void publishPrecLandStatus(uint8_t state, hrt_abstime timestamp)
	{
		prec_land_status_s msg{};
		msg.timestamp = timestamp;
		msg.state = state;
		ASSERT_TRUE(_prec_land_status_pub->publish(msg));
	}
#endif

	void flushInternalSubscriptions()
	{
		vte_test::flushSubscription<vehicle_attitude_s>(_vte->_vehicle_attitude_sub);
		vte_test::flushSubscription<vehicle_acceleration_s>(_vte->_vehicle_acceleration_sub);
		vte_test::flushSubscription<vehicle_angular_velocity_s>(_vte->_vehicle_angular_velocity_sub);
		vte_test::flushSubscription<vehicle_local_position_s>(_vte->_vehicle_local_position_sub);
		vte_test::flushSubscription<position_setpoint_triplet_s>(_vte->_pos_sp_triplet_sub);
		vte_test::flushSubscription<vehicle_land_detected_s>(_vte->_vehicle_land_detected_sub);
#if !defined(CONSTRAINED_FLASH)
		vte_test::flushSubscription<prec_land_status_s>(_vte->_prec_land_status_sub);
#endif
	}

	std::unique_ptr<VisionTargetEstTestable> _vte;
	std::unique_ptr<uORB::Publication<vehicle_attitude_s>> _attitude_pub;
	std::unique_ptr<uORB::Publication<vehicle_acceleration_s>> _accel_pub;
	std::unique_ptr<uORB::Publication<vehicle_angular_velocity_s>> _ang_vel_pub;
	std::unique_ptr<uORB::Publication<position_setpoint_triplet_s>> _pos_sp_triplet_pub;
	std::unique_ptr<uORB::Publication<vehicle_land_detected_s>> _land_detected_pub;
#if !defined(CONSTRAINED_FLASH)
	std::unique_ptr<uORB::Publication<prec_land_status_s>> _prec_land_status_pub;
#endif

	std::unique_ptr<uORB::SubscriptionData<vision_target_est_input_s>> _vte_input_sub;

	vte_test::ParamGuard _param_guard{};
};

TEST_F(VisionTargetEstTest, AdjustAidMaskResolvesConflicts)
{
	// WHY: If there is a GNSS on the target, do not use the mission position
	// WHAT: Enable mission position and GNSS on target, expect GNSS on target only
	vte::SensorFusionMaskU mask{};
	mask.flags.use_target_gps_pos = 1;
	mask.flags.use_mission_pos = 1;

	const uint16_t adjusted = _vte->adjustAidMask(mask.value);
	vte::SensorFusionMaskU adjusted_mask{};
	adjusted_mask.value = adjusted;

	EXPECT_TRUE(adjusted_mask.flags.use_target_gps_pos);
	EXPECT_FALSE(adjusted_mask.flags.use_mission_pos);
}

TEST_F(VisionTargetEstTest, UpdateParamsSetsGpsOffsetThreshold)
{
	// WHY: All measurements are fused at the center of mass. The GNSS measurement must be offset using the antenna's location
	// WHAT: Apply a non-zero offset, verify enabled, then reset to zero and verify disabled.

	const matrix::Vector3f gps_offset_gt {0.2, -0.015, 3.1};
	setParamFloat("EKF2_GPS_POS_X", gps_offset_gt(0));
	setParamFloat("EKF2_GPS_POS_Y", gps_offset_gt(1));
	setParamFloat("EKF2_GPS_POS_Z", gps_offset_gt(2));

	_vte->updateParams();

	EXPECT_TRUE(_vte->_gps_pos_is_offset);
	const matrix::Vector3f gps_offset = _vte->_gps_pos_offset_xyz;
	EXPECT_NEAR(gps_offset(0), gps_offset_gt(0), kTolerance);
	EXPECT_NEAR(gps_offset(1), gps_offset_gt(1), kTolerance);
	EXPECT_NEAR(gps_offset(2), gps_offset_gt(2), kTolerance);

	setParamFloat("EKF2_GPS_POS_X", 0.f);
	setParamFloat("EKF2_GPS_POS_Y", 0.f);
	setParamFloat("EKF2_GPS_POS_Z", 0.f);

	_vte->updateParams();

	EXPECT_FALSE(_vte->_gps_pos_is_offset);
	const matrix::Vector3f gps_offset_reset = _vte->_gps_pos_offset_xyz;
	EXPECT_NEAR(gps_offset_reset(0), 0.f, kTolerance);
	EXPECT_NEAR(gps_offset_reset(1), 0.f, kTolerance);
	EXPECT_NEAR(gps_offset_reset(2), 0.f, kTolerance);
}

TEST_F(VisionTargetEstTest, ComputeGpsVelocityOffsetRequiresOffsetAndData)
{
	// WHY: Compensate for the rotational velocity if the GNSS antenna is not at the center of mass,
	// i.e. when the drone rotates around the center of mass, the GNSS will record a velocity.
	// WHAT: Expect false without config/data, then true once angular velocity is published.
	matrix::Vector3f vel_offset{};
	EXPECT_FALSE(_vte->computeGpsVelocityOffset(vel_offset));

	const matrix::Vector3f gps_offset(1.f, 0.f, 0.f);
	_vte->_gps_pos_is_offset = true;
	_vte->_gps_pos_offset_xyz = gps_offset;

	EXPECT_FALSE(_vte->computeGpsVelocityOffset(vel_offset));

	const matrix::Vector3f ang_vel(0.f, 0.f, 1.f);
	publishAngularVelocity(ang_vel, vte_test::advanceMicroseconds(kStepUs));
	EXPECT_TRUE(_vte->computeGpsVelocityOffset(vel_offset));
	const matrix::Vector3f expected_offset = ang_vel.cross(gps_offset);
	EXPECT_NEAR(vel_offset(0), expected_offset(0), kTolerance);
	EXPECT_NEAR(vel_offset(1), expected_offset(1), kTolerance);
	EXPECT_NEAR(vel_offset(2), expected_offset(2), kTolerance);
}

TEST_F(VisionTargetEstTest, PollEstimatorInputRequiresAttitude)
{
	// WHY: Attitude is required to transform body-frame data into NED.
	// WHAT: Call pollEstimatorInput without attitude and expect it to fail.
	matrix::Vector3f acc_ned{};
	matrix::Quaternionf q_att{};
	matrix::Vector3f gps_offset{};
	matrix::Vector3f vel_offset{};
	const bool vel_offset_updated = true;
	bool acc_valid = true;

	EXPECT_FALSE(_vte->pollEstimatorInput(acc_ned, q_att, gps_offset, vel_offset, vel_offset_updated, acc_valid));
}

TEST_F(VisionTargetEstTest, PollEstimatorInputTransformsAccelerationAndOffsets)
{
	// WHY: Valid inputs should be rotated into NED and include gravity compensation.
	// WHAT: Publish attitude/accel with offsets and verify transformed outputs.
	const hrt_abstime timestamp = vte_test::nowUs();
	matrix::Quaternionf q{0.f, 0.f, 0.f, 1.f}; // 180 degrees rotation around z-axis
	const matrix::Vector3f non_rotated_vec{1.f, 2.f, 3.f};
	const matrix::Vector3f rotated_vec{-1.f, -2.f, 3.f};

	publishAttitude(q, timestamp);
	publishAcceleration(non_rotated_vec, timestamp);

	_vte->_gps_pos_is_offset = true;
	_vte->_gps_pos_offset_xyz = non_rotated_vec;

	matrix::Vector3f acc_ned{};
	matrix::Quaternionf q_att{};
	matrix::Vector3f gps_offset = non_rotated_vec;
	matrix::Vector3f vel_offset = non_rotated_vec;
	const bool vel_offset_updated = true;
	bool acc_valid = false;

	ASSERT_TRUE(_vte->pollEstimatorInput(acc_ned, q_att, gps_offset, vel_offset, vel_offset_updated, acc_valid));
	EXPECT_TRUE(acc_valid);
	EXPECT_NEAR(acc_ned(0), rotated_vec(0), kTolerance);
	EXPECT_NEAR(acc_ned(1), rotated_vec(1), kTolerance);
	EXPECT_NEAR(acc_ned(2), rotated_vec(2) + kGravity, kTolerance);

	EXPECT_NEAR(gps_offset(0), rotated_vec(0), kTolerance);
	EXPECT_NEAR(gps_offset(1), rotated_vec(1), kTolerance);
	EXPECT_NEAR(gps_offset(2), rotated_vec(2), kTolerance);

	EXPECT_NEAR(vel_offset(0), rotated_vec(0), kTolerance);
	EXPECT_NEAR(vel_offset(1), rotated_vec(1), kTolerance);
	EXPECT_NEAR(vel_offset(2), rotated_vec(2), kTolerance);
}

TEST_F(VisionTargetEstTest, PollEstimatorInputNoOffsetZerosVectors)
{
	// WHY: With GPS offset disabled, offset vectors must be zeroed.
	// WHAT: Disable offset and verify returned GNSSS/velocity offsets are zero.
	const hrt_abstime timestamp = vte_test::nowUs();
	publishAttitude(vte_test::identityQuat(), timestamp);
	publishAcceleration(matrix::Vector3f(0.1f, 0.2f, 0.3f), timestamp);

	_vte->_gps_pos_is_offset = false;
	_vte->_gps_pos_offset_xyz = matrix::Vector3f(1.f, 2.f, 3.f);

	matrix::Vector3f acc_ned{};
	matrix::Quaternionf q_att{};
	matrix::Vector3f gps_offset{};
	matrix::Vector3f vel_offset(4.f, 5.f, 6.f);
	const bool vel_offset_updated = true;
	bool acc_valid = false;

	ASSERT_TRUE(_vte->pollEstimatorInput(acc_ned, q_att, gps_offset, vel_offset, vel_offset_updated, acc_valid));
	EXPECT_TRUE(acc_valid);
	EXPECT_NEAR(gps_offset(0), 0.f, kTolerance);
	EXPECT_NEAR(gps_offset(1), 0.f, kTolerance);
	EXPECT_NEAR(gps_offset(2), 0.f, kTolerance);
	EXPECT_NEAR(vel_offset(0), 0.f, kTolerance);
	EXPECT_NEAR(vel_offset(1), 0.f, kTolerance);
	EXPECT_NEAR(vel_offset(2), 0.f, kTolerance);
}

TEST_F(VisionTargetEstTest, PublishVteInputPopulatesMessage)
{
	// WHY: Published inputs must reflect the sampled accel and attitude data.
	// WHAT: Populate sample timestamp and verify the published message fields.
	const hrt_abstime sample_time = vte_test::nowUs();
	_vte->_vehicle_acc_body.timestamp = sample_time;

	const matrix::Vector3f acc_ned(0.5f, -0.25f, 9.0f);
	const matrix::Quaternionf q_att = vte_test::identityQuat();

	_vte->publishVteInput(acc_ned, q_att);

	ASSERT_TRUE(_vte_input_sub->update());
	const auto msg = _vte_input_sub->get();

	EXPECT_EQ(msg.timestamp_sample, sample_time);
	EXPECT_NEAR(msg.acc_xyz[0], acc_ned(0), kTolerance);
	EXPECT_NEAR(msg.acc_xyz[1], acc_ned(1), kTolerance);
	EXPECT_NEAR(msg.acc_xyz[2], acc_ned(2), kTolerance);
	EXPECT_NEAR(msg.q_att[0], q_att(0), kTolerance);
	EXPECT_NEAR(msg.q_att[1], q_att(1), kTolerance);
	EXPECT_NEAR(msg.q_att[2], q_att(2), kTolerance);
	EXPECT_NEAR(msg.q_att[3], q_att(3), kTolerance);
}

TEST_F(VisionTargetEstTest, UpdateWhenIntervalElapsedRespectsInterval)
{
	// WHY: The update gate should only allow execution after the interval elapses.
	// WHAT: Expect false before the interval and true after waiting.
	const hrt_abstime interval = 10_ms;
	hrt_abstime last_time = vte_test::nowUs();
	EXPECT_FALSE(_vte->updateWhenIntervalElapsed(last_time, interval));

	vte_test::advanceMicroseconds(interval + 1_ms);
	EXPECT_TRUE(_vte->updateWhenIntervalElapsed(last_time, interval));
}

TEST_F(VisionTargetEstTest, FindLandSetpointSelectsCorrectSetpoint)
{
	// WHY: Precision landing should use the active land setpoint.
	// WHAT: Publish triplets with land as current/next and verify selection.
	EXPECT_EQ(_vte->findLandSetpoint(), nullptr);

	position_setpoint_triplet_s triplet{};
	triplet.timestamp = vte_test::advanceMicroseconds(kStepUs);
	triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	triplet.next.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	publishPositionSetpointTriplet(triplet);

	const position_setpoint_s *current_sp = _vte->findLandSetpoint();
	ASSERT_NE(current_sp, nullptr);
	EXPECT_EQ(current_sp, &_vte->_pos_sp_triplet_buffer.current);

	triplet = {};
	triplet.timestamp = vte_test::advanceMicroseconds(kStepUs);
	triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	triplet.next.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	publishPositionSetpointTriplet(triplet);

	const position_setpoint_s *next_sp = _vte->findLandSetpoint();
	ASSERT_NE(next_sp, nullptr);
	EXPECT_EQ(next_sp, &_vte->_pos_sp_triplet_buffer.next);
}

TEST_F(VisionTargetEstTest, PrecisionLandTaskRequestAndCompletion)
{
	// WHY: Precision landing tasks should start once requested and complete on land detect.
	// WHAT: Request the task, then publish land detected and verify completion.
	_vte->_vte_task_mask.value = 0;
	_vte->_vte_task_mask.flags.for_prec_land = true;
	_vte->_current_task.value = 0;
	_vte->_is_in_prec_land = true;

	EXPECT_TRUE(_vte->isNewTaskAvailable());
	EXPECT_TRUE(_vte->_current_task.flags.for_prec_land);

	publishLandDetected(true, vte_test::advanceMicroseconds(kStepUs));
	EXPECT_TRUE(_vte->isCurrentTaskComplete());
	EXPECT_FALSE(_vte->_is_in_prec_land);
}

#if !defined(CONSTRAINED_FLASH)
TEST_F(VisionTargetEstTest, UpdateTaskTopicsTracksPrecisionLandState)
{
	// WHY: Precision landing status topics should drive internal task state.
	// WHAT: Publish ongoing and stopped states and verify the internal flag toggles.
	_vte->_vte_task_mask.value = 0;
	_vte->_vte_task_mask.flags.for_prec_land = true;
	_vte->_is_in_prec_land = false;

	publishPrecLandStatus(prec_land_status_s::PREC_LAND_STATE_ONGOING, vte_test::advanceMicroseconds(kStepUs));
	_vte->updateTaskTopics();
	EXPECT_TRUE(_vte->_is_in_prec_land);

	publishPrecLandStatus(prec_land_status_s::PREC_LAND_STATE_STOPPED, vte_test::advanceMicroseconds(kStepUs));
	_vte->updateTaskTopics();
	EXPECT_FALSE(_vte->_is_in_prec_land);
}
#endif

TEST_F(VisionTargetEstTest, UpdatePosEstResetsDownsampleOnTimeout)
{
	// WHY: The acceleration downsample must reset after timeout to avoid stale averages.
	// WHAT: Force a timeout and verify the accumulator resets on update.
	_vte->_acc_sample_count = 3;
	_vte->_vehicle_acc_ned_sum = matrix::Vector3f(50.f, 60.f, 70.f);
	const hrt_abstime now = vte_test::nowUs();
	const hrt_abstime acc_downsample_timeout = VisionTargetEstTestable::kMinAccDownsampleTimeoutUs;
	_vte->_last_acc_reset = now - (acc_downsample_timeout + 1_ms);
	_vte->_last_update_pos = now;

	const matrix::Vector3f acc_sample(1.f, 2.f, 3.f);
	VisionTargetEstTestable::LocalPose local_pose{};
	_vte->updatePosEst(local_pose, false, acc_sample, matrix::Vector3f{}, matrix::Vector3f{}, false);

	EXPECT_EQ(_vte->_acc_sample_count, 1u);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(0), acc_sample(0), kTolerance);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(1), acc_sample(1), kTolerance);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(2), acc_sample(2), kTolerance);
}

TEST_F(VisionTargetEstTest, UpdatePosEstAveragesSamplesCorrectly)
{
	// WHY: The averaged acceleration must reflect the mean of collected samples.
	// WHAT: Feed multiple samples, trigger an update, and verify the published mean.
	VisionTargetEstTestable::LocalPose local_pose{};
	const hrt_abstime now = vte_test::nowUs();
	_vte->_vehicle_acc_ned_sum.setAll(0.f);
	_vte->_acc_sample_count = 0;
	_vte->_last_acc_reset = now;
	_vte->_last_update_pos = now;

	const matrix::Vector3f gps_offset{};
	const matrix::Vector3f vel_offset{};

	const matrix::Vector3f acc1(1.f, 2.f, 3.f);
	_vte->updatePosEst(local_pose, false, acc1, gps_offset, vel_offset, false);

	const matrix::Vector3f acc2(3.f, 4.f, 5.f);
	_vte->updatePosEst(local_pose, false, acc2, gps_offset, vel_offset, false);

	EXPECT_EQ(_vte->_acc_sample_count, 2u);
	const matrix::Vector3f acc_sum = acc1 + acc2;
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(0), acc_sum(0), kTolerance);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(1), acc_sum(1), kTolerance);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(2), acc_sum(2), kTolerance);

	vte_test::advanceMicroseconds(VisionTargetEstTestable::kPosUpdatePeriodUs + 1_ms);
	const matrix::Vector3f acc3(2.f, 6.f, 1.f);
	_vte->updatePosEst(local_pose, false, acc3, gps_offset, vel_offset, false);

	ASSERT_TRUE(_vte_input_sub->update());
	const auto msg = _vte_input_sub->get();
	const matrix::Vector3f mean_acc = (acc1 + acc2 + acc3) / 3.f;
	EXPECT_NEAR(msg.acc_xyz[0], mean_acc(0), kTolerance);
	EXPECT_NEAR(msg.acc_xyz[1], mean_acc(1), kTolerance);
	EXPECT_NEAR(msg.acc_xyz[2], mean_acc(2), kTolerance);

	EXPECT_EQ(_vte->_acc_sample_count, 0u);
}

TEST_F(VisionTargetEstTest, StartEstimatorsEnforceRestartTimeout)
{
	// WHY: Restart timeout prevents rapid toggling of estimators.
	// WHAT: Start, stop, and verify restart is blocked until the timeout passes.
	_vte->_vte_position_enabled = true;
	_vte->_vte_orientation_enabled = true;
	_vte->_current_task.value = 0;

	const hrt_abstime now = vte_test::nowUs();
	const hrt_abstime past_time = now - (VisionTargetEstTestable::kEstRestartTimeUs + 1_ms);
	_vte->_vte_position_stop_time = past_time;
	_vte->_vte_orientation_stop_time = past_time;

	EXPECT_TRUE(_vte->startPosEst());
	EXPECT_TRUE(_vte->startYawEst());

	_vte->stopPosEst();
	_vte->stopYawEst();

	EXPECT_FALSE(_vte->startPosEst());
	EXPECT_FALSE(_vte->startYawEst());

	_vte->_vte_position_stop_time = vte_test::nowUs() - (VisionTargetEstTestable::kEstRestartTimeUs - 1_ms);
	_vte->_vte_orientation_stop_time = vte_test::nowUs() - (VisionTargetEstTestable::kEstRestartTimeUs - 1_ms);

	EXPECT_FALSE(_vte->startPosEst());
	EXPECT_FALSE(_vte->startYawEst());

	vte_test::advanceMicroseconds(2_ms);
	EXPECT_TRUE(_vte->startPosEst());
	EXPECT_TRUE(_vte->startYawEst());
}

TEST_F(VisionTargetEstTest, EstimatorUpdatePeriod)
{
	// WHY: Changing the frequency of the filter has consequences.
	// If the period is changed, the OOSM windows:
	//	OOSMManager<KF_position, vtest::State::size, float, 25> _oosm;
	//	OOSMManager<KF_orientation, State::size, EmptyInput, 25> _oosm;
	// must be updated accordingly
	// Other constants such as kAccUpdatedTimeoutUs or kMinAccDownsampleTimeoutUs
	ASSERT_EQ(VisionTargetEstTestable::kPosUpdatePeriodUs, 20_ms);
	ASSERT_EQ(VisionTargetEstTestable::kYawUpdatePeriodUs, 20_ms);
	EXPECT_LT(VisionTargetEstTestable::kAccUpdatedTimeoutUs, VisionTargetEstTestable::kMinAccDownsampleTimeoutUs);
}

TEST_F(VisionTargetEstTest, AllEstStoppedDueToTimeoutStopsPositionEstimator)
{
	// WHY: Timeout detection must stop a running estimator and record stop time.
	// WHAT: Initialize, force a timeout, and verify the estimator stops.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;
	_vte->_orientation_estimator_running = false;

	ASSERT_TRUE(_vte->_vte_position.init());
	vte::SensorFusionMaskU aid_mask{};
	aid_mask.flags.use_vision_pos = 1;
	_vte->_vte_position.set_vte_aid_mask(aid_mask.value);

	const hrt_abstime timestamp = vte_test::nowUs();
	_vte->_vte_position.set_local_velocity(matrix::Vector3f{}, true, timestamp);

	uORB::Publication<fiducial_marker_pos_report_s> vision_pub{ORB_ID(fiducial_marker_pos_report)};
	ASSERT_TRUE(vte_test::publishVisionPos(vision_pub, matrix::Vector3f(1.f, 0.f, -1.f),
					       vte_test::identityQuat(),
					       matrix::Vector3f(0.01f, 0.01f, 0.01f), timestamp));

	_vte->_vte_position.update(matrix::Vector3f{0.f, 0.f, 0.f});
	const hrt_abstime time_param_update = vte_test::nowUs();
	_vte->_vte_position.set_vte_timeout(1_ms);
	EXPECT_LT(_vte->_vte_position_stop_time, time_param_update); // running

	vte_test::advanceMicroseconds(2_ms);

	EXPECT_TRUE(_vte->allEstStoppedDueToTimeout());
	EXPECT_FALSE(_vte->_position_estimator_running);
	EXPECT_GT(_vte->_vte_position_stop_time, time_param_update);
}

TEST_F(VisionTargetEstTest, UpdateEstimatorsHandlesMissingLocalPose)
{
	// WHY: Estimator updates should tolerate missing local pose data.
	// WHAT: Provide attitude and acceleration only and ensure update proceeds.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;

	const hrt_abstime timestamp = vte_test::nowUs();
	publishAttitude(vte_test::identityQuat(), timestamp);
	publishAcceleration(matrix::Vector3f(0.1f, -0.2f, 9.5f), timestamp);

	_vte->_last_update_pos = timestamp - (VisionTargetEstTestable::kPosUpdatePeriodUs + 1_ms);
	_vte->_last_acc_reset = timestamp;

	_vte->updateEstimators();

	ASSERT_TRUE(_vte_input_sub->update());
}

TEST_F(VisionTargetEstTest, UpdateEstimatorsWarnsOnStaleAcceleration)
{
	// WHY: Stale acceleration should reset downsampling and update warning timestamp.
	// WHAT: Publish stale accel data and verify reset and warning tracking.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;
	_vte->_acc_sample_count = 2;
	_vte->_vehicle_acc_ned_sum = matrix::Vector3f(1.f, 2.f, 3.f);

	const hrt_abstime warn_before = _vte->_acc_sample_warn_last;
	const hrt_abstime timestamp = vte_test::nowUs();
	publishAttitude(vte_test::identityQuat(), timestamp);

	const hrt_abstime stale_timestamp = timestamp - (VisionTargetEstTestable::kAccUpdatedTimeoutUs + 1_ms);
	publishAcceleration(matrix::Vector3f(0.1f, -0.2f, 9.5f), stale_timestamp);

	_vte->updateEstimators();

	EXPECT_EQ(_vte->_acc_sample_count, 0u);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(0), 0.f, kTolerance);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(1), 0.f, kTolerance);
	EXPECT_NEAR(_vte->_vehicle_acc_ned_sum(2), 0.f, kTolerance);
	EXPECT_GT(_vte->_acc_sample_warn_last, warn_before);
}

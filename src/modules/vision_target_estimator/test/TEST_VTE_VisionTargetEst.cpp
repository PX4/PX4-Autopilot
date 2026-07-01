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
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/fiducial_marker_yaw_report.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vte_input.h>
#include <uORB/uORBManager.hpp>
#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
#include <uORB/topics/prec_land_status.h>
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
#include <matrix/Quaternion.hpp>
#include <matrix/Vector.hpp>

#include "VTETestHelper.hpp"
#include "VisionTargetEst.h"
#include "../../navigator/navigation.h"

namespace vte = vision_target_estimator;

namespace
{
using namespace time_literals;

static constexpr float kTolerance = 1e-4f;
static constexpr float kGravity = 9.80665f;
static constexpr hrt_abstime kStepUs = 1_ms;

void expectVectorNear(const matrix::Vector3f &actual, const matrix::Vector3f &expected)
{
	EXPECT_NEAR(actual(0), expected(0), kTolerance);
	EXPECT_NEAR(actual(1), expected(1), kTolerance);
	EXPECT_NEAR(actual(2), expected(2), kTolerance);
}

void expectVectorArrayNear(const float *actual, const matrix::Vector3f &expected)
{
	EXPECT_NEAR(actual[0], expected(0), kTolerance);
	EXPECT_NEAR(actual[1], expected(1), kTolerance);
	EXPECT_NEAR(actual[2], expected(2), kTolerance);
}

void expectQuaternionArrayNear(const float *actual, const matrix::Quaternionf &expected)
{
	EXPECT_NEAR(actual[0], expected(0), kTolerance);
	EXPECT_NEAR(actual[1], expected(1), kTolerance);
	EXPECT_NEAR(actual[2], expected(2), kTolerance);
	EXPECT_NEAR(actual[3], expected(3), kTolerance);
}

position_setpoint_triplet_s makePositionSetpointTriplet(hrt_abstime timestamp, uint8_t current_type, uint8_t next_type)
{
	position_setpoint_triplet_s triplet{};
	triplet.timestamp = timestamp;
	triplet.current.type = current_type;
	triplet.next.type = next_type;
	return triplet;
}

navigator_mission_item_s makeNavigatorMissionItem(hrt_abstime timestamp, uint16_t nav_cmd,
		float lat, float lon, float alt, bool altitude_is_relative = false)
{
	navigator_mission_item_s mission_item{};
	mission_item.timestamp = timestamp;
	mission_item.nav_cmd = nav_cmd;
	mission_item.latitude = lat;
	mission_item.longitude = lon;
	mission_item.altitude = alt;
	mission_item.altitude_is_relative = altitude_is_relative;
	return mission_item;
}

} // namespace

class VisionTargetEstTestable : public vte::VisionTargetEst
{
public:
	using vte::VisionTargetEst::_acc_sample_count;
	using vte::VisionTargetEst::_current_task_ptr;
	using vte::VisionTargetEst::_gps_pos_is_offset;
	using vte::VisionTargetEst::_gps_pos_offset_xyz;
	using vte::VisionTargetEst::_last_acc_reset;
	using vte::VisionTargetEst::_last_update_pos;
	using vte::VisionTargetEst::_orientation_estimator_running;
	using vte::VisionTargetEst::_prec_land_task;
	using vte::VisionTargetEst::_vte_orientation;
	using vte::VisionTargetEst::_position_estimator_running;
	using vte::VisionTargetEst::_vehicle_acc_body;
	using vte::VisionTargetEst::_vehicle_acc_ned_sum;
	using vte::VisionTargetEst::_vehicle_acceleration_sub;
	using vte::VisionTargetEst::_vehicle_angular_velocity_sub;
	using vte::VisionTargetEst::_vehicle_attitude_sub;
	using vte::VisionTargetEst::_vehicle_gps_position_sub;
	using vte::VisionTargetEst::_vehicle_local_position_sub;
	using vte::VisionTargetEst::_vte_orientation_enabled;
	using vte::VisionTargetEst::_vte_position;
	using vte::VisionTargetEst::_vte_position_enabled;
	using vte::VisionTargetEst::_vte_task_mask;
	using vte::VisionTargetEst::adjustAidMask;
	using vte::VisionTargetEst::computeGpsVelocityOffset;
	using vte::VisionTargetEst::isCurrentTaskComplete;
	using vte::VisionTargetEst::restartTimedOutEstimators;
	using vte::VisionTargetEst::setNewTaskIfAvailable;
	using vte::VisionTargetEst::kAccUpdatedTimeoutUs;
	using vte::VisionTargetEst::kMinAccDownsampleTimeoutUs;
	using vte::VisionTargetEst::kPosUpdatePeriodUs;
	using vte::VisionTargetEst::kYawUpdatePeriodUs;
	using vte::VisionTargetEst::pollEstimatorInput;
	using vte::VisionTargetEst::publishVteInput;
	using vte::VisionTargetEst::startPosEst;
	using vte::VisionTargetEst::startYawEst;
	using vte::VisionTargetEst::stopPosEst;
	using vte::VisionTargetEst::stopYawEst;
	using vte::VisionTargetEst::updateGpsAntennaOffset;
	using vte::VisionTargetEst::updateEstimators;
	using vte::VisionTargetEst::updateParams;
	using vte::VisionTargetEst::updateTaskTopics;
	using vte::VisionTargetEst::updateWhenIntervalElapsed;

	void clearCurrentTask() { _current_task_ptr = nullptr; }
	void setCurrentTaskToPrecLand() { _current_task_ptr = &_prec_land_task; }
	bool isCurrentTaskPrecLand() const { return _current_task_ptr == &_prec_land_task; }

	const position_setpoint_s *findLandSetpoint() { return _prec_land_task.findLandSetpoint(); }
	const position_setpoint_s *currentTripletLandSetpoint() { return &_prec_land_task._pos_sp_triplet_buffer.current; }
	const position_setpoint_s *nextTripletLandSetpoint() { return &_prec_land_task._pos_sp_triplet_buffer.next; }

	void resetPrecLandMissionSetpoint() { _prec_land_task.resetMissionSetpoint(); }
	bool updatePrecLandMissionSetpoint() { return _prec_land_task.updateMissionSetpoint(); }
	const position_setpoint_s &precLandMissionSetpoint() const { return _prec_land_task._cached_mission_setpoint; }

	void setPrecLandActive(bool active) { _prec_land_task._is_in_prec_land = active; }
	bool isPrecLandActive() const { return _prec_land_task._is_in_prec_land; }
};

class VisionTargetEstTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite() { uORB::Manager::initialize(); }

	void SetUp() override
	{
		param_control_autosave(false);
		vte_test::useFakeTime();

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
		_uav_gps_pub = std::make_unique<uORB::Publication<sensor_gps_s>>(ORB_ID(vehicle_gps_position));
		_home_position_pub = std::make_unique<uORB::Publication<home_position_s>>(ORB_ID(home_position));
		_navigator_mission_item_pub = std::make_unique<uORB::Publication<navigator_mission_item_s>>(ORB_ID(navigator_mission_item));
		_pos_sp_triplet_pub = std::make_unique<uORB::Publication<position_setpoint_triplet_s>>(ORB_ID(position_setpoint_triplet));
		_land_detected_pub = std::make_unique<uORB::Publication<vehicle_land_detected_s>>(ORB_ID(vehicle_land_detected));
#if !defined(CONSTRAINED_FLASH)
		_prec_land_status_pub = std::make_unique<uORB::Publication<prec_land_status_s>>(ORB_ID(prec_land_status));
#endif

		_vte_input_sub = std::make_unique<uORB::SubscriptionData<vte_input_s>>(ORB_ID(vte_input));

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
		_navigator_mission_item_pub.reset();
		_home_position_pub.reset();
		_uav_gps_pub.reset();
		_ang_vel_pub.reset();
		_accel_pub.reset();
		_attitude_pub.reset();

		_vte.reset();
		vte_test::useRealTime();
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

	void publishUavGps(const matrix::Vector3f &antenna_offset, hrt_abstime timestamp)
	{
		ASSERT_TRUE(vte_test::publishUavGps(*_uav_gps_pub, 47.0, 8.0, 500.f, 0.5f, 0.5f,
						    matrix::Vector3f{}, 0.1f, true, timestamp, antenna_offset));
	}

	void publishHomePosition(float alt_amsl, hrt_abstime timestamp)
	{
		home_position_s msg{};
		msg.timestamp = timestamp;
		msg.alt = alt_amsl;
		ASSERT_TRUE(_home_position_pub->publish(msg));
	}

	void publishNavigatorMissionItem(const navigator_mission_item_s &mission_item)
	{
		ASSERT_TRUE(_navigator_mission_item_pub->publish(mission_item));
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
		vte_test::flushSubscription<sensor_gps_s>(_vte->_vehicle_gps_position_sub);
		vte_test::flushSubscription<vehicle_local_position_s>(_vte->_vehicle_local_position_sub);
		vte_test::flushSubscription<home_position_s>(_vte->_prec_land_task._home_position_sub);
		vte_test::flushSubscription<navigator_mission_item_s>(_vte->_prec_land_task._navigator_mission_item_sub);
		vte_test::flushSubscription<position_setpoint_triplet_s>(_vte->_prec_land_task._pos_sp_triplet_sub);
		vte_test::flushSubscription<vehicle_land_detected_s>(_vte->_prec_land_task._vehicle_land_detected_sub);
#if !defined(CONSTRAINED_FLASH)
		vte_test::flushSubscription<prec_land_status_s>(_vte->_prec_land_task._prec_land_status_sub);
#endif
	}

	std::unique_ptr<VisionTargetEstTestable> _vte;
	std::unique_ptr<uORB::Publication<vehicle_attitude_s>> _attitude_pub;
	std::unique_ptr<uORB::Publication<vehicle_acceleration_s>> _accel_pub;
	std::unique_ptr<uORB::Publication<vehicle_angular_velocity_s>> _ang_vel_pub;
	std::unique_ptr<uORB::Publication<sensor_gps_s>> _uav_gps_pub;
	std::unique_ptr<uORB::Publication<home_position_s>> _home_position_pub;
	std::unique_ptr<uORB::Publication<navigator_mission_item_s>> _navigator_mission_item_pub;
	std::unique_ptr<uORB::Publication<position_setpoint_triplet_s>> _pos_sp_triplet_pub;
	std::unique_ptr<uORB::Publication<vehicle_land_detected_s>> _land_detected_pub;
#if !defined(CONSTRAINED_FLASH)
	std::unique_ptr<uORB::Publication<prec_land_status_s>> _prec_land_status_pub;
#endif

	std::unique_ptr<uORB::SubscriptionData<vte_input_s>> _vte_input_sub;

	vte_test::ParamGuard _param_guard{};
};

// WHY: If there is a GNSS on the target, do not use the mission position
// WHAT: Enable mission position and GNSS on target, expect GNSS on target only
TEST_F(VisionTargetEstTest, AdjustAidMaskResolvesConflicts)
{
	// GIVEN: Mission position and target GNSS aiding are both enabled.
	vte::SensorFusionMaskU mask{};
	mask.flags.use_target_gps_pos = 1;
	mask.flags.use_mission_pos = 1;

	// WHEN: The aid mask is sanitized.
	const uint16_t adjusted = _vte->adjustAidMask(mask.value);
	vte::SensorFusionMaskU adjusted_mask{};
	adjusted_mask.value = adjusted;

	// THEN: Mission position aiding is dropped in favor of target GNSS.
	EXPECT_TRUE(adjusted_mask.flags.use_target_gps_pos);
	EXPECT_FALSE(adjusted_mask.flags.use_mission_pos);
}

#if defined(CONFIG_VTEST_MOVING)
// WHY: Moving-target builds cannot use a static mission landing point as a target position source.
// WHAT: Enable mission-position aiding by itself and expect adjustAidMask() to clear it.
TEST_F(VisionTargetEstTest, AdjustAidMaskDisablesMissionPositionForMovingTarget)
{
	vte::SensorFusionMaskU mask{};
	mask.flags.use_mission_pos = 1;

	vte::SensorFusionMaskU adjusted_mask{};
	adjusted_mask.value = _vte->adjustAidMask(mask.value);

	EXPECT_FALSE(adjusted_mask.flags.use_mission_pos);
}
#endif

// WHY: The active GPS antenna offset comes from vehicle_gps_position
// WHAT: Publish non-zero then zero antenna offsets and verify the cached lever arm tracks the topic output.
TEST_F(VisionTargetEstTest, UpdateGpsAntennaOffsetTracksVehicleGpsPosition)
{
	// GIVEN: The vehicle GPS topic reports a non-zero antenna lever arm.
	const matrix::Vector3f gps_offset_gt{0.2, -0.015, 3.1};
	publishUavGps(gps_offset_gt, vte_test::advanceMicroseconds(kStepUs));

	// WHEN: The module refreshes the cached GPS antenna offset.
	ASSERT_TRUE(_vte->updateGpsAntennaOffset());

	// THEN: The cached lever arm matches the published GPS offset.
	EXPECT_TRUE(_vte->_gps_pos_is_offset);
	expectVectorNear(_vte->_gps_pos_offset_xyz, gps_offset_gt);

	// GIVEN: The next GPS sample reports no antenna offset.
	publishUavGps(matrix::Vector3f{}, vte_test::advanceMicroseconds(kStepUs));

	// WHEN: The offset cache is refreshed again.
	ASSERT_TRUE(_vte->updateGpsAntennaOffset());

	// THEN: The cached lever arm is cleared.
	EXPECT_FALSE(_vte->_gps_pos_is_offset);
	expectVectorNear(_vte->_gps_pos_offset_xyz, matrix::Vector3f{});
}

// WHY: Compensate for the rotational velocity if the GNSS antenna is not at the center of mass,
// i.e. when the drone rotates around the center of mass, the GNSS will record a velocity.
// WHAT: Expect false without config/data, then true once angular velocity is published.
TEST_F(VisionTargetEstTest, ComputeGpsVelocityOffsetRequiresOffsetAndData)
{
	// GIVEN: No lever arm or angular velocity samples are available yet.
	matrix::Vector3f vel_offset{};

	// WHEN: The velocity compensation is computed without prerequisites.
	EXPECT_FALSE(_vte->computeGpsVelocityOffset(vel_offset));

	// GIVEN: A GPS antenna lever arm is configured, but angular velocity is still missing.
	const matrix::Vector3f gps_offset(1.f, 0.f, 0.f);
	_vte->_gps_pos_is_offset = true;
	_vte->_gps_pos_offset_xyz = gps_offset;

	// WHEN: The compensation is computed again without gyro data.
	EXPECT_FALSE(_vte->computeGpsVelocityOffset(vel_offset));

	// GIVEN: Angular velocity data is now available.
	const matrix::Vector3f ang_vel(0.f, 0.f, 1.f);
	publishAngularVelocity(ang_vel, vte_test::advanceMicroseconds(kStepUs));

	// WHEN: The velocity compensation is recomputed with all required inputs.
	EXPECT_TRUE(_vte->computeGpsVelocityOffset(vel_offset));
	const matrix::Vector3f expected_offset = ang_vel.cross(gps_offset);

	// THEN: The offset matches the rigid-body cross product.
	expectVectorNear(vel_offset, expected_offset);
}

// WHY: Attitude is required to transform body-frame data into NED.
// WHAT: Call pollEstimatorInput without attitude and expect it to fail.
TEST_F(VisionTargetEstTest, PollEstimatorInputRequiresAttitude)
{
	// GIVEN: No vehicle attitude sample has been published.
	matrix::Vector3f acc_ned{};
	matrix::Quaternionf q_att{};
	matrix::Vector3f gps_offset{};
	matrix::Vector3f vel_offset{};
	const bool vel_offset_updated = true;
	bool acc_valid = true;

	// WHEN: The estimator input is polled.
	EXPECT_FALSE(_vte->pollEstimatorInput(acc_ned, q_att, gps_offset, vel_offset,
					      vel_offset_updated, acc_valid));
}

// WHY: Valid inputs should be rotated into NED and include gravity compensation.
// WHAT: Publish attitude/accel with offsets and verify transformed outputs.
TEST_F(VisionTargetEstTest, PollEstimatorInputTransformsAccelerationAndOffsets)
{
	// GIVEN: Attitude, acceleration, and GPS lever-arm data are all available.
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

	// WHEN: The estimator input is polled.
	ASSERT_TRUE(_vte->pollEstimatorInput(acc_ned, q_att, gps_offset, vel_offset,
					     vel_offset_updated, acc_valid));

	// THEN: Acceleration and offset vectors are rotated into NED.
	EXPECT_TRUE(acc_valid);
	expectVectorNear(acc_ned, matrix::Vector3f(rotated_vec(0), rotated_vec(1), rotated_vec(2) + kGravity));
	expectVectorNear(gps_offset, rotated_vec);
	expectVectorNear(vel_offset, rotated_vec);
}

// WHY: With GPS offset disabled, offset vectors must be zeroed.
// WHAT: Disable offset and verify returned GNSS/velocity offsets are zero.
TEST_F(VisionTargetEstTest, PollEstimatorInputNoOffsetZerosVectors)
{
	// GIVEN: Attitude and acceleration are present, but GPS offset compensation is disabled.
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

	// WHEN: The estimator input is polled.
	ASSERT_TRUE(_vte->pollEstimatorInput(acc_ned, q_att, gps_offset, vel_offset,
					     vel_offset_updated, acc_valid));

	// THEN: Offset vectors are zeroed even if stale values were passed in.
	EXPECT_TRUE(acc_valid);
	expectVectorNear(gps_offset, matrix::Vector3f{});
	expectVectorNear(vel_offset, matrix::Vector3f{});
}

// WHY: Published inputs must reflect the sampled accel and attitude data.
// WHAT: Populate sample timestamp and verify the published message fields.
TEST_F(VisionTargetEstTest, PublishVteInputPopulatesMessage)
{
	// GIVEN: A sampled acceleration timestamp and estimator inputs are prepared.
	const hrt_abstime sample_time = vte_test::nowUs();
	_vte->_vehicle_acc_body.timestamp = sample_time;

	const matrix::Vector3f acc_ned(0.5f, -0.25f, 9.0f);
	const matrix::Quaternionf q_att = vte_test::identityQuat();
	const uint32_t sample_count = 4;

	// WHEN: The vision target estimator input topic is published.
	_vte->publishVteInput(acc_ned, q_att, sample_count);

	// THEN: The published message mirrors the sampled timestamp and data payload.
	ASSERT_TRUE(_vte_input_sub->update());
	const auto msg = _vte_input_sub->get();

	EXPECT_EQ(msg.timestamp_sample, sample_time);
	expectVectorArrayNear(msg.acc_xyz, acc_ned);
	expectQuaternionArrayNear(msg.q_att, q_att);
	EXPECT_EQ(msg.acc_sample_count, sample_count);
}

// WHY: The update gate should only allow execution after the interval elapses.
// WHAT: Expect false before the interval and true after waiting.
TEST_F(VisionTargetEstTest, UpdateWhenIntervalElapsedRespectsInterval)
{
	// GIVEN: An interval and a timestamp from the current simulated time.
	const hrt_abstime interval = 10_ms;
	hrt_abstime last_time = vte_test::nowUs();

	// WHEN: The interval has not elapsed yet.
	EXPECT_FALSE(_vte->updateWhenIntervalElapsed(last_time, interval));

	// GIVEN: The last update time is moved far enough into the past.
	last_time = vte_test::nowUs() - interval - 1_ms;

	// WHEN: The interval check runs again.
	// THEN: The update is allowed.
	EXPECT_TRUE(_vte->updateWhenIntervalElapsed(last_time, interval));
}

// WHY: The triplet fallback must ignore stale land types on invalid setpoints.
// WHAT: Publish invalid then valid land setpoints and verify only valid entries are selected.
TEST_F(VisionTargetEstTest, FindLandSetpointRequiresValidSetpoint)
{
	// GIVEN: No setpoint triplet has been published yet.
	EXPECT_EQ(_vte->findLandSetpoint(), nullptr);

	// GIVEN: The current setpoint is marked as land but invalid.
	position_setpoint_triplet_s triplet = makePositionSetpointTriplet(
			vte_test::advanceMicroseconds(kStepUs),
			position_setpoint_s::SETPOINT_TYPE_LAND,
			position_setpoint_s::SETPOINT_TYPE_IDLE);
	triplet.current.valid = false;
	publishPositionSetpointTriplet(triplet);

	// WHEN: The active land setpoint is queried.
	// THEN: Invalid land entries are ignored.
	EXPECT_EQ(_vte->findLandSetpoint(), nullptr);

	// GIVEN: The current setpoint is a valid land setpoint.
	triplet.current.valid = true;
	triplet.current.lat = 47.1;
	triplet.current.lon = 8.2;
	triplet.current.alt = 520.f;
	publishPositionSetpointTriplet(triplet);

	// WHEN: The active land setpoint is queried.
	const position_setpoint_s *current_sp = _vte->findLandSetpoint();

	// THEN: The current setpoint is returned.
	ASSERT_NE(current_sp, nullptr);
	EXPECT_EQ(current_sp, _vte->currentTripletLandSetpoint());

	// GIVEN: The next setpoint is now the land setpoint.
	triplet = makePositionSetpointTriplet(
			  vte_test::advanceMicroseconds(kStepUs),
			  position_setpoint_s::SETPOINT_TYPE_IDLE,
			  position_setpoint_s::SETPOINT_TYPE_LAND);
	triplet.current.valid = false;
	triplet.next.valid = true;
	triplet.next.lat = 47.2;
	triplet.next.lon = 8.3;
	triplet.next.alt = 530.f;
	publishPositionSetpointTriplet(triplet);

	// WHEN: The land setpoint is queried again.
	const position_setpoint_s *next_sp = _vte->findLandSetpoint();

	// THEN: The next setpoint is selected and remains cached for repeated reads.
	ASSERT_NE(next_sp, nullptr);
	EXPECT_EQ(next_sp, _vte->nextTripletLandSetpoint());

	const position_setpoint_s *cached_sp = _vte->findLandSetpoint();
	ASSERT_NE(cached_sp, nullptr);
	EXPECT_EQ(cached_sp, _vte->nextTripletLandSetpoint());
}

// WHY: Precision landing needs a stable mission waypoint even after precland mutates the live triplet.
// WHAT: Cache the navigator mission land item, then verify task start keeps using it after the triplet changes.
TEST_F(VisionTargetEstTest, StartPosEstUsesCachedNavigatorMissionLandSetpoint)
{
	// GIVEN: A precision-land task with a stable navigator mission land item and a mutable triplet.
	_vte->_vte_position_enabled = true;
	_vte->setCurrentTaskToPrecLand();
	_vte->resetPrecLandMissionSetpoint();

	const hrt_abstime timestamp = vte_test::advanceMicroseconds(kStepUs);
	publishHomePosition(500.f, timestamp);
	publishNavigatorMissionItem(makeNavigatorMissionItem(timestamp, NAV_CMD_LAND, 47.1f, 8.2f, 15.f, true));

	position_setpoint_triplet_s triplet = makePositionSetpointTriplet(timestamp,
					      position_setpoint_s::SETPOINT_TYPE_POSITION,
					      position_setpoint_s::SETPOINT_TYPE_IDLE);
	triplet.current.valid = true;
	triplet.current.lat = 47.9;
	triplet.current.lon = 8.9;
	triplet.current.alt = 650.f;
	publishPositionSetpointTriplet(triplet);

	// WHEN: The position estimator starts for precision landing.
	ASSERT_TRUE(_vte->startPosEst());

	// THEN: The cached mission waypoint is used instead of the rewritten triplet.
	EXPECT_TRUE(_vte->precLandMissionSetpoint().valid);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().lat, 47.1, kTolerance);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().lon, 8.2, kTolerance);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().alt, 515.f, kTolerance);
}

// WHY: The mission controller can expose the upcoming landing point in triplet.next before the current logical mission item becomes LAND.
// WHAT: Publish a non-land navigator mission item with a valid next land setpoint and verify VTE still caches the upcoming landing waypoint.
TEST_F(VisionTargetEstTest, UpdatePrecLandMissionSetpointFallsBackToNextLandSetpoint)
{
	// GIVEN: The current logical mission item is not LAND yet, but the next triplet setpoint already is.
	_vte->resetPrecLandMissionSetpoint();

	const hrt_abstime timestamp = vte_test::advanceMicroseconds(kStepUs);
	publishNavigatorMissionItem(makeNavigatorMissionItem(timestamp, NAV_CMD_WAYPOINT, 47.0f, 8.0f, 540.f));

	position_setpoint_triplet_s triplet = makePositionSetpointTriplet(timestamp,
					      position_setpoint_s::SETPOINT_TYPE_POSITION,
					      position_setpoint_s::SETPOINT_TYPE_LAND);
	triplet.current.valid = true;
	triplet.current.lat = 47.0;
	triplet.current.lon = 8.0;
	triplet.current.alt = 540.f;
	triplet.next.valid = true;
	triplet.next.lat = 47.5;
	triplet.next.lon = 8.5;
	triplet.next.alt = 520.f;
	publishPositionSetpointTriplet(triplet);

	// WHEN: The precision-land mission setpoint is updated.
	ASSERT_TRUE(_vte->updatePrecLandMissionSetpoint());

	// THEN: The upcoming LAND setpoint from triplet.next is cached.
	EXPECT_TRUE(_vte->precLandMissionSetpoint().valid);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().lat, triplet.next.lat, kTolerance);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().lon, triplet.next.lon, kTolerance);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().alt, triplet.next.alt, kTolerance);
}

// WHY: Internal estimator restarts during the same precision-land task must not switch to precland's dynamic triplet.
// WHAT: Start once from navigator mission data, mutate the triplet to a different land point, then restart and verify the cached point stays fixed.
TEST_F(VisionTargetEstTest, StartPosEstKeepsCachedMissionLandSetpointAcrossRestart)
{
	// GIVEN: Precision landing already cached the original mission land setpoint.
	_vte->_vte_position_enabled = true;
	_vte->setCurrentTaskToPrecLand();
	_vte->resetPrecLandMissionSetpoint();

	const hrt_abstime timestamp = vte_test::advanceMicroseconds(kStepUs);
	publishNavigatorMissionItem(makeNavigatorMissionItem(timestamp, NAV_CMD_LAND, 47.3f, 8.4f, 530.f));
	ASSERT_TRUE(_vte->startPosEst());

	const position_setpoint_s cached_setpoint = _vte->precLandMissionSetpoint();

	// GIVEN: Precland later rewrites the current triplet to another land position.
	position_setpoint_triplet_s triplet = makePositionSetpointTriplet(vte_test::advanceMicroseconds(kStepUs),
					      position_setpoint_s::SETPOINT_TYPE_LAND,
					      position_setpoint_s::SETPOINT_TYPE_IDLE);
	triplet.current.valid = true;
	triplet.current.lat = 48.0;
	triplet.current.lon = 9.0;
	triplet.current.alt = 700.f;
	publishPositionSetpointTriplet(triplet);

	// WHEN: The estimator is stopped and restarted within the same task.
	_vte->stopPosEst();
	ASSERT_TRUE(_vte->startPosEst());

	// THEN: The original mission land point is reused instead of the rewritten triplet.
	EXPECT_NEAR(_vte->precLandMissionSetpoint().lat, cached_setpoint.lat, kTolerance);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().lon, cached_setpoint.lon, kTolerance);
	EXPECT_NEAR(_vte->precLandMissionSetpoint().alt, cached_setpoint.alt, kTolerance);
}

// WHY: Precision landing tasks should start once requested and complete on land detect.
// WHAT: Request the task, then publish land detected and verify completion.
TEST_F(VisionTargetEstTest, PrecisionLandTaskRequestAndCompletion)
{
	// GIVEN: Precision landing is enabled and currently active.
	_vte->_vte_task_mask = vte::task_bits::kPrecLand;
	_vte->clearCurrentTask();
	_vte->setPrecLandActive(true);

	// WHEN: Task availability is checked.
	EXPECT_TRUE(_vte->setNewTaskIfAvailable());

	// THEN: The precision landing task becomes current.
	EXPECT_TRUE(_vte->isCurrentTaskPrecLand());

	// GIVEN: Land detection reports touchdown.
	publishLandDetected(true, vte_test::advanceMicroseconds(kStepUs));

	// WHEN: Task completion is checked.
	// THEN: The task completes and precision landing state is cleared.
	EXPECT_TRUE(_vte->isCurrentTaskComplete());
	EXPECT_FALSE(_vte->isPrecLandActive());
}

#if !defined(CONSTRAINED_FLASH)
// WHY: Precision landing status topics should drive internal task state.
// WHAT: Publish ongoing and stopped states and verify the internal flag toggles.
TEST_F(VisionTargetEstTest, UpdateTaskTopicsTracksPrecisionLandState)
{
	// GIVEN: Precision landing task tracking is enabled.
	_vte->_vte_task_mask = vte::task_bits::kPrecLand;
	_vte->setPrecLandActive(false);

	// WHEN: The precision landing status reports an active state.
	publishPrecLandStatus(prec_land_status_s::PREC_LAND_STATE_HORIZONTAL, vte_test::advanceMicroseconds(kStepUs));
	_vte->updateTaskTopics();

	// THEN: Internal task state reflects an active precision landing sequence.
	EXPECT_TRUE(_vte->isPrecLandActive());

	// WHEN: The precision landing status reports that it has stopped.
	publishPrecLandStatus(prec_land_status_s::PREC_LAND_STATE_STOPPED, vte_test::advanceMicroseconds(kStepUs));
	_vte->updateTaskTopics();

	// THEN: The internal precision landing state is cleared.
	EXPECT_FALSE(_vte->isPrecLandActive());
}
#endif

// WHY: The acceleration downsample must reset after timeout to avoid stale averages.
// WHAT: Force a timeout and verify the accumulator resets before admitting new samples.
TEST_F(VisionTargetEstTest, UpdateEstimatorsForcesDownsampleResetAfterTimeout)
{
	// GIVEN: The downsample accumulator already contains stale acceleration data.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;
	_vte->_acc_sample_count = 3;
	_vte->_vehicle_acc_ned_sum = matrix::Vector3f(50.f, 60.f, 70.f);

	const hrt_abstime now = vte_test::nowUs();
	const hrt_abstime acc_downsample_timeout = VisionTargetEstTestable::kMinAccDownsampleTimeoutUs;
	_vte->_last_acc_reset = now - (acc_downsample_timeout + 1_ms);
	_vte->_last_update_pos = now;  // prevent the position update from firing this cycle

	// WHEN: A cycle runs with a fresh sample but a stale accumulator timer.
	// Identity attitude: acc_ned = acc_body + (0, 0, g).
	publishAttitude(vte_test::identityQuat(), now);
	publishAcceleration(matrix::Vector3f(1.f, 2.f, 3.f - kGravity), now);
	_vte->updateEstimators();

	// THEN: The accumulator is reset and restarted from the new sample.
	EXPECT_EQ(_vte->_acc_sample_count, 1u);
	expectVectorNear(_vte->_vehicle_acc_ned_sum, matrix::Vector3f(1.f, 2.f, 3.f));
}

// WHY: The averaged acceleration must reflect the mean of collected samples.
// WHAT: Feed multiple samples, trigger an update, and verify the published mean.
TEST_F(VisionTargetEstTest, UpdateEstimatorsAveragesAccSamples)
{
	// GIVEN: A running position estimator with a clean accumulator.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;
	_vte->_vehicle_acc_ned_sum.setAll(0.f);
	_vte->_acc_sample_count = 0;

	const hrt_abstime t0 = vte_test::nowUs();
	_vte->_last_acc_reset = t0;
	_vte->_last_update_pos = t0;

	// WHEN: Two samples arrive before the publish interval elapses.
	// Identity attitude: acc_ned = acc_body + (0, 0, g).
	publishAttitude(vte_test::identityQuat(), t0);
	publishAcceleration(matrix::Vector3f(1.f, 2.f, 3.f - kGravity), t0);
	_vte->updateEstimators();

	vte_test::advanceMicroseconds(1_ms);
	const hrt_abstime t1 = vte_test::nowUs();
	publishAttitude(vte_test::identityQuat(), t1);
	publishAcceleration(matrix::Vector3f(3.f, 4.f, 5.f - kGravity), t1);
	_vte->updateEstimators();

	// THEN: The accumulator stores the running sum.
	EXPECT_EQ(_vte->_acc_sample_count, 2u);
	const matrix::Vector3f expected_sum(1.f + 3.f, 2.f + 4.f, 3.f + 5.f);
	expectVectorNear(_vte->_vehicle_acc_ned_sum, expected_sum);

	// WHEN: A third sample arrives after the publish interval has elapsed.
	vte_test::advanceMicroseconds(VisionTargetEstTestable::kPosUpdatePeriodUs + 1_ms);
	const hrt_abstime t2 = vte_test::nowUs();
	publishAttitude(vte_test::identityQuat(), t2);
	publishAcceleration(matrix::Vector3f(2.f, 6.f, 1.f - kGravity), t2);
	_vte->updateEstimators();

	// THEN: The published acceleration is the mean of all collected samples.
	ASSERT_TRUE(_vte_input_sub->update());
	const auto msg = _vte_input_sub->get();
	const matrix::Vector3f mean_acc((1.f + 3.f + 2.f) / 3.f,
					(2.f + 4.f + 6.f) / 3.f,
					(3.f + 5.f + 1.f) / 3.f);
	expectVectorArrayNear(msg.acc_xyz, mean_acc);

	EXPECT_EQ(_vte->_acc_sample_count, 0u);
}

// WHY: A stopped estimator should be available again as soon as the task needs it.
// WHAT: Start, stop, and verify restart is immediate.
TEST_F(VisionTargetEstTest, StartEstimatorsRestartImmediatelyAfterStop)
{
	// GIVEN: Both estimators are enabled.
	_vte->_vte_position_enabled = true;
	_vte->_vte_orientation_enabled = true;
	_vte->_current_task_ptr = nullptr;

	// WHEN: The estimators are started, stopped, and restarted immediately.
	EXPECT_TRUE(_vte->startPosEst());
	EXPECT_TRUE(_vte->startYawEst());

	_vte->stopPosEst();
	_vte->stopYawEst();

	// THEN: Both estimators can start again without a guard delay.
	EXPECT_TRUE(_vte->startPosEst());
	EXPECT_TRUE(_vte->startYawEst());
}

// WHY: Disabling the aid mask at runtime should stop any estimator that no longer has inputs.
// WHAT: Mark both estimators running, then clear VTE_AID_MASK and verify they are stopped.
TEST_F(VisionTargetEstTest, UpdateParamsStopsRunningEstimatorsWhenFusionDisabled)
{
	// GIVEN: Both estimators are running with a non-zero aid mask.
	_vte->_vte_position_enabled = true;
	_vte->_vte_orientation_enabled = true;
	_vte->_position_estimator_running = true;
	_vte->_orientation_estimator_running = true;

	vte::SensorFusionMaskU aid_mask{};
	aid_mask.flags.use_vision_pos = 1;
	setParamInt("VTE_AID_MASK", aid_mask.value);
	_vte->updateParams();

	// THEN: The estimators remain running while valid fusion input is configured.
	EXPECT_TRUE(_vte->_position_estimator_running);
	EXPECT_TRUE(_vte->_orientation_estimator_running);

	// WHEN: Fusion input is disabled at runtime.
	setParamInt("VTE_AID_MASK", 0);
	_vte->updateParams();

	// THEN: Both estimators stop.
	EXPECT_FALSE(_vte->_position_estimator_running);
	EXPECT_FALSE(_vte->_orientation_estimator_running);
}

// WHY: Changing the frequency of the filter has consequences.
// If the period is changed, the OOSM windows:
//      OOSMManager<KF_position, vtest::State::size, float, 25> _oosm;
//      OOSMManager<KF_orientation, State::size, EmptyInput, 25> _oosm;
// must be updated accordingly.
// Other constants such as kAccUpdatedTimeoutUs or kMinAccDownsampleTimeoutUs
// WHAT: Create a test that fails is the filter's period is changed.
TEST_F(VisionTargetEstTest, EstimatorUpdatePeriod)
{
	// THEN: The expected estimator timing constants stay in sync.
	ASSERT_EQ(VisionTargetEstTestable::kPosUpdatePeriodUs, 20_ms);
	ASSERT_EQ(VisionTargetEstTestable::kYawUpdatePeriodUs, 20_ms);
	EXPECT_EQ(vte::kOosmMinTimeUs, VisionTargetEstTestable::kPosUpdatePeriodUs);
	EXPECT_EQ(vte::kOosmMaxTimeUs, 500_ms);
	EXPECT_EQ(vte::kOosmHistorySize, 25);
	EXPECT_LT(VisionTargetEstTestable::kAccUpdatedTimeoutUs, VisionTargetEstTestable::kMinAccDownsampleTimeoutUs);
}

// WHY: Timeout detection must reset a running estimator without imposing a restart delay.
// WHAT: Initialize, force a timeout, and verify timeout handling re-arms the estimator immediately.
TEST_F(VisionTargetEstTest, RestartTimedOutEstimatorsRearmsPositionEstimatorImmediately)
{
	// GIVEN: A running position estimator with fresh vision input.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;
	_vte->_orientation_estimator_running = false;

	ASSERT_TRUE(_vte->_vte_position.init());
	vte::SensorFusionMaskU aid_mask{};
	aid_mask.flags.use_vision_pos = 1;
	_vte->_vte_position.setVteAidMask(aid_mask.value);

	const hrt_abstime timestamp = vte_test::nowUs();
	_vte->_vte_position.setLocalVelocity(matrix::Vector3f{}, true, timestamp);

	uORB::Publication<fiducial_marker_pos_report_s> vision_pub{ORB_ID(fiducial_marker_pos_report)};
	ASSERT_TRUE(vte_test::publishVisionPos(
			    vision_pub, matrix::Vector3f(1.f, 0.f, -1.f), vte_test::identityQuat(),
			    matrix::Vector3f(0.01f, 0.01f, 0.01f), timestamp));

	// WHEN: The estimator times out after receiving an initial update.
	_vte->_vte_position.update(matrix::Vector3f{0.f, 0.f, 0.f});
	_vte->_vte_position.setVteTimeout(1_ms);

	vte_test::advanceMicroseconds(2_ms);

	// THEN: Timeout handling resets and re-arms the position estimator in the same call.
	ASSERT_TRUE(_vte->_vte_position.timedOut());
	_vte->restartTimedOutEstimators();
	EXPECT_TRUE(_vte->_position_estimator_running);
	EXPECT_FALSE(_vte->_vte_position.timedOut());
}

// WHY: Timeout detection must reset a running yaw estimator without imposing a restart delay.
// WHAT: Initialize, force a timeout, and verify timeout handling re-arms the estimator immediately.
TEST_F(VisionTargetEstTest, RestartTimedOutEstimatorsRearmsOrientationEstimatorImmediately)
{
	// GIVEN: A running orientation estimator with fresh yaw input.
	_vte->_vte_orientation_enabled = true;
	_vte->_orientation_estimator_running = true;
	_vte->_position_estimator_running = false;

	ASSERT_TRUE(_vte->_vte_orientation.init());
	vte::SensorFusionMaskU aid_mask{};
	aid_mask.flags.use_vision_pos = 1;
	_vte->_vte_orientation.setVteAidMask(aid_mask.value);

	const hrt_abstime timestamp = vte_test::nowUs();
	uORB::Publication<fiducial_marker_yaw_report_s> vision_yaw_pub{ORB_ID(fiducial_marker_yaw_report)};
	ASSERT_TRUE(vte_test::publishVisionYaw(vision_yaw_pub, 0.2f, 0.01f, timestamp));

	// WHEN: The estimator times out after receiving an initial update.
	_vte->_vte_orientation.update();
	_vte->_vte_orientation.setVteTimeout(1_ms);

	vte_test::advanceMicroseconds(2_ms);

	// THEN: Timeout handling resets and re-arms the orientation estimator in the same call.
	ASSERT_TRUE(_vte->_vte_orientation.timedOut());
	_vte->restartTimedOutEstimators();
	EXPECT_TRUE(_vte->_orientation_estimator_running);
	EXPECT_FALSE(_vte->_vte_orientation.timedOut());
}

// WHY: Estimator updates should tolerate missing local pose data.
// WHAT: Provide attitude and acceleration only and ensure update proceeds.
TEST_F(VisionTargetEstTest, UpdateEstimatorsHandlesMissingLocalPose)
{
	// GIVEN: Position estimation is running with attitude and acceleration data only.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;

	const hrt_abstime timestamp = vte_test::nowUs();
	publishAttitude(vte_test::identityQuat(), timestamp);
	publishAcceleration(matrix::Vector3f(0.1f, -0.2f, 9.5f), timestamp);

	_vte->_last_update_pos = timestamp - (VisionTargetEstTestable::kPosUpdatePeriodUs + 1_ms);
	_vte->_last_acc_reset = timestamp;

	// WHEN: The estimators are updated without local pose input.
	_vte->updateEstimators();

	// THEN: A new estimator input message is still published.
	ASSERT_TRUE(_vte_input_sub->update());
}

// WHY: Stale acceleration must not be mixed into the downsample accumulator.
// WHAT: Publish stale accel data and verify the accumulator does not grow.
TEST_F(VisionTargetEstTest, UpdateEstimatorsDropsStaleAcceleration)
{
	// GIVEN: The estimator is running with a fresh accumulator.
	_vte->_vte_position_enabled = true;
	_vte->_position_estimator_running = true;
	_vte->_acc_sample_count = 0;
	_vte->_vehicle_acc_ned_sum.setAll(0.f);
	_vte->_last_acc_reset = vte_test::nowUs();

	const hrt_abstime timestamp = vte_test::nowUs();
	publishAttitude(vte_test::identityQuat(), timestamp);

	const hrt_abstime stale_timestamp =
		timestamp - (VisionTargetEstTestable::kAccUpdatedTimeoutUs + 1_ms);
	publishAcceleration(matrix::Vector3f(0.1f, -0.2f, 9.5f), stale_timestamp);

	// WHEN: Estimator updates run with stale acceleration input.
	_vte->updateEstimators();

	// THEN: Stale samples are not accumulated.
	EXPECT_EQ(_vte->_acc_sample_count, 0u);
	expectVectorNear(_vte->_vehicle_acc_ned_sum, matrix::Vector3f{});
}

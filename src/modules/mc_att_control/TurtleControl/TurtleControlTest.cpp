// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "TurtleControl.hpp"

#include <gtest/gtest.h>
#include <parameters/param.h>
#include <px4_platform_common/time.h>

#include <algorithm>
#include <cmath>
#include <memory>

using namespace time_literals;

class TurtleControlTestParams : public ModuleParams
{
public:
	TurtleControlTestParams() : ModuleParams(nullptr) {}
	void refresh() { updateParams(); }
};

class TurtleControlTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite() { hrt_init(); }

	template<typename T> void setParameter(const char *name, T value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	void SetUp() override
	{
		param_control_autosave(false);
		setParameter("TRTL_EN", int32_t{1});
		setParameter("TRTL_MAX", 0.3f);
		setParameter("TRTL_TIME", 1.f);

		resetController();
	}

	void resetController()
	{
		_controller.reset();
		_controller = std::make_unique<TurtleControl>(&_params);
		_selected = false;
		_vehicle = {};
		_vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_TURTLE;
		_vehicle.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
		_manual = {};
		_manual.valid = true;
		_manual.data_source = manual_control_setpoint_s::SOURCE_RC;
		_manual.throttle = -1.f;
		_attitude = {};
		_attitude.q[1] = 1.f;
		_allocation = {};
		_allocation.turtle_mode_active = true;
		_allocation.turtle_geometry_valid = true;
		_direction = {};
		_direction.supported_motor_mask = 0xf;
		_direction.state = motor_direction_status_s::STATE_IDLE;
		_request_sub.copy(&_request);
		turtle_control_s control{};
		_control_sub.copy(&control);
	}

	turtle_control_s step(bool fresh_input = true, bool fresh_direction = true, bool fresh_allocation = true,
			      uint64_t status_age = 0, uint64_t input_age = 0, uint64_t publication_age = 0)
	{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
		const auto next = std::max(hrt_absolute_time() + 20_ms, std::max(input_age, publication_age) + 20_ms);
		timespec time{};
		time.tv_sec = next / 1000000;
		time.tv_nsec = (next % 1000000) * 1000;
		px4_clock_settime(CLOCK_MONOTONIC, &time);
#endif
		const auto now = hrt_absolute_time();
		_vehicle.timestamp = now - status_age;
		_manual.timestamp = fresh_input ? now - publication_age : 0;
		_manual.timestamp_sample = fresh_input ? now - input_age : 0;
		_attitude.timestamp = now;
		_direction.timestamp = fresh_direction ? now : 0;
		_allocation.timestamp = fresh_allocation ? now : 0;
		_allocation_pub.publish(_allocation);
		_direction_pub.publish(_direction);
		const bool selected = _controller->update(_vehicle, _manual, _attitude);
		EXPECT_EQ(selected, _vehicle.nav_state == vehicle_status_s::NAVIGATION_STATE_TURTLE);
		turtle_control_s control{};
		EXPECT_EQ(_control_sub.update(&control), selected || _selected);
		_selected = selected;
		return control;
	}

	void startDriving(float thrust = 0.3f)
	{
		ASSERT_TRUE(step().ready_to_arm);
		ASSERT_FALSE(_request_sub.updated());
		_vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		expectStopped(step());
		ASSERT_TRUE(_request_sub.update(&_request));
		EXPECT_EQ(_request.timestamp_sample, _vehicle.timestamp);
		_direction.request_id = _request.request_id;
		_direction.motor_mask = 0xf;
		_direction.reversed_mask = 0xf;
		_direction.state = motor_direction_status_s::STATE_COMPLETE;
		expectStopped(step());
		_manual.roll = 1.f;
		expectDriving(step(), thrust);
		EXPECT_FALSE(_request_sub.updated());
	}

	static void expectStopped(const turtle_control_s &control)
	{
		EXPECT_FALSE(control.driving);
		EXPECT_FLOAT_EQ(control.roll, 0.f);
		EXPECT_FLOAT_EQ(control.pitch, 0.f);
		EXPECT_FLOAT_EQ(control.thrust, 0.f);
	}

	static void expectDriving(const turtle_control_s &control, float thrust = 0.3f)
	{
		EXPECT_TRUE(control.driving);
		EXPECT_FLOAT_EQ(control.roll, 1.f);
		EXPECT_FLOAT_EQ(control.pitch, 0.f);
		EXPECT_FLOAT_EQ(control.thrust, thrust);
		EXPECT_FALSE(control.disarm_requested);
	}

	TurtleControlTestParams _params;
	std::unique_ptr<TurtleControl> _controller;
	uORB::Publication<control_allocator_status_s> _allocation_pub{ORB_ID(control_allocator_status)};
	uORB::Publication<motor_direction_status_s> _direction_pub{ORB_ID(motor_direction_status)};
	uORB::Subscription _control_sub{ORB_ID(turtle_control)};
	uORB::Subscription _request_sub{ORB_ID(motor_direction_request)};
	control_allocator_status_s _allocation{};
	vehicle_status_s _vehicle{};
	manual_control_setpoint_s _manual{};
	vehicle_attitude_s _attitude{};
	motor_direction_status_s _direction{};
	motor_direction_request_s _request{};
	bool _selected{false};
};


TEST_F(TurtleControlTest, ReadinessRequiresSelectedModeAndValidInputs)
{
	EXPECT_TRUE(step().ready_to_arm);
	_vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	EXPECT_FALSE(step().ready_to_arm);
	_vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_TURTLE;
	_manual.roll = 0.5f;
	EXPECT_FALSE(step().ready_to_arm);
	_manual.roll = 0.f;
	_manual.throttle = 0.f;
	EXPECT_FALSE(step().ready_to_arm);
	_manual.throttle = -1.f;
	_manual.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	EXPECT_FALSE(step().ready_to_arm);
	_manual.data_source = manual_control_setpoint_s::SOURCE_RC;
	EXPECT_FALSE(step(false).ready_to_arm);
	EXPECT_FALSE(step(true, false).ready_to_arm);
	_attitude.q[0] = 1.f;
	_attitude.q[1] = 0.f;
	EXPECT_TRUE(step().ready_to_arm);
	_attitude.q[0] = NAN;
	EXPECT_FALSE(step().ready_to_arm);
	EXPECT_FALSE(_request_sub.updated());
}

TEST_F(TurtleControlTest, RecoveryCanStartUprightPartlyTiltedOrInverted)
{
	// Unit quaternions for upright, 60-degree tilt, and inverted attitudes.
	const float attitudes[][2] = {{1.f, 0.f}, {0.8660254f, 0.5f}, {0.f, 1.f}};

	for (const auto &attitude : attitudes) {
		SCOPED_TRACE(attitude[0]);
		resetController();
		_attitude.q[0] = attitude[0];
		_attitude.q[1] = attitude[1];
		ASSERT_NO_FATAL_FAILURE(startDriving());
	}
}

TEST_F(TurtleControlTest, ReverseRequestRequiresArmedRecoveryAndMatchingCompletion)
{
	_vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	_vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	step();
	EXPECT_FALSE(_request_sub.updated());
	_vehicle.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_TURTLE;
	step();
	EXPECT_FALSE(_request_sub.updated());
	_vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	const auto control = step(true, true, true, 1_us);
	ASSERT_TRUE(_request_sub.update(&_request));
	EXPECT_EQ(_request.timestamp_sample, _vehicle.timestamp);
	EXPECT_GT(_request.timestamp, _vehicle.timestamp);
	EXPECT_EQ(_request.motor_mask, 0xf);
	EXPECT_EQ(_request.request_id, control.direction_request_id);
	expectStopped(control);
	step();
	EXPECT_FALSE(_request_sub.updated());
	_direction.state = motor_direction_status_s::STATE_COMPLETE;
	_direction.request_id = _request.request_id + 1;
	_direction.motor_mask = 0xf;
	_direction.reversed_mask = 0xf;
	_manual.roll = 1.f;
	expectStopped(step());
	_direction.request_id = _request.request_id;
	expectStopped(step()); // Sticks must be centered after reversal completes.
	_manual.roll = 0.f;
	expectStopped(step());
	_manual.roll = 1.f;
	expectDriving(step());
}

TEST_F(TurtleControlTest, InputFaultsStopDriveAndRequestDisarm)
{
	for (unsigned fault = 0; fault < 6; ++fault) {
		SCOPED_TRACE(fault);
		resetController();
		ASSERT_NO_FATAL_FAILURE(startDriving());
		const uint32_t reverse_id = _request.request_id;

		if (fault == 0) {
			_manual.throttle = 0.f;

		} else if (fault == 1) {
			_manual.valid = false;

		} else if (fault == 2) {
			_manual.roll = NAN;

		} else if (fault == 3) {
			_manual.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;

		} else if (fault == 5) {
			_attitude.q[0] = NAN;
		}

		const auto control = step(fault != 4);
		expectStopped(control);
		EXPECT_TRUE(control.disarm_requested);
		EXPECT_EQ(control.direction_request_id, reverse_id);
		EXPECT_FALSE(_request_sub.updated());
		_manual.valid = true;
		_manual.data_source = manual_control_setpoint_s::SOURCE_RC;
		_manual.throttle = -1.f;
		_manual.roll = 0.f;
		_attitude.q[0] = 0.f;
		_attitude.q[1] = 1.f;
		EXPECT_TRUE(step().disarm_requested);
		EXPECT_FALSE(_request_sub.updated());
	}
}

TEST_F(TurtleControlTest, ValidManualInputToleratesObservedRcGaps)
{
	ASSERT_NO_FATAL_FAILURE(startDriving());
	// The flight logs contain 106-150 ms gaps, shorter than the normal RC-loss timeout.
	expectDriving(step(true, true, true, 0, 150_ms, 150_ms));
	// ManualControl can republish an older sample while it remains valid.
	expectDriving(step(true, true, true, 0, 200_ms));
	_manual.valid = false;
	const auto lost = step();
	expectStopped(lost);
	EXPECT_TRUE(lost.disarm_requested);
}

TEST_F(TurtleControlTest, StalledManualControlStopsDriveAndRequestsDisarm)
{
	ASSERT_NO_FATAL_FAILURE(startDriving());
	// A producer stall must expire even if its last message still says valid.
	const auto stale = step(true, true, true, 0, 350_ms, 350_ms);
	expectStopped(stale);
	EXPECT_TRUE(stale.disarm_requested);
	EXPECT_TRUE(step().disarm_requested);
}

TEST_F(TurtleControlTest, RightingDoesNotStopDriveButCenterAndDisarmDo)
{
	ASSERT_NO_FATAL_FAILURE(startDriving());
	_attitude.q[0] = 1.f;
	_attitude.q[1] = 0.f;
	expectDriving(step());
	_manual.roll = 0.f;
	expectStopped(step());
	_manual.roll = 1.f;
	expectDriving(step());
	EXPECT_FALSE(_request_sub.updated());
	_vehicle.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	const auto disarmed = step();
	expectStopped(disarmed);
	EXPECT_FALSE(disarmed.disarm_requested);
	EXPECT_EQ(disarmed.direction_request_id, _request.request_id);
	EXPECT_FALSE(_request_sub.updated());
}

TEST_F(TurtleControlTest, ReadinessAndReversalRequireCurrentAllocatorGeometry)
{
	_allocation.turtle_geometry_valid = false;
	EXPECT_FALSE(step().ready_to_arm);
	_allocation.turtle_geometry_valid = true;
	_allocation.turtle_mode_active = false;
	EXPECT_FALSE(step().ready_to_arm);
	_allocation.turtle_mode_active = true;
	EXPECT_FALSE(step(true, true, false).ready_to_arm);

	_vehicle.nav_state_timestamp = hrt_absolute_time() + 1_s;
	EXPECT_FALSE(step().ready_to_arm);
	_vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	const auto control = step();
	expectStopped(control);
	EXPECT_TRUE(control.disarm_requested);
	EXPECT_FALSE(_request_sub.updated());
}

TEST_F(TurtleControlTest, LosingAllocatorGeometryStopsDriveAndRequestsDisarm)
{
	ASSERT_NO_FATAL_FAILURE(startDriving());
	_allocation.turtle_geometry_valid = false;
	const auto control = step();
	expectStopped(control);
	EXPECT_TRUE(control.disarm_requested);
	EXPECT_EQ(control.direction_request_id, _request.request_id);
	EXPECT_FALSE(_request_sub.updated());
}

TEST_F(TurtleControlTest, ParameterUpdateWaitsForDisarmAndDirectionRestoration)
{
	ASSERT_NO_FATAL_FAILURE(startDriving());
	setParameter("TRTL_MAX", 0.6f);
	_params.refresh();
	expectDriving(step());

	_vehicle.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_manual.roll = 0.f;
	expectStopped(step());
	EXPECT_FALSE(step().ready_to_arm);
	_direction.reversed_mask = 0;
	_direction.state = motor_direction_status_s::STATE_COMPLETE;
	ASSERT_NO_FATAL_FAILURE(startDriving(0.6f));
}

TEST_F(TurtleControlTest, LeavingModeDisablesOutputAndRequestsDisarm)
{
	ASSERT_NO_FATAL_FAILURE(startDriving());
	_vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	const auto control = step();
	expectStopped(control);
	EXPECT_FALSE(control.enabled);
	EXPECT_TRUE(control.disarm_requested);
	EXPECT_EQ(control.direction_request_id, _request.request_id);
	EXPECT_FALSE(_request_sub.updated());
	step();
	EXPECT_FALSE(_request_sub.updated());

	_vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_TURTLE;
	_manual.roll = 0.f;
	_direction.reversed_mask = 0;
	const auto reentered = step();
	expectStopped(reentered);
	EXPECT_TRUE(reentered.disarm_requested);
	EXPECT_FALSE(_request_sub.updated());
}

TEST_F(TurtleControlTest, ShutdownStopsOutputAndRequestsDisarm)
{
	ASSERT_NO_FATAL_FAILURE(startDriving());
	_controller.reset();
	turtle_control_s control{};
	ASSERT_TRUE(_control_sub.update(&control));
	expectStopped(control);
	EXPECT_FALSE(control.enabled);
	EXPECT_TRUE(control.disarm_requested);
	EXPECT_EQ(control.direction_request_id, _request.request_id);
	EXPECT_FALSE(_request_sub.updated());
}

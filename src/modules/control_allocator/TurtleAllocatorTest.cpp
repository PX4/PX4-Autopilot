// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>

namespace
{
// Stop the real worker before ControlAllocator's destructor releases its state.
class TestAllocator : public ControlAllocator
{
public:
	void quiesce()
	{
		ScheduleClear();
		Deinit();
		ScheduleClear(); // Run() may have installed a backup before Deinit() finished waiting.
	}
};

bool stopped(const actuator_motors_s &motors)
{
	return std::all_of(std::begin(motors.control), std::end(motors.control), [](float value) {
		return std::isnan(value);
	});
}
}

class TurtleAllocatorTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		hrt_init();
		hrt_work_queue_init();
	}

	void SetUp() override
	{
		param_reset_all();
		const int32_t rotor_count = 4;
		ASSERT_EQ(param_set_no_notification(param_find("CA_ROTOR_COUNT"), &rotor_count), 0);
		const float positions[4][2] {{1.f, 1.f}, {-1.f, -1.f}, {1.f, -1.f}, {-1.f, 1.f}};

		for (unsigned i = 0; i < 4; ++i) {
			char name[17];
			snprintf(name, sizeof(name), "CA_ROTOR%u_PX", i);
			ASSERT_EQ(param_set_no_notification(param_find(name), &positions[i][0]), 0);
			snprintf(name, sizeof(name), "CA_ROTOR%u_PY", i);
			ASSERT_EQ(param_set_no_notification(param_find(name), &positions[i][1]), 0);
		}

		// Clear retained uORB samples between fixtures in the same test process.
		_control_pub.publish(turtle_control_s{});
		_direction_pub.publish(motor_direction_status_s{});
		_torque_pub.publish(vehicle_torque_setpoint_s{});
		_thrust_pub.publish(vehicle_thrust_setpoint_s{});
		selectMode(vehicle_status_s::NAVIGATION_STATE_ALTCTL, false);
		ASSERT_EQ(px4::WorkQueueManagerStart(), 0);
		_allocator = std::make_unique<TestAllocator>();
	}

	void TearDown() override
	{
		if (_allocator) {
			_allocator->quiesce();
			_allocator.reset();
			px4::WorkQueueManagerStop();
		}
	}

	void runOnce()
	{
		_allocator->Run();
		_allocator->ScheduleClear();
	}

	void startRuntime() { ASSERT_TRUE(_allocator->init()); }

	void publishMode(bool enabled)
	{
		vehicle_control_mode_s mode{};
		mode.timestamp = hrt_absolute_time();
		mode.flag_control_allocation_enabled = enabled;
		_control_mode_pub.publish(mode);
	}

	void publishStatus(uint8_t nav_state, bool armed = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		status.armed_time = armed ? status.timestamp : 0;
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.nav_state = nav_state;
		_vehicle_status_pub.publish(status);
	}

	void selectMode(uint8_t nav_state, bool allocation_enabled)
	{
		publishMode(allocation_enabled);
		publishStatus(nav_state);
	}

	template<typename Message, typename Predicate>
	bool waitFor(uORB::Subscription &sub, Message &message, Predicate matches)
	{
		const hrt_abstime started = hrt_absolute_time();

		// Generous host scheduling allowance; this is not a flight timing guarantee.
		while (hrt_elapsed_time(&started) < 1000000) {
			if (sub.update(&message) && matches(message)) {
				return true;
			}

			px4_usleep(1000);
		}

		return false;
	}

	uint64_t publishFlightSetpoints(bool torque = true, bool thrust = true)
	{
		const hrt_abstime now = hrt_absolute_time();

		if (thrust) {
			vehicle_thrust_setpoint_s sample{};
			sample.timestamp = sample.timestamp_sample = now;
			sample.xyz[2] = -0.2f;
			_thrust_pub.publish(sample);
		}

		if (torque) {
			vehicle_torque_setpoint_s sample{};
			sample.timestamp = sample.timestamp_sample = now;
			_torque_pub.publish(sample);
		}

		return now;
	}

	uint64_t publishRecovery(float roll = 1.f)
	{
		motor_direction_status_s direction{};
		direction.timestamp = hrt_absolute_time();
		direction.request_id = 42;
		direction.state = motor_direction_status_s::STATE_COMPLETE;
		direction.supported_motor_mask = direction.motor_mask = direction.reversed_mask = TurtleControlGate::MOTOR_MASK;
		_direction_pub.publish(direction);

		turtle_control_s control{};
		control.timestamp = control.timestamp_sample = hrt_absolute_time();
		control.direction_request_id = direction.request_id;
		control.enabled = control.driving = true;
		control.roll = roll;
		control.thrust = 0.4f;
		_control_pub.publish(control);
		return control.timestamp_sample;
	}

	std::unique_ptr<TestAllocator> _allocator;
	uORB::Publication<vehicle_control_mode_s> _control_mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<turtle_control_s> _control_pub{ORB_ID(turtle_control)};
	uORB::Publication<motor_direction_status_s> _direction_pub{ORB_ID(motor_direction_status)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
};

TEST_F(TurtleAllocatorTest, DirectActuatorModeOwnsMotorTopicAfterDisarmedTurtleExit)
{
	uORB::Subscription motors_sub{ORB_ID(actuator_motors)};
	actuator_motors_s motors{};
	selectMode(vehicle_status_s::NAVIGATION_STATE_TURTLE, true);
	runOnce();
	ASSERT_TRUE(motors_sub.update(&motors));
	EXPECT_TRUE(stopped(motors));

	selectMode(vehicle_status_s::NAVIGATION_STATE_OFFBOARD, false);
	uORB::Publication<actuator_motors_s> direct_pub{ORB_ID(actuator_motors)};
	actuator_motors_s direct{};
	direct.timestamp = hrt_absolute_time();
	direct.control[0] = 0.5f;
	ASSERT_TRUE(direct_pub.publish(direct));

	runOnce();
	ASSERT_TRUE(motors_sub.update(&motors));
	EXPECT_FLOAT_EQ(motors.control[0], direct.control[0]);

	selectMode(vehicle_status_s::NAVIGATION_STATE_ALTCTL, true);
	runOnce();
	ASSERT_TRUE(motors_sub.update(&motors));
	EXPECT_TRUE(stopped(motors));
	EXPECT_EQ(motors.timestamp_sample, 0u);

	const uint64_t sample = publishFlightSetpoints();
	runOnce();
	ASSERT_TRUE(motors_sub.update(&motors));
	EXPECT_EQ(motors.timestamp_sample, sample);
}

TEST_F(TurtleAllocatorTest, StatusTrafficDoesNotDriveNormalMotorPublication)
{
	uORB::Subscription motors_sub{ORB_ID(actuator_motors)};
	actuator_motors_s motors{};
	motors_sub.update(&motors);
	selectMode(vehicle_status_s::NAVIGATION_STATE_ALTCTL, true);
	startRuntime();
	ASSERT_TRUE(waitFor(motors_sub, motors, [](const auto &) { return true; }));
	const auto initial_generation = motors_sub.get_last_generation();
	const hrt_abstime started = hrt_absolute_time();

	// Status/control-mode traffic must not add motor-output callbacks to the 50 ms backup.
	for (unsigned i = 0; i < 100; ++i) {
		selectMode(vehicle_status_s::NAVIGATION_STATE_ALTCTL, true);
		px4_usleep(1000);
	}

	motors_sub.update(&motors);
	const auto publications = motors_sub.get_last_generation() - initial_generation;
	const uint64_t backup_budget = hrt_elapsed_time(&started) / 50000 + 3;
	EXPECT_GT(publications, 0u);
	EXPECT_LE(publications, backup_budget);

	const uint64_t sample = publishFlightSetpoints();
	ASSERT_TRUE(waitFor(motors_sub, motors, [sample](const auto & m) { return m.timestamp_sample == sample; }));
}

TEST_F(TurtleAllocatorTest, BackupObservesEntryAndDisarmedExitWithoutControllerMessages)
{
	uORB::Subscription motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription status_sub{ORB_ID(control_allocator_status)};
	actuator_motors_s motors{};
	control_allocator_status_s status{};
	motors_sub.update(&motors);
	status_sub.update(&status);
	publishStatus(vehicle_status_s::NAVIGATION_STATE_TURTLE);
	startRuntime();
	ASSERT_TRUE(waitFor(status_sub, status, [](const auto & s) { return s.turtle_mode_active; }));
	EXPECT_TRUE(status.turtle_geometry_valid);
	ASSERT_TRUE(waitFor(motors_sub, motors, stopped));

	// Allocation permission arrives after vehicle_status; neither message has an output callback.
	publishMode(true);
	ASSERT_TRUE(waitFor(motors_sub, motors, stopped));
	publishStatus(vehicle_status_s::NAVIGATION_STATE_ALTCTL);
	const hrt_abstime exited = hrt_absolute_time();
	ASSERT_TRUE(waitFor(status_sub, status, [exited](const auto & s) {
		return s.timestamp >= exited && !s.turtle_mode_active;
	}));
	ASSERT_TRUE(waitFor(motors_sub, motors, [exited](const auto & m) { return m.timestamp >= exited && stopped(m); }));

	const uint64_t torque_sample = publishFlightSetpoints(true, false);
	ASSERT_TRUE(waitFor(motors_sub, motors, [torque_sample](const auto & m) {
		return m.timestamp >= torque_sample && stopped(m);
	}));
	publishFlightSetpoints(false, true);
	ASSERT_TRUE(waitFor(motors_sub, motors, [torque_sample](const auto & m) {
		return m.timestamp_sample == torque_sample && !stopped(m);
	}));
}

TEST_F(TurtleAllocatorTest, EntryAfterAllocationPermissionUsesBackup)
{
	uORB::Subscription motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription status_sub{ORB_ID(control_allocator_status)};
	actuator_motors_s motors{};
	control_allocator_status_s status{};
	motors_sub.update(&motors);
	status_sub.update(&status);
	publishMode(true);
	startRuntime();
	ASSERT_TRUE(waitFor(motors_sub, motors, [](const auto &) { return true; }));
	publishStatus(vehicle_status_s::NAVIGATION_STATE_TURTLE);
	ASSERT_TRUE(waitFor(status_sub, status, [](const auto & s) { return s.turtle_mode_active; }));
	EXPECT_TRUE(status.turtle_geometry_valid);
	ASSERT_TRUE(waitFor(motors_sub, motors, stopped));
}

TEST_F(TurtleAllocatorTest, RecoveryUpdatesDriveAndBackupStopsStaleControl)
{
	uORB::Subscription motors_sub{ORB_ID(actuator_motors)};
	actuator_motors_s motors{};
	motors_sub.update(&motors);
	selectMode(vehicle_status_s::NAVIGATION_STATE_TURTLE, true);
	startRuntime();
	ASSERT_TRUE(waitFor(motors_sub, motors, stopped));
	publishStatus(vehicle_status_s::NAVIGATION_STATE_TURTLE, true);
	publishRecovery();
	ASSERT_TRUE(waitFor(motors_sub, motors, [](const auto & m) { return m.control[0] > 0.f; }));
	EXPECT_FLOAT_EQ(motors.control[0], 0.4f);
	EXPECT_FLOAT_EQ(motors.control[3], 0.4f);
	EXPECT_TRUE(std::isnan(motors.control[1]));
	EXPECT_TRUE(std::isnan(motors.control[2]));
	// No further control/torque/status message: backup alone must stop stale recovery.
	ASSERT_TRUE(waitFor(motors_sub, motors, stopped));
}

TEST_F(TurtleAllocatorTest, LiveRecoveryHonorsNeutralInputDisarmAndAllocationRevocation)
{
	uORB::Subscription motors_sub{ORB_ID(actuator_motors)};
	actuator_motors_s motors{};
	motors_sub.update(&motors);
	selectMode(vehicle_status_s::NAVIGATION_STATE_TURTLE, true);
	startRuntime();
	ASSERT_TRUE(waitFor(motors_sub, motors, stopped));
	publishStatus(vehicle_status_s::NAVIGATION_STATE_TURTLE, true);
	publishRecovery();
	ASSERT_TRUE(waitFor(motors_sub, motors, [](const auto & m) { return m.control[0] > 0.f; }));

	uint64_t sample = publishRecovery(0.f);
	ASSERT_TRUE(waitFor(motors_sub, motors, [sample](const auto & m) {
		return m.timestamp_sample == sample && stopped(m);
	}));
	EXPECT_LE(motors.timestamp - sample, TurtleControlGate::CONTROL_TIMEOUT_US);

	publishRecovery();
	ASSERT_TRUE(waitFor(motors_sub, motors, [](const auto & m) { return m.control[0] > 0.f; }));
	publishStatus(vehicle_status_s::NAVIGATION_STATE_TURTLE);
	sample = publishRecovery();
	ASSERT_TRUE(waitFor(motors_sub, motors, [sample](const auto & m) {
		return m.timestamp_sample == sample && stopped(m);
	}));
	EXPECT_LE(motors.timestamp - sample, TurtleControlGate::CONTROL_TIMEOUT_US);

	publishStatus(vehicle_status_s::NAVIGATION_STATE_TURTLE, true);
	publishRecovery();
	ASSERT_TRUE(waitFor(motors_sub, motors, [](const auto & m) { return m.control[0] > 0.f; }));
	publishMode(false);
	sample = publishRecovery();
	ASSERT_TRUE(waitFor(motors_sub, motors, [sample](const auto & m) {
		return m.timestamp_sample == sample && stopped(m);
	}));
	EXPECT_LE(motors.timestamp - sample, TurtleControlGate::CONTROL_TIMEOUT_US);
}

TEST_F(TurtleAllocatorTest, BackupPreservesDirectActuatorOwnershipAfterExit)
{
	uORB::Subscription motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription status_sub{ORB_ID(control_allocator_status)};
	actuator_motors_s motors{};
	control_allocator_status_s status{};
	motors_sub.update(&motors);
	status_sub.update(&status);
	selectMode(vehicle_status_s::NAVIGATION_STATE_TURTLE, true);
	startRuntime();
	ASSERT_TRUE(waitFor(motors_sub, motors, stopped));
	selectMode(vehicle_status_s::NAVIGATION_STATE_OFFBOARD, false);
	const hrt_abstime exited = hrt_absolute_time();
	ASSERT_TRUE(waitFor(status_sub, status, [exited](const auto & s) {
		return s.timestamp >= exited && !s.turtle_mode_active;
	}));
	uORB::Publication<actuator_motors_s> direct_pub{ORB_ID(actuator_motors)};
	actuator_motors_s direct{};
	direct.timestamp = hrt_absolute_time();
	direct.control[0] = 0.5f;
	ASSERT_TRUE(direct_pub.publish(direct));
	ASSERT_TRUE(motors_sub.update(&motors));
	const auto direct_generation = motors_sub.get_last_generation();

	// Observe real backup runs after the handoff, rather than just sleeping through them.
	for (unsigned i = 0; i < 3; ++i) {
		const uint64_t previous_status = status.timestamp;
		ASSERT_TRUE(waitFor(status_sub, status, [previous_status](const auto & s) {
			return s.timestamp > previous_status && !s.turtle_mode_active;
		}));
		EXPECT_FALSE(motors_sub.update(&motors));
		EXPECT_EQ(motors_sub.get_last_generation(), direct_generation);
	}

	EXPECT_FLOAT_EQ(motors.control[0], direct.control[0]);
}

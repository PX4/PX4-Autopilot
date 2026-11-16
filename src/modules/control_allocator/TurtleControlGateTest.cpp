// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "TurtleControlGate.hpp"
#include <gtest/gtest.h>

#include <utility>

namespace
{
constexpr uint64_t NOW = 1000000;

struct Inputs {
	turtle_control_s control{};
	motor_direction_status_s direction{};

	Inputs()
	{
		control.timestamp = NOW;
		control.timestamp_sample = NOW;
		control.enabled = true;
		control.direction_request_id = 42;
		control.driving = true;
		control.roll = 1.f;
		control.thrust = 0.4f;

		direction.timestamp = NOW;
		direction.request_id = control.direction_request_id;
		direction.state = motor_direction_status_s::STATE_COMPLETE;
		direction.supported_motor_mask = TurtleControlGate::MOTOR_MASK;
		direction.motor_mask = TurtleControlGate::MOTOR_MASK;
		direction.reversed_mask = TurtleControlGate::MOTOR_MASK;
	}
};

TurtleMixing::Geometry quad()
{
	TurtleMixing::Geometry geometry{};
	geometry.num_rotors = TurtleMixing::MOTOR_COUNT;
	geometry.rotors[0].position = {1.f, 1.f, 0.f};
	geometry.rotors[1].position = {-1.f, -1.f, 0.f};
	geometry.rotors[2].position = {1.f, -1.f, 0.f};
	geometry.rotors[3].position = {-1.f, 1.f, 0.f};

	for (unsigned i = 0; i < TurtleMixing::MOTOR_COUNT; ++i) {
		geometry.rotors[i].axis = {0.f, 0.f, -1.f};
		geometry.rotors[i].thrust_coef = 1.f;
		geometry.rotors[i].tilt_index = -1;
	}

	return geometry;
}

TurtleControlGate configuredGate()
{
	const auto geometry = quad();
	TurtleControlGate gate;
	gate.configureGeometry(&geometry);
	return gate;
}

void expectStopped(const float (&output)[actuator_motors_s::NUM_CONTROLS])
{
	for (float value : output) {
		EXPECT_TRUE(std::isnan(value));
	}
}
}

TEST(TurtleControlGate, RequiresSelectedModeArmingAndMatchingDirection)
{
	auto gate = configuredGate();
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
	gate.setSelected(true, NOW, 0);
	EXPECT_FALSE(gate.mixTurtleControls(NOW, false, input.control, input.direction, output));
	expectStopped(output);
	EXPECT_TRUE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	EXPECT_FLOAT_EQ(output[0], 0.4f);
	EXPECT_FLOAT_EQ(output[3], 0.4f);
	EXPECT_TRUE(std::isnan(output[1]));
	input.direction.request_id++;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
}

TEST(TurtleControlGate, RejectsPendingPartialAndUnsupportedDirection)
{
	auto gate = configuredGate();
	gate.setSelected(true, NOW, 0);
	float output[actuator_motors_s::NUM_CONTROLS];

	for (auto state : {
		     motor_direction_status_s::STATE_IDLE, motor_direction_status_s::STATE_PENDING,
		     motor_direction_status_s::STATE_REJECTED
	     }) {
		Inputs input;
		input.direction.state = state;
		EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
		expectStopped(output);
	}

	Inputs input;
	input.direction.reversed_mask = 0x3;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
	input = Inputs{};
	input.direction.supported_motor_mask = 0x7;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
	input = Inputs{};
	input.direction.motor_mask = 0x7;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
}

TEST(TurtleControlGate, StaleOrFutureMessagesStopOutput)
{
	auto gate = configuredGate();
	gate.setSelected(true, NOW - 1, 0);
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	EXPECT_TRUE(gate.mixTurtleControls(NOW + TurtleControlGate::CONTROL_TIMEOUT_US, true,
					   input.control, input.direction, output));
	EXPECT_FALSE(gate.mixTurtleControls(NOW + TurtleControlGate::CONTROL_TIMEOUT_US + 1, true,
					    input.control, input.direction, output));
	expectStopped(output);
	input = Inputs{};
	input.direction.timestamp = NOW - TurtleControlGate::DIRECTION_TIMEOUT_US - 1;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
	input = Inputs{};
	input.control.timestamp = NOW + 1;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
	input = Inputs{};
	input.direction.timestamp = NOW + 1;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
}

TEST(TurtleControlGate, ControllerOwnsManualInputFreshness)
{
	auto gate = configuredGate();
	gate.setSelected(true, NOW, 0);
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	// Match the longest RC sample gap observed in the flight logs.
	input.control.timestamp_sample = NOW - 150000;
	EXPECT_TRUE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	EXPECT_FLOAT_EQ(output[0], 0.4f);
	EXPECT_FALSE(gate.mixTurtleControls(NOW + TurtleControlGate::CONTROL_TIMEOUT_US + 1, true,
					    input.control, input.direction, output));
	expectStopped(output);
}

TEST(TurtleControlGate, RejectsEarlierSessionAndStopRequests)
{
	auto gate = configuredGate();
	gate.setSelected(true, NOW, 0);
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	input.control.timestamp = NOW - 1;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
	input = Inputs{};
	input.control.disarm_requested = true;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
	input = Inputs{};
	input.control.enabled = false;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
}

TEST(TurtleControlGate, RequiresDriveIntentAndValidGeometry)
{
	auto gate = configuredGate();
	gate.setSelected(true, NOW, 0);
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	input.control.pitch = 1.f;
	ASSERT_TRUE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	EXPECT_FLOAT_EQ(output[0], 0.4f);
	EXPECT_TRUE(std::isnan(output[1]));
	EXPECT_TRUE(std::isnan(output[2]));
	EXPECT_TRUE(std::isnan(output[3]));

	input.control.driving = false;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);

	input.control.driving = true;
	gate.configureGeometry(nullptr);
	EXPECT_FALSE(gate.geometryValid());
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
}

TEST(TurtleControlGate, CenteredAndInvalidIntentStopAllMotors)
{
	auto gate = configuredGate();
	gate.setSelected(true, NOW, 0);
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	input.control.roll = 0.f;
	EXPECT_TRUE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);

	input = Inputs{};
	input.control.thrust = NAN;
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);
}

TEST(TurtleControlGate, GeometryRemappingStopsAnArmedVisitUntilDisarm)
{
	auto gate = configuredGate();
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	input.control.pitch = 1.f;
	gate.setSelected(true, NOW, NOW);
	ASSERT_TRUE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	EXPECT_FLOAT_EQ(output[0], 0.4f);

	auto geometry = quad();
	geometry.rotors[0].position(0) *= 2.f;
	gate.configureGeometry(&geometry);
	EXPECT_TRUE(gate.geometryValid());
	ASSERT_TRUE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	EXPECT_FLOAT_EQ(output[0], 0.4f);

	std::swap(geometry.rotors[0], geometry.rotors[1]);
	gate.configureGeometry(&geometry);
	EXPECT_FALSE(gate.geometryValid());
	EXPECT_FALSE(gate.mixTurtleControls(NOW, true, input.control, input.direction, output));
	expectStopped(output);

	gate.setSelected(true, NOW + 1, NOW);
	EXPECT_FALSE(gate.geometryValid());
	gate.setSelected(true, NOW + 2, 0);
	EXPECT_TRUE(gate.geometryValid());
	gate.setSelected(true, NOW + 3, NOW + 3);
	input.control.timestamp = NOW + 3;
	input.control.timestamp_sample = NOW + 3;
	input.control.direction_request_id++;
	input.direction.timestamp = NOW + 3;
	input.direction.request_id = input.control.direction_request_id;
	ASSERT_TRUE(gate.mixTurtleControls(NOW + 3, true, input.control, input.direction, output));
	EXPECT_TRUE(std::isnan(output[0]));
	EXPECT_FLOAT_EQ(output[1], 0.4f);
}

TEST(TurtleControlGate, DisarmedVisitNeedsNewFlightSetpointsWithoutDirectionPublisher)
{
	TurtleControlGate gate;
	const motor_direction_status_s no_direction{};
	EXPECT_TRUE(gate.allowNormal(NOW, no_direction));
	gate.setSelected(true, NOW, 0);
	EXPECT_FALSE(gate.allowNormal(NOW, no_direction));
	gate.observeTorque(NOW);
	gate.observeThrust(NOW);

	gate.setSelected(false, NOW + 1, 0);
	EXPECT_TRUE(gate.waitingForNormal());
	EXPECT_FALSE(gate.allowNormal(NOW + 1, no_direction));
	gate.observeTorque(NOW + 1);
	EXPECT_FALSE(gate.allowNormal(NOW + 1, no_direction));
	gate.observeThrust(NOW + 1);
	EXPECT_TRUE(gate.allowNormal(NOW + 1, no_direction));
	EXPECT_FALSE(gate.waitingForNormal());
}

TEST(TurtleControlGate, ArmedVisitNeedsRestorationAfterTheArmingEpoch)
{
	auto gate = configuredGate();
	Inputs input;
	const uint64_t armed_at = NOW + 1;
	const uint64_t exit_at = NOW + 2;
	gate.setSelected(true, NOW, 0);
	gate.setSelected(true, armed_at, armed_at);
	gate.setSelected(false, exit_at, 0);
	gate.observeTorque(exit_at);
	gate.observeThrust(exit_at);
	EXPECT_FALSE(gate.allowNormal(exit_at, {}));

	input.direction.timestamp = exit_at;
	input.direction.timestamp_disarmed = NOW;
	input.direction.reversed_mask = 0;
	input.direction.state = motor_direction_status_s::STATE_IDLE;
	EXPECT_FALSE(gate.allowNormal(exit_at, input.direction));

	input.direction.timestamp_disarmed = exit_at;
	input.direction.state = motor_direction_status_s::STATE_PENDING;
	EXPECT_FALSE(gate.allowNormal(exit_at, input.direction));
	input.direction.state = motor_direction_status_s::STATE_COMPLETE;
	input.direction.reversed_mask = TurtleControlGate::MOTOR_MASK;
	EXPECT_FALSE(gate.allowNormal(exit_at, input.direction));

	input.direction.reversed_mask = 0;
	const uint64_t restored_at = exit_at + TurtleControlGate::DIRECTION_TIMEOUT_US + 1;
	EXPECT_FALSE(gate.allowNormal(restored_at, input.direction));
	input.direction.timestamp = restored_at;
	EXPECT_TRUE(gate.allowNormal(restored_at, input.direction));
	EXPECT_FALSE(gate.waitingForNormal());
}

TEST(TurtleControlGate, DisarmedReentryKeepsPriorArmingEpochUntilRestoration)
{
	auto gate = configuredGate();
	const uint64_t armed_at = NOW + 1;
	gate.setSelected(true, NOW, 0);
	gate.setSelected(true, armed_at, armed_at);
	gate.setSelected(false, NOW + 2, 0);
	gate.setSelected(true, NOW + 3, 0);
	gate.setSelected(false, NOW + 4, 0);
	gate.observeTorque(NOW + 4);
	gate.observeThrust(NOW + 4);

	motor_direction_status_s direction{};
	direction.timestamp = NOW + 4;
	direction.timestamp_disarmed = NOW;
	EXPECT_FALSE(gate.allowNormal(NOW + 4, direction));
	direction.timestamp_disarmed = NOW + 4;
	EXPECT_TRUE(gate.allowNormal(NOW + 4, direction));
}

TEST(TurtleControlGate, ReentryDoesNotReplayOldRecoveryOutput)
{
	auto gate = configuredGate();
	Inputs input;
	float output[actuator_motors_s::NUM_CONTROLS];
	gate.setSelected(true, NOW, 0);
	gate.setSelected(false, NOW + 1, 0);
	gate.setSelected(true, NOW + 2, 0);
	EXPECT_FALSE(gate.mixTurtleControls(NOW + 2, true, input.control, input.direction, output));
	expectStopped(output);
}

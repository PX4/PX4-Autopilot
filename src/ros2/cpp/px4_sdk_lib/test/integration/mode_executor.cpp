/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>

#include "util.h"

#include <px4_sdk/components/mode_executor.h>
#include <px4_sdk/components/wait_for_fmu.h>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace px4_sdk;
using namespace std;
using namespace std::chrono_literals;

static const char *name = "Test Flight Mode";


class Tester : public ::testing::Test
{
public:
	static void SetUpTestSuite()
	{
		rclcpp::init(0, nullptr);
	}
	static void TearDownTestSuite()
	{
		rclcpp::shutdown();
	}
};

class FlightModeTest : public ModeBase
{
public:
	FlightModeTest(rclcpp::Node &node)
		: ModeBase(node, Settings{name, false}, ModeRequirements::autonomous())
	{
		setSetpointUpdateRate(30.f);
	}

	virtual ~FlightModeTest() = default;

	void onActivate() override
	{
		_activation_time = node().get_clock()->now();
		setpoints().configureSetpointsSync(SetpointSender::SetpointConfiguration{});
		++num_activations;
	}

	void checkArmingAndRunConditions(HealthAndArmingCheckReporter &reporter) override
	{
		++num_arming_check_updates;
	}

	void onDeactivate() override
	{
		++num_deactivations;
	}

	void updateSetpoint() override
	{
		++num_setpoint_updates;
		rclcpp::Time now = node().get_clock()->now();

		if (now - _activation_time > rclcpp::Duration::from_seconds(8)) {
			completed(Result::Success);
			return;
		}

		// Send some random setpoints
		float elapsed_s = (now - _activation_time).seconds();
		Eigen::Vector3f velocity{5.f, elapsed_s, 0.f };
		setpoints().sendTrajectorySetpoint(velocity);
	}

	int num_activations{0};
	int num_deactivations{0};
	int num_setpoint_updates{0};
	int num_arming_check_updates{0};
private:
	rclcpp::Time _activation_time{};
};

class ModeExecutorTest : public ModeExecutorBase
{
public:
	ModeExecutorTest(rclcpp::Node &node, FlightModeTest &owned_mode, bool activate_immediately)
		: ModeExecutorBase(node, ModeExecutorBase::Settings{activate_immediately}, owned_mode),
		  _node(node)
	{}

	enum class State {
		Reset = 0,
		WaitForArming = 1,
		Arming = 2,
		TakingOff = 3,
		MyMode = 4,
		RTL = 5,
		WaitUntilDisarmed = 6,
		Completed = 7,
	};

	void onActivate() override
	{
		++num_activations;
		runState(State::WaitForArming, Result::Success);
	}

	void onDeactivate(DeactivateReason reason) override
	{
		++num_deactivations;
	}

	void runState(State state, Result previous_result)
	{
		on_state_completed(state, previous_result);

		if (previous_result != Result::Success) {
			RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
			return;
		}

		RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

		switch (state) {
		case State::Reset:
			break;

		case State::WaitForArming:
			waitReadyToArm([this](Result result) { runState(State::Arming, result); });
			break;

		case State::Arming:
			arm([this](Result result) { runState(State::TakingOff, result); });
			break;

		case State::TakingOff:
			takeoff([this](Result result) { runState(State::MyMode, result); });
			break;

		case State::MyMode:
			scheduleMode(ownedMode().id(), [this](Result result) { runState(State::RTL, result); });
			break;

		case State::RTL:
			rtl([this](Result result) { runState(State::WaitUntilDisarmed, result); });
			break;

		case State::WaitUntilDisarmed:
			waitUntilDisarmed([this](Result result) { runState(State::Completed, result); });
			break;

		case State::Completed:
			on_completed();
			rclcpp::shutdown();
			break;
		}
	}

	int num_activations{0};
	int num_deactivations{0};

	std::function<void()> on_completed;
	std::function<void(State, Result)> on_state_completed;

private:
	rclcpp::Node &_node;
};


class TestExecutionAutonomous
{
public:
	TestExecutionAutonomous(rclcpp::Node &node)
		: _node(node) {}

	void run();
private:
	rclcpp::Node &_node;
	rclcpp::TimerBase::SharedPtr _test_timeout;

	std::unique_ptr<FlightModeTest> _mode;
	std::unique_ptr<ModeExecutorTest> _mode_executor;
};

void TestExecutionAutonomous::run()
{
	_test_timeout = _node.create_wall_timer(80s, [] {
		EXPECT_TRUE(false); // Timed out
		rclcpp::shutdown();
	});

	_mode = std::make_unique<FlightModeTest>(_node);
	_mode_executor = std::make_unique<ModeExecutorTest>(_node, *_mode.get(), true);


	// The executor is expected to be activated and then run through its states (successfully)

	_mode_executor->on_completed = [this]() {
		EXPECT_EQ(_mode_executor->num_activations, 1);
		EXPECT_EQ(_mode_executor->num_deactivations, 0);
		EXPECT_EQ(_mode->num_activations, 1);
		EXPECT_EQ(_mode->num_deactivations, 1);
		EXPECT_GT(_mode->num_setpoint_updates, 1);
		EXPECT_GT(_mode->num_arming_check_updates, 1);
	};
	_mode_executor->on_state_completed = [](ModeExecutorTest::State, Result result) {
		EXPECT_EQ(result, Result::Success);
	};

	ASSERT_TRUE(_mode_executor->doRegister());
}

TEST_F(Tester, run_executor_autonomous)
{
	auto test_node = initNode();
	ASSERT_TRUE(waitForFMU(*test_node, 10s));
	TestExecutionAutonomous test_execution{*test_node};
	test_execution.run();
	rclcpp::spin(test_node);
}


class TestExecutionInCharge
{
public:
	TestExecutionInCharge(rclcpp::Node &node)
		: _node(node), _vehicle_state(node) {}

	void run();
private:

	enum class State {
		WaitForArming,
		TryToArm,
		SwitchMode,
		Arming,
		WaitForTakeoff,
		WaitForRTL,
		WaitForDisarm,
	};

	State _state{State::WaitForArming};

	rclcpp::Node &_node;
	rclcpp::TimerBase::SharedPtr _test_timeout;

	std::unique_ptr<FlightModeTest> _mode;
	std::unique_ptr<ModeExecutorTest> _mode_executor;
	VehicleState _vehicle_state;
	bool _was_armed{false};
	bool _land_activated{false};
};

void TestExecutionInCharge::run()
{
	_test_timeout = _node.create_wall_timer(80s, [] {
		EXPECT_TRUE(false); // Timed out
		rclcpp::shutdown();
	});

	_mode = std::make_unique<FlightModeTest>(_node);
	_mode_executor = std::make_unique<ModeExecutorTest>(_node, *_mode.get(), false);


	// Testing steps:
	// - Wait until we can arm
	// - try to arm via executor -> must fail because it's not in charge
	// - switch into owned mode + arm
	// - when in takeoff, switch into owned mode: must trigger executor re-activation
	// - wait until in owned mode
	// - switch to descend mode -> executor gets deactivated
	// - wait until disarmed

	_vehicle_state.setOnVehicleStatusUpdate([this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
		bool armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

		if (_state == State::WaitForArming) {
			if (msg->pre_flight_checks_pass) {
				RCLCPP_INFO(_node.get_logger(), "Arming possible");
				_state = State::TryToArm;
				_mode_executor->arm([this](Result result) {
					ASSERT_EQ(result, Result::Rejected);
					RCLCPP_INFO(_node.get_logger(), "Arming rejected, switching mode");
					_state = State::SwitchMode;
					_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE, _mode->id());

					_vehicle_state.callbackOnModeSet([this]() {
						RCLCPP_INFO(_node.get_logger(), "Mode Activated, arming");
						_state = State::Arming;
						_mode_executor->arm([this](Result result) {
							ASSERT_EQ(result, Result::Success);
							RCLCPP_INFO(_node.get_logger(), "Armed");

							// Now the executor must take over and trigger a takeoff
							_state = State::WaitForTakeoff;
							_vehicle_state.callbackOnModeSet([this]() {
								RCLCPP_INFO(_node.get_logger(), "Takeoff Activated, switching to owned mode");
								_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE, _mode->id());
								_state = State::WaitForRTL;

							}, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF);

						});
					}, _mode->id());
				});
			}
		}

		if (_was_armed && !armed && _state == State::WaitForDisarm) {

			// Disarming, complete the test
			EXPECT_TRUE(_land_activated);

			EXPECT_EQ(_mode_executor->num_activations, 2);
			EXPECT_EQ(_mode_executor->num_deactivations, 2);
			EXPECT_EQ(_mode->num_activations, 1);
			EXPECT_EQ(_mode->num_deactivations, 1);
			EXPECT_GT(_mode->num_setpoint_updates, 1);
			EXPECT_GT(_mode->num_arming_check_updates, 1);

			rclcpp::shutdown();
		}

		_was_armed = armed;
	});


	_mode_executor->on_completed = []() {
		EXPECT_FALSE(true); // never completed
	};
	_mode_executor->on_state_completed = [this](ModeExecutorTest::State next_state, Result result) {

		if (_state == State::WaitForRTL && next_state == ModeExecutorTest::State::RTL) {
			EXPECT_EQ(result, Result::Success);
			EXPECT_EQ(_mode_executor->num_activations, 2);
			EXPECT_EQ(_mode_executor->num_deactivations, 1);

			// Wait until RTL actually activated (otherwise we can have a race condition with the previous command from the executor)
			_vehicle_state.callbackOnModeSet([this]() {
				RCLCPP_INFO(_node.get_logger(), "Activating Land");
				_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
							   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);
				_vehicle_state.callbackOnModeSet([this]() {
					RCLCPP_INFO(_node.get_logger(), "Landing activated");
					_land_activated = true;
				}, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);

				_state = State::WaitForDisarm;
			}, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL);

		} else if (_state == State::WaitForRTL && next_state == ModeExecutorTest::State::MyMode) {
			// We can get here twice, when switching into the owned mode and after re-activation

		} else if (_state == State::WaitForDisarm) {
			EXPECT_TRUE(result == Result::Deactivated || result == Result::Rejected); // Switching to land deactivated the executor

		} else {
			EXPECT_EQ(result, Result::Success);
		}
	};

	ASSERT_TRUE(_mode_executor->doRegister());
}

TEST_F(Tester, run_executor_in_charge)
{
	auto test_node = initNode();
	ASSERT_TRUE(waitForFMU(*test_node, 10s));
	TestExecutionInCharge test_execution{*test_node};
	test_execution.run();
	rclcpp::spin(test_node);
}

class TestExecutionFailsafe
{
public:
	TestExecutionFailsafe(rclcpp::Node &node)
		: _node(node), _vehicle_state(node) {}

	void run();
private:
	rclcpp::Node &_node;
	rclcpp::TimerBase::SharedPtr _test_timeout;

	std::unique_ptr<FlightModeTest> _mode;
	std::unique_ptr<ModeExecutorTest> _mode_executor;
	VehicleState _vehicle_state;
	bool _failsafe_triggered{false};
	bool _land_activated{false};
	bool _was_armed{false};
};

void TestExecutionFailsafe::run()
{
	_test_timeout = _node.create_wall_timer(80s, [] {
		EXPECT_TRUE(false); // Timed out
		rclcpp::shutdown();
	});

	_mode = std::make_unique<FlightModeTest>(_node);
	_mode_executor = std::make_unique<ModeExecutorTest>(_node, *_mode.get(), true);


	// Run the executor and trigger a failsafe (descend) while it's in the custom mode

	_mode_executor->on_completed = []() {
		EXPECT_FALSE(true); // never completed
	};
	_mode_executor->on_state_completed = [this](ModeExecutorTest::State next_state, Result result) {

		if (next_state == ModeExecutorTest::State::TakingOff) {

			_vehicle_state.callbackOnModeSet([this]() {
				RCLCPP_INFO(_node.get_logger(), "Owned mode started, triggering failsafe");
				_vehicle_state.setGPSFailure(true);
				_failsafe_triggered = true;
			}, _mode->id());

			EXPECT_EQ(result, Result::Success);

		} else if (next_state == ModeExecutorTest::State::RTL) {
			EXPECT_EQ(result, Result::Deactivated); // Expect the failsafe to have triggered in MyMode

			_vehicle_state.callbackOnModeSet([this]() {
				RCLCPP_INFO(_node.get_logger(), "Landing activated");
				_land_activated = true;
			}, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);


		} else {
			EXPECT_EQ(result, Result::Success);
		}
	};

	_vehicle_state.setOnVehicleStatusUpdate([this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
		bool armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

		if (_was_armed && !armed) {

			_vehicle_state.setGPSFailure(false);

			// Disarming, complete the test
			EXPECT_TRUE(_land_activated);
			EXPECT_TRUE(_failsafe_triggered);

			EXPECT_EQ(_mode_executor->num_activations, 1);
			EXPECT_EQ(_mode_executor->num_deactivations, 1);
			EXPECT_EQ(_mode->num_activations, 1);
			EXPECT_EQ(_mode->num_deactivations, 1);
			EXPECT_GT(_mode->num_setpoint_updates, 1);
			EXPECT_GT(_mode->num_arming_check_updates, 1);

			rclcpp::shutdown();
		}

		_was_armed = armed;
	});

	ASSERT_TRUE(_mode_executor->doRegister());
}

TEST_F(Tester, run_executor_failsafe)
{
	auto test_node = initNode();
	ASSERT_TRUE(waitForFMU(*test_node, 10s));
	TestExecutionFailsafe test_execution{*test_node};
	test_execution.run();
	rclcpp::spin(test_node);
}

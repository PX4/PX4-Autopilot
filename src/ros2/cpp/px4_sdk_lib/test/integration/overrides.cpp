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
		Landing = 4,
		Wait = 5,
		TakingOff2 = 6,
		WaitForFailsafe = 7,
		WaitForDisarming = 8,
	};

	void onActivate() override
	{
		++num_activations;
		runState(State::WaitForArming, Result::Success);
	}

	void onDeactivate(DeactivateReason reason) override
	{
		EXPECT_EQ(_state, State::WaitForDisarming);
		++num_deactivations;
	}

	void onFailsafeDeferred() override
	{
		++num_failsafe_deferred;
		EXPECT_EQ(_state, State::WaitForFailsafe);
		// Re-enable failsafes -> failsafe will trigger and executor deactivated
		deferFailsafesSync(false);
		runState(State::WaitForDisarming, Result::Success);
	}

	void runState(State state, Result previous_result)
	{
		on_state_completed(state, previous_result);

		if (previous_result != Result::Success) {
			RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
			return;
		}

		RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

		_state = state;

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
			takeoff([this](Result result) { runState(State::Landing, result); });
			break;

		case State::Landing:
			land([this](Result result) { runState(State::Wait, result); });
			break;

		case State::Wait:
			_wait_timer = _node.create_wall_timer(10s, [this] {
				_wait_timer.reset();
				EXPECT_TRUE(isArmed());
				runState(State::TakingOff2, Result::Success);
			});
			break;

		case State::TakingOff2:
			takeoff([this](Result result) {
				EXPECT_TRUE(deferFailsafesSync(true, 60));
				runState(State::WaitForFailsafe, result);
			});
			break;

		case State::WaitForFailsafe:
			// Nothing to do
			break;

		case State::WaitForDisarming:
			// Nothing to do
			break;

		}
	}

	State getState() const { return _state; }

	int num_activations{0};
	int num_deactivations{0};
	int num_failsafe_deferred{0};

	std::function<void()> on_completed;
	std::function<void(State, Result)> on_state_completed;

private:
	State _state{};
	rclcpp::Node &_node;
	rclcpp::TimerBase::SharedPtr _wait_timer;
};


class TestExecutionOverrides
{
public:
	TestExecutionOverrides(rclcpp::Node &node)
		: _node(node), _vehicle_state(node) {}

	void run();
private:
	rclcpp::Node &_node;
	rclcpp::TimerBase::SharedPtr _test_timeout;

	std::unique_ptr<FlightModeTest> _mode;
	std::unique_ptr<ModeExecutorTest> _mode_executor;
	VehicleState _vehicle_state;
	bool _was_armed{false};
};

void TestExecutionOverrides::run()
{
	_test_timeout = _node.create_wall_timer(80s, [] {
		EXPECT_TRUE(false); // Timed out
		rclcpp::shutdown();
	});

	_mode = std::make_unique<FlightModeTest>(_node);
	_mode_executor = std::make_unique<ModeExecutorTest>(_node, *_mode.get(), true);


	// Testing steps:
	// - disable auto-disarm
	// - register & activate executor
	// - takeoff
	// - land & ensure no auto-disarm triggered
	// - takeoff
	// - defer failsafes & trigger failsafe
	// - wait for callback & enable failsafes
	// - wait for failsafe triggering, landing and disarming

	_mode_executor->on_state_completed = [this](ModeExecutorTest::State next_state, Result result) {
		if (next_state == ModeExecutorTest::State::WaitForFailsafe) {
			EXPECT_EQ(result, Result::Success);
			// trigger failsafe (one that can be deferred)
			_vehicle_state.setForceLowBattery(true);

		} else {
			EXPECT_EQ(result, Result::Success);
		}
	};

	_vehicle_state.setOnVehicleStatusUpdate([this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
		bool armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

		if (_was_armed && !armed) {
			// Finish the test
			_vehicle_state.setForceLowBattery(false);

			EXPECT_EQ(_mode_executor->getState(), ModeExecutorTest::State::WaitForDisarming);
			EXPECT_EQ(_mode_executor->num_activations, 1);
			EXPECT_EQ(_mode_executor->num_deactivations, 1);
			EXPECT_EQ(_mode_executor->num_failsafe_deferred, 1);

			rclcpp::shutdown();
		}

		_was_armed = armed;
	});

	_mode_executor->configOverrides().controlAutoDisarm(false);
	ASSERT_TRUE(_mode_executor->doRegister());
}

TEST_F(Tester, run_executor_overrides)
{
	auto test_node = initNode();
	ASSERT_TRUE(waitForFMU(*test_node, 10s));
	TestExecutionOverrides test_execution{*test_node};
	test_execution.run();
	rclcpp::spin(test_node);
}


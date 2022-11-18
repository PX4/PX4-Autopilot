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

#include <px4_sdk/components/mode.h>
#include <px4_sdk/components/wait_for_fmu.h>

#include <rclcpp/rclcpp.hpp>

using namespace px4_sdk;
using namespace std;
using namespace std::chrono_literals;

static const char *name = "Test Descend";


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
		: ModeBase(node, Settings{name, true, ModeBase::ID_NAVIGATION_STATE_DESCEND}, ModeRequirements::none())
	{
		setSetpointUpdateRate(50.f);
	}

	virtual ~FlightModeTest() = default;

	void onActivate() override
	{
		setpoints().configureSetpointsSync(SetpointSender::SetpointConfiguration{});
		++num_activations;
	}

	void checkArmingAndRunConditions(HealthAndArmingCheckReporter &reporter) override
	{
		if (check_should_fail) {
			/* EVENT
			 */
			reporter.armingCheckFailureExt(events::ID("check_custom_mode_test_failure"),
						       events::Log::Error, "Custom check failed");
		}

		++num_arming_check_updates;
	}

	void onDeactivate() override
	{
		++num_deactivations;
	}

	void updateSetpoint() override
	{
		++num_setpoint_updates;

		// Send some random setpoints, make sure it stays in the air, we don't want it to land
		Eigen::Vector3f velocity{1.f, 0.f, -0.5f };
		setpoints().sendTrajectorySetpoint(velocity);
	}

	int num_activations{0};
	int num_deactivations{0};
	int num_setpoint_updates{0};
	int num_arming_check_updates{0};
	bool check_should_fail{false};
private:
};


class TestExecution
{
public:
	TestExecution(rclcpp::Node &node)
		: _node(node), _vehicle_state(node) {}

	void run();
private:

	enum class State {
		ActivatingLand,
		WaitForExternalMode,
		WaitForArming,
		WaitUntilInAir,
		WaitForCustomMode,
		WaitForHold,
		WaitForFailsafe,
		TerminateMode,
		WaitForDisarm,
	};

	State _state{State::ActivatingLand};

	rclcpp::Node &_node;
	rclcpp::TimerBase::SharedPtr _test_timeout;
	rclcpp::TimerBase::SharedPtr _testing_timer;

	std::unique_ptr<FlightModeTest> _mode;
	VehicleState _vehicle_state;
	bool _was_armed{false};
	uint8_t _current_nav_state{};
	int _num_pre_flight_checks_pass{0};
};

void TestExecution::run()
{
	_test_timeout = _node.create_wall_timer(80s, [] {
		EXPECT_TRUE(false); // Timed out
		rclcpp::shutdown();
	});

	_mode = std::make_unique<FlightModeTest>(_node);

	// Testing steps:
	// - switch to descend
	// - register mode, replace internal descend
	// - ensure fmu switched to external mode
	// - switch to takeoff
	// - wait for arming
	// - takeoff + wait
	// - trigger extra failing mode check
	// - try to switch into custom mode -> prevented
	// - clear extra check
	// - try to switch into custom mode -> works
	// - switch to hold
	// - trigger failsafe
	// - ensure switch to external descend
	// - terminate mode -> switch to internal
	// - wait for disarming


	// First, switch to Descend and wait for it
	RCLCPP_INFO(_node.get_logger(), "Activating Land");
	_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
				   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);
	_vehicle_state.callbackOnModeSet([this]() {
		_state = State::WaitForExternalMode;
		RCLCPP_INFO(_node.get_logger(), "Registering");
		ASSERT_TRUE(_mode->doRegister());

		// We expect to switch into our custom mode, as it replaces the currently selected one
		_vehicle_state.callbackOnModeSet([this]() {
			_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
						   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF);
			_state = State::WaitForArming;
			RCLCPP_INFO(_node.get_logger(), "Wait for arming");

		}, _mode->id());

	}, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);


	_vehicle_state.setOnVehicleStatusUpdate([this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
		bool armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
		_current_nav_state = msg->nav_state;

		if (_state == State::WaitForArming) {
			if (msg->pre_flight_checks_pass) {
				++_num_pre_flight_checks_pass;

			} else {
				_num_pre_flight_checks_pass = 0;
			}

			if (_num_pre_flight_checks_pass >
			    3) { // Make sure we check after the mode switch happened (as we don't wait for an ack)
				RCLCPP_INFO(_node.get_logger(), "Arming possible");
				_state = State::WaitUntilInAir;
				_mode->check_should_fail = true;
				_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
				_testing_timer = _node.create_wall_timer(5s, [this] {
					EXPECT_TRUE(_was_armed);
					_testing_timer.reset();
					RCLCPP_INFO(_node.get_logger(), "In air, checking mode switch prevented");
					_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
								   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);

					_testing_timer = _node.create_wall_timer(1s, [this] {
						_testing_timer.reset();
						// Mode did not switch to Descend/our mode, as the custom check prevented it
						EXPECT_NE(_current_nav_state, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);
						EXPECT_NE(_current_nav_state, _mode->id());
						RCLCPP_INFO(_node.get_logger(), "Current mode: %i", _current_nav_state);

						// Now clear the check and try again
						_mode->check_should_fail = false;
						_testing_timer = _node.create_wall_timer(1s, [this] {
							_testing_timer.reset();
							RCLCPP_INFO(_node.get_logger(), "In air, checking mode switch again");
							// This must activate our mode
							_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
										   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);
							_state = State::WaitForCustomMode;
						});
					});
				});
			}

		} else if (_state == State::WaitForCustomMode) {
			if (msg->nav_state == _mode->id()) {
				RCLCPP_INFO(_node.get_logger(), "Custom mode active, switching to hold");
				_vehicle_state.sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
							   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
				_state = State::WaitForHold;
			}

		} else if (_state == State::WaitForHold) {
			if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
				RCLCPP_INFO(_node.get_logger(), "Hold mode active, triggering failsafe");
				_vehicle_state.setGPSFailure(true);
				_state = State::WaitForFailsafe;

				// Failsafe must switch into our mode
				_vehicle_state.callbackOnModeSet([this]() {
					RCLCPP_INFO(_node.get_logger(), "Custom mode got activated, stopping custom mode");
					_state = State::TerminateMode;

					// Now stop the mode
					EXPECT_GT(_mode->num_activations, 0);
					EXPECT_GT(_mode->num_deactivations, 0);
					EXPECT_GT(_mode->num_setpoint_updates, 1);
					EXPECT_GT(_mode->num_arming_check_updates, 1);
					_mode.reset();

					// The FMU must fall back to the internal mode
					_vehicle_state.callbackOnModeSet([this]() {
						RCLCPP_INFO(_node.get_logger(), "Descend mode got activated, waiting for landing & disarm");
						_state = State::WaitForDisarm;
					}, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);

				}, _mode->id());
			}
		}

		if (_was_armed && !armed && _state == State::WaitForDisarm) {
			// Disarming, complete the test
			_vehicle_state.setGPSFailure(false);
			rclcpp::shutdown();
		}

		_was_armed = armed;
	});
}

TEST_F(Tester, run_mode_tests)
{
	auto test_node = initNode();
	ASSERT_TRUE(waitForFMU(*test_node, 10s));
	TestExecution test_execution{*test_node};
	test_execution.run();
	rclcpp::spin(test_node);
}


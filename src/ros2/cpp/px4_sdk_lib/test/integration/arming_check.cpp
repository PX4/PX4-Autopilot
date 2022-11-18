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

#include <px4_sdk/components/health_and_arming_checks.h>
#include <px4_sdk/components/wait_for_fmu.h>

#include <rclcpp/rclcpp.hpp>

using namespace px4_sdk;
using namespace std;
using namespace std::chrono_literals;

class Tester : public ::testing::Test
{
public:
	static void SetUpTestCase()
	{
		rclcpp::init(0, nullptr);
	}
	static void TearDownTestCase()
	{
		rclcpp::shutdown();
	}
};

class TestExecution
{
public:
	TestExecution(rclcpp::Node &node)
		: _node(node), _vehicle_state(node) {}

	void run();
private:
	enum class State {
		WaitForFirstCallback,
		WaitUntilCanArm,
		WaitUntilCannotArm,
		Completed,
	};

	void setState(State state)
	{
		RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);
		_state = state;
	}

	rclcpp::Node &_node;
	State _state{State::WaitForFirstCallback};
	rclcpp::TimerBase::SharedPtr _test_timeout;
	VehicleState _vehicle_state;
	int _num_failures_reported{0};
	std::unique_ptr<HealthAndArmingChecks> _health_and_arming_checks;
};

void TestExecution::run()
{
	_test_timeout = _node.create_wall_timer(20s, [this] {
		EXPECT_EQ(_state, State::Completed);
		rclcpp::shutdown();
	});

	// The test does the following:
	// - wait until arming callback called
	// - wait until arming is possible
	// - report a failing check
	// - expect arming denied
	_vehicle_state.setOnVehicleStatusUpdate([this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
		switch (_state) {
		case State::WaitUntilCanArm:
			if (msg->pre_flight_checks_pass) {
				setState(State::WaitUntilCannotArm);
			}

			break;

		case State::WaitUntilCannotArm:

			// Ensure we got a vehicle status update after we reported a failing check (prevent race conditions)
			if (_num_failures_reported == 2) {
				ASSERT_FALSE(msg->pre_flight_checks_pass);
				setState(State::Completed);
				rclcpp::shutdown();
			}

			break;

		default:
			break;
		}
	});

	_health_and_arming_checks = std::make_unique<HealthAndArmingChecks>(_node, [this](HealthAndArmingCheckReporter &
	reporter) {
		if (_state == State::WaitForFirstCallback) {
			setState(State::WaitUntilCanArm);

		} else if (_state == State::WaitUntilCannotArm) {
			/* EVENT
			 */
			reporter.armingCheckFailureExt(events::ID("check_unit_test_failure"),
						       events::Log::Error, "Test Failure");
			++_num_failures_reported;
		}
	});
	ASSERT_TRUE(_health_and_arming_checks->doRegister("arming check test"));
}


TEST_F(Tester, deny_arming)
{
	auto test_node = initNode();
	ASSERT_TRUE(waitForFMU(*test_node, 10s));
	TestExecution test_execution{*test_node};
	test_execution.run();
	rclcpp::spin(test_node);
}

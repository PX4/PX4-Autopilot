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

#include "registration.h"
#include "px4_sdk/components/health_and_arming_checks.h"

#include <cassert>

using namespace std::chrono_literals;
using namespace px4_sdk;

HealthAndArmingChecks::HealthAndArmingChecks(rclcpp::Node &node, const CheckCallback &check_callback,
		const std::string &topic_namespace_prefix)
	: _node(node), _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
	  _check_callback(check_callback)
{
	_arming_check_reply_pub = _node.create_publisher<px4_msgs::msg::ArmingCheckReply>(
					  topic_namespace_prefix + "/fmu/in/arming_check_reply", 1);

	_arming_check_request_sub = _node.create_subscription<px4_msgs::msg::ArmingCheckRequest>(
					    topic_namespace_prefix + "/fmu/out/arming_check_request",
					    rclcpp::QoS(1).best_effort(),
	[this](px4_msgs::msg::ArmingCheckRequest::UniquePtr msg) {

		RCLCPP_DEBUG_ONCE(_node.get_logger(), "Arming check request (id=%i, only printed once)", msg->request_id);

		if (_registration->registered()) {
			px4_msgs::msg::ArmingCheckReply reply{};
			reply.registration_id = _registration->armingCheckId();
			reply.request_id = msg->request_id;
			reply.can_arm_and_run = true;

			HealthAndArmingCheckReporter reporter(reply);
			_check_callback(reporter);

			reply.mode_req_angular_velocity = _mode_requirements.angular_velocity;
			reply.mode_req_attitude = _mode_requirements.attitude;
			reply.mode_req_local_alt = _mode_requirements.local_alt;
			reply.mode_req_local_position = _mode_requirements.local_position;
			reply.mode_req_local_position_relaxed = _mode_requirements.local_position_relaxed;
			reply.mode_req_global_position = _mode_requirements.global_position;
			reply.mode_req_mission = _mode_requirements.mission;
			reply.mode_req_home_position = _mode_requirements.home_position;
			reply.mode_req_prevent_arming = _mode_requirements.prevent_arming;

			reply.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
			_arming_check_reply_pub->publish(reply);
			_check_triggered = true;

		} else {
			RCLCPP_DEBUG(_node.get_logger(), "...not registered yet");
		}
	});

	_watchdog_timer = _node.create_wall_timer(4s, std::bind(&HealthAndArmingChecks::watchdogTimerUpdate, this));
}

void HealthAndArmingChecks::overrideRegistration(const std::shared_ptr<Registration> &registration)
{
	assert(!_registration->registered());
	_registration = registration;
}

bool HealthAndArmingChecks::doRegister(const std::string &name)
{
	assert(!_registration->registered());
	RegistrationSettings settings{};
	settings.name = name;
	settings.register_arming_check = true;
	return _registration->doRegister(settings);
}

void HealthAndArmingChecks::watchdogTimerUpdate()
{
	if (_registration->registered()) {
		if (!_check_triggered && _shutdown_on_timeout) {
			RCLCPP_FATAL(_node.get_logger(), "Timeout, no request received from FMU, exiting (this can happen on FMU reboots)");
			rclcpp::shutdown();
		}

		_check_triggered = false;

	} else {
		// avoid false positives while unregistered
		_check_triggered = true;
	}
}


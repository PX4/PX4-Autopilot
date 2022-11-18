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

#include <cassert>
#include <random>

using namespace std::chrono_literals;

Registration::Registration(rclcpp::Node &node, const std::string &topic_namespace_prefix)
	: _node(node)
{
	_register_ext_component_reply_sub = node.create_subscription<px4_msgs::msg::RegisterExtComponentReply>(
			topic_namespace_prefix + "/fmu/out/register_ext_component_reply",
			rclcpp::QoS(1).best_effort(),
	[this](px4_msgs::msg::RegisterExtComponentReply::UniquePtr msg) {
	});

	_register_ext_component_request_pub = node.create_publisher<px4_msgs::msg::RegisterExtComponentRequest>(
			topic_namespace_prefix + "/fmu/in/register_ext_component_request", 1);

	_unregister_ext_component_pub = node.create_publisher<px4_msgs::msg::UnregisterExtComponent>(
						topic_namespace_prefix + "/fmu/in/unregister_ext_component", 1);

	_unregister_ext_component.mode_id = px4_sdk::ModeBase::ID_INVALID;
}

bool Registration::doRegister(const RegistrationSettings &settings)
{
	assert(!_registered);
	px4_msgs::msg::RegisterExtComponentRequest request{};

	if (settings.name.length() >= request.name.size() || settings.name.length() >= _unregister_ext_component.name.size()) {
		RCLCPP_ERROR(_node.get_logger(), "Name too long (%i >= %i)", (int)settings.name.length(), (int)request.name.size());
		return false;
	}

	RCLCPP_DEBUG(_node.get_logger(), "Registering '%s' (arming check: %i, mode: %i, mode executor: %i)",
		     settings.name.c_str(),
		     settings.register_arming_check, settings.register_mode, settings.register_mode_executor);

	strcpy((char *)request.name.data(), settings.name.c_str());
	request.register_arming_check = settings.register_arming_check;
	request.register_mode = settings.register_mode;
	request.register_mode_executor = settings.register_mode_executor;
	request.enable_replace_internal_mode = settings.enable_replace_internal_mode;
	request.replace_internal_mode = settings.replace_internal_mode;
	request.activate_mode_immediately = settings.activate_mode_immediately;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<uint64_t> distrib{};
	request.request_id = distrib(gen);

	// wait for subscription, it might take a while initially...
	for (int i = 0; i < 100; ++i) {
		if (_register_ext_component_request_pub->get_subscription_count() > 0) {
			RCLCPP_DEBUG(_node.get_logger(), "Subscriber found, continuing");
			break;
		}

		usleep(100000);
	}

	// send request and wait for response
	rclcpp::WaitSet wait_set;
	wait_set.add_subscription(_register_ext_component_reply_sub);

	bool got_reply = false;

	for (int retries = 0; retries < 5 && !got_reply; ++retries) {
		request.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
		_register_ext_component_request_pub->publish(request);

		// wait for publisher, it might take a while initially...
		for (int i = 0; i < 100 && retries == 0; ++i) {
			if (_register_ext_component_reply_sub->get_publisher_count() > 0) {
				RCLCPP_DEBUG(_node.get_logger(), "Publisher found, continuing");
				break;
			}

			usleep(100000);
		}

		auto start_time = std::chrono::steady_clock::now();
		auto timeout = 300ms;

		while (!got_reply) {
			auto now = std::chrono::steady_clock::now();

			if (now >= start_time + timeout) {
				break;
			}

			auto wait_ret = wait_set.wait(timeout - (now - start_time));

			if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
				px4_msgs::msg::RegisterExtComponentReply reply;
				rclcpp::MessageInfo info;

				if (_register_ext_component_reply_sub->take(reply, info)) {
					reply.name.back() = '\0';

					if (strcmp((const char *) reply.name.data(), settings.name.c_str()) == 0 &&
					    request.request_id == reply.request_id) {
						RCLCPP_DEBUG(_node.get_logger(), "Got RegisterExtComponentReply");

						if (reply.success) {
							_unregister_ext_component.arming_check_id = reply.arming_check_id;
							_unregister_ext_component.mode_id = reply.mode_id;
							_unregister_ext_component.mode_executor_id = reply.mode_executor_id;
							strcpy((char *) _unregister_ext_component.name.data(), settings.name.c_str());
							_registered = true;

						} else {
							RCLCPP_ERROR(_node.get_logger(), "Registration failed");
						}

						got_reply = true;
					}

				} else {
					RCLCPP_INFO(_node.get_logger(), "no message received");
				}

			} else {
				RCLCPP_INFO(_node.get_logger(), "timeout");
			}
		}
	}

	wait_set.remove_subscription(_register_ext_component_reply_sub);

	return _registered;
}

void Registration::doUnregister()
{
	if (_registered) {
		RCLCPP_DEBUG(_node.get_logger(), "Unregistering");
		_unregister_ext_component.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
		_unregister_ext_component_pub->publish(_unregister_ext_component);
		_registered = false;
	}
}

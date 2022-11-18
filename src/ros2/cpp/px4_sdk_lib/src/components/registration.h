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

#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/register_ext_component_request.hpp>
#include <px4_msgs/msg/register_ext_component_reply.hpp>
#include <px4_msgs/msg/unregister_ext_component.hpp>

#include <px4_sdk/components/mode.h>

struct RegistrationSettings {
	std::string name;
	bool register_arming_check{false};
	bool register_mode{false};
	bool register_mode_executor{false};

	bool enable_replace_internal_mode{false};
	px4_sdk::ModeBase::ID_t replace_internal_mode{};
	bool activate_mode_immediately{false};
};

class Registration
{
public:

	Registration(rclcpp::Node &node, const std::string &topic_namespace_prefix = "");
	~Registration()
	{
		doUnregister();
	}

	bool doRegister(const RegistrationSettings &settings);
	void doUnregister();

	bool registered() const { return _registered; }

	int armingCheckId() const { return _unregister_ext_component.arming_check_id; }
	px4_sdk::ModeBase::ID_t modeId() const { return _unregister_ext_component.mode_id; }
	int modeExecutorId() const { return _unregister_ext_component.mode_executor_id; }

	std::string name() const { return (const char *)_unregister_ext_component.name.data(); }
private:
	rclcpp::Subscription<px4_msgs::msg::RegisterExtComponentReply>::SharedPtr _register_ext_component_reply_sub;
	rclcpp::Publisher<px4_msgs::msg::RegisterExtComponentRequest>::SharedPtr _register_ext_component_request_pub;
	rclcpp::Publisher<px4_msgs::msg::UnregisterExtComponent>::SharedPtr _unregister_ext_component_pub;

	bool _registered{false};
	px4_msgs::msg::UnregisterExtComponent _unregister_ext_component{};
	rclcpp::Node &_node;
};


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

#include "px4_sdk/components/wait_for_fmu.h"
#include <px4_msgs/msg/vehicle_status.hpp>

bool px4_sdk::waitForFMU(rclcpp::Node &node, rclcpp::Duration timeout, const std::string &topic_namespace_prefix)
{
	RCLCPP_INFO(node.get_logger(), "Waiting for FMU...");
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub =
		node.create_subscription<px4_msgs::msg::VehicleStatus>(
			topic_namespace_prefix + "/fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(),
	[](px4_msgs::msg::VehicleStatus::UniquePtr msg) {});

	rclcpp::WaitSet wait_set;
	wait_set.add_subscription(vehicle_status_sub);

	bool got_message = false;
	auto start_time = node.now();

	while (!got_message) {
		auto now = node.now();

		if (now >= start_time + timeout) {
			break;
		}

		auto wait_ret = wait_set.wait((timeout - (now - start_time)).to_chrono<std::chrono::microseconds>());

		if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
			px4_msgs::msg::VehicleStatus msg;
			rclcpp::MessageInfo info;

			if (vehicle_status_sub->take(msg, info)) {
				got_message = true;

			} else {
				RCLCPP_DEBUG(node.get_logger(), "no message received");
			}

		} else {
			RCLCPP_DEBUG(node.get_logger(), "timeout");
		}
	}

	wait_set.remove_subscription(vehicle_status_sub);
	return got_message;
}

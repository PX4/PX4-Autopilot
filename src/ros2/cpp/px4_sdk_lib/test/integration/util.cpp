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

#include "util.h"
#include <gtest/gtest.h>

std::shared_ptr<rclcpp::Node> initNode()
{
	auto test_node = std::make_shared<rclcpp::Node>("testnode");
	auto ret = rcutils_logging_set_logger_level(test_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

	if (ret != RCUTILS_RET_OK) {
		RCLCPP_ERROR(test_node->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
		rcutils_reset_error();
	}

	return test_node;
}

VehicleState::VehicleState(rclcpp::Node &node, const std::string &topic_namespace_prefix)
	: _node(node)
{
	_vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
				      topic_namespace_prefix + "/fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(),
	[this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
		if (_on_vehicle_status_update) {
			_on_vehicle_status_update(msg);
		}

		if (_on_mode_set_callback && _waiting_for_nav_state == msg->nav_state) {
			if (++_matching_nav_state_set > 2) { // wait a bit longer for the mode to be set, and mode activation triggered
				// clear before executing the callback as it might trigger another one
				ModeSetCallback cb(std::move(_on_mode_set_callback));
				_on_mode_set_callback = nullptr;
				cb();
			}
		}
	});

	_vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
				       topic_namespace_prefix + "/fmu/in/vehicle_command", 1);

}

void VehicleState::callbackOnModeSet(const VehicleState::ModeSetCallback &callback, uint8_t nav_state)
{
	assert(_on_mode_set_callback == nullptr);
	_on_mode_set_callback = callback;
	_waiting_for_nav_state = nav_state;
	_matching_nav_state_set = 0;
}

void VehicleState::sendCommand(uint32_t command, float param1, float param2, float param3, float param4,
			       float param5, float param6, float param7)
{
	// Send command, don't wait for ack
	px4_msgs::msg::VehicleCommand cmd{};
	cmd.command = command;
	cmd.param1 = param1;
	cmd.param2 = param2;
	cmd.param3 = param3;
	cmd.param4 = param4;
	cmd.param5 = param5;
	cmd.param6 = param6;
	cmd.param7 = param7;
	cmd.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_vehicle_command_pub->publish(cmd);
}

void VehicleState::setGPSFailure(bool failure)
{
	sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_INJECT_FAILURE,
		    px4_msgs::msg::VehicleCommand::FAILURE_UNIT_SENSOR_GPS,
		    failure ? px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OFF : px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OK, 0);
}

void VehicleState::setForceLowBattery(bool enabled)
{
	sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_INJECT_FAILURE,
		    px4_msgs::msg::VehicleCommand::FAILURE_UNIT_SYSTEM_BATTERY,
		    enabled ? px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OFF : px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OK, 0);
}

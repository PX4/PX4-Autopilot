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

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

std::shared_ptr<rclcpp::Node> initNode();

class VehicleState
{
public:
	VehicleState(rclcpp::Node &node, const std::string &topic_namespace_prefix = "");

	using VehicleStatusUpdateCallback = std::function<void(const px4_msgs::msg::VehicleStatus::UniquePtr &)>;

	using ModeSetCallback = std::function<void()>;

	const VehicleStatusUpdateCallback &getOnVehicleStatusUpdate() const
	{ return _on_vehicle_status_update; }

	void setOnVehicleStatusUpdate(const VehicleStatusUpdateCallback &on_vehicle_status_update)
	{ _on_vehicle_status_update = on_vehicle_status_update; }

	void callbackOnModeSet(const ModeSetCallback &callback, uint8_t nav_state);

	void sendCommand(uint32_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN, float param4 = NAN,
			 float param5 = NAN, float param6 = NAN, float param7 = NAN);

	// Methods to trigger failsafes
	void setGPSFailure(bool failure);
	void setForceLowBattery(bool enabled);
private:
	rclcpp::Node &_node;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
	VehicleStatusUpdateCallback _on_vehicle_status_update;
	ModeSetCallback _on_mode_set_callback;
	uint8_t _waiting_for_nav_state{};
	int _matching_nav_state_set{0};
};

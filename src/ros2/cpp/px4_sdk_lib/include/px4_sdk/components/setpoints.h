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

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <Eigen/Core>

namespace px4_sdk
{

class ModeBase;

class SetpointSender
{
public:
	enum class SetpointConfigurationResult {
		Success = 0,
		Timeout,
		FailureOther
	};

	struct SetpointConfiguration {
		// TODO...
		bool manual_enabled{false};
		bool auto_enabled{false};
		bool rates_enabled{true};
		bool attitude_enabled{true};
		bool acceleration_enabled{true};
		bool velocity_enabled{true};
		bool position_enabled{true};
		bool altitude_enabled{true};
		bool climb_rate_enabled{false};
	};

	SetpointSender(rclcpp::Node &node, const ModeBase &mode, const std::string &topic_namespace_prefix = "");

	SetpointConfigurationResult configureSetpointsSync(const SetpointConfiguration &config);

	void sendTrajectorySetpoint(const Eigen::Vector3f &velocity);

	// TODO: goto, stop, ...

private:
	rclcpp::Node &_node;
	const ModeBase &_mode;
	SetpointConfiguration _setpoint_configuration{};
	rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr _config_control_setpoints_pub;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
};

} /* namespace px4_sdk */

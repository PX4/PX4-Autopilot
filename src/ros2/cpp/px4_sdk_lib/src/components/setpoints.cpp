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

#include "px4_sdk/components/setpoints.h"
#include "px4_sdk/components/mode.h"

#include <cassert>

using namespace px4_sdk;

SetpointSender::SetpointSender(rclcpp::Node &node, const ModeBase &mode, const std::string &topic_namespace_prefix)
	: _node(node), _mode(mode)
{
	_config_control_setpoints_pub = node.create_publisher<px4_msgs::msg::VehicleControlMode>(
						topic_namespace_prefix + "/fmu/in/config_control_setpoints", 1);

	_trajectory_setpoint_pub = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
					   topic_namespace_prefix + "/fmu/in/trajectory_setpoint", 1);
}

SetpointSender::SetpointConfigurationResult SetpointSender::configureSetpointsSync(const SetpointConfiguration &config)
{
	assert(_mode.id() != ModeBase::ID_INVALID); // Ensure mode is registered

	px4_msgs::msg::VehicleControlMode control_mode{};
	control_mode.source_id = (uint8_t)_mode.id();
	control_mode.flag_control_manual_enabled = config.manual_enabled;
	control_mode.flag_control_auto_enabled = config.auto_enabled;
	control_mode.flag_control_rates_enabled = config.rates_enabled;
	control_mode.flag_control_attitude_enabled = config.attitude_enabled;
	control_mode.flag_control_acceleration_enabled = config.acceleration_enabled;
	control_mode.flag_control_velocity_enabled = config.velocity_enabled;
	control_mode.flag_control_position_enabled = config.position_enabled;
	control_mode.flag_control_altitude_enabled = config.altitude_enabled;
	control_mode.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_config_control_setpoints_pub->publish(control_mode);
	// TODO: wait for feedback from FMU

	_setpoint_configuration = config;
	return SetpointConfigurationResult::Success;
}

void SetpointSender::sendTrajectorySetpoint(const Eigen::Vector3f &velocity)
{
	// TODO: check if configured setpoints match

	// TODO: add mode id to setpoint

	px4_msgs::msg::TrajectorySetpoint sp{};
	sp.yaw = NAN;
	sp.yawspeed = NAN;
	sp.position[0] = sp.position[1] = sp.position[2] = NAN; // TODO ...
	sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
	sp.velocity[0] = velocity(0);
	sp.velocity[1] = velocity(1);
	sp.velocity[2] = velocity(2);
	sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_trajectory_setpoint_pub->publish(sp);
}

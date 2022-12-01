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

#include "px4_sdk/components/mode.h"

#include "registration.h"

#include <cassert>
#include <cfloat>

using namespace px4_sdk;

ModeBase::ModeBase(rclcpp::Node &node, const ModeBase::Settings &settings,
		   const ModeRequirements &requirements, const std::string &topic_namespace_prefix)
	: _node(node), _registration(std::make_shared<Registration>(node, topic_namespace_prefix)), _settings(settings),
	  _health_and_arming_checks(node, std::bind(&ModeBase::checkArmingAndRunConditions, this, std::placeholders::_1),
				    topic_namespace_prefix),
	  _setpoint_sender(node, *this, topic_namespace_prefix), _config_overrides(node, topic_namespace_prefix)
{
	_vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
				      topic_namespace_prefix + "/fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(),
	[this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
		if (_registration->registered()) {
			vehicleStatusUpdated(msg);
		}
	});
	_mode_completed_pub = _node.create_publisher<px4_msgs::msg::ModeCompleted>(
				      topic_namespace_prefix + "/fmu/in/mode_completed", 1);

	_health_and_arming_checks.setModeRequirements(requirements);
}

ModeBase::ID_t ModeBase::id() const
{
	return _registration->modeId();
}

void ModeBase::overrideRegistration(const std::shared_ptr<Registration> &registration)
{
	assert(!_registration->registered());
	_health_and_arming_checks.overrideRegistration(registration);
	_registration = registration;
}

bool ModeBase::doRegister()
{
	assert(!_registration->registered());
	_health_and_arming_checks.overrideRegistration(_registration);
	RegistrationSettings settings = getRegistrationSettings();
	bool ret = _registration->doRegister(settings);

	if (ret) {
		onRegistered();
	}

	return ret;
}

RegistrationSettings ModeBase::getRegistrationSettings() const
{
	RegistrationSettings settings{};
	settings.name = _settings.name;
	settings.register_arming_check = true;
	settings.register_mode = true;

	if (_settings.replace_internal_mode != ID_INVALID) {
		settings.enable_replace_internal_mode = true;
		settings.replace_internal_mode = _settings.replace_internal_mode;
	}

	return settings;
}

void ModeBase::callOnActivate()
{
	RCLCPP_DEBUG(_node.get_logger(), "Mode '%s' activated", _registration->name().c_str());
	_is_active = true;
	_completed = false;
	onActivate();

	if (_setpoint_update_rate_hz > FLT_EPSILON) {
		updateSetpoint(); // Immediately update
	}

	updateSetpointUpdateTimer();
}

void ModeBase::callOnDeactivate()
{
	RCLCPP_DEBUG(_node.get_logger(), "Mode '%s' deactivated", _registration->name().c_str());
	_is_active = false;
	onDeactivate();
	updateSetpointUpdateTimer();
}

void ModeBase::updateSetpointUpdateTimer()
{
	bool activate = _is_active && _setpoint_update_rate_hz > FLT_EPSILON;

	if (activate) {
		if (!_setpoint_update_timer) {
			_setpoint_update_timer = _node.create_wall_timer(std::chrono::milliseconds((long)(1000.f /
			_setpoint_update_rate_hz)), [this]() { updateSetpoint(); });
		}

	} else {
		if (_setpoint_update_timer) {
			_setpoint_update_timer.reset();
		}
	}
}

void ModeBase::setSetpointUpdateRate(float rate_hz)
{
	_setpoint_update_timer.reset();
	_setpoint_update_rate_hz = rate_hz;
	updateSetpointUpdateTimer();
}

void ModeBase::unsubscribeVehicleStatus()
{
	_vehicle_status_sub.reset();
}

void ModeBase::vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr &msg, bool do_not_activate)
{
	// Update state
	_is_armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
	bool is_active = id() == msg->nav_state && (_is_armed || _settings.activate_even_while_disarmed);

	if (_is_active != is_active) {
		if (is_active) {
			if (!do_not_activate) {
				callOnActivate();
			}

		} else {
			callOnDeactivate();
		}
	}
}

void ModeBase::completed(Result result)
{
	if (_completed) {
		RCLCPP_DEBUG_ONCE(_node.get_logger(), "Mode '%s': completed was already called", _registration->name().c_str());
		return;
	}

	px4_msgs::msg::ModeCompleted mode_completed{};
	mode_completed.nav_state = (uint8_t)id();
	mode_completed.result = (uint8_t)result;
	mode_completed.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_mode_completed_pub->publish(mode_completed);
	_completed = true;
}

void ModeBase::onRegistered()
{
	_config_overrides.setup(px4_msgs::msg::ConfigOverrides::SOURCE_TYPE_MODE, _registration->modeId());
}

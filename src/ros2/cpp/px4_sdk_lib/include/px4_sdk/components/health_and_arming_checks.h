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
#include <px4_msgs/msg/arming_check_reply.hpp>
#include <px4_msgs/msg/arming_check_request.hpp>

#include "events.h"

#include <memory>
#include <functional>

class Registration;

namespace px4_sdk
{

struct ModeRequirements {

	static ModeRequirements none()
	{
		// None is the default already
		return ModeRequirements{};
	}
	static ModeRequirements autonomous()
	{
		ModeRequirements ret{};
		ret.angular_velocity = true;
		ret.attitude = true;
		ret.local_alt = true;
		ret.local_position = true;
		ret.global_position = true;
		ret.home_position = true;
		return ret;
	}
	static ModeRequirements manualControlledPosition()
	{
		ModeRequirements ret{};
		ret.angular_velocity = true;
		ret.attitude = true;
		ret.local_alt = true;
		ret.local_position_relaxed = true;
		return ret;
	}

	bool angular_velocity{false};
	bool attitude{false};
	bool local_alt{false};
	bool local_position{false};
	bool local_position_relaxed{false};
	bool global_position{false};
	bool mission{false};
	bool home_position{false};
	bool prevent_arming{false};
};


class HealthAndArmingCheckReporter
{
public:
	HealthAndArmingCheckReporter(px4_msgs::msg::ArmingCheckReply &arming_check_reply)
		: _arming_check_reply(arming_check_reply) {}

	template<typename... Args>
	void armingCheckFailureExt(uint32_t event_id,
				   events::Log log_level, const char *message, Args... args)
	{
		uint16_t navigation_mode_groups{};
		uint8_t health_component_index{};

		_arming_check_reply.can_arm_and_run = false;

		if (!addEvent(event_id, log_level, message, navigation_mode_groups,
			      health_component_index, args...)) {
			printf("Error: too many events\n");
		}
	}

	void setHealth(uint8_t health_component_index, bool is_present, bool warning, bool error);

private:
	template<typename... Args>
	bool addEvent(uint32_t event_id, const events::LogLevels &log_levels, const char *message, Args... args);

	px4_msgs::msg::ArmingCheckReply &_arming_check_reply;
};


template<typename... Args>
bool HealthAndArmingCheckReporter::addEvent(uint32_t event_id, const events::LogLevels &log_levels,
		const char *message, Args... args)
{
	if (_arming_check_reply.num_events >= _arming_check_reply.events.size()) {
		return false;
	}

	events::EventType &e = _arming_check_reply.events[_arming_check_reply.num_events];
	e.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	e.id = event_id;
	static_assert(events::util::sizeofArguments(args...) <= sizeof(e.arguments), "Too many arguments");
	events::util::fillEventArguments((uint8_t *)e.arguments.data(), args...);
	++_arming_check_reply.num_events;
	return true;
}


inline void HealthAndArmingCheckReporter::setHealth(uint8_t health_component_index, bool is_present, bool warning,
		bool error)
{
	_arming_check_reply.health_component_index = health_component_index;
	_arming_check_reply.health_component_is_present = is_present;
	_arming_check_reply.health_component_warning = warning;
	_arming_check_reply.health_component_error = error;
}


class HealthAndArmingChecks
{
public:
	using CheckCallback = std::function<void(HealthAndArmingCheckReporter &)>;

	HealthAndArmingChecks(rclcpp::Node &node, const CheckCallback &check_callback,
			      const std::string &topic_namespace_prefix = "");
	HealthAndArmingChecks(const HealthAndArmingChecks &) = delete;

	/**
	 * Register the checks. Call this once on startup. This is a blocking method.
	 * @param name registration name. Should uniquely identify the component with length < 25 characters
	 * @return true on success
	 */
	bool doRegister(const std::string &name);

	void setModeRequirements(const ModeRequirements &mode_requirements) { _mode_requirements = mode_requirements; }

private:
	friend class ModeBase;
	friend class ModeExecutorBase;
	void overrideRegistration(const std::shared_ptr<Registration> &registration);

	void watchdogTimerUpdate();

	rclcpp::Node &_node;
	std::shared_ptr<Registration> _registration;
	CheckCallback _check_callback;
	bool _check_triggered{true};

	rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest>::SharedPtr _arming_check_request_sub;
	rclcpp::Publisher<px4_msgs::msg::ArmingCheckReply>::SharedPtr _arming_check_reply_pub;

	ModeRequirements _mode_requirements{};
	rclcpp::TimerBase::SharedPtr _watchdog_timer;
	bool _shutdown_on_timeout{true};
};

} /* namespace px4_sdk */

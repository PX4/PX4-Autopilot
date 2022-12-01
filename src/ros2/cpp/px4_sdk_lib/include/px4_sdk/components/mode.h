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
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/mode_completed.hpp>
#include "health_and_arming_checks.h"
#include "setpoints.h"
#include "overrides.h"

class Registration;
struct RegistrationSettings;

namespace px4_sdk
{

enum class Result {
	Success = 0,
	Rejected, ///< The request was rejected
	Interrupted, ///< Ctrl-C or another error (from ROS)
	Timeout,
	Deactivated, ///< Mode or executor got deactivated

	// Mode-specific results
	ModeFailureOther = 100,
};

static_assert((int)Result::ModeFailureOther == (int)px4_msgs::msg::ModeCompleted::RESULT_FAILURE_OTHER,
	      "definition mismatch");

constexpr inline const char *resultToString(Result result) noexcept
{
	switch (result) {
	case Result::Success: return "Success";

	case Result::Rejected: return "Rejected";

	case Result::Interrupted: return "Interrupted";

	case Result::Timeout: return "Timeout";

	case Result::Deactivated: return "Deactivated";

	case Result::ModeFailureOther: return "Mode Failure (generic)";
	}

	return "Unknown";
}

class ModeBase
{
public:
	using ID_t = uint8_t; ///< Mode ID, corresponds to nav_state
	static constexpr ID_t ID_INVALID = 0xff;

	static constexpr ID_t ID_NAVIGATION_STATE_POSCTL = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;
	static constexpr ID_t ID_NAVIGATION_STATE_AUTO_TAKEOFF = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF;
	static constexpr ID_t ID_NAVIGATION_STATE_DESCEND = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND;
	static constexpr ID_t ID_NAVIGATION_STATE_AUTO_LAND = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
	static constexpr ID_t ID_NAVIGATION_STATE_AUTO_RTL = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL;

	struct Settings {
		std::string name; ///< Name of the mode with length < 25 characters
		bool activate_even_while_disarmed{true}; ///< If true, the mode is also activated while disarmed if selected
		ID_t replace_internal_mode{ID_INVALID}; ///< Can be used to replace an fmu-internal mode
	};

	ModeBase(rclcpp::Node &node, const Settings &settings, const ModeRequirements &requirements,
		 const std::string &topic_namespace_prefix = "");
	ModeBase(const ModeBase &) = delete;
	virtual ~ModeBase() = default;

	/**
	 * Register the mode. Call this once on startup, unless there's an associated executor. This is a blocking method.
	 * @return true on success
	 */
	bool doRegister();


	/**
	 * Report any custom mode requirements. This is called regularly, also while the mode is active.
	 */
	virtual void checkArmingAndRunConditions(HealthAndArmingCheckReporter &reporter) {}

	/**
	 * Called whenever the mode is activated, also if the vehicle is disarmed
	 */
	virtual void onActivate() = 0;

	/**
	 * Called whenever the mode is deactivated, also if the vehicle is disarmed
	 */
	virtual void onDeactivate() = 0;

	/**
	 * Set the update rate when the mode is active. This is disabled by default.
	 * @param rate_hz set to 0 to disable
	 */
	void setSetpointUpdateRate(float rate_hz);

	virtual void updateSetpoint() {}

	/**
	 * Mode completed signal. Call this when the mode is finished. A mode might never call this, but modes like
	 * RTL, Land or Takeoff are expected to signal their completion.
	 * @param result
	 */
	void completed(Result result);


	// Properties & state

	ID_t id() const;

	bool isArmed() const { return _is_armed; }

	bool isActive() const { return _is_active; }

	rclcpp::Node &node() { return _node; }

	SetpointSender &setpoints() { return _setpoint_sender; }

	ConfigOverrides &configOverrides() { return _config_overrides; }

private:
	friend class ModeExecutorBase;
	void overrideRegistration(const std::shared_ptr<Registration> &registration);
	RegistrationSettings getRegistrationSettings() const;
	void onRegistered();

	void unsubscribeVehicleStatus();
	void vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr &msg, bool do_not_activate = false);

	void callOnActivate();
	void callOnDeactivate();

	void updateSetpointUpdateTimer();

	rclcpp::Node &_node;
	std::shared_ptr<Registration> _registration;

	const Settings _settings;

	HealthAndArmingChecks _health_and_arming_checks;

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::Publisher<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_pub;

	bool _is_active{false}; ///< Mode is currently selected
	bool _is_armed{false}; ///< Is vehicle armed?
	bool _completed{false}; ///< Is mode completed?

	float _setpoint_update_rate_hz{0.f};
	rclcpp::TimerBase::SharedPtr _setpoint_update_timer;
	SetpointSender _setpoint_sender;

	ConfigOverrides _config_overrides;
};

} /* namespace px4_sdk */

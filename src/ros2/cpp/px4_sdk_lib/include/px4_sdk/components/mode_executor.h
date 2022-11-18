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

#include "mode.h"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/mode_completed.hpp>

#include <functional>

class Registration;

namespace px4_sdk
{

class ModeExecutorBase
{
public:
	using CompletedCallback = std::function<void(Result)>;

	struct Settings {
		bool activate_immediately{false}; ///< If set activate the mode (and executor) immediately. Only use this for fully autonomous executors that also arm the vehicle
	};

	enum class DeactivateReason {
		FailsafeActivated,
		Other
	};

	ModeExecutorBase(rclcpp::Node &node, const Settings &settings, ModeBase &owned_mode,
			 const std::string &topic_namespace_prefix = "");
	ModeExecutorBase(const ModeExecutorBase &) = delete;
	virtual ~ModeExecutorBase() = default;

	/**
	 * Register the mode executor. Call this once on startup. This is a blocking method.
	 * @return true on success
	 */
	bool doRegister();


	/**
	 * Called whenever the mode is activated, also if the vehicle is disarmed
	 */
	virtual void onActivate() = 0;

	/**
	 * Called whenever the mode is deactivated, also if the vehicle is disarmed
	 */
	virtual void onDeactivate(DeactivateReason reason) = 0;


	/**
	* Send command and wait for ack/nack
	*/
	Result sendCommandSync(uint32_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN, float param4 = NAN,
			       float param5 = NAN, float param6 = NAN, float param7 = NAN);

	/**
	 * Switch to a mode with a callback when it is finished.
	 * The callback is also executed when the mode is deactivated.
	 * If there's already a mode scheduling active, the previous one is cancelled.
	 */
	void scheduleMode(ModeBase::ID_t mode_id, const CompletedCallback &on_completed);

	void takeoff(const CompletedCallback &on_completed, float altitude = NAN, float heading = NAN);
	void land(const CompletedCallback &on_completed);
	void rtl(const CompletedCallback &on_completed);

	void arm(const CompletedCallback &on_completed);
	void waitReadyToArm(const CompletedCallback &on_completed);
	void waitUntilDisarmed(const CompletedCallback &on_completed);

	bool isInCharge() const { return _is_in_charge; }

	bool isArmed() const { return _is_armed; }

	ModeBase &ownedMode() { return _owned_mode; }

	int id() const;

	rclcpp::Node &node() { return _node; }

private:

	class ScheduledMode
	{
	public:
		ScheduledMode(rclcpp::Node &node, const std::string &topic_namespace_prefix);

		bool active() const { return _mode_id != ModeBase::ID_INVALID; }
		void activate(ModeBase::ID_t mode_id, const CompletedCallback &on_completed);
		void cancel();
		ModeBase::ID_t modeId() const { return _mode_id; }
	private:
		void reset() { _mode_id = ModeBase::ID_INVALID; }

		ModeBase::ID_t _mode_id{ModeBase::ID_INVALID};
		CompletedCallback _on_completed_callback;
		rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_sub;
	};

	class WaitForVehicleStatusCondition
	{
	public:
		using RunCheckCallback = std::function<bool(const px4_msgs::msg::VehicleStatus::UniquePtr &msg)>;
		WaitForVehicleStatusCondition() = default;

		bool active() const { return _on_completed_callback != nullptr; }
		void update(const px4_msgs::msg::VehicleStatus::UniquePtr &msg);

		void activate(const RunCheckCallback &run_check_callback, const CompletedCallback &on_completed);
		void cancel();
	private:
		CompletedCallback _on_completed_callback;
		RunCheckCallback _run_check_callback;
	};

	void callOnActivate();
	void callOnDeactivate(DeactivateReason reason);

	void vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr &msg);

	void scheduleMode(ModeBase::ID_t mode_id, const px4_msgs::msg::VehicleCommand &cmd,
			  const ModeExecutorBase::CompletedCallback &on_completed);

	rclcpp::Node &_node;
	const Settings _settings;
	ModeBase &_owned_mode;

	std::shared_ptr<Registration> _registration;

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
	rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr _vehicle_command_ack_sub;

	ScheduledMode _current_scheduled_mode;
	WaitForVehicleStatusCondition _current_wait_vehicle_status;

	bool _is_in_charge{false};
	bool _is_armed{false};
	bool _was_never_activated{true};
	ModeBase::ID_t _prev_nav_state{ModeBase::ID_INVALID};
};

} /* namespace px4_sdk */

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

#include "px4_sdk/components/mode_executor.h"

#include "registration.h"

#include <cassert>
#include <future>
using namespace std::chrono_literals;

using namespace px4_sdk;

ModeExecutorBase::ModeExecutorBase(rclcpp::Node &node, const ModeExecutorBase::Settings &settings,
				   ModeBase &owned_mode, const std::string &topic_namespace_prefix)
	: _node(node), _settings(settings), _owned_mode(owned_mode),
	  _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
	  _current_scheduled_mode(node, topic_namespace_prefix),
	  _config_overrides(node, topic_namespace_prefix)
{
	_vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
				      topic_namespace_prefix + "/fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(),
	[this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
		if (_registration->registered()) {
			vehicleStatusUpdated(msg);
		}
	});

	_vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
				       topic_namespace_prefix + "/fmu/in/vehicle_command_mode_executor", 1);

	_vehicle_command_ack_sub = _node.create_subscription<px4_msgs::msg::VehicleCommandAck>(
					   topic_namespace_prefix + "/fmu/out/vehicle_command_ack", rclcpp::QoS(1).best_effort(),
	[this](px4_msgs::msg::VehicleCommandAck::UniquePtr msg) {});
}

bool ModeExecutorBase::doRegister()
{
	if (_owned_mode._registration->registered()) {
		RCLCPP_FATAL(_node.get_logger(), "Mode executor %s: mode already registered", _registration->name().c_str());
	}

	assert(!_registration->registered());
	_owned_mode.overrideRegistration(_registration);
	_owned_mode.unsubscribeVehicleStatus();
	RegistrationSettings settings = _owned_mode.getRegistrationSettings();
	settings.register_mode_executor = true;
	settings.activate_mode_immediately = _settings.activate_immediately;
	bool ret = _registration->doRegister(settings);

	if (ret) {
		_owned_mode.onRegistered();
		onRegistered();
	}

	return ret;
}

void ModeExecutorBase::onRegistered()
{
	_config_overrides.setup(px4_msgs::msg::ConfigOverrides::SOURCE_TYPE_MODE_EXECUTOR, _registration->modeExecutorId());
}

void ModeExecutorBase::callOnActivate()
{
	RCLCPP_DEBUG(_node.get_logger(), "Mode executor '%s' activated", _registration->name().c_str());
	_is_in_charge = true;
	onActivate();
}

void ModeExecutorBase::callOnDeactivate(DeactivateReason reason)
{
	RCLCPP_DEBUG(_node.get_logger(), "Mode executor '%s' deactivated (%i)", _registration->name().c_str(), (int)reason);
	_current_scheduled_mode.cancel();
	_current_wait_vehicle_status.cancel();
	_is_in_charge = false;
	_was_never_activated = false; // Set on deactivation, so we stay activated for the first time (while disarmed)
	onDeactivate(reason);
}

int ModeExecutorBase::id() const
{
	return _registration->modeExecutorId();
}

Result ModeExecutorBase::sendCommandSync(uint32_t command, float param1, float param2, float param3, float param4,
		float param5, float param6, float param7)
{
	// Send command and wait for ack
	Result result{Result::Rejected};
	px4_msgs::msg::VehicleCommand cmd{};
	cmd.command = command;
	cmd.param1 = param1;
	cmd.param2 = param2;
	cmd.param3 = param3;
	cmd.param4 = param4;
	cmd.param5 = param5;
	cmd.param6 = param6;
	cmd.param7 = param7;
	cmd.source_component = px4_msgs::msg::VehicleCommand::COMPONENT_MODE_EXECUTOR_START + id();
	cmd.timestamp = _node.get_clock()->now().nanoseconds() / 1000;


	rclcpp::WaitSet wait_set;
	wait_set.add_subscription(_vehicle_command_ack_sub);

	bool got_reply = false;
	auto start_time = std::chrono::steady_clock::now();
	auto timeout = 200ms;
	_vehicle_command_pub->publish(cmd);

	while (!got_reply) {
		auto now = std::chrono::steady_clock::now();

		if (now >= start_time + timeout) {
			break;
		}

		auto wait_ret = wait_set.wait(timeout - (now - start_time));

		if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
			px4_msgs::msg::VehicleCommandAck ack;
			rclcpp::MessageInfo info;

			if (_vehicle_command_ack_sub->take(ack, info)) {
				if (ack.command == cmd.command && ack.target_component == cmd.source_component) {
					if (ack.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
						result = Result::Success;
					}

					got_reply = true;
				}

			} else {
				RCLCPP_DEBUG(_node.get_logger(), "no message received");
			}

		} else {
			RCLCPP_DEBUG(_node.get_logger(), "timeout");
		}
	}

	wait_set.remove_subscription(_vehicle_command_ack_sub);

	if (!got_reply) {
		// We don't expect to run into an ack timeout
		result = Result::Timeout;
		RCLCPP_WARN(_node.get_logger(), "Cmd %i: timeout, no ack received", cmd.command);
	}

	return result;
}

void ModeExecutorBase::scheduleMode(ModeBase::ID_t mode_id, const CompletedCallback &on_completed)
{
	px4_msgs::msg::VehicleCommand cmd{};
	cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
	cmd.param1 = mode_id;
	scheduleMode(mode_id, cmd, on_completed);
}

void ModeExecutorBase::scheduleMode(ModeBase::ID_t mode_id, const px4_msgs::msg::VehicleCommand &cmd,
				    const CompletedCallback &on_completed)
{
	if (!_is_armed) {
		on_completed(Result::Rejected);
		return;
	}

	// If there's already an active mode, cancel it (it will call the callback with a failure result)
	_current_scheduled_mode.cancel();

	Result result = sendCommandSync(cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6,
					cmd.param7);

	if (result != Result::Success) {
		on_completed(result);
		return;
	}

	// Store the callback and ensure it's called eventually. There's a number of outcomes:
	// - The mode finishes and publishes the completion result.
	// - Failsafe is entered or the user switches out. In that case the executor gets deactivated.
	// - The user switches into the owned mode. In that case the fmu does not deactivate the executor.
	_current_scheduled_mode.activate(mode_id, on_completed);
}

void ModeExecutorBase::takeoff(const CompletedCallback &on_completed, float altitude, float heading)
{
	px4_msgs::msg::VehicleCommand cmd{};
	cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
	cmd.param1 = NAN;
	cmd.param2 = NAN;
	cmd.param3 = NAN;
	cmd.param4 = heading;
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param7 = altitude; // TODO: this is AMSL, local (relative to home) would be better
	scheduleMode(ModeBase::ID_NAVIGATION_STATE_AUTO_TAKEOFF, cmd, on_completed);
}

void ModeExecutorBase::land(const CompletedCallback &on_completed)
{
	scheduleMode(ModeBase::ID_NAVIGATION_STATE_AUTO_LAND, on_completed);
}

void ModeExecutorBase::rtl(const CompletedCallback &on_completed)
{
	scheduleMode(ModeBase::ID_NAVIGATION_STATE_AUTO_RTL, on_completed);
}

void ModeExecutorBase::arm(const CompletedCallback &on_completed)
{
	if (_is_armed) {
		on_completed(Result::Success);
		return;
	}

	Result result = sendCommandSync(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);

	if (result != Result::Success) {
		on_completed(result);
		return;
	}

	// Wait until our internal state changes to armed
	_current_wait_vehicle_status.activate(
	[this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) { return _is_armed; }, on_completed);
}

void ModeExecutorBase::waitReadyToArm(const CompletedCallback &on_completed)
{
	if (_is_armed) {
		on_completed(Result::Success);
		return;
	}

	RCLCPP_DEBUG(_node.get_logger(), "Waiting until ready to arm...");
	_current_wait_vehicle_status.activate([](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) { return msg->pre_flight_checks_pass; },
	on_completed);
}

void ModeExecutorBase::waitUntilDisarmed(const CompletedCallback &on_completed)
{
	if (!_is_armed) {
		on_completed(Result::Success);
		return;
	}

	RCLCPP_DEBUG(_node.get_logger(), "Waiting until disarmed...");
	_current_wait_vehicle_status.activate([this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) { return !_is_armed; },
	on_completed);
}

void ModeExecutorBase::vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr &msg)
{
	// Update state
	const bool was_armed = _is_armed;
	const ModeBase::ID_t current_mode = (ModeBase::ID_t)msg->nav_state;
	_is_armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
	const bool wants_to_activate_immediately = _settings.activate_immediately && _was_never_activated;
	const bool is_in_charge = id() == msg->executor_in_charge && (_is_armed || wants_to_activate_immediately);
	const bool changed_to_armed = !was_armed && _is_armed;

	if (_is_in_charge != is_in_charge) {
		if (is_in_charge) {
			callOnActivate();

		} else {
			callOnDeactivate(msg->failsafe ? DeactivateReason::FailsafeActivated : DeactivateReason::Other);
		}
	}

	if (_is_in_charge && _current_scheduled_mode.active() && _current_scheduled_mode.modeId() == _prev_nav_state
	    && current_mode == _owned_mode.id() && _prev_nav_state != current_mode) {
		// If the user switched from the currently scheduled mode to the owned mode, the executor stays in charge.
		// In order for the executor to restore the correct state, we re-activate it (which also cancels the scheduled
		// mode)
		callOnDeactivate(DeactivateReason::Other);
		callOnActivate();
	}


	if (_is_in_charge && _prev_failsafe_defer_state != msg->failsafe_defer_state
	    && msg->failsafe_defer_state == px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_WOULD_FAILSAFE) {
		// FMU wants to failsafe, but got deferred -> notify the executor
		onFailsafeDeferred();
	}

	_prev_failsafe_defer_state = msg->failsafe_defer_state;

	// Do not activate the mode if we're scheduling another mode. This is only expected to happen for a brief moment,
	// e.g. when the executor gets activated or right after arming. It thus prevents unnecessary mode activation toggling.
	bool do_not_activate_mode = (_current_scheduled_mode.active() && _owned_mode.id() != _current_scheduled_mode.modeId())
				    || (changed_to_armed && _current_wait_vehicle_status.active());
	// To avoid race conditions and ensure consistent ordering we update vehicle status of the mode directly.
	_owned_mode.vehicleStatusUpdated(msg, do_not_activate_mode && _is_in_charge);

	_prev_nav_state = current_mode;

	_current_wait_vehicle_status.update(msg);
}

bool ModeExecutorBase::deferFailsafesSync(bool enabled, int timeout_s)
{
	_config_overrides.deferFailsafes(enabled, timeout_s);

	// To avoid race conditions we wait until the FMU sets it if the executor is in charge
	if (enabled && _is_in_charge && _registration->registered()
	    && _prev_failsafe_defer_state == px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_DISABLED) {
		rclcpp::WaitSet wait_set;
		wait_set.add_subscription(_vehicle_status_sub);

		bool got_message = false;
		auto start_time = _node.now();
		const rclcpp::Duration timeout = 1s;

		while (!got_message) {
			auto now = _node.now();

			if (now >= start_time + timeout) {
				break;
			}

			auto wait_ret = wait_set.wait((timeout - (now - start_time)).to_chrono<std::chrono::microseconds>());

			if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
				px4_msgs::msg::VehicleStatus msg;
				rclcpp::MessageInfo info;

				if (_vehicle_status_sub->take(msg, info)) {
					if (msg.failsafe_defer_state != px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_DISABLED) {
						got_message = true;
					}

				} else {
					RCLCPP_DEBUG(_node.get_logger(), "no message received");
				}

			} else {
				RCLCPP_DEBUG(_node.get_logger(), "timeout");
			}
		}

		wait_set.remove_subscription(_vehicle_status_sub);

		return got_message;
	}

	return true;
}

ModeExecutorBase::ScheduledMode::ScheduledMode(rclcpp::Node &node, const std::string &topic_namespace_prefix)
{
	_mode_completed_sub = node.create_subscription<px4_msgs::msg::ModeCompleted>(
				      topic_namespace_prefix + "/fmu/out/mode_completed", rclcpp::QoS(1).best_effort(),
	[this, &node](px4_msgs::msg::ModeCompleted::UniquePtr msg) {
		if (active() && msg->nav_state == (uint8_t)_mode_id) {
			RCLCPP_DEBUG(node.get_logger(), "Got matching ModeCompleted message, result: %i", msg->result);
			CompletedCallback on_completed_callback(std::move(_on_completed_callback));
			reset();
			on_completed_callback((Result)msg->result); // Call after, as it might trigger new requests
		}
	});
}

void ModeExecutorBase::ScheduledMode::activate(ModeBase::ID_t mode_id, const CompletedCallback &on_completed)
{
	assert(!active());
	_mode_id = mode_id;
	_on_completed_callback = on_completed;
}

void ModeExecutorBase::ScheduledMode::cancel()
{
	if (active()) {
		CompletedCallback on_completed_callback(std::move(_on_completed_callback));
		reset();
		on_completed_callback(Result::Deactivated); // Call after, as it might trigger new requests
	}
}

void ModeExecutorBase::WaitForVehicleStatusCondition::update(const px4_msgs::msg::VehicleStatus::UniquePtr &msg)
{
	if (_on_completed_callback && _run_check_callback(msg)) {
		CompletedCallback on_completed_callback(std::move(_on_completed_callback));
		_on_completed_callback = nullptr;
		_run_check_callback = nullptr;
		on_completed_callback(Result::Success); // Call after, as it might trigger new requests
	}
}

void ModeExecutorBase::WaitForVehicleStatusCondition::activate(const RunCheckCallback &run_check_callback,
		const CompletedCallback &on_completed)
{
	assert(!_on_completed_callback);
	_on_completed_callback = on_completed;
	_run_check_callback = run_check_callback;
}

void ModeExecutorBase::WaitForVehicleStatusCondition::cancel()
{
	if (_on_completed_callback) {
		CompletedCallback on_completed_callback(std::move(_on_completed_callback));
		_on_completed_callback = nullptr;
		_run_check_callback = nullptr;
		on_completed_callback(Result::Deactivated); // Call after, as it might trigger new requests
	}
}

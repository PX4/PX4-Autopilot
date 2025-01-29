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

#ifndef CONSTRAINED_FLASH

#include "ModeManagement.hpp"

#include <px4_platform_common/events.h>

bool ModeExecutors::hasFreeExecutors() const
{
	for (int i = 0; i < MAX_NUM; ++i) {
		if (!_mode_executors[i].valid) {
			return true;
		}
	}

	return false;
}

int ModeExecutors::addExecutor(const ModeExecutors::ModeExecutor &executor)
{
	for (int i = 0; i < MAX_NUM; ++i) {
		if (!_mode_executors[i].valid) {
			_mode_executors[i] = executor;
			_mode_executors[i].valid = true;
			return i + FIRST_EXECUTOR_ID;
		}
	}

	PX4_ERR("logic error");
	return -1;
}

void ModeExecutors::removeExecutor(int id)
{
	if (valid(id)) {
		_mode_executors[id - FIRST_EXECUTOR_ID].valid = false;
	}
}

void ModeExecutors::printStatus(int executor_in_charge) const
{
	for (int i = 0; i < MAX_NUM; ++i) {
		if (_mode_executors[i].valid) {
			int executor_id = i + FIRST_EXECUTOR_ID;
			PX4_INFO("Mode Executor %i: owned nav_state: %i, in charge: %s", executor_id, _mode_executors[i].owned_nav_state,
				 executor_id == executor_in_charge ? "yes" : "no");

		}
	}
}

bool Modes::hasFreeExternalModes() const
{
	for (int i = 0; i < MAX_NUM; ++i) {
		if (!_modes[i].valid) {
			return true;
		}
	}

	return false;
}

uint8_t Modes::addExternalMode(const Modes::Mode &mode)
{
	int32_t mode_name_hash = (int32_t)events::util::hash_32_fnv1a_const(mode.name);

	if (mode_name_hash == 0) { // 0 is reserved for unused indexes
		mode_name_hash = 1;
	}

	// Try to find the index with matching hash (if mode was already registered before),
	// so that the same mode always gets the same index (required for RC switch mode assignment)
	int first_unused_idx = -1;
	int first_invalid_idx = -1;
	int matching_idx = -1;

	for (int i = 0; i < MAX_NUM; ++i) {
		char hash_param_name[20];
		snprintf(hash_param_name, sizeof(hash_param_name), "COM_MODE%d_HASH", i);
		const param_t handle = param_find(hash_param_name);
		int32_t current_hash{};

		if (handle != PARAM_INVALID && param_get(handle, &current_hash) == 0) {
			if (!_modes[i].valid && current_hash == 0 && first_unused_idx == -1) {
				first_unused_idx = i;
			}

			if (current_hash == mode_name_hash) {
				matching_idx = i;
			}

			if (!_modes[i].valid && first_invalid_idx == -1) {
				first_invalid_idx = i;
			}
		}
	}

	bool need_to_update_param = false;
	int new_mode_idx = -1;

	if (matching_idx != -1) {
		// If we found a match, try to use it but check for hash collisions or duplicate mode name
		if (_modes[matching_idx].valid) {
			// This can happen when restarting modes while armed
			PX4_WARN("Mode '%s' already registered (as '%s')", mode.name, _modes[matching_idx].name);

			if (first_unused_idx != -1) {
				new_mode_idx = first_unused_idx;
				// Do not update the hash

			} else {
				// Need to overwrite a hash. Reset it as we can't store duplicate hashes anyway
				new_mode_idx = first_invalid_idx;
				need_to_update_param = true;
				mode_name_hash = 0;
			}

		} else {
			new_mode_idx = matching_idx;
		}

	} else if (first_unused_idx != -1) {
		// Mode registers the first time and there's still unused indexes
		need_to_update_param = true;
		new_mode_idx = first_unused_idx;

	} else {
		// Mode registers the first time but all indexes are used so we need to overwrite one
		need_to_update_param = true;
		new_mode_idx = first_invalid_idx;
	}

	if (new_mode_idx != -1 && !_modes[new_mode_idx].valid) {
		if (need_to_update_param) {
			char hash_param_name[20];
			snprintf(hash_param_name, sizeof(hash_param_name), "COM_MODE%d_HASH", new_mode_idx);
			const param_t handle = param_find(hash_param_name);

			if (handle != PARAM_INVALID) {
				param_set_no_notification(handle, &mode_name_hash);
			}
		}

		_modes[new_mode_idx] = mode;
		_modes[new_mode_idx].valid = true;
		return new_mode_idx + FIRST_EXTERNAL_NAV_STATE;
	}

	PX4_ERR("logic error");
	return -1;
}

bool Modes::removeExternalMode(uint8_t nav_state, const char *name)
{
	if (valid(nav_state) && strncmp(name, _modes[nav_state - FIRST_EXTERNAL_NAV_STATE].name, sizeof(Mode::name)) == 0) {
		_modes[nav_state - FIRST_EXTERNAL_NAV_STATE].valid = false;
		return true;
	}

	PX4_ERR("trying to remove invalid mode %s", name);
	return false;
}

void Modes::printStatus() const
{
	for (int i = Modes::FIRST_EXTERNAL_NAV_STATE; i <= Modes::LAST_EXTERNAL_NAV_STATE; ++i) {
		if (valid(i)) {
			const Modes::Mode &cur_mode = mode(i);
			PX4_INFO("External Mode %i: nav_state: %i, name: %s", i - vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + 1, i,
				 cur_mode.name);

			if (cur_mode.replaces_nav_state != Mode::REPLACES_NAV_STATE_NONE
			    && cur_mode.replaces_nav_state < vehicle_status_s::NAVIGATION_STATE_MAX) {
				PX4_INFO("  Replaces mode: %s", mode_util::nav_state_names[cur_mode.replaces_nav_state]);
			}
		}
	}
}

ModeManagement::ModeManagement(ExternalChecks &external_checks)
	: _external_checks(external_checks)
{
	_external_checks.setExternalNavStates(Modes::FIRST_EXTERNAL_NAV_STATE, Modes::LAST_EXTERNAL_NAV_STATE);
}

void ModeManagement::checkNewRegistrations(UpdateRequest &update_request)
{
	register_ext_component_request_s request;
	int max_updates = 5;

	while (!update_request.change_user_intended_nav_state && _register_ext_component_request_sub.update(&request)
	       && --max_updates >= 0) {
		request.name[sizeof(request.name) - 1] = '\0';
		PX4_DEBUG("got registration request: %s %llu, arming: %i mode: %i executor: %i", request.name, request.request_id,
			  request.register_arming_check, request.register_mode, request.register_mode_executor);
		register_ext_component_reply_s reply{};
		reply.mode_executor_id = -1;
		reply.mode_id = -1;
		reply.arming_check_id = -1;
		static_assert(sizeof(request.name) == sizeof(reply.name), "size mismatch");
		memcpy(reply.name, request.name, sizeof(request.name));
		reply.request_id = request.request_id;
		reply.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;

		// validate
		bool request_valid = true;

		if (request.register_mode_executor && !request.register_mode) {
			request_valid = false;
		}

		if (request.register_mode && !request.register_arming_check) {
			request_valid = false;
		}

		reply.success = false;

		if (request_valid) {
			// check free space
			reply.success = true;

			if (request.register_arming_check && !_external_checks.hasFreeRegistrations()) {
				PX4_WARN("No free slots for arming checks");
				reply.success = false;
			}

			if (request.register_mode) {
				if (!_modes.hasFreeExternalModes()) {
					PX4_WARN("No free slots for modes");
					reply.success = false;

				} else if (request.enable_replace_internal_mode) {
					// Check if another one already replaces the same mode
					for (int i = Modes::FIRST_EXTERNAL_NAV_STATE; i <= Modes::LAST_EXTERNAL_NAV_STATE; ++i) {
						if (_modes.valid(i)) {
							const Modes::Mode &cur_mode = _modes.mode(i);

							if (cur_mode.replaces_nav_state == request.replace_internal_mode) {
								// TODO: we could add priorities and allow the highest priority to do the replacement
								PX4_ERR("Trying to replace an already replaced mode (%i)", request.replace_internal_mode);
								reply.success = false;
							}
						}
					}
				}
			}

			if (request.register_mode_executor && !_mode_executors.hasFreeExecutors()) {
				PX4_WARN("No free slots for executors");
				reply.success = false;
			}

			// register component(s)
			if (reply.success) {
				int nav_mode_id = -1;

				if (request.register_mode) {
					Modes::Mode mode{};
					strncpy(mode.name, request.name, sizeof(mode.name));

					if (request.enable_replace_internal_mode) {
						mode.replaces_nav_state = request.replace_internal_mode;
					}

					nav_mode_id = _modes.addExternalMode(mode);
					reply.mode_id = nav_mode_id;
				}

				if (request.register_mode_executor) {
					ModeExecutors::ModeExecutor executor{};
					executor.owned_nav_state = nav_mode_id;
					int registration_id = _mode_executors.addExecutor(executor);

					if (nav_mode_id != -1) {
						_modes.mode(nav_mode_id).mode_executor_registration_id = registration_id;
					}

					reply.mode_executor_id = registration_id;
				}

				if (request.register_arming_check) {
					int8_t replace_nav_state = request.enable_replace_internal_mode ? request.replace_internal_mode : -1;
					int registration_id = _external_checks.addRegistration(nav_mode_id, replace_nav_state);

					if (nav_mode_id != -1) {
						_modes.mode(nav_mode_id).arming_check_registration_id = registration_id;
					}

					reply.arming_check_id = registration_id;
				}

				// Activate the mode?
				if (request.register_mode_executor && request.activate_mode_immediately && nav_mode_id != -1) {
					update_request.change_user_intended_nav_state = true;
					update_request.user_intended_nav_state = nav_mode_id;
				}
			}
		}

		reply.timestamp = hrt_absolute_time();
		_register_ext_component_reply_pub.publish(reply);
	}
}

void ModeManagement::checkUnregistrations(uint8_t user_intended_nav_state, UpdateRequest &update_request)
{
	unregister_ext_component_s request;
	int max_updates = 5;

	while (!update_request.change_user_intended_nav_state && _unregister_ext_component_sub.update(&request)
	       && --max_updates >= 0) {
		request.name[sizeof(request.name) - 1] = '\0';
		PX4_DEBUG("got unregistration request: %s arming: %i mode: %i executor: %i", request.name,
			  (int)request.arming_check_id, (int)request.mode_id, (int)request.mode_executor_id);

		if (request.arming_check_id != -1) {
			_external_checks.removeRegistration(request.arming_check_id, request.mode_id);
		}

		if (request.mode_id != -1) {
			if (_modes.removeExternalMode(request.mode_id, request.name)) {
				removeModeExecutor(request.mode_executor_id);
				// else: if the mode was already removed (due to a timeout), the executor was also removed already
			}

			// If the removed mode is currently active, switch to Hold
			if (user_intended_nav_state == request.mode_id) {
				update_request.change_user_intended_nav_state = true;
				update_request.user_intended_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			}
		}
	}
}

void ModeManagement::update(bool armed, uint8_t user_intended_nav_state, bool failsafe_action_active,
			    UpdateRequest &update_request)
{
	_failsafe_action_active = failsafe_action_active;
	_external_checks.update();

	bool allow_update_while_armed = _external_checks.allowUpdateWhileArmed();

	if (armed && !allow_update_while_armed) {
		// Reject registration requests
		register_ext_component_request_s request;

		if (_register_ext_component_request_sub.update(&request)) {
			PX4_ERR("Not accepting registration requests while armed");
			register_ext_component_reply_s reply{};
			reply.success = false;
			static_assert(sizeof(request.name) == sizeof(reply.name), "size mismatch");
			memcpy(reply.name, request.name, sizeof(request.name));
			reply.request_id = request.request_id;
			reply.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
			reply.timestamp = hrt_absolute_time();
			_register_ext_component_reply_pub.publish(reply);
		}

	} else {
		// Check for unresponsive modes
		for (int i = Modes::FIRST_EXTERNAL_NAV_STATE; i <= Modes::LAST_EXTERNAL_NAV_STATE; ++i) {
			if (_modes.valid(i)) {
				const Modes::Mode &mode = _modes.mode(i);

				// Remove only if not currently selected
				if (user_intended_nav_state != i && _external_checks.isUnresponsive(mode.arming_check_registration_id)) {
					PX4_DEBUG("Removing unresponsive mode %i", i);
					_external_checks.removeRegistration(mode.arming_check_registration_id, i);
					removeModeExecutor(mode.mode_executor_registration_id);
					_modes.removeExternalMode(i, mode.name);
				}
			}
		}

		// As we're disarmed we can use the user intended mode, as no failsafe will be active.
		// Note that this might not be true if COM_MODE_ARM_CHK is set
		checkNewRegistrations(update_request);
		checkUnregistrations(user_intended_nav_state, update_request);
	}

	update_request.control_setpoint_update = checkConfigControlSetpointUpdates();
	checkConfigOverrides();
}

void ModeManagement::onUserIntendedNavStateChange(ModeChangeSource source, uint8_t user_intended_nav_state)
{
	// Update mode executor in charge
	int mode_executor_for_intended_nav_state = -1;

	if (_modes.valid(user_intended_nav_state)) {
		mode_executor_for_intended_nav_state = _modes.mode(user_intended_nav_state).mode_executor_registration_id;
	}

	if (mode_executor_for_intended_nav_state == -1) {
		// Not an owned mode: check source
		if (source == ModeChangeSource::User) {
			// Give control to the pilot
			_mode_executor_in_charge = ModeExecutors::AUTOPILOT_EXECUTOR_ID;
		}

	} else {
		// Switched into an owned mode: put executor in charge
		_mode_executor_in_charge = mode_executor_for_intended_nav_state;
	}
}

uint8_t ModeManagement::getNavStateReplacementIfValid(uint8_t nav_state, bool report_error)
{
	for (int i = Modes::FIRST_EXTERNAL_NAV_STATE; i <= Modes::LAST_EXTERNAL_NAV_STATE; ++i) {
		if (_modes.valid(i)) {
			Modes::Mode &mode = _modes.mode(i);

			if (mode.replaces_nav_state == nav_state) {
				if (_external_checks.isUnresponsive(mode.arming_check_registration_id)) {
					if (!mode.unresponsive_reported && report_error) {
						mode.unresponsive_reported = true;
						events::send(events::ID("commander_mode_fallback_internal"), events::Log::Critical,
							     "External mode is unresponsive, falling back to internal");
					}

					return nav_state;

				} else {
					return i;
				}
			}
		}
	}

	return nav_state;
}

uint8_t ModeManagement::getReplacedModeIfAny(uint8_t nav_state)
{
	if (_modes.valid(nav_state)) {
		const Modes::Mode &mode = _modes.mode(nav_state);

		if (mode.replaces_nav_state != Modes::Mode::REPLACES_NAV_STATE_NONE) {
			return mode.replaces_nav_state;
		}
	}

	return nav_state;
}

void ModeManagement::removeModeExecutor(int mode_executor_id)
{
	if (mode_executor_id == -1) {
		return;
	}

	if (_mode_executor_in_charge == mode_executor_id) {
		_mode_executor_in_charge = ModeExecutors::AUTOPILOT_EXECUTOR_ID;
	}

	_mode_executors.removeExecutor(mode_executor_id);
}

int ModeManagement::modeExecutorInCharge() const
{
	if (_failsafe_action_active) {
		return ModeExecutors::AUTOPILOT_EXECUTOR_ID;
	}

	return _mode_executor_in_charge;
}

bool ModeManagement::updateControlMode(uint8_t nav_state, vehicle_control_mode_s &control_mode) const
{
	bool ret = false;

	if (nav_state >= Modes::FIRST_EXTERNAL_NAV_STATE && nav_state <= Modes::LAST_EXTERNAL_NAV_STATE) {
		if (_modes.valid(nav_state)) {
			control_mode = _modes.mode(nav_state).config_control_setpoint;
			ret = true;

		} else {
			Modes::Mode::setControlModeDefaults(control_mode);
		}
	}

	return ret;
}

void ModeManagement::printStatus() const
{
	_modes.printStatus();
	_mode_executors.printStatus(modeExecutorInCharge());
}

void ModeManagement::updateActiveConfigOverrides(uint8_t nav_state, config_overrides_s &overrides_in_out)
{
	config_overrides_s current_overrides;

	if (_modes.valid(nav_state)) {
		current_overrides = _modes.mode(nav_state).overrides;

	} else {
		current_overrides = {};
	}

	// Apply the overrides from executors on top (executors take precedence)
	const int executor_in_charge = modeExecutorInCharge();

	if (_mode_executors.valid(executor_in_charge)) {
		const config_overrides_s &executor_overrides = _mode_executors.executor(executor_in_charge).overrides;

		if (executor_overrides.disable_auto_disarm) {
			current_overrides.disable_auto_disarm = true;
		}

		if (executor_overrides.defer_failsafes) {
			current_overrides.defer_failsafes = true;
			current_overrides.defer_failsafes_timeout_s = executor_overrides.defer_failsafes_timeout_s;
		}
	}

	// Publish if changed or at low rate
	current_overrides.timestamp = overrides_in_out.timestamp;

	if (memcmp(&overrides_in_out, &current_overrides, sizeof(current_overrides)) != 0
	    || hrt_elapsed_time(&current_overrides.timestamp) > 500_ms) {
		current_overrides.timestamp = hrt_absolute_time();
		_config_overrides_pub.publish(current_overrides);
		overrides_in_out = current_overrides;
	}
}

bool ModeManagement::checkConfigControlSetpointUpdates()
{
	bool had_update = false;
	vehicle_control_mode_s config_control_setpoint;
	int max_updates = 5;

	while (_config_control_setpoints_sub.update(&config_control_setpoint) && --max_updates >= 0) {
		if (_modes.valid(config_control_setpoint.source_id)) {
			_modes.mode(config_control_setpoint.source_id).config_control_setpoint = config_control_setpoint;
			had_update = true;

		} else {
			if (!_invalid_mode_printed) {
				PX4_ERR("Control sp config request for invalid mode: %i", config_control_setpoint.source_id);
				_invalid_mode_printed = true;
			}
		}
	}

	return had_update;
}

void ModeManagement::checkConfigOverrides()
{
	config_overrides_s override_request;
	int max_updates = config_overrides_s::ORB_QUEUE_LENGTH;

	while (_config_overrides_request_sub.update(&override_request) && --max_updates >= 0) {
		switch (override_request.source_type) {
		case config_overrides_s::SOURCE_TYPE_MODE_EXECUTOR:
			if (_mode_executors.valid(override_request.source_id)) {
				ModeExecutors::ModeExecutor &executor = _mode_executors.executor(override_request.source_id);
				memcpy(&executor.overrides, &override_request, sizeof(executor.overrides));
				static_assert(sizeof(executor.overrides) == sizeof(override_request), "size mismatch");
			}

			break;

		case config_overrides_s::SOURCE_TYPE_MODE:
			if (_modes.valid(override_request.source_id)) {
				Modes::Mode &mode = _modes.mode(override_request.source_id);
				memcpy(&mode.overrides, &override_request, sizeof(mode.overrides));
			}

			break;
		}
	}
}

void ModeManagement::getModeStatus(uint32_t &valid_nav_state_mask, uint32_t &can_set_nav_state_mask) const
{
	valid_nav_state_mask = mode_util::getValidNavStates();
	can_set_nav_state_mask = valid_nav_state_mask & ~(1u << vehicle_status_s::NAVIGATION_STATE_TERMINATION);

	// Add external modes
	for (int i = Modes::FIRST_EXTERNAL_NAV_STATE; i <= Modes::LAST_EXTERNAL_NAV_STATE; ++i) {
		if (_modes.valid(i)) {
			valid_nav_state_mask |= 1u << i;
			can_set_nav_state_mask |= 1u << i;
			const Modes::Mode &cur_mode = _modes.mode(i);

			if (cur_mode.replaces_nav_state != Modes::Mode::REPLACES_NAV_STATE_NONE) {
				// Hide the internal mode if it's replaced
				can_set_nav_state_mask &= ~(1u << cur_mode.replaces_nav_state);
			}

		} else {
			// Still set the mode as valid but not as selectable. This is because an external mode could still
			// be selected via RC when not yet running, so we make sure to display some mode label indicating it's not
			// available.
			valid_nav_state_mask |= 1u << i;
		}
	}
}

#endif /* CONSTRAINED_FLASH */

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

#include "externalChecks.hpp"

static void setOrClearRequirementBits(bool requirement_set, int8_t nav_state, int8_t replaces_nav_state, uint32_t &bits)
{
	if (requirement_set) {
		bits |= 1u << nav_state;
	}

	if (replaces_nav_state != -1) {
		if (requirement_set) {
			bits |= 1u << replaces_nav_state;

		} else {
			bits &= ~(1u << replaces_nav_state);
		}
	}
}

int ExternalChecks::addRegistration(int8_t nav_mode_id, int8_t replaces_nav_state)
{
	int free_registration_index = -1;

	for (int i = 0; i < MAX_NUM_REGISTRATIONS; ++i) {
		if (!registrationValid(i)) {
			free_registration_index = i;
			break;
		}
	}

	if (free_registration_index != -1) {
		_active_registrations_mask |= 1 << free_registration_index;
		_registrations[free_registration_index].nav_mode_id = nav_mode_id;
		_registrations[free_registration_index].replaces_nav_state = replaces_nav_state;
		_registrations[free_registration_index].waiting_for_first_response = true;
		_registrations[free_registration_index].num_no_response = 0;
		_registrations[free_registration_index].unresponsive = false;
		_registrations[free_registration_index].total_num_unresponsive = 0;

		if (!_registrations[free_registration_index].reply) {
			_registrations[free_registration_index].reply = new arming_check_reply_s();
		}
	}

	return free_registration_index;
}

bool ExternalChecks::removeRegistration(int registration_id, int8_t nav_mode_id)
{
	if (registration_id < 0 || registration_id >= MAX_NUM_REGISTRATIONS) {
		return false;
	}

	if (registrationValid(registration_id)) {
		if (_registrations[registration_id].nav_mode_id == nav_mode_id) {
			_active_registrations_mask &= ~(1u << registration_id);
			return true;
		}
	}

	PX4_ERR("trying to remove inactive external check");
	return false;
}

bool ExternalChecks::isUnresponsive(int registration_id)
{
	if (registration_id < 0 || registration_id >= MAX_NUM_REGISTRATIONS) {
		return false;
	}

	if (registrationValid(registration_id)) {
		return _registrations[registration_id].unresponsive;
	}

	return false;
}


void ExternalChecks::checkAndReport(const Context &context, Report &reporter)
{
	checkNonRegisteredModes(context, reporter);

	if (_active_registrations_mask == 0) {
		return;
	}

	NavModes unresponsive_modes{NavModes::None};

	for (int reg_idx = 0; reg_idx < MAX_NUM_REGISTRATIONS; ++reg_idx) {
		if (!registrationValid(reg_idx) || !_registrations[reg_idx].reply) {
			continue;
		}

		arming_check_reply_s &reply = *_registrations[reg_idx].reply;

		int8_t nav_mode_id = _registrations[reply.registration_id].nav_mode_id;

		if (_registrations[reply.registration_id].unresponsive) {

			if (nav_mode_id != -1) {
				unresponsive_modes = unresponsive_modes | reporter.getModeGroup(nav_mode_id);
				setOrClearRequirementBits(true, nav_mode_id, -1, reporter.failsafeFlags().mode_req_other);
			}

		} else {
			NavModes modes;

			// We distinguish between two cases:
			// - external navigation mode: in that case we set the single arming can_run bit for the mode
			// - generic external arming check: set all arming bits
			if (nav_mode_id == -1) {
				modes = NavModes::All;

			} else {
				modes = reporter.getModeGroup(nav_mode_id);

				int8_t replaces_nav_state = _registrations[reply.registration_id].replaces_nav_state;

				if (replaces_nav_state != -1) {
					modes = modes | reporter.getModeGroup(replaces_nav_state);
					// Also clear the arming bits for the replaced mode, as the user intention is always set to the
					// replaced mode.
					// We only have to clear the bits, as for the internal/replaced mode, the bits are not cleared yet.
				}

				if (!reply.can_arm_and_run) {
					setOrClearRequirementBits(true, nav_mode_id, replaces_nav_state, reporter.failsafeFlags().mode_req_other);
				}

				// Mode requirements
				// A replacement mode will also replace the mode requirements of the internal/replaced mode
				setOrClearRequirementBits(reply.mode_req_angular_velocity, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_angular_velocity);
				setOrClearRequirementBits(reply.mode_req_attitude, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_attitude);
				setOrClearRequirementBits(reply.mode_req_local_alt, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_local_alt);
				setOrClearRequirementBits(reply.mode_req_local_position, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_local_position);
				setOrClearRequirementBits(reply.mode_req_local_position_relaxed, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_local_position_relaxed);
				setOrClearRequirementBits(reply.mode_req_global_position, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_global_position);
				setOrClearRequirementBits(reply.mode_req_mission, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_mission);
				setOrClearRequirementBits(reply.mode_req_home_position, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_home_position);
				setOrClearRequirementBits(reply.mode_req_prevent_arming, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_prevent_arming);
				setOrClearRequirementBits(reply.mode_req_manual_control, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_manual_control);
			}

			if (!reply.can_arm_and_run) {
				reporter.clearArmingBits(modes);
			}

			if (reply.health_component_index > 0) {
				reporter.setHealth((health_component_t)(1ull << reply.health_component_index),
						   reply.health_component_is_present, reply.health_component_warning,
						   reply.health_component_error);
			}

			for (int i = 0; i < reply.num_events; ++i) {
				// set the modes, which is the first argument
				memcpy(reply.events[i].arguments, &modes, sizeof(modes));

				reporter.addExternalEvent(reply.events[i], modes);
			}
		}
	}

	if (unresponsive_modes != NavModes::None) {
		/* EVENT
		 * @description
		 * The application running the mode might have crashed or the CPU load is too high.
		 */
		reporter.armingCheckFailure(unresponsive_modes, health_component_t::system,
					    events::ID("check_external_modes_unresponsive"),
					    events::Log::Critical, "Mode is unresponsive");
	}

}

void ExternalChecks::update()
{
	if (_active_registrations_mask == 0) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	// Check for incoming replies
	arming_check_reply_s reply;
	int max_num_updates = arming_check_reply_s::ORB_QUEUE_LENGTH;

	while (_arming_check_reply_sub.update(&reply) && --max_num_updates >= 0) {
		if (reply.registration_id < MAX_NUM_REGISTRATIONS && registrationValid(reply.registration_id)
		    && _current_request_id == reply.request_id) {
			_reply_received_mask |= 1u << reply.registration_id;
			_registrations[reply.registration_id].num_no_response = 0;
			_registrations[reply.registration_id].waiting_for_first_response = false;

			// Prevent toggling between unresponsive & responsive state
			if (_registrations[reply.registration_id].total_num_unresponsive <= 3) {
				_registrations[reply.registration_id].unresponsive = false;
			}

			if (_registrations[reply.registration_id].reply) {
				*_registrations[reply.registration_id].reply = reply;
			}

//			PX4_DEBUG("Registration id=%i: %i events", reply.registration_id, reply.num_events);
		}
	}

	if (_last_update > 0) {
		if (_reply_received_mask == _active_registrations_mask) { // Got all responses
			// Nothing to do
		} else if (now > _last_update + REQUEST_TIMEOUT && !_had_timeout) { // Timeout
			_had_timeout = true;
			unsigned no_reply = _active_registrations_mask & ~_reply_received_mask;

			for (int i = 0; i < MAX_NUM_REGISTRATIONS; ++i) {
				if ((1u << i) & no_reply) {
					const int max_num_no_reply =
						_registrations[i].waiting_for_first_response ? NUM_NO_REPLY_UNTIL_UNRESPONSIVE_INIT : NUM_NO_REPLY_UNTIL_UNRESPONSIVE;

					if (!_registrations[i].unresponsive && ++_registrations[i].num_no_response > max_num_no_reply) {
						// Clear immediately if not a mode
						if (_registrations[i].nav_mode_id == -1) {
							removeRegistration(i, -1);
							PX4_WARN("No response from %i, removing", i);

						} else {
							_registrations[i].unresponsive = true;

							if (_registrations[i].total_num_unresponsive < 100) {
								++_registrations[i].total_num_unresponsive;
							}

							PX4_WARN("No response from %i, flagging unresponsive", i);
						}
					}
				}
			}
		}
	}

	// Start a new request?
	if (now > _last_update + UPDATE_INTERVAL) {
		_reply_received_mask = 0;
		_last_update = now;
		_had_timeout = false;

		// Request the state from all registered components
		arming_check_request_s request{};
		request.request_id = ++_current_request_id;
		request.timestamp = hrt_absolute_time();
		_arming_check_request_pub.publish(request);
	}
}

void ExternalChecks::setExternalNavStates(uint8_t first_external_nav_state, uint8_t last_external_nav_state)
{
	_first_external_nav_state = first_external_nav_state;
	_last_external_nav_state = last_external_nav_state;
}

void ExternalChecks::checkNonRegisteredModes(const Context &context, Report &reporter) const
{
	// Clear the arming bits for all non-registered external modes.
	// But only report if one of them is selected, so we don't need to generate the extra event in most cases.
	bool report_mode_not_available = false;

	for (uint8_t external_nav_state = _first_external_nav_state; external_nav_state <= _last_external_nav_state;
	     ++external_nav_state) {
		bool found = false;

		for (int reg_idx = 0; reg_idx < MAX_NUM_REGISTRATIONS; ++reg_idx) {
			if (registrationValid(reg_idx) && _registrations[reg_idx].nav_mode_id == external_nav_state) {
				found = true;
				break;
			}
		}

		if (!found) {
			if (external_nav_state == context.status().nav_state_user_intention) {
				report_mode_not_available = true;
			}

			reporter.clearArmingBits(reporter.getModeGroup(external_nav_state));
			setOrClearRequirementBits(true, external_nav_state, -1, reporter.failsafeFlags().mode_req_other);
		}
	}

	if (report_mode_not_available) {
		/* EVENT
		 * @description
		 * The application running the mode is not started.
		 */
		reporter.armingCheckFailure(reporter.getModeGroup(context.status().nav_state_user_intention),
					    health_component_t::system,
					    events::ID("check_external_modes_unavailable"),
					    events::Log::Error, "Mode is not registered");
	}
}

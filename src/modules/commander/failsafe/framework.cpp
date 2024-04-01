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

#include "framework.h"
#define DEFINE_GET_PX4_CUSTOM_MODE
#include "../px4_custom_mode.h"

#include <uORB/topics/vehicle_status.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>

using failsafe_action_t = events::px4::enums::failsafe_action_t;
using failsafe_cause_t = events::px4::enums::failsafe_cause_t;

using namespace time_literals;

FailsafeBase::FailsafeBase(ModuleParams *parent) : ModuleParams(parent)
{
	_current_start_delay = _param_com_fail_act_t.get() * 1_s;
}

uint8_t FailsafeBase::update(const hrt_abstime &time_us, const State &state, bool user_intended_mode_updated,
			     bool rc_sticks_takeover_request,
			     const failsafe_flags_s &status_flags)
{
	if (_last_update == 0) {
		_last_update = time_us;
	}

	if ((_last_armed && !state.armed) || (!_last_armed && state.armed)) { // Disarming or Arming
		removeActions(ClearCondition::OnDisarm);
		removeActions(ClearCondition::OnModeChangeOrDisarm);
		_user_takeover_active = false;
	}

	user_intended_mode_updated |= _last_user_intended_mode != state.user_intended_mode;

	if (user_intended_mode_updated || _user_takeover_active) {
		removeActions(ClearCondition::OnModeChangeOrDisarm);
	}

	if (_defer_failsafes && _failsafe_defer_started != 0 && _defer_timeout > 0
	    && time_us > _failsafe_defer_started + _defer_timeout) {
		_defer_failsafes = false;
	}

	if (_failsafe_defer_started == 0) {
		updateDelay(time_us - _last_update);
	}

	checkStateAndMode(time_us, state, status_flags);
	removeNonActivatedActions();
	clearDelayIfNeeded(state, status_flags);

	SelectedActionState action_state{};
	getSelectedAction(state, status_flags, user_intended_mode_updated, rc_sticks_takeover_request, action_state);

	updateStartDelay(time_us - _last_update, action_state.delayed_action != Action::None);
	updateFailsafeDeferState(time_us, action_state.failsafe_deferred);

	// Notify user if the action is worse than before, or a new action got added
	if (action_state.action > _selected_action || (action_state.action != Action::None && _notification_required)) {
		notifyUser(state.user_intended_mode, action_state.action, action_state.delayed_action, action_state.cause);
	}

	_notification_required = false;

	_last_user_intended_mode = modifyUserIntendedMode(_selected_action, action_state.action,
				   action_state.updated_user_intended_mode);
	_user_takeover_active = action_state.user_takeover;
	_selected_action = action_state.action;
	_last_update = time_us;
	_last_status_flags = status_flags;
	_last_armed = state.armed;
	return _last_user_intended_mode;
}

void FailsafeBase::updateFailsafeDeferState(const hrt_abstime &time_us, bool defer)
{
	if (defer) {
		if (_failsafe_defer_started == 0) {
			_failsafe_defer_started = time_us;
		}

	} else {
		_failsafe_defer_started = 0;
	}
}

void FailsafeBase::updateStartDelay(const hrt_abstime &dt, bool delay_active)
{
	// Ensure that even with a toggling state the delayed action is executed at some point.
	// This is done by increasing the delay slower than reducing it.
	if (delay_active) {
		if (dt < _current_start_delay) {
			_current_start_delay -= dt;

		} else {
			_current_start_delay = 0;
		}

	} else {
		_current_start_delay += dt / 4;
		hrt_abstime configured_delay = _param_com_fail_act_t.get() * 1_s;

		if (_current_start_delay > configured_delay) {
			_current_start_delay = configured_delay;
		}
	}
}

void FailsafeBase::updateParams()
{
	ModuleParams::updateParams();
	_current_start_delay = _param_com_fail_act_t.get() * 1_s;
}

void FailsafeBase::updateDelay(const hrt_abstime &elapsed_us)
{
	if (_current_delay < elapsed_us) {
		_current_delay = 0;

	} else {
		_current_delay -= elapsed_us;
	}
}

void FailsafeBase::removeActions(ClearCondition condition)
{
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid() && !cur_action.state_failure && cur_action.clear_condition == condition) {
			PX4_DEBUG("Caller %i: clear condition triggered, removing", cur_action.id);
			cur_action.setInvalid();
		}
	}
}

void FailsafeBase::notifyUser(uint8_t user_intended_mode, Action action, Action delayed_action, Cause cause)
{
	int delay_s = (_current_delay + 500_ms) / 1_s;
	PX4_DEBUG("User notification: failsafe triggered (action=%i, delayed_action=%i, cause=%i, delay=%is)", (int)action,
		  (int)delayed_action, (int)cause, delay_s);

#ifdef EMSCRIPTEN_BUILD
	(void)_mavlink_log_pub;
#else

	px4_custom_mode custom_mode = get_px4_custom_mode(user_intended_mode);
	uint32_t mavlink_mode = custom_mode.data;

	static_assert((int)failsafe_cause_t::_max + 1 == (int)Cause::Count, "Enum needs to be extended");
	static_assert((int)failsafe_action_t::_max + 1 == (int)Action::Count, "Enum needs to be extended");
	failsafe_cause_t failsafe_cause = (failsafe_cause_t)cause;

	if (action == Action::Hold && delayed_action != Action::None) {
		failsafe_action_t failsafe_action = (failsafe_action_t)delayed_action;

		if (cause == Cause::Generic) {
			/* EVENT
			* @type append_health_and_arming_messages
			*/
			events::send<uint32_t, events::px4::enums::failsafe_action_t, uint16_t>(
				events::ID("commander_failsafe_enter_generic_hold"),
			{events::Log::Critical, events::LogInternal::Warning},
			"Failsafe activated: switching to {2} in {3} seconds", mavlink_mode, failsafe_action,
			(uint16_t) delay_s);

		} else {
			/* EVENT
			*/
			events::send<uint32_t, events::px4::enums::failsafe_action_t, uint16_t, events::px4::enums::failsafe_cause_t>(
				events::ID("commander_failsafe_enter_hold"),
			{events::Log::Critical, events::LogInternal::Warning},
			"{4}: switching to {2} in {3} seconds", mavlink_mode, failsafe_action,
			(uint16_t) delay_s, failsafe_cause);
		}

		mavlink_log_critical(&_mavlink_log_pub, "Failsafe activated: entering Hold for %i seconds\t", delay_s);

	} else { // no delay
		failsafe_action_t failsafe_action = (failsafe_action_t)action;

		if (cause == Cause::Generic) {
			if (action == Action::Warn) {
				/* EVENT
				* @description No action is triggered.
				* @type append_health_and_arming_messages
				*/
				events::send<uint32_t>(
					events::ID("commander_failsafe_enter_generic_warn"),
				{events::Log::Warning, events::LogInternal::Warning},
				"Failsafe warning:", mavlink_mode);

			} else {
				/* EVENT
				* @type append_health_and_arming_messages
				*/
				events::send<uint32_t, events::px4::enums::failsafe_action_t>(
					events::ID("commander_failsafe_enter_generic"),
				{events::Log::Critical, events::LogInternal::Warning},
				"Failsafe activated: Autopilot disengaged, switching to {2}", mavlink_mode, failsafe_action);
			}

		} else {
			if (action == Action::Warn) {
				if (cause == Cause::BatteryLow) {
					events::send(events::ID("commander_failsafe_enter_low_bat"),
					{events::Log::Warning, events::LogInternal::Info},
					"Low battery level, return advised");

				} else if (cause == Cause::BatteryCritical) {
					events::send(events::ID("commander_failsafe_enter_crit_bat_warn"),
					{events::Log::Critical, events::LogInternal::Info},
					"Critical battery level, land now");

				} else if (cause == Cause::BatteryEmergency) {
					events::send(events::ID("commander_failsafe_enter_crit_low_bat_warn"), {events::Log::Emergency, events::LogInternal::Info},
						     "Emergency battery level, land immediately");

				} else if (cause == Cause::RemainingFlightTimeLow) {
					events::send(events::ID("commander_failsafe_enter_low_flight_time_warn"),
					{events::Log::Warning, events::LogInternal::Info},
					"Low remaining flight time, return advised");

				} else {
					/* EVENT
					* @description No action is triggered.
					*/
					events::send<uint32_t, events::px4::enums::failsafe_cause_t>(
						events::ID("commander_failsafe_enter_warn"),
					{events::Log::Warning, events::LogInternal::Warning},
					"Failsafe warning: {2}", mavlink_mode, failsafe_cause);

				}

			} else { // action != Warn
				/* EVENT
				*/
				events::send<uint32_t, events::px4::enums::failsafe_action_t, events::px4::enums::failsafe_cause_t>(
					events::ID("commander_failsafe_enter"),
				{events::Log::Critical, events::LogInternal::Warning},
				"{3}: switching to {2}", mavlink_mode, failsafe_action, failsafe_cause);
			}
		}

		mavlink_log_critical(&_mavlink_log_pub, "Failsafe activated\t");
	}

#endif /* EMSCRIPTEN_BUILD */
}

bool FailsafeBase::checkFailsafe(int caller_id, bool last_state_failure, bool cur_state_failure,
				 const ActionOptions &options)
{
	if (cur_state_failure) {
		// Invalid state: find or add action
		int free_idx = -1;
		int found_idx = -1;

		for (int i = 0; i < max_num_actions; ++i) {
			if (!_actions[i].valid()) {
				free_idx = i;

			} else if (_actions[i].id == caller_id) {
				found_idx = i;
				break;
			}
		}

		if (found_idx != -1) {
			if (_actions[found_idx].activated && !_duplicate_reported_once) {
				PX4_ERR("BUG: duplicate check for caller_id %i", caller_id);
				_duplicate_reported_once = true;
			}

			_actions[found_idx].state_failure = true;
			_actions[found_idx].activated = true;
			_actions[found_idx].action = options.action; // Allow action to be updated, but keep the rest

			if (!last_state_failure) {
				PX4_DEBUG("Caller %i: state changed to failed, action already active", caller_id);
			}

		} else {
			if (free_idx == -1) {
				PX4_ERR("No free failsafe action idx");

				// replace based on action severity
				for (int i = 0; i < max_num_actions; ++i) {
					if (options.action > _actions[i].action) {
						free_idx = i;
					}
				}
			}

			if (free_idx != -1) {
				_actions[free_idx] = options;
				_actions[free_idx].id = caller_id;
				_actions[free_idx].state_failure = true;
				_actions[free_idx].activated = true;

				if (options.allow_user_takeover == UserTakeoverAllowed::Auto) {
					if (_param_com_fail_act_t.get() > 0.1f) {
						if (options.action != Action::Warn && _current_delay == 0) {
							_current_delay = _current_start_delay;
						}

						_actions[free_idx].allow_user_takeover = UserTakeoverAllowed::Always;

					} else {
						_actions[free_idx].allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly;
					}
				}

				if (options.action == Action::Warn) {
					_notification_required = true;
				}

				if (options.action >= Action::Hold) { // If not a Fallback
					_user_takeover_active = false; // Clear takeover
				}

				PX4_DEBUG("Caller %i: state changed to failed, adding action", caller_id);
			}
		}

	} else if (last_state_failure && !cur_state_failure) {
		// Invalid -> valid transition: remove action
		bool found = false;

		for (int i = 0; i < max_num_actions; ++i) {
			if (_actions[i].id == caller_id) {
				if (found) {
					PX4_ERR("Dup action with ID %i", caller_id);
				}

				removeAction(_actions[i]);
				found = true;
			}
		}

		// It's ok if we did not find the action, it might already have been removed due to not being activated
	}

	return cur_state_failure;
}

void FailsafeBase::removeAction(ActionOptions &action) const
{
	if (action.clear_condition == ClearCondition::WhenConditionClears) {
		// Remove action
		PX4_DEBUG("Caller %i: state changed to valid, removing action", action.id);
		action.setInvalid();

	} else {
		if (action.state_failure) {
			PX4_DEBUG("Caller %i: state changed to valid, keeping action", action.id);
		}

		// Keep action, just flag the state
		action.state_failure = false;
	}
}

void FailsafeBase::removeNonActivatedActions()
{
	// A non-activated action means the check was not called during the last update:
	// treat the state as valid and remove the action depending on the clear_condition
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid() && !cur_action.activated) {
			if (_actions[action_idx].state_failure) {
				PX4_DEBUG("Caller %i: action not activated", cur_action.id);
			}

			removeAction(cur_action);
		}

		cur_action.activated = false;
	}
}


void FailsafeBase::getSelectedAction(const State &state, const failsafe_flags_s &status_flags,
				     bool user_intended_mode_updated,
				     bool rc_sticks_takeover_request,
				     SelectedActionState &returned_state) const
{
	returned_state.updated_user_intended_mode = state.user_intended_mode;
	returned_state.cause = Cause::Generic;

	if (_selected_action == Action::Terminate) { // Terminate never clears
		returned_state.action = Action::Terminate;
		return;
	}

	if (!state.armed) {
		returned_state.action = Action::None;
		return;
	}

	returned_state.action = Action::None;
	Action &selected_action = returned_state.action;
	UserTakeoverAllowed allow_user_takeover = UserTakeoverAllowed::Always;
	bool allow_failsafe_to_be_deferred{true};

	// Select the worst action based on the current active actions
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		const ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid()) {
			if (cur_action.allow_user_takeover > allow_user_takeover) {
				// Use the most restrictive setting among all active actions
				allow_user_takeover = cur_action.allow_user_takeover;
			}

			if (cur_action.action > selected_action) {
				selected_action = cur_action.action;
				returned_state.cause = cur_action.cause;
			}

			if (!cur_action.can_be_deferred) {
				allow_failsafe_to_be_deferred = false;
			}
		}
	}

	if (_defer_failsafes && allow_failsafe_to_be_deferred && selected_action != Action::None) {
		returned_state.failsafe_deferred = selected_action > Action::Warn;
		returned_state.action = Action::None;
		return;
	}

	// Check if we should enter delayed Hold
	if (_current_delay > 0 && !_user_takeover_active && allow_user_takeover <= UserTakeoverAllowed::AlwaysModeSwitchOnly
	    && selected_action != Action::Disarm && selected_action != Action::Terminate) {
		returned_state.delayed_action = selected_action;
		selected_action = Action::Hold;
		allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly;
	}

	// User takeover is activated on user intented mode update (w/o action change, so takeover is not immediately
	// requested when entering failsafe) or rc stick movements
	bool want_user_takeover_mode_switch = user_intended_mode_updated && _selected_action == selected_action;
	bool want_user_takeover = want_user_takeover_mode_switch || rc_sticks_takeover_request;
	bool takeover_allowed =
		(allow_user_takeover == UserTakeoverAllowed::Always && (_user_takeover_active || want_user_takeover))
		|| (allow_user_takeover == UserTakeoverAllowed::AlwaysModeSwitchOnly && (_user_takeover_active
				|| want_user_takeover_mode_switch));

	if (actionAllowsUserTakeover(selected_action) && takeover_allowed) {
		if (!_user_takeover_active && rc_sticks_takeover_request) {
			// TODO: if the user intended mode is a stick-controlled mode, switch back to that instead
			returned_state.updated_user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		}

		selected_action = Action::Warn;
		returned_state.user_takeover = true;
		returned_state.delayed_action = Action::None;

		if (!_user_takeover_active) {
			PX4_DEBUG("Activating user takeover");
#ifndef EMSCRIPTEN_BUILD
			events::send(events::ID("failsafe_rc_override"), events::Log::Info, "Pilot took over using sticks");
#endif // EMSCRIPTEN_BUILD
		}

		// We must check for mode fallback again here
		Action mode_fallback = checkModeFallback(status_flags, modeFromAction(selected_action,
				       returned_state.updated_user_intended_mode));

		if (mode_fallback > selected_action) {
			selected_action = mode_fallback;
		}
	}

	// Check if the selected action is possible, and fall back if needed
	switch (selected_action) {

	case Action::FallbackPosCtrl:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_POSCTL)) {
			selected_action = Action::FallbackPosCtrl;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::FallbackAltCtrl:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_ALTCTL)) {
			selected_action = Action::FallbackAltCtrl;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::FallbackStab:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_STAB)) {
			selected_action = Action::FallbackStab;
			break;
		} // else: fall through here as well. If stabilized isn't available, we most certainly end up in Terminate

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Hold:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)) {
			selected_action = Action::Hold;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::RTL:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)) {
			selected_action = Action::RTL;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Land:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)) {
			selected_action = Action::Land;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Descend:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_DESCEND)) {
			selected_action = Action::Descend;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Terminate:
		selected_action = Action::Terminate;
		break;

	case Action::Disarm:
		selected_action = Action::Disarm;
		break;

	case Action::None:
	case Action::Warn:
	case Action::Count:
		break;
	}

	// UX improvement (this is optional for safety): change failsafe to a warning in certain situations.
	// If already landing, do not go into RTL
	if (returned_state.updated_user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		if ((selected_action == Action::RTL || returned_state.delayed_action == Action::RTL)
		    && modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)) {
			selected_action = Action::Warn;
			returned_state.delayed_action = Action::None;
		}
	}

	// If already precision landing, do not go into RTL or Land
	if (returned_state.updated_user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND) {
		if ((selected_action == Action::RTL || selected_action == Action::Land ||
		     returned_state.delayed_action == Action::RTL || returned_state.delayed_action == Action::Land)
		    && modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND)) {
			selected_action = Action::Warn;
			returned_state.delayed_action = Action::None;
		}
	}
}

bool FailsafeBase::actionAllowsUserTakeover(Action action) const
{
	// Stick-controlled modes do not need user takeover
	return action == Action::Hold || action == Action::RTL || action == Action::Land || action == Action::Descend;
}

void FailsafeBase::clearDelayIfNeeded(const State &state,
				      const failsafe_flags_s &status_flags)
{
	// Clear delay if one of the following is true:
	// - Already in a failsafe
	// - Hold not available
	// - Takeover is active (due to a mode switch during the delay)
	if (_selected_action > Action::Hold || !modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
	    || _user_takeover_active) {
		if (_current_delay > 0) {
			PX4_DEBUG("Clearing delay, Hold not available, already in failsafe or taken over");
		}

		_current_delay = 0;
	}
}

uint8_t FailsafeBase::modeFromAction(const Action &action, uint8_t user_intended_mode)
{
	switch (action) {

	case Action::FallbackPosCtrl: return vehicle_status_s::NAVIGATION_STATE_POSCTL;

	case Action::FallbackAltCtrl: return vehicle_status_s::NAVIGATION_STATE_ALTCTL;

	case Action::FallbackStab: return vehicle_status_s::NAVIGATION_STATE_STAB;

	case Action::Hold: return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	case Action::RTL: return vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

	case Action::Land: return vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

	case Action::Descend: return vehicle_status_s::NAVIGATION_STATE_DESCEND;

	case Action::Terminate:
	case Action::Disarm:
	case Action::None:
	case Action::Warn:
	case Action::Count:
		break;
	}

	return user_intended_mode;
}

bool FailsafeBase::modeCanRun(const failsafe_flags_s &status_flags, uint8_t mode)
{
	uint32_t mode_mask = 1u << mode;
	// mode_req_wind_and_flight_time_compliance: does not need to be handled here (these are separate failsafe triggers)
	// mode_req_manual_control: is handled separately
	return
		(!status_flags.angular_velocity_invalid || ((status_flags.mode_req_angular_velocity & mode_mask) == 0)) &&
		(!status_flags.attitude_invalid || ((status_flags.mode_req_attitude & mode_mask) == 0)) &&
		(!status_flags.local_position_invalid || ((status_flags.mode_req_local_position & mode_mask) == 0)) &&
		(!status_flags.local_position_invalid_relaxed || ((status_flags.mode_req_local_position_relaxed & mode_mask) == 0)) &&
		(!status_flags.global_position_invalid || ((status_flags.mode_req_global_position & mode_mask) == 0)) &&
		(!status_flags.local_altitude_invalid || ((status_flags.mode_req_local_alt & mode_mask) == 0)) &&
		(!status_flags.auto_mission_missing || ((status_flags.mode_req_mission & mode_mask) == 0)) &&
		(!status_flags.offboard_control_signal_lost || ((status_flags.mode_req_offboard_signal & mode_mask) == 0)) &&
		(!status_flags.home_position_invalid || ((status_flags.mode_req_home_position & mode_mask) == 0)) &&
		((status_flags.mode_req_other & mode_mask) == 0);
}

bool FailsafeBase::deferFailsafes(bool enabled, int timeout_s)
{
	if (enabled && _selected_action > Action::Warn) {
		return false;
	}

	if (timeout_s == 0) {
		_defer_timeout = DEFAULT_DEFER_TIMEOUT;

	} else if (timeout_s < 0) {
		_defer_timeout = 0;

	} else {
		_defer_timeout = timeout_s * 1_s;
	}

	_defer_failsafes = enabled;
	return true;
}

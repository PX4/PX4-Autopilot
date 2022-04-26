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

#include <lib/mathlib/mathlib.h>
#include <uORB/topics/vehicle_status.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>

using namespace time_literals;

FailsafeBase::FailsafeBase(ModuleParams *parent) : ModuleParams(parent)
{
	// TODO: change flags so that false == no failsafe
	_last_status_flags.battery_healthy = true;

	_current_delay = _param_com_bat_act_t.get() * 1_s;
}

uint8_t FailsafeBase::update(const hrt_abstime &time_us, const State &state, bool rc_sticks_takeover_request,
			     const vehicle_status_flags_s &status_flags)
{
	if (_last_update == 0) {
		_last_update = time_us;
	}

	if (_last_armed && !state.armed) {
		removeActions(ClearCondition::OnDisarm);
		removeActions(ClearCondition::OnModeChangeOrDisarm);
		_user_takeover_active = false;
	}

	if (_last_user_intended_mode != state.user_intended_mode || _user_takeover_active) {
		removeActions(ClearCondition::OnModeChangeOrDisarm);
	}

	updateActionDelays(time_us - _last_update);

	checkStateAndMode(time_us, state, status_flags);
	removeNonActivatedActions();
	clearDelayIfHoldNotAvailableOrInFailsafe(state);

	SelectedActionState action_state;
	getSelectedAction(state, rc_sticks_takeover_request, action_state);

	updateDelay(time_us - _last_update, action_state.delayed_action != Action::None);

	// Notify user if the action is worse than before
	if (action_state.action > _selected_action) {
		notifyUser(action_state.action, action_state.delayed_action, action_state.cause);
	}

	_last_user_intended_mode = modifyUserIntendedMode(_selected_action, action_state.action,
				   action_state.updated_user_intended_mode);
	_user_takeover_active = action_state.user_takeover;
	_selected_action = action_state.action;
	_last_update = time_us;
	_last_status_flags = status_flags;
	_last_armed = state.armed;
	return _last_user_intended_mode;
}

void FailsafeBase::updateDelay(const hrt_abstime &dt, bool delay_active)
{
	// Ensure that even with a toggling state the delayed action is executed at some point.
	// This is done by increasing the delay slower than reducing it.
	if (delay_active) {
		if (dt < _current_delay) {
			_current_delay -= dt;

		} else {
			_current_delay = 0;
		}

	} else {
		_current_delay += dt / 4;
		hrt_abstime configured_delay = _param_com_bat_act_t.get() * 1_s;

		if (_current_delay > configured_delay) {
			_current_delay = configured_delay;
		}
	}
}

void FailsafeBase::updateParams()
{
	ModuleParams::updateParams();
	_current_delay = _param_com_bat_act_t.get() * 1_s;
}

void FailsafeBase::updateActionDelays(const hrt_abstime &elapsed_us)
{
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid() && cur_action.delay > 0) {
			if (cur_action.delay < elapsed_us) {
				cur_action.delay = 0;

			} else {
				cur_action.delay -= elapsed_us;
			}
		}
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

void FailsafeBase::notifyUser(Action action, Action delayed_action, Cause cause)
{
	PX4_DEBUG("User notification: failsafe triggered (action=%i, delayed_action=%i, cause=%i)", (int)action,
		  (int)delayed_action, (int)cause);
#ifndef EMSCRIPTEN_BUILD

	// TODO: add action to report
	if (action == Action::Hold && delayed_action != Action::None) {
		/* EVENT
		* @description Entering Hold for {1} seconds before triggering the failsafe action.
		* @type append_health_and_arming_messages
		*/
		events::send<uint16_t>(events::ID("commander_failsafe_enter_generic_hold"), {events::Log::Critical, events::LogInternal::Critical},
				       "Failsafe activated:", (uint16_t)_param_com_bat_act_t.get());

	} else {
		switch (cause) {
		case Cause::Generic:
			/* EVENT
			* @type append_health_and_arming_messages
			*/
			events::send(events::ID("commander_failsafe_enter_generic"), {events::Log::Critical, events::LogInternal::Critical},
				     "Failsafe triggered:");
			break;

		case Cause::RCLoss:
			/* EVENT
			*/
			events::send(events::ID("commander_failsafe_enter_rcloss"), {events::Log::Critical, events::LogInternal::Info},
				     "Failsafe triggered due to RC loss");
			break;

		case Cause::DatalinkLoss:
			/* EVENT
			*/
			events::send(events::ID("commander_failsafe_enter_dlloss"), {events::Log::Critical, events::LogInternal::Info},
				     "Failsafe triggered due to datalink loss");
			break;
		}
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
					if (_param_com_bat_act_t.get() > 0.1f) {
						if (options.action != Action::Warn) {
							_actions[free_idx].delay = _current_delay;
						}

						_actions[free_idx].allow_user_takeover = UserTakeoverAllowed::Always;

					} else {
						_actions[free_idx].allow_user_takeover = UserTakeoverAllowed::Never;
					}
				}

				if (options.action >= Action::Hold) { // If not a Fallback
					_user_takeover_active = false; // Clear takeover if active
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


void FailsafeBase::getSelectedAction(const State &state, bool rc_sticks_takeover_request,
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
	bool allow_user_takeover = true;
	int delayed_action_idx = -1;

	// Select the worst action based on the current active actions
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		const ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid()) {
			allow_user_takeover &= cur_action.allow_user_takeover != UserTakeoverAllowed::Never;

			if (cur_action.delay == 0) {
				if (cur_action.action > selected_action) {
					selected_action = cur_action.action;
					returned_state.cause = cur_action.cause;
				}

			} else {
				delayed_action_idx = action_idx;
			}
		}
	}

	if (delayed_action_idx != -1 && selected_action <= Action::Hold && !_user_takeover_active) {
		selected_action = Action::Hold;
		allow_user_takeover = false;
		returned_state.delayed_action = _actions[delayed_action_idx].action;
		returned_state.cause = _actions[delayed_action_idx].cause;
	}

	// User takeover is activated on user intented mode change (w/o action change, so takeover is not immediately
	// requested when entering failsafe) or rc stick movements
	bool want_user_takeover = (_last_user_intended_mode != state.user_intended_mode && _selected_action == selected_action)
				  || rc_sticks_takeover_request;

	if (actionAllowsUserTakeover(selected_action) && allow_user_takeover && (_user_takeover_active || want_user_takeover)) {
		if (!_user_takeover_active && rc_sticks_takeover_request) {
			returned_state.updated_user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		}

		selected_action = Action::Warn;
		returned_state.user_takeover = true;

		if (!_user_takeover_active) {
			PX4_DEBUG("Activating user takeover");
		}

		// We must check for mode fallback again here
		Action mode_fallback = checkModeFallback(state.can_run_mode_flags, modeFromAction(selected_action,
				       returned_state.updated_user_intended_mode));

		if (mode_fallback > selected_action) {
			selected_action = mode_fallback;
		}
	}

	// Check if the selected action is possible, and fall back if needed
	switch (selected_action) {

	case Action::FallbackPosCtrl:
		if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_POSCTL)) {
			selected_action = Action::FallbackPosCtrl;
			break;
		}

	case Action::FallbackAltCtrl:
		if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_ALTCTL)) {
			selected_action = Action::FallbackAltCtrl;
			break;
		}

	case Action::FallbackStab:
		if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_STAB)) {
			selected_action = Action::FallbackStab;
			break;
		} // else: fall through here as well. If stabilized isn't available, we most certainly end up in Terminate

	case Action::Hold:
		if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)) {
			selected_action = Action::Hold;
			break;
		}

	case Action::RTL:
		if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)) {
			selected_action = Action::RTL;
			break;
		}

	case Action::Land:
		if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)) {
			selected_action = Action::Land;
			break;
		}

	case Action::Descend:
		if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_DESCEND)) {
			selected_action = Action::Descend;
			break;
		}

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
		    && modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)) {
			selected_action = Action::Warn;
			returned_state.delayed_action = Action::None;
		}
	}

	// If already precision landing, do not go into RTL or Land
	if (returned_state.updated_user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND) {
		if ((selected_action == Action::RTL || selected_action == Action::Land ||
		     returned_state.delayed_action == Action::RTL || returned_state.delayed_action == Action::Land)
		    && modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND)) {
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

void FailsafeBase::clearDelayIfHoldNotAvailableOrInFailsafe(const State &state)
{
	if (modeCanRun(state.can_run_mode_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
	    && _selected_action <= Action::Hold) {
		return;
	}

	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid() && cur_action.delay > 0) {
			cur_action.delay = 0;
			PX4_DEBUG("Caller %i: clearing delay, Hold not available, or already in failsafe", cur_action.id);
		}
	}
}

uint8_t FailsafeBase::modeFromAction(const Action &action, uint8_t user_intended_mode) const
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

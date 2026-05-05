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


#include "UserModeIntention.hpp"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>

UserModeIntention::UserModeIntention(const vehicle_status_s &vehicle_status,
				     const HealthAndArmingChecks &health_and_arming_checks, ModeChangeHandler *handler)
	: _vehicle_status(vehicle_status), _health_and_arming_checks(health_and_arming_checks),
	  _handler(handler)
{
	///////add by naor ////////////////
	_param_pos_wait_limit = param_find("COM_POS_WAIT_LIM");
	///////add by naor ////////////////
}

bool UserModeIntention::change(uint8_t user_intended_nav_state, ModeChangeSource source, bool allow_fallback,
			       bool force)
{
	_ever_had_mode_change = true;

	if (_handler) {
		// If a replacement mode is selected, select the internal one instead. The replacement will be selected after.
		user_intended_nav_state = _handler->getReplacedModeIfAny(user_intended_nav_state);
	}

	// Always allow mode change while disarmed
	bool always_allow = force || !isArmed();
	bool allow_change = true;

	if (!always_allow) {
		allow_change = _health_and_arming_checks.canRun(user_intended_nav_state);

		// Check fallback
		if (!allow_change && allow_fallback) {
			if (user_intended_nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL) {
				allow_change = _health_and_arming_checks.canRun(vehicle_status_s::NAVIGATION_STATE_ALTCTL);
				// We still use the original user intended mode. The failsafe state machine will then set the
				// fallback and once can_run becomes true, the actual user intended mode will be selected.
			}
		}

		///////add by naor ////////////////
		// If change failed because position is not yet valid, park the request.
		// tick() will retry once position has been stable for POS_STABLE_THRESHOLD iterations.
		if (!allow_change && modeRequiresPosition(user_intended_nav_state)) {
			_pending_nav_state = user_intended_nav_state;
			_pos_wait_count    = 0;
			int32_t limit = 10;
			param_get(_param_pos_wait_limit, &limit);
			PX4_INFO("Mode %d requires position - waiting up to %d iterations for solution", user_intended_nav_state, (int)limit);
			// Signal immediately that we are trying to enter a position mode.
			publish_formic_state_machine(true);
		}
		///////add by naor ////////////////
	}

	// never allow to change out of termination state
	allow_change &= _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_TERMINATION;

	if (allow_change) {
		_had_mode_change = true;
		_user_intented_nav_state = user_intended_nav_state;

		///////add by naor ////////////////
		// A successful explicit change cancels any pending position-wait.
		_pending_nav_state = UINT8_MAX;
		_pos_wait_count    = 0;

		// Publish let_update_ev: true when entering a position mode, false otherwise.
		publish_formic_state_machine(modeRequiresPosition(user_intended_nav_state));
		///////add by naor ////////////////

		// Special case termination state: even though this mode prevents arming,
		// still don't switch out of it after disarm and thus store it in _nav_state_after_disarming.
		if (!_health_and_arming_checks.modePreventsArming(user_intended_nav_state)
		    || user_intended_nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION) {
			_nav_state_after_disarming = user_intended_nav_state;
		}

		if (_handler) {
			_handler->onUserIntendedNavStateChange(source, user_intended_nav_state);
		}
	}

	return allow_change;
}

///////add by naor ////////////////
bool UserModeIntention::modeRequiresPosition(uint8_t nav_state) const
{
	const failsafe_flags_s &flags = _health_and_arming_checks.failsafeFlags();
	const uint32_t bit = 1u << nav_state;
	return (flags.mode_req_local_position & bit) || (flags.mode_req_global_position & bit)
	       || (flags.mode_req_local_position_relaxed & bit) || (flags.mode_req_global_position_relaxed & bit);
}

void UserModeIntention::tick()
{
	///////add by naor ////////////////
	if (_ev_yaw_sub.updated()) {
		_ev_yaw_sub.copy(&_ev_yaw);
	}

	const bool ev_yaw_available = (_ev_yaw.timestamp != 0);

	if (_pending_nav_state == UINT8_MAX) {
		return;
	}

	const bool pos_ok = _health_and_arming_checks.canRun(_pending_nav_state);

	if (pos_ok && ev_yaw_available && _ev_yaw.fused) {
		PX4_INFO("Position available - switching to pending mode %d after %d iterations", _pending_nav_state, _pos_wait_count);
		const uint8_t mode = _pending_nav_state;
		_pending_nav_state = UINT8_MAX;

		_pos_wait_count    = 0;
		change(mode, ModeChangeSource::User, false, true);
		publish_formic_state_machine(true);

	} else {
		_pos_wait_count++;
		publish_formic_state_machine(true);

		int32_t limit = 200;
		param_get(_param_pos_wait_limit, &limit);

		if (_pos_wait_count >= (int)limit) {
			_pending_nav_state = UINT8_MAX;
			_pos_wait_count    = 0;
			change(vehicle_status_s::NAVIGATION_STATE_ALTCTL, ModeChangeSource::User, false, true);
			publish_formic_state_machine(false);
		}
	}
}


void UserModeIntention::onFailsafeNavState(uint8_t actual_nav_state)
{
	// If the failsafe forced the drone into a non-position mode (e.g. ALTCTL) while
	// the user intention is still a position mode, we must signal that EV updates
	// are no longer useful and cancel any pending position-wait.
	if (!modeRequiresPosition(actual_nav_state)) {
		_pending_nav_state = UINT8_MAX;
		_pos_wait_count    = 0;
		publish_formic_state_machine(false);
	}
}

void UserModeIntention::publish_formic_state_machine(bool let_update_ev)
{
	formic_state_machine_s formic_state_machine{};
	formic_state_machine.timestamp = hrt_absolute_time();
	formic_state_machine.let_update_ev = let_update_ev;
	_formic_state_machine_pub.publish(formic_state_machine);
}
///////add by naor ////////////////

void UserModeIntention::onDisarm()
{
	if (_handler) {
		_user_intented_nav_state = _handler->onDisarm(_nav_state_after_disarming);

	} else {
		_user_intented_nav_state = _nav_state_after_disarming;
	}
}

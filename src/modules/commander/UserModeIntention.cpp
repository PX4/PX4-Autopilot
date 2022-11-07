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

UserModeIntention::UserModeIntention(ModuleParams *parent, const vehicle_status_s &vehicle_status,
				     const HealthAndArmingChecks &health_and_arming_checks, ModeChangeHandler *handler)
	: ModuleParams(parent), _vehicle_status(vehicle_status), _health_and_arming_checks(health_and_arming_checks),
	  _handler(handler)
{
}

bool UserModeIntention::change(uint8_t user_intended_nav_state, ModeChangeSource source, bool allow_fallback,
			       bool force)
{
	_ever_had_mode_change = true;
	_had_mode_change = true;

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
		if (!allow_change && allow_fallback && _param_com_posctl_navl.get() == 0) {
			if (user_intended_nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL) {
				allow_change = _health_and_arming_checks.canRun(vehicle_status_s::NAVIGATION_STATE_ALTCTL);
				// We still use the original user intended mode. The failsafe state machine will then set the
				// fallback and once can_run becomes true, the actual user intended mode will be selected.
			}
		}
	}

	if (allow_change) {
		_had_mode_change = true;
		_user_intented_nav_state = user_intended_nav_state;

		if (!_health_and_arming_checks.modePreventsArming(user_intended_nav_state)) {
			_nav_state_after_disarming = user_intended_nav_state;
		}

		if (_handler) {
			_handler->onUserIntendedNavStateChange(source, user_intended_nav_state);
		}
	}

	return allow_change;
}

void UserModeIntention::onDisarm()
{
	_user_intented_nav_state = _nav_state_after_disarming;
}

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

#include "failsafe.h"

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

FailsafeBase::ActionOptions Failsafe::fromNavDllOrRclActParam(int param_value) const
{
	ActionOptions options{};

	switch (param_value) {
	case 0:
		options.action = Action::None;
		break;

	case 1:
		options.action = Action::Hold;
		break;

	case 2:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 3:
		options.action = Action::Land;
		break;

	case 5:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case 6: // Lockdown
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Disarm;
		break;

	default:
		options.action = Action::None;
		break;
	}

	return options;
}

FailsafeBase::ActionOptions Failsafe::fromGfActParam(int param_value) const
{
	ActionOptions options{};

	switch (param_value) {
	case 0:
		options.action = Action::None;
		break;

	case 1:
		options.action = Action::Warn;
		break;

	case 2:
		options.allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly; // ensure the user can escape again
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 3:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 4:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case 5:
		options.action = Action::Land;
		break;

	default:
		options.action = Action::Warn;
		break;
	}

	return options;
}

void Failsafe::checkStateAndMode(const hrt_abstime &time_us, const State &state,
				 const vehicle_status_flags_s &status_flags)
{
	const bool in_forward_flight = state.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				       || state.vtol_in_transition_mode;

	// Do not enter failsafe while doing a vtol takeoff after the vehicle has started a transition and before it reaches the loiter
	// altitude. The vtol takeoff navigaton mode will set mission_finished to true as soon as the loiter is established
	const bool ignore_link_failsafe = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF
					  && in_forward_flight && !state.mission_finished;

	// RC loss
	const bool rc_loss_ignored_mission = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
					     && (_param_com_rcl_except.get() & (int)RCLossExceptionBits::Mission);
	const bool rc_loss_ignored_loiter = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
					    && (_param_com_rcl_except.get() & (int)RCLossExceptionBits::Hold);
	const bool rc_loss_ignored_offboard = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
					      && (_param_com_rcl_except.get() & (int)RCLossExceptionBits::Offboard);
	const bool rc_loss_ignored_takeoff = (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
					      state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF)
					     && (_param_com_rcl_except.get() & (int)RCLossExceptionBits::Hold);
	const bool rc_loss_ignored = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL ||
				     state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND ||
				     state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND ||
				     rc_loss_ignored_mission || rc_loss_ignored_loiter || rc_loss_ignored_offboard || rc_loss_ignored_takeoff ||
				     ignore_link_failsafe;

	if (_param_com_rc_in_mode.get() != 4 && !rc_loss_ignored) {
		CHECK_FAILSAFE(status_flags, rc_signal_lost,
			       fromNavDllOrRclActParam(_param_nav_rcl_act.get()).causedBy(Cause::RCLoss));
	}

	// Datalink loss
	const bool data_link_loss_ignored = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND ||
					    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND || ignore_link_failsafe;

	if (_param_nav_dll_act.get() != 0 && !data_link_loss_ignored) {
		CHECK_FAILSAFE(status_flags, data_link_lost,
			       fromNavDllOrRclActParam(_param_nav_dll_act.get()).causedBy(Cause::DatalinkLoss));
	}

	// VTOL transition failure
	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF) {
		CHECK_FAILSAFE(status_flags, vtol_transition_failure, Action::RTL);
	}

	// Mission
	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
		CHECK_FAILSAFE(status_flags, mission_failure, Action::RTL);

		// If RC loss and datalink loss are disabled and we lose both command links and the mission finished,
		// trigger RTL to avoid losing the vehicle
		if ((_param_com_rc_in_mode.get() == 4 || rc_loss_ignored_mission) && _param_nav_dll_act.get() == 0
		    && state.mission_finished) {
			_last_state_mission_control_lost = checkFailsafe(_caller_id_mission_control_lost, _last_state_mission_control_lost,
							   status_flags.data_link_lost, Action::RTL);
		}
	}




	CHECK_FAILSAFE(status_flags, geofence_violated, fromGfActParam(_param_gf_action.get()));


	// Mode fallback (last)
	Action mode_fallback_action = checkModeFallback(status_flags, state.user_intended_mode);
	_last_state_mode_fallback = checkFailsafe(_caller_id_mode_fallback, _last_state_mode_fallback,
				    mode_fallback_action != Action::None,
				    ActionOptions(mode_fallback_action).allowUserTakeover(UserTakeoverAllowed::Always));
}

FailsafeBase::Action Failsafe::checkModeFallback(const vehicle_status_flags_s &status_flags,
		uint8_t user_intended_mode) const
{
	Action action = Action::None;

	// offboard signal
	if (status_flags.offboard_control_signal_lost && (status_flags.mode_req_offboard_signal & (1u << user_intended_mode))) {
		action = Action::FallbackPosCtrl; // TODO: use param
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	}

	// posctrl
	switch (_param_com_posctl_navl.get()) {
	case 0: // AltCtrl/Manual

		// PosCtrl -> AltCtrl
		if (user_intended_mode == vehicle_status_s::NAVIGATION_STATE_POSCTL
		    && !modeCanRun(status_flags, user_intended_mode)) {
			action = Action::FallbackAltCtrl;
			user_intended_mode = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
		}

		// AltCtrl -> Stabilized
		if (user_intended_mode == vehicle_status_s::NAVIGATION_STATE_ALTCTL
		    && !modeCanRun(status_flags, user_intended_mode)) {
			action = Action::FallbackStab;
			user_intended_mode = vehicle_status_s::NAVIGATION_STATE_STAB;
		}

		break;

	case 1: // Land/Terminate

		// PosCtrl -> Land
		if (user_intended_mode == vehicle_status_s::NAVIGATION_STATE_POSCTL
		    && !modeCanRun(status_flags, user_intended_mode)) {
			action = Action::Land;
			user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			// Land -> Descend
			if (!modeCanRun(status_flags, user_intended_mode)) {
				action = Action::Descend;
				user_intended_mode = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			}
		}

		break;
	}


	// Last, check can_run for intended mode
	if (!modeCanRun(status_flags, user_intended_mode)) {
		action = Action::RTL;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	}

	return action;
}

uint8_t Failsafe::modifyUserIntendedMode(Action previous_action, Action current_action,
		uint8_t user_intended_mode) const
{
	// If we switch from a failsafe back into orbit, switch to loiter instead
	if ((int)previous_action > (int)Action::Warn
	    && modeFromAction(current_action, user_intended_mode) == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		PX4_DEBUG("Failsafe cleared, switching from ORBIT to LOITER");
		return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	}

	return user_intended_mode;
}

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
#include <uORB/topics/battery_status.h>
#include <lib/circuit_breaker/circuit_breaker.h>

using namespace time_literals;

FailsafeBase::ActionOptions Failsafe::fromNavDllOrRclActParam(int param_value)
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

FailsafeBase::ActionOptions Failsafe::fromGfActParam(int param_value)
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

FailsafeBase::ActionOptions Failsafe::fromImbalancedPropActParam(int param_value)
{
	ActionOptions options{};

	switch (param_value) {
	case -1:
	default:
		options.action = Action::None;
		break;

	case 0:
		options.action = Action::Warn;
		break;

	case 1:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 2:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

FailsafeBase::ActionOptions Failsafe::fromActuatorFailureActParam(int param_value)
{
	ActionOptions options{};

	switch (param_value) {
	case 0:
	default:
		options.action = Action::Warn;
		break;

	case 1:
		options.action = Action::Hold;
		break;

	case 2:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 3:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 4:
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;
	}

	return options;
}

FailsafeBase::ActionOptions Failsafe::fromBatteryWarningActParam(int param_value, uint8_t battery_warning)
{
	ActionOptions options{};

	switch (battery_warning) {
	default:
	case battery_status_s::BATTERY_WARNING_NONE:
		options.action = Action::None;
		break;

	case battery_status_s::BATTERY_WARNING_LOW:
		options.action = Action::Warn;
		options.cause = Cause::BatteryLow;
		break;

	case battery_status_s::BATTERY_WARNING_CRITICAL:
		options.action = Action::Warn;
		options.cause = Cause::BatteryCritical;

		switch ((LowBatteryAction)param_value) {
		case LowBatteryAction::Return:
		case LowBatteryAction::ReturnOrLand:
			options.action = Action::RTL;
			break;

		case LowBatteryAction::Land:
			options.action = Action::Land;
			break;

		case LowBatteryAction::Warning:
			options.action = Action::Warn;
			break;
		}

		break;

	case battery_status_s::BATTERY_WARNING_EMERGENCY:
		options.action = Action::Warn;
		options.cause = Cause::BatteryEmergency;

		switch ((LowBatteryAction)param_value) {
		case LowBatteryAction::Return:
			options.action = Action::RTL;
			break;

		case LowBatteryAction::ReturnOrLand:
		case LowBatteryAction::Land:
			options.action = Action::Land;
			break;

		case LowBatteryAction::Warning:
			options.action = Action::Warn;
			break;
		}

		break;
	}

	return options;
}

FailsafeBase::ActionOptions Failsafe::fromQuadchuteActParam(int param_value)
{
	ActionOptions options{};

	switch (param_value) {
	case -1:
	default:
		options.action = Action::Warn;
		break;

	case 0:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 1:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case 2:
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

FailsafeBase::Action Failsafe::fromOffboardLossActParam(int param_value, uint8_t &user_intended_mode)
{
	Action action{Action::None};

	switch (param_value) {
	case 0:
	default:
		action = Action::FallbackPosCtrl;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		break;

	case 1:
		action = Action::FallbackAltCtrl;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
		break;

	case 2:
		action = Action::FallbackStab;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_STAB;
		break;

	case 3:
		action = Action::RTL;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		break;

	case 4:
		action = Action::Land;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		break;

	case 5:
		action = Action::Hold;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		break;

	case 6:
		action = Action::Terminate;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		break;

	case 7:
		action = Action::Disarm;
		break;
	}

	return action;
}

void Failsafe::checkStateAndMode(const hrt_abstime &time_us, const State &state,
				 const vehicle_status_flags_s &status_flags)
{
	updateArmingState(time_us, state.armed, status_flags);

	const bool in_forward_flight = state.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				       || state.vtol_in_transition_mode;

	// Do not enter failsafe while doing a vtol takeoff after the vehicle has started a transition and before it reaches the loiter
	// altitude. The vtol takeoff navigaton mode will set mission_finished to true as soon as the loiter is established
	const bool ignore_link_failsafe = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF
					  && in_forward_flight && !state.mission_finished;

	// RC loss
	if (!status_flags.rc_signal_lost) {
		// If RC was lost and arming was allowed, consider it optional until we regain RC
		_rc_lost_at_arming = false;
	}

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
				     ignore_link_failsafe || _rc_lost_at_arming;

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

	// VTOL transition failure (quadchute)
	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF) {
		CHECK_FAILSAFE(status_flags, vtol_transition_failure, fromQuadchuteActParam(_param_com_qc_act.get()));
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


	CHECK_FAILSAFE(status_flags, wind_limit_exceeded,
		       ActionOptions(Action::RTL).clearOn(ClearCondition::OnModeChangeOrDisarm));
	CHECK_FAILSAFE(status_flags, flight_time_limit_exceeded,
		       ActionOptions(Action::RTL).allowUserTakeover(UserTakeoverAllowed::Never));

	CHECK_FAILSAFE(status_flags, geofence_violated, fromGfActParam(_param_gf_action.get()));

	// Battery
	CHECK_FAILSAFE(status_flags, battery_low_remaining_time, ActionOptions(Action::RTL).causedBy(Cause::BatteryLow));
	CHECK_FAILSAFE(status_flags, battery_unhealthy, Action::Warn);

	if (status_flags.battery_warning == battery_status_s::BATTERY_WARNING_LOW) {
		_last_state_battery_warning_low = checkFailsafe(_caller_id_battery_warning_low, _last_state_battery_warning_low,
						  true, fromBatteryWarningActParam(_param_com_low_bat_act.get(), battery_status_s::BATTERY_WARNING_LOW));

	} else if (status_flags.battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
		_last_state_battery_warning_critical = checkFailsafe(_caller_id_battery_warning_critical,
						       _last_state_battery_warning_critical,
						       true, fromBatteryWarningActParam(_param_com_low_bat_act.get(), battery_status_s::BATTERY_WARNING_CRITICAL));

	} else if (status_flags.battery_warning == battery_status_s::BATTERY_WARNING_EMERGENCY) {
		_last_state_battery_warning_emergency = checkFailsafe(_caller_id_battery_warning_emergency,
							_last_state_battery_warning_emergency,
							true, fromBatteryWarningActParam(_param_com_low_bat_act.get(), battery_status_s::BATTERY_WARNING_EMERGENCY));
	}


	// Failure detector
	if (_armed_time != 0 && time_us - _armed_time < _param_com_spoolup_time.get() * 1_s) {
		CHECK_FAILSAFE(status_flags, fd_esc_arming_failure, Action::Disarm);
	}

	if (_armed_time != 0 && time_us - _armed_time < (_param_com_lkdown_tko.get() + _param_com_spoolup_time.get()) * 1_s) {
		// This handles the case where something fails during the early takeoff phase
		CHECK_FAILSAFE(status_flags, fd_critical_failure, Action::Disarm);

	} else if (!circuit_breaker_enabled_by_val(_param_cbrk_flightterm.get(), CBRK_FLIGHTTERM_KEY)) {
		CHECK_FAILSAFE(status_flags, fd_critical_failure, Action::Terminate);
	}

	CHECK_FAILSAFE(status_flags, fd_imbalanced_prop, fromImbalancedPropActParam(_param_com_imb_prop_act.get()));
	CHECK_FAILSAFE(status_flags, fd_motor_failure, fromActuatorFailureActParam(_param_com_actuator_failure_act.get()));



	// Mode fallback (last)
	Action mode_fallback_action = checkModeFallback(status_flags, state.user_intended_mode);
	_last_state_mode_fallback = checkFailsafe(_caller_id_mode_fallback, _last_state_mode_fallback,
				    mode_fallback_action != Action::None,
				    ActionOptions(mode_fallback_action).allowUserTakeover(UserTakeoverAllowed::Always));
}

void Failsafe::updateArmingState(const hrt_abstime &time_us, bool armed, const vehicle_status_flags_s &status_flags)
{
	if (!_was_armed && armed) {
		_armed_time = time_us;
		_rc_lost_at_arming = status_flags.rc_signal_lost;

	} else if (!armed) {
		_rc_lost_at_arming = status_flags.rc_signal_lost; // ensure action isn't added while disarmed
		_armed_time = 0;
	}

	_was_armed = armed;
}

FailsafeBase::Action Failsafe::checkModeFallback(const vehicle_status_flags_s &status_flags,
		uint8_t user_intended_mode) const
{
	Action action = Action::None;

	// offboard signal
	if (status_flags.offboard_control_signal_lost && (status_flags.mode_req_offboard_signal & (1u << user_intended_mode))) {
		action = fromOffboardLossActParam(_param_com_obl_rc_act.get(), user_intended_mode);
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

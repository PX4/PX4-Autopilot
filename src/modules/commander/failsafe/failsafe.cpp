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

	switch (gcs_connection_loss_failsafe_mode(param_value)) {
	case gcs_connection_loss_failsafe_mode::Disabled:
		options.action = Action::None;
		break;

	case gcs_connection_loss_failsafe_mode::Hold_mode:
		options.action = Action::Hold;
		break;

	case gcs_connection_loss_failsafe_mode::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case gcs_connection_loss_failsafe_mode::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case gcs_connection_loss_failsafe_mode::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case gcs_connection_loss_failsafe_mode::Disarm: // Lockdown
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

	switch (geofence_violation_action(param_value)) {
	case geofence_violation_action::None:
		options.action = Action::None;
		break;

	case geofence_violation_action::Warning:
		options.action = Action::Warn;
		break;

	case geofence_violation_action::Hold_mode:
		options.allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly; // ensure the user can escape again
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case geofence_violation_action::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case geofence_violation_action::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case geofence_violation_action::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
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

	switch (imbalanced_propeller_failsafe_mode(param_value)) {
	case imbalanced_propeller_failsafe_mode::Disabled:
	default:
		options.action = Action::None;
		break;

	case imbalanced_propeller_failsafe_mode::Warning:
		options.action = Action::Warn;
		break;

	case imbalanced_propeller_failsafe_mode::Return:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case imbalanced_propeller_failsafe_mode::Land:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

FailsafeBase::ActionOptions Failsafe::fromActuatorFailureActParam(int param_value)
{
	ActionOptions options{};

	switch (actuator_failure_failsafe_mode(param_value)) {
	case actuator_failure_failsafe_mode::Warning_only:
	default:
		options.action = Action::Warn;
		break;

	case actuator_failure_failsafe_mode::Hold_mode:
		options.action = Action::Hold;
		break;

	case actuator_failure_failsafe_mode::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case actuator_failure_failsafe_mode::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case actuator_failure_failsafe_mode::Terminate:
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

	switch (command_after_quadchute(param_value)) {
	case command_after_quadchute::Warning_only:
	default:
		options.action = Action::Warn;
		break;

	case command_after_quadchute::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_quadchute::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_quadchute::Hold_mode:
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

FailsafeBase::Action Failsafe::fromOffboardLossActParam(int param_value, uint8_t &user_intended_mode)
{
	Action action{Action::None};

	switch (offboard_loss_failsafe_mode(param_value)) {
	case offboard_loss_failsafe_mode::Position_mode:
	default:
		action = Action::FallbackPosCtrl;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		break;

	case offboard_loss_failsafe_mode::Altitude_mode:
		action = Action::FallbackAltCtrl;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
		break;

	case offboard_loss_failsafe_mode::Stabilized:
		action = Action::FallbackStab;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_STAB;
		break;

	case offboard_loss_failsafe_mode::Return_mode:
		action = Action::RTL;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		break;

	case offboard_loss_failsafe_mode::Land_mode:
		action = Action::Land;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		break;

	case offboard_loss_failsafe_mode::Hold_mode:
		action = Action::Hold;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		break;

	case offboard_loss_failsafe_mode::Terminate:
		action = Action::Terminate;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		break;

	case offboard_loss_failsafe_mode::Disarm:
		action = Action::Disarm;
		break;
	}

	return action;
}

FailsafeBase::ActionOptions Failsafe::fromHighWindLimitActParam(int param_value)
{
	ActionOptions options{};

	switch (command_after_high_wind_failsafe(param_value)) {
	case command_after_high_wind_failsafe::None:
		options.action = Action::None;
		break;

	case command_after_high_wind_failsafe::Warning:
		options.action = Action::Warn;
		break;

	case command_after_high_wind_failsafe::Hold_mode:
		options.allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly; // ensure the user can escape again
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_high_wind_failsafe::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_high_wind_failsafe::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case command_after_high_wind_failsafe::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	default:
		options.action = Action::Warn;
		break;
	}

	return options;
}

FailsafeBase::ActionOptions Failsafe::fromPosLowActParam(int param_value)
{
	ActionOptions options{};
	options.allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly; // ensure the user can escape again

	switch (command_after_pos_low_failsafe(param_value)) {
	case command_after_pos_low_failsafe::None:
		options.action = Action::None;
		break;

	case command_after_pos_low_failsafe::Warning:
		options.action = Action::Warn;
		break;

	case command_after_pos_low_failsafe::Hold_mode:
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::WhenConditionClears;
		break;

	case command_after_pos_low_failsafe::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::WhenConditionClears;
		break;

	case command_after_pos_low_failsafe::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case command_after_pos_low_failsafe::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::WhenConditionClears;
		break;

	default:
		options.action = Action::Warn;
		break;
	}

	return options;
}

FailsafeBase::ActionOptions Failsafe::fromRemainingFlightTimeLowActParam(int param_value)
{
	ActionOptions options{};

	options.allow_user_takeover = UserTakeoverAllowed::Auto;
	options.cause = Cause::RemainingFlightTimeLow;

	switch (command_after_remaining_flight_time_low(param_value)) {
	case command_after_remaining_flight_time_low::None:
		options.action = Action::None;
		break;

	case command_after_remaining_flight_time_low::Warning:
		options.action = Action::Warn;
		break;

	case command_after_remaining_flight_time_low::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	default:
		options.action = Action::None;
		break;

	}

	return options;
}

void Failsafe::checkStateAndMode(const hrt_abstime &time_us, const State &state,
				 const failsafe_flags_s &status_flags)
{
	updateArmingState(time_us, state.armed, status_flags);

	const bool in_forward_flight = state.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				       || state.vtol_in_transition_mode;

	// Do not enter failsafe while doing a vtol takeoff after the vehicle has started a transition and before it reaches the loiter
	// altitude. The vtol takeoff navigaton mode will set mission_finished to true as soon as the loiter is established
	const bool ignore_link_failsafe = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF
					  && in_forward_flight && !state.mission_finished;

	// Manual control (RC) loss
	if (!status_flags.manual_control_signal_lost) {
		// If manual control was lost and arming was allowed, consider it optional until we regain manual control
		_manual_control_lost_at_arming = false;
	}

	const bool rc_loss_ignored_mission = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
					     && (_param_com_rcl_except.get() & (int)ManualControlLossExceptionBits::Mission);
	const bool rc_loss_ignored_loiter = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
					    && (_param_com_rcl_except.get() & (int)ManualControlLossExceptionBits::Hold);
	const bool rc_loss_ignored_offboard = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
					      && (_param_com_rcl_except.get() & (int)ManualControlLossExceptionBits::Offboard);
	const bool rc_loss_ignored_takeoff = (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
					      state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF)
					     && (_param_com_rcl_except.get() & (int)ManualControlLossExceptionBits::Hold);
	const bool rc_loss_ignored = rc_loss_ignored_mission || rc_loss_ignored_loiter || rc_loss_ignored_offboard ||
				     rc_loss_ignored_takeoff || ignore_link_failsafe || _manual_control_lost_at_arming;

	if (_param_com_rc_in_mode.get() != int32_t(RcInMode::StickInputDisabled) && !rc_loss_ignored) {
		CHECK_FAILSAFE(status_flags, manual_control_signal_lost,
			       fromNavDllOrRclActParam(_param_nav_rcl_act.get()).causedBy(Cause::ManualControlLoss));
	}

	// GCS connection loss
	const bool gcs_connection_loss_ignored = state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND ||
			state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND || ignore_link_failsafe;

	if (_param_nav_dll_act.get() != int32_t(gcs_connection_loss_failsafe_mode::Disabled) && !gcs_connection_loss_ignored) {
		CHECK_FAILSAFE(status_flags, gcs_connection_lost,
			       fromNavDllOrRclActParam(_param_nav_dll_act.get()).causedBy(Cause::GCSConnectionLoss));
	}

	// VTOL transition failure (quadchute)
	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF) {
		CHECK_FAILSAFE(status_flags, vtol_fixed_wing_system_failure, fromQuadchuteActParam(_param_com_qc_act.get()));
	}

	// Mission
	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
		CHECK_FAILSAFE(status_flags, mission_failure, Action::RTL);

		// If manual control loss and GCS connection loss are disabled and we lose both command links and the mission finished,
		// trigger RTL to avoid losing the vehicle
		if ((_param_com_rc_in_mode.get() == int32_t(RcInMode::StickInputDisabled) || rc_loss_ignored_mission)
		    && _param_nav_dll_act.get() == int32_t(gcs_connection_loss_failsafe_mode::Disabled)
		    && state.mission_finished) {
			_last_state_mission_control_lost = checkFailsafe(_caller_id_mission_control_lost, _last_state_mission_control_lost,
							   status_flags.gcs_connection_lost, Action::RTL);
		}
	}

	CHECK_FAILSAFE(status_flags, wind_limit_exceeded,
		       ActionOptions(fromHighWindLimitActParam(_param_com_wind_max_act.get()).cannotBeDeferred()));
	CHECK_FAILSAFE(status_flags, flight_time_limit_exceeded, ActionOptions(Action::RTL).cannotBeDeferred());

	// trigger Low Position Accuracy Failsafe (only in auto mission and auto loiter)
	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
		CHECK_FAILSAFE(status_flags, local_position_accuracy_low, fromPosLowActParam(_param_com_pos_low_act.get()));
	}

	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
	    state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		CHECK_FAILSAFE(status_flags, navigator_failure,
			       ActionOptions(Action::Land).clearOn(ClearCondition::OnModeChangeOrDisarm));

	} else {
		CHECK_FAILSAFE(status_flags, navigator_failure,
			       ActionOptions(Action::Hold).clearOn(ClearCondition::OnModeChangeOrDisarm));
	}

	CHECK_FAILSAFE(status_flags, geofence_breached, fromGfActParam(_param_gf_action.get()).cannotBeDeferred());

	// Battery flight time remaining failsafe
	CHECK_FAILSAFE(status_flags, battery_low_remaining_time,
		       ActionOptions(fromRemainingFlightTimeLowActParam(_param_com_fltt_low_act.get())));

	if ((_armed_time != 0)
	    && (time_us < _armed_time + static_cast<hrt_abstime>(_param_com_spoolup_time.get() * 1_s))
	   ) {
		CHECK_FAILSAFE(status_flags, battery_unhealthy, ActionOptions(Action::Disarm).cannotBeDeferred());

	} else {
		CHECK_FAILSAFE(status_flags, battery_unhealthy, Action::Warn);
	}

	// Battery low failsafe
	// If battery was low and arming was allowed through COM_ARM_BAT_MIN, don't failsafe immediately for the current low battery warning state
	const bool warning_worse_than_at_arming = (status_flags.battery_warning > _battery_warning_at_arming);
	const int32_t low_battery_action = warning_worse_than_at_arming ?
					   _param_com_low_bat_act.get() : (int32_t)LowBatteryAction::Warning;

	switch (status_flags.battery_warning) {
	case battery_status_s::BATTERY_WARNING_LOW:
		_last_state_battery_warning_low = checkFailsafe(_caller_id_battery_warning_low, _last_state_battery_warning_low,
						  true, fromBatteryWarningActParam(low_battery_action, battery_status_s::BATTERY_WARNING_LOW));
		break;

	case battery_status_s::BATTERY_WARNING_CRITICAL:
		_last_state_battery_warning_critical = checkFailsafe(_caller_id_battery_warning_critical,
						       _last_state_battery_warning_critical,
						       true, fromBatteryWarningActParam(low_battery_action, battery_status_s::BATTERY_WARNING_CRITICAL));
		break;

	case battery_status_s::BATTERY_WARNING_EMERGENCY:
		_last_state_battery_warning_emergency = checkFailsafe(_caller_id_battery_warning_emergency,
							_last_state_battery_warning_emergency,
							true, fromBatteryWarningActParam(low_battery_action, battery_status_s::BATTERY_WARNING_EMERGENCY));
		break;

	default:
		break;
	}


	// Handle fails during spoolup just after arming
	if ((_armed_time != 0)
	    && (time_us < _armed_time + static_cast<hrt_abstime>(_param_com_spoolup_time.get() * 1_s))
	   ) {
		CHECK_FAILSAFE(status_flags, fd_esc_arming_failure, ActionOptions(Action::Disarm).cannotBeDeferred());
		CHECK_FAILSAFE(status_flags, battery_unhealthy, ActionOptions(Action::Disarm).cannotBeDeferred());
	}

	// Handle fails during the early takeoff phase
	if ((_armed_time != 0)
	    && (time_us < _armed_time
		+ static_cast<hrt_abstime>((_param_com_lkdown_tko.get() + _param_com_spoolup_time.get()) * 1_s))
	   ) {
		CHECK_FAILSAFE(status_flags, fd_critical_failure, ActionOptions(Action::Disarm).cannotBeDeferred());

	} else if (!circuit_breaker_enabled_by_val(_param_cbrk_flightterm.get(), CBRK_FLIGHTTERM_KEY)) {
		CHECK_FAILSAFE(status_flags, fd_critical_failure, ActionOptions(Action::Terminate).cannotBeDeferred());

	} else {
		CHECK_FAILSAFE(status_flags, fd_critical_failure, Action::Warn);
	}

	CHECK_FAILSAFE(status_flags, fd_imbalanced_prop, fromImbalancedPropActParam(_param_com_imb_prop_act.get()));
	CHECK_FAILSAFE(status_flags, fd_motor_failure, fromActuatorFailureActParam(_param_com_actuator_failure_act.get()));



	// Mode fallback (last)
	Action mode_fallback_action = checkModeFallback(status_flags, state.user_intended_mode);
	_last_state_mode_fallback = checkFailsafe(_caller_id_mode_fallback, _last_state_mode_fallback,
				    mode_fallback_action != Action::None,
				    ActionOptions(mode_fallback_action).allowUserTakeover(UserTakeoverAllowed::Always).cannotBeDeferred());
}

void Failsafe::updateArmingState(const hrt_abstime &time_us, bool armed, const failsafe_flags_s &status_flags)
{
	if (!_was_armed && armed) {
		_armed_time = time_us;
		_manual_control_lost_at_arming = status_flags.manual_control_signal_lost;
		_battery_warning_at_arming = status_flags.battery_warning;

	} else if (!armed) {
		_manual_control_lost_at_arming = status_flags.manual_control_signal_lost; // ensure action isn't added while disarmed
		_armed_time = 0;
	}

	_was_armed = armed;
}

FailsafeBase::Action Failsafe::checkModeFallback(const failsafe_flags_s &status_flags,
		uint8_t user_intended_mode) const
{
	Action action = Action::None;

	// offboard signal
	if (status_flags.offboard_control_signal_lost && (status_flags.mode_req_offboard_signal & (1u << user_intended_mode))) {
		action = fromOffboardLossActParam(_param_com_obl_rc_act.get(), user_intended_mode);

		// for this specific case, user_intended_mode is not modified, we shouldn't check additional fallbacks
		if (action == Action::Disarm) {
			return action;
		}
	}

	// posctrl
	switch (position_control_navigation_loss_response(_param_com_posctl_navl.get())) {
	case position_control_navigation_loss_response::Altitude_Manual: // AltCtrl/Manual

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

	case position_control_navigation_loss_response::Land_Descend: // Land/Terminate

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

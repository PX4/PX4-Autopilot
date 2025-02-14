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

#pragma once

#include "framework.h"


class Failsafe : public FailsafeBase
{
public:
	Failsafe(ModuleParams *parent) : FailsafeBase(parent) {}

protected:

	void checkStateAndMode(const hrt_abstime &time_us, const State &state,
			       const failsafe_flags_s &status_flags) override;
	Action checkModeFallback(const failsafe_flags_s &status_flags, uint8_t user_intended_mode) const override;

	uint8_t modifyUserIntendedMode(Action previous_action, Action current_action,
				       uint8_t user_intended_mode) const override;

private:
	void updateArmingState(const hrt_abstime &time_us, bool armed, const failsafe_flags_s &status_flags);

	enum class ManualControlLossExceptionBits : int32_t {
		Mission = (1 << 0),
		Hold = (1 << 1),
		Offboard = (1 << 2)
	};

	// COM_LOW_BAT_ACT parameter values
	enum class LowBatteryAction : int32_t {
		Warning = 0,        // Warning
		Return = 1,         // Return mode (deprecated)
		Land = 2,           // Land mode
		ReturnOrLand = 3    // Return mode at critically low level, Land mode at current position if reaching dangerously low levels
	};

	enum class offboard_loss_failsafe_mode : int32_t {
		Position_mode = 0,
		Altitude_mode = 1,
		Stabilized = 2,
		Return_mode = 3,
		Land_mode = 4,
		Hold_mode = 5,
		Terminate = 6,
		Disarm = 7,
	};

	enum class position_control_navigation_loss_response : int32_t {
		Altitude_Manual = 0,
		Land_Descend = 1,
	};

	enum class actuator_failure_failsafe_mode : int32_t {
		Warning_only = 0,
		Hold_mode = 1,
		Land_mode = 2,
		Return_mode = 3,
		Terminate = 4,
	};

	enum class imbalanced_propeller_failsafe_mode : int32_t {
		Disabled = -1,
		Warning = 0,
		Return = 1,
		Land = 2,
	};

	enum class geofence_violation_action : int32_t {
		None = 0,
		Warning = 1,
		Hold_mode = 2,
		Return_mode = 3,
		Terminate = 4,
		Land_mode = 5,
	};

	enum class gcs_connection_loss_failsafe_mode : int32_t {
		Disabled = 0,
		Hold_mode = 1,
		Return_mode = 2,
		Land_mode = 3,
		Terminate = 5,
		Disarm = 6,
	};

	enum class command_after_quadchute : int32_t {
		Warning_only = -1,
		Return_mode = 0,
		Land_mode = 1,
		Hold_mode = 2,
	};

	// COM_RC_IN_MODE parameter values
	enum class RcInMode : int32_t {
		RcTransmitterOnly = 0, 		// RC Transmitter only
		JoystickOnly = 1,		// Joystick only
		RcAndJoystickWithFallback = 2,	// RC And Joystick with fallback
		RcOrJoystickKeepFirst = 3,	// RC or Joystick keep first
		StickInputDisabled = 4		// input disabled
	};

	enum class command_after_high_wind_failsafe : int32_t {
		None = 0,
		Warning = 1,
		Hold_mode = 2,
		Return_mode = 3,
		Terminate = 4,
		Land_mode = 5
	};

	enum class command_after_pos_low_failsafe : int32_t {
		None = 0,
		Warning = 1,
		Hold_mode = 2,
		Return_mode = 3,
		Terminate = 4,
		Land_mode = 5
	};

	enum class command_after_remaining_flight_time_low : int32_t {
		None = 0,
		Warning = 1,
		Return_mode = 3
	};

	static ActionOptions fromNavDllOrRclActParam(int param_value);

	static ActionOptions fromGfActParam(int param_value);
	static ActionOptions fromImbalancedPropActParam(int param_value);
	static ActionOptions fromActuatorFailureActParam(int param_value);
	static ActionOptions fromBatteryWarningActParam(int param_value, uint8_t battery_warning);
	static ActionOptions fromQuadchuteActParam(int param_value);
	static Action fromOffboardLossActParam(int param_value, uint8_t &user_intended_mode);
	static ActionOptions fromHighWindLimitActParam(int param_value);
	static ActionOptions fromPosLowActParam(int param_value);
	static ActionOptions fromRemainingFlightTimeLowActParam(int param_value);

	const int _caller_id_mode_fallback{genCallerId()};
	bool _last_state_mode_fallback{false};
	const int _caller_id_mission_control_lost{genCallerId()};
	bool _last_state_mission_control_lost{false};

	const int _caller_id_battery_warning_low{genCallerId()}; ///< battery warning caller ID's: use different ID's as they have different actions
	bool _last_state_battery_warning_low{false};
	const int _caller_id_battery_warning_critical{genCallerId()};
	bool _last_state_battery_warning_critical{false};
	const int _caller_id_battery_warning_emergency{genCallerId()};
	bool _last_state_battery_warning_emergency{false};

	hrt_abstime _armed_time{0};
	bool _was_armed{false};
	bool _manual_control_lost_at_arming{false}; ///< true if manual control was lost at arming time
	uint8_t _battery_warning_at_arming{0}; ///< low battery state at arming time

	DEFINE_PARAMETERS_CUSTOM_PARENT(FailsafeBase,
					(ParamInt<px4::params::NAV_DLL_ACT>) 	_param_nav_dll_act,
					(ParamInt<px4::params::NAV_RCL_ACT>) 	_param_nav_rcl_act,
					(ParamInt<px4::params::COM_RCL_EXCEPT>) _param_com_rcl_except,
					(ParamInt<px4::params::COM_RC_IN_MODE>) _param_com_rc_in_mode,
					(ParamInt<px4::params::COM_POSCTL_NAVL>) _param_com_posctl_navl,
					(ParamInt<px4::params::GF_ACTION>)  	_param_gf_action,
					(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,
					(ParamInt<px4::params::COM_IMB_PROP_ACT>) _param_com_imb_prop_act,
					(ParamFloat<px4::params::COM_LKDOWN_TKO>) _param_com_lkdown_tko,
					(ParamInt<px4::params::CBRK_FLIGHTTERM>) _param_cbrk_flightterm,
					(ParamInt<px4::params::COM_ACT_FAIL_ACT>) _param_com_actuator_failure_act,
					(ParamInt<px4::params::COM_LOW_BAT_ACT>) _param_com_low_bat_act,
					(ParamInt<px4::params::COM_OBL_RC_ACT>) _param_com_obl_rc_act,
					(ParamInt<px4::params::COM_QC_ACT>) _param_com_qc_act,
					(ParamInt<px4::params::COM_WIND_MAX_ACT>) _param_com_wind_max_act,
					(ParamInt<px4::params::COM_FLTT_LOW_ACT>) _param_com_fltt_low_act,
					(ParamInt<px4::params::COM_POS_LOW_ACT>) _param_com_pos_low_act
				       );

};

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

#include <uORB/topics/vehicle_status_flags.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

#include <cstddef>

#define CHECK_FAILSAFE(status_flags, flag_name, options) \
	checkFailsafe((int)offsetof(vehicle_status_flags_s, flag_name), lastStatusFlags().flag_name, status_flags.flag_name, options)

class FailsafeBase: public ModuleParams
{
public:
	enum class Action : uint8_t {
		// Actions further down take precedence
		None,
		Warn,

		// Fallbacks
		FallbackPosCtrl,
		FallbackAltCtrl,
		FallbackStab,

		Hold,
		RTL,
		Land,
		Descend,
		Disarm,
		Terminate,

		Count
	};

	enum class ClearCondition : uint8_t {
		WhenConditionClears,  ///< Disable action when condition that triggered it cleared
		OnModeChangeOrDisarm, ///< Disable after condition cleared AND (mode switch OR disarm happens)
		OnDisarm,             ///< Disable after condition cleared AND disarm happens
		Never,                ///< Never clear the condition, require reboot to clear
	};

	enum class Cause : uint8_t {
		Generic,
		RCLoss,
		DatalinkLoss,
		BatteryLow,
		BatteryCritical,
		BatteryEmergency,

		Count
	};

	static const char *actionStr(Action action)
	{
		switch (action) {
		case Action::None: return "None";

		case Action::Warn: return "Warn";

		case Action::FallbackPosCtrl: return "Fallback to PosCtrl";

		case Action::FallbackAltCtrl: return "Fallback to AltCtrl";

		case Action::FallbackStab: return "Fallback to Stabilized";

		case Action::Hold: return "Hold";

		case Action::RTL: return "RTL";

		case Action::Land: return "Land";

		case Action::Descend: return "Descend";

		case Action::Disarm: return "Disarm";

		case Action::Terminate: return "Terminate";

		case Action::Count: return "(invalid)";
		}
	}

	enum class RcOverrideBits : int32_t {
		AUTO_MODE_BIT = (1 << 0),
		OFFBOARD_MODE_BIT = (1 << 1),
	};

	struct State {
		bool armed{false};
		uint8_t user_intended_mode{0};
		uint8_t vehicle_type;
		bool vtol_in_transition_mode{false};
		bool mission_finished{false};
	};

	FailsafeBase(ModuleParams *parent);

	/**
	 * update the state machine
	 *
	 * @param time_us current time
	 * @param state vehicle state
	 * @param user_intended_mode_updated true if state.user_intended_mode got set (it may not have changed)
	 * @param rc_sticks_takeover_request true if sticks are moved, and therefore requesting user takeover if in failsafe
	 * @param status_flags condition flags
	 * @return uint8_t updated user intended mode (changes when user wants to take over)
	 */
	uint8_t update(const hrt_abstime &time_us, const State &state, bool user_intended_mode_updated,
		       bool rc_sticks_takeover_request,
		       const vehicle_status_flags_s &status_flags);

	bool inFailsafe() const { return _selected_action != Action::None; }

	Action selectedAction() const { return _selected_action; }

	static uint8_t modeFromAction(const Action &action, uint8_t user_intended_mode);

	bool userTakeoverActive() const { return _user_takeover_active; }

protected:
	enum class UserTakeoverAllowed {
		Always, ///< allow takeover (immediately)
		AlwaysModeSwitchOnly, ///<  allow takeover (immediately), but not via stick movements, only by mode switch
		Auto,   ///< allow takeover depending on delay parameter: if >0 go into hold first (w/o stick takeover possible), then take action
		Never,  ///< never allow takeover
	};

	struct ActionOptions {
		ActionOptions(Action action_ = Action::None) : action(action_) {}
		ActionOptions &allowUserTakeover(UserTakeoverAllowed allow = UserTakeoverAllowed::Auto) { allow_user_takeover = allow; return *this; }
		ActionOptions &clearOn(ClearCondition clear_condition_) { clear_condition = clear_condition_; return *this; }
		ActionOptions &causedBy(Cause cause_) { cause = cause_; return *this; }

		bool valid() const { return id != -1; }
		void setInvalid() { id = -1; }

		Action action{Action::None};
		ClearCondition clear_condition{ClearCondition::WhenConditionClears};
		Cause cause{Cause::Generic};
		UserTakeoverAllowed allow_user_takeover{UserTakeoverAllowed::Auto};

		bool state_failure{false}; ///< used when the clear_condition isn't set to clear immediately
		bool activated{false}; ///< true if checkFailsafe was called during current update
		int id{-1}; ///< unique caller id
	};

	struct SelectedActionState {
		Action action{Action::None};
		Action delayed_action{Action::None};
		Cause cause{Cause::Generic};
		uint8_t updated_user_intended_mode{};
		bool user_takeover{false};
	};

	virtual void checkStateAndMode(const hrt_abstime &time_us, const State &state,
				       const vehicle_status_flags_s &status_flags) = 0;
	virtual Action checkModeFallback(const vehicle_status_flags_s &status_flags, uint8_t user_intended_mode) const = 0;

	const vehicle_status_flags_s &lastStatusFlags() const { return _last_status_flags; }

	bool checkFailsafe(int caller_id, bool last_state_failure, bool cur_state_failure, const ActionOptions &options);

	virtual void getSelectedAction(const State &state, const vehicle_status_flags_s &status_flags,
				       bool user_intended_mode_updated,
				       bool rc_sticks_takeover_request,
				       SelectedActionState &returned_state) const;

	int genCallerId() { return ++_next_caller_id; }

	static bool modeCanRun(const vehicle_status_flags_s &status_flags, uint8_t mode);

	/**
	 * Allows to modify the user intended mode. Use only in limited cases.
	 *
	 * @param previous_action previous action
	 * @param current_action current action
	 * @param user_intended_mode current user intended mode
	 * @return uint8_t new user intended mode
	 */
	virtual uint8_t modifyUserIntendedMode(Action previous_action, Action current_action,
					       uint8_t user_intended_mode) const { return user_intended_mode; }

	void updateParams() override;

private:
	/**
	 * Remove actions matching a condition
	 */
	void removeActions(ClearCondition condition);
	/**
	 * Try to remove an action, depending on the clear_condition
	 */
	void removeAction(ActionOptions &action) const;
	void removeNonActivatedActions();

	void updateDelay(const hrt_abstime &elapsed_us);
	void notifyUser(uint8_t user_intended_mode, Action action, Action delayed_action, Cause cause);

	void clearDelayIfNeeded(const State &state, const vehicle_status_flags_s &status_flags);

	bool actionAllowsUserTakeover(Action action) const;

	void updateDelay(const hrt_abstime &dt, bool delay_active);

	static constexpr int max_num_actions{8};
	ActionOptions _actions[max_num_actions]; ///< currently active actions

	hrt_abstime _last_update{};
	bool _last_armed{false};
	uint8_t _last_user_intended_mode{0};
	vehicle_status_flags_s _last_status_flags{};
	Action _selected_action{Action::None};
	bool _user_takeover_active{false};
	bool _notification_required{false};

	hrt_abstime _current_start_delay{0}; ///< _current_delay is set to this value when starting the delay
	hrt_abstime _current_delay{0}; ///< If > 0, stay in Hold, and take action once delay reaches 0

	int _next_caller_id{sizeof(vehicle_status_flags_s) + 1};
	bool _duplicate_reported_once{false};

	orb_advert_t _mavlink_log_pub{nullptr};

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamFloat<px4::params::COM_FAIL_ACT_T>) 	_param_com_fail_act_t
				       );

};


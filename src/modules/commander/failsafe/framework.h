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

#include <uORB/topics/failsafe_flags.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

#include <cstddef>

using namespace time_literals;

#define CHECK_FAILSAFE(status_flags, flag_name, options) \
	checkFailsafe((int)offsetof(failsafe_flags_s, flag_name), lastStatusFlags().flag_name, status_flags.flag_name, options)

class FailsafeBase: public ModuleParams
{
public:
	static constexpr hrt_abstime DEFAULT_DEFER_TIMEOUT = 30_s;

	/**
	 * Failsafe Priority Policy (axioms)
	 *
	 * A1. Severity is strictly ordered: every action further down in this enum is more
	 *     severe than all actions above it.  When multiple failsafe conditions are
	 *     simultaneously active the highest-severity action is always selected.
	 *
	 * A2. The dominant action always comes from the condition with the highest severity.
	 *     No subsystem may silently suppress or downgrade that action except through the
	 *     explicit UX-improvement skip logic in getSelectedAction() (e.g. "already in
	 *     RTL → only Warn"), which is documented in-place.
	 *
	 * A3. When the desired action cannot execute (e.g. RTL requires a valid home position
	 *     that is currently unavailable), the framework falls through the severity list
	 *     until it finds an action that can run (RTL → Land → Descend → Terminate).
	 *     This fallback is automatic and logged at debug level.
	 *
	 * A4. The decision is stable: a condition that is already being serviced at severity N
	 *     does not restart the Hold/delay timer when a second condition also resolves to
	 *     severity N, preventing oscillation between Hold and the desired action.
	 *
	 * A5. Concurrent battery and geofence failsafes are handled by A1: each subsystem
	 *     independently adds its action to the pool and the worst wins.  Example: a
	 *     battery-EMERGENCY (→Land, severity 7) dominates a simultaneous geofence-Return
	 *     (→RTL, severity 6).
	 */
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
		ManualControlLoss,
		GCSConnectionLoss,
		BatteryLow,
		BatteryCritical,
		BatteryEmergency,
		RemainingFlightTimeLow,

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

		case Action::Count:
		default: return "(invalid)";
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
		       const failsafe_flags_s &status_flags);

	bool inFailsafe() const { return (_selected_action != Action::None && _selected_action != Action::Warn); }

	Action selectedAction() const { return _selected_action; }

	static uint8_t modeFromAction(const Action &action, uint8_t user_intended_mode);

	bool userTakeoverActive() const { return _user_takeover_active; }

	/**
	 * Defer all failsafes that can be deferred. Can be used to avoid triggering failsafes in critical situations.
	 * @param enabled
	 * @param timeout_s timeout in seconds, if set to 0, DEFAULT_DEFER_TIMEOUT is used, -1=no timeout
	 * @return true on success, false if failsafe already active
	 */
	bool deferFailsafes(bool enabled, int timeout_s);
	bool getDeferFailsafes() const { return _defer_failsafes; }
	bool failsafeDeferred() const { return _failsafe_defer_started != 0; }

	using UserCallback = void(*)(void *);

	/**
	 * Register a callback that is called before notifying the user.
	 */
	void setOnNotifyUserCallback(UserCallback callback, void *arg)
	{
		_on_notify_user_cb = callback;
		_on_notify_user_arg = arg;
	}

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
		ActionOptions &cannotBeDeferred() { can_be_deferred = false; return *this; }

		bool valid() const { return id != -1; }
		void setInvalid() { id = -1; }

		Action action{Action::None};
		ClearCondition clear_condition{ClearCondition::WhenConditionClears};
		Cause cause{Cause::Generic};
		UserTakeoverAllowed allow_user_takeover{UserTakeoverAllowed::Auto};
		bool can_be_deferred{true};

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
		bool failsafe_deferred{false};
		/**
		 * Set when the A3 fallback chain selects a more-severe action than what the
		 * pool originally requested (e.g. RTL was desired but home is invalid so Land
		 * was selected instead).  Action::None means no fallback occurred this cycle.
		 */
		Action a3_desired_action{Action::None};
	};

	virtual void checkStateAndMode(const hrt_abstime &time_us, const State &state,
				       const failsafe_flags_s &status_flags) = 0;
	virtual Action checkModeFallback(const failsafe_flags_s &status_flags, uint8_t user_intended_mode) const = 0;

	const failsafe_flags_s &lastStatusFlags() const { return _last_status_flags; }

	bool checkFailsafe(int caller_id, bool last_state_failure, bool cur_state_failure, const ActionOptions &options);

	virtual void getSelectedAction(const State &state, const failsafe_flags_s &status_flags,
				       bool user_intended_mode_updated,
				       bool rc_sticks_takeover_request,
				       SelectedActionState &returned_state) const;

	int genCallerId() { return ++_next_caller_id; }

	static bool modeCanRun(const failsafe_flags_s &status_flags, uint8_t mode);

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

	/**
	 * Called once per cycle when the A3 fallback chain had to select a more-severe
	 * action because the originally desired action could not run.
	 * Covers: RTL → Land (home/global-position unavailable) and
	 *         Land → Descend (relaxed local-position unavailable).
	 *
	 * The default implementation emits a single operator-facing event via events::send()
	 * so that ground-station operators can see why the degraded action was chosen.
	 * Override in test subclasses to capture calls without sending real events.
	 *
	 * @param desired  The action originally requested by the active failsafe condition(s).
	 * @param actual   The less-capable action that was ultimately selected.
	 */
	virtual void notifyA3Fallback(Action desired, Action actual);

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

	void clearDelayIfNeeded(const State &state, const failsafe_flags_s &status_flags);

	bool actionAllowsUserTakeover(Action action) const;

	void updateStartDelay(const hrt_abstime &dt, bool delay_active);

	void updateFailsafeDeferState(const hrt_abstime &time_us, bool defer);

	static constexpr int max_num_actions{8};
	ActionOptions _actions[max_num_actions]; ///< currently active actions

	hrt_abstime _last_update{};
	bool _last_armed{false};
	uint8_t _last_user_intended_mode{0};
	failsafe_flags_s _last_status_flags{};
	Action _selected_action{Action::None};
	bool _user_takeover_active{false};
	bool _notification_required{false};

	bool _defer_failsafes{false};
	hrt_abstime _defer_timeout{0};
	hrt_abstime _failsafe_defer_started{0};

	hrt_abstime _current_start_delay{0}; ///< _current_delay is set to this value when starting the delay
	hrt_abstime _current_delay{0}; ///< If > 0, stay in Hold, and take action once delay reaches 0

	int _next_caller_id{sizeof(failsafe_flags_s) + 1};
	bool _duplicate_reported_once{false};

	orb_advert_t _mavlink_log_pub{nullptr};

	UserCallback _on_notify_user_cb{nullptr};
	void *_on_notify_user_arg{nullptr};

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamFloat<px4::params::COM_FAIL_ACT_T>) 	_param_com_fail_act_t
				       );

};

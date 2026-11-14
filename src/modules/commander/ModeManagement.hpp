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

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/setpoint_config.h>
#include <uORB/topics/setpoint_config_reply.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/config_overrides.h>
#include <uORB/topics/failsafe_flags.h>

#include <lib/modes/ui.hpp>
#include "UserModeIntention.hpp"
#include "HealthAndArmingChecks/checks/externalChecks.hpp"
#include "ModeUtil/setpoint_types.hpp"

class ModeExecutors
{
public:
	static constexpr int AUTOPILOT_EXECUTOR_ID = 0;
	static constexpr int FIRST_EXECUTOR_ID = 1;
	static constexpr int MAX_NUM = 5;

	struct ModeExecutor {
		config_overrides_s overrides{};
		uint8_t owned_nav_state{};
		bool valid{false};
	};

	void printStatus(int executor_in_charge) const;

	bool valid(int id) const { return id >= FIRST_EXECUTOR_ID && id < FIRST_EXECUTOR_ID + MAX_NUM && _mode_executors[id - FIRST_EXECUTOR_ID].valid; }
	const ModeExecutor &executor(int id) const { return _mode_executors[id - FIRST_EXECUTOR_ID]; }
	ModeExecutor &executor(int id) { return _mode_executors[id - FIRST_EXECUTOR_ID]; }

	bool hasFreeExecutors() const;
	int addExecutor(const ModeExecutor &executor);
	void removeExecutor(int id);
private:
	ModeExecutor _mode_executors[MAX_NUM] {};
};

class Modes
{
public:
	static constexpr uint8_t FIRST_EXTERNAL_NAV_STATE = vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;
	static constexpr uint8_t LAST_EXTERNAL_NAV_STATE = vehicle_status_s::NAVIGATION_STATE_EXTERNAL8;
	static constexpr int MAX_NUM = LAST_EXTERNAL_NAV_STATE - FIRST_EXTERNAL_NAV_STATE + 1;

	struct Mode {
		static constexpr auto kDefaultSetpointType = mode_util::SetpointType::Trajectory;

		Mode() = default;

		static constexpr uint8_t REPLACES_NAV_STATE_NONE = 0xff;

		char name[sizeof(register_ext_component_request_s::name)] {};
		bool valid{false};
		uint8_t replaces_nav_state{REPLACES_NAV_STATE_NONE};
		bool unresponsive_reported{false};
		bool setpoint_timeout_reported{false};
		int arming_check_registration_id{-1};
		int mode_executor_registration_id{-1};
		bool request_offboard_setpoints{false};
		config_overrides_s overrides{};
		mode_util::SetpointType current_setpoint_type{kDefaultSetpointType};
		uint16_t setpoint_timeout_ms{0}; ///< from the applied SetpointConfig, 0 = disabled
	};

	Modes();

	void printStatus() const;

	bool valid(uint8_t nav_state) const { return nav_state >= FIRST_EXTERNAL_NAV_STATE && nav_state <= LAST_EXTERNAL_NAV_STATE && _modes[nav_state - FIRST_EXTERNAL_NAV_STATE].valid; }
	Mode &mode(uint8_t nav_state) { return _modes[nav_state - FIRST_EXTERNAL_NAV_STATE]; }
	const Mode &mode(uint8_t nav_state) const { return _modes[nav_state - FIRST_EXTERNAL_NAV_STATE]; }

	bool hasFreeExternalModes() const;
	uint8_t addExternalMode(const Mode &mode);
	bool removeExternalMode(uint8_t nav_state, const char *name);

private:
	Mode _modes[MAX_NUM] {};
	param_t _mode_hash_handles[MAX_NUM];
};


#ifndef CONSTRAINED_FLASH

class ModeManagement : public ModeChangeHandler
{
public:
	ModeManagement(ExternalChecks &external_checks);
	virtual ~ModeManagement() = default;

	struct UpdateRequest {
		bool change_user_intended_nav_state{false};
		uint8_t user_intended_nav_state{};
		bool control_setpoint_update{false};
		bool setpoint_timeout_changed{false};
	};

	void update(uint8_t vehicle_type, bool is_vtol, bool armed, bool landed, uint8_t user_intended_nav_state,
		    uint8_t nav_state, hrt_abstime now, UpdateRequest &update_request);
	void setFailsafeState(bool failsafe_action_active)
	{
		_failsafe_action_active = failsafe_action_active;
	}

	/**
	 * Mode executor ID for who is currently in charge (and can send commands etc).
	 * This is ModeExecutors::AUTOPILOT_EXECUTOR_ID if no executor is in charge currently.
	 */
	int modeExecutorInCharge() const;

	/**
	 * Returns the executor's navigation state if active, otherwise nav_state.
	 * Exposes the high-level goal rather than the effective sub-mode.
	 */
	uint8_t getNavStateDisplay(uint8_t nav_state) const;

	void onUserIntendedNavStateChange(ModeChangeSource source, uint8_t user_intended_nav_state) override;
	uint8_t getReplacedModeIfAny(uint8_t nav_state) override;

	uint8_t onDisarm(uint8_t stored_nav_state) override;

	uint8_t getNavStateReplacementIfValid(uint8_t nav_state, bool report_error = true);

	mode_util::SetpointType getSetpointType(uint8_t nav_state);

	void printStatus() const;

	void getModeStatus(uint32_t &valid_nav_state_mask, uint32_t &can_set_nav_state_mask, const failsafe_flags_s &failsafe_flags) const;

	bool currentModeAcceptsOffboardSetpoints(uint8_t nav_state) const;

	void updateActiveConfigOverrides(uint8_t nav_state, config_overrides_s &overrides_in_out);

private:
	/**
	 * Subscription that only tracks whether new samples got published, without copying them
	 */
	class SetpointSubscription : public uORB::Subscription
	{
	public:
		/**
		 * Change the topic. Only samples published after this call count as updates.
		 */
		void setTopic(ORB_ID id)
		{
			unsubscribe();
			_orb_id = id;
			consumeUpdates();
		}

		/**
		 * @return true if a sample got published since the last call
		 */
		bool consumeUpdates()
		{
			if (!subscribe()) {
				return false;
			}

			const unsigned num_updates = uORB::Manager::updates_available(_node, _last_generation);
			_last_generation += num_updates;
			return num_updates > 0;
		}
	};

	static constexpr uint8_t NAV_STATE_NONE = 0xff;

	bool checkConfigControlSetpointUpdates(uint8_t vehicle_type, bool is_vtol);
	void checkSetpointTimeout(bool armed, bool landed, uint8_t user_intended_nav_state, uint8_t nav_state,
				  hrt_abstime now, UpdateRequest &update_request);
	void checkNewRegistrations(UpdateRequest &update_request);
	void checkUnregistrations(uint8_t user_intended_nav_state, UpdateRequest &update_request);
	void checkConfigOverrides();

	void removeModeExecutor(int mode_executor_id);
	bool resendIfCachedRequest(const register_ext_component_request_s &request);

	uORB::Subscription _setpoint_config_sub{ORB_ID(setpoint_config)};
	uORB::Publication<setpoint_config_reply_s> _setpoint_config_reply_pub{ORB_ID(setpoint_config_reply)};
	uORB::Subscription _register_ext_component_request_sub{ORB_ID(register_ext_component_request)};
	uORB::Subscription _unregister_ext_component_sub{ORB_ID(unregister_ext_component)};
	uORB::Publication<register_ext_component_reply_s> _register_ext_component_reply_pub{ORB_ID(register_ext_component_reply)};
	uORB::Publication<config_overrides_s> _config_overrides_pub{ORB_ID(config_overrides)};
	uORB::Subscription _config_overrides_request_sub{ORB_ID(config_overrides_request)};
	uORB::Publication<config_overrides_s> _config_overrides_confirm_pub{ORB_ID(config_overrides_confirm)};

	ExternalChecks &_external_checks;
	ModeExecutors _mode_executors;
	Modes _modes;

	bool _failsafe_action_active{false};
	int _mode_executor_in_charge{ModeExecutors::AUTOPILOT_EXECUTOR_ID};

	bool _invalid_mode_printed{false};

	uint8_t _last_served_nav_state{0xff};

	SetpointSubscription _setpoint_subs[2];
	uint8_t _setpoint_watch_nav_state{NAV_STATE_NONE}; ///< selected mode whose setpoints are watched, also while disarmed
	hrt_abstime _last_setpoint_time{0}; ///< time of the last setpoint (now while landed), 0 while waiting for the first one
	uint32_t _setpoint_configs_applied{0}; ///< bit per nav_state with a setpoint configuration applied since the last check
	uint8_t _setpoint_timeout_user_intention{NAV_STATE_NONE}; ///< user intention when modes got flagged for a setpoint timeout
};

#else /* CONSTRAINED_FLASH */

class ModeManagement : public ModeChangeHandler
{
public:
	ModeManagement() = default;
	virtual ~ModeManagement() = default;

	struct UpdateRequest {
		bool change_user_intended_nav_state{false};
		uint8_t user_intended_nav_state{};
		bool control_setpoint_update{false};
		bool setpoint_timeout_changed{false};
	};

	void update(uint8_t vehicle_type, bool is_vtol, bool armed, bool landed, uint8_t user_intended_nav_state,
		    uint8_t nav_state, hrt_abstime now, UpdateRequest &update_request) {}
	void setFailsafeState(bool failsafe_action_active) {}

	int modeExecutorInCharge() const { return ModeExecutors::AUTOPILOT_EXECUTOR_ID; }
	uint8_t getNavStateDisplay(uint8_t nav_state) const { return nav_state; }

	void onUserIntendedNavStateChange(ModeChangeSource source, uint8_t user_intended_nav_state) override {}
	uint8_t getReplacedModeIfAny(uint8_t nav_state) override { return nav_state; }
	uint8_t onDisarm(uint8_t stored_nav_state) override { return stored_nav_state; }

	uint8_t getNavStateReplacementIfValid(uint8_t nav_state, bool report_error = true) { return nav_state; }

	mode_util::SetpointType getSetpointType(uint8_t nav_state) { return mode_util::SetpointType::Trajectory; }

	void printStatus() const {}

	void getModeStatus(uint32_t &valid_nav_state_mask, uint32_t &can_set_nav_state_mask, const failsafe_flags_s &failsafe_flags) const
	{
		valid_nav_state_mask = mode_util::getValidNavStates(failsafe_flags.mode_not_supported);
		can_set_nav_state_mask = valid_nav_state_mask & ~(1u << vehicle_status_s::NAVIGATION_STATE_TERMINATION);
	}

	bool currentModeAcceptsOffboardSetpoints(uint8_t nav_state) const
	{
		return nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	}

	void updateActiveConfigOverrides(uint8_t nav_state, config_overrides_s &overrides_in_out) { }

private:
};

#endif /* CONSTRAINED_FLASH */

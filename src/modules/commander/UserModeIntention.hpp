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

#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/formic_state_machine.h>

#include "HealthAndArmingChecks/HealthAndArmingChecks.hpp"

enum class ModeChangeSource {
	User,           ///< RC or MAVLink
	ModeExecutor,
};

class ModeChangeHandler
{
public:
	virtual void onUserIntendedNavStateChange(ModeChangeSource source, uint8_t user_intended_nav_state) = 0;

	/**
	 * Get the replaced (internal) mode for a given (external) mode
	 * @param nav_state
	 * @return nav_state or the mode that nav_state replaces
	 */
	virtual uint8_t getReplacedModeIfAny(uint8_t nav_state) = 0;

	virtual uint8_t onDisarm(uint8_t stored_nav_state) = 0;
};


class UserModeIntention
{
public:
	UserModeIntention(const vehicle_status_s &vehicle_status,
			  const HealthAndArmingChecks &health_and_arming_checks, ModeChangeHandler *handler);
	~UserModeIntention() = default;

	/**
	 * Change the user intended mode
	 * @param user_intended_nav_state new mode
	 * @param source calling reason
	 * @param allow_fallback allow to fallback to a lower mode if current mode cannot run
	 * @param force always set if true
	 * @return true if successfully set (also if unchanged)
	 */
	bool change(uint8_t user_intended_nav_state, ModeChangeSource source = ModeChangeSource::User,
		    bool allow_fallback = false, bool force = false);

	uint8_t get() const { return _user_intented_nav_state; }

	/**
	 * Change the user intention to the last user intended mode where arming is possible
	 */
	void onDisarm();

	/**
	 * Returns false if there has not been any mode change yet
	 */
	bool everHadModeChange() const { return _ever_had_mode_change; }

	bool getHadModeChangeAndClear() { bool ret = _had_mode_change; _had_mode_change = false; return ret; }

	///////add by naor ////////////////
	// Called every Commander run-loop iteration.
	// If a position-required mode is pending, counts consecutive iterations
	// where position is valid and switches into the mode after POS_STABLE_THRESHOLD.
	void tick();

	// Called from Commander after failsafe resolves the actual nav_state.
	// If the drone was forced to a non-position mode by failsafe (user intention may still
	// be position), clear the pending request and publish let_update_ev = false.
	void onFailsafeNavState(uint8_t actual_nav_state);
	///////add by naor ////////////////

private:
	bool isArmed() const { return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED; }

	///////add by naor ////////////////
	// Returns true if nav_state is one that requires local or global position.
	bool modeRequiresPosition(uint8_t nav_state) const;
	void publish_formic_state_machine(bool let_update_ev);
	///////add by naor ////////////////

	const vehicle_status_s &_vehicle_status;
	const HealthAndArmingChecks &_health_and_arming_checks;
	ModeChangeHandler *const _handler{nullptr};

	uint8_t _user_intented_nav_state{vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER}; ///< Current user intended mode
	uint8_t _nav_state_after_disarming{vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER}; ///< Mode that is switched into after landing/disarming

	bool _ever_had_mode_change{false}; ///< true if there was ever a mode change call (also if the same mode as already set)
	bool _had_mode_change{false}; ///< true if there was a mode change call since the last getHadModeChangeAndClear()

	///////add by naor ////////////////
	param_t _param_pos_wait_limit{PARAM_INVALID}; ///< handle for COM_POS_WAIT_LIM parameter
	uint8_t _pending_nav_state{UINT8_MAX};        ///< mode waiting for position lock (UINT8_MAX = none)
	int     _pos_wait_count{0};                   ///< total iterations waited since the pending request was parked
	uORB::Subscription _ev_yaw_sub{ORB_ID(estimator_aid_src_ev_yaw)}; ///< EV yaw aid source
	estimator_aid_source1d_s _ev_yaw{};           ///< last received EV yaw aid data (timestamp==0 means never received)
	uORB::Publication<formic_state_machine_s> _formic_state_machine_pub{ORB_ID(formic_state_machine)}; ///< Formic state machine publisher
	///////add by naor ////////////////
};

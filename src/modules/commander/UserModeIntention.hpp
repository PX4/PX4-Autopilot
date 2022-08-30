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

#include <uORB/topics/vehicle_status.h>
#include "HealthAndArmingChecks/HealthAndArmingChecks.hpp"
#include <px4_platform_common/module_params.h>

class UserModeIntention : ModuleParams
{
public:
	UserModeIntention(ModuleParams *parent, const vehicle_status_s &vehicle_status,
			  const HealthAndArmingChecks &health_and_arming_checks);
	~UserModeIntention() = default;

	/**
	 * Change the user intended mode
	 * @param user_intended_nav_state new mode
	 * @param allow_fallback allow to fallback to a lower mode if current mode cannot run
	 * @param force always set if true
	 * @return true if successfully set (also if unchanged)
	 */
	bool change(uint8_t user_intended_nav_state, bool allow_fallback = false, bool force = false);

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

private:
	bool isArmed() const { return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED; }

	const vehicle_status_s &_vehicle_status;
	const HealthAndArmingChecks &_health_and_arming_checks;

	uint8_t _user_intented_nav_state{vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER}; ///< Current user intended mode
	uint8_t _nav_state_after_disarming{vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER}; ///< Mode that is switched into after landing/disarming

	bool _ever_had_mode_change{false}; ///< true if there was ever a mode change call (also if the same mode as already set)
	bool _had_mode_change{false}; ///< true if there was a mode change call since the last getHadModeChangeAndClear()

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::COM_POSCTL_NAVL>) _param_com_posctl_navl
	);
};

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

#include "../PreFlightCheck/PreFlightCheck.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

#include "../../state_machine_helper.h" // TODO: get independent of transition_result_t

using arm_disarm_reason_t = events::px4::enums::arm_disarm_reason_t;

class ArmStateMachine
{
public:
	ArmStateMachine() = default;
	~ArmStateMachine() = default;

	void forceArmState(uint8_t new_arm_state) { _arm_state = new_arm_state; }

	transition_result_t
	arming_state_transition(vehicle_status_s &status, const vehicle_control_mode_s &control_mode,
				const bool safety_button_available, const bool safety_off, const arming_state_t new_arming_state,
				actuator_armed_s &armed, const bool fRunPreArmChecks, orb_advert_t *mavlink_log_pub,
				vehicle_status_flags_s &status_flags, const PreFlightCheck::arm_requirements_t &arm_requirements,
				const hrt_abstime &time_since_boot, arm_disarm_reason_t calling_reason);

	// Getters
	uint8_t getArmState() const { return _arm_state; }

	bool isInit() const { return (_arm_state == vehicle_status_s::ARMING_STATE_INIT); }
	bool isStandby() const { return (_arm_state == vehicle_status_s::ARMING_STATE_STANDBY); }
	bool isArmed() const { return (_arm_state == vehicle_status_s::ARMING_STATE_ARMED); }
	bool isShutdown() const { return (_arm_state == vehicle_status_s::ARMING_STATE_SHUTDOWN); }

	static const char *getArmStateName(uint8_t arming_state);
	const char *getArmStateName() const { return getArmStateName(_arm_state); }

private:
	static inline events::px4::enums::arming_state_t getArmStateEvent(uint8_t arming_state);

	uint8_t _arm_state{vehicle_status_s::ARMING_STATE_INIT};
	hrt_abstime _last_preflight_check = 0; ///< initialize so it gets checked immediately

	// This array defines the arming state transitions. The rows are the new state, and the columns
	// are the current state. Using new state and current state you can index into the array which
	// will be true for a valid transition or false for a invalid transition. In some cases even
	// though the transition is marked as true additional checks must be made. See arming_state_transition
	// code for those checks.
	static constexpr bool arming_transitions[vehicle_status_s::ARMING_STATE_MAX][vehicle_status_s::ARMING_STATE_MAX]
	= {
		//                                                    INIT,  STANDBY, ARMED, STANDBY_ERROR, SHUTDOWN, IN_AIR_RESTORE
		{ /* vehicle_status_s::ARMING_STATE_INIT */           true,  true,    false, true,          false,    false },
		{ /* vehicle_status_s::ARMING_STATE_STANDBY */        true,  true,    true,  false,         false,    false },
		{ /* vehicle_status_s::ARMING_STATE_ARMED */          false, true,    true,  false,         false,    true },
		{ /* vehicle_status_s::ARMING_STATE_STANDBY_ERROR */  true,  true,    true,  true,          false,    false },
		{ /* vehicle_status_s::ARMING_STATE_SHUTDOWN */       true,  true,    false, true,          true,     true },
		{ /* vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE */ false, false,   false, false,         false,    false }, // NYI
	};
};

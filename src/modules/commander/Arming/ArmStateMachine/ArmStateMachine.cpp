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

#include "ArmStateMachine.hpp"

#include <systemlib/mavlink_log.h>

constexpr bool
ArmStateMachine::arming_transitions[vehicle_status_s::ARMING_STATE_MAX][vehicle_status_s::ARMING_STATE_MAX];

transition_result_t ArmStateMachine::arming_state_transition(vehicle_status_s &status,
		const vehicle_control_mode_s &control_mode, const bool safety_button_available, const bool safety_off,
		const arming_state_t new_arming_state, actuator_armed_s &armed, const bool fRunPreArmChecks,
		orb_advert_t *mavlink_log_pub, vehicle_status_flags_s &status_flags,
		const PreFlightCheck::arm_requirements_t &arm_requirements,
		const hrt_abstime &time_since_boot, arm_disarm_reason_t calling_reason)
{
	// Double check that our static arrays are still valid
	static_assert(vehicle_status_s::ARMING_STATE_INIT == 0, "ARMING_STATE_INIT == 0");
	static_assert(vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE == vehicle_status_s::ARMING_STATE_MAX - 1,
		      "ARMING_STATE_IN_AIR_RESTORE == ARMING_STATE_MAX - 1");

	transition_result_t ret = TRANSITION_DENIED;
	bool feedback_provided = false;

	const bool hil_enabled = (status.hil_state == vehicle_status_s::HIL_STATE_ON);

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == _arm_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		/*
		 * Get sensing state if necessary
		 */
		bool preflight_check_ret = true;

		/* only perform the pre-arm check if we have to */
		if (fRunPreArmChecks && (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		    && !hil_enabled) {

			preflight_check_ret = PreFlightCheck::preflightCheck(mavlink_log_pub, status, status_flags, control_mode,
					      true, true, time_since_boot);

			if (preflight_check_ret) {
				status_flags.system_sensors_initialized = true;
			}

			feedback_provided = true;
		}

		/* re-run the pre-flight check as long as sensors are failing */
		if (!status_flags.system_sensors_initialized
		    && fRunPreArmChecks
		    && ((new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			|| (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY))
		    && !hil_enabled) {

			if ((_last_preflight_check == 0) || (hrt_elapsed_time(&_last_preflight_check) > 1000 * 1000)) {

				status_flags.system_sensors_initialized = PreFlightCheck::preflightCheck(mavlink_log_pub, status,
						status_flags, control_mode, false, !isArmed(),
						time_since_boot);

				_last_preflight_check = hrt_absolute_time();
			}
		}

		// Check that we have a valid state transition
		bool valid_transition = arming_transitions[new_arming_state][_arm_state];

		if (valid_transition) {
			// We have a good transition. Now perform any secondary validation.
			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				//      Do not perform pre-arm checks if coming from in air restore
				//      Allow if vehicle_status_s::HIL_STATE_ON
				if (_arm_state != vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE) {

					bool prearm_check_ret = true;

					if (fRunPreArmChecks && preflight_check_ret) {
						// only bother running prearm if preflight was successful
						prearm_check_ret = PreFlightCheck::preArmCheck(mavlink_log_pub, status_flags, control_mode,
								   safety_button_available, safety_off,
								   arm_requirements, status);
					}

					if (!preflight_check_ret || !prearm_check_ret) {
						// the prearm and preflight checks already print the rejection reason
						feedback_provided = true;
						valid_transition = false;
					}
				}
			}
		}

		if (hil_enabled) {
			/* enforce lockdown in HIL */
			armed.lockdown = true;
			status_flags.system_sensors_initialized = true;

			/* recover from a prearm fail */
			if (_arm_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {
				_arm_state = vehicle_status_s::ARMING_STATE_STANDBY;
			}

			// HIL can always go to standby
			if (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
				valid_transition = true;
			}
		}

		if (!hil_enabled &&
		    (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) &&
		    (_arm_state != vehicle_status_s::ARMING_STATE_STANDBY_ERROR)) {

			// Sensors need to be initialized for STANDBY state, except for HIL
			if (!status_flags.system_sensors_initialized) {
				feedback_provided = true;
				valid_transition = false;
			}
		}

		// Finish up the state transition
		if (valid_transition) {
			ret = TRANSITION_CHANGED;

			// Record arm/disarm reason
			if (isArmed() && (new_arming_state != vehicle_status_s::ARMING_STATE_ARMED)) { // disarm transition
				status.latest_disarming_reason = (uint8_t)calling_reason;

			} else if (!isArmed() && (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)) { // arm transition
				status.latest_arming_reason = (uint8_t)calling_reason;
			}

			// Switch state
			_arm_state = new_arming_state;

			if (isArmed()) {
				status.armed_time = hrt_absolute_time();

			} else {
				status.armed_time = 0;
			}
		}
	}

	if (ret == TRANSITION_DENIED) {
		/* print to MAVLink and console if we didn't provide any feedback yet */
		if (!feedback_provided) {
			// FIXME: this catch-all does not provide helpful information to the user
			mavlink_log_critical(mavlink_log_pub, "Transition denied: %s to %s\t",
					     getArmStateName(_arm_state), getArmStateName(new_arming_state));
			events::send<events::px4::enums::arming_state_t, events::px4::enums::arming_state_t>(
				events::ID("commander_transition_denied"), events::Log::Critical,
				"Arming state transition denied: {1} to {2}",
				getArmStateEvent(_arm_state), getArmStateEvent(new_arming_state));
		}
	}

	return ret;
}

const char *ArmStateMachine::getArmStateName(uint8_t arming_state)
{
	switch (arming_state) {

	case vehicle_status_s::ARMING_STATE_INIT: return "Init";

	case vehicle_status_s::ARMING_STATE_STANDBY: return "Standby";

	case vehicle_status_s::ARMING_STATE_ARMED: return "Armed";

	case vehicle_status_s::ARMING_STATE_STANDBY_ERROR: return "Standby error";

	case vehicle_status_s::ARMING_STATE_SHUTDOWN: return "Shutdown";

	case vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE: return "In-air restore";

	default: return "Unknown";
	}

	static_assert(vehicle_status_s::ARMING_STATE_MAX - 1 == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE,
		      "enum def mismatch");
}

events::px4::enums::arming_state_t ArmStateMachine::getArmStateEvent(uint8_t arming_state)
{
	switch (arming_state) {
	case vehicle_status_s::ARMING_STATE_INIT: return events::px4::enums::arming_state_t::init;

	case vehicle_status_s::ARMING_STATE_STANDBY: return events::px4::enums::arming_state_t::standby;

	case vehicle_status_s::ARMING_STATE_ARMED: return events::px4::enums::arming_state_t::armed;

	case vehicle_status_s::ARMING_STATE_STANDBY_ERROR: return events::px4::enums::arming_state_t::standby_error;

	case vehicle_status_s::ARMING_STATE_SHUTDOWN: return events::px4::enums::arming_state_t::shutdown;

	case vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE: return events::px4::enums::arming_state_t::inair_restore;
	}

	static_assert(vehicle_status_s::ARMING_STATE_MAX - 1 == (int)events::px4::enums::arming_state_t::inair_restore,
		      "enum def mismatch");

	return events::px4::enums::arming_state_t::init;
}

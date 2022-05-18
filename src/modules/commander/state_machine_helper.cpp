/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file state_machine_helper.cpp
 * State machine helper functions implementations
 *
 * @author Thomas Gubler	<thomas@px4.io>
 * @author Julian Oes		<julian@oes.ch>
 * @author Sander Smeets	<sander@droneslab.com>
 */
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <float.h>

#include "state_machine_helper.h"
#include "commander_helper.h"

using namespace time_literals;

static constexpr const char reason_no_rc[] = "No manual control stick input";
static constexpr const char reason_no_offboard[] = "no offboard";
static constexpr const char reason_no_rc_and_no_offboard[] = "no RC and no offboard";
static constexpr const char reason_no_local_position[] = "no local position";
static constexpr const char reason_no_global_position[] = "no global position";
static constexpr const char reason_no_datalink[] = "no datalink";
static constexpr const char reason_no_rc_and_no_datalink[] = "no RC and no datalink";
static constexpr const char reason_no_gps[] = "no GPS";

// You can index into the array with an navigation_state_t in order to get its textual representation
const char *const nav_state_names[vehicle_status_s::NAVIGATION_STATE_MAX] = {
	"MANUAL",
	"ALTCTL",
	"POSCTL",
	"AUTO_MISSION",
	"AUTO_LOITER",
	"AUTO_RTL",
	"6: unallocated",
	"7: unallocated",
	"AUTO_LANDENGFAIL",
	"9: unallocated",
	"ACRO",
	"11: UNUSED",
	"DESCEND",
	"TERMINATION",
	"OFFBOARD",
	"STAB",
	"16: UNUSED2",
	"AUTO_TAKEOFF",
	"AUTO_LAND",
	"AUTO_FOLLOW_TARGET",
	"AUTO_PRECLAND",
	"ORBIT"
};

void set_link_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
			     const vehicle_status_flags_s &status_flags, commander_state_s &internal_state, link_loss_actions_t link_loss_act,
			     const float ll_delay);

void reset_link_loss_globals(actuator_armed_s &armed, const bool old_failsafe, const link_loss_actions_t link_loss_act);

void set_offboard_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				 const vehicle_status_flags_s &status_flags,
				 const offboard_loss_actions_t offboard_loss_act);

void set_quadchute_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
			     const vehicle_status_flags_s &status_flags,
			     const quadchute_actions_t quadchute_act);

void set_offboard_loss_rc_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				    const vehicle_status_flags_s &status_flags,
				    const offboard_loss_rc_actions_t offboard_loss_rc_act);

void reset_offboard_loss_globals(actuator_armed_s &armed, const bool old_failsafe,
				 const offboard_loss_actions_t offboard_loss_act,
				 const offboard_loss_rc_actions_t offboard_loss_rc_act);

transition_result_t
main_state_transition(const vehicle_status_s &status, const main_state_t new_main_state,
		      const vehicle_status_flags_s &status_flags, commander_state_s &internal_state)
{
	// IMPORTANT: The assumption of callers of this function is that the execution of
	// this check is essentially "free". Therefore any runtime checking in here has to be
	// kept super lightweight. No complex logic or calls on external function should be
	// implemented here.

	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if the same state is requested because conditions may have changed */
	switch (new_main_state) {
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ACRO:
		ret = TRANSITION_CHANGED;
		break;

	case commander_state_s::MAIN_STATE_ALTCTL:

		/* need at minimum altitude estimate */
		if (status_flags.local_altitude_valid ||
		    status_flags.global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL:

		/* need at minimum local position estimate */
		if (status_flags.local_position_valid ||
		    status_flags.global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* need global position estimate */
		if (status_flags.global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:
	case commander_state_s::MAIN_STATE_ORBIT:

		/* Follow and orbit only implemented for multicopter */
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_VTOL_TAKEOFF:
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* need global position, home position, and a valid mission */
		if (status_flags.global_position_valid &&
		    status_flags.auto_mission_available) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* need global position and home position */
		if (status_flags.global_position_valid && status_flags.home_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:
	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* need local position */
		if (status_flags.local_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND:

		/* need local and global position, and precision land only implemented for multicopters */
		if (status_flags.local_position_valid
		    && status_flags.global_position_valid
		    && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* need offboard signal */
		if (!status_flags.offboard_control_signal_lost) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_MAX:
	default:
		break;
	}

	if (ret == TRANSITION_CHANGED) {
		if (internal_state.main_state != new_main_state) {
			internal_state.main_state = new_main_state;
			internal_state.main_state_changes++;
			internal_state.timestamp = hrt_absolute_time();

		} else {
			ret = TRANSITION_NOT_CHANGED;
		}
	}

	return ret;
}

using event_failsafe_reason_t = events::px4::enums::failsafe_reason_t;
/**
 * Enable failsafe and report to user
 */
static void enable_failsafe(vehicle_status_s &status, bool old_failsafe, orb_advert_t *mavlink_log_pub,
			    event_failsafe_reason_t event_failsafe_reason)
{
	if (!old_failsafe && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		// make sure intermittent failsafes don't lead to infinite delay by not constantly reseting the timestamp
		if (status.failsafe_timestamp == 0 ||
		    hrt_elapsed_time(&status.failsafe_timestamp) > 30_s) {
			status.failsafe_timestamp = hrt_absolute_time();
		}

		const char *reason = "";

		switch (event_failsafe_reason) {
		case event_failsafe_reason_t::no_rc: reason = reason_no_rc; break;

		case event_failsafe_reason_t::no_offboard: reason = reason_no_offboard; break;

		case event_failsafe_reason_t::no_rc_and_no_offboard: reason = reason_no_rc_and_no_offboard; break;

		case event_failsafe_reason_t::no_local_position: reason = reason_no_local_position; break;

		case event_failsafe_reason_t::no_global_position: reason = reason_no_global_position; break;

		case event_failsafe_reason_t::no_datalink: reason = reason_no_datalink; break;

		case event_failsafe_reason_t::no_rc_and_no_datalink: reason = reason_no_rc_and_no_datalink; break;

		case event_failsafe_reason_t::no_gps: reason = reason_no_gps; break;
		}

		mavlink_log_critical(mavlink_log_pub, "Failsafe enabled: %s\t", reason);
		events::send<events::px4::enums::failsafe_reason_t>(
			events::ID("commander_enable_failsafe"), {events::Log::Critical, events::LogInternal::Info},
			"Failsafe enabled: {1}", event_failsafe_reason);
	}

	status.failsafe = true;
}

/**
 * Check failsafe and main status and set navigation status for navigator accordingly
 */
bool set_nav_state(vehicle_status_s &status, actuator_armed_s &armed, commander_state_s &internal_state,
		   orb_advert_t *mavlink_log_pub, const link_loss_actions_t data_link_loss_act, const bool mission_finished,
		   const bool stay_in_failsafe, const vehicle_status_flags_s &status_flags, bool landed,
		   const link_loss_actions_t rc_loss_act, const offboard_loss_actions_t offb_loss_act,
		   const quadchute_actions_t quadchute_act,
		   const offboard_loss_rc_actions_t offb_loss_rc_act,
		   const position_nav_loss_actions_t posctl_nav_loss_act,
		   const float param_com_rcl_act_t, const int param_com_rcl_except)
{
	const navigation_state_t nav_state_old = status.nav_state;

	const bool is_armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool data_link_loss_act_configured = (data_link_loss_act > link_loss_actions_t::DISABLED);

	bool old_failsafe = status.failsafe;
	status.failsafe = false;

	// Safe to do reset flags here, as if loss state persists flags will be restored in the code below
	reset_link_loss_globals(armed, old_failsafe, rc_loss_act);
	reset_link_loss_globals(armed, old_failsafe, data_link_loss_act);
	reset_offboard_loss_globals(armed, old_failsafe, offb_loss_act, offb_loss_rc_act);

	// Failsafe decision logic for every normal non-failsafe mode
	switch (internal_state.main_state) {
	case commander_state_s::MAIN_STATE_ACRO:
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ALTCTL:

		// Require RC for all manual modes
		if (status.rc_signal_lost && is_armed) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else {
			switch (internal_state.main_state) {
			case commander_state_s::MAIN_STATE_ACRO:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_ACRO;
				break;

			case commander_state_s::MAIN_STATE_MANUAL:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;

			case commander_state_s::MAIN_STATE_STAB:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
				break;

			case commander_state_s::MAIN_STATE_ALTCTL:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
				break;

			default:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;
			}
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL: {

			const bool rc_fallback_allowed = (posctl_nav_loss_act != position_nav_loss_actions_t::LAND_TERMINATE) || !is_armed;

			if (status.rc_signal_lost && is_armed) {
				enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);
				set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

				/* As long as there is RC, we can fallback to ALTCTL, or STAB. */
				/* A local position estimate is enough for POSCTL for multirotors,
				 * this enables POSCTL using e.g. flow.
				 * For fixedwing, a global position is needed. */

			} else if (check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags,
							       rc_fallback_allowed, status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
				// nothing to do - everything done in check_invalid_pos_nav_state

			} else {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			}
		}
		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* go into failsafe
		 * - if we have an engine failure
		 * - if we have vtol transition failure
		 * - on data and RC link loss */

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status_flags.vtol_transition_failure) {

			set_quadchute_nav_state(status, armed, status_flags, quadchute_act);

		} else if (status.mission_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		} else if (status.data_link_lost && data_link_loss_act_configured
			   && is_armed && !landed) {
			// Data link lost, data link loss reaction configured -> do configured reaction
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_MISSION)
			   && status_flags.rc_signal_found_once && is_armed && !landed) {
			// RC link lost, rc loss not disabled in mission, RC was used before -> RC loss reaction after delay
			// Safety pilot expects to be able to take over by RC in case anything unexpected happens
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_MISSION)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed) {
			// All links lost, no data link loss reaction configured -> immediately do RC loss reaction
			// Lost all communication, by default it's considered unsafe to continue the mission
			// This is only reached when flying mission completely without RC (it was not present since boot)
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc_and_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

		} else if (status.rc_signal_lost && (param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_MISSION)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed
			   && mission_finished) {
			// All links lost, all link loss reactions disabled -> return after mission finished
			// Pilot disabled all reactions, finish mission but then return to avoid lost vehicle
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc_and_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, link_loss_actions_t::AUTO_RTL, 0);

		} else if (!stay_in_failsafe) {
			// normal mission operation if there's no need to stay in failsafe
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* go into failsafe on a engine failure */
		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else if (status_flags.vtol_transition_failure) {

			set_quadchute_nav_state(status, armed, status_flags, quadchute_act);

		} else if (status.data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			// Data link lost, data link loss reaction configured -> do configured reaction
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status_flags.rc_signal_found_once && is_armed && !landed) {
			// RC link lost, rc loss not disabled in loiter, RC was used before -> RC loss reaction after delay
			// Safety pilot expects to be able to take over by RC in case anything unexpected happens
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed) {
			// All links lost, no data link loss reaction configured -> immediately do RC loss reaction
			// Lost all communication, by default it's considered unsafe to continue the mission
			// This is only reached when flying mission completely without RC (it was not present since boot)
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc_and_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* require global position and home, also go into failsafe on an engine failure */

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:

		// require global position and home

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;
		}

		break;

	case commander_state_s::MAIN_STATE_ORBIT:
		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// failsafe: necessary position estimate lost; switching is done in check_invalid_pos_nav_state

			// Orbit can only be started via vehicle_command (mavlink). Consequently, recovery from failsafe into orbit
			// is not possible and therefore the internal_state needs to be adjusted.
			internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;

		} else if (status.data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			// failsafe: just datalink is lost and we're in air
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_datalink);

			// Orbit can only be started via vehicle_command (mavlink). Consequently, recovery from failsafe into orbit
			// is not possible and therefore the internal_state needs to be adjusted.
			internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;

		} else if (status.rc_signal_lost && status.data_link_lost && !data_link_loss_act_configured && is_armed) {
			// Orbit does not depend on RC but while armed & all links lost & when datalink loss is not set up, we failsafe
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);

			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

			// Orbit can only be started via vehicle_command (mavlink). Consequently, recovery from failsafe into orbit
			// is not possible and therefore the internal_state needs to be adjusted.
			internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;

		} else {
			// no failsafe, RC is not mandatory for orbit
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_ORBIT;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:
	case commander_state_s::MAIN_STATE_AUTO_VTOL_TAKEOFF:

		// require local position

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else if (status_flags.vtol_transition_failure) {

			set_quadchute_nav_state(status, armed, status_flags, quadchute_act);

		} else if (status.data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			// Data link lost, data link loss reaction configured -> do configured reaction
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status_flags.rc_signal_found_once && is_armed && !landed) {
			// RC link lost, rc loss not disabled in loiter, RC was used before -> RC loss reaction after delay
			// Safety pilot expects to be able to take over by RC in case anything unexpected happens
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed) {
			// All links lost, no data link loss reaction configured -> immediately do RC loss reaction
			// Lost all communication, by default it's considered unsafe to continue the mission
			// This is only reached when flying mission completely without RC (it was not present since boot)
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc_and_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

		} else {
			status.nav_state = internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF ?
					   vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF : vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LAND:

		// require local position

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {

			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND:

		// must be rotary wing plus same requirements as normal landing

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		if (status_flags.offboard_control_signal_lost) {
			if (status.rc_signal_lost) {
				// Offboard and RC are lost
				enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc_and_no_offboard);
				set_offboard_loss_nav_state(status, armed, status_flags, offb_loss_act);

			} else {
				// Offboard is lost, RC is ok
				if (param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_OFFBOARD) {
					enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_offboard);
					set_offboard_loss_nav_state(status, armed, status_flags, offb_loss_act);

				} else {
					enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_offboard);
					set_offboard_loss_rc_nav_state(status, armed, status_flags, offb_loss_rc_act);

				}

			}

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_OFFBOARD)) {
			// Only RC is lost
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		}

		break;

	default:
		break;
	}

	return status.nav_state != nav_state_old;
}

bool check_invalid_pos_nav_state(vehicle_status_s &status, bool old_failsafe, orb_advert_t *mavlink_log_pub,
				 const vehicle_status_flags_s &status_flags, const bool use_rc, const bool using_global_pos)
{
	bool fallback_required = false;

	if (using_global_pos && !status_flags.global_position_valid) {
		fallback_required = true;

	} else if (!using_global_pos
		   && (!status_flags.local_position_valid || !status_flags.local_velocity_valid)) {

		fallback_required = true;
	}

	if (fallback_required) {
		if (use_rc) {
			// fallback to a mode that gives the operator stick control
			if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
			    && status_flags.local_position_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

			} else if (status_flags.local_altitude_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

			} else {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
			}

		} else {
			// go into a descent that does not require stick control
			if (status_flags.local_position_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags.local_altitude_valid) {

				status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}
		}

		if (using_global_pos) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_global_position);

		} else {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_local_position);
		}

	}

	return fallback_required;

}

void set_link_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
			     const vehicle_status_flags_s &status_flags, commander_state_s &internal_state, link_loss_actions_t link_loss_act,
			     const float ll_delay)
{
	if (hrt_elapsed_time(&status.failsafe_timestamp) < (ll_delay * 1_s)
	    && link_loss_act != link_loss_actions_t::DISABLED) {
		// delay failsafe reaction by trying to hold position if configured
		link_loss_act = link_loss_actions_t::AUTO_LOITER;
	}

	// do the best you can according to the action set
	switch (link_loss_act) {
	case link_loss_actions_t::DISABLED:
		// If datalink loss failsafe is disabled then no action must be taken.
		break;

	case link_loss_actions_t::AUTO_RTL:
		if (status_flags.global_position_valid && status_flags.home_position_valid) {
			main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags, internal_state);
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
			return;
		}

	// FALLTHROUGH
	case link_loss_actions_t::AUTO_LOITER:
		if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			return;
		}

	// FALLTHROUGH
	case link_loss_actions_t::AUTO_LAND:
		if (status_flags.global_position_valid) {
			main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags, internal_state);
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			return;

		} else {
			if (status_flags.local_position_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
				return;

			} else if (status_flags.local_altitude_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
				return;
			}

		}

	// FALLTHROUGH
	case link_loss_actions_t::TERMINATE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed.force_failsafe = true;
		break;

	case link_loss_actions_t::LOCKDOWN:
		armed.lockdown = true;
		break;
	}
}

void reset_link_loss_globals(actuator_armed_s &armed, const bool old_failsafe, const link_loss_actions_t link_loss_act)
{
	if (old_failsafe) {
		if (link_loss_act == link_loss_actions_t::TERMINATE) {
			armed.force_failsafe = false;

		} else if (link_loss_act == link_loss_actions_t::LOCKDOWN) {
			armed.lockdown = false;
		}
	}
}

void set_quadchute_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
			     const vehicle_status_flags_s &status_flags,
			     const quadchute_actions_t quadchute_act)
{
	switch (quadchute_act) {
	case quadchute_actions_t::NO_ACTION:
		// If quadchute action is disabled then no action must be taken.
		break;

	default:
	case quadchute_actions_t::AUTO_RTL:
		if (status_flags.global_position_valid && status_flags.home_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		break;

	// FALLTHROUGH
	case quadchute_actions_t::AUTO_LAND:
		if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		}

		break;

	// FALLTHROUGH
	case quadchute_actions_t::AUTO_LOITER:
		if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		break;
	}
}

void set_offboard_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				 const vehicle_status_flags_s &status_flags,
				 const offboard_loss_actions_t offboard_loss_act)
{
	switch (offboard_loss_act) {
	case offboard_loss_actions_t::DISABLED:
		// If offboard loss failsafe is disabled then no action must be taken.
		return;

	case offboard_loss_actions_t::TERMINATE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed.force_failsafe = true;
		return;

	case offboard_loss_actions_t::LOCKDOWN:
		armed.lockdown = true;
		return;

	case offboard_loss_actions_t::AUTO_RTL:
		if (status_flags.global_position_valid && status_flags.home_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_actions_t::AUTO_LOITER:
		if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_actions_t::AUTO_LAND:
		if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			return;
		}
	}

	// If none of the above worked, try to mitigate
	if (status_flags.local_altitude_valid) {
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

	} else {
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
	}
}

void set_offboard_loss_rc_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				    const vehicle_status_flags_s &status_flags,
				    const offboard_loss_rc_actions_t offboard_loss_rc_act)
{
	switch (offboard_loss_rc_act) {
	case offboard_loss_rc_actions_t::DISABLED:
		// If offboard loss failsafe is disabled then no action must be taken.
		return;

	case offboard_loss_rc_actions_t::TERMINATE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed.force_failsafe = true;
		return;

	case offboard_loss_rc_actions_t::LOCKDOWN:
		armed.lockdown = true;
		return;

	case offboard_loss_rc_actions_t::MANUAL_POSITION:
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && status_flags.local_position_valid) {

			status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			return;

		} else if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::MANUAL_ALTITUDE:
		if (status_flags.local_altitude_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::MANUAL_ATTITUDE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
		return;

	case offboard_loss_rc_actions_t::AUTO_RTL:
		if (status_flags.global_position_valid && status_flags.home_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::AUTO_LOITER:
		if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::AUTO_LAND:
		if (status_flags.global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			return;
		}
	}

	// If none of the above worked, try to mitigate
	if (status_flags.local_altitude_valid) {
		//TODO: Add case for rover
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

	} else {
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
	}
}

void reset_offboard_loss_globals(actuator_armed_s &armed, const bool old_failsafe,
				 const offboard_loss_actions_t offboard_loss_act,
				 const offboard_loss_rc_actions_t offboard_loss_rc_act)
{
	if (old_failsafe) {
		if (offboard_loss_act == offboard_loss_actions_t::TERMINATE
		    || offboard_loss_rc_act == offboard_loss_rc_actions_t::TERMINATE) {
			armed.force_failsafe = false;

		}

		if (offboard_loss_act == offboard_loss_actions_t::LOCKDOWN
		    || offboard_loss_rc_act == offboard_loss_rc_actions_t::LOCKDOWN) {
			armed.lockdown = false;
		}
	}
}

void warn_user_about_battery(orb_advert_t *mavlink_log_pub, const uint8_t battery_warning,
			     const uint8_t failsafe_action, const float param_com_bat_act_t,
			     const char *failsafe_action_string, const events::px4::enums::navigation_mode_t failsafe_action_navigation_mode)
{
	static constexpr char battery_level[] = "battery level";

	// User warning
	switch (battery_warning) {
	case battery_status_s::BATTERY_WARNING_LOW:
		mavlink_log_critical(mavlink_log_pub, "Low %s, return advised\t", battery_level);
		events::send(events::ID("commander_bat_low"), {events::Log::Warning, events::LogInternal::Info},
			     "Low battery level, return advised");

		break;

	case battery_status_s::BATTERY_WARNING_CRITICAL:
		if (failsafe_action == commander_state_s::MAIN_STATE_MAX) {
			mavlink_log_critical(mavlink_log_pub, "Critical %s, return now\t", battery_level);
			events::send(events::ID("commander_bat_crit"), {events::Log::Critical, events::LogInternal::Info},
				     "Critical battery level, return now");

		} else {
			mavlink_log_critical(mavlink_log_pub, "Critical %s, executing %s in %d seconds\t", battery_level,
					     failsafe_action_string, static_cast<uint16_t>(param_com_bat_act_t));
			events::send<events::px4::enums::navigation_mode_t, uint16_t>(events::ID("commander_bat_crit_act"), {events::Log::Critical, events::LogInternal::Info},
					"Critical battery level, executing {1} in {2} seconds",
					failsafe_action_navigation_mode, static_cast<uint16_t>(param_com_bat_act_t));
		}

		break;

	case battery_status_s::BATTERY_WARNING_EMERGENCY:
		if (failsafe_action == commander_state_s::MAIN_STATE_MAX) {
			mavlink_log_emergency(mavlink_log_pub, "Dangerous %s, land now\t", battery_level);
			events::send(events::ID("commander_bat_emerg"), {events::Log::Emergency, events::LogInternal::Info},
				     "Dangerous battery level, land now");

		} else {
			mavlink_log_emergency(mavlink_log_pub, "Dangerous %s, executing %s in %d seconds\t", battery_level,
					      failsafe_action_string, static_cast<uint16_t>(param_com_bat_act_t));
			events::send<events::px4::enums::navigation_mode_t, uint16_t>(events::ID("commander_bat_emerg_act"), {events::Log::Emergency, events::LogInternal::Info},
					"Dangerous battery level, executing {1} in {2} seconds",
					failsafe_action_navigation_mode, static_cast<uint16_t>(param_com_bat_act_t));
		}

		break;

	case battery_status_s::BATTERY_WARNING_FAILED:
		mavlink_log_emergency(mavlink_log_pub, "Battery failure detected, land and check battery\t");
		events::send(events::ID("commander_bat_failure"), events::Log::Emergency,
			     "Battery failure detected, land and check battery");
		break;

	case battery_status_s::BATTERY_WARNING_NONE: break; // no warning
	}
}

uint8_t get_battery_failsafe_action(const commander_state_s &internal_state, const uint8_t battery_warning,
				    const low_battery_action_t param_com_low_bat_act)
{
	uint8_t ret = commander_state_s::MAIN_STATE_MAX;

	const bool already_landing = internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND
				     || internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND;
	const bool already_landing_or_rtl = already_landing
					    || internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL;

	// The main state is directly changed for the action because we need the fallbacks by the navigation state.
	switch (battery_warning) {
	case battery_status_s::BATTERY_WARNING_NONE:
	case battery_status_s::BATTERY_WARNING_LOW:
		break;

	case battery_status_s::BATTERY_WARNING_CRITICAL:
		switch (param_com_low_bat_act) {
		case LOW_BAT_ACTION::RETURN:
		case LOW_BAT_ACTION::RETURN_OR_LAND:
			if (!already_landing_or_rtl) {
				ret = commander_state_s::MAIN_STATE_AUTO_RTL;
			}

			break;

		case LOW_BAT_ACTION::LAND:
			if (!already_landing) {
				ret = commander_state_s::MAIN_STATE_AUTO_LAND;
			}

			break;

		case LOW_BAT_ACTION::WARNING: break; // no action
		}

		break;

	case battery_status_s::BATTERY_WARNING_EMERGENCY:
		switch (param_com_low_bat_act) {
		case LOW_BAT_ACTION::RETURN:
			if (!already_landing_or_rtl) {
				ret = commander_state_s::MAIN_STATE_AUTO_RTL;
			}

			break;

		case LOW_BAT_ACTION::RETURN_OR_LAND:
		case LOW_BAT_ACTION::LAND:
			if (!already_landing) {
				ret = commander_state_s::MAIN_STATE_AUTO_LAND;
			}

			break;

		case LOW_BAT_ACTION::WARNING: break; // no action
		}

		break;
	}

	return ret;
}

void imbalanced_prop_failsafe(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
			      const vehicle_status_flags_s &status_flags, commander_state_s *internal_state,
			      const imbalanced_propeller_action_t failsafe_action)
{
	static constexpr char failure_msg[] = "Imbalanced propeller detected";

	switch (failsafe_action) {
	case imbalanced_propeller_action_t::DISABLED:
		break;

	case imbalanced_propeller_action_t::WARNING:
		mavlink_log_warning(mavlink_log_pub, "%s, landing advised", failure_msg);
		break;

	case imbalanced_propeller_action_t::RETURN:

		if (status_flags.global_position_valid && status_flags.home_position_valid) {
			if (!(internal_state->main_state == commander_state_s::MAIN_STATE_AUTO_RTL ||
			      internal_state->main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
			      internal_state->main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {

				internal_state->main_state = commander_state_s::MAIN_STATE_AUTO_RTL;
				internal_state->timestamp = hrt_absolute_time();
				mavlink_log_warning(mavlink_log_pub, "%s, executing RTL", failure_msg);
			}

		} else {
			if (!(internal_state->main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
			      internal_state->main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {
				internal_state->main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
				internal_state->timestamp = hrt_absolute_time();
				mavlink_log_warning(mavlink_log_pub, "%s, can't execute RTL, landing instead", failure_msg);
			}
		}

		break;

	case imbalanced_propeller_action_t::LAND:
		if (!(internal_state->main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
		      internal_state->main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {
			internal_state->main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
			internal_state->timestamp = hrt_absolute_time();
			mavlink_log_warning(mavlink_log_pub, "%s, landing", failure_msg);
		}

		break;
	}
}

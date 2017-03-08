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
#include <px4_config.h>

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>

#include <px4_posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/differential_pressure.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_device.h>

#include "state_machine_helper.h"
#include "commander_helper.h"
#include "PreflightCheck.h"
#ifndef __PX4_NUTTX
#include "DevMgr.hpp"
using namespace DriverFramework;
#endif

static const char reason_no_rc[] = "no RC";
static const char reason_no_offboard[] = "no offboard";
static const char reason_no_rc_and_no_offboard[] = "no RC and no offboard";
static const char reason_no_gps[] = "no gps";
static const char reason_no_gps_cmd[] = "no gps cmd";
static const char reason_no_local_position[] = "no local position";
static const char reason_no_global_position[] = "no global position";
static const char reason_no_datalink[] = "no datalink";

// This array defines the arming state transitions. The rows are the new state, and the columns
// are the current state. Using new state and current state you can index into the array which
// will be true for a valid transition or false for a invalid transition. In some cases even
// though the transition is marked as true additional checks must be made. See arming_state_transition
// code for those checks.
static const bool arming_transitions[vehicle_status_s::ARMING_STATE_MAX][vehicle_status_s::ARMING_STATE_MAX] = {
	//                                                    INIT,  STANDBY, ARMED, ARMED_ERROR, STANDBY_ERROR, REBOOT, IN_AIR_RESTORE
	{ /* vehicle_status_s::ARMING_STATE_INIT */           true,  true,    false, false,       true,          false,  false },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY */        true,  true,    true,  true,        false,         false,  false },
	{ /* vehicle_status_s::ARMING_STATE_ARMED */          false, true,    true,  false,       false,         false,  true },
	{ /* vehicle_status_s::ARMING_STATE_ARMED_ERROR */    false, false,   true,  true,        false,         false,  false },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY_ERROR */  true,  true,    true,  true,        true,          false,  false },
	{ /* vehicle_status_s::ARMING_STATE_REBOOT */         true,  true,    false, false,       true,          true,   true },
	{ /* vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE */ false, false,   false, false,       false,         false,  false }, // NYI
};

// You can index into the array with an arming_state_t in order to get its textual representation
static const char *const state_names[vehicle_status_s::ARMING_STATE_MAX] = {
	"ARMING_STATE_INIT",
	"ARMING_STATE_STANDBY",
	"ARMING_STATE_ARMED",
	"ARMING_STATE_ARMED_ERROR",
	"ARMING_STATE_STANDBY_ERROR",
	"ARMING_STATE_REBOOT",
	"ARMING_STATE_IN_AIR_RESTORE",
};

static hrt_abstime last_preflight_check = 0;	///< initialize so it gets checked immediately
static int last_prearm_ret = 1;			///< initialize to fail

void set_link_loss_nav_state(struct vehicle_status_s *status,
			     struct actuator_armed_s *armed,
			     status_flags_s *status_flags,
			     const link_loss_actions_t link_loss_act,
			     uint8_t auto_recovery_nav_state);

void reset_link_loss_globals(struct actuator_armed_s *armed,
			     const bool old_failsafe,
			     const link_loss_actions_t link_loss_act);

transition_result_t arming_state_transition(struct vehicle_status_s *status,
                                            struct battery_status_s *battery,
                                            const struct safety_s *safety,
                                            arming_state_t new_arming_state,
                                            struct actuator_armed_s *armed,
                                            bool fRunPreArmChecks,
                                            orb_advert_t *mavlink_log_pub,	///< uORB handle for mavlink log
                                            status_flags_s *status_flags,
                                            float avionics_power_rail_voltage,
                                            bool can_arm_without_gps,
                                            hrt_abstime time_since_boot)
{
	// Double check that our static arrays are still valid
	static_assert(vehicle_status_s::ARMING_STATE_INIT == 0, "ARMING_STATE_INIT == 0");
	static_assert(vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE == vehicle_status_s::ARMING_STATE_MAX - 1,
		      "ARMING_STATE_IN_AIR_RESTORE == ARMING_STATE_MAX - 1");

	transition_result_t ret = TRANSITION_DENIED;
	arming_state_t current_arming_state = status->arming_state;
	bool feedback_provided = false;

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == current_arming_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		/*
		 * Get sensing state if necessary
		 */
		int prearm_ret = OK;

		/* only perform the pre-arm check if we have to */
		if (fRunPreArmChecks && new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
		    && status->hil_state == vehicle_status_s::HIL_STATE_OFF) {

			prearm_ret = preflight_check(status, mavlink_log_pub, true /* pre-arm */, false /* force_report */,
						     status_flags, battery, can_arm_without_gps, time_since_boot);
		}

		/* re-run the pre-flight check as long as sensors are failing */
		if (!status_flags->condition_system_sensors_initialized
		    && (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
			|| new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
		    && status->hil_state == vehicle_status_s::HIL_STATE_OFF) {

			if (last_preflight_check == 0 || hrt_absolute_time() - last_preflight_check > 1000 * 1000) {
				prearm_ret = preflight_check(status, mavlink_log_pub, false /* pre-flight */, false /* force_report */,
							     status_flags, battery, can_arm_without_gps, time_since_boot);
				status_flags->condition_system_sensors_initialized = (prearm_ret == OK);
				last_preflight_check = hrt_absolute_time();
				last_prearm_ret = prearm_ret;

			} else {
				prearm_ret = last_prearm_ret;
			}
		}

		/*
		 * Perform an atomic state update
		 */
#ifdef __PX4_NUTTX
		irqstate_t flags = px4_enter_critical_section();
#endif

		/* enforce lockdown in HIL */
		if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
			armed->lockdown = true;
			prearm_ret = OK;
			status_flags->condition_system_sensors_initialized = true;

			/* recover from a prearm fail */
			if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {
				status->arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
			}

		} else {
			armed->lockdown = false;
		}

		// Check that we have a valid state transition
		bool valid_transition = arming_transitions[new_arming_state][status->arming_state];

		if (valid_transition) {
			// We have a good transition. Now perform any secondary validation.
			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				//      Do not perform pre-arm checks if coming from in air restore
				//      Allow if vehicle_status_s::HIL_STATE_ON
				if (status->arming_state != vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE &&
				    status->hil_state == vehicle_status_s::HIL_STATE_OFF) {

					// Fail transition if pre-arm check fails
					if (prearm_ret) {
						/* the prearm check already prints the reject reason */
						feedback_provided = true;
						valid_transition = false;

						// Fail transition if we need safety switch press

					} else if (safety->safety_switch_available && !safety->safety_off) {

						mavlink_log_critical(mavlink_log_pub, "NOT ARMING: Press safety switch first!");
						feedback_provided = true;
						valid_transition = false;
					}

					// Perform power checks only if circuit breaker is not
					// engaged for these checks
					if (!status_flags->circuit_breaker_engaged_power_check) {
						// Fail transition if power is not good
						if (!status_flags->condition_power_input_valid) {

							mavlink_log_critical(mavlink_log_pub, "NOT ARMING: Connect power module.");
							feedback_provided = true;
							valid_transition = false;
						}

						// Fail transition if power levels on the avionics rail
						// are measured but are insufficient
						if (status_flags->condition_power_input_valid && (avionics_power_rail_voltage > 0.0f)) {
							// Check avionics rail voltages
							if (avionics_power_rail_voltage < 4.5f) {
								mavlink_log_critical(mavlink_log_pub, "NOT ARMING: Avionics power low: %6.2f Volt",
										     (double)avionics_power_rail_voltage);
								feedback_provided = true;
								valid_transition = false;

							} else if (avionics_power_rail_voltage < 4.9f) {
								mavlink_log_critical(mavlink_log_pub, "CAUTION: Avionics power low: %6.2f Volt", (double)avionics_power_rail_voltage);
								feedback_provided = true;

							} else if (avionics_power_rail_voltage > 5.4f) {
								mavlink_log_critical(mavlink_log_pub, "CAUTION: Avionics power high: %6.2f Volt", (double)avionics_power_rail_voltage);
								feedback_provided = true;
							}
						}
					}

				}

			} else if (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY
				   && status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) {
				new_arming_state = vehicle_status_s::ARMING_STATE_STANDBY_ERROR;
			}
		}

		// HIL can always go to standby
		if (status->hil_state == vehicle_status_s::HIL_STATE_ON && new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			valid_transition = true;
		}

		// Check if we are trying to arm, checks look good but we are in STANDBY_ERROR
		if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {

			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				if (status_flags->condition_system_sensors_initialized) {
					mavlink_log_critical(mavlink_log_pub, "Preflight check resolved, reboot before arming");

				} else {
					mavlink_log_critical(mavlink_log_pub, "Preflight check failed, refusing to arm");
				}

				feedback_provided = true;

			} else if ((new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) &&
				   status_flags->condition_system_sensors_initialized) {
				mavlink_log_critical(mavlink_log_pub, "Preflight check resolved, reboot to complete");
				feedback_provided = true;

			} else {
				// Silent ignore
				feedback_provided = true;
			}

			// Sensors need to be initialized for STANDBY state, except for HIL

		} else if ((status->hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) &&
			   (status->arming_state != vehicle_status_s::ARMING_STATE_STANDBY_ERROR)) {

			if (!status_flags->condition_system_sensors_initialized) {

				if (status_flags->condition_system_hotplug_timeout) {
					if (!status_flags->condition_system_prearm_error_reported) {
						mavlink_log_critical(mavlink_log_pub, "Not ready to fly: Sensors not set up correctly");
						status_flags->condition_system_prearm_error_reported = true;
					}
				}

				feedback_provided = true;
				valid_transition = false;
			}
		}

		// Finish up the state transition
		if (valid_transition) {
			armed->armed = new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
				       || new_arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR;
			armed->ready_to_arm = new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
					      || new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY;
			ret = TRANSITION_CHANGED;
			status->arming_state = new_arming_state;
		}

		/* reset feedback state */
		if (status->arming_state != vehicle_status_s::ARMING_STATE_STANDBY_ERROR &&
		    status->arming_state != vehicle_status_s::ARMING_STATE_INIT &&
		    valid_transition) {
			status_flags->condition_system_prearm_error_reported = false;
		}

		/* end of atomic state update */
#ifdef __PX4_NUTTX
		px4_leave_critical_section(flags);
#endif
	}

	if (ret == TRANSITION_DENIED) {
		/* print to MAVLink and console if we didn't provide any feedback yet */
		if (!feedback_provided) {
			mavlink_log_critical(mavlink_log_pub, "TRANSITION_DENIED: %s - %s", state_names[status->arming_state],
					     state_names[new_arming_state]);
		}
	}

	return ret;
}

bool is_safe(const struct safety_s *safety, const struct actuator_armed_s *armed)
{
	// System is safe if:
	// 1) Not armed
	// 2) Armed, but in software lockdown (HIL)
	// 3) Safety switch is present AND engaged -> actuators locked
	const bool lockdown = (armed->lockdown || armed->manual_lockdown);

	if (!armed->armed || (armed->armed && lockdown) || (safety->safety_switch_available && !safety->safety_off)) {
		return true;

	} else {
		return false;
	}
}

transition_result_t
main_state_transition(struct vehicle_status_s *status, main_state_t new_main_state, uint8_t &main_state_prev,
		      status_flags_s *status_flags, struct commander_state_s *internal_state)
{
	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if the same state is requested because conditions may have changed */
	switch (new_main_state) {
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ACRO:
	case commander_state_s::MAIN_STATE_RATTITUDE:
		ret = TRANSITION_CHANGED;
		break;

	case commander_state_s::MAIN_STATE_ALTCTL:

		/* need at minimum altitude estimate */
		if (status_flags->condition_local_altitude_valid ||
		    status_flags->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL:

		/* need at minimum local position estimate */
		if (status_flags->condition_local_position_valid ||
		    status_flags->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* need global position estimate */
		if (status_flags->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:

		/* FOLLOW only implemented in MC */
		if (status->is_rotary_wing) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:
	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* need global position and home position */
		if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:
	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* need local position */
		if (status_flags->condition_local_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* need offboard signal
		 */
		if (!status_flags->offboard_control_signal_lost) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_MAX:
	default:
		break;
	}

	if (ret == TRANSITION_CHANGED) {
		if (internal_state->main_state != new_main_state) {
			main_state_prev = internal_state->main_state;
			internal_state->main_state = new_main_state;
			internal_state->timestamp = hrt_absolute_time();

		} else {
			ret = TRANSITION_NOT_CHANGED;
		}
	}

	return ret;
}

/**
 * Transition from one hil state to another
 */
transition_result_t hil_state_transition(hil_state_t new_state, orb_advert_t status_pub,
		struct vehicle_status_s *current_status, orb_advert_t *mavlink_log_pub)
{
	transition_result_t ret = TRANSITION_DENIED;

	if (current_status->hil_state == new_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {
		switch (new_state) {
		case vehicle_status_s::HIL_STATE_OFF:
			/* we're in HIL and unexpected things can happen if we disable HIL now */
			mavlink_log_critical(mavlink_log_pub, "Not switching off HIL (safety)");
			ret = TRANSITION_DENIED;
			break;

		case vehicle_status_s::HIL_STATE_ON:
			if (current_status->arming_state == vehicle_status_s::ARMING_STATE_INIT
			    || current_status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY
			    || current_status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {

				ret = TRANSITION_CHANGED;
				mavlink_log_critical(mavlink_log_pub, "Switched to ON hil state");

			} else {
				mavlink_log_critical(mavlink_log_pub, "Not switching to HIL when armed");
				ret = TRANSITION_DENIED;
			}

			break;

		default:
			warnx("Unknown HIL state");
			break;
		}
	}

	if (ret == TRANSITION_CHANGED) {
		current_status->hil_state = new_state;
		current_status->timestamp = hrt_absolute_time();
		// XXX also set lockdown here
		orb_publish(ORB_ID(vehicle_status), status_pub, current_status);
	}

	return ret;
}

/**
 * Enable failsafe and report to user
 */
void enable_failsafe(struct vehicle_status_s *status, bool old_failsafe, orb_advert_t *mavlink_log_pub,
		     const char *reason)
{
	if (!old_failsafe) {
		mavlink_log_critical(mavlink_log_pub, "Failsafe enabled: %s", reason);
	}

	status->failsafe = true;
}

/**
 * Check failsafe and main status and set navigation status for navigator accordingly
 */
bool set_nav_state(struct vehicle_status_s *status,
		   struct actuator_armed_s *armed,
		   struct commander_state_s *internal_state,
		   orb_advert_t *mavlink_log_pub,
		   const link_loss_actions_t data_link_loss_act,
		   const bool mission_finished,
		   const bool stay_in_failsafe,
		   status_flags_s *status_flags,
		   bool landed,
		   const link_loss_actions_t rc_loss_act,
		   const int offb_loss_act,
		   const int offb_loss_rc_act)
{
	navigation_state_t nav_state_old = status->nav_state;

	const bool data_link_loss_act_configured = data_link_loss_act > link_loss_actions_t::DISABLED;
	const bool rc_loss_act_configured = rc_loss_act > link_loss_actions_t::DISABLED;
	const bool rc_lost = rc_loss_act_configured && (status->rc_signal_lost || status_flags->rc_signal_lost_cmd);

	bool is_armed = (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED
			 || status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);
	bool old_failsafe = status->failsafe;
	status->failsafe = false;

	// Safe to do reset flags here, as if loss state persists flags will be restored in the code below
	reset_link_loss_globals(armed, old_failsafe, rc_loss_act);
	reset_link_loss_globals(armed, old_failsafe, data_link_loss_act);

	/* evaluate main state to decide in normal (non-failsafe) mode */
	switch (internal_state->main_state) {
	case commander_state_s::MAIN_STATE_ACRO:
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_RATTITUDE:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ALTCTL:

		/* require RC for all manual modes */
		if (rc_lost && is_armed) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

			set_rc_loss_nav_state(status, armed, status_flags, rc_loss_act);

		} else {
			switch (internal_state->main_state) {
			case commander_state_s::MAIN_STATE_ACRO:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ACRO;
				break;

			case commander_state_s::MAIN_STATE_MANUAL:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;

			case commander_state_s::MAIN_STATE_RATTITUDE:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_RATTITUDE;
				break;

			case commander_state_s::MAIN_STATE_STAB:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
				break;

			case commander_state_s::MAIN_STATE_ALTCTL:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
				break;

			default:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;
			}
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL: {

			if (rc_lost && is_armed) {
				enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

				set_rc_loss_nav_state(status, armed, status_flags, rc_loss_act);

				/* As long as there is RC, we can fallback to ALTCTL, or STAB. */
				/* A local position estimate is enough for POSCTL for multirotors,
				 * this enables POSCTL using e.g. flow.
				 * For fixedwing, a global position is needed. */

			} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, true, !status->is_rotary_wing)) {
				// nothing to do - everything done in check_invalid_pos_nav_state
			} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, true, status->is_rotary_wing)) {
				// nothing to do - everything done in check_invalid_pos_nav_state
			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			}
		}
		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* go into failsafe
		 * - if commanded to do so
		 * - if we have an engine failure
		 * - if we have vtol transition failure
		 * - depending on datalink, RC and if the mission is finished */

		/* first look at the commands */
		if (status->engine_failure_cmd) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->data_link_lost_cmd) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS;

		} else if (status_flags->gps_failure_cmd) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_gps_cmd);

		} else if (status_flags->rc_signal_lost_cmd) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER;

		} else if (status_flags->vtol_transition_failure_cmd) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

			/* finished handling commands which have priority, now handle failures */

		} else if (status_flags->gps_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_gps);

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->vtol_transition_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		} else if (status->mission_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

			/* datalink loss enabled:
			 * check for datalink lost: this should always trigger RTGS */

		} else if (data_link_loss_act_configured && status->data_link_lost) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);

			set_data_link_loss_nav_state(status, armed, status_flags, data_link_loss_act);

			/* datalink loss DISABLED:
			 * check if both, RC and datalink are lost during the mission
			 * or all links are lost after the mission finishes in air: this should always trigger RCRECOVER */

		} else if (!data_link_loss_act_configured && status->rc_signal_lost && status->data_link_lost && !landed
			   && mission_finished) {

			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);

			set_rc_loss_nav_state(status, armed, status_flags, rc_loss_act);

			/* stay where you are if you should stay in failsafe, otherwise everything is perfect */

		} else if (!stay_in_failsafe) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* go into failsafe on a engine failure */
		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->gps_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_gps);

			/* also go into failsafe if just datalink is lost, and we're actually in air */

			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);
		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status->data_link_lost && data_link_loss_act_configured && !landed) {

			set_data_link_loss_nav_state(status, armed, status_flags, data_link_loss_act);

			/* go into failsafe if RC is lost and datalink loss is not set up and rc loss is not DISABLED */

		} else if (rc_lost && !data_link_loss_act_configured) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

			set_rc_loss_nav_state(status, armed, status_flags, rc_loss_act);

			/* don't bother if RC is lost if datalink is connected */

		} else if (status->rc_signal_lost) {

			/* this mode is ok, we don't need RC for LOITERing */
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

		} else {
			/* everything is perfect */
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* require global position and home, also go into failsafe on an engine failure */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->gps_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_gps);

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:

		/* require global position and home */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:

		/* require local position */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* require local position */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* require offboard control, otherwise stay where you are */
		if (status_flags->offboard_control_signal_lost && !status->rc_signal_lost) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_offboard);

			if (status_flags->offboard_control_loss_timeout && offb_loss_rc_act < 5 && offb_loss_rc_act >= 0) {
				if (offb_loss_rc_act == 3 && status_flags->condition_global_position_valid
				    && status_flags->condition_home_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				} else if (offb_loss_rc_act == 0 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

				} else if (offb_loss_rc_act == 1 && status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

				} else if (offb_loss_rc_act == 2) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

				} else if (offb_loss_rc_act == 4 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}

			} else {
				if (status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}
			}

		} else if (status_flags->offboard_control_signal_lost && status->rc_signal_lost) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc_and_no_offboard);

			if (status_flags->offboard_control_loss_timeout && offb_loss_act < 3 && offb_loss_act >= 0) {
				if (offb_loss_act == 2 && status_flags->condition_global_position_valid
				    && status_flags->condition_home_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				} else if (offb_loss_act == 1 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

				} else if (offb_loss_act == 0 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}

			} else {
				if (status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}
			}

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		}

	default:
		break;
	}

	return status->nav_state != nav_state_old;
}

void set_rc_loss_nav_state(struct vehicle_status_s *status,
			   struct actuator_armed_s *armed,
			   status_flags_s *status_flags,
			   const link_loss_actions_t link_loss_act)
{
	set_link_loss_nav_state(status, armed, status_flags, link_loss_act, vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);
}

bool check_invalid_pos_nav_state(struct vehicle_status_s *status,
			       bool old_failsafe,
			       orb_advert_t *mavlink_log_pub,
			       status_flags_s *status_flags,
			       const bool use_rc, // true if we can fallback to a mode that uses RC inputs
			       const bool using_global_pos) // true if the current flight mode requires a global position
{
	bool fallback_required = false;

	if (using_global_pos && (!status_flags->condition_global_position_valid || !status_flags->condition_global_velocity_valid)) {
		fallback_required = true;
	} else if (!using_global_pos && (!status_flags->condition_local_position_valid || !status_flags->condition_local_velocity_valid)) {
		fallback_required = true;
	}

	if (fallback_required) {
		if (use_rc) {
			// fallback to a mode that gives the operator stick control
			if (status->is_rotary_wing && status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
			}
		} else {
			// go into a descent that does not require stick control
			if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			} else  if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}
		}

		if (using_global_pos) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_global_position);
		} else {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_local_position);
		}

	}

	return fallback_required;

}

void set_data_link_loss_nav_state(struct vehicle_status_s *status,
				  struct actuator_armed_s *armed,
				  status_flags_s *status_flags,
				  const link_loss_actions_t link_loss_act)
{
	set_link_loss_nav_state(status, armed, status_flags, link_loss_act, vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS);
}

void set_link_loss_nav_state(struct vehicle_status_s *status,
			     struct actuator_armed_s *armed,
			     status_flags_s *status_flags,
			     const link_loss_actions_t link_loss_act,
			     uint8_t auto_recovery_nav_state)
{
	// do the best you can according to the action set
	if (link_loss_act == link_loss_actions_t::AUTO_RECOVER
	    && status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
		status->nav_state = auto_recovery_nav_state;

	} else if (link_loss_act == link_loss_actions_t::AUTO_LOITER && status_flags->condition_global_position_valid) {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	} else if (link_loss_act == link_loss_actions_t::AUTO_RTL
		   && status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

	} else if (link_loss_act == link_loss_actions_t::AUTO_LAND && status_flags->condition_local_position_valid) {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

	} else if (link_loss_act == link_loss_actions_t::TERMINATE) {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed->force_failsafe = true;

	} else if (link_loss_act == link_loss_actions_t::LOCKDOWN) {
		armed->lockdown = true;

		// do the best you can according to the current state

	} else if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

	} else if (status_flags->condition_local_position_valid) {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

	} else if (status_flags->condition_local_altitude_valid) {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

	} else {
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
	}
}

void reset_link_loss_globals(struct actuator_armed_s *armed, const bool old_failsafe,
			     const link_loss_actions_t link_loss_act)
{
	if (old_failsafe) {
		if (link_loss_act == link_loss_actions_t::TERMINATE) {
			armed->force_failsafe = false;

		} else if (link_loss_act == link_loss_actions_t::LOCKDOWN) {
			armed->lockdown = false;
		}
	}
}

int preflight_check(struct vehicle_status_s *status, orb_advert_t *mavlink_log_pub, bool prearm, bool force_report,
		    status_flags_s *status_flags, battery_status_s *battery, bool can_arm_without_gps, hrt_abstime time_since_boot)
{
	bool reportFailures = force_report || (!status_flags->condition_system_prearm_error_reported &&
					       status_flags->condition_system_hotplug_timeout);

	bool checkAirspeed = false;

	/* Perform airspeed check only if circuit breaker is not
	 * engaged and it's not a rotary wing */
	if (!status_flags->circuit_breaker_engaged_airspd_check && (!status->is_rotary_wing || status->is_vtol)) {
		checkAirspeed = true;
	}

	bool preflight_ok = Commander::preflightCheck(mavlink_log_pub, true, true, true, true,
			    checkAirspeed, (status->rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT),
			    !can_arm_without_gps, true, status->is_vtol, reportFailures, prearm, time_since_boot);

	if (!status_flags->circuit_breaker_engaged_usb_check && status_flags->usb_connected && prearm) {
		preflight_ok = false;

		if (reportFailures) {
			mavlink_log_critical(mavlink_log_pub, "ARMING DENIED: Flying with USB is not safe");
		}
	}

	if (battery->warning == battery_status_s::BATTERY_WARNING_LOW) {
		preflight_ok = false;

		if (reportFailures) {
			mavlink_log_critical(mavlink_log_pub, "ARMING DENIED: LOW BATTERY");
		}
	}

	/* report once, then set the flag */
	if (reportFailures && !preflight_ok) {
		status_flags->condition_system_prearm_error_reported = true;
	}

	return !preflight_ok;
}

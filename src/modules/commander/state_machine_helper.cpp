/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <dirent.h>
#include <fcntl.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_device.h>
#include <mavlink/mavlink_log.h>

#include "state_machine_helper.h"
#include "commander_helper.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static bool arming_state_changed = true;
static bool main_state_changed = true;
static bool failsafe_state_changed = true;

// This array defines the arming state transitions. The rows are the new state, and the columns
// are the current state. Using new state and current  state you can index into the array which
// will be true for a valid transition or false for a invalid transition. In some cases even
// though the transition is marked as true additional checks must be made. See arming_state_transition
// code for those checks.
static const bool arming_transitions[ARMING_STATE_MAX][ARMING_STATE_MAX] = {
	//                                  INIT,   STANDBY,    ARMED,  ARMED_ERROR,    STANDBY_ERROR,  REBOOT,     IN_AIR_RESTORE
	{ /* ARMING_STATE_INIT */           true,   true,       false,  false,          false,          false,      false },
	{ /* ARMING_STATE_STANDBY */        true,   true,       true,   true,           false,          false,      false },
	{ /* ARMING_STATE_ARMED */          false,  true,       true,   false,          false,          false,      true },
	{ /* ARMING_STATE_ARMED_ERROR */    false,  false,      true,   true,           false,          false,      false },
	{ /* ARMING_STATE_STANDBY_ERROR */  true,   true,       false,  true,           true,           false,      false },
	{ /* ARMING_STATE_REBOOT */         true,   true,       false,  false,          true,           true,       true },
	{ /* ARMING_STATE_IN_AIR_RESTORE */ false,  false,      false,  false,          false,          false,      false }, // NYI
};

// You can index into the array with an arming_state_t in order to get it's textual representation
static const char *state_names[ARMING_STATE_MAX] = {
	"ARMING_STATE_INIT",
	"ARMING_STATE_STANDBY",
	"ARMING_STATE_ARMED",
	"ARMING_STATE_ARMED_ERROR",
	"ARMING_STATE_STANDBY_ERROR",
	"ARMING_STATE_REBOOT",
	"ARMING_STATE_IN_AIR_RESTORE",
};

transition_result_t
arming_state_transition(struct vehicle_status_s *status,            /// current vehicle status
			const struct safety_s   *safety,            /// current safety settings
			arming_state_t          new_arming_state,   /// arming state requested
			struct actuator_armed_s *armed,             /// current armed status
			const int               mavlink_fd)         /// mavlink fd for error reporting, 0 for none
{
	// Double check that our static arrays are still valid
	ASSERT(ARMING_STATE_INIT == 0);
	ASSERT(ARMING_STATE_IN_AIR_RESTORE == ARMING_STATE_MAX - 1);

	/*
	 * Perform an atomic state update
	 */
	irqstate_t flags = irqsave();

	transition_result_t ret = TRANSITION_DENIED;

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == status->arming_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {
		/* enforce lockdown in HIL */
		if (status->hil_state == HIL_STATE_ON) {
			armed->lockdown = true;

		} else {
			armed->lockdown = false;
		}

		// Check that we have a valid state transition
		bool valid_transition = arming_transitions[new_arming_state][status->arming_state];

		if (valid_transition) {
			// We have a good transition. Now perform any secondary validation.
			if (new_arming_state == ARMING_STATE_ARMED) {
				// Fail transition if we need safety switch press
				//      Allow if coming from in air restore
				//      Allow if HIL_STATE_ON
				if (status->arming_state != ARMING_STATE_IN_AIR_RESTORE && status->hil_state == HIL_STATE_OFF && safety->safety_switch_available && !safety->safety_off) {
					if (mavlink_fd) {
						mavlink_log_critical(mavlink_fd, "#audio: NOT ARMING: Press safety switch first.");
					}

					valid_transition = false;
				}

			} else if (new_arming_state == ARMING_STATE_STANDBY && status->arming_state == ARMING_STATE_ARMED_ERROR) {
				new_arming_state = ARMING_STATE_STANDBY_ERROR;
			}
		}

		// HIL can always go to standby
		if (status->hil_state == HIL_STATE_ON && new_arming_state == ARMING_STATE_STANDBY) {
			valid_transition = true;
		}

		/* Sensors need to be initialized for STANDBY state */
		if (new_arming_state == ARMING_STATE_STANDBY && !status->condition_system_sensors_initialized) {
			valid_transition = false;
		}

		// Finish up the state transition
		if (valid_transition) {
			armed->armed = new_arming_state == ARMING_STATE_ARMED || new_arming_state == ARMING_STATE_ARMED_ERROR;
			armed->ready_to_arm = new_arming_state == ARMING_STATE_ARMED || new_arming_state == ARMING_STATE_STANDBY;
			ret = TRANSITION_CHANGED;
			status->arming_state = new_arming_state;
			arming_state_changed = true;
		}
	}

	/* end of atomic state update */
	irqrestore(flags);

	if (ret == TRANSITION_DENIED) {
		static const char *errMsg = "Invalid arming transition from %s to %s";

		if (mavlink_fd) {
			mavlink_log_critical(mavlink_fd, errMsg, state_names[status->arming_state], state_names[new_arming_state]);
		}

		warnx(errMsg, state_names[status->arming_state], state_names[new_arming_state]);
	}

	return ret;
}

bool is_safe(const struct vehicle_status_s *status, const struct safety_s *safety, const struct actuator_armed_s *armed)
{
	// System is safe if:
	// 1) Not armed
	// 2) Armed, but in software lockdown (HIL)
	// 3) Safety switch is present AND engaged -> actuators locked
	if (!armed->armed || (armed->armed && armed->lockdown) || (safety->safety_switch_available && !safety->safety_off)) {
		return true;

	} else {
		return false;
	}
}

bool
check_arming_state_changed()
{
	if (arming_state_changed) {
		arming_state_changed = false;
		return true;

	} else {
		return false;
	}
}

transition_result_t
main_state_transition(struct vehicle_status_s *status, main_state_t new_main_state)
{
	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if requested the same state because conditions may be changed */
	switch (new_main_state) {
	case MAIN_STATE_MANUAL:
		ret = TRANSITION_CHANGED;
		break;

	case MAIN_STATE_ACRO:
		ret = TRANSITION_CHANGED;
		break;

	case MAIN_STATE_ALTCTL:

		/* need at minimum altitude estimate */
		if (!status->is_rotary_wing ||
		    (status->condition_local_altitude_valid ||
		     status->condition_global_position_valid)) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case MAIN_STATE_POSCTL:

		/* need at minimum local position estimate */
		if (status->condition_local_position_valid ||
		    status->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case MAIN_STATE_AUTO:

		/* need global position estimate */
		if (status->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;
	}

	if (ret == TRANSITION_CHANGED) {
		if (status->main_state != new_main_state) {
			status->main_state = new_main_state;
			main_state_changed = true;

		} else {
			ret = TRANSITION_NOT_CHANGED;
		}
	}

	return ret;
}

bool
check_main_state_changed()
{
	if (main_state_changed) {
		main_state_changed = false;
		return true;

	} else {
		return false;
	}
}

bool
check_failsafe_state_changed()
{
	if (failsafe_state_changed) {
		failsafe_state_changed = false;
		return true;

	} else {
		return false;
	}
}

/**
* Transition from one hil state to another
*/
int hil_state_transition(hil_state_t new_state, int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	bool valid_transition = false;
	int ret = ERROR;

	if (current_status->hil_state == new_state) {
		valid_transition = true;

	} else {

		switch (new_state) {

		case HIL_STATE_OFF:

			/* we're in HIL and unexpected things can happen if we disable HIL now */
			mavlink_log_critical(mavlink_fd, "#audio: Not switching off HIL (safety)");
			valid_transition = false;

			break;

		case HIL_STATE_ON:

			if (current_status->arming_state == ARMING_STATE_INIT
			    || current_status->arming_state == ARMING_STATE_STANDBY
			    || current_status->arming_state == ARMING_STATE_STANDBY_ERROR) {

				mavlink_log_critical(mavlink_fd, "Switched to ON hil state");
				valid_transition = true;

				// Disable publication of all attached sensors

				/* list directory */
				DIR		*d;
				d = opendir("/dev");

				if (d) {

					struct dirent	*direntry;
					char devname[24];

					while ((direntry = readdir(d)) != NULL) {

						/* skip serial ports */
						if (!strncmp("tty", direntry->d_name, 3)) {
							continue;
						}

						/* skip mtd devices */
						if (!strncmp("mtd", direntry->d_name, 3)) {
							continue;
						}

						/* skip ram devices */
						if (!strncmp("ram", direntry->d_name, 3)) {
							continue;
						}

						/* skip MMC devices */
						if (!strncmp("mmc", direntry->d_name, 3)) {
							continue;
						}

						/* skip mavlink */
						if (!strcmp("mavlink", direntry->d_name)) {
							continue;
						}

						/* skip console */
						if (!strcmp("console", direntry->d_name)) {
							continue;
						}

						/* skip null */
						if (!strcmp("null", direntry->d_name)) {
							continue;
						}

						snprintf(devname, sizeof(devname), "/dev/%s", direntry->d_name);

						int sensfd = ::open(devname, 0);

						if (sensfd < 0) {
							warn("failed opening device %s", devname);
							continue;
						}

						int block_ret = ::ioctl(sensfd, DEVIOCSPUBBLOCK, 1);
						close(sensfd);

						printf("Disabling %s: %s\n", devname, (block_ret == OK) ? "OK" : "ERROR");
					}

					closedir(d);

				} else {
					/* failed opening dir */
					warnx("FAILED LISTING DEVICE ROOT DIRECTORY");
					return 1;
				}
			}

			break;

		default:
			warnx("Unknown hil state");
			break;
		}
	}

	if (valid_transition) {
		current_status->hil_state = new_state;

		current_status->timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(vehicle_status), status_pub, current_status);

		// XXX also set lockdown here

		ret = OK;

	} else {
		mavlink_log_critical(mavlink_fd, "REJECTING invalid hil state transition");
	}

	return ret;
}


/**
* Transition from one failsafe state to another
*/
transition_result_t failsafe_state_transition(struct vehicle_status_s *status, failsafe_state_t new_failsafe_state)
{
	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if requested the same state because conditions may be changed */
	if (status->failsafe_state == FAILSAFE_STATE_TERMINATION) {
		/* transitions from TERMINATION to other states not allowed */
		if (new_failsafe_state == FAILSAFE_STATE_TERMINATION) {
			ret = TRANSITION_NOT_CHANGED;
		}

	} else {
		switch (new_failsafe_state) {
		case FAILSAFE_STATE_NORMAL:
			/* always allowed (except from TERMINATION state) */
			ret = TRANSITION_CHANGED;
			break;

		case FAILSAFE_STATE_RTL:

			/* global position and home position required for RTL */
			if (status->condition_global_position_valid && status->condition_home_position_valid) {
				status->set_nav_state = NAV_STATE_RTL;
				status->set_nav_state_timestamp = hrt_absolute_time();
				ret = TRANSITION_CHANGED;
			}

			break;

		case FAILSAFE_STATE_LAND:

			/* at least relative altitude estimate required for landing */
			if (status->condition_local_altitude_valid || status->condition_global_position_valid) {
				status->set_nav_state = NAV_STATE_LAND;
				status->set_nav_state_timestamp = hrt_absolute_time();
				ret = TRANSITION_CHANGED;
			}

			break;

		case FAILSAFE_STATE_TERMINATION:
			/* always allowed */
			ret = TRANSITION_CHANGED;
			break;

		default:
			break;
		}

		if (ret == TRANSITION_CHANGED) {
			if (status->failsafe_state != new_failsafe_state) {
				status->failsafe_state = new_failsafe_state;
				failsafe_state_changed = true;

			} else {
				ret = TRANSITION_NOT_CHANGED;
			}
		}
	}

	return ret;
}



// /*
//  * Wrapper functions (to be used in the commander), all functions assume lock on current_status
//  */

// /* These functions decide if an emergency exits and then switch to SYSTEM_STATE_MISSION_ABORT or SYSTEM_STATE_GROUND_ERROR
//  *
//  * START SUBSYSTEM/EMERGENCY FUNCTIONS
//  * */

// void update_state_machine_subsystem_present(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type)
// {
// 	current_status->onboard_control_sensors_present |= 1 << *subsystem_type;
// 	current_status->counter++;
// 	current_status->timestamp = hrt_absolute_time();
// 	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);
// }

// void update_state_machine_subsystem_notpresent(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type)
// {
// 	current_status->onboard_control_sensors_present &= ~(1 << *subsystem_type);
// 	current_status->counter++;
// 	current_status->timestamp = hrt_absolute_time();
// 	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);

// 	/* if a subsystem was removed something went completely wrong */

// 	switch (*subsystem_type) {
// 	case SUBSYSTEM_TYPE_GYRO:
// 		//global_data_send_mavlink_statustext_message_out("Commander: gyro not present", MAV_SEVERITY_EMERGENCY);
// 		state_machine_emergency_always_critical(status_pub, current_status);
// 		break;

// 	case SUBSYSTEM_TYPE_ACC:
// 		//global_data_send_mavlink_statustext_message_out("Commander: accelerometer not present", MAV_SEVERITY_EMERGENCY);
// 		state_machine_emergency_always_critical(status_pub, current_status);
// 		break;

// 	case SUBSYSTEM_TYPE_MAG:
// 		//global_data_send_mavlink_statustext_message_out("Commander: magnetometer not present", MAV_SEVERITY_EMERGENCY);
// 		state_machine_emergency_always_critical(status_pub, current_status);
// 		break;

// 	case SUBSYSTEM_TYPE_GPS:
// 		{
// 			uint8_t flight_env = global_data_parameter_storage->pm.param_values[PARAM_FLIGHT_ENV];

// 			if (flight_env == PX4_FLIGHT_ENVIRONMENT_OUTDOOR) {
// 				//global_data_send_mavlink_statustext_message_out("Commander: GPS not present", MAV_SEVERITY_EMERGENCY);
// 				state_machine_emergency(status_pub, current_status);
// 			}
// 		}
// 		break;

// 	default:
// 		break;
// 	}

// }

// void update_state_machine_subsystem_enabled(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type)
// {
// 	current_status->onboard_control_sensors_enabled |= 1 << *subsystem_type;
// 	current_status->counter++;
// 	current_status->timestamp = hrt_absolute_time();
// 	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);
// }

// void update_state_machine_subsystem_disabled(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type)
// {
// 	current_status->onboard_control_sensors_enabled &= ~(1 << *subsystem_type);
// 	current_status->counter++;
// 	current_status->timestamp = hrt_absolute_time();
// 	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);

// 	/* if a subsystem was disabled something went completely wrong */

// 	switch (*subsystem_type) {
// 	case SUBSYSTEM_TYPE_GYRO:
// 		//global_data_send_mavlink_statustext_message_out("Commander: EMERGENCY - gyro disabled", MAV_SEVERITY_EMERGENCY);
// 		state_machine_emergency_always_critical(status_pub, current_status);
// 		break;

// 	case SUBSYSTEM_TYPE_ACC:
// 		//global_data_send_mavlink_statustext_message_out("Commander: EMERGENCY - accelerometer disabled", MAV_SEVERITY_EMERGENCY);
// 		state_machine_emergency_always_critical(status_pub, current_status);
// 		break;

// 	case SUBSYSTEM_TYPE_MAG:
// 		//global_data_send_mavlink_statustext_message_out("Commander: EMERGENCY - magnetometer disabled", MAV_SEVERITY_EMERGENCY);
// 		state_machine_emergency_always_critical(status_pub, current_status);
// 		break;

// 	case SUBSYSTEM_TYPE_GPS:
// 		{
// 			uint8_t flight_env = (uint8_t)(global_data_parameter_storage->pm.param_values[PARAM_FLIGHT_ENV]);

// 			if (flight_env == PX4_FLIGHT_ENVIRONMENT_OUTDOOR) {
// 				//global_data_send_mavlink_statustext_message_out("Commander: EMERGENCY - GPS disabled", MAV_SEVERITY_EMERGENCY);
// 				state_machine_emergency(status_pub, current_status);
// 			}
// 		}
// 		break;

// 	default:
// 		break;
// 	}

// }


///* END SUBSYSTEM/EMERGENCY FUNCTIONS*/
//
//int update_state_machine_mode_request(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, uint8_t mode)
//{
//	int ret = 1;
//
////	/* Switch on HIL if in standby and not already in HIL mode */
////	if ((mode & VEHICLE_MODE_FLAG_HIL_ENABLED)
////	    && !current_status->flag_hil_enabled) {
////		if ((current_status->state_machine == SYSTEM_STATE_STANDBY)) {
////			/* Enable HIL on request */
////			current_status->flag_hil_enabled = true;
////			ret = OK;
////			state_machine_publish(status_pub, current_status, mavlink_fd);
////			publish_armed_status(current_status);
////			printf("[cmd] Enabling HIL, locking down all actuators for safety.\n\t(Arming the system will not activate them while in HIL mode)\n");
////
////		} else if (current_status->state_machine != SYSTEM_STATE_STANDBY &&
////			   current_status->flag_fmu_armed) {
////
////			mavlink_log_critical(mavlink_fd, "REJECTING HIL, disarm first!")
////
////		} else {
////
////			mavlink_log_critical(mavlink_fd, "REJECTING HIL, not in standby.")
////		}
////	}
//
//	/* switch manual / auto */
//	if (mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
//		update_state_machine_mode_auto(status_pub, current_status, mavlink_fd);
//
//	} else if (mode & VEHICLE_MODE_FLAG_STABILIZED_ENABLED) {
//		update_state_machine_mode_stabilized(status_pub, current_status, mavlink_fd);
//
//	} else if (mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
//		update_state_machine_mode_guided(status_pub, current_status, mavlink_fd);
//
//	} else if (mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
//		update_state_machine_mode_manual(status_pub, current_status, mavlink_fd);
//	}
//
//	/* vehicle is disarmed, mode requests arming */
//	if (!(current_status->flag_fmu_armed) && (mode & VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
//		/* only arm in standby state */
//		// XXX REMOVE
//		if (current_status->state_machine == SYSTEM_STATE_STANDBY || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
//			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_GROUND_READY);
//			ret = OK;
//			printf("[cmd] arming due to command request\n");
//		}
//	}
//
//	/* vehicle is armed, mode requests disarming */
//	if (current_status->flag_fmu_armed && !(mode & VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
//		/* only disarm in ground ready */
//		if (current_status->state_machine == SYSTEM_STATE_GROUND_READY) {
//			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);
//			ret = OK;
//			printf("[cmd] disarming due to command request\n");
//		}
//	}
//
//	/* NEVER actually switch off HIL without reboot */
//	if (current_status->flag_hil_enabled && !(mode & VEHICLE_MODE_FLAG_HIL_ENABLED)) {
//		warnx("DENYING request to switch off HIL. Please power cycle (safety reasons)\n");
//		mavlink_log_critical(mavlink_fd, "Power-cycle to exit HIL");
//		ret = ERROR;
//	}
//
//	return ret;
//}


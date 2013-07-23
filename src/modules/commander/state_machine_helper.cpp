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

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <mavlink/mavlink_log.h>

#include "state_machine_helper.h"
#include "commander_helper.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

int arming_state_transition(int status_pub, struct vehicle_status_s *current_state, arming_state_t new_arming_state, int armed_pub, struct actuator_armed_s *armed, const int mavlink_fd) {


	int ret = ERROR;

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == current_state->arming_state) {
		ret = OK;
	} else {

		switch (new_arming_state) {
			case ARMING_STATE_INIT:

				/* allow going back from INIT for calibration */
				if (current_state->arming_state == ARMING_STATE_STANDBY) {
					ret = OK;
					armed->armed = false;
					armed->ready_to_arm = false;
				}
				break;
			case ARMING_STATE_STANDBY:

				/* allow coming from INIT and disarming from ARMED */
				if (current_state->arming_state == ARMING_STATE_INIT
				 || current_state->arming_state == ARMING_STATE_ARMED) {

					/* sensors need to be initialized for STANDBY state */
					if (current_state->condition_system_sensors_initialized) {
						ret = OK;
						armed->armed = false;
						armed->ready_to_arm = true;
					} else {
						mavlink_log_critical(mavlink_fd, "Rej. STANDBY state, sensors not initialized");
					}
				}
				break;
			case ARMING_STATE_ARMED:

				/* allow arming from STANDBY and IN-AIR-RESTORE */
				if (current_state->arming_state == ARMING_STATE_STANDBY
				 || current_state->arming_state == ARMING_STATE_IN_AIR_RESTORE) {

					/* XXX conditions for arming? */
					ret = OK;
					armed->armed = true;
				}
				break;
			case ARMING_STATE_ARMED_ERROR:

				/* an armed error happens when ARMED obviously */
				if (current_state->arming_state == ARMING_STATE_ARMED) {
					
					/* XXX conditions for an error state? */
					ret = OK;
					armed->armed = true;
				}
				break;
			case ARMING_STATE_STANDBY_ERROR:
				/* a disarmed error happens when in STANDBY or in INIT or after ARMED_ERROR */
				if (current_state->arming_state == ARMING_STATE_STANDBY
				 || current_state->arming_state == ARMING_STATE_INIT
				 || current_state->arming_state == ARMING_STATE_ARMED_ERROR) {
					ret = OK;
					armed->armed = false;
					armed->ready_to_arm = false;
				}
				break;
			case ARMING_STATE_REBOOT:

				/* an armed error happens when ARMED obviously */
				if (current_state->arming_state == ARMING_STATE_INIT
				 || current_state->arming_state == ARMING_STATE_STANDBY
				 || current_state->arming_state == ARMING_STATE_STANDBY_ERROR) {

					ret = OK;
					armed->armed = false;
					armed->ready_to_arm = false;

				}
				break;
			case ARMING_STATE_IN_AIR_RESTORE:

				/* XXX implement */
				break;
			default:
				break;
		}

		if (ret == OK) {
			current_state->arming_state = new_arming_state;
			current_state->counter++;
			current_state->timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(vehicle_status), status_pub, current_state);

			armed->timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(actuator_armed), armed_pub, armed);
		}
	}

	return ret;
}



/*
 * This functions does not evaluate any input flags but only checks if the transitions
 * are valid.
 */
int navigation_state_transition(int status_pub, struct vehicle_status_s *current_state, navigation_state_t new_navigation_state, int control_mode_pub, struct vehicle_control_mode_s *control_mode, const int mavlink_fd) {

	int ret = ERROR;

	/* only check transition if the new state is actually different from the current one */
	if (new_navigation_state == current_state->navigation_state) {
		ret = OK;
	} else {

		switch (new_navigation_state) {
			case NAVIGATION_STATE_INIT:

				/* transitions back to INIT are possible for calibration */
				if (current_state->navigation_state == NAVIGATION_STATE_MANUAL_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_STANDBY) {

					ret = OK;
					control_mode->flag_control_rates_enabled = false;
					control_mode->flag_control_attitude_enabled = false;
					control_mode->flag_control_velocity_enabled = false;
					control_mode->flag_control_position_enabled = false;
					control_mode->flag_control_altitude_enabled = false;
					control_mode->flag_control_manual_enabled = false;
				}
				break;

			case NAVIGATION_STATE_MANUAL_STANDBY:

				/* transitions from INIT and other STANDBY states as well as MANUAL are possible */
				if (current_state->navigation_state == NAVIGATION_STATE_INIT
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL) {

					/* need to be disarmed first */
					if (current_state->arming_state != ARMING_STATE_STANDBY) {
						mavlink_log_critical(mavlink_fd, "Rej. MANUAL_STANDBY: not disarmed");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = false;
						control_mode->flag_control_position_enabled = false;
						control_mode->flag_control_altitude_enabled = false;
						control_mode->flag_control_manual_enabled = true;
					}
				}
				break;

			case NAVIGATION_STATE_MANUAL:

				/* need to be armed first */
				if (current_state->arming_state != ARMING_STATE_ARMED) {
					mavlink_log_critical(mavlink_fd, "Rej. MANUAL: not armed");
					tune_negative();
				} else {
					ret = OK;
					control_mode->flag_control_rates_enabled = true;
					control_mode->flag_control_attitude_enabled = true;
					control_mode->flag_control_velocity_enabled = false;
					control_mode->flag_control_position_enabled = false;
					control_mode->flag_control_altitude_enabled = false;
					control_mode->flag_control_manual_enabled = true;
				}
				break;

			case NAVIGATION_STATE_ASSISTED_STANDBY:

				/* transitions from INIT and other STANDBY states as well as SEATBELT and SEATBELT_DESCENT are possible */
				if (current_state->navigation_state == NAVIGATION_STATE_INIT
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SEATBELT
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SIMPLE
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_DESCENT) {

					/* need to be disarmed and have a position estimate */
					if (current_state->arming_state != ARMING_STATE_STANDBY) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT_STANDBY: not disarmed");
						tune_negative();
					} else if (!current_state->condition_local_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT_STANDBY: no position estimate");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = false;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = true;
					}
				}
				break;

			case NAVIGATION_STATE_ASSISTED_SEATBELT:

				/* transitions from all AUTO modes except AUTO_STANDBY and except MANUAL_STANDBY and INIT*/
				if (current_state->navigation_state == NAVIGATION_STATE_ASSISTED_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SIMPLE
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_DESCENT
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LAND
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LOITER
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_MISSION
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_READY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_RTL
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_TAKEOFF) {

					/* need to be armed and have a position estimate */
					if (current_state->arming_state != ARMING_STATE_ARMED) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT: not armed");
						tune_negative();
					} else if (!current_state->condition_local_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT: no pos estimate");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = false;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = true;
					}
				}
				break;

			case NAVIGATION_STATE_ASSISTED_SIMPLE:

				/* transitions from all AUTO modes except AUTO_STANDBY and except MANUAL_STANDBY and INIT*/
				if (current_state->navigation_state == NAVIGATION_STATE_ASSISTED_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SEATBELT
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_DESCENT
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LAND
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LOITER
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_MISSION
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_READY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_RTL
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_TAKEOFF) {

					/* need to be armed and have a position estimate */
					if (current_state->arming_state != ARMING_STATE_ARMED) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT: not armed");
						tune_negative();
					} else if (!current_state->condition_local_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT: no pos estimate");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = true;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = true;
					}
				}
				break;

			case NAVIGATION_STATE_ASSISTED_DESCENT:

				/* transitions from all AUTO modes except AUTO_STANDBY and except MANUAL_STANDBY and INIT and SEATBELT_STANDBY */
				if (current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SEATBELT
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SIMPLE
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LAND
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LOITER
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_MISSION
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_READY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_RTL
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_TAKEOFF) {

					/* need to be armed and have a position estimate */
					if (current_state->arming_state != ARMING_STATE_ARMED) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT_DESCENT: not armed");
						tune_negative();
					} else if (!current_state->condition_local_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. SEATBELT_DESCENT: no pos estimate");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = false;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = true;
					}
				}
				break;

			case NAVIGATION_STATE_AUTO_STANDBY:

				/* transitions from INIT or from other STANDBY modes or from AUTO READY */
				if (current_state->navigation_state == NAVIGATION_STATE_INIT
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_READY) {

					/* need to be disarmed and have a position and home lock */
					if (current_state->arming_state != ARMING_STATE_STANDBY) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_STANDBY: not disarmed");
						tune_negative();
					} else if (!current_state->condition_global_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_STANDBY: no pos lock");
						tune_negative();
					} else if (!current_state->condition_home_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_STANDBY: no home pos");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = true;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = false;
					}
				}
				break;

			case NAVIGATION_STATE_AUTO_READY:

				/* transitions from AUTO_STANDBY or AUTO_LAND */
				if (current_state->navigation_state == NAVIGATION_STATE_AUTO_STANDBY
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LAND) {

					// XXX flag test needed?

					/* need to be armed and have a position and home lock */
					if (current_state->arming_state != ARMING_STATE_ARMED) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_READY: not armed");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = true;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = false;
					}
				}
				break;

			case NAVIGATION_STATE_AUTO_TAKEOFF:

				/* only transitions from AUTO_READY */
				if (current_state->navigation_state == NAVIGATION_STATE_AUTO_READY) {

					ret = OK;
					control_mode->flag_control_rates_enabled = true;
					control_mode->flag_control_attitude_enabled = true;
					control_mode->flag_control_velocity_enabled = true;
					control_mode->flag_control_position_enabled = true;
					control_mode->flag_control_altitude_enabled = true;
					control_mode->flag_control_manual_enabled = false;
				}
				break;

			case NAVIGATION_STATE_AUTO_LOITER:

				/* from everywhere flying except AUTO_LAND and SEATBELT_DESCENT */
				if (current_state->navigation_state == NAVIGATION_STATE_AUTO_TAKEOFF
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_MISSION
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_RTL
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SEATBELT
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SIMPLE
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL) {

					/* need to have a position and home lock */
					if (!current_state->condition_global_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_LOITER: no pos lock");
						tune_negative();
					} else if (!current_state->condition_home_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_LOITER: no home pos");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = true;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = false;
					}
				}
				break;

			case NAVIGATION_STATE_AUTO_MISSION:

				/* from everywhere flying except AUTO_LAND and SEATBELT_DESCENT */
				if (current_state->navigation_state == NAVIGATION_STATE_AUTO_TAKEOFF
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LOITER
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_RTL
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SEATBELT
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SIMPLE
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL) {

					/* need to have a mission ready */
					if (!current_state-> condition_auto_mission_available) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_MISSION: no mission available");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = true;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = false;
					}
				}
				break;

			case NAVIGATION_STATE_AUTO_RTL:

				/* from everywhere flying except AUTO_LAND and SEATBELT_DESCENT */
				if (current_state->navigation_state == NAVIGATION_STATE_AUTO_TAKEOFF
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_MISSION
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LOITER
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SEATBELT
				 || current_state->navigation_state == NAVIGATION_STATE_ASSISTED_SIMPLE
				 || current_state->navigation_state == NAVIGATION_STATE_MANUAL) {

					/* need to have a position and home lock */
					if (!current_state->condition_global_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_RTL: no pos lock");
						tune_negative();
					} else if (!current_state->condition_home_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_RTL: no home pos");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = true;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = false;
					}
				}
				break;

			case NAVIGATION_STATE_AUTO_LAND:
				/* after AUTO_RTL or when in AUTO_LOITER or AUTO_MISSION */
				if (current_state->navigation_state == NAVIGATION_STATE_AUTO_RTL
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_MISSION
				 || current_state->navigation_state == NAVIGATION_STATE_AUTO_LOITER) {

					/* need to have a position and home lock */
					if (!current_state->condition_global_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_LAND: no pos lock");
						tune_negative();
					} else if (!current_state->condition_home_position_valid) {
						mavlink_log_critical(mavlink_fd, "Rej. AUTO_LAND: no home pos");
						tune_negative();
					} else {
						ret = OK;
						control_mode->flag_control_rates_enabled = true;
						control_mode->flag_control_attitude_enabled = true;
						control_mode->flag_control_velocity_enabled = true;
						control_mode->flag_control_position_enabled = true;
						control_mode->flag_control_altitude_enabled = true;
						control_mode->flag_control_manual_enabled = false;
					}
				}
				break;

			default:
				break;
		}

		if (ret == OK) {
			current_state->navigation_state = new_navigation_state;
			current_state->counter++;
			current_state->timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(vehicle_status), status_pub, current_state);

			control_mode->timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, control_mode);
		}
	}

	

	return ret;
}


/**
* Transition from one hil state to another
*/
int hil_state_transition(hil_state_t new_state, int status_pub, struct vehicle_status_s *current_status, int control_mode_pub, struct vehicle_control_mode_s *current_control_mode, const int mavlink_fd)
{
	bool valid_transition = false;
	int ret = ERROR;

	warnx("Current state: %d, requested state: %d", current_status->hil_state, new_state);

	if (current_status->hil_state == new_state) {
		warnx("Hil state not changed");
		valid_transition = true;

	} else {

		switch (new_state) {

			case HIL_STATE_OFF:

				if (current_status->arming_state == ARMING_STATE_INIT
				 || current_status->arming_state == ARMING_STATE_STANDBY) {

					current_control_mode->flag_system_hil_enabled = false;
					mavlink_log_critical(mavlink_fd, "Switched to OFF hil state");
					valid_transition = true;
				}
				break;

			case HIL_STATE_ON:

				if (current_status->arming_state == ARMING_STATE_INIT
				 || current_status->arming_state == ARMING_STATE_STANDBY) {

					current_control_mode->flag_system_hil_enabled = true;
					mavlink_log_critical(mavlink_fd, "Switched to ON hil state");
					valid_transition = true;
				}
				break;

			default:
				warnx("Unknown hil state");
				break;
		}
	}

	if (valid_transition) {
		current_status->hil_state = new_state;

		current_status->counter++;
		current_status->timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(vehicle_status), status_pub, current_status);

		current_control_mode->timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, current_control_mode);

		ret = OK;
	} else {
		mavlink_log_critical(mavlink_fd, "REJECTING invalid hil state transition");
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


/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file state_machine_helper.c
 * State machine helper functions implementations
 */

#include <stdio.h>
#include <unistd.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#include <mavlink/mavlink_log.h>

#include "state_machine_helper.h"

static const char *system_state_txt[] = {
	"SYSTEM_STATE_PREFLIGHT",
	"SYSTEM_STATE_STANDBY",
	"SYSTEM_STATE_GROUND_READY",
	"SYSTEM_STATE_MANUAL",
	"SYSTEM_STATE_STABILIZED",
	"SYSTEM_STATE_AUTO",
	"SYSTEM_STATE_MISSION_ABORT",
	"SYSTEM_STATE_EMCY_LANDING",
	"SYSTEM_STATE_EMCY_CUTOFF",
	"SYSTEM_STATE_GROUND_ERROR",
	"SYSTEM_STATE_REBOOT",

};

/**
 * Transition from one state to another
 */
int do_state_update(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, commander_state_machine_t new_state)
{
	int invalid_state = false;
	int ret = ERROR;

	commander_state_machine_t old_state = current_status->state_machine;

	switch (new_state) {
	case SYSTEM_STATE_MISSION_ABORT: {
			/* Indoor or outdoor */
			// if (flight_environment_parameter == PX4_FLIGHT_ENVIRONMENT_OUTDOOR) {
			ret = do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_EMCY_LANDING);

			// } else {
			// 	ret = do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_EMCY_CUTOFF);
			// }
		}
		break;

	case SYSTEM_STATE_EMCY_LANDING:
		/* Tell the controller to land */

		/* set system flags according to state */
		current_status->flag_system_armed = true;

		warnx("EMERGENCY LANDING!\n");
		mavlink_log_critical(mavlink_fd, "EMERGENCY LANDING!");
		break;

	case SYSTEM_STATE_EMCY_CUTOFF:
		/* Tell the controller to cutoff the motors (thrust = 0) */

		/* set system flags according to state */
		current_status->flag_system_armed = false;

		warnx("EMERGENCY MOTOR CUTOFF!\n");
		mavlink_log_critical(mavlink_fd, "EMERGENCY MOTOR CUTOFF!");
		break;

	case SYSTEM_STATE_GROUND_ERROR:

		/* set system flags according to state */

		/* prevent actuators from arming */
		current_status->flag_system_armed = false;

		warnx("GROUND ERROR, locking down propulsion system\n");
		mavlink_log_critical(mavlink_fd, "GROUND ERROR, locking down system");
		break;

	case SYSTEM_STATE_PREFLIGHT:
		if (current_status->state_machine == SYSTEM_STATE_STANDBY
		    || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
			/* set system flags according to state */
			current_status->flag_system_armed = false;
			mavlink_log_critical(mavlink_fd, "Switched to PREFLIGHT state");

		} else {
			invalid_state = true;
			mavlink_log_critical(mavlink_fd, "REFUSED to switch to PREFLIGHT state");
		}

		break;

	case SYSTEM_STATE_REBOOT:
		if (current_status->state_machine == SYSTEM_STATE_STANDBY
			|| current_status->state_machine == SYSTEM_STATE_PREFLIGHT
			|| current_status->flag_hil_enabled) {
			invalid_state = false;
			/* set system flags according to state */
			current_status->flag_system_armed = false;
			mavlink_log_critical(mavlink_fd, "REBOOTING SYSTEM");
			usleep(500000);
			up_systemreset();
			/* SPECIAL CASE: NEVER RETURNS FROM THIS FUNCTION CALL */

		} else {
			invalid_state = true;
			mavlink_log_critical(mavlink_fd, "REFUSED to REBOOT");
		}

		break;

	case SYSTEM_STATE_STANDBY:
		/* set system flags according to state */

		/* standby enforces disarmed */
		current_status->flag_system_armed = false;

		mavlink_log_critical(mavlink_fd, "Switched to STANDBY state");
		break;

	case SYSTEM_STATE_GROUND_READY:

		/* set system flags according to state */

		/* ground ready has motors / actuators armed */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "Switched to GROUND READY state");
		break;

	case SYSTEM_STATE_AUTO:

		/* set system flags according to state */

		/* auto is airborne and in auto mode, motors armed */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "Switched to FLYING / AUTO mode");
		break;

	case SYSTEM_STATE_STABILIZED:

		/* set system flags according to state */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "Switched to FLYING / STABILIZED mode");
		break;

	case SYSTEM_STATE_MANUAL:

		/* set system flags according to state */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "Switched to FLYING / MANUAL mode");
		break;

	default:
		invalid_state = true;
		break;
	}

	if (invalid_state == false || old_state != new_state) {
		current_status->state_machine = new_state;
		state_machine_publish(status_pub, current_status, mavlink_fd);
		publish_armed_status(current_status);
		ret = OK;
	}

	if (invalid_state) {
		mavlink_log_critical(mavlink_fd, "REJECTING invalid state transition");
		ret = ERROR;
	}

	return ret;
}

void state_machine_publish(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	/* publish the new state */
	current_status->counter++;
	current_status->timestamp = hrt_absolute_time();

	/* assemble state vector based on flag values */
	if (current_status->flag_control_rates_enabled) {
		current_status->onboard_control_sensors_present |= 0x400;

	} else {
		current_status->onboard_control_sensors_present &= ~0x400;
	}

	current_status->onboard_control_sensors_present |= (current_status->flag_control_attitude_enabled) ? 0x800 : 0;
	current_status->onboard_control_sensors_present |= (current_status->flag_control_attitude_enabled) ? 0x1000 : 0;
	current_status->onboard_control_sensors_present |= (current_status->flag_control_velocity_enabled || current_status->flag_control_position_enabled) ? 0x2000 : 0;
	current_status->onboard_control_sensors_present |= (current_status->flag_control_velocity_enabled || current_status->flag_control_position_enabled) ? 0x4000 : 0;

	current_status->onboard_control_sensors_enabled |= (current_status->flag_control_rates_enabled) ? 0x400 : 0;
	current_status->onboard_control_sensors_enabled |= (current_status->flag_control_attitude_enabled) ? 0x800 : 0;
	current_status->onboard_control_sensors_enabled |= (current_status->flag_control_attitude_enabled) ? 0x1000 : 0;
	current_status->onboard_control_sensors_enabled |= (current_status->flag_control_velocity_enabled || current_status->flag_control_position_enabled) ? 0x2000 : 0;
	current_status->onboard_control_sensors_enabled |= (current_status->flag_control_velocity_enabled || current_status->flag_control_position_enabled) ? 0x4000 : 0;

	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);
	printf("[cmd] new state: %s\n", system_state_txt[current_status->state_machine]);
}

void publish_armed_status(const struct vehicle_status_s *current_status)
{
	struct actuator_armed_s armed;
	armed.armed = current_status->flag_system_armed;

	/* XXX allow arming by external components on multicopters only if not yet armed by RC */
	/* XXX allow arming only if core sensors are ok */
	armed.ready_to_arm = true;

	/* lock down actuators if required, only in HIL */
	armed.lockdown = (current_status->flag_hil_enabled) ? true : false;
	orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);
	orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
}


/*
 * Private functions, update the state machine
 */
void state_machine_emergency_always_critical(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	warnx("EMERGENCY HANDLER\n");
	/* Depending on the current state go to one of the error states */

	if (current_status->state_machine == SYSTEM_STATE_PREFLIGHT || current_status->state_machine == SYSTEM_STATE_STANDBY || current_status->state_machine == SYSTEM_STATE_GROUND_READY) {
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_GROUND_ERROR);

	} else if (current_status->state_machine == SYSTEM_STATE_AUTO || current_status->state_machine == SYSTEM_STATE_MANUAL) {

		// DO NOT abort mission
		//do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_MISSION_ABORT);

	} else {
		warnx("Unknown system state: #%d\n", current_status->state_machine);
	}
}

void state_machine_emergency(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd) //do not call state_machine_emergency_always_critical if we are in manual mode for these errors
{
	if (current_status->state_machine != SYSTEM_STATE_MANUAL) { //if we are in manual: user can react to errors themself
		state_machine_emergency_always_critical(status_pub, current_status, mavlink_fd);

	} else {
		//global_data_send_mavlink_statustext_message_out("[cmd] ERROR: take action immediately! (did not switch to error state because the system is in manual mode)", MAV_SEVERITY_CRITICAL);
	}

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


// void update_state_machine_subsystem_healthy(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type)
// {
// 	current_status->onboard_control_sensors_health |= 1 << *subsystem_type;
// 	current_status->counter++;
// 	current_status->timestamp = hrt_absolute_time();
// 	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);

// 	switch (*subsystem_type) {
// 	case SUBSYSTEM_TYPE_GYRO:
// 		//TODO state machine change (recovering)
// 		break;

// 	case SUBSYSTEM_TYPE_ACC:
// 		//TODO state machine change
// 		break;

// 	case SUBSYSTEM_TYPE_MAG:
// 		//TODO state machine change
// 		break;

// 	case SUBSYSTEM_TYPE_GPS:
// 		//TODO state machine change
// 		break;

// 	default:
// 		break;
// 	}


// }


// void update_state_machine_subsystem_unhealthy(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type)
// {
// 	bool previosly_healthy = (bool)(current_status->onboard_control_sensors_health & 1 << *subsystem_type);
// 	current_status->onboard_control_sensors_health &= ~(1 << *subsystem_type);
// 	current_status->counter++;
// 	current_status->timestamp = hrt_absolute_time();
// 	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);

// 	/* if we received unhealthy message more than *_HEALTH_COUNTER_LIMIT, switch to error state */

// 	switch (*subsystem_type) {
// 	case SUBSYSTEM_TYPE_GYRO:
// 		//global_data_send_mavlink_statustext_message_out("Commander: gyro unhealthy", MAV_SEVERITY_CRITICAL);

// 		if (previosly_healthy) 	//only throw emergency if previously healthy
// 			state_machine_emergency_always_critical(status_pub, current_status);

// 		break;

// 	case SUBSYSTEM_TYPE_ACC:
// 		//global_data_send_mavlink_statustext_message_out("Commander: accelerometer unhealthy", MAV_SEVERITY_CRITICAL);

// 		if (previosly_healthy) 	//only throw emergency if previously healthy
// 			state_machine_emergency_always_critical(status_pub, current_status);

// 		break;

// 	case SUBSYSTEM_TYPE_MAG:
// 		//global_data_send_mavlink_statustext_message_out("Commander: magnetometer unhealthy", MAV_SEVERITY_CRITICAL);

// 		if (previosly_healthy) 	//only throw emergency if previously healthy
// 			state_machine_emergency_always_critical(status_pub, current_status);

// 		break;

// 	case SUBSYSTEM_TYPE_GPS:
// //			//TODO: remove this block
// //			break;
// //			///////////////////
// 		//global_data_send_mavlink_statustext_message_out("Commander: GPS unhealthy", MAV_SEVERITY_CRITICAL);

// //				printf("previosly_healthy = %u\n", previosly_healthy);
// 		if (previosly_healthy) 	//only throw emergency if previously healthy
// 			state_machine_emergency(status_pub, current_status);

// 		break;

// 	default:
// 		break;
// 	}

// }


/* END SUBSYSTEM/EMERGENCY FUNCTIONS*/


void update_state_machine_got_position_fix(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	/* Depending on the current state switch state */
	if (current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);
	}
}

void update_state_machine_no_position_fix(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	/* Depending on the current state switch state */
	if (current_status->state_machine == SYSTEM_STATE_STANDBY || current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_AUTO) {
		state_machine_emergency(status_pub, current_status, mavlink_fd);
	}
}

void update_state_machine_arm(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	if (current_status->state_machine == SYSTEM_STATE_STANDBY) {
		printf("[cmd] arming\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_GROUND_READY);
	}
}

void update_state_machine_disarm(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_MANUAL || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
		printf("[cmd] going standby\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);

	} else if (current_status->state_machine == SYSTEM_STATE_STABILIZED || current_status->state_machine == SYSTEM_STATE_AUTO) {
		printf("[cmd] MISSION ABORT!\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);
	}
}

void update_state_machine_mode_manual(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	int old_mode = current_status->flight_mode;
	current_status->flight_mode = VEHICLE_FLIGHT_MODE_MANUAL;

	current_status->flag_control_manual_enabled = true;

	/* set behaviour based on airframe */
	if ((current_status->system_type == VEHICLE_TYPE_QUADROTOR) ||
	    (current_status->system_type == VEHICLE_TYPE_HEXAROTOR) ||
	    (current_status->system_type == VEHICLE_TYPE_OCTOROTOR)) {

		/* assuming a rotary wing, set to SAS */
		current_status->manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_SAS;
		current_status->flag_control_attitude_enabled = true;
		current_status->flag_control_rates_enabled = true;

	} else {

		/* assuming a fixed wing, set to direct pass-through */
		current_status->manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_DIRECT;
		current_status->flag_control_attitude_enabled = false;
		current_status->flag_control_rates_enabled = false;
	}

	if (old_mode != current_status->flight_mode) state_machine_publish(status_pub, current_status, mavlink_fd);

	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_STABILIZED || current_status->state_machine == SYSTEM_STATE_AUTO) {
		printf("[cmd] manual mode\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_MANUAL);
	}
}

void update_state_machine_mode_stabilized(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_STABILIZED || current_status->state_machine == SYSTEM_STATE_MANUAL || current_status->state_machine == SYSTEM_STATE_AUTO) {
		int old_mode = current_status->flight_mode;
		int old_manual_control_mode = current_status->manual_control_mode;
		current_status->flight_mode = VEHICLE_FLIGHT_MODE_MANUAL;
		current_status->manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_SAS;
		current_status->flag_control_attitude_enabled = true;
		current_status->flag_control_rates_enabled = true;
		current_status->flag_control_manual_enabled = true;

		if (old_mode != current_status->flight_mode ||
		    old_manual_control_mode != current_status->manual_control_mode) {
			printf("[cmd] att stabilized mode\n");
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_MANUAL);
			state_machine_publish(status_pub, current_status, mavlink_fd);
		}

	}
}

void update_state_machine_mode_guided(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	if (!current_status->flag_vector_flight_mode_ok) {
		mavlink_log_critical(mavlink_fd, "NO POS LOCK, REJ. GUIDED MODE");
		tune_error();
		return;
	}

	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_MANUAL || current_status->state_machine == SYSTEM_STATE_AUTO) {
		printf("[cmd] position guided mode\n");
		int old_mode = current_status->flight_mode;
		current_status->flight_mode = VEHICLE_FLIGHT_MODE_STAB;
		current_status->flag_control_manual_enabled = false;
		current_status->flag_control_attitude_enabled = true;
		current_status->flag_control_rates_enabled = true;
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STABILIZED);

		if (old_mode != current_status->flight_mode) state_machine_publish(status_pub, current_status, mavlink_fd);

	}
}

void update_state_machine_mode_auto(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	if (!current_status->flag_vector_flight_mode_ok) {
		mavlink_log_critical(mavlink_fd, "NO POS LOCK, REJ. AUTO MODE");
		return;
	}

	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_MANUAL || current_status->state_machine == SYSTEM_STATE_STABILIZED) {
		printf("[cmd] auto mode\n");
		int old_mode = current_status->flight_mode;
		current_status->flight_mode = VEHICLE_FLIGHT_MODE_AUTO;
		current_status->flag_control_manual_enabled = false;
		current_status->flag_control_attitude_enabled = true;
		current_status->flag_control_rates_enabled = true;
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_AUTO);

		if (old_mode != current_status->flight_mode) state_machine_publish(status_pub, current_status, mavlink_fd);
	}
}


uint8_t update_state_machine_mode_request(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, uint8_t mode)
{
	uint8_t ret = 1;

	/* Switch on HIL if in standby and not already in HIL mode */
	if ((mode & VEHICLE_MODE_FLAG_HIL_ENABLED)
	    && !current_status->flag_hil_enabled) {
		if ((current_status->state_machine == SYSTEM_STATE_STANDBY)) {
			/* Enable HIL on request */
			current_status->flag_hil_enabled = true;
			ret = OK;
			state_machine_publish(status_pub, current_status, mavlink_fd);
			publish_armed_status(current_status);
			printf("[cmd] Enabling HIL, locking down all actuators for safety.\n\t(Arming the system will not activate them while in HIL mode)\n");

		} else if (current_status->state_machine != SYSTEM_STATE_STANDBY &&
			   current_status->flag_system_armed) {

			mavlink_log_critical(mavlink_fd, "REJECTING HIL, disarm first!")

		} else {

			mavlink_log_critical(mavlink_fd, "REJECTING HIL, not in standby.")
		}
	}

	/* switch manual / auto */
	if (mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
		update_state_machine_mode_auto(status_pub, current_status, mavlink_fd);

	} else if (mode & VEHICLE_MODE_FLAG_STABILIZED_ENABLED) {
		update_state_machine_mode_stabilized(status_pub, current_status, mavlink_fd);

	} else if (mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
		update_state_machine_mode_guided(status_pub, current_status, mavlink_fd);

	} else if (mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
		update_state_machine_mode_manual(status_pub, current_status, mavlink_fd);
	}

	/* vehicle is disarmed, mode requests arming */
	if (!(current_status->flag_system_armed) && (mode & VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
		/* only arm in standby state */
		// XXX REMOVE
		if (current_status->state_machine == SYSTEM_STATE_STANDBY || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_GROUND_READY);
			ret = OK;
			printf("[cmd] arming due to command request\n");
		}
	}

	/* vehicle is armed, mode requests disarming */
	if (current_status->flag_system_armed && !(mode & VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
		/* only disarm in ground ready */
		if (current_status->state_machine == SYSTEM_STATE_GROUND_READY) {
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);
			ret = OK;
			printf("[cmd] disarming due to command request\n");
		}
	}

	/* NEVER actually switch off HIL without reboot */
	if (current_status->flag_hil_enabled && !(mode & VEHICLE_MODE_FLAG_HIL_ENABLED)) {
		warnx("DENYING request to switch off HIL. Please power cycle (safety reasons)\n");
		mavlink_log_critical(mavlink_fd, "Power-cycle to exit HIL");
		ret = ERROR;
	}

	return ret;
}

uint8_t update_state_machine_custom_mode_request(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, uint8_t custom_mode) //TODO: add more checks to avoid state switching in critical situations
{
	commander_state_machine_t current_system_state = current_status->state_machine;

	uint8_t ret = 1;

	switch (custom_mode) {
	case SYSTEM_STATE_GROUND_READY:
		break;

	case SYSTEM_STATE_STANDBY:
		break;

	case SYSTEM_STATE_REBOOT:
		printf("try to reboot\n");

		if (current_system_state == SYSTEM_STATE_STANDBY
				|| current_system_state == SYSTEM_STATE_PREFLIGHT
				|| current_status->flag_hil_enabled) {
			printf("system will reboot\n");
			mavlink_log_critical(mavlink_fd, "Rebooting..");
			usleep(200000);
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_REBOOT);
			ret = 0;
		}

		break;

	case SYSTEM_STATE_AUTO:
		printf("try to switch to auto/takeoff\n");

		if (current_system_state == SYSTEM_STATE_GROUND_READY || current_system_state == SYSTEM_STATE_MANUAL) {
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_AUTO);
			printf("state: auto\n");
			ret = 0;
		}

		break;

	case SYSTEM_STATE_MANUAL:
		printf("try to switch to manual\n");

		if (current_system_state == SYSTEM_STATE_GROUND_READY || current_system_state == SYSTEM_STATE_AUTO) {
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_MANUAL);
			printf("state: manual\n");
			ret = 0;
		}

		break;

	default:
		break;
	}

	return ret;
}


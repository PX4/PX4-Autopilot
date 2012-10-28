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

static const char* system_state_txt[] = {
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

		fprintf(stderr, "[commander] EMERGENCY LANDING!\n");
		mavlink_log_critical(mavlink_fd, "[commander] EMERGENCY LANDING!");
		break;

	case SYSTEM_STATE_EMCY_CUTOFF:
		/* Tell the controller to cutoff the motors (thrust = 0) */

		/* set system flags according to state */
		current_status->flag_system_armed = false;

		fprintf(stderr, "[commander] EMERGENCY MOTOR CUTOFF!\n");
		mavlink_log_critical(mavlink_fd, "[commander] EMERGENCY MOTOR CUTOFF!");
		break;

	case SYSTEM_STATE_GROUND_ERROR:

		/* set system flags according to state */

		/* prevent actuators from arming */
		current_status->flag_system_armed = false;

		fprintf(stderr, "[commander] GROUND ERROR, locking down propulsion system\n");
		mavlink_log_critical(mavlink_fd, "[commander] GROUND ERROR, locking down propulsion system");
		break;

	case SYSTEM_STATE_PREFLIGHT:
		if (current_status->state_machine == SYSTEM_STATE_STANDBY
		 || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
			/* set system flags according to state */
			current_status->flag_system_armed = false;
			mavlink_log_critical(mavlink_fd, "[commander] Switched to PREFLIGHT state");
		} else {
			invalid_state = true;
			mavlink_log_critical(mavlink_fd, "[commander] REFUSED to switch to PREFLIGHT state");
		}
		break;

	case SYSTEM_STATE_REBOOT:
		if (current_status->state_machine == SYSTEM_STATE_STANDBY
		 || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
			invalid_state = false;
			/* set system flags according to state */
			current_status->flag_system_armed = false;
			mavlink_log_critical(mavlink_fd, "[commander] REBOOTING SYSTEM");
			usleep(500000);
			up_systemreset();
			/* SPECIAL CASE: NEVER RETURNS FROM THIS FUNCTION CALL */
		} else {
			invalid_state = true;
			mavlink_log_critical(mavlink_fd, "[commander] REFUSED to REBOOT");
		}
		break;

	case SYSTEM_STATE_STANDBY:
		/* set system flags according to state */

		/* standby enforces disarmed */
		current_status->flag_system_armed = false;

		mavlink_log_critical(mavlink_fd, "[commander] Switched to STANDBY state");
		break;

	case SYSTEM_STATE_GROUND_READY:

		/* set system flags according to state */

		/* ground ready has motors / actuators armed */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "[commander] Switched to GROUND READY state");
		break;

	case SYSTEM_STATE_AUTO:

		/* set system flags according to state */

		/* auto is airborne and in auto mode, motors armed */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "[commander] Switched to FLYING / AUTO mode");
		break;

	case SYSTEM_STATE_STABILIZED:

		/* set system flags according to state */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "[commander] Switched to FLYING / STABILIZED mode");
		break;

	case SYSTEM_STATE_MANUAL:

		/* set system flags according to state */
		current_status->flag_system_armed = true;

		mavlink_log_critical(mavlink_fd, "[commander] Switched to FLYING / MANUAL mode");
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
		mavlink_log_critical(mavlink_fd, "[commander] REJECTING invalid state transition");
		ret = ERROR;
	}
	return ret;
}

void state_machine_publish(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd) {
	/* publish the new state */
	current_status->counter++;
	current_status->timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(vehicle_status), status_pub, current_status);
	printf("[commander] new state: %s\n", system_state_txt[current_status->state_machine]);
}

void publish_armed_status(const struct vehicle_status_s *current_status) {
	struct actuator_armed_s armed;
	armed.armed = current_status->flag_system_armed;
	/* lock down actuators if required */
	// XXX FIXME Currently any loss of RC will completely disable all actuators
	// needs proper failsafe
	armed.lockdown = ((current_status->rc_signal_lost && current_status->offboard_control_signal_lost)
	 || current_status->flag_hil_enabled) ? true : false;
	orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);
	orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
}


/*
 * Private functions, update the state machine
 */
void state_machine_emergency_always_critical(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	fprintf(stderr, "[commander] EMERGENCY HANDLER\n");
	/* Depending on the current state go to one of the error states */

	if (current_status->state_machine == SYSTEM_STATE_PREFLIGHT || current_status->state_machine == SYSTEM_STATE_STANDBY || current_status->state_machine == SYSTEM_STATE_GROUND_READY) {
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_GROUND_ERROR);

	} else if (current_status->state_machine == SYSTEM_STATE_AUTO || current_status->state_machine == SYSTEM_STATE_MANUAL) {
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_MISSION_ABORT);

	} else {
		fprintf(stderr, "[commander] Unknown system state: #%d\n", current_status->state_machine);
	}
}

void state_machine_emergency(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd) //do not call state_machine_emergency_always_critical if we are in manual mode for these errors
{
	if (current_status->state_machine != SYSTEM_STATE_MANUAL) { //if we are in manual: user can react to errors themself
		state_machine_emergency_always_critical(status_pub, current_status, mavlink_fd);

	} else {
		//global_data_send_mavlink_statustext_message_out("[commander] ERROR: take action immediately! (did not switch to error state because the system is in manual mode)", MAV_SEVERITY_CRITICAL);
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
		printf("[commander] arming\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_GROUND_READY);
	}
}

void update_state_machine_disarm(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_MANUAL || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
		printf("[commander] going standby\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);

	} else if (current_status->state_machine == SYSTEM_STATE_STABILIZED || current_status->state_machine == SYSTEM_STATE_AUTO) {
		printf("[commander] MISSION ABORT!\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);
	}
}

void update_state_machine_mode_manual(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	int old_mode = current_status->flight_mode;
	current_status->flight_mode = VEHICLE_FLIGHT_MODE_MANUAL;
	current_status->flag_control_manual_enabled = true;
	/* enable attitude control per default */
	current_status->flag_control_attitude_enabled = true;
	current_status->flag_control_rates_enabled = true;
	if (old_mode != current_status->flight_mode) state_machine_publish(status_pub, current_status, mavlink_fd);

	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_STABILIZED || current_status->state_machine == SYSTEM_STATE_AUTO) {
		printf("[commander] manual mode\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_MANUAL);
	}
}

void update_state_machine_mode_stabilized(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	int old_mode = current_status->flight_mode;
	current_status->flight_mode = VEHICLE_FLIGHT_MODE_STABILIZED;
	current_status->flag_control_manual_enabled = true;
	current_status->flag_control_attitude_enabled = true;
	current_status->flag_control_rates_enabled = true;
	if (old_mode != current_status->flight_mode) state_machine_publish(status_pub, current_status, mavlink_fd);

	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_MANUAL || current_status->state_machine == SYSTEM_STATE_AUTO) {
		printf("[commander] stabilized mode\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STABILIZED);
	}
}

void update_state_machine_mode_auto(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	int old_mode = current_status->flight_mode;
	current_status->flight_mode = VEHICLE_FLIGHT_MODE_AUTO;
	current_status->flag_control_manual_enabled = true;
	current_status->flag_control_attitude_enabled = true;
	current_status->flag_control_rates_enabled = true;
	if (old_mode != current_status->flight_mode) state_machine_publish(status_pub, current_status, mavlink_fd);

	if (current_status->state_machine == SYSTEM_STATE_GROUND_READY || current_status->state_machine == SYSTEM_STATE_MANUAL || current_status->state_machine == SYSTEM_STATE_STABILIZED) {
		printf("[commander] auto mode\n");
		do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_AUTO);
	}
}


uint8_t update_state_machine_mode_request(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, uint8_t mode)
{
	printf("[commander] Requested new mode: %d\n", (int)mode);
	uint8_t ret = 1;

	/* vehicle is disarmed, mode requests arming */
	if (!(current_status->flag_system_armed) && (mode & VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
		/* only arm in standby state */
		// XXX REMOVE
		if (current_status->state_machine == SYSTEM_STATE_STANDBY || current_status->state_machine == SYSTEM_STATE_PREFLIGHT) {
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_GROUND_READY);
			ret = OK;
			printf("[commander] arming due to command request\n");
		}
	}

	/* vehicle is armed, mode requests disarming */
	if (current_status->flag_system_armed && !(mode & VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
		/* only disarm in ground ready */
		if (current_status->state_machine == SYSTEM_STATE_GROUND_READY) {
			do_state_update(status_pub, current_status, mavlink_fd, (commander_state_machine_t)SYSTEM_STATE_STANDBY);
			ret = OK;
			printf("[commander] disarming due to command request\n");
		}
	}

	/* Switch on HIL if in standby and not already in HIL mode */
	if ((current_status->state_machine == SYSTEM_STATE_STANDBY) && (mode & VEHICLE_MODE_FLAG_HIL_ENABLED)
		&& !current_status->flag_hil_enabled) {
		/* Enable HIL on request */
		current_status->flag_hil_enabled = true;
		ret = OK;
		state_machine_publish(status_pub, current_status, mavlink_fd);
		publish_armed_status(current_status);
		printf("[commander] Enabling HIL, locking down all actuators for safety.\n\t(Arming the system will not activate them while in HIL mode)\n");
	}

	/* NEVER actually switch off HIL without reboot */
	if (current_status->flag_hil_enabled && !(mode & VEHICLE_MODE_FLAG_HIL_ENABLED)) {
		fprintf(stderr, "[commander] DENYING request to switch of HIL. Please power cycle (safety reasons)\n");
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

		if (current_system_state == SYSTEM_STATE_STANDBY || current_system_state == SYSTEM_STATE_PREFLIGHT) {
			printf("system will reboot\n");
			//global_data_send_mavlink_statustext_message_out("Rebooting autopilot.. ", MAV_SEVERITY_INFO);
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


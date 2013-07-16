/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *           Thomas Gubler <thomasgubler@student.ethz.ch>
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
 * @file commander.c
 * Main system state machine implementation.
 *
 */

#include "commander.h"

#include <nuttx/config.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <sys/prctl.h>
#include <string.h>
#include <math.h>
#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/safety.h>
#include <mavlink/mavlink_log.h>

#include <drivers/drv_led.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/cpuload.h>

#include "commander_helper.h"
#include "state_machine_helper.h"
#include "calibration_routines.h"
#include "accelerometer_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "baro_calibration.h"
#include "rc_calibration.h"
#include "airspeed_calibration.h"

PARAM_DEFINE_INT32(SYS_FAILSAVE_LL, 0);	/**< Go into low-level failsafe after 0 ms */
//PARAM_DEFINE_INT32(SYS_FAILSAVE_HL, 0);	/**< Go into high-level failsafe after 0 ms */
PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);


extern struct system_load_s system_load;

#define LOW_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS 1000.0f
#define CRITICAL_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS 100.0f

/* Decouple update interval and hysteris counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 50000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))
#define LOW_VOLTAGE_BATTERY_COUNTER_LIMIT (LOW_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)
#define CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT (CRITICAL_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define STICK_ON_OFF_LIMIT 0.75f
#define STICK_THRUST_RANGE 1.0f
#define STICK_ON_OFF_HYSTERESIS_TIME_MS 1000
#define STICK_ON_OFF_COUNTER_LIMIT (STICK_ON_OFF_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define GPS_FIX_TYPE_2D 2
#define GPS_FIX_TYPE_3D 3
#define GPS_QUALITY_GOOD_HYSTERIS_TIME_MS 5000
#define GPS_QUALITY_GOOD_COUNTER_LIMIT (GPS_QUALITY_GOOD_HYSTERIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define LOCAL_POSITION_TIMEOUT 1000000 /**< consider the local position estimate invalid after 1s */

/* Mavlink file descriptors */
static int mavlink_fd;

/* flags */
static bool commander_initialized = false;
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/* timout until lowlevel failsafe */
static unsigned int failsafe_lowlevel_timeout_ms;

/* tasks waiting for low prio thread */
enum {
	LOW_PRIO_TASK_NONE = 0,
	LOW_PRIO_TASK_PARAM_SAVE,
	LOW_PRIO_TASK_PARAM_LOAD,
	LOW_PRIO_TASK_GYRO_CALIBRATION,
	LOW_PRIO_TASK_MAG_CALIBRATION,
	LOW_PRIO_TASK_ALTITUDE_CALIBRATION,
	LOW_PRIO_TASK_RC_CALIBRATION,
	LOW_PRIO_TASK_ACCEL_CALIBRATION,
	LOW_PRIO_TASK_AIRSPEED_CALIBRATION
} low_prio_task;


/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
__EXPORT int commander_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
void usage(const char *reason);

/**
 * React to commands that are sent e.g. from the mavlink module.
 */
void handle_command(int status_pub, struct vehicle_status_s *current_status, struct vehicle_command_s *cmd, int armed_pub, struct actuator_armed_s *armed);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);


int commander_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("commander already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("commander",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 40,
					 3000,
					 commander_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\tcommander is running\n");

		} else {
			warnx("\tcommander not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

void handle_command(int status_pub, struct vehicle_status_s *current_vehicle_status, struct vehicle_command_s *cmd, int armed_pub, struct actuator_armed_s *armed)
{
	/* result of the command */
	uint8_t result = VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd->command) {
		case VEHICLE_CMD_DO_SET_MODE:

			/* request to activate HIL */
			if ((int)cmd->param1 & VEHICLE_MODE_FLAG_HIL_ENABLED) {

				if (OK == hil_state_transition(status_pub, current_vehicle_status, mavlink_fd, HIL_STATE_ON)) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					result = VEHICLE_CMD_RESULT_DENIED;
				}
			}

			if ((int)cmd->param1 & VEHICLE_MODE_FLAG_SAFETY_ARMED) {
				if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_ARMED, armed_pub, armed, mavlink_fd)) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					result = VEHICLE_CMD_RESULT_DENIED;
				}
			} else {
				if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_STANDBY, armed_pub, armed, mavlink_fd)) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					result = VEHICLE_CMD_RESULT_DENIED;
				}
			}

			break;

		case VEHICLE_CMD_COMPONENT_ARM_DISARM:

			/* request to arm */
			if ((int)cmd->param1 == 1) {
				if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_ARMED, armed_pub, armed, mavlink_fd)) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					result = VEHICLE_CMD_RESULT_DENIED;
				}
			/* request to disarm */
			} else if ((int)cmd->param1 == 0) {
				if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_STANDBY, armed_pub, armed, mavlink_fd)) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					result = VEHICLE_CMD_RESULT_DENIED;
				}
			}

			break;

		case VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {

			/* request for an autopilot reboot */
			if ((int)cmd->param1 == 1) {
					if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_REBOOT, armed_pub, armed, mavlink_fd)) {
						/* reboot is executed later, after positive tune is sent */
						result = VEHICLE_CMD_RESULT_ACCEPTED;
						tune_positive();
						mavlink_log_critical(mavlink_fd, "REBOOTING SYSTEM");
						usleep(500000);
						up_systemreset();
						/* SPECIAL CASE: NEVER RETURNS FROM THIS FUNCTION CALL */

					} else {
						/* system may return here */
						result = VEHICLE_CMD_RESULT_DENIED;
						tune_negative();
					}
				}
			}
			break;

	//		/* request to land */
	//		case VEHICLE_CMD_NAV_LAND:
	//		 {
	//				//TODO: add check if landing possible
	//				//TODO: add landing maneuver
	//
	//				if (0 == update_state_machine_custom_mode_request(status_pub, current_vehicle_status, SYSTEM_STATE_ARMED)) {
	//					result = VEHICLE_CMD_RESULT_ACCEPTED;
	//		}		}
	//		break;
	//
	//		/* request to takeoff */
	//		case VEHICLE_CMD_NAV_TAKEOFF:
	//		{
	//			//TODO: add check if takeoff possible
	//			//TODO: add takeoff maneuver
	//
	//			if (0 == update_state_machine_custom_mode_request(status_pub, current_vehicle_status, SYSTEM_STATE_AUTO)) {
	//				result = VEHICLE_CMD_RESULT_ACCEPTED;
	//			}
	//		}
	//		break;
	//
			/* preflight calibration */
		case VEHICLE_CMD_PREFLIGHT_CALIBRATION:

			/* gyro calibration */
			if ((int)(cmd->param1) == 1) {

				/* check if no other task is scheduled */
				if(low_prio_task == LOW_PRIO_TASK_NONE) {

					/* try to go to INIT/PREFLIGHT arming state */
					if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_INIT, armed_pub, armed, mavlink_fd)) {
						result = VEHICLE_CMD_RESULT_ACCEPTED;
						/* now set the task for the low prio thread */
						low_prio_task = LOW_PRIO_TASK_GYRO_CALIBRATION;
					} else {
						result = VEHICLE_CMD_RESULT_DENIED;
					}
				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}

			/* magnetometer calibration */
			if ((int)(cmd->param2) == 1) {

				/* check if no other task is scheduled */
				if(low_prio_task == LOW_PRIO_TASK_NONE) {

					/* try to go to INIT/PREFLIGHT arming state */
					if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_INIT, armed_pub, armed, mavlink_fd)) {
						result = VEHICLE_CMD_RESULT_ACCEPTED;
						/* now set the task for the low prio thread */
						low_prio_task = LOW_PRIO_TASK_MAG_CALIBRATION;
					} else {
						result = VEHICLE_CMD_RESULT_DENIED;
					}
				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}


			// /* zero-altitude pressure calibration */
			// if ((int)(cmd->param3) == 1) {

			// 	/* check if no other task is scheduled */
			// 	if(low_prio_task == LOW_PRIO_TASK_NONE) {

			// 		/* try to go to INIT/PREFLIGHT arming state */
			// 		if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_INIT, armed_pub, armed, mavlink_fd)) {
			// 			result = VEHICLE_CMD_RESULT_ACCEPTED;
			// 			/* now set the task for the low prio thread */
			// 			low_prio_task = LOW_PRIO_TASK_ALTITUDE_CALIBRATION;
			// 		} else {
			// 			result = VEHICLE_CMD_RESULT_DENIED;
			// 		}
			// 	} else {
			// 		result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			// 	}
			// }



			// /* trim calibration */
			// if ((int)(cmd->param4) == 1) {

			// 	/* check if no other task is scheduled */
			// 	if(low_prio_task == LOW_PRIO_TASK_NONE) {

			// 		/* try to go to INIT/PREFLIGHT arming state */
			// 		if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_INIT, armed_pub, armed, mavlink_fd)) {
			// 			result = VEHICLE_CMD_RESULT_ACCEPTED;
			// 			/* now set the task for the low prio thread */
			// 			low_prio_task = LOW_PRIO_TASK_RC_CALIBRATION;
			// 		} else {
			// 			result = VEHICLE_CMD_RESULT_DENIED;
			// 		}
			// 	} else {
			// 		result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			// 	}
			// }


			/* accel calibration */
			if ((int)(cmd->param5) == 1) {

				/* check if no other task is scheduled */
				if(low_prio_task == LOW_PRIO_TASK_NONE) {

					/* try to go to INIT/PREFLIGHT arming state */
					if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_INIT, armed_pub, armed, mavlink_fd)) {
						result = VEHICLE_CMD_RESULT_ACCEPTED;
						/* now set the task for the low prio thread */
						low_prio_task = LOW_PRIO_TASK_ACCEL_CALIBRATION;
					} else {
						result = VEHICLE_CMD_RESULT_DENIED;
					}
				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}

			/* airspeed calibration */
			if ((int)(cmd->param6) == 1) {

				/* check if no other task is scheduled */
				if(low_prio_task == LOW_PRIO_TASK_NONE) {

					/* try to go to INIT/PREFLIGHT arming state */
					if (OK == arming_state_transition(status_pub, current_vehicle_status, ARMING_STATE_INIT, armed_pub, armed, mavlink_fd)) {
						result = VEHICLE_CMD_RESULT_ACCEPTED;
						/* now set the task for the low prio thread */
						low_prio_task = LOW_PRIO_TASK_AIRSPEED_CALIBRATION;
					} else {
						result = VEHICLE_CMD_RESULT_DENIED;
					}
				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}
			break;

		case VEHICLE_CMD_PREFLIGHT_STORAGE:

			if (((int)(cmd->param1)) == 0) {

				/* check if no other task is scheduled */
				if(low_prio_task == LOW_PRIO_TASK_NONE) {
					low_prio_task = LOW_PRIO_TASK_PARAM_LOAD;
					result = VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}


			} else if (((int)(cmd->param1)) == 1) {

				/* check if no other task is scheduled */
				if(low_prio_task == LOW_PRIO_TASK_NONE) {
					low_prio_task = LOW_PRIO_TASK_PARAM_SAVE;
					result = VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}
		break;

	default:
		mavlink_log_critical(mavlink_fd, "[cmd] refusing unsupported command");
		result = VEHICLE_CMD_RESULT_UNSUPPORTED;
		break;
	}

	/* supported command handling stop */
	if (result == VEHICLE_CMD_RESULT_FAILED ||
	    result == VEHICLE_CMD_RESULT_DENIED ||
	    result == VEHICLE_CMD_RESULT_UNSUPPORTED ||
	    result == VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {

		tune_negative();

	} else if (result == VEHICLE_CMD_RESULT_ACCEPTED) {

		tune_positive();
	}

	/* send any requested ACKs */
	if (cmd->confirmation > 0) {
		/* send acknowledge command */
		// XXX TODO
	}

}

int commander_thread_main(int argc, char *argv[])
{
	/* not yet initialized */
	commander_initialized = false;
	bool home_position_set = false;

	bool battery_tune_played = false;
	bool arm_tune_played = false;

	/* set parameters */
	failsafe_lowlevel_timeout_ms = 0;
	param_get(param_find("SYS_FAILSAVE_LL"), &failsafe_lowlevel_timeout_ms);

	param_t _param_sys_type = param_find("MAV_TYPE");
	param_t _param_system_id = param_find("MAV_SYS_ID");
	param_t _param_component_id = param_find("MAV_COMP_ID");

	/* welcome user */
	warnx("[commander] starting");

	/* pthread for slow low prio thread */
	pthread_t commander_low_prio_thread;

	/* initialize */
	if (led_init() != 0) {
		warnx("ERROR: Failed to initialize leds");
	}

	if (buzzer_init() != OK) {
		warnx("ERROR: Failed to initialize buzzer");
	}

	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	if (mavlink_fd < 0) {
		warnx("ERROR: Failed to open MAVLink log stream, start mavlink app first.");
	}
	
	/* Main state machine */
	struct vehicle_status_s current_status;
	orb_advert_t status_pub;
	/* make sure we are in preflight state */
	memset(&current_status, 0, sizeof(current_status));

	/* armed topic */
	struct actuator_armed_s armed;
	orb_advert_t armed_pub;
	/* Initialize armed with all false */
	memset(&armed, 0, sizeof(armed));

	/* flags for control apps */
	struct vehicle_control_mode_s control_mode;
	orb_advert_t control_mode_pub;

	/* Initialize all flags to false */
	memset(&control_mode, 0, sizeof(control_mode));

	current_status.navigation_state = NAVIGATION_STATE_INIT;
	current_status.arming_state = ARMING_STATE_INIT;
	current_status.hil_state = HIL_STATE_OFF;

	/* neither manual nor offboard control commands have been received */
	current_status.offboard_control_signal_found_once = false;
	current_status.rc_signal_found_once = false;

	/* mark all signals lost as long as they haven't been found */
	current_status.rc_signal_lost = true;
	current_status.offboard_control_signal_lost = true;

	/* allow manual override initially */
	current_status.flag_external_manual_override_ok = true;

	/* flag position info as bad, do not allow auto mode */
	// current_status.flag_vector_flight_mode_ok = false;

	/* set battery warning flag */
	current_status.battery_warning = VEHICLE_BATTERY_WARNING_NONE;
	
	/* set safety device detection flag */
	/* XXX do we need this? */
	//current_status.flag_safety_present = false;

	// XXX for now just set sensors as initialized
	current_status.condition_system_sensors_initialized = true;

	// XXX just disable offboard control for now
	control_mode.flag_control_offboard_enabled = false;

	/* advertise to ORB */
	status_pub = orb_advertise(ORB_ID(vehicle_status), &current_status);
	/* publish current state machine */
	
	/* publish the new state */
	current_status.counter++;
	current_status.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(vehicle_status), status_pub, &current_status);


	armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

	control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &control_mode);	

	/* home position */
	orb_advert_t home_pub = -1;
	struct home_position_s home;
	memset(&home, 0, sizeof(home));

	if (status_pub < 0) {
		warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
		warnx("exiting.");
		exit(ERROR);
	}

	// XXX needed?
	mavlink_log_info(mavlink_fd, "system is running");

	pthread_attr_t commander_low_prio_attr;
	pthread_attr_init(&commander_low_prio_attr);
	pthread_attr_setstacksize(&commander_low_prio_attr, 2048);

	struct sched_param param;
	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
	(void)pthread_attr_setschedparam(&commander_low_prio_attr, &param);
	pthread_create(&commander_low_prio_thread, &commander_low_prio_attr, commander_low_prio_loop, NULL);

	/* Start monitoring loop */
	uint16_t counter = 0;

	/* Initialize to 0.0V */
	float battery_voltage = 0.0f;
	bool battery_voltage_valid = false;
	bool low_battery_voltage_actions_done = false;
	bool critical_battery_voltage_actions_done = false;
	uint8_t low_voltage_counter = 0;
	uint16_t critical_voltage_counter = 0;
	float bat_remain = 1.0f;

	uint16_t stick_off_counter = 0;
	uint16_t stick_on_counter = 0;

	/* To remember when last notification was sent */
	uint64_t last_print_time = 0;

	float voltage_previous = 0.0f;

	uint64_t last_idle_time = 0;

	uint64_t start_time = 0;

	/* XXX use this! */
	//uint64_t failsave_ll_start_time = 0;

	bool state_changed = true;
	bool param_init_forced = true;

	bool new_data = false;

	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	struct safety_s safety;
	memset(&safety, 0, sizeof(safety));

	/* Subscribe to manual control data */
	int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s sp_man;
	memset(&sp_man, 0, sizeof(sp_man));

	/* Subscribe to offboard control data */
	int sp_offboard_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	struct offboard_control_setpoint_s sp_offboard;
	memset(&sp_offboard, 0, sizeof(sp_offboard));

	/* Subscribe to global position */
	int global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_s global_position;
	memset(&global_position, 0, sizeof(global_position));
	uint64_t last_global_position_time = 0;

	/* Subscribe to local position data */
	int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	struct vehicle_local_position_s local_position;
	memset(&local_position, 0, sizeof(local_position));
	uint64_t last_local_position_time = 0;

	/* 
	 * The home position is set based on GPS only, to prevent a dependency between
	 * position estimator and commander. RAW GPS is more than good enough for a
	 * non-flying vehicle.
	 */

	/* Subscribe to GPS topic */
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	struct vehicle_gps_position_s gps_position;
	memset(&gps_position, 0, sizeof(gps_position));

	/* Subscribe to sensor topic */
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s sensors;
	memset(&sensors, 0, sizeof(sensors));

	/* Subscribe to differential pressure topic */
	int diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	struct differential_pressure_s diff_pres;
	memset(&diff_pres, 0, sizeof(diff_pres));
	uint64_t last_diff_pres_time = 0;

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	/* Subscribe to parameters changed topic */
	int param_changed_sub = orb_subscribe(ORB_ID(parameter_update));
	struct parameter_update_s param_changed;
	memset(&param_changed, 0, sizeof(param_changed));

	/* Subscribe to battery topic */
	int battery_sub = orb_subscribe(ORB_ID(battery_status));
	struct battery_status_s battery;
	memset(&battery, 0, sizeof(battery));
	battery.voltage_v = 0.0f;

	/* Subscribe to subsystem info topic */
	int subsys_sub = orb_subscribe(ORB_ID(subsystem_info));
	struct subsystem_info_s info;
	memset(&info, 0, sizeof(info));

	/* now initialized */
	commander_initialized = true;
	thread_running = true;

	start_time = hrt_absolute_time();

	while (!thread_should_exit) {

		/* update parameters */
		orb_check(param_changed_sub, &new_data);

		if (new_data || param_init_forced) {
			param_init_forced = false;
			/* parameters changed */
			orb_copy(ORB_ID(parameter_update), param_changed_sub, &param_changed);

			/* update parameters */
			if (!armed.armed) {
				if (param_get(_param_sys_type, &(current_status.system_type)) != OK) {
					warnx("failed getting new system type");
				}

				/* disable manual override for all systems that rely on electronic stabilization */
				if (current_status.system_type == VEHICLE_TYPE_QUADROTOR ||
				    current_status.system_type == VEHICLE_TYPE_HEXAROTOR ||
				    current_status.system_type == VEHICLE_TYPE_OCTOROTOR) {
					current_status.flag_external_manual_override_ok = false;

				} else {
					current_status.flag_external_manual_override_ok = true;
				}

				/* check and update system / component ID */
				param_get(_param_system_id, &(current_status.system_id));
				param_get(_param_component_id, &(current_status.component_id));

			}
		}

		orb_check(sp_man_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
		}

		orb_check(sp_offboard_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(offboard_control_setpoint), sp_offboard_sub, &sp_offboard);
		}

		orb_check(sensor_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);
		}

		orb_check(diff_pres_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);
			last_diff_pres_time = diff_pres.timestamp;
		}

		orb_check(cmd_sub, &new_data);

		if (new_data) {
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			handle_command(status_pub, &current_status, &cmd, armed_pub, &armed);
		}

		/* update safety topic */
		orb_check(safety_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(safety), safety_sub, &safety);
		}

		/* update global position estimate */
		orb_check(global_position_sub, &new_data);

		if (new_data) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
			last_global_position_time = global_position.timestamp;
		}

		/* update local position estimate */
		orb_check(local_position_sub, &new_data);

		if (new_data) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
			last_local_position_time = local_position.timestamp;
		}

		/* set the condition to valid if there has recently been a local position estimate */
		if (hrt_absolute_time() - last_local_position_time < LOCAL_POSITION_TIMEOUT) {
			current_status.condition_local_position_valid = true;
		} else {
			current_status.condition_local_position_valid = false;
		}

		/* update battery status */
		orb_check(battery_sub, &new_data);
		if (new_data) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);
			battery_voltage = battery.voltage_v;
			battery_voltage_valid = true;

			/*
			 * Only update battery voltage estimate if system has
			 * been running for two and a half seconds.
			 */
			if (hrt_absolute_time() - start_time > 2500000) {
				bat_remain = battery_remaining_estimate_voltage(battery_voltage);
			}
		}

		/* update subsystem */
		orb_check(subsys_sub, &new_data);
		
		if (new_data) {
			orb_copy(ORB_ID(subsystem_info), subsys_sub, &info);

			warnx("Subsys changed: %d\n", (int)info.subsystem_type);

			/* mark / unmark as present */
			if (info.present) {
				current_status.onboard_control_sensors_present |= info.subsystem_type;

			} else {
				current_status.onboard_control_sensors_present &= ~info.subsystem_type;
			}

			/* mark / unmark as enabled */
			if (info.enabled) {
				current_status.onboard_control_sensors_enabled |= info.subsystem_type;

			} else {
				current_status.onboard_control_sensors_enabled &= ~info.subsystem_type;
			}

			/* mark / unmark as ok */
			if (info.ok) {
				current_status.onboard_control_sensors_health |= info.subsystem_type;

			} else {
				current_status.onboard_control_sensors_health &= ~info.subsystem_type;
			}
		}

		/* Slow but important 8 Hz checks */
		if (counter % ((1000000 / COMMANDER_MONITORING_INTERVAL) / 8) == 0) {

			/* XXX if armed */
			if ((false /*current_status.state_machine == SYSTEM_STATE_GROUND_READY ||
			     current_status.state_machine == SYSTEM_STATE_AUTO  ||
			     current_status.state_machine == SYSTEM_STATE_MANUAL*/)) {
				/* armed, solid */
				led_on(LED_AMBER);

			} else if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
				/* not armed */
				led_toggle(LED_AMBER);
			}

			if (hrt_absolute_time() - gps_position.timestamp_position < 2000000) {

				/* toggle GPS (blue) led at 1 Hz if GPS present but no lock, make is solid once locked */
				if ((hrt_absolute_time() - gps_position.timestamp_position < 2000000)
					&& (gps_position.fix_type == GPS_FIX_TYPE_3D)) {
					/* GPS lock */
					led_on(LED_BLUE);

				} else if ((counter + 4) % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
					/* no GPS lock, but GPS module is aquiring lock */
					led_toggle(LED_BLUE);
				}

			} else {
				/* no GPS info, don't light the blue led */
				led_off(LED_BLUE);
			}

			/* toggle GPS led at 5 Hz in HIL mode */
			if (current_status.flag_hil_enabled) {
				/* hil enabled */
				led_toggle(LED_BLUE);

			} else if (bat_remain < 0.3f && (low_voltage_counter > LOW_VOLTAGE_BATTERY_COUNTER_LIMIT)) {
				/* toggle arming (red) at 5 Hz on low battery or error */
				led_toggle(LED_AMBER);
			}

		}

		if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
			/* compute system load */
			uint64_t interval_runtime = system_load.tasks[0].total_runtime - last_idle_time;

			if (last_idle_time > 0)
				current_status.load = 1000 - (interval_runtime / 1000);	//system load is time spent in non-idle

			last_idle_time = system_load.tasks[0].total_runtime;
		}

		/* Check battery voltage */
		/* write to sys_status */
		if (battery_voltage_valid) {
			current_status.voltage_battery = battery_voltage;

		} else {
			current_status.voltage_battery = 0.0f;
		}

		/* if battery voltage is getting lower, warn using buzzer, etc. */
		if (battery_voltage_valid && (bat_remain < 0.15f /* XXX MAGIC NUMBER */) && (false == low_battery_voltage_actions_done)) { //TODO: add filter, or call emergency after n measurements < VOLTAGE_BATTERY_MINIMAL_MILLIVOLTS

			if (low_voltage_counter > LOW_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				low_battery_voltage_actions_done = true;
				mavlink_log_critical(mavlink_fd, "[cmd] WARNING! LOW BATTERY!");
				current_status.battery_warning = VEHICLE_BATTERY_WARNING_WARNING;
			}

			low_voltage_counter++;
		}

		/* Critical, this is rather an emergency, change state machine */
		else if (battery_voltage_valid && (bat_remain < 0.1f /* XXX MAGIC NUMBER */) && (false == critical_battery_voltage_actions_done && true == low_battery_voltage_actions_done)) {
			if (critical_voltage_counter > CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				critical_battery_voltage_actions_done = true;
				mavlink_log_critical(mavlink_fd, "[cmd] EMERGENCY! CRITICAL BATTERY!");
				current_status.battery_warning = VEHICLE_BATTERY_WARNING_ALERT;
				// XXX implement this
				// state_machine_emergency(status_pub, &current_status, mavlink_fd);
			}

			critical_voltage_counter++;

		} else {
			low_voltage_counter = 0;
			critical_voltage_counter = 0;
		}

		/* End battery voltage check */

		/* If in INIT state, try to proceed to STANDBY state */
		if (current_status.arming_state == ARMING_STATE_INIT) {
			// XXX check for sensors
			arming_state_transition(status_pub, &current_status, ARMING_STATE_STANDBY, armed_pub, &armed, mavlink_fd);
		} else {
			// XXX: Add emergency stuff if sensors are lost
		}


		/*
		 * Check for valid position information.
		 *
		 * If the system has a valid position source from an onboard
		 * position estimator, it is safe to operate it autonomously.
		 * The flag_vector_flight_mode_ok flag indicates that a minimum
		 * set of position measurements is available.
		 */

		/* store current state to reason later about a state change */
		// bool vector_flight_mode_ok = current_status.flag_vector_flight_mode_ok;
		bool global_pos_valid = current_status.condition_global_position_valid;
		bool local_pos_valid = current_status.condition_local_position_valid;
		bool airspeed_valid = current_status.condition_airspeed_valid;


		/* check for global or local position updates, set a timeout of 2s */
		if (hrt_absolute_time() - last_global_position_time < 2000000 && hrt_absolute_time() > 2000000) {
			current_status.condition_global_position_valid = true;
			// XXX check for controller status and home position as well

		} else {
			current_status.condition_global_position_valid = false;
		}

		if (hrt_absolute_time() - last_local_position_time < 2000000 && hrt_absolute_time() > 2000000) {
			current_status.condition_local_position_valid = true;
			// XXX check for controller status and home position as well

		} else {
			current_status.condition_local_position_valid = false;
		}

		/* Check for valid airspeed/differential pressure measurements */
		if (hrt_absolute_time() - last_diff_pres_time < 2000000 && hrt_absolute_time() > 2000000) {
			current_status.condition_airspeed_valid = true;

		} else {
			current_status.condition_airspeed_valid = false;
		}

		/*
		 * Consolidate global position and local position valid flags
		 * for vector flight mode.
		 */
		// if (current_status.condition_local_position_valid ||
		//     current_status.condition_global_position_valid) {
		// 	current_status.flag_vector_flight_mode_ok = true;

		// } else {
		// 	current_status.flag_vector_flight_mode_ok = false;
		// }

		/* consolidate state change, flag as changed if required */
		if (global_pos_valid != current_status.condition_global_position_valid ||
		    local_pos_valid != current_status.condition_local_position_valid ||
		    airspeed_valid != current_status.condition_airspeed_valid) {
			state_changed = true;
		}

		/*
		 * Mark the position of the first position lock as return to launch (RTL)
		 * position. The MAV will return here on command or emergency.
		 *
		 * Conditions:
		 *
		 * 	1) The system aquired position lock just now
		 *	2) The system has not aquired position lock before
		 *	3) The system is not armed (on the ground)
		 */
		// if (!current_status.flag_valid_launch_position &&
		//     !vector_flight_mode_ok && current_status.flag_vector_flight_mode_ok &&
		//     !current_status.flag_system_armed) {
		// 	 first time a valid position, store it and emit it 

		// 	// XXX implement storage and publication of RTL position
		// 	current_status.flag_valid_launch_position = true;
		// 	current_status.flag_auto_flight_mode_ok = true;
		// 	state_changed = true;
		// }

		orb_check(gps_sub, &new_data);
		if (new_data) {


			orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_position);

			/* check for first, long-term and valid GPS lock -> set home position */
			float hdop_m = gps_position.eph_m;
			float vdop_m = gps_position.epv_m;

			/* check if gps fix is ok */
			// XXX magic number
			float hdop_threshold_m = 4.0f;
			float vdop_threshold_m = 8.0f;

			/*
			 * If horizontal dilution of precision (hdop / eph)
			 * and vertical diluation of precision (vdop / epv)
			 * are below a certain threshold (e.g. 4 m), AND
			 * home position is not yet set AND the last GPS
			 * GPS measurement is not older than two seconds AND
			 * the system is currently not armed, set home
			 * position to the current position.
			 */

			if (gps_position.fix_type == GPS_FIX_TYPE_3D
				&& (hdop_m < hdop_threshold_m)
				&& (vdop_m < vdop_threshold_m) // XXX note that vdop is 0 for mtk
				&& !home_position_set
				&& (hrt_absolute_time() - gps_position.timestamp_position < 2000000)
				&& !armed.armed) {
				warnx("setting home position");

				/* copy position data to uORB home message, store it locally as well */
				home.lat = gps_position.lat;
				home.lon = gps_position.lon;
				home.alt = gps_position.alt;

				home.eph_m = gps_position.eph_m;
				home.epv_m = gps_position.epv_m;

				home.s_variance_m_s = gps_position.s_variance_m_s;
				home.p_variance_m = gps_position.p_variance_m;

				/* announce new home position */
				if (home_pub > 0) {
					orb_publish(ORB_ID(home_position), home_pub, &home);
				} else {
					home_pub = orb_advertise(ORB_ID(home_position), &home);
				}

				/* mark home position as set */
				home_position_set = true;
				tune_positive();
			}
		}

		/* ignore RC signals if in offboard control mode */
		if (!current_status.offboard_control_signal_found_once && sp_man.timestamp != 0) {
			/* Start RC state check */

			if ((hrt_absolute_time() - sp_man.timestamp) < 100000) {

				/*
				* Check if manual control modes have to be switched
				*/
				if (!isfinite(sp_man.mode_switch)) {

					warnx("mode sw not finite");
					/* no valid stick position, go to default */
					current_status.mode_switch = MODE_SWITCH_MANUAL;

				} else if (sp_man.mode_switch < -STICK_ON_OFF_LIMIT) {

					/* bottom stick position, go to manual mode */
					current_status.mode_switch = MODE_SWITCH_MANUAL;

				} else if (sp_man.mode_switch > STICK_ON_OFF_LIMIT) {

					/* top stick position, set auto/mission for all vehicle types */
					current_status.mode_switch = MODE_SWITCH_AUTO;

				} else {

					/* center stick position, set seatbelt/simple control */
					current_status.mode_switch = MODE_SWITCH_SEATBELT;
				}

				// warnx("man ctrl mode: %d\n", (int)current_status.manual_control_mode);

				/*
				* Check if land/RTL is requested
				*/
				if (!isfinite(sp_man.return_switch)) {

					/* this switch is not properly mapped, set default */
					current_status.return_switch = RETURN_SWITCH_NONE;

				} else if (sp_man.return_switch < -STICK_ON_OFF_LIMIT) {

					/* bottom stick position, set altitude hold */
					current_status.return_switch = RETURN_SWITCH_NONE;

				} else if (sp_man.return_switch > STICK_ON_OFF_LIMIT) {

					/* top stick position */
					current_status.return_switch = RETURN_SWITCH_RETURN;

				} else {
					/* center stick position, set default */
					current_status.return_switch = RETURN_SWITCH_NONE;
				}

				/* check mission switch  */
				if (!isfinite(sp_man.mission_switch)) {

					/* this switch is not properly mapped, set default */
					current_status.mission_switch = MISSION_SWITCH_NONE;

				} else if (sp_man.mission_switch > STICK_ON_OFF_LIMIT) {

					/* top switch position */
					current_status.mission_switch = MISSION_SWITCH_MISSION;

				} else if (sp_man.mission_switch < -STICK_ON_OFF_LIMIT) {

					/* bottom switch position */
					current_status.mission_switch = MISSION_SWITCH_NONE;

				} else {

					/* center switch position, set default */
					current_status.mission_switch = MISSION_SWITCH_NONE; // XXX default?
				}

				/* Now it's time to handle the stick inputs */
				
				switch (current_status.arming_state) {
			
					/* evaluate the navigation state when disarmed */
					case ARMING_STATE_STANDBY:
			
						/* just manual, XXX this might depend on the return switch */
						if (current_status.mode_switch == MODE_SWITCH_MANUAL) {
							
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL_STANDBY, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// These is not supposed to happen
								warnx("ERROR: Navigation state MANUAL_STANDBY rejected");
							}
			
						/* Try seatbelt or fallback to manual */
						} else if (current_status.mode_switch == MODE_SWITCH_SEATBELT) {
			
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT_STANDBY, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// fallback to MANUAL_STANDBY
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL_STANDBY, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// These is not supposed to happen
								warnx("ERROR: Navigation state MANUAL_STANDBY rejected");
								}
							}
			
						/* Try auto or fallback to seatbelt or even manual */
						} else if (current_status.mode_switch == MODE_SWITCH_AUTO) {
							
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_AUTO_STANDBY, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// first fallback to SEATBELT_STANDY
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT_STANDBY, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// or fallback to MANUAL_STANDBY
									if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL_STANDBY, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// These is not supposed to happen
									warnx("ERROR: Navigation state MANUAL_STANDBY rejected");
									}
								}
							}
						}
			
						break;
			
					/* evaluate the navigation state when armed */
					case ARMING_STATE_ARMED:
			
						/* Always accept manual mode */
						if (current_status.mode_switch == MODE_SWITCH_MANUAL) {

							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// These is not supposed to happen
								warnx("ERROR: Navigation state MANUAL rejected");
							}
			
						/* SEATBELT_STANDBY (fallback: MANUAL) */
						} else if (current_status.mode_switch == MODE_SWITCH_SEATBELT
								&& current_status.return_switch == RETURN_SWITCH_NONE) {
			
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// fallback to MANUAL_STANDBY
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// These is not supposed to happen
								warnx("ERROR: Navigation state MANUAL rejected");
								}
							}
			
						/* SEATBELT_DESCENT (fallback: MANUAL) */
						} else if (current_status.mode_switch == MODE_SWITCH_SEATBELT
								&& current_status.return_switch == RETURN_SWITCH_RETURN) {
			
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT_DESCENT, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// fallback to MANUAL_STANDBY
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								// These is not supposed to happen
								warnx("ERROR: Navigation state MANUAL rejected");
								}
							}
			
						/* AUTO_LOITER (fallback: SEATBELT, MANUAL) */
						} else if (current_status.mode_switch == MODE_SWITCH_AUTO
								&& current_status.return_switch == RETURN_SWITCH_NONE
								&& current_status.mission_switch == MISSION_SWITCH_NONE) {
			
							/* we might come from the disarmed state AUTO_STANDBY */
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_AUTO_READY, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// fallback to MANUAL_STANDBY
									if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// These is not supposed to happen
									warnx("ERROR: Navigation state MANUAL rejected");
									}
								}
							/* or from some other armed state like SEATBELT or MANUAL */
							} else if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_AUTO_LOITER, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// fallback to MANUAL_STANDBY
									if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// These is not supposed to happen
									warnx("ERROR: Navigation state MANUAL rejected");
									}
								}
							}
			
						/* AUTO_MISSION (fallback: SEATBELT, MANUAL) */
						} else if (current_status.mode_switch == MODE_SWITCH_AUTO
								&& current_status.return_switch == RETURN_SWITCH_NONE
								&& current_status.mission_switch == MISSION_SWITCH_MISSION) {
							
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_AUTO_MISSION, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// fallback to MANUAL_STANDBY
									if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// These is not supposed to happen
									warnx("ERROR: Navigation state MANUAL rejected");
									}
								}
							}
			
						/* AUTO_RTL (fallback: SEATBELT_DESCENT, MANUAL) */
						} else if (current_status.mode_switch == MODE_SWITCH_AUTO
								&& current_status.return_switch == RETURN_SWITCH_RETURN
								&& (current_status.mission_switch == MISSION_SWITCH_NONE || current_status.mission_switch == MISSION_SWITCH_MISSION)) {
			
							if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_AUTO_RTL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
								if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_SEATBELT_DESCENT, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// fallback to MANUAL_STANDBY
									if (navigation_state_transition(status_pub, &current_status, NAVIGATION_STATE_MANUAL, control_mode_pub, &control_mode, mavlink_fd) != OK) {
									// These is not supposed to happen
									warnx("ERROR: Navigation state MANUAL rejected");
									}
								}
							}
						}
						break;

						// XXX we might be missing something that triggers a transition from RTL to LAND
			
					case ARMING_STATE_ARMED_ERROR:
			
						// XXX work out fail-safe scenarios
						break;
			
					case ARMING_STATE_STANDBY_ERROR:
			
						// XXX work out fail-safe scenarios
						break;
			
					case ARMING_STATE_REBOOT:
			
						// XXX I don't think we should end up here
						break;
			
					case ARMING_STATE_IN_AIR_RESTORE:
			
						// XXX not sure what to do here
						break;
					default:
						break;
				}
				/* handle the case where RC signal was regained */
				if (!current_status.rc_signal_found_once) {
					current_status.rc_signal_found_once = true;
					mavlink_log_critical(mavlink_fd, "DETECTED RC SIGNAL FIRST TIME.");

				} else {
					if (current_status.rc_signal_lost) {
						mavlink_log_critical(mavlink_fd, "[cmd] RECOVERY - RC SIGNAL GAINED!");
					}
				}

				/*
				* Check if left stick is in lower left position --> switch to standby state.
				* Do this only for multirotors, not for fixed wing aircraft.
				*/
				// printf("checking\n");
				if ((sp_man.yaw < -STICK_ON_OFF_LIMIT) && (sp_man.throttle < STICK_THRUST_RANGE * 0.2f)) {

					if (stick_off_counter > STICK_ON_OFF_COUNTER_LIMIT) {

						if((current_status.system_type == VEHICLE_TYPE_QUADROTOR) ||
						 (current_status.system_type == VEHICLE_TYPE_HEXAROTOR) ||
						 (current_status.system_type == VEHICLE_TYPE_OCTOROTOR)
						) {
							if (current_status.mode_switch != MODE_SWITCH_MANUAL) {
								mavlink_log_critical(mavlink_fd, "DISARM DENY, go manual mode first");
								tune_negative();
							} else {
								arming_state_transition(status_pub, &current_status, ARMING_STATE_STANDBY, armed_pub, &armed, mavlink_fd);
								tune_positive();
							}
							
						} else {
							mavlink_log_critical(mavlink_fd, "DISARM not allowed");
							tune_negative();
						}
						stick_off_counter = 0;

					} else {
						stick_off_counter++;
						stick_on_counter = 0;
					}
				}

				/* check if left stick is in lower right position and we're in manual --> arm */
				if (!control_mode.flag_control_position_enabled && !control_mode.flag_control_velocity_enabled &&
					sp_man.yaw > STICK_ON_OFF_LIMIT &&
					sp_man.throttle < STICK_THRUST_RANGE * 0.1f) {
					if (stick_on_counter > STICK_ON_OFF_COUNTER_LIMIT) {
						arming_state_transition(status_pub, &current_status, ARMING_STATE_ARMED, armed_pub, &armed, mavlink_fd);
						stick_on_counter = 0;
						tune_positive();

					} else {
						stick_on_counter++;
						stick_off_counter = 0;
					}
				}

				current_status.rc_signal_cutting_off = false;
				current_status.rc_signal_lost = false;
				current_status.rc_signal_lost_interval = 0;

			} else {

				/* print error message for first RC glitch and then every 5 s / 5000 ms) */
				if (!current_status.rc_signal_cutting_off || ((hrt_absolute_time() - last_print_time) > 5000000)) {
					/* only complain if the offboard control is NOT active */
					current_status.rc_signal_cutting_off = true;
					mavlink_log_critical(mavlink_fd, "CRITICAL - NO REMOTE SIGNAL!");

					if (!current_status.rc_signal_cutting_off) {
						printf("Reason: not rc_signal_cutting_off\n");
					} else {
						printf("last print time: %llu\n", last_print_time);
					}

					last_print_time = hrt_absolute_time();
				}

				/* flag as lost and update interval since when the signal was lost (to initiate RTL after some time) */
				current_status.rc_signal_lost_interval = hrt_absolute_time() - sp_man.timestamp;

				/* if the RC signal is gone for a full second, consider it lost */
				if (current_status.rc_signal_lost_interval > 1000000) {
					current_status.rc_signal_lost = true;
					current_status.failsave_lowlevel = true;
					state_changed = true;
				}

				// if (hrt_absolute_time() - current_status.failsave_ll_start_time > failsafe_lowlevel_timeout_ms*1000) {
				// 	publish_armed_status(&current_status);
				// }
			}
		}




		/* End mode switch */

		/* END RC state check */


		/* State machine update for offboard control */
		if (!current_status.rc_signal_found_once && sp_offboard.timestamp != 0) {
			if ((hrt_absolute_time() - sp_offboard.timestamp) < 5000000) {

				// /* decide about attitude control flag, enable in att/pos/vel */
				// bool attitude_ctrl_enabled = (sp_offboard.mode == OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE ||
				// 				  sp_offboard.mode == OFFBOARD_CONTROL_MODE_DIRECT_VELOCITY ||
				// 				  sp_offboard.mode == OFFBOARD_CONTROL_MODE_DIRECT_POSITION);

				// /* decide about rate control flag, enable it always XXX (for now) */
				// bool rates_ctrl_enabled = true;

				// /* set up control mode */
				// if (current_status.flag_control_attitude_enabled != attitude_ctrl_enabled) {
				// 	current_status.flag_control_attitude_enabled = attitude_ctrl_enabled;
				// 	state_changed = true;
				// }

				// if (current_status.flag_control_rates_enabled != rates_ctrl_enabled) {
				// 	current_status.flag_control_rates_enabled = rates_ctrl_enabled;
				// 	state_changed = true;
				// }

				// /* handle the case where offboard control signal was regained */
				// if (!current_status.offboard_control_signal_found_once) {
				// 	current_status.offboard_control_signal_found_once = true;
				// 	/* enable offboard control, disable manual input */
				// 	current_status.flag_control_manual_enabled = false;
				// 	current_status.flag_control_offboard_enabled = true;
				// 	state_changed = true;
				// 	tune_positive();

				// 	mavlink_log_critical(mavlink_fd, "DETECTED OFFBOARD SIGNAL FIRST");

				// } else {
				// 	if (current_status.offboard_control_signal_lost) {
				// 		mavlink_log_critical(mavlink_fd, "RECOVERY OFFBOARD CONTROL");
				// 		state_changed = true;
				// 		tune_positive();
				// 	}
				// }

				current_status.offboard_control_signal_weak = false;
				current_status.offboard_control_signal_lost = false;
				current_status.offboard_control_signal_lost_interval = 0;

				// XXX check if this is correct
				/* arm / disarm on request */
				if (sp_offboard.armed && !armed.armed) {

					arming_state_transition(status_pub, &current_status, ARMING_STATE_ARMED, armed_pub, &armed, mavlink_fd);

				} else if (!sp_offboard.armed && armed.armed) {

					arming_state_transition(status_pub, &current_status, ARMING_STATE_STANDBY, armed_pub, &armed, mavlink_fd);
				}

			} else {

				/* print error message for first RC glitch and then every 5 s / 5000 ms) */
				if (!current_status.offboard_control_signal_weak || ((hrt_absolute_time() - last_print_time) > 5000000)) {
					current_status.offboard_control_signal_weak = true;
					mavlink_log_critical(mavlink_fd, "CRIT:NO OFFBOARD CONTROL!");
					last_print_time = hrt_absolute_time();
				}

				/* flag as lost and update interval since when the signal was lost (to initiate RTL after some time) */
				current_status.offboard_control_signal_lost_interval = hrt_absolute_time() - sp_offboard.timestamp;

				/* if the signal is gone for 0.1 seconds, consider it lost */
				if (current_status.offboard_control_signal_lost_interval > 100000) {
					current_status.offboard_control_signal_lost = true;
					current_status.failsave_lowlevel_start_time = hrt_absolute_time();
					tune_positive();

					/* kill motors after timeout */
					if (hrt_absolute_time() - current_status.failsave_lowlevel_start_time > failsafe_lowlevel_timeout_ms * 1000) {
						current_status.failsave_lowlevel = true;
						state_changed = true;
					}
				}
			}
		}




		current_status.counter++;
		current_status.timestamp = hrt_absolute_time();


		// XXX this is missing
		/* If full run came back clean, transition to standby */
		// if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT &&
		//     current_status.flag_preflight_gyro_calibration == false &&
		//     current_status.flag_preflight_mag_calibration == false &&
		//     current_status.flag_preflight_accel_calibration == false) {
		// 	/* All ok, no calibration going on, go to standby */
		// 	do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_STANDBY);
		// }

		/* publish at least with 1 Hz */
		if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0 || state_changed) {

			orb_publish(ORB_ID(vehicle_status), status_pub, &current_status);
			state_changed = false;
		}



		/* Store old modes to detect and act on state transitions */
		voltage_previous = current_status.voltage_battery;


		/* play tone according to evaluation result */
		/* check if we recently armed */
		if (!arm_tune_played && armed.armed && ( !safety.safety_switch_available || (safety.safety_off && safety.safety_switch_available))) {
			if (tune_arm() == OK)
				arm_tune_played = true;

		/* Trigger audio event for low battery */
		} else if (bat_remain < 0.1f && battery_voltage_valid) {
			if (tune_critical_bat() == OK)
				battery_tune_played = true;
		} else if (bat_remain < 0.2f && battery_voltage_valid) {
			if (tune_low_bat() == OK)
				battery_tune_played = true;
		} else if(battery_tune_played) {
			tune_stop();
			battery_tune_played = false;
		}

		/* reset arm_tune_played when disarmed */
		if (!(armed.armed && ( !safety.safety_switch_available || (safety.safety_off && safety.safety_switch_available)))) {
			arm_tune_played = false;
		}


		/* XXX use this voltage_previous */
		fflush(stdout);
		counter++;
		usleep(COMMANDER_MONITORING_INTERVAL);
	}

	/* wait for threads to complete */
	pthread_join(commander_low_prio_thread, NULL);

	/* close fds */
	led_deinit();
	buzzer_deinit();
	close(sp_man_sub);
	close(sp_offboard_sub);
	close(local_position_sub);
	close(global_position_sub);
	close(gps_sub);
	close(sensor_sub);
	close(safety_sub);
	close(cmd_sub);
	close(subsys_sub);
	close(diff_pres_sub);
	close(param_changed_sub);
	close(battery_sub);

	warnx("exiting");
	fflush(stdout);

	thread_running = false;

	return 0;
}


void *commander_low_prio_loop(void *arg)
{
	/* Set thread name */
	prctl(PR_SET_NAME, "commander low prio", getpid());

	while (!thread_should_exit) {

		switch (low_prio_task) {

			case LOW_PRIO_TASK_PARAM_LOAD:

				if (0 == param_load_default()) {
					mavlink_log_info(mavlink_fd, "Param load success");
				} else {
					mavlink_log_critical(mavlink_fd, "Param load ERROR");
					tune_error();
				}
				low_prio_task = LOW_PRIO_TASK_NONE;
				break;

			case LOW_PRIO_TASK_PARAM_SAVE:

				if (0 == param_save_default()) {
					mavlink_log_info(mavlink_fd, "Param save success");
				} else {
					mavlink_log_critical(mavlink_fd, "Param save ERROR");
					tune_error();
				}
				low_prio_task = LOW_PRIO_TASK_NONE;
				break;

			case LOW_PRIO_TASK_GYRO_CALIBRATION:

				do_gyro_calibration(mavlink_fd);

				low_prio_task = LOW_PRIO_TASK_NONE;
				break;

			case LOW_PRIO_TASK_MAG_CALIBRATION:

				do_mag_calibration(mavlink_fd);

				low_prio_task = LOW_PRIO_TASK_NONE;
				break;

			case LOW_PRIO_TASK_ALTITUDE_CALIBRATION:

				// do_baro_calibration(mavlink_fd);

			case LOW_PRIO_TASK_RC_CALIBRATION:

				// do_rc_calibration(mavlink_fd);

				low_prio_task = LOW_PRIO_TASK_NONE;
				break;

			case LOW_PRIO_TASK_ACCEL_CALIBRATION:

				do_accel_calibration(mavlink_fd);

				low_prio_task = LOW_PRIO_TASK_NONE;
				break;

			case LOW_PRIO_TASK_AIRSPEED_CALIBRATION:

				do_airspeed_calibration(mavlink_fd);

				low_prio_task = LOW_PRIO_TASK_NONE;
				break;

			case LOW_PRIO_TASK_NONE:
			default:
				/* slow down to 10Hz */
				usleep(100000);
				break;
		}

	}

	return 0;
}

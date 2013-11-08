/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *           Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 * @file commander.cpp
 * Main system state machine implementation.
 *
 */

#include <nuttx/config.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <systemlib/err.h>
#include <debug.h>
#include <sys/prctl.h>
#include <sys/stat.h>
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

#include <drivers/drv_led.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/cpuload.h>
#include <systemlib/rc_check.h>

#include "px4_custom_mode.h"
#include "commander_helper.h"
#include "state_machine_helper.h"
#include "calibration_routines.h"
#include "accelerometer_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "baro_calibration.h"
#include "rc_calibration.h"
#include "airspeed_calibration.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

extern struct system_load_s system_load;

#define LOW_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS 1000.0f
#define CRITICAL_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS 100.0f

/* Decouple update interval and hysteris counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 50000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))
#define LOW_VOLTAGE_BATTERY_COUNTER_LIMIT (LOW_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)
#define CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT (CRITICAL_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define MAVLINK_OPEN_INTERVAL 50000

#define STICK_ON_OFF_LIMIT 0.75f
#define STICK_THRUST_RANGE 1.0f
#define STICK_ON_OFF_HYSTERESIS_TIME_MS 1000
#define STICK_ON_OFF_COUNTER_LIMIT (STICK_ON_OFF_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define POSITION_TIMEOUT 1000000 /**< consider the local or global position estimate invalid after 1s */
#define RC_TIMEOUT 100000
#define DIFFPRESS_TIMEOUT 2000000

#define PRINT_INTERVAL	5000000
#define PRINT_MODE_REJECT_INTERVAL	2000000

enum MAV_MODE_FLAG {
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	MAV_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	MAV_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	MAV_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	MAV_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	MAV_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	MAV_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	MAV_MODE_FLAG_ENUM_END = 129, /*  | */
};

/* Mavlink file descriptors */
static int mavlink_fd;

/* flags */
static bool commander_initialized = false;
static volatile bool thread_should_exit = false;		/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

static unsigned int leds_counter;
/* To remember when last notification was sent */
static uint64_t last_print_mode_reject_time = 0;
/* if connected via USB */
static bool on_usb_power = false;

static float takeoff_alt = 5.0f;

/* tasks waiting for low prio thread */
typedef enum {
	LOW_PRIO_TASK_NONE = 0,
	LOW_PRIO_TASK_PARAM_SAVE,
	LOW_PRIO_TASK_PARAM_LOAD,
	LOW_PRIO_TASK_GYRO_CALIBRATION,
	LOW_PRIO_TASK_MAG_CALIBRATION,
	LOW_PRIO_TASK_ALTITUDE_CALIBRATION,
	LOW_PRIO_TASK_RC_CALIBRATION,
	LOW_PRIO_TASK_ACCEL_CALIBRATION,
	LOW_PRIO_TASK_AIRSPEED_CALIBRATION
} low_prio_task_t;

static low_prio_task_t low_prio_task = LOW_PRIO_TASK_NONE;

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 *
 * @ingroup apps
 */
extern "C" __EXPORT int commander_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
void usage(const char *reason);

/**
 * React to commands that are sent e.g. from the mavlink module.
 */
void handle_command(struct vehicle_status_s *status, struct vehicle_control_mode_s *control_mode, struct vehicle_command_s *cmd, struct actuator_armed_s *armed);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

void control_status_leds(vehicle_status_s *status, const actuator_armed_s *actuator_armed, bool changed);

void check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed);

void check_mode_switches(struct manual_control_setpoint_s *sp_man, struct vehicle_status_s *current_status);

transition_result_t check_main_state_machine(struct vehicle_status_s *current_status);

void print_reject_mode(const char *msg);

void print_reject_arm(const char *msg);

void print_status();

transition_result_t check_navigation_state_machine(struct vehicle_status_s *status, struct vehicle_control_mode_s *control_mode, struct vehicle_local_position_s *local_pos);

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);

void answer_command(struct vehicle_command_s &cmd, enum VEHICLE_CMD_RESULT result);


int commander_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("commander already running");
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

		while (!thread_running) {
			usleep(200);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running)
			errx(0, "commander already stopped");

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		warnx("terminated.");

		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\tcommander is running");
			print_status();

		} else {
			warnx("\tcommander not started");
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

void print_status()
{
	warnx("usb powered: %s", (on_usb_power) ? "yes" : "no");

	/* read all relevant states */
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	struct vehicle_status_s state;
	orb_copy(ORB_ID(vehicle_status), state_sub, &state);

	const char *armed_str;

	switch (state.arming_state) {
	case ARMING_STATE_INIT:
		armed_str = "INIT";
		break;

	case ARMING_STATE_STANDBY:
		armed_str = "STANDBY";
		break;

	case ARMING_STATE_ARMED:
		armed_str = "ARMED";
		break;

	case ARMING_STATE_ARMED_ERROR:
		armed_str = "ARMED_ERROR";
		break;

	case ARMING_STATE_STANDBY_ERROR:
		armed_str = "STANDBY_ERROR";
		break;

	case ARMING_STATE_REBOOT:
		armed_str = "REBOOT";
		break;

	case ARMING_STATE_IN_AIR_RESTORE:
		armed_str = "IN_AIR_RESTORE";
		break;

	default:
		armed_str = "ERR: UNKNOWN STATE";
		break;
	}

	close(state_sub);


	warnx("arming: %s", armed_str);
}

static orb_advert_t control_mode_pub;
static orb_advert_t status_pub;

void handle_command(struct vehicle_status_s *status, const struct safety_s *safety, struct vehicle_control_mode_s *control_mode, struct vehicle_command_s *cmd, struct actuator_armed_s *armed)
{
	/* result of the command */
	uint8_t result = VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* only handle high-priority commands here */

	/* request to set different system mode */
	switch (cmd->command) {
	case VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t) cmd->param1;
			uint8_t custom_main_mode = (uint8_t) cmd->param2;
			transition_result_t arming_res = TRANSITION_NOT_CHANGED;

			/* set HIL state */
			hil_state_t new_hil_state = (base_mode & MAV_MODE_FLAG_HIL_ENABLED) ? HIL_STATE_ON : HIL_STATE_OFF;
			int hil_ret = hil_state_transition(new_hil_state, status_pub, status, control_mode_pub, control_mode, mavlink_fd);

			/* if HIL got enabled, reset battery status state */
			if (hil_ret == OK && control_mode->flag_system_hil_enabled) {
				/* reset the arming mode to disarmed */
				arming_res = arming_state_transition(status, safety, control_mode, ARMING_STATE_STANDBY, armed);

				if (arming_res != TRANSITION_DENIED) {
					mavlink_log_info(mavlink_fd, "[cmd] HIL: Reset ARMED state to standby");

				} else {
					mavlink_log_info(mavlink_fd, "[cmd] HIL: FAILED resetting armed state");
				}
			}

			// TODO remove debug code
			//mavlink_log_critical(mavlink_fd, "#audio: command setmode: %d %d", base_mode, custom_main_mode);
			/* set arming state */
			arming_res = TRANSITION_NOT_CHANGED;

			if (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
				if ((safety->safety_switch_available && !safety->safety_off) && !control_mode->flag_system_hil_enabled) {
					print_reject_arm("NOT ARMING: Press safety switch first.");
					arming_res = TRANSITION_DENIED;

				} else {
					arming_res = arming_state_transition(status, safety, control_mode, ARMING_STATE_ARMED, armed);
				}

				if (arming_res == TRANSITION_CHANGED) {
					mavlink_log_info(mavlink_fd, "[cmd] ARMED by command");
				}

			} else {
				if (status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR) {
					arming_state_t new_arming_state = (status->arming_state == ARMING_STATE_ARMED ? ARMING_STATE_STANDBY : ARMING_STATE_STANDBY_ERROR);
					arming_res = arming_state_transition(status, safety, control_mode, new_arming_state, armed);

					if (arming_res == TRANSITION_CHANGED) {
						mavlink_log_info(mavlink_fd, "[cmd] DISARMED by command");
					}

				} else {
					arming_res = TRANSITION_NOT_CHANGED;
				}
			}

			/* set main state */
			transition_result_t main_res = TRANSITION_DENIED;

			if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					/* MANUAL */
					main_res = main_state_transition(status, MAIN_STATE_MANUAL);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_SEATBELT) {
					/* SEATBELT */
					main_res = main_state_transition(status, MAIN_STATE_SEATBELT);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_EASY) {
					/* EASY */
					main_res = main_state_transition(status, MAIN_STATE_EASY);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					/* AUTO */
					main_res = main_state_transition(status, MAIN_STATE_AUTO);
				}

			} else {
				/* use base mode */
				if (base_mode & MAV_MODE_FLAG_AUTO_ENABLED) {
					/* AUTO */
					main_res = main_state_transition(status, MAIN_STATE_AUTO);

				} else if (base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) {
						/* EASY */
						main_res = main_state_transition(status, MAIN_STATE_EASY);

					} else if (base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) {
						/* MANUAL */
						main_res = main_state_transition(status, MAIN_STATE_MANUAL);
					}
				}
			}

			if (arming_res != TRANSITION_DENIED && main_res != TRANSITION_DENIED) {
				result = VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

			break;
		}

	case VEHICLE_CMD_NAV_TAKEOFF: {
			if (armed->armed) {
				transition_result_t nav_res = navigation_state_transition(status, NAVIGATION_STATE_AUTO_TAKEOFF, control_mode);

				if (nav_res == TRANSITION_CHANGED) {
					mavlink_log_info(mavlink_fd, "[cmd] TAKEOFF on command");
				}

				if (nav_res != TRANSITION_DENIED) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

			} else {
				/* reject TAKEOFF not armed */
				result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

			break;
		}

	case VEHICLE_CMD_COMPONENT_ARM_DISARM: {
			transition_result_t arming_res = TRANSITION_NOT_CHANGED;

			if (!armed->armed && ((int)(cmd->param1 + 0.5f)) == 1) {
				if (safety->safety_switch_available && !safety->safety_off) {
					print_reject_arm("NOT ARMING: Press safety switch first.");
					arming_res = TRANSITION_DENIED;

				} else {
					arming_res = arming_state_transition(status, safety, control_mode, ARMING_STATE_ARMED, armed);
				}

				if (arming_res == TRANSITION_CHANGED) {
					mavlink_log_critical(mavlink_fd, "#audio: ARMED by component arm cmd");
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					mavlink_log_critical(mavlink_fd, "#audio: REJECTING component arm cmd");
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}
		}
		break;

	default:
		break;
	}

	/* supported command handling stop */
	if (result == VEHICLE_CMD_RESULT_ACCEPTED) {
		tune_positive();

	} else if (result == VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* we do not care in the high prio loop about commands we don't know */
	} else {
		tune_negative();

		if (result == VEHICLE_CMD_RESULT_DENIED) {
			mavlink_log_critical(mavlink_fd, "#audio: command denied: %u", cmd->command);

		} else if (result == VEHICLE_CMD_RESULT_FAILED) {
			mavlink_log_critical(mavlink_fd, "#audio: command failed: %u", cmd->command);

		} else if (result == VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {
			mavlink_log_critical(mavlink_fd, "#audio: command temporarily rejected: %u", cmd->command);

		}
	}

	/* send any requested ACKs */
	if (cmd->confirmation > 0 && result != VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* send acknowledge command */
		// XXX TODO
	}

}

static struct vehicle_status_s status;

/* armed topic */
static struct actuator_armed_s armed;

static struct safety_s safety;

/* flags for control apps */
struct vehicle_control_mode_s control_mode;

int commander_thread_main(int argc, char *argv[])
{
	/* not yet initialized */
	commander_initialized = false;
	bool home_position_set = false;

	bool battery_tune_played = false;
	bool arm_tune_played = false;

	/* set parameters */
	param_t _param_sys_type = param_find("MAV_TYPE");
	param_t _param_system_id = param_find("MAV_SYS_ID");
	param_t _param_component_id = param_find("MAV_COMP_ID");
	param_t _param_takeoff_alt = param_find("NAV_TAKEOFF_ALT");

	/* welcome user */
	warnx("starting");

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

	/* Main state machine */
	/* make sure we are in preflight state */
	memset(&status, 0, sizeof(status));
	status.condition_landed = true;	// initialize to safe value

	/* armed topic */
	orb_advert_t armed_pub;
	/* Initialize armed with all false */
	memset(&armed, 0, sizeof(armed));

	/* Initialize all flags to false */
	memset(&control_mode, 0, sizeof(control_mode));

	status.main_state = MAIN_STATE_MANUAL;
	status.navigation_state = NAVIGATION_STATE_DIRECT;
	status.arming_state = ARMING_STATE_INIT;
	status.hil_state = HIL_STATE_OFF;

	/* neither manual nor offboard control commands have been received */
	status.offboard_control_signal_found_once = false;
	status.rc_signal_found_once = false;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = true;
	status.offboard_control_signal_lost = true;

	/* allow manual override initially */
	control_mode.flag_external_manual_override_ok = true;

	/* set battery warning flag */
	status.battery_warning = VEHICLE_BATTERY_WARNING_NONE;
	status.condition_battery_voltage_valid = false;

	// XXX for now just set sensors as initialized
	status.condition_system_sensors_initialized = true;

	// XXX just disable offboard control for now
	control_mode.flag_control_offboard_enabled = false;

	/* advertise to ORB */
	status_pub = orb_advertise(ORB_ID(vehicle_status), &status);
	/* publish current state machine */

	/* publish initial state */
	status.counter++;
	status.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(vehicle_status), status_pub, &status);

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

	mavlink_log_info(mavlink_fd, "[cmd] started");

	int ret;

	pthread_attr_t commander_low_prio_attr;
	pthread_attr_init(&commander_low_prio_attr);
	pthread_attr_setstacksize(&commander_low_prio_attr, 2992);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&commander_low_prio_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
	(void)pthread_attr_setschedparam(&commander_low_prio_attr, &param);
	pthread_create(&commander_low_prio_thread, &commander_low_prio_attr, commander_low_prio_loop, NULL);
	pthread_attr_destroy(&commander_low_prio_attr);

	/* Start monitoring loop */
	unsigned counter = 0;
	unsigned low_voltage_counter = 0;
	unsigned critical_voltage_counter = 0;
	unsigned stick_off_counter = 0;
	unsigned stick_on_counter = 0;

	bool low_battery_voltage_actions_done = false;
	bool critical_battery_voltage_actions_done = false;

	uint64_t last_idle_time = 0;
	uint64_t start_time = 0;

	bool status_changed = true;
	bool param_init_forced = true;

	bool updated = false;

	bool rc_calibration_ok = (OK == rc_calibration_check(mavlink_fd));

	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	memset(&safety, 0, sizeof(safety));
	safety.safety_switch_available = false;
	safety.safety_off = false;

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

	/* Subscribe to local position data */
	int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	struct vehicle_local_position_s local_position;
	memset(&local_position, 0, sizeof(local_position));

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

	control_status_leds(&status, &armed, true);

	/* now initialized */
	commander_initialized = true;
	thread_running = true;

	start_time = hrt_absolute_time();

	while (!thread_should_exit) {

		if (mavlink_fd < 0 && counter % (1000000 / MAVLINK_OPEN_INTERVAL) == 0) {
			/* try to open the mavlink log device every once in a while */
			mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		/* update parameters */
		orb_check(param_changed_sub, &updated);

		if (updated || param_init_forced) {
			param_init_forced = false;
			/* parameters changed */
			orb_copy(ORB_ID(parameter_update), param_changed_sub, &param_changed);

			/* update parameters */
			if (!armed.armed) {
				if (param_get(_param_sys_type, &(status.system_type)) != OK) {
					warnx("failed getting new system type");
				}

				/* disable manual override for all systems that rely on electronic stabilization */
				if (status.system_type == VEHICLE_TYPE_COAXIAL ||
				    status.system_type == VEHICLE_TYPE_HELICOPTER ||
				    status.system_type == VEHICLE_TYPE_TRICOPTER ||
				    status.system_type == VEHICLE_TYPE_QUADROTOR ||
				    status.system_type == VEHICLE_TYPE_HEXAROTOR ||
				    status.system_type == VEHICLE_TYPE_OCTOROTOR) {
					control_mode.flag_external_manual_override_ok = false;
					status.is_rotary_wing = true;

				} else {
					control_mode.flag_external_manual_override_ok = true;
					status.is_rotary_wing = false;
				}

				/* check and update system / component ID */
				param_get(_param_system_id, &(status.system_id));
				param_get(_param_component_id, &(status.component_id));
				status_changed = true;

				/* re-check RC calibration */
				rc_calibration_ok = (OK == rc_calibration_check(mavlink_fd));

				/* navigation parameters */
				param_get(_param_takeoff_alt, &takeoff_alt);
			}
		}

		orb_check(sp_man_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
		}

		orb_check(sp_offboard_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(offboard_control_setpoint), sp_offboard_sub, &sp_offboard);
		}

		orb_check(sensor_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);
		}

		orb_check(diff_pres_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);
		}

		check_valid(diff_pres.timestamp, DIFFPRESS_TIMEOUT, true, &(status.condition_airspeed_valid), &status_changed);

		/* update safety topic */
		orb_check(safety_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(safety), safety_sub, &safety);

			// XXX this would be the right approach to do it, but do we *WANT* this?
			// /* disarm if safety is now on and still armed */
			// if (safety.safety_switch_available && !safety.safety_off) {
			// 	(void)arming_state_transition(&status, &safety, &control_mode, ARMING_STATE_STANDBY, &armed);
			// }
		}

		/* update global position estimate */
		orb_check(global_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
		}

		/* update condition_global_position_valid */
		check_valid(global_position.timestamp, POSITION_TIMEOUT, global_position.valid, &(status.condition_global_position_valid), &status_changed);

		/* update local position estimate */
		orb_check(local_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
		}

		/* update condition_local_position_valid and condition_local_altitude_valid */
		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.xy_valid, &(status.condition_local_position_valid), &status_changed);
		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.z_valid, &(status.condition_local_altitude_valid), &status_changed);

		if (status.condition_local_altitude_valid) {
			if (status.condition_landed != local_position.landed) {
				status.condition_landed = local_position.landed;
				status_changed = true;

				if (status.condition_landed) {
					mavlink_log_critical(mavlink_fd, "#audio: LANDED");

				} else {
					mavlink_log_critical(mavlink_fd, "#audio: IN AIR");
				}
			}
		}

		/* update battery status */
		orb_check(battery_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);

			// warnx("bat v: %2.2f", battery.voltage_v);

			/* only consider battery voltage if system has been running 2s and battery voltage is higher than 4V */
			if (hrt_absolute_time() > start_time + 2000000 && battery.voltage_v > 4.0f) {
				status.battery_voltage = battery.voltage_v;
				status.condition_battery_voltage_valid = true;
				status.battery_remaining = battery_remaining_estimate_voltage(status.battery_voltage);
			}
		}

		/* update subsystem */
		orb_check(subsys_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(subsystem_info), subsys_sub, &info);

			warnx("subsystem changed: %d\n", (int)info.subsystem_type);

			/* mark / unmark as present */
			if (info.present) {
				status.onboard_control_sensors_present |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_present &= ~info.subsystem_type;
			}

			/* mark / unmark as enabled */
			if (info.enabled) {
				status.onboard_control_sensors_enabled |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_enabled &= ~info.subsystem_type;
			}

			/* mark / unmark as ok */
			if (info.ok) {
				status.onboard_control_sensors_health |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_health &= ~info.subsystem_type;
			}

			status_changed = true;
		}

		if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
			/* compute system load */
			uint64_t interval_runtime = system_load.tasks[0].total_runtime - last_idle_time;

			if (last_idle_time > 0)
				status.load = 1.0f - ((float)interval_runtime / 1e6f);	//system load is time spent in non-idle

			last_idle_time = system_load.tasks[0].total_runtime;

			/* check if board is connected via USB */
			//struct stat statbuf;
			//on_usb_power = (stat("/dev/ttyACM0", &statbuf) == 0);
		}

		// XXX remove later
		//warnx("bat remaining: %2.2f", status.battery_remaining);

		/* if battery voltage is getting lower, warn using buzzer, etc. */
		if (status.condition_battery_voltage_valid && status.battery_remaining < 0.25f && !low_battery_voltage_actions_done) {
			//TODO: add filter, or call emergency after n measurements < VOLTAGE_BATTERY_MINIMAL_MILLIVOLTS
			if (low_voltage_counter > LOW_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				low_battery_voltage_actions_done = true;
				mavlink_log_critical(mavlink_fd, "#audio: WARNING: LOW BATTERY");
				status.battery_warning = VEHICLE_BATTERY_WARNING_LOW;
				status_changed = true;
				battery_tune_played = false;
			}

			low_voltage_counter++;

		} else if (status.condition_battery_voltage_valid && status.battery_remaining < 0.1f && !critical_battery_voltage_actions_done && low_battery_voltage_actions_done) {
			/* critical battery voltage, this is rather an emergency, change state machine */
			if (critical_voltage_counter > CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				critical_battery_voltage_actions_done = true;
				mavlink_log_critical(mavlink_fd, "#audio: EMERGENCY: CRITICAL BATTERY");
				status.battery_warning = VEHICLE_BATTERY_WARNING_CRITICAL;
				battery_tune_played = false;

				if (armed.armed) {
					arming_state_transition(&status, &safety, &control_mode, ARMING_STATE_ARMED_ERROR, &armed);

				} else {
					arming_state_transition(&status, &safety, &control_mode, ARMING_STATE_STANDBY_ERROR, &armed);
				}

				status_changed = true;
			}

			critical_voltage_counter++;

		} else {

			low_voltage_counter = 0;
			critical_voltage_counter = 0;
		}

		/* End battery voltage check */

		/* If in INIT state, try to proceed to STANDBY state */
		if (status.arming_state == ARMING_STATE_INIT && low_prio_task == LOW_PRIO_TASK_NONE) {
			// XXX check for sensors
			arming_state_transition(&status, &safety, &control_mode, ARMING_STATE_STANDBY, &armed);

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

		orb_check(gps_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_position);
			/* check if GPS fix is ok */
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

			if (!home_position_set && gps_position.fix_type >= 3 &&
			    (gps_position.eph_m < hdop_threshold_m) && (gps_position.epv_m < vdop_threshold_m) &&	// XXX note that vdop is 0 for mtk
			    (hrt_absolute_time() < gps_position.timestamp_position + POSITION_TIMEOUT) && !armed.armed) {
				/* copy position data to uORB home message, store it locally as well */
				// TODO use global position estimate
				home.lat = gps_position.lat;
				home.lon = gps_position.lon;
				home.alt = gps_position.alt;

				home.eph_m = gps_position.eph_m;
				home.epv_m = gps_position.epv_m;

				home.s_variance_m_s = gps_position.s_variance_m_s;
				home.p_variance_m = gps_position.p_variance_m;

				double home_lat_d = home.lat * 1e-7;
				double home_lon_d = home.lon * 1e-7;
				warnx("home: lat = %.7f, lon = %.7f", home_lat_d, home_lon_d);
				mavlink_log_info(mavlink_fd, "[cmd] home: %.7f, %.7f", home_lat_d, home_lon_d);

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
		if (!status.offboard_control_signal_found_once && sp_man.timestamp != 0) {
			/* start RC input check */
			if (hrt_absolute_time() < sp_man.timestamp + RC_TIMEOUT) {
				/* handle the case where RC signal was regained */
				if (!status.rc_signal_found_once) {
					status.rc_signal_found_once = true;
					mavlink_log_critical(mavlink_fd, "#audio: detected RC signal first time");
					status_changed = true;

				} else {
					if (status.rc_signal_lost) {
						mavlink_log_critical(mavlink_fd, "#audio: RC signal regained");
						status_changed = true;
					}
				}

				status.rc_signal_lost = false;

				transition_result_t res;	// store all transitions results here

				/* arm/disarm by RC */
				res = TRANSITION_NOT_CHANGED;

				/* check if left stick is in lower left position and we are in MANUAL or AUTO_READY mode or (ASSISTED mode and landed) -> disarm
				 * do it only for rotary wings */
				if (status.is_rotary_wing &&
				    (status.arming_state == ARMING_STATE_ARMED || status.arming_state == ARMING_STATE_ARMED_ERROR) &&
				    (status.main_state == MAIN_STATE_MANUAL || status.navigation_state == NAVIGATION_STATE_AUTO_READY ||
				     (status.condition_landed && (
					      status.navigation_state == NAVIGATION_STATE_ALTHOLD ||
					      status.navigation_state == NAVIGATION_STATE_VECTOR
				      ))) && sp_man.yaw < -STICK_ON_OFF_LIMIT && sp_man.throttle < STICK_THRUST_RANGE * 0.1f) {
					if (stick_off_counter > STICK_ON_OFF_COUNTER_LIMIT) {
						/* disarm to STANDBY if ARMED or to STANDBY_ERROR if ARMED_ERROR */
						arming_state_t new_arming_state = (status.arming_state == ARMING_STATE_ARMED ? ARMING_STATE_STANDBY : ARMING_STATE_STANDBY_ERROR);
						res = arming_state_transition(&status, &safety, &control_mode, new_arming_state, &armed);
						stick_off_counter = 0;

					} else {
						stick_off_counter++;
					}

				} else {
					stick_off_counter = 0;
				}

				/* check if left stick is in lower right position and we're in MANUAL mode -> arm */
				if (status.arming_state == ARMING_STATE_STANDBY &&
				    sp_man.yaw > STICK_ON_OFF_LIMIT && sp_man.throttle < STICK_THRUST_RANGE * 0.1f) {
					if (stick_on_counter > STICK_ON_OFF_COUNTER_LIMIT) {
						if (safety.safety_switch_available && !safety.safety_off) {
							print_reject_arm("NOT ARMING: Press safety switch first.");

						} else if (status.main_state != MAIN_STATE_MANUAL) {
							print_reject_arm("NOT ARMING: Switch to MANUAL mode first.");

						} else {
							res = arming_state_transition(&status, &safety, &control_mode, ARMING_STATE_ARMED, &armed);
						}

						stick_on_counter = 0;

					} else {
						stick_on_counter++;
					}

				} else {
					stick_on_counter = 0;
				}

				if (res == TRANSITION_CHANGED) {
					if (status.arming_state == ARMING_STATE_ARMED) {
						mavlink_log_info(mavlink_fd, "[cmd] ARMED by RC");

					} else {
						mavlink_log_info(mavlink_fd, "[cmd] DISARMED by RC");
					}

				} else if (res == TRANSITION_DENIED) {
					warnx("ERROR: main denied: arm %d main %d mode_sw %d", status.arming_state, status.main_state, status.mode_switch);
					mavlink_log_critical(mavlink_fd, "#audio: ERROR: main denied: arm %d main %d mode_sw %d", status.arming_state, status.main_state, status.mode_switch);
				}

				/* fill current_status according to mode switches */
				check_mode_switches(&sp_man, &status);

				/* evaluate the main state machine */
				res = check_main_state_machine(&status);

				if (res == TRANSITION_CHANGED) {
					//mavlink_log_info(mavlink_fd, "[cmd] main state: %d", status.main_state);
					tune_positive();

				} else if (res == TRANSITION_DENIED) {
					/* DENIED here indicates bug in the commander */
					warnx("ERROR: main denied: arm %d main %d mode_sw %d", status.arming_state, status.main_state, status.mode_switch);
					mavlink_log_critical(mavlink_fd, "#audio: ERROR: main denied: arm %d main %d mode_sw %d", status.arming_state, status.main_state, status.mode_switch);
				}

			} else {
				if (!status.rc_signal_lost) {
					mavlink_log_critical(mavlink_fd, "#audio: CRITICAL: RC SIGNAL LOST");
					status.rc_signal_lost = true;
					status_changed = true;
				}
			}
		}


		/* handle commands last, as the system needs to be updated to handle them */
		orb_check(cmd_sub, &updated);

		if (updated) {
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			handle_command(&status, &safety, &control_mode, &cmd, &armed);
		}

		/* evaluate the navigation state machine */
		transition_result_t res = check_navigation_state_machine(&status, &control_mode, &local_position);

		if (res == TRANSITION_DENIED) {
			/* DENIED here indicates bug in the commander */
			warnx("ERROR: nav denied: arm %d main %d nav %d", status.arming_state, status.main_state, status.navigation_state);
			mavlink_log_critical(mavlink_fd, "#audio: ERROR: nav denied: arm %d main %d nav %d", status.arming_state, status.main_state, status.navigation_state);
		}

		/* check which state machines for changes, clear "changed" flag */
		bool arming_state_changed = check_arming_state_changed();
		bool main_state_changed = check_main_state_changed();
		bool navigation_state_changed = check_navigation_state_changed();

		hrt_abstime t1 = hrt_absolute_time();

		if (navigation_state_changed || arming_state_changed) {
			control_mode.flag_armed = armed.armed;	// copy armed state to vehicle_control_mode topic
		}

		if (arming_state_changed || main_state_changed || navigation_state_changed) {
			mavlink_log_info(mavlink_fd, "[cmd] state: arm %d, main %d, nav %d", status.arming_state, status.main_state, status.navigation_state);
			status_changed = true;
		}

		/* publish states (armed, control mode, vehicle status) at least with 5 Hz */
		if (counter % (200000 / COMMANDER_MONITORING_INTERVAL) == 0 || status_changed) {
			status.timestamp = t1;
			orb_publish(ORB_ID(vehicle_status), status_pub, &status);
			control_mode.timestamp = t1;
			orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, &control_mode);
			armed.timestamp = t1;
			orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
		}

		/* play arming and battery warning tunes */
		if (!arm_tune_played && armed.armed && (!safety.safety_switch_available || (safety.safety_switch_available && safety.safety_off))) {
			/* play tune when armed */
			if (tune_arm() == OK)
				arm_tune_played = true;

		} else if (status.battery_warning == VEHICLE_BATTERY_WARNING_LOW) {
			/* play tune on battery warning */
			if (tune_low_bat() == OK)
				battery_tune_played = true;

		} else if (status.battery_warning == VEHICLE_BATTERY_WARNING_CRITICAL) {
			/* play tune on battery critical */
			if (tune_critical_bat() == OK)
				battery_tune_played = true;

		} else if (battery_tune_played) {
			tune_stop();
			battery_tune_played = false;
		}

		/* reset arm_tune_played when disarmed */
		if (status.arming_state != ARMING_STATE_ARMED || (safety.safety_switch_available && !safety.safety_off)) {
			arm_tune_played = false;
		}

		fflush(stdout);
		counter++;

		int blink_state = blink_msg_state();

		if (blink_state > 0) {
			/* blinking LED message, don't touch LEDs */
			if (blink_state == 2) {
				/* blinking LED message completed, restore normal state */
				control_status_leds(&status, &armed, true);
			}

		} else {
			/* normal state */
			control_status_leds(&status, &armed, status_changed);
		}

		status_changed = false;

		usleep(COMMANDER_MONITORING_INTERVAL);
	}

	/* wait for threads to complete */
	ret = pthread_join(commander_low_prio_thread, NULL);

	if (ret) {
		warn("join failed: %d", ret);
	}

	rgbled_set_mode(RGBLED_MODE_OFF);

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

	thread_running = false;

	return 0;
}

void
check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed)
{
	hrt_abstime t = hrt_absolute_time();
	bool valid_new = (t < timestamp + timeout && t > timeout && valid_in);

	if (*valid_out != valid_new) {
		*valid_out = valid_new;
		*changed = true;
	}
}

void
control_status_leds(vehicle_status_s *status, const actuator_armed_s *actuator_armed, bool changed)
{
	/* driving rgbled */
	if (changed) {
		bool set_normal_color = false;

		/* set mode */
		if (status->arming_state == ARMING_STATE_ARMED) {
			rgbled_set_mode(RGBLED_MODE_ON);
			set_normal_color = true;

		} else if (status->arming_state == ARMING_STATE_ARMED_ERROR) {
			rgbled_set_mode(RGBLED_MODE_BLINK_FAST);
			rgbled_set_color(RGBLED_COLOR_RED);

		} else if (status->arming_state == ARMING_STATE_STANDBY) {
			rgbled_set_mode(RGBLED_MODE_BREATHE);
			set_normal_color = true;

		} else {	// STANDBY_ERROR and other states
			rgbled_set_mode(RGBLED_MODE_BLINK_NORMAL);
			rgbled_set_color(RGBLED_COLOR_RED);
		}

		if (set_normal_color) {
			/* set color */
			if (status->battery_warning != VEHICLE_BATTERY_WARNING_NONE) {
				if (status->battery_warning == VEHICLE_BATTERY_WARNING_LOW) {
					rgbled_set_color(RGBLED_COLOR_AMBER);
				}

				/* VEHICLE_BATTERY_WARNING_CRITICAL handled as ARMING_STATE_ARMED_ERROR / ARMING_STATE_STANDBY_ERROR */

			} else {
				if (status->condition_local_position_valid) {
					rgbled_set_color(RGBLED_COLOR_GREEN);

				} else {
					rgbled_set_color(RGBLED_COLOR_BLUE);
				}
			}
		}
	}

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1

	/* this runs at around 20Hz, full cycle is 16 ticks = 10/16Hz */
	if (actuator_armed->armed) {
		/* armed, solid */
		led_on(LED_BLUE);

	} else if (actuator_armed->ready_to_arm) {
		/* ready to arm, blink at 1Hz */
		if (leds_counter % 20 == 0)
			led_toggle(LED_BLUE);

	} else {
		/* not ready to arm, blink at 10Hz */
		if (leds_counter % 2 == 0)
			led_toggle(LED_BLUE);
	}

#endif

	/* give system warnings on error LED, XXX maybe add memory usage warning too */
	if (status->load > 0.95f) {
		if (leds_counter % 2 == 0)
			led_toggle(LED_AMBER);

	} else {
		led_off(LED_AMBER);
	}

	leds_counter++;
}

void
check_mode_switches(struct manual_control_setpoint_s *sp_man, struct vehicle_status_s *current_status)
{
	/* main mode switch */
	if (!isfinite(sp_man->mode_switch)) {
		warnx("mode sw not finite");
		current_status->mode_switch = MODE_SWITCH_MANUAL;

	} else if (sp_man->mode_switch > STICK_ON_OFF_LIMIT) {
		current_status->mode_switch = MODE_SWITCH_AUTO;

	} else if (sp_man->mode_switch < -STICK_ON_OFF_LIMIT) {
		current_status->mode_switch = MODE_SWITCH_MANUAL;

	} else {
		current_status->mode_switch = MODE_SWITCH_ASSISTED;
	}

	/* land switch */
	if (!isfinite(sp_man->return_switch)) {
		current_status->return_switch = RETURN_SWITCH_NONE;

	} else if (sp_man->return_switch > STICK_ON_OFF_LIMIT) {
		current_status->return_switch = RETURN_SWITCH_RETURN;

	} else {
		current_status->return_switch = RETURN_SWITCH_NONE;
	}

	/* assisted switch */
	if (!isfinite(sp_man->assisted_switch)) {
		current_status->assisted_switch = ASSISTED_SWITCH_SEATBELT;

	} else if (sp_man->assisted_switch > STICK_ON_OFF_LIMIT) {
		current_status->assisted_switch = ASSISTED_SWITCH_EASY;

	} else {
		current_status->assisted_switch = ASSISTED_SWITCH_SEATBELT;
	}

	/* mission switch  */
	if (!isfinite(sp_man->mission_switch)) {
		current_status->mission_switch = MISSION_SWITCH_MISSION;

	} else if (sp_man->mission_switch > STICK_ON_OFF_LIMIT) {
		current_status->mission_switch = MISSION_SWITCH_NONE;

	} else {
		current_status->mission_switch = MISSION_SWITCH_MISSION;
	}
}

transition_result_t
check_main_state_machine(struct vehicle_status_s *current_status)
{
	/* evaluate the main state machine */
	transition_result_t res = TRANSITION_DENIED;

	switch (current_status->mode_switch) {
	case MODE_SWITCH_MANUAL:
		res = main_state_transition(current_status, MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	case MODE_SWITCH_ASSISTED:
		if (current_status->assisted_switch == ASSISTED_SWITCH_EASY) {
			res = main_state_transition(current_status, MAIN_STATE_EASY);

			if (res != TRANSITION_DENIED)
				break;	// changed successfully or already in this state

			// else fallback to SEATBELT
			print_reject_mode("EASY");
		}

		res = main_state_transition(current_status, MAIN_STATE_SEATBELT);

		if (res != TRANSITION_DENIED)
			break;	// changed successfully or already in this mode

		if (current_status->assisted_switch != ASSISTED_SWITCH_EASY)	// don't print both messages
			print_reject_mode("SEATBELT");

		// else fallback to MANUAL
		res = main_state_transition(current_status, MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	case MODE_SWITCH_AUTO:
		res = main_state_transition(current_status, MAIN_STATE_AUTO);

		if (res != TRANSITION_DENIED)
			break;	// changed successfully or already in this state

		// else fallback to SEATBELT (EASY likely will not work too)
		print_reject_mode("AUTO");
		res = main_state_transition(current_status, MAIN_STATE_SEATBELT);

		if (res != TRANSITION_DENIED)
			break;	// changed successfully or already in this state

		// else fallback to MANUAL
		res = main_state_transition(current_status, MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	default:
		break;
	}

	return res;
}

void
print_reject_mode(const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		char s[80];
		sprintf(s, "#audio: warning: reject %s", msg);
		mavlink_log_critical(mavlink_fd, s);
		tune_negative();
	}
}

void
print_reject_arm(const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		char s[80];
		sprintf(s, "#audio: %s", msg);
		mavlink_log_critical(mavlink_fd, s);
		tune_negative();
	}
}

transition_result_t
check_navigation_state_machine(struct vehicle_status_s *status, struct vehicle_control_mode_s *control_mode, struct vehicle_local_position_s *local_pos)
{
	transition_result_t res = TRANSITION_DENIED;

	if (status->main_state == MAIN_STATE_AUTO) {
		if (status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR) {
			// TODO AUTO_LAND handling
			if (status->navigation_state == NAVIGATION_STATE_AUTO_TAKEOFF) {
				/* don't switch to other states until takeoff not completed */
				if (local_pos->z > -takeoff_alt || status->condition_landed) {
					return TRANSITION_NOT_CHANGED;
				}
			}

			if (status->navigation_state != NAVIGATION_STATE_AUTO_TAKEOFF &&
			    status->navigation_state != NAVIGATION_STATE_AUTO_LOITER &&
			    status->navigation_state != NAVIGATION_STATE_AUTO_MISSION &&
			    status->navigation_state != NAVIGATION_STATE_AUTO_RTL) {
				/* possibly on ground, switch to TAKEOFF if needed */
				if (local_pos->z > -takeoff_alt || status->condition_landed) {
					res = navigation_state_transition(status, NAVIGATION_STATE_AUTO_TAKEOFF, control_mode);
					return res;
				}
			}

			/* switch to AUTO mode */
			if (status->rc_signal_found_once && !status->rc_signal_lost) {
				/* act depending on switches when manual control enabled */
				if (status->return_switch == RETURN_SWITCH_RETURN) {
					/* RTL */
					res = navigation_state_transition(status, NAVIGATION_STATE_AUTO_RTL, control_mode);

				} else {
					if (status->mission_switch == MISSION_SWITCH_MISSION) {
						/* MISSION */
						res = navigation_state_transition(status, NAVIGATION_STATE_AUTO_MISSION, control_mode);

					} else {
						/* LOITER */
						res = navigation_state_transition(status, NAVIGATION_STATE_AUTO_LOITER, control_mode);
					}
				}

			} else {
				/* switch to MISSION when no RC control and first time in some AUTO mode  */
				if (status->navigation_state == NAVIGATION_STATE_AUTO_LOITER ||
				    status->navigation_state == NAVIGATION_STATE_AUTO_MISSION ||
				    status->navigation_state == NAVIGATION_STATE_AUTO_RTL ||
				    status->navigation_state == NAVIGATION_STATE_AUTO_LAND) {
					res = TRANSITION_NOT_CHANGED;

				} else {
					res = navigation_state_transition(status, NAVIGATION_STATE_AUTO_MISSION, control_mode);
				}
			}

		} else {
			/* disarmed, always switch to AUTO_READY */
			res = navigation_state_transition(status, NAVIGATION_STATE_AUTO_READY, control_mode);
		}

	} else {
		/* manual control modes */
		if (status->rc_signal_lost && (status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR)) {
			/* switch to failsafe mode */
			bool manual_control_old = control_mode->flag_control_manual_enabled;

			if (!status->condition_landed) {
				/* in air: try to hold position */
				res = navigation_state_transition(status, NAVIGATION_STATE_VECTOR, control_mode);

			} else {
				/* landed: don't try to hold position but land (if taking off) */
				res = TRANSITION_DENIED;
			}

			if (res == TRANSITION_DENIED) {
				res = navigation_state_transition(status, NAVIGATION_STATE_ALTHOLD, control_mode);
			}

			control_mode->flag_control_manual_enabled = false;

			if (res == TRANSITION_NOT_CHANGED && manual_control_old) {
				/* mark navigation state as changed to force immediate flag publishing */
				set_navigation_state_changed();
				res = TRANSITION_CHANGED;
			}

			if (res == TRANSITION_CHANGED) {
				if (control_mode->flag_control_position_enabled) {
					mavlink_log_critical(mavlink_fd, "#audio: FAILSAFE: POS HOLD");

				} else {
					if (status->condition_landed) {
						mavlink_log_critical(mavlink_fd, "#audio: FAILSAFE: ALT HOLD (LAND)");

					} else {
						mavlink_log_critical(mavlink_fd, "#audio: FAILSAFE: ALT HOLD");
					}
				}
			}

		} else {
			switch (status->main_state) {
			case MAIN_STATE_MANUAL:
				res = navigation_state_transition(status, status->is_rotary_wing ? NAVIGATION_STATE_STABILIZE : NAVIGATION_STATE_DIRECT, control_mode);
				break;

			case MAIN_STATE_SEATBELT:
				res = navigation_state_transition(status, NAVIGATION_STATE_ALTHOLD, control_mode);
				break;

			case MAIN_STATE_EASY:
				res = navigation_state_transition(status, NAVIGATION_STATE_VECTOR, control_mode);
				break;

			default:
				break;
			}
		}
	}

	return res;
}

void answer_command(struct vehicle_command_s &cmd, enum VEHICLE_CMD_RESULT result)
{
	switch (result) {
	case VEHICLE_CMD_RESULT_ACCEPTED:
			tune_positive();
		break;

	case VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(mavlink_fd, "#audio: command denied: %u", cmd.command);
		tune_negative();
		break;

	case VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(mavlink_fd, "#audio: command failed: %u", cmd.command);
		tune_negative();
		break;

	case VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(mavlink_fd, "#audio: command temporarily rejected: %u", cmd.command);
		tune_negative();
		break;

	case VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(mavlink_fd, "#audio: command unsupported: %u", cmd.command);
		tune_negative();
		break;

	default:
		break;
	}
}

void *commander_low_prio_loop(void *arg)
{
	/* Set thread name */
	prctl(PR_SET_NAME, "commander_low_prio", getpid());

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		/* wait for up to 200ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 200);

		/* timed out - periodic check for thread_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* if we reach here, we have a valid command */
		orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

		/* ignore commands the high-prio loop handles */
		if (cmd.command == VEHICLE_CMD_DO_SET_MODE ||
		    cmd.command == VEHICLE_CMD_COMPONENT_ARM_DISARM ||
		    cmd.command == VEHICLE_CMD_NAV_TAKEOFF)
			continue;

		/* only handle low-priority commands here */
		switch (cmd.command) {

		case VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
			if (is_safe(&status, &safety, &armed)) {

				if (((int)(cmd.param1)) == 1) {
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					usleep(100000);
					/* reboot */
					systemreset(false);

				} else if (((int)(cmd.param1)) == 3) {
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					usleep(100000);
					/* reboot to bootloader */
					systemreset(true);

				} else {
					answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
				}

			} else {
				answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
			}

			break;

		case VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

				int calib_ret = ERROR;

				/* try to go to INIT/PREFLIGHT arming state */

				// XXX disable interrupts in arming_state_transition
				if (TRANSITION_DENIED == arming_state_transition(&status, &safety, &control_mode, ARMING_STATE_INIT, &armed)) {
					answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
					break;
				}

				if ((int)(cmd.param1) == 1) {
					/* gyro calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_gyro_calibration(mavlink_fd);

				} else if ((int)(cmd.param2) == 1) {
					/* magnetometer calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_mag_calibration(mavlink_fd);

				} else if ((int)(cmd.param3) == 1) {
					/* zero-altitude pressure calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);

				} else if ((int)(cmd.param4) == 1) {
					/* RC calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_rc_calibration(mavlink_fd);

				} else if ((int)(cmd.param5) == 1) {
					/* accelerometer calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_accel_calibration(mavlink_fd);

				} else if ((int)(cmd.param6) == 1) {
					/* airspeed calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_airspeed_calibration(mavlink_fd);
				}

				if (calib_ret == OK)
					tune_positive();
				else
					tune_negative();

				arming_state_transition(&status, &safety, &control_mode, ARMING_STATE_STANDBY, &armed);

				break;
			}

		case VEHICLE_CMD_PREFLIGHT_STORAGE: {

				if (((int)(cmd.param1)) == 0) {
					int ret = param_load_default();

					if (ret == OK) {
						mavlink_log_info(mavlink_fd, "[cmd] parameters loaded");
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);

					} else {
						mavlink_log_critical(mavlink_fd, "#audio: parameters load ERROR");

						/* convenience as many parts of NuttX use negative errno */
						if (ret < 0)
							ret = -ret;

						if (ret < 1000)
							mavlink_log_critical(mavlink_fd, "#audio: %s", strerror(ret));

						answer_command(cmd, VEHICLE_CMD_RESULT_FAILED);
					}

				} else if (((int)(cmd.param1)) == 1) {
					int ret = param_save_default();

					if (ret == OK) {
						mavlink_log_info(mavlink_fd, "[cmd] parameters saved");
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);

					} else {
						mavlink_log_critical(mavlink_fd, "#audio: parameters save error");

						/* convenience as many parts of NuttX use negative errno */
						if (ret < 0)
							ret = -ret;

						if (ret < 1000)
							mavlink_log_critical(mavlink_fd, "#audio: %s", strerror(ret));

						answer_command(cmd, VEHICLE_CMD_RESULT_FAILED);
					}
				}

				break;
			}

		default:
			answer_command(cmd, VEHICLE_CMD_RESULT_UNSUPPORTED);
			break;
		}

		/* send any requested ACKs */
		if (cmd.confirmation > 0 && cmd.command != VEHICLE_CMD_DO_SET_MODE
		    && cmd.command != VEHICLE_CMD_COMPONENT_ARM_DISARM) {
			/* send acknowledge command */
			// XXX TODO
		}
	}

	close(cmd_sub);

	return NULL;
}

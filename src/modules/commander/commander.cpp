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

/* Decouple update interval and hysteris counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 50000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))

#define MAVLINK_OPEN_INTERVAL 50000

#define STICK_ON_OFF_LIMIT 0.75f
#define STICK_THRUST_RANGE 1.0f
#define STICK_ON_OFF_HYSTERESIS_TIME_MS 1000
#define STICK_ON_OFF_COUNTER_LIMIT (STICK_ON_OFF_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define POSITION_TIMEOUT 20000 /**< consider the local or global position estimate invalid after 20ms */
#define RC_TIMEOUT 100000
#define RC_TIMEOUT_HIL 500000
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
static int parachute_enabled = 0;

static struct vehicle_status_s status;
static struct actuator_armed_s armed;
static struct safety_s safety;
static struct vehicle_control_mode_s control_mode;

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
bool handle_command(struct vehicle_status_s *status, struct vehicle_command_s *cmd, struct actuator_armed_s *armed);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

void control_status_leds(vehicle_status_s *status, const actuator_armed_s *actuator_armed, bool changed);

void check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed);

void check_mode_switches(struct manual_control_setpoint_s *sp_man, struct vehicle_status_s *status);

transition_result_t set_main_state_rc(struct vehicle_status_s *status);

void set_control_mode();

void print_reject_mode(struct vehicle_status_s *current_status, const char *msg);

void print_reject_arm(const char *msg);

void print_status();

int arm();
int disarm();

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

	if (!strcmp(argv[1], "arm")) {
		arm();
		exit(0);
	}

	if (!strcmp(argv[1], "disarm")) {
		disarm();
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

static orb_advert_t status_pub;

int arm()
{
	int arming_res = arming_state_transition(&status, &safety, ARMING_STATE_ARMED, &armed);

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(mavlink_fd, "[cmd] ARMED by commandline");
		return 0;

	} else {
		return 1;
	}
}

int disarm()
{
	int arming_res = arming_state_transition(&status, &safety, ARMING_STATE_STANDBY, &armed);

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(mavlink_fd, "[cmd] ARMED by commandline");
		return 0;

	} else {
		return 1;
	}
}

bool handle_command(struct vehicle_status_s *status, const struct safety_s *safety, struct vehicle_command_s *cmd, struct actuator_armed_s *armed)
{
	/* result of the command */
	enum VEHICLE_CMD_RESULT result = VEHICLE_CMD_RESULT_UNSUPPORTED;
	bool ret = false;

	/* only handle high-priority commands here */

	/* request to set different system mode */
	switch (cmd->command) {
	case VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t) cmd->param1;
			uint8_t custom_main_mode = (uint8_t) cmd->param2;
			transition_result_t arming_res = TRANSITION_NOT_CHANGED;

			/* set HIL state */
			hil_state_t new_hil_state = (base_mode & MAV_MODE_FLAG_HIL_ENABLED) ? HIL_STATE_ON : HIL_STATE_OFF;
			int hil_ret = hil_state_transition(new_hil_state, status_pub, status, mavlink_fd);

			/* if HIL got enabled, reset battery status state */
			if (hil_ret == OK && status->hil_state == HIL_STATE_ON) {
				/* reset the arming mode to disarmed */
				arming_res = arming_state_transition(status, safety, ARMING_STATE_STANDBY, armed);

				if (arming_res != TRANSITION_DENIED) {
					mavlink_log_info(mavlink_fd, "[cmd] HIL: Reset ARMED state to standby");

				} else {
					mavlink_log_info(mavlink_fd, "[cmd] HIL: FAILED resetting armed state");
				}
			}

			if (hil_ret == OK)
				ret = true;

			// TODO remove debug code
			//mavlink_log_critical(mavlink_fd, "#audio: command setmode: %d %d", base_mode, custom_main_mode);
			/* set arming state */
			arming_res = TRANSITION_NOT_CHANGED;

			if (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
				if (safety->safety_switch_available && !safety->safety_off && status->hil_state == HIL_STATE_OFF) {
					print_reject_arm("NOT ARMING: Press safety switch first.");
					arming_res = TRANSITION_DENIED;

				} else {
					arming_res = arming_state_transition(status, safety, ARMING_STATE_ARMED, armed);
				}

				if (arming_res == TRANSITION_CHANGED) {
					mavlink_log_info(mavlink_fd, "[cmd] ARMED by command");
				}

			} else {
				if (status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR) {
					arming_state_t new_arming_state = (status->arming_state == ARMING_STATE_ARMED ? ARMING_STATE_STANDBY : ARMING_STATE_STANDBY_ERROR);
					arming_res = arming_state_transition(status, safety, new_arming_state, armed);

					if (arming_res == TRANSITION_CHANGED) {
						mavlink_log_info(mavlink_fd, "[cmd] DISARMED by command");
					}

				} else {
					arming_res = TRANSITION_NOT_CHANGED;
				}
			}

			if (arming_res == TRANSITION_CHANGED)
				ret = true;

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

			if (main_res == TRANSITION_CHANGED)
				ret = true;

			if (arming_res != TRANSITION_DENIED && main_res != TRANSITION_DENIED) {
				result = VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

			break;
		}

	case VEHICLE_CMD_COMPONENT_ARM_DISARM: {
			transition_result_t arming_res = TRANSITION_NOT_CHANGED;

			if (!armed->armed && ((int)(cmd->param1 + 0.5f)) == 1) {
				if (safety->safety_switch_available && !safety->safety_off && status->hil_state == HIL_STATE_OFF) {
					print_reject_arm("NOT ARMING: Press safety switch first.");
					arming_res = TRANSITION_DENIED;

				} else {
					arming_res = arming_state_transition(status, safety, ARMING_STATE_ARMED, armed);
				}

				if (arming_res == TRANSITION_CHANGED) {
					mavlink_log_critical(mavlink_fd, "#audio: ARMED by component arm cmd");
					result = VEHICLE_CMD_RESULT_ACCEPTED;
					ret = true;

				} else {
					mavlink_log_critical(mavlink_fd, "#audio: REJECTING component arm cmd");
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}
		}
		break;

	case VEHICLE_CMD_OVERRIDE_GOTO: {
			// TODO listen vehicle_command topic directly from navigator (?)
			unsigned int mav_goto = cmd->param1;

			if (mav_goto == 0) {	// MAV_GOTO_DO_HOLD
				status->set_nav_state = NAV_STATE_LOITER;
				status->set_nav_state_timestamp = hrt_absolute_time();
				mavlink_log_critical(mavlink_fd, "#audio: pause mission cmd");
				result = VEHICLE_CMD_RESULT_ACCEPTED;
				ret = true;

			} else if (mav_goto == 1) {	// MAV_GOTO_DO_CONTINUE
				status->set_nav_state = NAV_STATE_MISSION;
				status->set_nav_state_timestamp = hrt_absolute_time();
				mavlink_log_critical(mavlink_fd, "#audio: continue mission cmd");
				result = VEHICLE_CMD_RESULT_ACCEPTED;
				ret = true;

			} else {
				mavlink_log_info(mavlink_fd, "Unsupported OVERRIDE_GOTO: %f %f %f %f %f %f %f %f", cmd->param1, cmd->param2, cmd->param3, cmd->param4, cmd->param5, cmd->param6, cmd->param7);
			}
		}
		break;

		/* Flight termination */
	case VEHICLE_CMD_DO_SET_SERVO: { //xxx: needs its own mavlink command

			//XXX: to enable the parachute, a param needs to be set
			//xxx: for safety only for now, param3 is unused by VEHICLE_CMD_DO_SET_SERVO
			if (armed->armed && cmd->param3 > 0.5 && parachute_enabled) {
				transition_result_t failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_TERMINATION);
				result = VEHICLE_CMD_RESULT_ACCEPTED;
				ret = true;

			} else {
				/* reject parachute depoyment not armed */
				result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

		}
		break;

	case VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
	case VEHICLE_CMD_PREFLIGHT_CALIBRATION:
	case VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case VEHICLE_CMD_PREFLIGHT_STORAGE:
		/* ignore commands that handled in low prio loop */
		break;

	default:
		/* warn about unsupported commands */
		answer_command(*cmd, VEHICLE_CMD_RESULT_UNSUPPORTED);
		break;
	}

	if (result != VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(*cmd, result);
	}

	/* send any requested ACKs */
	if (cmd->confirmation > 0 && result != VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* send acknowledge command */
		// XXX TODO
	}

}

int commander_thread_main(int argc, char *argv[])
{
	/* not yet initialized */
	commander_initialized = false;

	bool arm_tune_played = false;
	bool was_armed = false;

	/* set parameters */
	param_t _param_sys_type = param_find("MAV_TYPE");
	param_t _param_system_id = param_find("MAV_SYS_ID");
	param_t _param_component_id = param_find("MAV_COMP_ID");
	param_t _param_takeoff_alt = param_find("NAV_TAKEOFF_ALT");
	param_t _param_enable_parachute = param_find("NAV_PARACHUTE_EN");

	/* welcome user */
	warnx("starting");

	char *main_states_str[MAIN_STATE_MAX];
	main_states_str[0] = "MANUAL";
	main_states_str[1] = "SEATBELT";
	main_states_str[2] = "EASY";
	main_states_str[3] = "AUTO";

	char *arming_states_str[ARMING_STATE_MAX];
	arming_states_str[0] = "INIT";
	arming_states_str[1] = "STANDBY";
	arming_states_str[2] = "ARMED";
	arming_states_str[3] = "ARMED_ERROR";
	arming_states_str[4] = "STANDBY_ERROR";
	arming_states_str[5] = "REBOOT";
	arming_states_str[6] = "IN_AIR_RESTORE";

	char *failsafe_states_str[FAILSAFE_STATE_MAX];
	failsafe_states_str[0] = "NORMAL";
	failsafe_states_str[1] = "RTL";
	failsafe_states_str[2] = "LAND";
	failsafe_states_str[3] = "TERMINATION";

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

	/* vehicle status topic */
	memset(&status, 0, sizeof(status));
	status.condition_landed = true;	// initialize to safe value
	// We want to accept RC inputs as default
	status.rc_input_blocked = false;
	status.main_state = MAIN_STATE_MANUAL;
	status.set_nav_state = NAV_STATE_NONE;
	status.set_nav_state_timestamp = 0;
	status.arming_state = ARMING_STATE_INIT;
	status.hil_state = HIL_STATE_OFF;
	status.failsafe_state = FAILSAFE_STATE_NORMAL;

	/* neither manual nor offboard control commands have been received */
	status.offboard_control_signal_found_once = false;
	status.rc_signal_found_once = false;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = true;
	status.offboard_control_signal_lost = true;

	/* set battery warning flag */
	status.battery_warning = VEHICLE_BATTERY_WARNING_NONE;
	status.condition_battery_voltage_valid = false;

	// XXX for now just set sensors as initialized
	status.condition_system_sensors_initialized = true;

	status.counter++;
	status.timestamp = hrt_absolute_time();

	/* publish initial state */
	status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

	/* armed topic */
	orb_advert_t armed_pub;
	/* Initialize armed with all false */
	memset(&armed, 0, sizeof(armed));

	/* vehicle control mode topic */
	memset(&control_mode, 0, sizeof(control_mode));
	orb_advert_t control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &control_mode);

	armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

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
					status.is_rotary_wing = true;

				} else {
					status.is_rotary_wing = false;
				}

				/* check and update system / component ID */
				param_get(_param_system_id, &(status.system_id));
				param_get(_param_component_id, &(status.component_id));
				status_changed = true;

				/* re-check RC calibration */
				rc_calibration_ok = (OK == rc_calibration_check(mavlink_fd));
			}
			/* navigation parameters */
			param_get(_param_takeoff_alt, &takeoff_alt);
			param_get(_param_enable_parachute, &parachute_enabled);
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

			/* disarm if safety is now on and still armed */
			if (status.hil_state == HIL_STATE_OFF && safety.safety_switch_available && !safety.safety_off && armed.armed) {
				arming_state_t new_arming_state = (status.arming_state == ARMING_STATE_ARMED ? ARMING_STATE_STANDBY : ARMING_STATE_STANDBY_ERROR);
				if (TRANSITION_CHANGED == arming_state_transition(&status, &safety, new_arming_state, &armed)) {
					mavlink_log_info(mavlink_fd, "[cmd] DISARMED by safety switch");
				}
			}
		}

		/* update global position estimate */
		orb_check(global_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
		}

		/* update condition_global_position_valid */
		check_valid(global_position.timestamp, POSITION_TIMEOUT, true, &(status.condition_global_position_valid), &status_changed);

		/* check if GPS fix is ok */
		static float hdop_threshold_m = 4.0f;
		static float vdop_threshold_m = 8.0f;

		/* update home position */
		if (!status.condition_home_position_valid && status.condition_global_position_valid && !armed.armed &&
			(global_position.eph < hdop_threshold_m) && (global_position.epv < vdop_threshold_m)) {

			home.lat = global_position.lat;
			home.lon = global_position.lon;
			home.alt = global_position.alt;

			warnx("home: lat = %.7f, lon = %.7f, alt = %.2f ", home.lat, home.lon, (double)home.alt);
			mavlink_log_info(mavlink_fd, "[cmd] home: %.7f, %.7f, %.2f", home.lat, home.lon, (double)home.alt);

			/* announce new home position */
			if (home_pub > 0) {
				orb_publish(ORB_ID(home_position), home_pub, &home);

			} else {
				home_pub = orb_advertise(ORB_ID(home_position), &home);
			}

			/* mark home position as set */
			status.condition_home_position_valid = true;
			tune_positive(true);
		}

		/* update local position estimate */
		orb_check(local_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
		}

		/* update condition_local_position_valid and condition_local_altitude_valid */
		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.xy_valid, &(status.condition_local_position_valid), &status_changed);
		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.z_valid, &(status.condition_local_altitude_valid), &status_changed);

		static bool published_condition_landed_fw = false;
		if (status.is_rotary_wing && status.condition_local_altitude_valid) {
			if (status.condition_landed != local_position.landed) {
				status.condition_landed = local_position.landed;
				status_changed = true;
				published_condition_landed_fw = false; //make sure condition_landed is published again if the system type changes

				if (status.condition_landed) {
					mavlink_log_critical(mavlink_fd, "#audio: LANDED");

				} else {
					mavlink_log_critical(mavlink_fd, "#audio: IN AIR");
				}
			}
		} else {
			if (!published_condition_landed_fw) {
				status.condition_landed = false; // Fixedwing does not have a landing detector currently
				published_condition_landed_fw = true;
				status_changed = true;
			}
		}

		/* update battery status */
		orb_check(battery_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);

			/* only consider battery voltage if system has been running 2s and battery voltage is valid */
			if (status.hil_state == HIL_STATE_OFF && hrt_absolute_time() > start_time + 2000000 && battery.voltage_filtered_v > 0.0f) {
				status.battery_voltage = battery.voltage_filtered_v;
				status.battery_current = battery.current_a;
				status.condition_battery_voltage_valid = true;
				status.battery_remaining = battery_remaining_estimate_voltage(battery.voltage_filtered_v, battery.discharged_mah);
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

		/* if battery voltage is getting lower, warn using buzzer, etc. */
		if (status.condition_battery_voltage_valid && status.battery_remaining < 0.25f && !low_battery_voltage_actions_done) {
			low_battery_voltage_actions_done = true;
			mavlink_log_critical(mavlink_fd, "#audio: WARNING: LOW BATTERY");
			status.battery_warning = VEHICLE_BATTERY_WARNING_LOW;
			status_changed = true;

		} else if (status.condition_battery_voltage_valid && status.battery_remaining < 0.1f && !critical_battery_voltage_actions_done && low_battery_voltage_actions_done) {
			/* critical battery voltage, this is rather an emergency, change state machine */
			critical_battery_voltage_actions_done = true;
			mavlink_log_critical(mavlink_fd, "#audio: EMERGENCY: CRITICAL BATTERY");
			status.battery_warning = VEHICLE_BATTERY_WARNING_CRITICAL;

			if (armed.armed) {
				arming_state_transition(&status, &safety, ARMING_STATE_ARMED_ERROR, &armed);

			} else {
				arming_state_transition(&status, &safety, ARMING_STATE_STANDBY_ERROR, &armed);
			}

			status_changed = true;
		}

		/* End battery voltage check */

		/* If in INIT state, try to proceed to STANDBY state */
		if (status.arming_state == ARMING_STATE_INIT && low_prio_task == LOW_PRIO_TASK_NONE) {
			// XXX check for sensors
			arming_state_transition(&status, &safety, ARMING_STATE_STANDBY, &armed);

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
		}


		/*
		 * XXX workaround:
		 * Prevent RC loss in HIL when sensors.cpp is only publishing sp_man at a low rate (e.g. 30Hz)
		 * which can trigger RC loss if the computer/simulator lags.
		 */
		uint64_t rc_timeout = status.hil_state == HIL_STATE_ON ? RC_TIMEOUT_HIL : RC_TIMEOUT;

		/* start RC input check */
		if (!status.rc_input_blocked && sp_man.timestamp != 0 && hrt_absolute_time() < sp_man.timestamp + rc_timeout) {
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
			    (status.main_state == MAIN_STATE_MANUAL || status.condition_landed) &&
			    sp_man.yaw < -STICK_ON_OFF_LIMIT && sp_man.throttle < STICK_THRUST_RANGE * 0.1f) {

				if (stick_off_counter > STICK_ON_OFF_COUNTER_LIMIT) {
					/* disarm to STANDBY if ARMED or to STANDBY_ERROR if ARMED_ERROR */
					arming_state_t new_arming_state = (status.arming_state == ARMING_STATE_ARMED ? ARMING_STATE_STANDBY : ARMING_STATE_STANDBY_ERROR);
					res = arming_state_transition(&status, &safety, new_arming_state, &armed);
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
					if (safety.safety_switch_available && !safety.safety_off && status.hil_state == HIL_STATE_OFF) {
						print_reject_arm("NOT ARMING: Press safety switch first.");

					} else if (status.main_state != MAIN_STATE_MANUAL) {
						print_reject_arm("NOT ARMING: Switch to MANUAL mode first.");

					} else {
						res = arming_state_transition(&status, &safety, ARMING_STATE_ARMED, &armed);
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
				/* DENIED here indicates bug in the commander */
				mavlink_log_critical(mavlink_fd, "ERROR: arming state transition denied");
			}

			if (status.failsafe_state != FAILSAFE_STATE_NORMAL) {
				/* recover from failsafe */
				transition_result_t res = failsafe_state_transition(&status, FAILSAFE_STATE_NORMAL);
			}

			/* fill status according to mode switches */
			check_mode_switches(&sp_man, &status);

			/* evaluate the main state machine according to mode switches */
			res = set_main_state_rc(&status);

			/* play tune on mode change only if armed, blink LED always */
			if (res == TRANSITION_CHANGED) {
				tune_positive(armed.armed);

			} else if (res == TRANSITION_DENIED) {
				/* DENIED here indicates bug in the commander */
				mavlink_log_critical(mavlink_fd, "ERROR: main state transition denied");
			}

		} else {
			if (!status.rc_signal_lost) {
				mavlink_log_critical(mavlink_fd, "#audio: CRITICAL: RC SIGNAL LOST");
				status.rc_signal_lost = true;
				status_changed = true;
			}

			if (armed.armed) {
				if (status.main_state == MAIN_STATE_AUTO) {
					/* check if AUTO mode still allowed */
					transition_result_t res = main_state_transition(&status, MAIN_STATE_AUTO);

					if (res == TRANSITION_DENIED) {
						/* AUTO mode denied, don't try RTL, switch to failsafe state LAND */
						res = failsafe_state_transition(&status, FAILSAFE_STATE_LAND);

						if (res == TRANSITION_DENIED) {
							/* LAND not allowed, set TERMINATION state */
							transition_result_t res = failsafe_state_transition(&status, FAILSAFE_STATE_TERMINATION);
						}
					}

				} else {
					/* failsafe for manual modes */
					transition_result_t res = failsafe_state_transition(&status, FAILSAFE_STATE_RTL);

					if (res == TRANSITION_DENIED) {
						/* RTL not allowed (no global position estimate), try LAND */
						res = failsafe_state_transition(&status, FAILSAFE_STATE_LAND);

						if (res == TRANSITION_DENIED) {
							/* LAND not allowed, set TERMINATION state */
							res = failsafe_state_transition(&status, FAILSAFE_STATE_TERMINATION);
						}
					}
				}

			} else {
				if (status.failsafe_state != FAILSAFE_STATE_NORMAL) {
					/* reset failsafe when disarmed */
					transition_result_t res = failsafe_state_transition(&status, FAILSAFE_STATE_NORMAL);
				}
			}
		}

		// TODO remove this hack
		/* flight termination in manual mode if assisted switch is on easy position */
		if (!status.is_rotary_wing && parachute_enabled && armed.armed && status.main_state == MAIN_STATE_MANUAL && sp_man.assisted_switch > STICK_ON_OFF_LIMIT) {
			if (TRANSITION_CHANGED == failsafe_state_transition(&status, FAILSAFE_STATE_TERMINATION)) {
				tune_positive(armed.armed);
			}
		}

		/* handle commands last, as the system needs to be updated to handle them */
		orb_check(cmd_sub, &updated);

		if (updated) {
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			if (handle_command(&status, &safety, &cmd, &armed))
				status_changed = true;
		}

		/* check which state machines for changes, clear "changed" flag */
		bool arming_state_changed = check_arming_state_changed();
		bool main_state_changed = check_main_state_changed();
		bool failsafe_state_changed = check_failsafe_state_changed();

		hrt_abstime t1 = hrt_absolute_time();

		/* print new state */
		if (arming_state_changed) {
			status_changed = true;
			mavlink_log_info(mavlink_fd, "[cmd] arming state: %s", arming_states_str[status.arming_state]);

			/* update home position on arming */
			if (armed.armed && !was_armed && status.condition_global_position_valid &&
				(global_position.eph < hdop_threshold_m) && (global_position.epv < vdop_threshold_m)) {

				// TODO remove code duplication
				home.lat = global_position.lat;
				home.lon = global_position.lon;
				home.alt = global_position.alt;

				warnx("home: lat = %.7f, lon = %.7f, alt = %.2f ", home.lat, home.lon, (double)home.alt);
				mavlink_log_info(mavlink_fd, "[cmd] home: %.7f, %.7f, %.2f", home.lat, home.lon, (double)home.alt);

				/* announce new home position */
				if (home_pub > 0) {
					orb_publish(ORB_ID(home_position), home_pub, &home);

				} else {
					home_pub = orb_advertise(ORB_ID(home_position), &home);
				}

				/* mark home position as set */
				status.condition_home_position_valid = true;
			}
		}
		was_armed = armed.armed;

		if (main_state_changed) {
			status_changed = true;
			mavlink_log_info(mavlink_fd, "[cmd] main state: %s", main_states_str[status.main_state]);
		}

		if (failsafe_state_changed) {
			status_changed = true;
			mavlink_log_info(mavlink_fd, "[cmd] failsafe state: %s", failsafe_states_str[status.failsafe_state]);
		}

		/* publish states (armed, control mode, vehicle status) at least with 5 Hz */
		if (counter % (200000 / COMMANDER_MONITORING_INTERVAL) == 0 || status_changed) {
			set_control_mode();
			control_mode.timestamp = t1;
			orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, &control_mode);

			status.timestamp = t1;
			orb_publish(ORB_ID(vehicle_status), status_pub, &status);

			armed.timestamp = t1;
			orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
		}

		/* play arming and battery warning tunes */
		if (!arm_tune_played && armed.armed && (!safety.safety_switch_available || (safety.safety_switch_available && safety.safety_off))) {
			/* play tune when armed */
			set_tune(TONE_ARMING_WARNING_TUNE);
			arm_tune_played = true;

		} else if (status.battery_warning == VEHICLE_BATTERY_WARNING_CRITICAL) {
			/* play tune on battery critical */
			set_tune(TONE_BATTERY_WARNING_FAST_TUNE);

		} else if (status.battery_warning == VEHICLE_BATTERY_WARNING_LOW || status.failsafe_state != FAILSAFE_STATE_NORMAL) {
			/* play tune on battery warning or failsafe */
			set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);

		} else {
			set_tune(TONE_STOP_TUNE);
		}

		/* reset arm_tune_played when disarmed */
		if (!armed.armed || (safety.safety_switch_available && !safety.safety_off)) {
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
			if (status->battery_warning == VEHICLE_BATTERY_WARNING_LOW || status->failsafe_state != FAILSAFE_STATE_NORMAL) {
				rgbled_set_color(RGBLED_COLOR_AMBER);
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
check_mode_switches(struct manual_control_setpoint_s *sp_man, struct vehicle_status_s *status)
{
	/* main mode switch */
	if (!isfinite(sp_man->mode_switch)) {
		/* default to manual if signal is invalid */
		status->mode_switch = MODE_SWITCH_MANUAL;

	} else if (sp_man->mode_switch > STICK_ON_OFF_LIMIT) {
		status->mode_switch = MODE_SWITCH_AUTO;

	} else if (sp_man->mode_switch < -STICK_ON_OFF_LIMIT) {
		status->mode_switch = MODE_SWITCH_MANUAL;

	} else {
		status->mode_switch = MODE_SWITCH_ASSISTED;
	}

	/* return switch */
	if (!isfinite(sp_man->return_switch)) {
		status->return_switch = RETURN_SWITCH_NONE;

	} else if (sp_man->return_switch > STICK_ON_OFF_LIMIT) {
		status->return_switch = RETURN_SWITCH_RETURN;

	} else {
		status->return_switch = RETURN_SWITCH_NORMAL;
	}

	/* assisted switch */
	if (!isfinite(sp_man->assisted_switch)) {
		status->assisted_switch = ASSISTED_SWITCH_SEATBELT;

	} else if (sp_man->assisted_switch > STICK_ON_OFF_LIMIT) {
		status->assisted_switch = ASSISTED_SWITCH_EASY;

	} else {
		status->assisted_switch = ASSISTED_SWITCH_SEATBELT;
	}

	/* mission switch  */
	if (!isfinite(sp_man->mission_switch)) {
		status->mission_switch = MISSION_SWITCH_NONE;

	} else if (sp_man->mission_switch > STICK_ON_OFF_LIMIT) {
		status->mission_switch = MISSION_SWITCH_LOITER;

	} else {
		status->mission_switch = MISSION_SWITCH_MISSION;
	}
}

transition_result_t
set_main_state_rc(struct vehicle_status_s *status)
{
	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_DENIED;

	switch (status->mode_switch) {
	case MODE_SWITCH_MANUAL:
		res = main_state_transition(status, MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	case MODE_SWITCH_ASSISTED:
		if (status->assisted_switch == ASSISTED_SWITCH_EASY) {
			res = main_state_transition(status, MAIN_STATE_EASY);

			if (res != TRANSITION_DENIED)
				break;	// changed successfully or already in this state

			// else fallback to SEATBELT
			print_reject_mode(status, "EASY");
		}

		res = main_state_transition(status, MAIN_STATE_SEATBELT);

		if (res != TRANSITION_DENIED)
			break;	// changed successfully or already in this mode

		if (status->assisted_switch != ASSISTED_SWITCH_EASY)	// don't print both messages
			print_reject_mode(status, "SEATBELT");

		// else fallback to MANUAL
		res = main_state_transition(status, MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	case MODE_SWITCH_AUTO:
		res = main_state_transition(status, MAIN_STATE_AUTO);

		if (res != TRANSITION_DENIED)
			break;	// changed successfully or already in this state

		// else fallback to SEATBELT (EASY likely will not work too)
		print_reject_mode(status, "AUTO");
		res = main_state_transition(status, MAIN_STATE_SEATBELT);

		if (res != TRANSITION_DENIED)
			break;	// changed successfully or already in this state

		// else fallback to MANUAL
		res = main_state_transition(status, MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	default:
		break;
	}

	return res;
}

void

set_control_mode()
{
	/* set vehicle_control_mode according to main state and failsafe state */
	control_mode.flag_armed = armed.armed;
	control_mode.flag_external_manual_override_ok = !status.is_rotary_wing;
	control_mode.flag_system_hil_enabled = status.hil_state == HIL_STATE_ON;

	control_mode.flag_control_termination_enabled = false;

	/* set this flag when navigator should act */
	bool navigator_enabled = false;

	switch (status.failsafe_state) {
	case FAILSAFE_STATE_NORMAL:
		switch (status.main_state) {
		case MAIN_STATE_MANUAL:
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_auto_enabled = false;
			control_mode.flag_control_rates_enabled = status.is_rotary_wing;
			control_mode.flag_control_attitude_enabled = status.is_rotary_wing;
			control_mode.flag_control_altitude_enabled = false;
			control_mode.flag_control_climb_rate_enabled = false;
			control_mode.flag_control_position_enabled = false;
			control_mode.flag_control_velocity_enabled = false;
			break;

		case MAIN_STATE_SEATBELT:
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_auto_enabled = false;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_altitude_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_position_enabled = false;
			control_mode.flag_control_velocity_enabled = false;
			break;

		case MAIN_STATE_EASY:
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_auto_enabled = false;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_altitude_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_position_enabled = true;
			control_mode.flag_control_velocity_enabled = true;
			break;

		case MAIN_STATE_AUTO:
			navigator_enabled = true;

		default:
			break;
		}

		break;

	case FAILSAFE_STATE_RTL:
		navigator_enabled = true;
		break;

	case FAILSAFE_STATE_LAND:
		navigator_enabled = true;
		break;

	case FAILSAFE_STATE_TERMINATION:
		/* disable all controllers on termination */
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = false;
		control_mode.flag_control_attitude_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_termination_enabled = true;
		break;

	default:
		break;
	}

	/* navigator has control, set control mode flags according to nav state*/
	if (navigator_enabled) {
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
	}
}

void
print_reject_mode(struct vehicle_status_s *status, const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		char s[80];
		sprintf(s, "#audio: REJECT %s", msg);
		mavlink_log_critical(mavlink_fd, s);

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		tune_negative(armed.armed);
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
		tune_negative(true);
	}
}

void answer_command(struct vehicle_command_s &cmd, enum VEHICLE_CMD_RESULT result)
{
	switch (result) {
	case VEHICLE_CMD_RESULT_ACCEPTED:
			tune_positive(true);
		break;

	case VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(mavlink_fd, "#audio: command denied: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(mavlink_fd, "#audio: command failed: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(mavlink_fd, "#audio: command temporarily rejected: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(mavlink_fd, "#audio: command unsupported: %u", cmd.command);
		tune_negative(true);
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
		    cmd.command == VEHICLE_CMD_NAV_TAKEOFF ||
		    cmd.command == VEHICLE_CMD_DO_SET_SERVO)
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
				if (TRANSITION_DENIED == arming_state_transition(&status, &safety, ARMING_STATE_INIT, &armed)) {
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
					/* disable RC control input completely */
					status.rc_input_blocked = true;
					calib_ret = OK;
					mavlink_log_info(mavlink_fd, "CAL: Disabling RC IN");

				} else if ((int)(cmd.param4) == 2) {
					/* RC trim calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_trim_calibration(mavlink_fd);

				} else if ((int)(cmd.param5) == 1) {
					/* accelerometer calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_accel_calibration(mavlink_fd);

				} else if ((int)(cmd.param6) == 1) {
					/* airspeed calibration */
					answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					calib_ret = do_airspeed_calibration(mavlink_fd);
				} else if ((int)(cmd.param4) == 0) {
					/* RC calibration ended - have we been in one worth confirming? */
					if (status.rc_input_blocked) {
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						/* enable RC control input */
						status.rc_input_blocked = false;
						mavlink_log_info(mavlink_fd, "CAL: Re-enabling RC IN");
					}

					/* this always succeeds */
					calib_ret = OK;

				}

				if (calib_ret == OK)
					tune_positive(true);
				else
					tune_negative(true);

				arming_state_transition(&status, &safety, ARMING_STATE_STANDBY, &armed);

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

		case VEHICLE_CMD_START_RX_PAIR:
		/* handled in the IO driver */
		break;

		default:
			/* don't answer on unsupported commands, it will be done in main loop */
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

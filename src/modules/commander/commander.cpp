/****************************************************************************
 *
 *   Copyright (C) 2013-2014 PX4 Development Team. All rights reserved.
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
 * Main fail-safe handling.
 *
 * @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
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
#include <systemlib/circuit_breaker.h>
#include <debug.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <string.h>
#include <math.h>
#include <poll.h>
#include <float.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/telemetry_status.h>

#include <drivers/drv_led.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/cpuload.h>
#include <systemlib/rc_check.h>
#include <geo/geo.h>
#include <systemlib/state_table.h>
#include <dataman/dataman.h>

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

#define STICK_ON_OFF_LIMIT 0.9f
#define STICK_ON_OFF_HYSTERESIS_TIME_MS 1000
#define STICK_ON_OFF_COUNTER_LIMIT (STICK_ON_OFF_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define POSITION_TIMEOUT		(2 * 1000 * 1000)	/**< consider the local or global position estimate invalid after 600ms */
#define FAILSAFE_DEFAULT_TIMEOUT	(3 * 1000 * 1000)	/**< hysteresis time - the failsafe will trigger after 3 seconds in this state */
#define OFFBOARD_TIMEOUT		500000
#define DIFFPRESS_TIMEOUT		2000000

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
static int mavlink_fd = 0;

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
static float eph_threshold = 5.0f;
static float epv_threshold = 10.0f;

static struct vehicle_status_s status;
static struct actuator_armed_s armed;
static struct safety_s safety;
static struct vehicle_control_mode_s control_mode;
static struct offboard_control_setpoint_s sp_offboard;

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
bool handle_command(struct vehicle_status_s *status, const struct safety_s *safety, struct vehicle_command_s *cmd,
		    struct actuator_armed_s *armed, struct home_position_s *home, struct vehicle_global_position_s *global_pos,
		    orb_advert_t *home_pub);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

void control_status_leds(vehicle_status_s *status, const actuator_armed_s *actuator_armed, bool changed);

void check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed);

transition_result_t set_main_state_rc(struct vehicle_status_s *status, struct manual_control_setpoint_s *sp_man);

void set_control_mode();

void print_reject_mode(struct vehicle_status_s *current_status, const char *msg);

void print_reject_arm(const char *msg);

void print_status();

transition_result_t check_navigation_state_machine(struct vehicle_status_s *status,
		struct vehicle_control_mode_s *control_mode, struct vehicle_local_position_s *local_pos);

transition_result_t arm_disarm(bool arm, const int mavlink_fd, const char *armedBy);

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);

void answer_command(struct vehicle_command_s &cmd, enum VEHICLE_CMD_RESULT result);


int commander_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

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
					     3200,
					     commander_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);

		while (!thread_running) {
			usleep(200);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running) {
			errx(0, "commander already stopped");
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		warnx("terminated.");

		exit(0);
	}

	/* commands needing the app to run below */
	if (!thread_running) {
		warnx("\tcommander not started");
		exit(1);
	}

	if (!strcmp(argv[1], "status")) {
		print_status();
		exit(0);
	}

	if (!strcmp(argv[1], "check")) {
		int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
		int checkres = prearm_check(&status, mavlink_fd_local);
		close(mavlink_fd_local);
		warnx("FINAL RESULT: %s", (checkres == 0) ? "OK" : "FAILED");
		exit(0);
	}

	if (!strcmp(argv[1], "arm")) {
		int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
		arm_disarm(true, mavlink_fd_local, "command line");
		close(mavlink_fd_local);
		exit(0);
	}

	if (!strcmp(argv[1], "disarm")) {
		int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
		arm_disarm(false, mavlink_fd_local, "command line");
		close(mavlink_fd_local);
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

void print_status()
{
	warnx("type: %s", (status.is_rotary_wing) ? "ROTARY" : "PLANE");
	warnx("usb powered: %s", (on_usb_power) ? "yes" : "no");
	warnx("avionics rail: %6.2f V", (double)status.avionics_power_rail_voltage);

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

transition_result_t arm_disarm(bool arm, const int mavlink_fd_local, const char *armedBy)
{
	transition_result_t arming_res = TRANSITION_NOT_CHANGED;

	// Transition the armed state. By passing mavlink_fd to arming_state_transition it will
	// output appropriate error messages if the state cannot transition.
	arming_res = arming_state_transition(&status, &safety, arm ? ARMING_STATE_ARMED : ARMING_STATE_STANDBY, &armed,
					     true /* fRunPreArmChecks */, mavlink_fd_local);

	if (arming_res == TRANSITION_CHANGED && mavlink_fd) {
		mavlink_log_info(mavlink_fd_local, "[cmd] %s by %s", arm ? "ARMED" : "DISARMED", armedBy);

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

bool handle_command(struct vehicle_status_s *status_local, const struct safety_s *safety_local,
		    struct vehicle_command_s *cmd, struct actuator_armed_s *armed_local,
		    struct home_position_s *home, struct vehicle_global_position_s *global_pos, orb_advert_t *home_pub)
{
	/* only handle commands that are meant to be handled by this system and component */
	if (cmd->target_system != status_local->system_id || ((cmd->target_component != status_local->component_id)
			&& (cmd->target_component != 0))) { // component_id 0: valid for all components
		return false;
	}

	/* result of the command */
	enum VEHICLE_CMD_RESULT cmd_result = VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd->command) {
	case VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t)cmd->param1;
			uint8_t custom_main_mode = (uint8_t)cmd->param2;

			transition_result_t arming_ret = TRANSITION_NOT_CHANGED;

			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			/* set HIL state */
			hil_state_t new_hil_state = (base_mode & MAV_MODE_FLAG_HIL_ENABLED) ? HIL_STATE_ON : HIL_STATE_OFF;
			transition_result_t hil_ret = hil_state_transition(new_hil_state, status_pub, status_local, mavlink_fd);

			// Transition the arming state
			arming_ret = arm_disarm(base_mode & MAV_MODE_FLAG_SAFETY_ARMED, mavlink_fd, "set mode command");

			if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					/* MANUAL */
					main_ret = main_state_transition(status_local, MAIN_STATE_MANUAL);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					/* ALTCTL */
					main_ret = main_state_transition(status_local, MAIN_STATE_ALTCTL);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					/* POSCTL */
					main_ret = main_state_transition(status_local, MAIN_STATE_POSCTL);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					/* AUTO */
					main_ret = main_state_transition(status_local, MAIN_STATE_AUTO_MISSION);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					/* ACRO */
					main_ret = main_state_transition(status_local, MAIN_STATE_ACRO);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					/* OFFBOARD */
					main_ret = main_state_transition(status_local, MAIN_STATE_OFFBOARD);
				}

			} else {
				/* use base mode */
				if (base_mode & MAV_MODE_FLAG_AUTO_ENABLED) {
					/* AUTO */
					main_ret = main_state_transition(status_local, MAIN_STATE_AUTO_MISSION);

				} else if (base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) {
						/* POSCTL */
						main_ret = main_state_transition(status_local, MAIN_STATE_POSCTL);

					} else if (base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) {
						/* MANUAL */
						main_ret = main_state_transition(status_local, MAIN_STATE_MANUAL);
					}
				}
			}

			if (hil_ret != TRANSITION_DENIED && arming_ret != TRANSITION_DENIED && main_ret != TRANSITION_DENIED) {
				cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case VEHICLE_CMD_COMPONENT_ARM_DISARM: {
			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters

			if (static_cast<int>(cmd->param1 + 0.5f) != 0 && static_cast<int>(cmd->param1 + 0.5f) != 1) {
				mavlink_log_critical(mavlink_fd, "Unsupported ARM_DISARM param: %.3f", (double)cmd->param1);

			} else {

				bool cmd_arms = (static_cast<int>(cmd->param1 + 0.5f) == 1);

				// Flick to inair restore first if this comes from an onboard system
				if (cmd->source_system == status_local->system_id && cmd->source_component == status_local->component_id) {
					status_local->arming_state = ARMING_STATE_IN_AIR_RESTORE;
				}

				transition_result_t arming_res = arm_disarm(cmd_arms, mavlink_fd, "arm/disarm component command");

				if (arming_res == TRANSITION_DENIED) {
					mavlink_log_critical(mavlink_fd, "REJECTING component arm cmd");
					cmd_result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				} else {
					cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;
				}
			}
		}
		break;

	case VEHICLE_CMD_OVERRIDE_GOTO: {
			// TODO listen vehicle_command topic directly from navigator (?)

			// Increase by 0.5f and rely on the integer cast
			// implicit floor(). This is the *safest* way to
			// convert from floats representing small ints to actual ints.
			unsigned int mav_goto = (cmd->param1 + 0.5f);

			if (mav_goto == 0) {	// MAV_GOTO_DO_HOLD
				status_local->nav_state = NAVIGATION_STATE_AUTO_LOITER;
				mavlink_log_critical(mavlink_fd, "Pause mission cmd");
				cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;

			} else if (mav_goto == 1) {	// MAV_GOTO_DO_CONTINUE
				status_local->nav_state = NAVIGATION_STATE_AUTO_MISSION;
				mavlink_log_critical(mavlink_fd, "Continue mission cmd");
				cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(mavlink_fd, "REJ CMD: %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
						     (double)cmd->param1,
						     (double)cmd->param2,
						     (double)cmd->param3,
						     (double)cmd->param4,
						     (double)cmd->param5,
						     (double)cmd->param6,
						     (double)cmd->param7);
			}
		}
		break;

	/* Flight termination */
	case VEHICLE_CMD_DO_FLIGHTTERMINATION: {
			if (cmd->param1 > 0.5f) {
				//XXX update state machine?
				armed_local->force_failsafe = true;
				warnx("forcing failsafe (termination)");

			} else {
				armed_local->force_failsafe = false;
				warnx("disabling failsafe (termination)");
			}

			/* param2 is currently used for other failsafe modes */
			status_local->engine_failure_cmd = false;
			status_local->data_link_lost_cmd = false;
			status_local->gps_failure_cmd = false;
			status_local->rc_signal_lost_cmd = false;

			if ((int)cmd->param2 <= 0) {
				/* reset all commanded failure modes */
				warnx("reset all non-flighttermination failsafe commands");

			} else if ((int)cmd->param2 == 1) {
				/* trigger engine failure mode */
				status_local->engine_failure_cmd = true;
				warnx("engine failure mode commanded");

			} else if ((int)cmd->param2 == 2) {
				/* trigger data link loss mode */
				status_local->data_link_lost_cmd = true;
				warnx("data link loss mode commanded");

			} else if ((int)cmd->param2 == 3) {
				/* trigger gps loss mode */
				status_local->gps_failure_cmd = true;
				warnx("gps loss mode commanded");

			} else if ((int)cmd->param2 == 4) {
				/* trigger rc loss mode */
				status_local->rc_signal_lost_cmd = true;
				warnx("rc loss mode commanded");
			}

			cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;
		}
		break;

	case VEHICLE_CMD_DO_SET_HOME: {
			bool use_current = cmd->param1 > 0.5f;

			if (use_current) {
				/* use current position */
				if (status_local->condition_global_position_valid) {
					home->lat = global_pos->lat;
					home->lon = global_pos->lon;
					home->alt = global_pos->alt;

					home->timestamp = hrt_absolute_time();

					cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

			} else {
				/* use specified position */
				home->lat = cmd->param5;
				home->lon = cmd->param6;
				home->alt = cmd->param7;

				home->timestamp = hrt_absolute_time();

				cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;
			}

			if (cmd_result == VEHICLE_CMD_RESULT_ACCEPTED) {
				warnx("home: lat = %.7f, lon = %.7f, alt = %.2f ", home->lat, home->lon, (double)home->alt);
				mavlink_log_info(mavlink_fd, "[cmd] home: %.7f, %.7f, %.2f", home->lat, home->lon, (double)home->alt);

				/* announce new home position */
				if (*home_pub > 0) {
					orb_publish(ORB_ID(home_position), *home_pub, home);

				} else {
					*home_pub = orb_advertise(ORB_ID(home_position), home);
				}

				/* mark home position as set */
				status_local->condition_home_position_valid = true;
			}
		}
		break;

	case VEHICLE_CMD_NAV_GUIDED_ENABLE: {
			transition_result_t res = TRANSITION_DENIED;
			static main_state_t main_state_pre_offboard = MAIN_STATE_MANUAL;

			if (status_local->main_state != MAIN_STATE_OFFBOARD) {
				main_state_pre_offboard = status_local->main_state;
			}

			if (cmd->param1 > 0.5f) {
				res = main_state_transition(status_local, MAIN_STATE_OFFBOARD);

				if (res == TRANSITION_DENIED) {
					print_reject_mode(status_local, "OFFBOARD");
					status_local->offboard_control_set_by_command = false;

				} else {
					/* Set flag that offboard was set via command, main state is not overridden by rc */
					status_local->offboard_control_set_by_command = true;
				}

			} else {
				/* If the mavlink command is used to enable or disable offboard control:
				 * switch back to previous mode when disabling */
				res = main_state_transition(status_local, main_state_pre_offboard);
				status_local->offboard_control_set_by_command = false;
			}
		}
		break;

	case VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
	case VEHICLE_CMD_PREFLIGHT_CALIBRATION:
	case VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case VEHICLE_CMD_PREFLIGHT_STORAGE:
	case VEHICLE_CMD_CUSTOM_0:
	case VEHICLE_CMD_CUSTOM_1:
	case VEHICLE_CMD_CUSTOM_2:
	case VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:
	case VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
		/* ignore commands that handled in low prio loop */
		break;

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(*cmd, VEHICLE_CMD_RESULT_UNSUPPORTED);
		break;
	}

	if (cmd_result != VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(*cmd, cmd_result);
	}

	/* send any requested ACKs */
	if (cmd->confirmation > 0 && cmd_result != VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* send acknowledge command */
		// XXX TODO
	}

	return true;
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
	param_t _param_enable_datalink_loss = param_find("COM_DL_LOSS_EN");
	param_t _param_datalink_loss_timeout = param_find("COM_DL_LOSS_T");
	param_t _param_rc_loss_timeout = param_find("COM_RC_LOSS_T");
	param_t _param_datalink_regain_timeout = param_find("COM_DL_REG_T");
	param_t _param_ef_throttle_thres = param_find("COM_EF_THROT");
	param_t _param_ef_current2throttle_thres = param_find("COM_EF_C2T");
	param_t _param_ef_time_thres = param_find("COM_EF_TIME");

	/* welcome user */
	warnx("starting");

	const char *main_states_str[MAIN_STATE_MAX];
	main_states_str[MAIN_STATE_MANUAL]			= "MANUAL";
	main_states_str[MAIN_STATE_ALTCTL]			= "ALTCTL";
	main_states_str[MAIN_STATE_POSCTL]			= "POSCTL";
	main_states_str[MAIN_STATE_AUTO_MISSION]		= "AUTO_MISSION";
	main_states_str[MAIN_STATE_AUTO_LOITER]			= "AUTO_LOITER";
	main_states_str[MAIN_STATE_AUTO_RTL]			= "AUTO_RTL";
	main_states_str[MAIN_STATE_ACRO]			= "ACRO";
	main_states_str[MAIN_STATE_OFFBOARD]			= "OFFBOARD";

	const char *arming_states_str[ARMING_STATE_MAX];
	arming_states_str[ARMING_STATE_INIT]			= "INIT";
	arming_states_str[ARMING_STATE_STANDBY]			= "STANDBY";
	arming_states_str[ARMING_STATE_ARMED]			= "ARMED";
	arming_states_str[ARMING_STATE_ARMED_ERROR]		= "ARMED_ERROR";
	arming_states_str[ARMING_STATE_STANDBY_ERROR]		= "STANDBY_ERROR";
	arming_states_str[ARMING_STATE_REBOOT]			= "REBOOT";
	arming_states_str[ARMING_STATE_IN_AIR_RESTORE]		= "IN_AIR_RESTORE";

	const char *nav_states_str[NAVIGATION_STATE_MAX];
	nav_states_str[NAVIGATION_STATE_MANUAL]			= "MANUAL";
	nav_states_str[NAVIGATION_STATE_ALTCTL]			= "ALTCTL";
	nav_states_str[NAVIGATION_STATE_POSCTL]			= "POSCTL";
	nav_states_str[NAVIGATION_STATE_AUTO_MISSION]		= "AUTO_MISSION";
	nav_states_str[NAVIGATION_STATE_AUTO_LOITER]		= "AUTO_LOITER";
	nav_states_str[NAVIGATION_STATE_AUTO_RTL]		= "AUTO_RTL";
	nav_states_str[NAVIGATION_STATE_AUTO_RCRECOVER]		= "AUTO_RCRECOVER";
	nav_states_str[NAVIGATION_STATE_AUTO_RTGS]		= "AUTO_RTGS";
	nav_states_str[NAVIGATION_STATE_AUTO_LANDENGFAIL]	= "AUTO_LANDENGFAIL";
	nav_states_str[NAVIGATION_STATE_AUTO_LANDGPSFAIL]	= "AUTO_LANDGPSFAIL";
	nav_states_str[NAVIGATION_STATE_ACRO]			= "ACRO";
	nav_states_str[NAVIGATION_STATE_LAND]			= "LAND";
	nav_states_str[NAVIGATION_STATE_DESCEND]		= "DESCEND";
	nav_states_str[NAVIGATION_STATE_TERMINATION]		= "TERMINATION";
	nav_states_str[NAVIGATION_STATE_OFFBOARD]		= "OFFBOARD";

	/* pthread for slow low prio thread */
	pthread_t commander_low_prio_thread;

	/* initialize */
	if (led_init() != 0) {
		warnx("ERROR: LED INIT FAIL");
	}

	if (buzzer_init() != OK) {
		warnx("ERROR: BUZZER INIT FAIL");
	}

	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* vehicle status topic */
	memset(&status, 0, sizeof(status));
	status.condition_landed = true;	// initialize to safe value
	// We want to accept RC inputs as default
	status.rc_input_blocked = false;
	status.main_state = MAIN_STATE_MANUAL;
	status.nav_state = NAVIGATION_STATE_MANUAL;
	status.arming_state = ARMING_STATE_INIT;
	status.hil_state = HIL_STATE_OFF;
	status.failsafe = false;

	/* neither manual nor offboard control commands have been received */
	status.offboard_control_signal_found_once = false;
	status.rc_signal_found_once = false;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = true;
	status.offboard_control_signal_lost = true;
	status.data_link_lost = true;

	/* set battery warning flag */
	status.battery_warning = VEHICLE_BATTERY_WARNING_NONE;
	status.condition_battery_voltage_valid = false;

	// XXX for now just set sensors as initialized
	status.condition_system_sensors_initialized = true;

	status.counter++;
	status.timestamp = hrt_absolute_time();

	status.condition_power_input_valid = true;
	status.avionics_power_rail_voltage = -1.0f;

	// CIRCUIT BREAKERS
	status.circuit_breaker_engaged_power_check = false;
	status.circuit_breaker_engaged_airspd_check = false;
	status.circuit_breaker_engaged_enginefailure_check = false;
	status.circuit_breaker_engaged_gpsfailure_check = false;

	/* publish initial state */
	status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

	if (status_pub < 0) {
		warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
		warnx("exiting.");
		exit(ERROR);
	}

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

	/* init mission state, do it here to allow navigator to use stored mission even if mavlink failed to start */
	orb_advert_t mission_pub = -1;
	mission_s mission;

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id >= 0 && mission.dataman_id <= 1) {
			warnx("loaded mission state: dataman_id=%d, count=%u, current=%d", mission.dataman_id, mission.count,
			      mission.current_seq);
			mavlink_log_info(mavlink_fd, "[cmd] dataman_id=%d, count=%u, current=%d",
					 mission.dataman_id, mission.count, mission.current_seq);

		} else {
			const char *missionfail = "reading mission state failed";
			warnx("%s", missionfail);
			mavlink_log_critical(mavlink_fd, missionfail);

			/* initialize mission state in dataman */
			mission.dataman_id = 0;
			mission.count = 0;
			mission.current_seq = 0;
			dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
		}

		mission_pub = orb_advertise(ORB_ID(offboard_mission), &mission);
		orb_publish(ORB_ID(offboard_mission), mission_pub, &mission);
	}

	int ret;

	pthread_attr_t commander_low_prio_attr;
	pthread_attr_init(&commander_low_prio_attr);
	pthread_attr_setstacksize(&commander_low_prio_attr, 2900);

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

	hrt_abstime last_idle_time = 0;
	hrt_abstime start_time = 0;

	bool status_changed = true;
	bool param_init_forced = true;

	bool updated = false;

	rc_calibration_check(mavlink_fd);

	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	memset(&safety, 0, sizeof(safety));
	safety.safety_switch_available = false;
	safety.safety_off = false;

	/* Subscribe to mission result topic */
	int mission_result_sub = orb_subscribe(ORB_ID(mission_result));
	struct mission_result_s mission_result;
	memset(&mission_result, 0, sizeof(mission_result));

	/* Subscribe to manual control data */
	int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s sp_man;
	memset(&sp_man, 0, sizeof(sp_man));

	/* Subscribe to offboard control data */
	int sp_offboard_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	memset(&sp_offboard, 0, sizeof(sp_offboard));

	/* Subscribe to telemetry status topics */
	int telemetry_subs[TELEMETRY_STATUS_ORB_ID_NUM];
	uint64_t telemetry_last_heartbeat[TELEMETRY_STATUS_ORB_ID_NUM];
	uint64_t telemetry_last_dl_loss[TELEMETRY_STATUS_ORB_ID_NUM];
	bool telemetry_lost[TELEMETRY_STATUS_ORB_ID_NUM];

	for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {
		telemetry_subs[i] = orb_subscribe(telemetry_status_orb_id[i]);
		telemetry_last_heartbeat[i] = 0;
		telemetry_last_dl_loss[i] = 0;
		telemetry_lost[i] = true;
	}

	/* Subscribe to global position */
	int global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_s global_position;
	memset(&global_position, 0, sizeof(global_position));
	/* Init EPH and EPV */
	global_position.eph = 1000.0f;
	global_position.epv = 1000.0f;

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
	gps_position.eph = FLT_MAX;
	gps_position.epv = FLT_MAX;

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

	/* Subscribe to position setpoint triplet */
	int pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	struct position_setpoint_triplet_s pos_sp_triplet;
	memset(&pos_sp_triplet, 0, sizeof(pos_sp_triplet));

	/* Subscribe to system power */
	int system_power_sub = orb_subscribe(ORB_ID(system_power));
	struct system_power_s system_power;
	memset(&system_power, 0, sizeof(system_power));

	/* Subscribe to actuator controls (outputs) */
	int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));

	control_status_leds(&status, &armed, true);

	/* now initialized */
	commander_initialized = true;
	thread_running = true;

	start_time = hrt_absolute_time();

	transition_result_t arming_ret;

	int32_t datalink_loss_enabled = false;
	int32_t datalink_loss_timeout = 10;
	float rc_loss_timeout = 0.5;
	int32_t datalink_regain_timeout = 0;

	/* Thresholds for engine failure detection */
	int32_t ef_throttle_thres = 1.0f;
	int32_t ef_current2throttle_thres = 0.0f;
	int32_t ef_time_thres = 1000.0f;
	uint64_t timestamp_engine_healthy = 0; /**< absolute time when engine was healty */

	/* check which state machines for changes, clear "changed" flag */
	bool arming_state_changed = false;
	bool main_state_changed = false;
	bool failsafe_old = false;

	while (!thread_should_exit) {

		if (mavlink_fd < 0 && counter % (1000000 / MAVLINK_OPEN_INTERVAL) == 0) {
			/* try to open the mavlink log device every once in a while */
			mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		arming_ret = TRANSITION_NOT_CHANGED;


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

				status.circuit_breaker_engaged_power_check =
					circuit_breaker_enabled("CBRK_SUPPLY_CHK", CBRK_SUPPLY_CHK_KEY);
				status.circuit_breaker_engaged_airspd_check =
					circuit_breaker_enabled("CBRK_AIRSPD_CHK", CBRK_AIRSPD_CHK_KEY);
				status.circuit_breaker_engaged_enginefailure_check =
					circuit_breaker_enabled("CBRK_ENGINEFAIL", CBRK_ENGINEFAIL_KEY);
				status.circuit_breaker_engaged_gpsfailure_check =
					circuit_breaker_enabled("CBRK_GPSFAIL", CBRK_GPSFAIL_KEY);

				status_changed = true;

				/* re-check RC calibration */
				rc_calibration_check(mavlink_fd);
			}

			/* navigation parameters */
			param_get(_param_takeoff_alt, &takeoff_alt);
			param_get(_param_enable_parachute, &parachute_enabled);
			param_get(_param_enable_datalink_loss, &datalink_loss_enabled);
			param_get(_param_datalink_loss_timeout, &datalink_loss_timeout);
			param_get(_param_rc_loss_timeout, &rc_loss_timeout);
			param_get(_param_datalink_regain_timeout, &datalink_regain_timeout);
			param_get(_param_ef_throttle_thres, &ef_throttle_thres);
			param_get(_param_ef_current2throttle_thres, &ef_current2throttle_thres);
			param_get(_param_ef_time_thres, &ef_time_thres);
		}

		orb_check(sp_man_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
		}

		orb_check(sp_offboard_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(offboard_control_setpoint), sp_offboard_sub, &sp_offboard);
		}

		if (sp_offboard.timestamp != 0 &&
		    sp_offboard.timestamp + OFFBOARD_TIMEOUT > hrt_absolute_time()) {
			if (status.offboard_control_signal_lost) {
				status.offboard_control_signal_lost = false;
				status_changed = true;
			}

		} else {
			if (!status.offboard_control_signal_lost) {
				status.offboard_control_signal_lost = true;
				status_changed = true;
			}
		}

		for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {
			orb_check(telemetry_subs[i], &updated);

			if (updated) {
				struct telemetry_status_s telemetry;
				memset(&telemetry, 0, sizeof(telemetry));

				orb_copy(telemetry_status_orb_id[i], telemetry_subs[i], &telemetry);

				/* perform system checks when new telemetry link connected */
				if (mavlink_fd &&
				    telemetry_last_heartbeat[i] == 0 &&
				    telemetry.heartbeat_time > 0 &&
				    hrt_elapsed_time(&telemetry.heartbeat_time) < datalink_loss_timeout * 1e6) {

					(void)rc_calibration_check(mavlink_fd);
				}

				telemetry_last_heartbeat[i] = telemetry.heartbeat_time;
			}
		}

		orb_check(sensor_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);

			/* Check if the barometer is healthy and issue a warning in the GCS if not so.
			 * Because the barometer is used for calculating AMSL altitude which is used to ensure
			 * vertical separation from other airtraffic the operator has to know when the
			 * barometer is inoperational.
			 * */
			if (hrt_elapsed_time(&sensors.baro_timestamp) < FAILSAFE_DEFAULT_TIMEOUT) {
				/* handle the case where baro was regained */
				if (status.barometer_failure) {
					status.barometer_failure = false;
					status_changed = true;
					mavlink_log_critical(mavlink_fd, "baro healthy");
				}

			} else {
				if (!status.barometer_failure) {
					status.barometer_failure = true;
					status_changed = true;
					mavlink_log_critical(mavlink_fd, "baro failed");
				}
			}
		}

		orb_check(diff_pres_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);
		}

		orb_check(system_power_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(system_power), system_power_sub, &system_power);

			if (hrt_elapsed_time(&system_power.timestamp) < 200000) {
				if (system_power.servo_valid &&
				    !system_power.brick_valid &&
				    !system_power.usb_connected) {
					/* flying only on servo rail, this is unsafe */
					status.condition_power_input_valid = false;

				} else {
					status.condition_power_input_valid = true;
				}

				/* copy avionics voltage */
				status.avionics_power_rail_voltage = system_power.voltage5V_v;
			}
		}

		check_valid(diff_pres.timestamp, DIFFPRESS_TIMEOUT, true, &(status.condition_airspeed_valid), &status_changed);

		/* update safety topic */
		orb_check(safety_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(safety), safety_sub, &safety);

			/* disarm if safety is now on and still armed */
			if (status.hil_state == HIL_STATE_OFF && safety.safety_switch_available && !safety.safety_off && armed.armed) {
				arming_state_t new_arming_state = (status.arming_state == ARMING_STATE_ARMED ? ARMING_STATE_STANDBY :
								   ARMING_STATE_STANDBY_ERROR);

				if (TRANSITION_CHANGED == arming_state_transition(&status, &safety, new_arming_state, &armed,
						true /* fRunPreArmChecks */, mavlink_fd)) {
					mavlink_log_info(mavlink_fd, "DISARMED by safety switch");
					arming_state_changed = true;
				}
			}
		}

		/* update global position estimate */
		orb_check(global_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
		}

		/* update local position estimate */
		orb_check(local_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
		}

		/* update condition_global_position_valid */
		/* hysteresis for EPH/EPV */
		bool eph_good;

		if (status.condition_global_position_valid) {
			if (global_position.eph > eph_threshold * 2.5f) {
				eph_good = false;

			} else {
				eph_good = true;
			}

		} else {
			if (global_position.eph < eph_threshold) {
				eph_good = true;

			} else {
				eph_good = false;
			}
		}

		check_valid(global_position.timestamp, POSITION_TIMEOUT, eph_good, &(status.condition_global_position_valid),
			    &status_changed);

		/* update home position */
		if (!status.condition_home_position_valid && status.condition_global_position_valid && !armed.armed &&
		    (global_position.eph < eph_threshold) && (global_position.epv < epv_threshold)) {

			home.lat = global_position.lat;
			home.lon = global_position.lon;
			home.alt = global_position.alt;

			home.x = local_position.x;
			home.y = local_position.y;
			home.z = local_position.z;

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

		/* update condition_local_position_valid and condition_local_altitude_valid */
		/* hysteresis for EPH */
		bool local_eph_good;

		if (status.condition_local_position_valid) {
			if (local_position.eph > eph_threshold * 2.5f) {
				local_eph_good = false;

			} else {
				local_eph_good = true;
			}

		} else {
			if (local_position.eph < eph_threshold) {
				local_eph_good = true;

			} else {
				local_eph_good = false;
			}
		}

		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.xy_valid
			    && local_eph_good, &(status.condition_local_position_valid), &status_changed);
		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.z_valid,
			    &(status.condition_local_altitude_valid), &status_changed);

		if (status.condition_local_altitude_valid) {
			if (status.condition_landed != local_position.landed) {
				status.condition_landed = local_position.landed;
				status_changed = true;

				if (status.condition_landed) {
					mavlink_log_critical(mavlink_fd, "LANDED MODE");

				} else {
					mavlink_log_critical(mavlink_fd, "IN AIR MODE");
				}
			}
		}

		/* update battery status */
		orb_check(battery_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);
			orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_controls_sub, &actuator_controls);

			/* only consider battery voltage if system has been running 2s and battery voltage is valid */
			if (hrt_absolute_time() > start_time + 2000000 && battery.voltage_filtered_v > 0.0f) {
				status.battery_voltage = battery.voltage_filtered_v;
				status.battery_current = battery.current_a;
				status.condition_battery_voltage_valid = true;

				/* get throttle (if armed), as we only care about energy negative throttle also counts */
				float throttle = (armed.armed) ? fabsf(actuator_controls.control[3]) : 0.0f;
				status.battery_remaining = battery_remaining_estimate_voltage(battery.voltage_filtered_v, battery.discharged_mah,
							   throttle);
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

		/* update position setpoint triplet */
		orb_check(pos_sp_triplet_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(position_setpoint_triplet), pos_sp_triplet_sub, &pos_sp_triplet);
		}

		if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
			/* compute system load */
			uint64_t interval_runtime = system_load.tasks[0].total_runtime - last_idle_time;

			if (last_idle_time > 0) {
				status.load = 1.0f - ((float)interval_runtime / 1e6f);        //system load is time spent in non-idle
			}

			last_idle_time = system_load.tasks[0].total_runtime;

			/* check if board is connected via USB */
			struct stat statbuf;
			on_usb_power = (stat("/dev/ttyACM0", &statbuf) == 0);
		}

		/* if battery voltage is getting lower, warn using buzzer, etc. */
		if (status.condition_battery_voltage_valid && status.battery_remaining < 0.18f && !low_battery_voltage_actions_done) {
			low_battery_voltage_actions_done = true;
			mavlink_log_critical(mavlink_fd, "LOW BATTERY, RETURN TO LAND ADVISED");
			status.battery_warning = VEHICLE_BATTERY_WARNING_LOW;
			status_changed = true;

		} else if (!on_usb_power && status.condition_battery_voltage_valid && status.battery_remaining < 0.09f
			   && !critical_battery_voltage_actions_done && low_battery_voltage_actions_done) {
			/* critical battery voltage, this is rather an emergency, change state machine */
			critical_battery_voltage_actions_done = true;
			mavlink_log_emergency(mavlink_fd, "CRITICAL BATTERY, LAND IMMEDIATELY");
			status.battery_warning = VEHICLE_BATTERY_WARNING_CRITICAL;

			if (armed.armed) {
				arming_ret = arming_state_transition(&status, &safety, ARMING_STATE_ARMED_ERROR, &armed, true /* fRunPreArmChecks */,
								     mavlink_fd);

				if (arming_ret == TRANSITION_CHANGED) {
					arming_state_changed = true;
				}

			} else {
				arming_ret = arming_state_transition(&status, &safety, ARMING_STATE_STANDBY_ERROR, &armed, true /* fRunPreArmChecks */,
								     mavlink_fd);

				if (arming_ret == TRANSITION_CHANGED) {
					arming_state_changed = true;
				}
			}

			status_changed = true;
		}

		/* End battery voltage check */

		/* If in INIT state, try to proceed to STANDBY state */
		if (status.arming_state == ARMING_STATE_INIT && low_prio_task == LOW_PRIO_TASK_NONE) {
			/* TODO: check for sensors */
			arming_ret = arming_state_transition(&status, &safety, ARMING_STATE_STANDBY, &armed, true /* fRunPreArmChecks */,
							     mavlink_fd);

			if (arming_ret == TRANSITION_CHANGED) {
				arming_state_changed = true;
			}

		} else {
			/* TODO: Add emergency stuff if sensors are lost */
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

		/* Initialize map projection if gps is valid */
		if (!map_projection_global_initialized()
		    && (gps_position.eph < eph_threshold)
		    && (gps_position.epv < epv_threshold)
		    && hrt_elapsed_time((hrt_abstime *)&gps_position.timestamp_position) < 1e6) {
			/* set reference for global coordinates <--> local coordiantes conversion and map_projection */
			globallocalconverter_init((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
						  (float)gps_position.alt * 1.0e-3f, hrt_absolute_time());
		}

		/* check if GPS fix is ok */
		if (status.circuit_breaker_engaged_gpsfailure_check ||
		    (gps_position.fix_type >= 3 &&
		     hrt_elapsed_time(&gps_position.timestamp_position) < FAILSAFE_DEFAULT_TIMEOUT)) {
			/* handle the case where gps was regained */
			if (status.gps_failure) {
				status.gps_failure = false;
				status_changed = true;
				mavlink_log_critical(mavlink_fd, "gps regained");
			}

		} else {
			if (!status.gps_failure) {
				status.gps_failure = true;
				status_changed = true;
				mavlink_log_critical(mavlink_fd, "gps fix lost");
			}
		}

		/* start mission result check */
		orb_check(mission_result_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(mission_result), mission_result_sub, &mission_result);

			/* Check for geofence violation */
			if (armed.armed && (mission_result.geofence_violated || mission_result.flight_termination)) {
				//XXX: make this configurable to select different actions (e.g. navigation modes)
				/* this will only trigger if geofence is activated via param and a geofence file is present, also there is a circuit breaker to disable the actual flight termination in the px4io driver */
				armed.force_failsafe = true;
				status_changed = true;
				static bool flight_termination_printed = false;

				if (!flight_termination_printed) {
					warnx("Flight termination because of navigator request or geofence");
					mavlink_log_critical(mavlink_fd, "GF violation: flight termination");
					flight_termination_printed = true;
				}

				if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
					mavlink_log_critical(mavlink_fd, "GF violation: flight termination");
				}
			} // no reset is done here on purpose, on geofence violation we want to stay in flighttermination
		}

		/* RC input check */
		if (!status.rc_input_blocked && sp_man.timestamp != 0 &&
		    hrt_absolute_time() < sp_man.timestamp + (uint64_t)(rc_loss_timeout * 1e6f)) {
			/* handle the case where RC signal was regained */
			if (!status.rc_signal_found_once) {
				status.rc_signal_found_once = true;
				mavlink_log_critical(mavlink_fd, "detected RC signal first time");
				status_changed = true;

			} else {
				if (status.rc_signal_lost) {
					mavlink_log_critical(mavlink_fd, "RC SIGNAL REGAINED after %llums",(hrt_absolute_time()-status.rc_signal_lost_timestamp)/1000);
					status_changed = true;
				}
			}

			status.rc_signal_lost = false;

			/* check if left stick is in lower left position and we are in MANUAL or AUTO_READY mode or (ASSIST mode and landed) -> disarm
			 * do it only for rotary wings */
			if (status.is_rotary_wing &&
			    (status.arming_state == ARMING_STATE_ARMED || status.arming_state == ARMING_STATE_ARMED_ERROR) &&
			    (status.main_state == MAIN_STATE_MANUAL || status.main_state == MAIN_STATE_ACRO || status.condition_landed) &&
			    sp_man.r < -STICK_ON_OFF_LIMIT && sp_man.z < 0.1f) {

				if (stick_off_counter > STICK_ON_OFF_COUNTER_LIMIT) {
					/* disarm to STANDBY if ARMED or to STANDBY_ERROR if ARMED_ERROR */
					arming_state_t new_arming_state = (status.arming_state == ARMING_STATE_ARMED ? ARMING_STATE_STANDBY :
									   ARMING_STATE_STANDBY_ERROR);
					arming_ret = arming_state_transition(&status, &safety, new_arming_state, &armed, true /* fRunPreArmChecks */,
									     mavlink_fd);

					if (arming_ret == TRANSITION_CHANGED) {
						arming_state_changed = true;
					}

					stick_off_counter = 0;

				} else {
					stick_off_counter++;
				}

			} else {
				stick_off_counter = 0;
			}

			/* check if left stick is in lower right position and we're in MANUAL mode -> arm */
			if (status.arming_state == ARMING_STATE_STANDBY &&
			    sp_man.r > STICK_ON_OFF_LIMIT && sp_man.z < 0.1f) {
				if (stick_on_counter > STICK_ON_OFF_COUNTER_LIMIT) {

					/* we check outside of the transition function here because the requirement
					 * for being in manual mode only applies to manual arming actions.
					 * the system can be armed in auto if armed via the GCS.
					 */
					if (status.main_state != MAIN_STATE_MANUAL) {
						print_reject_arm("NOT ARMING: Switch to MANUAL mode first.");

					} else {
						arming_ret = arming_state_transition(&status, &safety, ARMING_STATE_ARMED, &armed, true /* fRunPreArmChecks */,
										     mavlink_fd);

						if (arming_ret == TRANSITION_CHANGED) {
							arming_state_changed = true;
						}
					}

					stick_on_counter = 0;

				} else {
					stick_on_counter++;
				}

			} else {
				stick_on_counter = 0;
			}

			if (arming_ret == TRANSITION_CHANGED) {
				if (status.arming_state == ARMING_STATE_ARMED) {
					mavlink_log_info(mavlink_fd, "ARMED by RC");

				} else {
					mavlink_log_info(mavlink_fd, "DISARMED by RC");
				}

				arming_state_changed = true;

			} else if (arming_ret == TRANSITION_DENIED) {
				/*
				 * the arming transition can be denied to a number of reasons:
				 *  - pre-flight check failed (sensors not ok or not calibrated)
				 *  - safety not disabled
				 *  - system not in manual mode
				 */
				tune_negative(true);
			}

			/* evaluate the main state machine according to mode switches */
			transition_result_t main_res = set_main_state_rc(&status, &sp_man);

			/* play tune on mode change only if armed, blink LED always */
			if (main_res == TRANSITION_CHANGED) {
				tune_positive(armed.armed);
				main_state_changed = true;

			} else if (main_res == TRANSITION_DENIED) {
				/* DENIED here indicates bug in the commander */
				mavlink_log_critical(mavlink_fd, "main state transition denied");
			}

		} else {
			if (!status.rc_signal_lost) {
				mavlink_log_critical(mavlink_fd, "RC SIGNAL LOST (at t=%llums)",hrt_absolute_time()/1000);
				status.rc_signal_lost = true;
				status.rc_signal_lost_timestamp=sp_man.timestamp;
				status_changed = true;
			}
		}

		/* data links check */
		bool have_link = false;

		for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {
			if (telemetry_last_heartbeat[i] != 0 &&
			    hrt_elapsed_time(&telemetry_last_heartbeat[i]) < datalink_loss_timeout * 1e6) {
				/* handle the case where data link was regained,
				 * accept datalink as healthy only after datalink_regain_timeout seconds
				 * */
				if (telemetry_lost[i] &&
				    hrt_elapsed_time(&telemetry_last_dl_loss[i]) > datalink_regain_timeout * 1e6) {

					mavlink_log_info(mavlink_fd, "data link %i regained", i);
					telemetry_lost[i] = false;
					have_link = true;

				} else if (!telemetry_lost[i]) {
					/* telemetry was healthy also in last iteration
					 * we don't have to check a timeout */
					have_link = true;
				}

			} else {
				telemetry_last_dl_loss[i]  = hrt_absolute_time();

				if (!telemetry_lost[i]) {
					mavlink_log_info(mavlink_fd, "data link %i lost", i);
					telemetry_lost[i] = true;
				}
			}
		}

		if (have_link) {
			/* handle the case where data link was regained */
			if (status.data_link_lost) {
				status.data_link_lost = false;
				status_changed = true;
			}

		} else {
			if (!status.data_link_lost) {
				mavlink_log_info(mavlink_fd, "ALL DATA LINKS LOST");
				status.data_link_lost = true;
				status.data_link_lost_counter++;
				status_changed = true;
			}
		}

		/* Check engine failure
		 * only for fixed wing for now
		 */
		if (!status.circuit_breaker_engaged_enginefailure_check &&
		    status.is_rotary_wing == false &&
		    armed.armed &&
		    ((actuator_controls.control[3] > ef_throttle_thres &&
		      battery.current_a / actuator_controls.control[3] <
		      ef_current2throttle_thres) ||
		     (status.engine_failure))) {
			/* potential failure, measure time */
			if (timestamp_engine_healthy > 0 &&
			    hrt_elapsed_time(&timestamp_engine_healthy) >
			    ef_time_thres * 1e6 &&
			    !status.engine_failure) {
				status.engine_failure = true;
				status_changed = true;
				mavlink_log_critical(mavlink_fd, "Engine Failure");
			}

		} else {
			/* no failure reset flag */
			timestamp_engine_healthy = hrt_absolute_time();

			if (status.engine_failure) {
				status.engine_failure = false;
				status_changed = true;
			}
		}


		/* handle commands last, as the system needs to be updated to handle them */
		orb_check(cmd_sub, &updated);

		if (updated) {
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			if (handle_command(&status, &safety, &cmd, &armed, &home, &global_position, &home_pub)) {
				status_changed = true;
			}
		}

		/* Check for failure combinations which lead to flight termination */
		if (armed.armed) {
			/* At this point the data link and the gps system have been checked
			 * If  we are not in a manual (RC stick controlled mode)
			 * and both failed we want to terminate the flight */
			if (status.main_state != MAIN_STATE_MANUAL &&
			    status.main_state != MAIN_STATE_ACRO &&
			    status.main_state != MAIN_STATE_ALTCTL &&
			    status.main_state != MAIN_STATE_POSCTL &&
			    ((status.data_link_lost && status.gps_failure) ||
			     (status.data_link_lost_cmd && status.gps_failure_cmd))) {
				armed.force_failsafe = true;
				status_changed = true;
				static bool flight_termination_printed = false;

				if (!flight_termination_printed) {
					warnx("Flight termination because of data link loss && gps failure");
					mavlink_log_critical(mavlink_fd, "DL and GPS lost: flight termination");
					flight_termination_printed = true;
				}

				if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
					mavlink_log_critical(mavlink_fd, "DL and GPS lost: flight termination");
				}
			}

			/* At this point the rc signal and the gps system have been checked
			 * If we are in manual (controlled with RC):
			 * if both failed we want to terminate the flight */
			if ((status.main_state == MAIN_STATE_ACRO ||
			     status.main_state == MAIN_STATE_MANUAL ||
			     status.main_state == MAIN_STATE_ALTCTL ||
			     status.main_state == MAIN_STATE_POSCTL) &&
			    ((status.rc_signal_lost && status.gps_failure) ||
			     (status.rc_signal_lost_cmd && status.gps_failure_cmd))) {
				armed.force_failsafe = true;
				status_changed = true;
				static bool flight_termination_printed = false;

				if (!flight_termination_printed) {
					warnx("Flight termination because of RC signal loss && gps failure");
					flight_termination_printed = true;
				}

				if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
					mavlink_log_critical(mavlink_fd, "RC and GPS lost: flight termination");
				}
			}
		}


		hrt_abstime t1 = hrt_absolute_time();

		/* print new state */
		if (arming_state_changed) {
			status_changed = true;
			mavlink_log_info(mavlink_fd, "[cmd] arming state: %s", arming_states_str[status.arming_state]);

			/* update home position on arming if at least 2s from commander start spent to avoid setting home on in-air restart */
			if (armed.armed && !was_armed && hrt_absolute_time() > start_time + 2000000 && status.condition_global_position_valid &&
			    (global_position.eph < eph_threshold) && (global_position.epv < epv_threshold)) {

				// TODO remove code duplication
				home.lat = global_position.lat;
				home.lon = global_position.lon;
				home.alt = global_position.alt;

				home.x = local_position.x;
				home.y = local_position.y;
				home.z = local_position.z;

				warnx("home: lat = %.7f, lon = %.7f, alt = %.2f ", home.lat, home.lon, (double)home.alt);
				mavlink_log_info(mavlink_fd, "home: %.7f, %.7f, %.2f", home.lat, home.lon, (double)home.alt);

				/* announce new home position */
				if (home_pub > 0) {
					orb_publish(ORB_ID(home_position), home_pub, &home);

				} else {
					home_pub = orb_advertise(ORB_ID(home_position), &home);
				}

				/* mark home position as set */
				status.condition_home_position_valid = true;
			}

			arming_state_changed = false;
		}

		was_armed = armed.armed;

		/* now set navigation state according to failsafe and main state */
		bool nav_state_changed = set_nav_state(&status, (bool)datalink_loss_enabled,
						       mission_result.finished,
						       mission_result.stay_in_failsafe);

		// TODO handle mode changes by commands
		if (main_state_changed) {
			status_changed = true;
			warnx("main state: %s", main_states_str[status.main_state]);
			mavlink_log_info(mavlink_fd, "[cmd] main state: %s", main_states_str[status.main_state]);
			main_state_changed = false;
		}

		if (status.failsafe != failsafe_old) {
			status_changed = true;
			if (status.failsafe) {
				mavlink_log_critical(mavlink_fd, "failsafe mode on");
			} else {
				mavlink_log_critical(mavlink_fd, "failsafe mode off");
			}
			failsafe_old = status.failsafe;
		}

		if (nav_state_changed) {
			status_changed = true;
			warnx("nav state: %s", nav_states_str[status.nav_state]);
			mavlink_log_info(mavlink_fd, "[cmd] nav state: %s", nav_states_str[status.nav_state]);
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
		if (!arm_tune_played && armed.armed && (!safety.safety_switch_available || (safety.safety_switch_available
							&& safety.safety_off))) {
			/* play tune when armed */
			set_tune(TONE_ARMING_WARNING_TUNE);
			arm_tune_played = true;

		} else if (status.battery_warning == VEHICLE_BATTERY_WARNING_CRITICAL) {
			/* play tune on battery critical */
			set_tune(TONE_BATTERY_WARNING_FAST_TUNE);

		} else if (status.battery_warning == VEHICLE_BATTERY_WARNING_LOW || status.failsafe) {
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
	close(mission_pub);

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
control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed, bool changed)
{
	/* driving rgbled */
	if (changed) {
		bool set_normal_color = false;

		/* set mode */
		if (status_local->arming_state == ARMING_STATE_ARMED) {
			rgbled_set_mode(RGBLED_MODE_ON);
			set_normal_color = true;

		} else if (status_local->arming_state == ARMING_STATE_ARMED_ERROR) {
			rgbled_set_mode(RGBLED_MODE_BLINK_FAST);
			rgbled_set_color(RGBLED_COLOR_RED);

		} else if (status_local->arming_state == ARMING_STATE_STANDBY) {
			rgbled_set_mode(RGBLED_MODE_BREATHE);
			set_normal_color = true;

		} else {	// STANDBY_ERROR and other states
			rgbled_set_mode(RGBLED_MODE_BLINK_NORMAL);
			rgbled_set_color(RGBLED_COLOR_RED);
		}

		if (set_normal_color) {
			/* set color */
			if (status_local->battery_warning == VEHICLE_BATTERY_WARNING_LOW || status_local->failsafe) {
				rgbled_set_color(RGBLED_COLOR_AMBER);
				/* VEHICLE_BATTERY_WARNING_CRITICAL handled as ARMING_STATE_ARMED_ERROR / ARMING_STATE_STANDBY_ERROR */

			} else {
				if (status_local->condition_local_position_valid) {
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
		if (leds_counter % 20 == 0) {
			led_toggle(LED_BLUE);
		}

	} else {
		/* not ready to arm, blink at 10Hz */
		if (leds_counter % 2 == 0) {
			led_toggle(LED_BLUE);
		}
	}

#endif

	/* give system warnings on error LED, XXX maybe add memory usage warning too */
	if (status_local->load > 0.95f) {
		if (leds_counter % 2 == 0) {
			led_toggle(LED_AMBER);
		}

	} else {
		led_off(LED_AMBER);
	}

	leds_counter++;
}

transition_result_t
set_main_state_rc(struct vehicle_status_s *status_local, struct manual_control_setpoint_s *sp_man)
{
	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_DENIED;

	/* if offboard is set allready by a mavlink command, abort */
	if (status.offboard_control_set_by_command) {
		return main_state_transition(status_local, MAIN_STATE_OFFBOARD);
	}

	/* offboard switch overrides main switch */
	if (sp_man->offboard_switch == SWITCH_POS_ON) {
		res = main_state_transition(status_local, MAIN_STATE_OFFBOARD);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "OFFBOARD");

		} else {
			return res;
		}
	}

	/* offboard switched off or denied, check main mode switch */
	switch (sp_man->mode_switch) {
	case SWITCH_POS_NONE:
		res = TRANSITION_NOT_CHANGED;
		break;

	case SWITCH_POS_OFF:		// MANUAL
		if (sp_man->acro_switch == SWITCH_POS_ON) {
			res = main_state_transition(status_local, MAIN_STATE_ACRO);

		} else {
			res = main_state_transition(status_local, MAIN_STATE_MANUAL);
		}

		// TRANSITION_DENIED is not possible here
		break;

	case SWITCH_POS_MIDDLE:		// ASSIST
		if (sp_man->posctl_switch == SWITCH_POS_ON) {
			res = main_state_transition(status_local, MAIN_STATE_POSCTL);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "POSCTL");
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, MAIN_STATE_ALTCTL);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this mode
		}

		if (sp_man->posctl_switch != SWITCH_POS_ON) {
			print_reject_mode(status_local, "ALTCTL");
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	case SWITCH_POS_ON:			// AUTO
		if (sp_man->return_switch == SWITCH_POS_ON) {
			res = main_state_transition(status_local, MAIN_STATE_AUTO_RTL);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "AUTO_RTL");

			// fallback to LOITER if home position not set
			res = main_state_transition(status_local, MAIN_STATE_AUTO_LOITER);

			if (res != TRANSITION_DENIED) {
				break;  // changed successfully or already in this state
			}

		} else if (sp_man->loiter_switch == SWITCH_POS_ON) {
			res = main_state_transition(status_local, MAIN_STATE_AUTO_LOITER);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "AUTO_LOITER");

		} else {
			res = main_state_transition(status_local, MAIN_STATE_AUTO_MISSION);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "AUTO_MISSION");

			// fallback to LOITER if home position not set
			res = main_state_transition(status_local, MAIN_STATE_AUTO_LOITER);

			if (res != TRANSITION_DENIED) {
				break;  // changed successfully or already in this state
			}
		}

		// fallback to POSCTL
		res = main_state_transition(status_local, MAIN_STATE_POSCTL);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, MAIN_STATE_ALTCTL);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, MAIN_STATE_MANUAL);
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
	/* set vehicle_control_mode according to set_navigation_state */
	control_mode.flag_armed = armed.armed;
	/* TODO: check this */
	control_mode.flag_external_manual_override_ok = !status.is_rotary_wing;
	control_mode.flag_system_hil_enabled = status.hil_state == HIL_STATE_ON;
	control_mode.flag_control_offboard_enabled = false;

	switch (status.nav_state) {
	case NAVIGATION_STATE_MANUAL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = status.is_rotary_wing;
		control_mode.flag_control_attitude_enabled = status.is_rotary_wing;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case NAVIGATION_STATE_ALTCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case NAVIGATION_STATE_POSCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case NAVIGATION_STATE_AUTO_MISSION:
	case NAVIGATION_STATE_AUTO_LOITER:
	case NAVIGATION_STATE_AUTO_RTL:
	case NAVIGATION_STATE_AUTO_RCRECOVER:
	case NAVIGATION_STATE_AUTO_RTGS:
	case NAVIGATION_STATE_AUTO_LANDENGFAIL:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case NAVIGATION_STATE_AUTO_LANDGPSFAIL:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case NAVIGATION_STATE_ACRO:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;


	case NAVIGATION_STATE_LAND:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		/* in failsafe LAND mode position may be not available */
		control_mode.flag_control_position_enabled = status.condition_local_position_valid;
		control_mode.flag_control_velocity_enabled = status.condition_local_position_valid;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case NAVIGATION_STATE_DESCEND:
		/* TODO: check if this makes sense */
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case NAVIGATION_STATE_TERMINATION:
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

	case NAVIGATION_STATE_OFFBOARD:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_offboard_enabled = true;

		switch (sp_offboard.mode) {
		case OFFBOARD_CONTROL_MODE_DIRECT_RATES:
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_attitude_enabled = false;
			control_mode.flag_control_altitude_enabled = false;
			control_mode.flag_control_climb_rate_enabled = false;
			control_mode.flag_control_position_enabled = false;
			control_mode.flag_control_velocity_enabled = false;
			break;

		case OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE:
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_altitude_enabled = false;
			control_mode.flag_control_climb_rate_enabled = false;
			control_mode.flag_control_position_enabled = false;
			control_mode.flag_control_velocity_enabled = false;
			break;

		case OFFBOARD_CONTROL_MODE_DIRECT_FORCE:
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_attitude_enabled = false;
			control_mode.flag_control_force_enabled = true;
			control_mode.flag_control_altitude_enabled = false;
			control_mode.flag_control_climb_rate_enabled = false;
			control_mode.flag_control_position_enabled = false;
			control_mode.flag_control_velocity_enabled = false;
			break;

		case OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED:
		case OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_OFFSET_NED:
		case OFFBOARD_CONTROL_MODE_DIRECT_BODY_NED:
		case OFFBOARD_CONTROL_MODE_DIRECT_BODY_OFFSET_NED:
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_altitude_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_position_enabled = true;
			control_mode.flag_control_velocity_enabled = true;
			//XXX: the flags could depend on sp_offboard.ignore
			break;

		default:
			control_mode.flag_control_rates_enabled = false;
			control_mode.flag_control_attitude_enabled = false;
			control_mode.flag_control_altitude_enabled = false;
			control_mode.flag_control_climb_rate_enabled = false;
			control_mode.flag_control_position_enabled = false;
			control_mode.flag_control_velocity_enabled = false;
		}
		break;

	default:
		break;
	}
}

void
print_reject_mode(struct vehicle_status_s *status_local, const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		mavlink_log_critical(mavlink_fd, "REJECT %s", msg);

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
		mavlink_log_critical(mavlink_fd, msg);
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
		mavlink_log_critical(mavlink_fd, "command denied: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(mavlink_fd, "command failed: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		/* this needs additional hints to the user - so let other messages pass and be spoken */
		mavlink_log_critical(mavlink_fd, "command temporarily rejected: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(mavlink_fd, "command unsupported: %u", cmd.command);
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
		if (pret == 0) {
			continue;
		}

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
		    cmd.command == VEHICLE_CMD_DO_SET_SERVO) {
			continue;
		}

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
				if (TRANSITION_DENIED == arming_state_transition(&status, &safety, ARMING_STATE_INIT, &armed,
						true /* fRunPreArmChecks */, mavlink_fd)) {
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

				if (calib_ret == OK) {
					tune_positive(true);

				} else {
					tune_negative(true);
				}

				arming_state_transition(&status, &safety, ARMING_STATE_STANDBY, &armed, true /* fRunPreArmChecks */, mavlink_fd);

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
						if (ret < 0) {
							ret = -ret;
						}

						if (ret < 1000) {
							mavlink_log_critical(mavlink_fd, "#audio: %s", strerror(ret));
						}

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
						if (ret < 0) {
							ret = -ret;
						}

						if (ret < 1000) {
							mavlink_log_critical(mavlink_fd, "#audio: %s", strerror(ret));
						}

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

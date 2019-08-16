/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 *
 * Main state machine / business logic
 *
 * @TODO This application is currently in a rewrite process. Main changes:
 *			- Calibration routines are moved into the event system
 *			- Commander is rewritten as class
 *			- State machines will be model driven
 */

#include "Commander.hpp"

/* commander module headers */
#include "accelerometer_calibration.h"
#include "airspeed_calibration.h"
#include "arm_auth.h"
#include "baro_calibration.h"
#include "calibration_routines.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "PreflightCheck.h"
#include "px4_custom_mode.h"
#include "rc_calibration.h"
#include "state_machine_helper.h"
#include "health_flag_helper.h"

/* PX4 headers */
#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <navigator/navigation.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <circuit_breaker/circuit_breaker.h>
#include <systemlib/mavlink_log.h>

#include <cmath>
#include <float.h>
#include <cstring>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/esc_status.h>

typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	VEHICLE_MODE_FLAG_ENUM_END = 129, /*  | */
} VEHICLE_MODE_FLAG;

/* Decouple update interval and hysteresis counters, all depends on intervals */
static constexpr uint64_t COMMANDER_MONITORING_INTERVAL = 10_ms;
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))

static constexpr float STICK_ON_OFF_LIMIT = 0.9f;

static constexpr uint64_t OFFBOARD_TIMEOUT = 500_ms;
static constexpr uint64_t HOTPLUG_SENS_TIMEOUT = 8_s;	/**< wait for hotplug sensors to come online for upto 8 seconds */
static constexpr uint64_t PRINT_MODE_REJECT_INTERVAL = 500_ms;
static constexpr uint64_t INAIR_RESTART_HOLDOFF_INTERVAL = 500_ms;

/* Mavlink log uORB handle */
static orb_advert_t mavlink_log_pub = nullptr;
static orb_advert_t power_button_state_pub = nullptr;

/* flags */
static volatile bool thread_should_exit = false;	/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */

static hrt_abstime commander_boot_timestamp = 0;

static unsigned int leds_counter;
/* To remember when last notification was sent */
static uint64_t last_print_mode_reject_time = 0;

static float min_stick_change = 0.25f;

static struct vehicle_status_s status = {};
static struct actuator_armed_s armed = {};
static struct safety_s safety = {};
static int32_t _flight_mode_slots[manual_control_setpoint_s::MODE_SLOT_NUM];
static struct commander_state_s internal_state = {};

static uint8_t main_state_before_rtl = commander_state_s::MAIN_STATE_MAX;

static manual_control_setpoint_s sp_man = {};		///< the current manual control setpoint
static manual_control_setpoint_s _last_sp_man = {};	///< the manual control setpoint valid at the last mode switch
static uint8_t _last_sp_man_arm_switch = 0;

static struct vtol_vehicle_status_s vtol_status = {};
static struct cpuload_s cpuload = {};

static bool last_overload = false;

static struct vehicle_status_flags_s status_flags = {};

static uint64_t rc_signal_lost_timestamp;		// Time at which the RC reception was lost

static uint8_t arm_requirements = ARM_REQ_NONE;

static bool _last_condition_local_altitude_valid = false;
static bool _last_condition_global_position_valid = false;

static struct vehicle_land_detected_s land_detector = {};

static float _eph_threshold_adj =
	INFINITY;	///< maximum allowable horizontal position uncertainty after adjustment for flight condition
static bool _skip_pos_accuracy_check = false;

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

void control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed, bool changed,
			 const uint8_t battery_warning, const cpuload_s *cpuload_local);

void get_circuit_breaker_params();

bool stabilization_required();

void print_reject_mode(const char *msg);

void print_reject_arm(const char *msg);

void print_status();

bool shutdown_if_allowed();

transition_result_t arm_disarm(bool arm, bool run_preflight_checks, orb_advert_t *mavlink_log_pub, const char *armedBy);

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);

static void answer_command(const vehicle_command_s &cmd, unsigned result,
			   uORB::PublicationQueued<vehicle_command_ack_s> &command_ack_pub);

static int power_button_state_notification_cb(board_power_button_state_notification_e request)
{
	// Note: this can be called from IRQ handlers, so we publish a message that will be handled
	// on the main thread of commander.
	power_button_state_s button_state{};
	button_state.timestamp = hrt_absolute_time();
	int ret = PWR_BUTTON_RESPONSE_SHUT_DOWN_PENDING;

	switch (request) {
	case PWR_BUTTON_IDEL:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_IDEL;
		break;

	case PWR_BUTTON_DOWN:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_DOWN;
		break;

	case PWR_BUTTON_UP:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_UP;
		break;

	case PWR_BUTTON_REQUEST_SHUT_DOWN:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN;
		break;

	default:
		PX4_ERR("unhandled power button state: %i", (int)request);
		return ret;
	}

	if (power_button_state_pub != nullptr) {
		orb_publish(ORB_ID(power_button_state), power_button_state_pub, &button_state);

	} else {
		PX4_ERR("power_button_state_pub not properly initialized");
	}

	return ret;
}

static bool send_vehicle_command(uint16_t cmd, float param1 = NAN, float param2 = NAN)
{
	vehicle_command_s vcmd{};

	vcmd.timestamp = hrt_absolute_time();
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = NAN;
	vcmd.param4 = NAN;
	vcmd.param5 = (double)NAN;
	vcmd.param6 = (double)NAN;
	vcmd.param7 = NAN;
	vcmd.command = cmd;
	vcmd.target_system = status.system_id;
	vcmd.target_component = status.component_id;

	uORB::PublicationQueued<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};

	return vcmd_pub.publish(vcmd);
}

int commander_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;

		Commander::main(argc, argv);

		unsigned constexpr max_wait_us = 1000000;
		unsigned constexpr max_wait_steps = 2000;

		unsigned i;

		for (i = 0; i < max_wait_steps; i++) {
			px4_usleep(max_wait_us / max_wait_steps);

			if (thread_running) {
				break;
			}
		}

		return !(i < max_wait_steps);
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running) {
			PX4_WARN("already stopped");
			return 0;
		}

		thread_should_exit = true;

		Commander::main(argc, argv);

		PX4_INFO("terminated.");

		return 0;
	}

	/* commands needing the app to run below */
	if (!thread_running) {
		PX4_ERR("not started");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		print_status();
		return 0;
	}

	if (!strcmp(argv[1], "calibrate")) {
		if (argc > 2) {
			int calib_ret = OK;

			if (!strcmp(argv[2], "mag")) {
				calib_ret = do_mag_calibration(&mavlink_log_pub);

			} else if (!strcmp(argv[2], "accel")) {
				calib_ret = do_accel_calibration(&mavlink_log_pub);

			} else if (!strcmp(argv[2], "gyro")) {
				calib_ret = do_gyro_calibration(&mavlink_log_pub);

			} else if (!strcmp(argv[2], "level")) {
				calib_ret = do_level_calibration(&mavlink_log_pub);

			} else if (!strcmp(argv[2], "esc")) {
				calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);

			} else if (!strcmp(argv[2], "airspeed")) {
				calib_ret = do_airspeed_calibration(&mavlink_log_pub);

			} else {
				PX4_ERR("argument %s unsupported.", argv[2]);
			}

			if (calib_ret) {
				PX4_ERR("calibration failed, exiting.");
				return 1;

			} else {
				return 0;
			}

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[1], "check")) {
		bool preflight_check_res = Commander::preflight_check(true);
		PX4_INFO("Preflight check: %s", preflight_check_res ? "OK" : "FAILED");

		bool prearm_check_res = prearm_check(&mavlink_log_pub, status_flags, safety, arm_requirements);
		PX4_INFO("Prearm check: %s", prearm_check_res ? "OK" : "FAILED");

		return 0;
	}

	if (!strcmp(argv[1], "arm")) {
		bool force_arming = false;

		if (argc > 2 && !strcmp(argv[2], "-f")) {
			force_arming = true;
		}

		if (TRANSITION_CHANGED != arm_disarm(true, !force_arming, &mavlink_log_pub, "command line")) {
			PX4_ERR("arming failed");
		}

		return 0;
	}

	if (!strcmp(argv[1], "disarm")) {
		if (TRANSITION_DENIED == arm_disarm(false, true, &mavlink_log_pub, "command line")) {
			PX4_ERR("rejected disarm");
		}

		return 0;
	}

	if (!strcmp(argv[1], "takeoff")) {

		bool ret = false;

		/* see if we got a home position */
		if (status_flags.condition_local_position_valid) {

			if (TRANSITION_DENIED != arm_disarm(true, true, &mavlink_log_pub, "command line")) {
				ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);

			} else {
				PX4_ERR("arming failed");
			}

		} else {
			PX4_ERR("rejecting takeoff, no position lock yet. Please retry..");
		}

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[1], "land")) {
		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[1], "transition")) {

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
						(float)(status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ?
							vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW :
							vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC));

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[1], "mode")) {
		if (argc > 2) {
			uint8_t new_main_state = commander_state_s::MAIN_STATE_MAX;

			if (!strcmp(argv[2], "manual")) {
				new_main_state = commander_state_s::MAIN_STATE_MANUAL;

			} else if (!strcmp(argv[2], "altctl")) {
				new_main_state = commander_state_s::MAIN_STATE_ALTCTL;

			} else if (!strcmp(argv[2], "posctl")) {
				new_main_state = commander_state_s::MAIN_STATE_POSCTL;

			} else if (!strcmp(argv[2], "auto:mission")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;

			} else if (!strcmp(argv[2], "auto:loiter")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_LOITER;

			} else if (!strcmp(argv[2], "auto:rtl")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_RTL;

			} else if (!strcmp(argv[2], "acro")) {
				new_main_state = commander_state_s::MAIN_STATE_ACRO;

			} else if (!strcmp(argv[2], "offboard")) {
				new_main_state = commander_state_s::MAIN_STATE_OFFBOARD;

			} else if (!strcmp(argv[2], "stabilized")) {
				new_main_state = commander_state_s::MAIN_STATE_STAB;

			} else if (!strcmp(argv[2], "rattitude")) {
				new_main_state = commander_state_s::MAIN_STATE_RATTITUDE;

			} else if (!strcmp(argv[2], "auto:takeoff")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_TAKEOFF;

			} else if (!strcmp(argv[2], "auto:land")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_LAND;

			} else if (!strcmp(argv[2], "auto:precland")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_PRECLAND;

			} else {
				PX4_ERR("argument %s unsupported.", argv[2]);
			}

			if (TRANSITION_DENIED == main_state_transition(status, new_main_state, status_flags, &internal_state)) {
				PX4_ERR("mode change failed");
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[1], "lockdown")) {

		if (argc < 3) {
			usage("not enough arguments, missing [on, off]");
			return 1;
		}

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION,
						strcmp(argv[2], "off") ? 2.0f : 0.0f /* lockdown */, 0.0f);

		return (ret ? 0 : 1);
	}

	usage("unrecognized command");
	return 1;
}

void usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The commander module contains the state machine for mode switching and failsafe behavior.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("commander", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Enable HIL mode", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Run sensor calibration");
	PRINT_MODULE_USAGE_ARG("mag|accel|gyro|level|esc|airspeed", "Calibration type", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("check", "Run preflight checks");
	PRINT_MODULE_USAGE_COMMAND("arm");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Force arming (do not run preflight checks)", true);
	PRINT_MODULE_USAGE_COMMAND("disarm");
	PRINT_MODULE_USAGE_COMMAND("takeoff");
	PRINT_MODULE_USAGE_COMMAND("land");
	PRINT_MODULE_USAGE_COMMAND_DESCR("transition", "VTOL transition");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode", "Change flight mode");
	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|rattitude|altctl|posctl|auto:mission|auto:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland",
			"Flight mode", false);
	PRINT_MODULE_USAGE_COMMAND("lockdown");
	PRINT_MODULE_USAGE_ARG("off", "Turn lockdown off", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void print_status()
{
	PX4_INFO("arming: %s", arming_state_names[status.arming_state]);
}

bool shutdown_if_allowed()
{
	return TRANSITION_DENIED != arming_state_transition(&status, safety, vehicle_status_s::ARMING_STATE_SHUTDOWN,
			&armed, false /* fRunPreArmChecks */, &mavlink_log_pub, &status_flags, arm_requirements,
			hrt_elapsed_time(&commander_boot_timestamp));
}

transition_result_t arm_disarm(bool arm, bool run_preflight_checks, orb_advert_t *mavlink_log_pub_local,
			       const char *armedBy)
{
	transition_result_t arming_res = TRANSITION_NOT_CHANGED;

	// Transition the armed state. By passing mavlink_log_pub to arming_state_transition it will
	// output appropriate error messages if the state cannot transition.
	arming_res = arming_state_transition(&status,
					     safety,
					     arm ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_STANDBY,
					     &armed,
					     run_preflight_checks,
					     mavlink_log_pub_local,
					     &status_flags,
					     arm_requirements,
					     hrt_elapsed_time(&commander_boot_timestamp));

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(mavlink_log_pub_local, "%s by %s", arm ? "ARMED" : "DISARMED", armedBy);

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

Commander::Commander() :
	ModuleParams(nullptr),
	_failure_detector(this)
{
	_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);
	_auto_disarm_killed.set_hysteresis_time_from(false, 5_s);

	// We want to accept RC inputs as default
	status.rc_input_mode = vehicle_status_s::RC_IN_MODE_DEFAULT;
	internal_state.main_state = commander_state_s::MAIN_STATE_MANUAL;
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	status.arming_state = vehicle_status_s::ARMING_STATE_INIT;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = true;
	status_flags.offboard_control_signal_lost = true;
	status.data_link_lost = true;

	status_flags.condition_power_input_valid = true;
	status_flags.rc_calibration_valid = true;

	status_flags.avoidance_system_valid = false;
}

Commander::~Commander()
{
	delete[] _airspeed_fault_type;
}

bool
Commander::handle_command(vehicle_status_s *status_local, const vehicle_command_s &cmd, actuator_armed_s *armed_local,
			  uORB::PublicationQueued<vehicle_command_ack_s> &command_ack_pub, bool *changed)
{
	/* only handle commands that are meant to be handled by this system and component */
	if (cmd.target_system != status_local->system_id || ((cmd.target_component != status_local->component_id)
			&& (cmd.target_component != 0))) { // component_id 0: valid for all components
		return false;
	}

	/* result of the command */
	unsigned cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd.command) {
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION: {

			// Just switch the flight mode here, the navigator takes care of
			// doing something sensible with the coordinates. Its designed
			// to not require navigator and command to receive / process
			// the data at the exact same time.

			// Check if a mode switch had been requested
			if ((((uint32_t)cmd.param2) & 1) > 0) {
				transition_result_t main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LOITER,
							       status_flags, &internal_state);

				if ((main_ret != TRANSITION_DENIED)) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					mavlink_log_critical(&mavlink_log_pub, "Reposition command rejected");
				}

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t)cmd.param1;
			uint8_t custom_main_mode = (uint8_t)cmd.param2;
			uint8_t custom_sub_mode = (uint8_t)cmd.param3;

			transition_result_t arming_ret = TRANSITION_NOT_CHANGED;

			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			// We ignore base_mode & VEHICLE_MODE_FLAG_SAFETY_ARMED because
			// the command VEHICLE_CMD_COMPONENT_ARM_DISARM should be used
			// instead according to the latest mavlink spec.

			if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					/* MANUAL */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					/* ALTCTL */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_ALTCTL, status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					/* POSCTL */
					reset_posvel_validity(changed);
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					/* AUTO */
					if (custom_sub_mode > 0) {
						reset_posvel_validity(changed);

						switch (custom_sub_mode) {
						case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags,
											 &internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
							if (status_flags.condition_auto_mission_available) {
								main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
												 &internal_state);

							} else {
								main_ret = TRANSITION_DENIED;
							}

							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags, &internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, status_flags,
											 &internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags, &internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET, status_flags,
											 &internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_PRECLAND, status_flags,
											 &internal_state);
							break;

						default:
							main_ret = TRANSITION_DENIED;
							mavlink_log_critical(&mavlink_log_pub, "Unsupported auto mode");
							break;
						}

					} else {
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
										 &internal_state);
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					/* ACRO */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_ACRO, status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_RATTITUDE) {
					/* RATTITUDE */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_RATTITUDE, status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
					/* STABILIZED */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					reset_posvel_validity(changed);
					/* OFFBOARD */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_OFFBOARD, status_flags, &internal_state);
				}

			} else {
				/* use base mode */
				if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
					/* AUTO */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
									 &internal_state);

				} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
						/* POSCTL */
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &internal_state);

					} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
						/* STABILIZED */
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &internal_state);

					} else {
						/* MANUAL */
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);
					}
				}
			}

			if ((arming_ret != TRANSITION_DENIED) && (main_ret != TRANSITION_DENIED)) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				if (arming_ret == TRANSITION_DENIED) {
					mavlink_log_critical(&mavlink_log_pub, "Arming command rejected");
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {

			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters
			if (static_cast<int>(cmd.param1 + 0.5f) != 0 && static_cast<int>(cmd.param1 + 0.5f) != 1) {
				mavlink_log_critical(&mavlink_log_pub, "Unsupported ARM_DISARM param: %.3f", (double)cmd.param1);

			} else {

				bool cmd_arms = (static_cast<int>(cmd.param1 + 0.5f) == 1);

				// Arm/disarm is enforced only when param2 is set to a magic number.
				const bool enforce_in_air = (static_cast<int>(std::round(cmd.param2)) == 21196);

				if (!enforce_in_air && !land_detector.landed && !is_ground_rover(&status)) {
					if (cmd_arms) {
						mavlink_log_critical(&mavlink_log_pub, "Arming denied! Not landed");

					} else {
						mavlink_log_critical(&mavlink_log_pub, "Disarming denied! Not landed");
					}

					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
					break;
				}

				// Flick to inair restore first if this comes from an onboard system
				if (cmd.source_system == status_local->system_id && cmd.source_component == status_local->component_id) {
					status.arming_state = vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE;

				} else {
					// Refuse to arm if preflight checks have failed
					if ((!status_local->hil_state) != vehicle_status_s::HIL_STATE_ON
					    && !status_flags.condition_system_sensors_initialized) {
						mavlink_log_critical(&mavlink_log_pub, "Arming denied! Preflight checks have failed");
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
						break;
					}

					// Refuse to arm if in manual with non-zero throttle
					if (cmd_arms
					    && (status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL
						|| status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO
						|| status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_STAB
						|| status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_RATTITUDE)
					    && (sp_man.z > 0.1f)) {

						mavlink_log_critical(&mavlink_log_pub, "Arming denied! Manual throttle not zero");
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
						break;
					}
				}

				transition_result_t arming_res = arm_disarm(cmd_arms, true, &mavlink_log_pub, "Arm/Disarm component command");

				if (arming_res == TRANSITION_DENIED) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
					if (cmd_arms && (arming_res == TRANSITION_CHANGED) &&
					    (hrt_absolute_time() > (commander_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL)) && !_home_pub.get().manual_home) {

						set_home_position();
					}
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION: {
			if (cmd.param1 > 1.5f) {
				armed_local->lockdown = true;
				warnx("forcing lockdown (motors off)");

			} else if (cmd.param1 > 0.5f) {
				//XXX update state machine?
				armed_local->force_failsafe = true;
				warnx("forcing failsafe (termination)");

				if ((int)cmd.param2 <= 0) {
					/* reset all commanded failure modes */
					warnx("reset all non-flighttermination failsafe commands");
				}

			} else {
				armed_local->force_failsafe = false;
				armed_local->lockdown = false;
				warnx("disabling failsafe and lockdown");
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME: {
			bool use_current = cmd.param1 > 0.5f;

			if (use_current) {
				/* use current position */
				if (set_home_position()) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

			} else {
				const double lat = cmd.param5;
				const double lon = cmd.param6;
				const float alt = cmd.param7;

				if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt)) {
					const vehicle_local_position_s &local_pos = _local_position_sub.get();

					if (local_pos.xy_global && local_pos.z_global) {
						/* use specified position */
						home_position_s home{};
						home.timestamp = hrt_absolute_time();

						home.lat = lat;
						home.lon = lon;
						home.alt = alt;

						home.manual_home = true;
						home.valid_alt = true;
						home.valid_hpos = true;

						// update local projection reference including altitude
						struct map_projection_reference_s ref_pos;
						map_projection_init(&ref_pos, local_pos.ref_lat, local_pos.ref_lon);
						map_projection_project(&ref_pos, lat, lon, &home.x, &home.y);
						home.z = -(alt - local_pos.ref_alt);

						/* mark home position as set */
						status_flags.condition_home_position_valid = _home_pub.update(home);

						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					}

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE: {
			transition_result_t res = TRANSITION_DENIED;
			static main_state_t main_state_pre_offboard = commander_state_s::MAIN_STATE_MANUAL;

			if (internal_state.main_state != commander_state_s::MAIN_STATE_OFFBOARD) {
				main_state_pre_offboard = internal_state.main_state;
			}

			if (cmd.param1 > 0.5f) {
				res = main_state_transition(*status_local, commander_state_s::MAIN_STATE_OFFBOARD, status_flags, &internal_state);

				if (res == TRANSITION_DENIED) {
					print_reject_mode("OFFBOARD");
					status_flags.offboard_control_set_by_command = false;

				} else {
					/* Set flag that offboard was set via command, main state is not overridden by rc */
					status_flags.offboard_control_set_by_command = true;
				}

			} else {
				/* If the mavlink command is used to enable or disable offboard control:
				 * switch back to previous mode when disabling */
				res = main_state_transition(*status_local, main_state_pre_offboard, status_flags, &internal_state);
				status_flags.offboard_control_set_by_command = false;
			}

			if (res == TRANSITION_DENIED) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: {
			/* switch to RTL which ends the mission */
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags,
					&internal_state)) {
				mavlink_and_console_log_info(&mavlink_log_pub, "Returning to launch");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Return to launch denied");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
			/* ok, home set, use it to take off */
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, status_flags,
					&internal_state)) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else if (internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Takeoff denied! Please disarm and retry");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND: {
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags,
					&internal_state)) {
				mavlink_and_console_log_info(&mavlink_log_pub, "Landing at current position");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Landing denied! Please land manually");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND: {
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_PRECLAND,
					status_flags, &internal_state)) {
				mavlink_and_console_log_info(&mavlink_log_pub, "Precision landing");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Precision landing denied! Please land manually");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;

			// check if current mission and first item are valid
			if (status_flags.condition_auto_mission_available) {

				// requested first mission item valid
				if (PX4_ISFINITE(cmd.param1) && (cmd.param1 >= -1) && (cmd.param1 < _mission_result_sub.get().seq_total)) {

					// switch to AUTO_MISSION and ARM
					if ((TRANSITION_DENIED != main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
							&internal_state))
					    && (TRANSITION_DENIED != arm_disarm(true, true, &mavlink_log_pub, "Mission start command"))) {

						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
						mavlink_log_critical(&mavlink_log_pub, "Mission start denied");
					}
				}

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Mission start denied! No valid mission");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY: {
			// if no high latency telemetry exists send a failed acknowledge
			if (_high_latency_datalink_heartbeat > commander_boot_timestamp) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_FAILED;
				mavlink_log_critical(&mavlink_log_pub, "Control high latency failed! Telemetry unavailable");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		// Switch to orbit state and let the orbit task handle the command further
		main_state_transition(*status_local, commander_state_s::MAIN_STATE_ORBIT, status_flags, &internal_state);
		break;

	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_2:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_UAVCAN:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED:
	case vehicle_command_s::VEHICLE_CMD_DO_LAND_START:
	case vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND:
	case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_START:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_STOP:
	case vehicle_command_s::VEHICLE_CMD_NAV_DELAY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
	case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
		/* ignore commands that are handled by other parts of the system */
		break;

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED, command_ack_pub);
		break;
	}

	if (cmd_result != vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(cmd, cmd_result, command_ack_pub);
	}

	return true;
}

/**
* @brief This function initializes the home position an altitude of the vehicle. This happens first time we get a good GPS fix and each
*		 time the vehicle is armed with a good GPS fix.
**/
bool
Commander::set_home_position()
{
	// Need global and local position fix to be able to set home
	if (status_flags.condition_global_position_valid && status_flags.condition_local_position_valid) {

		const vehicle_global_position_s &gpos = _global_position_sub.get();

		// Ensure that the GPS accuracy is good enough for intializing home
		if ((gpos.eph <= _param_com_home_h_t.get()) && (gpos.epv <= _param_com_home_v_t.get())) {

			const vehicle_local_position_s &lpos = _local_position_sub.get();

			// Set home position
			home_position_s home{};

			home.timestamp = hrt_absolute_time();

			home.lat = gpos.lat;
			home.lon = gpos.lon;
			home.valid_hpos = true;

			home.alt = gpos.alt;
			home.valid_alt = true;

			home.x = lpos.x;
			home.y = lpos.y;
			home.z = lpos.z;

			home.yaw = lpos.yaw;

			home.manual_home = false;

			// play tune first time we initialize HOME
			if (!status_flags.condition_home_position_valid) {
				tune_home_set(true);
			}

			// mark home position as set
			status_flags.condition_home_position_valid = _home_pub.update(home);

			return status_flags.condition_home_position_valid;
		}
	}

	return false;
}

bool
Commander::set_home_position_alt_only()
{
	const vehicle_local_position_s &lpos = _local_position_sub.get();

	if (!_home_pub.get().valid_alt && lpos.z_global) {
		// handle special case where we are setting only altitude using local position reference
		home_position_s home{};
		home.alt = lpos.ref_alt;
		home.valid_alt = true;

		home.timestamp = hrt_absolute_time();

		return _home_pub.update(home);
	}

	return false;
}

void
Commander::run()
{
	bool sensor_fail_tune_played = false;
	bool arm_tune_played = false;
	bool was_landed = true;
	bool was_falling = false;
	bool was_armed = false;

	// XXX for now just set sensors as initialized
	status_flags.condition_system_sensors_initialized = true;

	/* set parameters */
	param_t _param_sys_type = param_find("MAV_TYPE");
	param_t _param_system_id = param_find("MAV_SYS_ID");
	param_t _param_component_id = param_find("MAV_COMP_ID");
	param_t _param_ef_throttle_thres = param_find("COM_EF_THROT");
	param_t _param_ef_current2throttle_thres = param_find("COM_EF_C2T");
	param_t _param_ef_time_thres = param_find("COM_EF_TIME");
	param_t _param_rc_in_off = param_find("COM_RC_IN_MODE");
	param_t _param_rc_arm_hyst = param_find("COM_RC_ARM_HYST");
	param_t _param_min_stick_change = param_find("COM_RC_STICK_OV");
	param_t _param_geofence_action = param_find("GF_ACTION");
	param_t _param_arm_without_gps = param_find("COM_ARM_WO_GPS");
	param_t _param_arm_switch_is_button = param_find("COM_ARM_SWISBTN");
	param_t _param_rc_override = param_find("COM_RC_OVERRIDE");
	param_t _param_arm_mission_required = param_find("COM_ARM_MIS_REQ");
	param_t _param_escs_checks_required = param_find("COM_ARM_CHK_ESCS");
	param_t _param_flight_uuid = param_find("COM_FLIGHT_UUID");
	param_t _param_takeoff_finished_action = param_find("COM_TAKEOFF_ACT");

	param_t _param_fmode_1 = param_find("COM_FLTMODE1");
	param_t _param_fmode_2 = param_find("COM_FLTMODE2");
	param_t _param_fmode_3 = param_find("COM_FLTMODE3");
	param_t _param_fmode_4 = param_find("COM_FLTMODE4");
	param_t _param_fmode_5 = param_find("COM_FLTMODE5");
	param_t _param_fmode_6 = param_find("COM_FLTMODE6");

	param_t _param_airmode = param_find("MC_AIRMODE");
	param_t _param_rc_map_arm_switch = param_find("RC_MAP_ARM_SW");

	/* failsafe response to loss of navigation accuracy */
	param_t _param_posctl_nav_loss_act = param_find("COM_POSCTL_NAVL");

	status_flags.avoidance_system_required = _param_com_obs_avoid.get();

	/* pthread for slow low prio thread */
	pthread_t commander_low_prio_thread;

	/* initialize */
	if (led_init() != OK) {
		PX4_WARN("LED init failed");
	}

	if (buzzer_init() != OK) {
		PX4_WARN("Buzzer init failed");
	}

	uORB::Subscription power_button_state_sub{ORB_ID(power_button_state)};
	{
		// we need to do an initial publication to make sure uORB allocates the buffer, which cannot happen
		// in IRQ context.
		power_button_state_s button_state{};
		button_state.event = 0xff;
		power_button_state_pub = orb_advertise(ORB_ID(power_button_state), &button_state);

		power_button_state_sub.copy(&button_state);
	}

	if (board_register_power_state_notification_cb(power_button_state_notification_cb) != 0) {
		PX4_ERR("Failed to register power notification callback");
	}


	get_circuit_breaker_params();

	/* armed topic */
	hrt_abstime last_disarmed_timestamp = 0;

	/* command ack */
	uORB::PublicationQueued<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};

	/* init mission state, do it here to allow navigator to use stored mission even if mavlink failed to start */
	mission_init();

	/* Start monitoring loop */
	unsigned counter = 0;
	int stick_off_counter = 0;
	int stick_on_counter = 0;

	bool status_changed = true;
	bool param_init_forced = true;

	bool updated = false;

	uORB::Subscription actuator_controls_sub{ORB_ID_VEHICLE_ATTITUDE_CONTROLS};
	uORB::Subscription cmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription geofence_result_sub{ORB_ID(geofence_result)};
	uORB::Subscription land_detector_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription param_changed_sub{ORB_ID(parameter_update)};
	uORB::Subscription safety_sub{ORB_ID(safety)};
	uORB::Subscription sp_man_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription subsys_sub{ORB_ID(subsystem_info)};
	uORB::Subscription system_power_sub{ORB_ID(system_power)};
	uORB::Subscription vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	uORB::Subscription esc_status_sub{ORB_ID(esc_status)};

	geofence_result_s geofence_result {};

	land_detector.landed = true;

	vtol_status.vtol_in_rw_mode = true;		// default for vtol is rotary wing

	control_status_leds(&status, &armed, true, _battery_warning, &cpuload);

	thread_running = true;

	/* update vehicle status to find out vehicle type (required for preflight checks) */
	int32_t system_type;
	param_get(_param_sys_type, &system_type); // get system type
	status.system_type = (uint8_t)system_type;

	if (is_rotary_wing(&status) || is_vtol(&status)) {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	} else if (is_fixed_wing(&status)) {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	} else if (is_ground_rover(&status)) {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;

	} else {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;
	}

	status.is_vtol = is_vtol(&status);

	commander_boot_timestamp = hrt_absolute_time();

	// initially set to failed
	_last_lpos_fail_time_us = commander_boot_timestamp;
	_last_gpos_fail_time_us = commander_boot_timestamp;
	_last_lvel_fail_time_us = commander_boot_timestamp;

	// Run preflight check
	int32_t rc_in_off = 0;

	param_get(_param_rc_in_off, &rc_in_off);

	int32_t arm_switch_is_button = 0;
	param_get(_param_arm_switch_is_button, &arm_switch_is_button);

	int32_t arm_without_gps_param = 0;
	param_get(_param_arm_without_gps, &arm_without_gps_param);
	arm_requirements = (arm_without_gps_param == 1) ? ARM_REQ_NONE : ARM_REQ_GPS_BIT;

	int32_t arm_mission_required_param = 0;
	param_get(_param_arm_mission_required, &arm_mission_required_param);
	arm_requirements |= (arm_mission_required_param & (ARM_REQ_MISSION_BIT | ARM_REQ_ARM_AUTH_BIT));

	int32_t arm_escs_checks_required_param = 0;
	param_get(_param_escs_checks_required, &arm_escs_checks_required_param);
	arm_requirements |= (arm_escs_checks_required_param == 0) ? ARM_REQ_NONE : ARM_REQ_ESCS_CHECK_BIT;

	status.rc_input_mode = rc_in_off;

	// user adjustable duration required to assert arm/disarm via throttle/rudder stick
	int32_t rc_arm_hyst = 100;
	param_get(_param_rc_arm_hyst, &rc_arm_hyst);
	rc_arm_hyst *= COMMANDER_MONITORING_LOOPSPERMSEC;

	int32_t posctl_nav_loss_act = 0;
	int32_t geofence_action = 0;
	int32_t flight_uuid = 0;
	int32_t airmode = 0;
	int32_t rc_map_arm_switch = 0;

	/* RC override auto modes */
	int32_t rc_override = 0;

	int32_t takeoff_complete_act = 0;

	/* Thresholds for engine failure detection */
	float ef_throttle_thres = 1.0f;
	float ef_current2throttle_thres = 0.0f;
	float ef_time_thres = 1000.0f;
	uint64_t timestamp_engine_healthy = 0; /**< absolute time when engine was healty */

	/* check which state machines for changes, clear "changed" flag */
	bool failsafe_old = false;

	bool have_taken_off_since_arming = false;

	/* initialize low priority thread */
	pthread_attr_t commander_low_prio_attr;
	pthread_attr_init(&commander_low_prio_attr);
	pthread_attr_setstacksize(&commander_low_prio_attr, PX4_STACK_ADJUSTED(3000));

#ifndef __PX4_QURT
	// This is not supported by QURT (yet).
	struct sched_param param;
	(void)pthread_attr_getschedparam(&commander_low_prio_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
	(void)pthread_attr_setschedparam(&commander_low_prio_attr, &param);
#endif

	pthread_create(&commander_low_prio_thread, &commander_low_prio_attr, commander_low_prio_loop, nullptr);
	pthread_attr_destroy(&commander_low_prio_attr);

	arm_auth_init(&mavlink_log_pub, &status.system_id);

	// run preflight immediately to find all relevant parameters, but don't report
	preflight_check(false);

	while (!should_exit()) {

		transition_result_t arming_ret = TRANSITION_NOT_CHANGED;

		/* update parameters */
		bool params_updated = param_changed_sub.updated();

		if (params_updated || param_init_forced) {

			/* parameters changed */
			parameter_update_s param_changed;
			param_changed_sub.copy(&param_changed);

			updateParams();

			/* update parameters */
			if (!armed.armed) {
				if (param_get(_param_sys_type, &system_type) != OK) {
					PX4_ERR("failed getting new system type");

				} else {
					status.system_type = (uint8_t)system_type;
				}

				const bool is_rotary = is_rotary_wing(&status) || (is_vtol(&status) && vtol_status.vtol_in_rw_mode);
				const bool is_fixed = is_fixed_wing(&status) || (is_vtol(&status) && !vtol_status.vtol_in_rw_mode);
				const bool is_ground = is_ground_rover(&status);

				/* disable manual override for all systems that rely on electronic stabilization */
				if (is_rotary) {
					status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

				} else if (is_fixed) {
					status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

				} else if (is_ground) {
					status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
				}

				/* set vehicle_status.is_vtol flag */
				status.is_vtol = is_vtol(&status);

				/* check and update system / component ID */
				int32_t sys_id = 0;
				param_get(_param_system_id, &sys_id);
				status.system_id = sys_id;

				int32_t comp_id = 0;
				param_get(_param_component_id, &comp_id);
				status.component_id = comp_id;

				get_circuit_breaker_params();

				status_changed = true;
			}

			/* Safety parameters */
			param_get(_param_rc_in_off, &rc_in_off);
			status.rc_input_mode = rc_in_off;
			param_get(_param_rc_arm_hyst, &rc_arm_hyst);
			param_get(_param_min_stick_change, &min_stick_change);
			param_get(_param_rc_override, &rc_override);
			// percentage (* 0.01) needs to be doubled because RC total interval is 2, not 1
			min_stick_change *= 0.02f;
			rc_arm_hyst *= COMMANDER_MONITORING_LOOPSPERMSEC;
			param_get(_param_ef_throttle_thres, &ef_throttle_thres);
			param_get(_param_ef_current2throttle_thres, &ef_current2throttle_thres);
			param_get(_param_ef_time_thres, &ef_time_thres);
			param_get(_param_geofence_action, &geofence_action);
			param_get(_param_flight_uuid, &flight_uuid);

			param_get(_param_arm_switch_is_button, &arm_switch_is_button);

			param_get(_param_arm_without_gps, &arm_without_gps_param);
			arm_requirements = (arm_without_gps_param == 1) ? ARM_REQ_NONE : ARM_REQ_GPS_BIT;
			param_get(_param_arm_mission_required, &arm_mission_required_param);
			arm_requirements |= (arm_mission_required_param & (ARM_REQ_MISSION_BIT | ARM_REQ_ARM_AUTH_BIT));
			param_get(_param_escs_checks_required, &arm_escs_checks_required_param);
			arm_requirements |= (arm_escs_checks_required_param == 0) ? ARM_REQ_NONE : ARM_REQ_ESCS_CHECK_BIT;


			/* flight mode slots */
			param_get(_param_fmode_1, &_flight_mode_slots[0]);
			param_get(_param_fmode_2, &_flight_mode_slots[1]);
			param_get(_param_fmode_3, &_flight_mode_slots[2]);
			param_get(_param_fmode_4, &_flight_mode_slots[3]);
			param_get(_param_fmode_5, &_flight_mode_slots[4]);
			param_get(_param_fmode_6, &_flight_mode_slots[5]);

			/* failsafe response to loss of navigation accuracy */
			param_get(_param_posctl_nav_loss_act, &posctl_nav_loss_act);

			param_get(_param_takeoff_finished_action, &takeoff_complete_act);

			/* check for unsafe Airmode settings: yaw airmode requires the use of an arming switch */
			if (_param_airmode != PARAM_INVALID && _param_rc_map_arm_switch != PARAM_INVALID) {
				param_get(_param_airmode, &airmode);
				param_get(_param_rc_map_arm_switch, &rc_map_arm_switch);

				if (airmode == 2 && rc_map_arm_switch == 0) {
					airmode = 1; // change to roll/pitch airmode
					param_set(_param_airmode, &airmode);
					mavlink_log_critical(&mavlink_log_pub, "Yaw Airmode requires the use of an Arm Switch")
				}
			}

			param_init_forced = false;
		}

		/* Update OA parameter */
		status_flags.avoidance_system_required = _param_com_obs_avoid.get();

		/* handle power button state */
		if (power_button_state_sub.updated()) {
			power_button_state_s button_state;
			power_button_state_sub.copy(&button_state);

			if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
				px4_shutdown_request(false, false);
			}
		}

		sp_man_sub.update(&sp_man);

		offboard_control_update(status_changed);

		if (system_power_sub.updated()) {
			system_power_s system_power = {};
			system_power_sub.copy(&system_power);

			if (hrt_elapsed_time(&system_power.timestamp) < 200_ms) {
				if (system_power.servo_valid &&
				    !system_power.brick_valid &&
				    !system_power.usb_connected) {
					/* flying only on servo rail, this is unsafe */
					status_flags.condition_power_input_valid = false;

				} else {
					status_flags.condition_power_input_valid = true;
				}

				/* if the USB hardware connection went away, reboot */
				if (status_flags.usb_connected && !system_power.usb_connected && shutdown_if_allowed()) {
					/*
					 * apparently the USB cable went away but we are still powered,
					 * so lets reset to a classic non-usb state.
					 */
					mavlink_log_critical(&mavlink_log_pub, "USB disconnected, rebooting.")
					px4_usleep(400000);
					px4_shutdown_request(true, false);
				}
			}
		}

		/* update safety topic */
		if (safety_sub.updated()) {
			bool previous_safety_off = safety.safety_off;

			if (safety_sub.copy(&safety)) {

				/* disarm if safety is now on and still armed */
				if (armed.armed && (status.hil_state == vehicle_status_s::HIL_STATE_OFF)
				    && safety.safety_switch_available && !safety.safety_off) {

					if (TRANSITION_CHANGED == arming_state_transition(&status, safety, vehicle_status_s::ARMING_STATE_STANDBY,
							&armed, true /* fRunPreArmChecks */, &mavlink_log_pub,
							&status_flags, arm_requirements, hrt_elapsed_time(&commander_boot_timestamp))
					   ) {
						status_changed = true;
					}
				}

				// Notify the user if the status of the safety switch changes
				if (safety.safety_switch_available && previous_safety_off != safety.safety_off) {

					if (safety.safety_off) {
						set_tune(TONE_NOTIFY_POSITIVE_TUNE);

					} else {
						tune_neutral(true);
					}

					status_changed = true;
				}
			}
		}

		/* update vtol vehicle status*/
		if (vtol_vehicle_status_sub.updated()) {
			/* vtol status changed */
			vtol_vehicle_status_sub.copy(&vtol_status);
			status.vtol_fw_permanent_stab = vtol_status.fw_permanent_stab;

			/* Make sure that this is only adjusted if vehicle really is of type vtol */
			if (is_vtol(&status)) {

				// Check if there has been any change while updating the flags
				const auto new_vehicle_type = vtol_status.vtol_in_rw_mode ?
							      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
							      vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

				if (new_vehicle_type != status.vehicle_type) {
					status.vehicle_type = vtol_status.vtol_in_rw_mode ?
							      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
							      vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
					status_changed = true;
				}

				if (status.in_transition_mode != vtol_status.vtol_in_trans_mode) {
					status.in_transition_mode = vtol_status.vtol_in_trans_mode;
					status_changed = true;
				}

				if (status.in_transition_to_fw != vtol_status.in_transition_to_fw) {
					status.in_transition_to_fw = vtol_status.in_transition_to_fw;
					status_changed = true;
				}

				if (status_flags.vtol_transition_failure != vtol_status.vtol_transition_failsafe) {
					status_flags.vtol_transition_failure = vtol_status.vtol_transition_failsafe;
					status_changed = true;
				}

				const bool should_soft_stop = (status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

				if (armed.soft_stop != should_soft_stop) {
					armed.soft_stop = should_soft_stop;
					status_changed = true;
				}
			}
		}

		if (esc_status_sub.updated()) {
			/* ESCs status changed */
			esc_status_s esc_status = {};

			esc_status_sub.copy(&esc_status);
			esc_status_check(esc_status);
		}

		estimator_check(&status_changed);
		airspeed_use_check();

		/* Update land detector */
		if (land_detector_sub.updated()) {
			land_detector_sub.copy(&land_detector);

			// Only take actions if armed
			if (armed.armed) {
				if (was_landed != land_detector.landed) {
					if (land_detector.landed) {
						mavlink_and_console_log_info(&mavlink_log_pub, "Landing detected");

					} else {
						mavlink_and_console_log_info(&mavlink_log_pub, "Takeoff detected");
						have_taken_off_since_arming = true;

						// Set all position and velocity test probation durations to takeoff value
						// This is a larger value to give the vehicle time to complete a failsafe landing
						// if faulty sensors cause loss of navigation shortly after takeoff.
						_gpos_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
						_lpos_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
						_lvel_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
					}
				}

				if (was_falling != land_detector.freefall) {
					if (land_detector.freefall) {
						mavlink_and_console_log_info(&mavlink_log_pub, "Freefall detected");
					}
				}
			}

			was_landed = land_detector.landed;
			was_falling = land_detector.freefall;
		}


		// Auto disarm when landed or kill switch engaged
		if (armed.armed) {

			// Check for auto-disarm on landing or pre-flight
			if (_param_com_disarm_land.get() > 0 || _param_com_disarm_preflight.get() > 0) {

				if (_param_com_disarm_land.get() > 0 && have_taken_off_since_arming) {
					_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_land.get() * 1_s);
					_auto_disarm_landed.set_state_and_update(land_detector.landed, hrt_absolute_time());

				} else if (_param_com_disarm_preflight.get() > 0 && !have_taken_off_since_arming) {
					_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);
					_auto_disarm_landed.set_state_and_update(true, hrt_absolute_time());
				}

				if (_auto_disarm_landed.get_state()) {
					arm_disarm(false, true, &mavlink_log_pub, "Auto disarm initiated");
				}
			}

			// Auto disarm after 5 seconds if kill switch is engaged
			_auto_disarm_killed.set_state_and_update(armed.manual_lockdown, hrt_absolute_time());

			if (_auto_disarm_killed.get_state()) {
				arm_disarm(false, true, &mavlink_log_pub, "Kill-switch still engaged, disarming");
			}

		} else {
			_auto_disarm_landed.set_state_and_update(false, hrt_absolute_time());
			_auto_disarm_killed.set_state_and_update(false, hrt_absolute_time());
		}


		if (!_geofence_warning_action_on) {
			// store the last good main_state when not in an navigation
			// hold state
			main_state_before_rtl = internal_state.main_state;

		} else if (internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
			   && internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LOITER
			   && internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {
			// reset flag again when we switched out of it
			_geofence_warning_action_on = false;
		}

		cpuload_sub.update(&cpuload);

		battery_status_check();

		/* update subsystem info which arrives from outside of commander*/
		do {
			if (subsys_sub.updated()) {
				subsystem_info_s info{};
				subsys_sub.copy(&info);
				set_health_flags(info.subsystem_type, info.present, info.enabled, info.ok, status);
				status_changed = true;
			}
		} while (updated);

		/* If in INIT state, try to proceed to STANDBY state */
		if (!status_flags.condition_calibration_enabled && status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {

			arming_ret = arming_state_transition(&status, safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
							     true /* fRunPreArmChecks */, &mavlink_log_pub, &status_flags,
							     arm_requirements, hrt_elapsed_time(&commander_boot_timestamp));

			if (arming_ret == TRANSITION_DENIED) {
				/* do not complain if not allowed into standby */
				arming_ret = TRANSITION_NOT_CHANGED;
			}
		}

		/* start mission result check */
		const auto prev_mission_instance_count = _mission_result_sub.get().instance_count;

		if (_mission_result_sub.update()) {
			const mission_result_s &mission_result = _mission_result_sub.get();

			// if mission_result is valid for the current mission
			const bool mission_result_ok = (mission_result.timestamp > commander_boot_timestamp)
						       && (mission_result.instance_count > 0);

			status_flags.condition_auto_mission_available = mission_result_ok && mission_result.valid;

			if (mission_result_ok) {

				if (status.mission_failure != mission_result.failure) {
					status.mission_failure = mission_result.failure;
					status_changed = true;

					if (status.mission_failure) {
						mavlink_log_critical(&mavlink_log_pub, "Mission cannot be completed");
					}
				}

				/* Only evaluate mission state if home is set */
				if (status_flags.condition_home_position_valid &&
				    (prev_mission_instance_count != mission_result.instance_count)) {

					if (!status_flags.condition_auto_mission_available) {
						/* the mission is invalid */
						tune_mission_fail(true);

					} else if (mission_result.warning) {
						/* the mission has a warning */
						tune_mission_fail(true);

					} else {
						/* the mission is valid */
						tune_mission_ok(true);
					}
				}
			}
		}

		/* start geofence result check */
		geofence_result_sub.update(&geofence_result);

		// Geofence actions
		const bool geofence_action_enabled = geofence_result.geofence_action != geofence_result_s::GF_ACTION_NONE;
		const bool not_in_battery_failsafe = _battery_warning < battery_status_s::BATTERY_WARNING_CRITICAL;

		if (armed.armed
		    && geofence_action_enabled
		    && not_in_battery_failsafe) {

			// check for geofence violation transition
			if (geofence_result.geofence_violated && !_geofence_violated_prev) {

				switch (geofence_result.geofence_action) {
				case (geofence_result_s::GF_ACTION_NONE) : {
						// do nothing
						break;
					}

				case (geofence_result_s::GF_ACTION_WARN) : {
						// do nothing, mavlink critical messages are sent by navigator
						break;
					}

				case (geofence_result_s::GF_ACTION_LOITER) : {
						if (TRANSITION_CHANGED == main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags,
								&internal_state)) {
							_geofence_loiter_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_RTL) : {
						if (TRANSITION_CHANGED == main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags,
								&internal_state)) {
							_geofence_rtl_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_TERMINATE) : {
						warnx("Flight termination because of geofence");
						mavlink_log_critical(&mavlink_log_pub, "Geofence violation! Flight terminated");
						armed.force_failsafe = true;
						status_changed = true;
						break;
					}
				}
			}

			_geofence_violated_prev = geofence_result.geofence_violated;

			// reset if no longer in LOITER or if manually switched to LOITER
			const bool in_loiter_mode = internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER;
			const bool manual_loiter_switch_on = sp_man.loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON;

			if (!in_loiter_mode || manual_loiter_switch_on) {
				_geofence_loiter_on = false;
			}


			// reset if no longer in RTL or if manually switched to RTL
			const bool in_rtl_mode = internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL;
			const bool manual_return_switch_on = sp_man.return_switch == manual_control_setpoint_s::SWITCH_POS_ON;

			if (!in_rtl_mode || manual_return_switch_on) {
				_geofence_rtl_on = false;
			}

			_geofence_warning_action_on = _geofence_warning_action_on || (_geofence_loiter_on || _geofence_rtl_on);

		} else {
			// No geofence checks, reset flags
			_geofence_loiter_on = false;
			_geofence_rtl_on = false;
			_geofence_warning_action_on = false;
			_geofence_violated_prev = false;
		}

		// abort auto mode or geofence reaction if sticks are moved significantly
		// but only if not in a low battery handling action
		const bool not_in_low_battery_reaction = _battery_warning < battery_status_s::BATTERY_WARNING_CRITICAL;
		const bool is_rotary_wing = status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		const bool manual_mode_before_geofence =
			main_state_before_rtl == commander_state_s::MAIN_STATE_MANUAL ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_ALTCTL ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_POSCTL ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_ACRO ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_RATTITUDE ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_STAB;
		const bool in_auto_mode =
			internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
			internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_MISSION ||
			internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER;

		if (rc_override != 0 && is_rotary_wing && not_in_low_battery_reaction
		    && (in_auto_mode || (_geofence_warning_action_on && manual_mode_before_geofence))) {
			// transition to previous state if sticks are touched
			if ((_last_sp_man.timestamp != sp_man.timestamp) &&
			    ((fabsf(sp_man.x - _last_sp_man.x) > min_stick_change) ||
			     (fabsf(sp_man.y - _last_sp_man.y) > min_stick_change) ||
			     (fabsf(sp_man.z - _last_sp_man.z) > min_stick_change) ||
			     (fabsf(sp_man.r - _last_sp_man.r) > min_stick_change))) {

				// revert to position control in any case
				main_state_transition(status, commander_state_s::MAIN_STATE_POSCTL, status_flags, &internal_state);
				mavlink_log_critical(&mavlink_log_pub, "Autopilot off! Returning control to pilot");
			}
		}

		/* Check for mission flight termination */
		if (armed.armed && _mission_result_sub.get().flight_termination &&
		    !status_flags.circuit_breaker_flight_termination_disabled) {

			armed.force_failsafe = true;
			status_changed = true;
			static bool flight_termination_printed = false;

			if (!flight_termination_printed) {
				mavlink_log_critical(&mavlink_log_pub, "Geofence violation! Flight terminated");
				flight_termination_printed = true;
			}

			if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
				mavlink_log_critical(&mavlink_log_pub, "Flight termination active");
			}
		}

		/* RC input check */
		if (!status_flags.rc_input_blocked && sp_man.timestamp != 0 &&
		    (hrt_elapsed_time(&sp_man.timestamp) < (_param_com_rc_loss_t.get() * 1_s))) {

			/* handle the case where RC signal was regained */
			if (!status_flags.rc_signal_found_once) {
				status_flags.rc_signal_found_once = true;
				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, true
						 && status_flags.rc_calibration_valid, status);
				status_changed = true;

			} else {
				if (status.rc_signal_lost) {
					mavlink_log_info(&mavlink_log_pub, "Manual control regained after %llums",
							 hrt_elapsed_time(&rc_signal_lost_timestamp) / 1000);
					set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, true
							 && status_flags.rc_calibration_valid, status);
					status_changed = true;
				}
			}

			status.rc_signal_lost = false;

			const bool in_armed_state = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			const bool arm_switch_or_button_mapped = sp_man.arm_switch != manual_control_setpoint_s::SWITCH_POS_NONE;
			const bool arm_button_pressed = arm_switch_is_button == 1
							&& sp_man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON;

			/* DISARM
			 * check if left stick is in lower left position or arm button is pushed or arm switch has transition from arm to disarm
			 * and we are in MANUAL, Rattitude, or AUTO_READY mode or (ASSIST mode and landed)
			 * do it only for rotary wings in manual mode or fixed wing if landed.
			 * Disable stick-disarming if arming switch or button is mapped */
			const bool stick_in_lower_left = sp_man.r < -STICK_ON_OFF_LIMIT && sp_man.z < 0.1f
							 && !arm_switch_or_button_mapped;
			const bool arm_switch_to_disarm_transition =  arm_switch_is_button == 0 &&
					_last_sp_man_arm_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
					sp_man.arm_switch == manual_control_setpoint_s::SWITCH_POS_OFF;

			if (in_armed_state &&
			    status.rc_input_mode != vehicle_status_s::RC_IN_MODE_OFF &&
			    (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || land_detector.landed) &&
			    (stick_in_lower_left || arm_button_pressed || arm_switch_to_disarm_transition)) {

				if (internal_state.main_state != commander_state_s::MAIN_STATE_MANUAL &&
				    internal_state.main_state != commander_state_s::MAIN_STATE_ACRO &&
				    internal_state.main_state != commander_state_s::MAIN_STATE_STAB &&
				    internal_state.main_state != commander_state_s::MAIN_STATE_RATTITUDE &&
				    !land_detector.landed) {
					print_reject_arm("Not disarming! Not yet in manual mode or landed");

				} else if ((stick_off_counter == rc_arm_hyst && stick_on_counter < rc_arm_hyst) || arm_switch_to_disarm_transition) {
					arming_ret = arming_state_transition(&status, safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
									     true /* fRunPreArmChecks */,
									     &mavlink_log_pub, &status_flags, arm_requirements, hrt_elapsed_time(&commander_boot_timestamp));
				}

				stick_off_counter++;

			} else if (!(arm_switch_is_button == 1 && sp_man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON)) {
				/* do not reset the counter when holding the arm button longer than needed */
				stick_off_counter = 0;
			}

			/* ARM
			 * check if left stick is in lower right position or arm button is pushed or arm switch has transition from disarm to arm
			 * and we're in MANUAL mode.
			 * Disable stick-arming if arming switch or button is mapped */
			const bool stick_in_lower_right = sp_man.r > STICK_ON_OFF_LIMIT && sp_man.z < 0.1f
							  && !arm_switch_or_button_mapped;
			/* allow a grace period for re-arming: preflight checks don't need to pass during that time,
			 * for example for accidential in-air disarming */
			const bool in_arming_grace_period = last_disarmed_timestamp != 0 && hrt_elapsed_time(&last_disarmed_timestamp) < 5_s;
			const bool arm_switch_to_arm_transition = arm_switch_is_button == 0 &&
					_last_sp_man_arm_switch == manual_control_setpoint_s::SWITCH_POS_OFF &&
					sp_man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
					(sp_man.z < 0.1f || in_arming_grace_period);

			if (!in_armed_state &&
			    status.rc_input_mode != vehicle_status_s::RC_IN_MODE_OFF &&
			    (stick_in_lower_right || arm_button_pressed || arm_switch_to_arm_transition)) {
				if ((stick_on_counter == rc_arm_hyst && stick_off_counter < rc_arm_hyst) || arm_switch_to_arm_transition) {

					/* we check outside of the transition function here because the requirement
					 * for being in manual mode only applies to manual arming actions.
					 * the system can be armed in auto if armed via the GCS.
					 */

					if ((internal_state.main_state != commander_state_s::MAIN_STATE_MANUAL)
					    && (internal_state.main_state != commander_state_s::MAIN_STATE_ACRO)
					    && (internal_state.main_state != commander_state_s::MAIN_STATE_STAB)
					    && (internal_state.main_state != commander_state_s::MAIN_STATE_ALTCTL)
					    && (internal_state.main_state != commander_state_s::MAIN_STATE_POSCTL)
					    && (internal_state.main_state != commander_state_s::MAIN_STATE_RATTITUDE)
					   ) {
						print_reject_arm("Not arming: Switch to a manual mode first");

					} else if (!status_flags.condition_home_position_valid &&
						   geofence_action == geofence_result_s::GF_ACTION_RTL) {
						print_reject_arm("Not arming: Geofence RTL requires valid home");

					} else if (status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
						arming_ret = arming_state_transition(&status, safety, vehicle_status_s::ARMING_STATE_ARMED, &armed,
										     !in_arming_grace_period /* fRunPreArmChecks */,
										     &mavlink_log_pub, &status_flags, arm_requirements, hrt_elapsed_time(&commander_boot_timestamp));

						if (arming_ret != TRANSITION_CHANGED) {
							px4_usleep(100000);
							print_reject_arm("Not arming: Preflight checks failed");
						}
					}
				}

				stick_on_counter++;

			} else if (!(arm_switch_is_button == 1 && sp_man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON)) {
				/* do not reset the counter when holding the arm button longer than needed */
				stick_on_counter = 0;
			}

			_last_sp_man_arm_switch = sp_man.arm_switch;

			if (arming_ret == TRANSITION_DENIED) {
				/*
				 * the arming transition can be denied to a number of reasons:
				 *  - pre-flight check failed (sensors not ok or not calibrated)
				 *  - safety not disabled
				 *  - system not in manual mode
				 */
				tune_negative(true);
			}

			/* evaluate the main state machine according to mode switches */
			bool first_rc_eval = (_last_sp_man.timestamp == 0) && (sp_man.timestamp > 0);
			transition_result_t main_res = set_main_state(status, &status_changed);

			/* store last position lock state */
			_last_condition_local_altitude_valid = status_flags.condition_local_altitude_valid;
			_last_condition_global_position_valid = status_flags.condition_global_position_valid;

			/* play tune on mode change only if armed, blink LED always */
			if (main_res == TRANSITION_CHANGED || first_rc_eval) {
				tune_positive(armed.armed);
				status_changed = true;

			} else if (main_res == TRANSITION_DENIED) {
				/* DENIED here indicates bug in the commander */
				mavlink_log_critical(&mavlink_log_pub, "Switching to this mode is currently not possible");
			}

			/* check throttle kill switch */
			if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* set lockdown flag */
				if (!armed.manual_lockdown) {
					mavlink_log_emergency(&mavlink_log_pub, "Manual kill-switch engaged");
					status_changed = true;
					armed.manual_lockdown = true;
				}

			} else if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
				if (armed.manual_lockdown) {
					mavlink_log_emergency(&mavlink_log_pub, "Manual kill-switch disengaged");
					status_changed = true;
					armed.manual_lockdown = false;
				}
			}

			/* no else case: do not change lockdown flag in unconfigured case */

		} else {
			if (!status_flags.rc_input_blocked && !status.rc_signal_lost) {
				mavlink_log_critical(&mavlink_log_pub, "Manual control lost");
				status.rc_signal_lost = true;
				rc_signal_lost_timestamp = sp_man.timestamp;
				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, false, status);
				status_changed = true;
			}
		}

		// data link checks which update the status
		data_link_check(status_changed);

		// engine failure detection
		// TODO: move out of commander
		if (actuator_controls_sub.updated()) {
			/* Check engine failure
			 * only for fixed wing for now
			 */
			if (!status_flags.circuit_breaker_engaged_enginefailure_check &&
			    status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !status.is_vtol && armed.armed) {

				actuator_controls_s actuator_controls = {};
				actuator_controls_sub.copy(&actuator_controls);

				const float throttle = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
				const float current2throttle = _battery_current / throttle;

				if (((throttle > ef_throttle_thres) && (current2throttle < ef_current2throttle_thres))
				    || status.engine_failure) {

					const float elapsed = hrt_elapsed_time(&timestamp_engine_healthy) / 1e6f;

					/* potential failure, measure time */
					if ((timestamp_engine_healthy > 0) && (elapsed > ef_time_thres)
					    && !status.engine_failure) {

						status.engine_failure = true;
						status_changed = true;

						PX4_ERR("Engine Failure");
						set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, true, true, false, status);
					}
				}

			} else {
				/* no failure reset flag */
				timestamp_engine_healthy = hrt_absolute_time();

				if (status.engine_failure) {
					status.engine_failure = false;
					status_changed = true;
				}
			}
		}

		/* Reset main state to loiter or auto-mission after takeoff is completed.
		 * Sometimes, the mission result topic is outdated and the mission is still signaled
		 * as finished even though we only just started with the takeoff. Therefore, we also
		 * check the timestamp of the mission_result topic. */
		if (internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF
		    && (_mission_result_sub.get().timestamp > internal_state.timestamp)
		    && _mission_result_sub.get().finished) {

			const bool mission_available = (_mission_result_sub.get().timestamp > commander_boot_timestamp)
						       && (_mission_result_sub.get().instance_count > 0) && _mission_result_sub.get().valid;

			if ((takeoff_complete_act == 1) && mission_available) {
				main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags, &internal_state);

			} else {
				main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &internal_state);
			}
		}

		/* check if we are disarmed and there is a better mode to wait in */
		if (!armed.armed) {

			/* if there is no radio control but GPS lock the user might want to fly using
			 * just a tablet. Since the RC will force its mode switch setting on connecting
			 * we can as well just wait in a hold mode which enables tablet control.
			 */
			if (status.rc_signal_lost && (internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL)
			    && status_flags.condition_home_position_valid) {

				main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &internal_state);
			}
		}

		/* handle commands last, as the system needs to be updated to handle them */
		if (cmd_sub.updated()) {
			vehicle_command_s cmd{};

			/* got command */
			cmd_sub.copy(&cmd);

			/* handle it */
			if (handle_command(&status, cmd, &armed, command_ack_pub, &status_changed)) {
				status_changed = true;
			}
		}

		/* Check for failure detector status */
		const bool failure_detector_updated = _failure_detector.update();

		if (failure_detector_updated) {

			const uint8_t failure_status = _failure_detector.getStatus();

			if (failure_status != status.failure_detector_status) {
				status.failure_detector_status = failure_status;
				status_changed = true;
			}
		}

		if (armed.armed &&
		    failure_detector_updated &&
		    !_flight_termination_triggered &&
		    !status_flags.circuit_breaker_flight_termination_disabled) {

			if (_failure_detector.isFailure()) {

				armed.force_failsafe = true;
				status_changed = true;

				_flight_termination_triggered = true;

				mavlink_log_critical(&mavlink_log_pub, "Critical failure detected: terminate flight");
				set_tune_override(TONE_PARACHUTE_RELEASE_TUNE);
			}
		}

		/* Get current timestamp */
		const hrt_abstime now = hrt_absolute_time();

		// automatically set or update home position
		if (!_home_pub.get().manual_home) {
			const vehicle_local_position_s &local_position = _local_position_sub.get();

			if (armed.armed) {
				if ((!was_armed || (was_landed && !land_detector.landed)) &&
				    (hrt_elapsed_time(&commander_boot_timestamp) > INAIR_RESTART_HOLDOFF_INTERVAL)) {

					/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
					set_home_position();
				}

			} else {
				if (status_flags.condition_home_position_valid) {
					if (land_detector.landed && local_position.xy_valid && local_position.z_valid) {
						/* distance from home */
						float home_dist_xy = -1.0f;
						float home_dist_z = -1.0f;
						mavlink_wpm_distance_to_point_local(_home_pub.get().x, _home_pub.get().y, _home_pub.get().z,
										    local_position.x, local_position.y, local_position.z,
										    &home_dist_xy, &home_dist_z);

						if ((home_dist_xy > local_position.eph * 2.0f) || (home_dist_z > local_position.epv * 2.0f)) {

							/* update when disarmed, landed and moved away from current home position */
							set_home_position();
						}
					}

				} else {
					/* First time home position update - but only if disarmed */
					set_home_position();

					/* Set home position altitude to EKF origin height if home is not set and the EKF has a global origin.
					 * This allows home altitude to be used in the calculation of height above takeoff location when GPS
					 * use has commenced after takeoff. */
					if (!status_flags.condition_home_position_valid) {
						set_home_position_alt_only();
					}
				}
			}
		}

		// check for arming state change
		if (was_armed != armed.armed) {
			status_changed = true;

			if (!armed.armed) { // increase the flight uuid upon disarming
				++flight_uuid;
				// no need for param notification: the only user is mavlink which reads the param upon request
				param_set_no_notification(_param_flight_uuid, &flight_uuid);
				last_disarmed_timestamp = hrt_absolute_time();
			}
		}

		was_armed = armed.armed;

		/* now set navigation state according to failsafe and main state */
		bool nav_state_changed = set_nav_state(&status,
						       &armed,
						       &internal_state,
						       &mavlink_log_pub,
						       (link_loss_actions_t)_param_nav_dll_act.get(),
						       _mission_result_sub.get().finished,
						       _mission_result_sub.get().stay_in_failsafe,
						       status_flags,
						       land_detector.landed,
						       (link_loss_actions_t)_param_nav_rcl_act.get(),
						       _param_com_obl_act.get(),
						       _param_com_obl_rc_act.get(),
						       posctl_nav_loss_act);

		if (status.failsafe != failsafe_old) {
			status_changed = true;

			if (status.failsafe) {
				mavlink_log_info(&mavlink_log_pub, "Failsafe mode activated");

			} else {
				mavlink_log_info(&mavlink_log_pub, "Failsafe mode deactivated");
			}

			failsafe_old = status.failsafe;
		}

		/* publish states (armed, control_mode, vehicle_status, commander_state, vehicle_status_flags) at 1 Hz or immediately when changed */
		if (hrt_elapsed_time(&status.timestamp) >= 1_s || status_changed || nav_state_changed) {

			update_control_mode();

			status.timestamp = hrt_absolute_time();
			_status_pub.publish(status);

			/* set prearmed state if safety is off, or safety is not present and 5 seconds passed */
			if (safety.safety_switch_available) {

				/* safety is off, go into prearmed */
				armed.prearmed = safety.safety_off;

			} else {
				/* safety is not present, go into prearmed
				 * (all output drivers should be started / unlocked last in the boot process
				 * when the rest of the system is fully initialized)
				 */
				armed.prearmed = (hrt_elapsed_time(&commander_boot_timestamp) > 5_s);
			}

			armed.timestamp = hrt_absolute_time();
			_armed_pub.publish(armed);

			/* publish internal state for logging purposes */
			internal_state.timestamp = hrt_absolute_time();
			_commander_state_pub.publish(internal_state);

			/* publish vehicle_status_flags */
			status_flags.timestamp = hrt_absolute_time();
			_vehicle_status_flags_pub.publish(status_flags);
		}

		/* play arming and battery warning tunes */
		if (!arm_tune_played && armed.armed && (!safety.safety_switch_available || (safety.safety_switch_available
							&& safety.safety_off))) {
			/* play tune when armed */
			set_tune(TONE_ARMING_WARNING_TUNE);
			arm_tune_played = true;

		} else if (!status_flags.usb_connected &&
			   (status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			/* play tune on battery critical */
			set_tune(TONE_BATTERY_WARNING_FAST_TUNE);

		} else if ((status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_LOW)) {
			/* play tune on battery warning */
			set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);

		} else if (status.failsafe) {
			tune_failsafe(true);

		} else {
			set_tune(TONE_STOP_TUNE);
		}

		/* reset arm_tune_played when disarmed */
		if (!armed.armed || (safety.safety_switch_available && !safety.safety_off)) {

			//Notify the user that it is safe to approach the vehicle
			if (arm_tune_played) {
				tune_neutral(true);
			}

			arm_tune_played = false;
		}

		/* play sensor failure tunes if we already waited for hotplug sensors to come up and failed */
		status_flags.condition_system_hotplug_timeout = (hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT);

		if (!sensor_fail_tune_played && (!status_flags.condition_system_sensors_initialized
						 && status_flags.condition_system_hotplug_timeout)) {

			set_tune_override(TONE_GPS_WARNING_TUNE);
			sensor_fail_tune_played = true;
			status_changed = true;
		}

		counter++;

		int blink_state = blink_msg_state();

		if (blink_state > 0) {
			/* blinking LED message, don't touch LEDs */
			if (blink_state == 2) {
				/* blinking LED message completed, restore normal state */
				control_status_leds(&status, &armed, true, _battery_warning, &cpuload);
			}

		} else {
			/* normal state */
			control_status_leds(&status, &armed, status_changed, _battery_warning, &cpuload);
		}

		status_changed = false;

		if (!armed.armed) {
			/* Reset the flag if disarmed. */
			have_taken_off_since_arming = false;
		}

		arm_auth_update(now, params_updated || param_init_forced);

		px4_usleep(COMMANDER_MONITORING_INTERVAL);
	}

	thread_should_exit = true;

	/* wait for threads to complete */
	int ret = pthread_join(commander_low_prio_thread, nullptr);

	if (ret) {
		warn("join failed: %d", ret);
	}

	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	/* close fds */
	led_deinit();
	buzzer_deinit();

	thread_running = false;
}

void
get_circuit_breaker_params()
{
	status_flags.circuit_breaker_engaged_power_check = circuit_breaker_enabled("CBRK_SUPPLY_CHK", CBRK_SUPPLY_CHK_KEY);
	status_flags.circuit_breaker_engaged_usb_check = circuit_breaker_enabled("CBRK_USB_CHK", CBRK_USB_CHK_KEY);
	status_flags.circuit_breaker_engaged_airspd_check = circuit_breaker_enabled("CBRK_AIRSPD_CHK", CBRK_AIRSPD_CHK_KEY);
	status_flags.circuit_breaker_engaged_enginefailure_check = circuit_breaker_enabled("CBRK_ENGINEFAIL",
			CBRK_ENGINEFAIL_KEY);
	status_flags.circuit_breaker_engaged_gpsfailure_check = circuit_breaker_enabled("CBRK_GPSFAIL", CBRK_GPSFAIL_KEY);
	status_flags.circuit_breaker_flight_termination_disabled = circuit_breaker_enabled("CBRK_FLIGHTTERM",
			CBRK_FLIGHTTERM_KEY);
	status_flags.circuit_breaker_engaged_posfailure_check = circuit_breaker_enabled("CBRK_VELPOSERR", CBRK_VELPOSERR_KEY);
}

void
Commander::check_valid(const hrt_abstime &timestamp, const hrt_abstime &timeout, const bool valid_in, bool *valid_out,
		       bool *changed)
{
	hrt_abstime t = hrt_absolute_time();
	bool valid_new = (t < timestamp + timeout && t > timeout && valid_in);

	if (*valid_out != valid_new) {
		*valid_out = valid_new;
		*changed = true;
	}
}

void
control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed,
		    bool changed, const uint8_t battery_warning, const cpuload_s *cpuload_local)
{
	static hrt_abstime overload_start = 0;

	bool overload = (cpuload_local->load > 0.80f) || (cpuload_local->ram_usage > 0.98f);

	if (overload_start == 0 && overload) {
		overload_start = hrt_absolute_time();

	} else if (!overload) {
		overload_start = 0;
	}

	/* driving rgbled */
	if (changed || last_overload != overload) {
		uint8_t led_mode = led_control_s::MODE_OFF;
		uint8_t led_color = led_control_s::COLOR_WHITE;
		bool set_normal_color = false;

		uint64_t overload_warn_delay = (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1_ms : 250_ms;

		/* set mode */
		if (overload && (hrt_elapsed_time(&overload_start) > overload_warn_delay)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_PURPLE;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			led_mode = led_control_s::MODE_ON;
			set_normal_color = true;

		} else if (!status_flags.condition_system_sensors_initialized && status_flags.condition_system_hotplug_timeout) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_RED;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (!status_flags.condition_system_sensors_initialized && !status_flags.condition_system_hotplug_timeout) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_INIT) {
			// if in init status it should not be in the error state
			led_mode = led_control_s::MODE_OFF;

		} else {	// STANDBY_ERROR and other states
			led_mode = led_control_s::MODE_BLINK_NORMAL;
			led_color = led_control_s::COLOR_RED;
		}

		if (set_normal_color) {
			/* set color */
			if (status.failsafe) {
				led_color = led_control_s::COLOR_PURPLE;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_LOW) {
				led_color = led_control_s::COLOR_AMBER;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
				led_color = led_control_s::COLOR_RED;

			} else {
				if (status_flags.condition_home_position_valid && status_flags.condition_global_position_valid) {
					led_color = led_control_s::COLOR_GREEN;

				} else {
					led_color = led_control_s::COLOR_BLUE;
				}
			}
		}

		if (led_mode != led_control_s::MODE_OFF) {
			rgbled_set_color_and_mode(led_color, led_mode);
		}
	}

	last_overload = overload;

	/* board supports HW armed indicator */

	BOARD_INDICATE_ARMED_STATE(actuator_armed->armed);

#if !defined(CONFIG_ARCH_LEDS) && defined(BOARD_HAS_CONTROL_STATUS_LEDS)

	/* this runs at around 20Hz, full cycle is 16 ticks = 10/16Hz */
	if (actuator_armed->armed) {
		if (status.failsafe) {
			BOARD_ARMED_LED_OFF();

			if (leds_counter % 5 == 0) {
				BOARD_ARMED_STATE_LED_TOGGLE();
			}

		} else {
			BOARD_ARMED_STATE_LED_OFF();

			/* armed, solid */
			BOARD_ARMED_LED_ON();
		}

	} else if (actuator_armed->ready_to_arm) {
		BOARD_ARMED_LED_OFF();

		/* ready to arm, blink at 1Hz */
		if (leds_counter % 20 == 0) {
			BOARD_ARMED_STATE_LED_TOGGLE();
		}

	} else {
		BOARD_ARMED_LED_OFF();

		/* not ready to arm, blink at 10Hz */
		if (leds_counter % 2 == 0) {
			BOARD_ARMED_STATE_LED_TOGGLE();
		}
	}

#endif

	/* give system warnings on error LED */
	if (overload) {
		if (leds_counter % 2 == 0) {
			BOARD_OVERLOAD_LED_TOGGLE();
		}

	} else {
		BOARD_OVERLOAD_LED_OFF();
	}

	leds_counter++;
}

transition_result_t
Commander::set_main_state(const vehicle_status_s &status_local, bool *changed)
{
	if (safety.override_available && safety.override_enabled) {
		return set_main_state_override_on(status_local, changed);

	} else {
		return set_main_state_rc(status_local, changed);
	}
}

transition_result_t
Commander::set_main_state_override_on(const vehicle_status_s &status_local, bool *changed)
{
	transition_result_t res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags,
				  &internal_state);
	*changed = (res == TRANSITION_CHANGED);

	return res;
}

transition_result_t
Commander::set_main_state_rc(const vehicle_status_s &status_local, bool *changed)
{
	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_DENIED;

	// Note: even if status_flags.offboard_control_set_by_command is set
	// we want to allow rc mode change to take precidence.  This is a safety
	// feature, just in case offboard control goes crazy.

	const bool altitude_got_valid = !_last_condition_local_altitude_valid && status_flags.condition_local_altitude_valid;
	const bool position_got_valid = !_last_condition_global_position_valid && status_flags.condition_global_position_valid;
	const bool first_time_rc = _last_sp_man.timestamp == 0;
	const bool rc_values_updated = _last_sp_man.timestamp != sp_man.timestamp;
	const bool some_switch_changed =
		(_last_sp_man.offboard_switch != sp_man.offboard_switch)
		|| (_last_sp_man.return_switch != sp_man.return_switch)
		|| (_last_sp_man.mode_switch != sp_man.mode_switch)
		|| (_last_sp_man.acro_switch != sp_man.acro_switch)
		|| (_last_sp_man.rattitude_switch != sp_man.rattitude_switch)
		|| (_last_sp_man.posctl_switch != sp_man.posctl_switch)
		|| (_last_sp_man.loiter_switch != sp_man.loiter_switch)
		|| (_last_sp_man.mode_slot != sp_man.mode_slot)
		|| (_last_sp_man.stab_switch != sp_man.stab_switch)
		|| (_last_sp_man.man_switch != sp_man.man_switch);

	// only switch mode based on RC switch if necessary to also allow mode switching via MAVLink
	const bool should_evaluate_rc_mode_switch = first_time_rc
			|| altitude_got_valid
			|| position_got_valid
			|| (rc_values_updated && some_switch_changed);

	if (!should_evaluate_rc_mode_switch) {

		// store the last manual control setpoint set by the pilot in a manual state
		// if the system now later enters an autonomous state the pilot can move
		// the sticks to break out of the autonomous state

		if (!_geofence_warning_action_on
		    && (internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL ||
			internal_state.main_state == commander_state_s::MAIN_STATE_ALTCTL ||
			internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL ||
			internal_state.main_state == commander_state_s::MAIN_STATE_ACRO ||
			internal_state.main_state == commander_state_s::MAIN_STATE_RATTITUDE ||
			internal_state.main_state == commander_state_s::MAIN_STATE_STAB)) {

			_last_sp_man.timestamp = sp_man.timestamp;
			_last_sp_man.x = sp_man.x;
			_last_sp_man.y = sp_man.y;
			_last_sp_man.z = sp_man.z;
			_last_sp_man.r = sp_man.r;
		}

		/* no timestamp change or no switch change -> nothing changed */
		return TRANSITION_NOT_CHANGED;
	}

	_last_sp_man = sp_man;

	// reset the position and velocity validity calculation to give the best change of being able to select
	// the desired mode
	reset_posvel_validity(changed);

	/* offboard switch overrides main switch */
	if (sp_man.offboard_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_OFFBOARD, status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode("OFFBOARD");
			/* mode rejected, continue to evaluate the main system mode */

		} else {
			/* changed successfully or already in this state */
			return res;
		}
	}

	/* RTL switch overrides main switch */
	if (sp_man.return_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode("AUTO RTL");

			/* fallback to LOITER if home position not set */
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &internal_state);
		}

		if (res != TRANSITION_DENIED) {
			/* changed successfully or already in this state */
			return res;
		}

		/* if we get here mode was rejected, continue to evaluate the main system mode */
	}

	/* Loiter switch overrides main switch */
	if (sp_man.loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode("AUTO HOLD");

		} else {
			return res;
		}
	}

	/* we know something has changed - check if we are in mode slot operation */
	if (sp_man.mode_slot != manual_control_setpoint_s::MODE_SLOT_NONE) {

		if (sp_man.mode_slot > manual_control_setpoint_s::MODE_SLOT_NUM) {
			warnx("m slot overflow");
			return TRANSITION_DENIED;
		}

		int new_mode = _flight_mode_slots[sp_man.mode_slot - 1];

		if (new_mode < 0) {
			/* slot is unused */
			res = TRANSITION_NOT_CHANGED;

		} else {
			res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

			/* ensure that the mode selection does not get stuck here */
			int maxcount = 5;

			/* enable the use of break */
			/* fallback strategies, give the user the closest mode to what he wanted */
			while (res == TRANSITION_DENIED && maxcount > 0) {

				maxcount--;

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_MISSION) {

					/* fall back to loiter */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO MISSION");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_RTL) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO RTL");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LAND) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO LAND");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO TAKEOFF");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO FOLLOW");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LOITER) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_POSCTL;
					print_reject_mode("AUTO HOLD");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_POSCTL) {

					/* fall back to altitude control */
					new_mode = commander_state_s::MAIN_STATE_ALTCTL;
					print_reject_mode("POSITION CONTROL");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_ALTCTL) {

					/* fall back to stabilized */
					new_mode = commander_state_s::MAIN_STATE_STAB;
					print_reject_mode("ALTITUDE CONTROL");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_STAB) {

					/* fall back to manual */
					new_mode = commander_state_s::MAIN_STATE_MANUAL;
					print_reject_mode("STABILIZED");
					res = main_state_transition(status_local, new_mode, status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}
			}
		}

		return res;
	}

	/* offboard and RTL switches off or denied, check main mode switch */
	switch (sp_man.mode_switch) {
	case manual_control_setpoint_s::SWITCH_POS_NONE:
		res = TRANSITION_NOT_CHANGED;
		break;

	case manual_control_setpoint_s::SWITCH_POS_OFF:		// MANUAL
		if (sp_man.stab_switch == manual_control_setpoint_s::SWITCH_POS_NONE &&
		    sp_man.man_switch == manual_control_setpoint_s::SWITCH_POS_NONE) {
			/*
			 * Legacy mode:
			 * Acro switch being used as stabilized switch in FW.
			 */
			if (sp_man.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* manual mode is stabilized already for multirotors, so switch to acro
				 * for any non-manual mode
				 */
				if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, status_flags, &internal_state);

				} else if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &internal_state);

				} else {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);
				}

			} else if (sp_man.rattitude_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* Similar to acro transitions for multirotors.  FW aircraft don't need a
				 * rattitude mode.*/
				if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, status_flags, &internal_state);

				} else {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &internal_state);
				}

			} else {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);
			}

		} else {
			/* New mode:
			 * - Acro is Acro
			 * - Manual is not default anymore when the manaul switch is assigned
			 */
			if (sp_man.man_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);

			} else if (sp_man.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, status_flags, &internal_state);

			} else if (sp_man.rattitude_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, status_flags, &internal_state);

			} else if (sp_man.stab_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &internal_state);

			} else if (sp_man.man_switch == manual_control_setpoint_s::SWITCH_POS_NONE) {
				// default to MANUAL when no manual switch is set
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);

			} else {
				// default to STAB when the manual switch is assigned (but off)
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &internal_state);
			}
		}

		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_MIDDLE:		// ASSIST
		if (sp_man.posctl_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &internal_state);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode("POSITION CONTROL");
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this mode
		}

		if (sp_man.posctl_switch != manual_control_setpoint_s::SWITCH_POS_ON) {
			print_reject_mode("ALTITUDE CONTROL");
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_ON:			// AUTO
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		print_reject_mode("AUTO MISSION");

		// fallback to LOITER if home position not set
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to POSCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	default:
		break;
	}

	return res;
}

void
Commander::reset_posvel_validity(bool *changed)
{
	// reset all the check probation times back to the minimum value
	_gpos_probation_time_us = POSVEL_PROBATION_MIN;
	_lpos_probation_time_us = POSVEL_PROBATION_MIN;
	_lvel_probation_time_us = POSVEL_PROBATION_MIN;

	const vehicle_local_position_s &local_position = _local_position_sub.get();
	const vehicle_global_position_s &global_position = _global_position_sub.get();

	// recheck validity
	if (!_skip_pos_accuracy_check) {
		check_posvel_validity(true, global_position.eph, _eph_threshold_adj, global_position.timestamp,
				      &_last_gpos_fail_time_us, &_gpos_probation_time_us, &status_flags.condition_global_position_valid, changed);
	}

	check_posvel_validity(local_position.xy_valid, local_position.eph, _eph_threshold_adj, local_position.timestamp,
			      &_last_lpos_fail_time_us, &_lpos_probation_time_us, &status_flags.condition_local_position_valid, changed);
	check_posvel_validity(local_position.v_xy_valid, local_position.evh, _param_com_vel_fs_evh.get(),
			      local_position.timestamp,
			      &_last_lvel_fail_time_us, &_lvel_probation_time_us, &status_flags.condition_local_velocity_valid, changed);
}

bool
Commander::check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				 const hrt_abstime &data_timestamp_us, hrt_abstime *last_fail_time_us, hrt_abstime *probation_time_us, bool *valid_state,
				 bool *validity_changed)
{
	const bool was_valid = *valid_state;
	bool valid = was_valid;

	// constrain probation times
	if (land_detector.landed) {
		*probation_time_us = POSVEL_PROBATION_MIN;
	}

	const bool data_stale = ((hrt_elapsed_time(&data_timestamp_us) > _param_com_pos_fs_delay.get() * 1_s)
				 || (data_timestamp_us == 0));
	const float req_accuracy = (was_valid ? required_accuracy * 2.5f : required_accuracy);

	const bool level_check_pass = data_valid && !data_stale && (data_accuracy < req_accuracy);

	// Check accuracy with hysteresis in both test level and time
	if (level_check_pass) {
		if (was_valid) {
			// still valid, continue to decrease probation time
			const int64_t probation_time_new = *probation_time_us - hrt_elapsed_time(last_fail_time_us);
			*probation_time_us = math::constrain(probation_time_new, POSVEL_PROBATION_MIN, POSVEL_PROBATION_MAX);

		} else {
			// check if probation period has elapsed
			if (hrt_elapsed_time(last_fail_time_us) > *probation_time_us) {
				valid = true;
			}
		}

	} else {
		// level check failed
		if (was_valid) {
			// FAILURE! no longer valid
			valid = false;

		} else {
			// failed again, increase probation time
			const int64_t probation_time_new = *probation_time_us + hrt_elapsed_time(last_fail_time_us) *
							   _param_com_pos_fs_gain.get();
			*probation_time_us = math::constrain(probation_time_new, POSVEL_PROBATION_MIN, POSVEL_PROBATION_MAX);
		}

		*last_fail_time_us = hrt_absolute_time();
	}

	if (was_valid != valid) {
		*validity_changed = true;
		*valid_state = valid;
	}

	return valid;
}

void
Commander::update_control_mode()
{
	vehicle_control_mode_s control_mode{};

	control_mode.timestamp = hrt_absolute_time();

	/* set vehicle_control_mode according to set_navigation_state */
	control_mode.flag_armed = armed.armed;
	control_mode.flag_external_manual_override_ok = (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
			&& !status.is_vtol);

	switch (status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = stabilization_required();
		control_mode.flag_control_attitude_enabled = stabilization_required();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
		/* override is not ok for the RTL and recovery mode */
		control_mode.flag_external_manual_override_ok = false;

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		/* TODO: check if this makes sense */
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: {

			const offboard_control_mode_s &offboard_control_mode = _offboard_control_mode_sub.get();

			control_mode.flag_control_offboard_enabled = true;

			/*
			 * The control flags depend on what is ignored according to the offboard control mode topic
			 * Inner loop flags (e.g. attitude) also depend on outer loop ignore flags (e.g. position)
			 */
			control_mode.flag_control_rates_enabled =
				!offboard_control_mode.ignore_bodyrate_x ||
				!offboard_control_mode.ignore_bodyrate_y ||
				!offboard_control_mode.ignore_bodyrate_z ||
				!offboard_control_mode.ignore_attitude ||
				!offboard_control_mode.ignore_position ||
				!offboard_control_mode.ignore_velocity ||
				!offboard_control_mode.ignore_acceleration_force;

			control_mode.flag_control_attitude_enabled = !offboard_control_mode.ignore_attitude ||
					!offboard_control_mode.ignore_position ||
					!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_acceleration_force;

			// TO-DO: Add support for other modes than yawrate control
			control_mode.flag_control_yawrate_override_enabled =
				offboard_control_mode.ignore_bodyrate_x &&
				offboard_control_mode.ignore_bodyrate_y &&
				!offboard_control_mode.ignore_bodyrate_z &&
				!offboard_control_mode.ignore_attitude;

			control_mode.flag_control_rattitude_enabled = false;

			control_mode.flag_control_acceleration_enabled = !offboard_control_mode.ignore_acceleration_force &&
					!status.in_transition_mode;

			control_mode.flag_control_velocity_enabled = (!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_position) && !status.in_transition_mode &&
					!control_mode.flag_control_acceleration_enabled;

			control_mode.flag_control_climb_rate_enabled = (!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

			control_mode.flag_control_position_enabled = !offboard_control_mode.ignore_position && !status.in_transition_mode &&
					!control_mode.flag_control_acceleration_enabled;

			control_mode.flag_control_altitude_enabled = (!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

		}
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	default:
		break;
	}

	_control_mode_pub.publish(control_mode);
}

bool
stabilization_required()
{
	return (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ||		// is a rotary wing, or
		status.vtol_fw_permanent_stab || 	// is a VTOL in fixed wing mode and stabilisation is on, or
		(vtol_status.vtol_in_trans_mode && 	// is currently a VTOL transitioning AND
		 status.vehicle_type ==
		 vehicle_status_s::VEHICLE_TYPE_FIXED_WING));	// is a fixed wing, ie: transitioning back to rotary wing mode
}

void
print_reject_mode(const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		mavlink_log_critical(&mavlink_log_pub, "REJECT %s", msg);

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
		mavlink_log_critical(&mavlink_log_pub, "%s", msg);
		tune_negative(true);
	}
}

void answer_command(const vehicle_command_s &cmd, unsigned result,
		    uORB::PublicationQueued<vehicle_command_ack_s> &command_ack_pub)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		tune_positive(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		tune_negative(true);
		break;

	default:
		break;
	}

	/* publish ACK */
	vehicle_command_ack_s command_ack{};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.result = (uint8_t)result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;


	command_ack_pub.publish(command_ack);
}

void *commander_low_prio_loop(void *arg)
{
	/* Set thread name */
	px4_prctl(PR_SET_NAME, "commander_low_prio", px4_getpid());

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* command ack */
	uORB::PublicationQueued<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			warn("commander: poll error %d, %d", pret, errno);
			continue;

		} else if (pret != 0) {
			struct vehicle_command_s cmd {};

			/* if we reach here, we have a valid command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* ignore commands the high-prio loop or the navigator handles */
			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_SERVO ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {

				continue;
			}

			/* only handle low-priority commands here */
			switch (cmd.command) {

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				if (is_safe(safety, armed)) {

					if (((int)(cmd.param1)) == 1) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						px4_usleep(100000);
						/* reboot */
						px4_shutdown_request(true, false);

					} else if (((int)(cmd.param1)) == 2) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						px4_usleep(100000);
						/* shutdown */
						px4_shutdown_request(false, false);

					} else if (((int)(cmd.param1)) == 3) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						px4_usleep(100000);
						/* reboot to bootloader */
						px4_shutdown_request(true, true);

					} else {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
					}

				} else {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
				}

				break;

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

					int calib_ret = PX4_ERROR;

					/* try to go to INIT/PREFLIGHT arming state */
					if (TRANSITION_DENIED == arming_state_transition(&status, safety, vehicle_status_s::ARMING_STATE_INIT, &armed,
							false /* fRunPreArmChecks */, &mavlink_log_pub, &status_flags,
							arm_requirements, hrt_elapsed_time(&commander_boot_timestamp))) {

						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
						break;

					} else {
						status_flags.condition_calibration_enabled = true;
					}

					if ((int)(cmd.param1) == 1) {
						/* gyro calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_gyro_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
						   (int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
						   (int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
						/* temperature calibration: handled in events module */
						break;

					} else if ((int)(cmd.param2) == 1) {
						/* magnetometer calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_mag_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param3) == 1) {
						/* zero-altitude pressure calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);

					} else if ((int)(cmd.param4) == 1) {
						/* RC calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						/* disable RC control input completely */
						status_flags.rc_input_blocked = true;
						calib_ret = OK;
						mavlink_log_info(&mavlink_log_pub, "Calibration: Disabling RC input");

					} else if ((int)(cmd.param4) == 2) {
						/* RC trim calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_trim_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param5) == 1) {
						/* accelerometer calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_accel_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param5) == 2) {
						// board offset calibration
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_level_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
						// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
						/* airspeed calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_airspeed_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param7) == 1) {
						/* do esc calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);

					} else if ((int)(cmd.param4) == 0) {
						/* RC calibration ended - have we been in one worth confirming? */
						if (status_flags.rc_input_blocked) {
							/* enable RC control input */
							status_flags.rc_input_blocked = false;
							mavlink_log_info(&mavlink_log_pub, "Calibration: Restoring RC input");
						}

						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						/* this always succeeds */
						calib_ret = OK;

					} else {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED, command_ack_pub);
					}

					status_flags.condition_calibration_enabled = false;

					if (calib_ret == OK) {
						tune_positive(true);

						Commander::preflight_check(false);

						arming_state_transition(&status, safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
									false /* fRunPreArmChecks */,
									&mavlink_log_pub, &status_flags, arm_requirements, hrt_elapsed_time(&commander_boot_timestamp));

					} else {
						tune_negative(true);
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

					if (((int)(cmd.param1)) == 0) {
						int ret = param_load_default();

						if (ret == OK) {
							mavlink_log_info(&mavlink_log_pub, "Settings loaded");
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

						} else {
							mavlink_log_critical(&mavlink_log_pub, "Error loading settings");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(&mavlink_log_pub, "Error: %s", strerror(ret));
							}

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
						}

					} else if (((int)(cmd.param1)) == 1) {

#ifdef __PX4_QURT
						// TODO FIXME: on snapdragon the save happens too early when the params
						// are not set yet. We therefore need to wait some time first.
						px4_usleep(1000000);
#endif

						int ret = param_save_default();

						if (ret == OK) {
							/* do not spam MAVLink, but provide the answer / green led mechanism */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

						} else {
							mavlink_log_critical(&mavlink_log_pub, "Error saving settings");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(&mavlink_log_pub, "Error: %s", strerror(ret));
							}

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
						}

					} else if (((int)(cmd.param1)) == 2) {

						/* reset parameters and save empty file */
						param_reset_all();

						/* do not spam MAVLink, but provide the answer / green led mechanism */
						mavlink_log_critical(&mavlink_log_pub, "Onboard parameters reset");
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
				/* just ack, implementation handled in the IO driver */
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
				break;

			default:
				/* don't answer on unsupported commands, it will be done in main loop */
				break;
			}
		}
	}

	px4_close(cmd_sub);

	return nullptr;
}

int Commander::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Commander::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	return 0;
}

int Commander::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("commander",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 40,
				      3250,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Commander *Commander::instantiate(int argc, char *argv[])
{
	Commander *instance = new Commander();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-h")) {
			instance->enable_hil();
		}
	}

	return instance;
}

void Commander::enable_hil()
{
	status.hil_state = vehicle_status_s::HIL_STATE_ON;
}

void Commander::mission_init()
{
	/* init mission state, do it here to allow navigator to use stored mission even if mavlink failed to start */
	mission_s mission = {};

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
			if (mission.count > 0) {
				PX4_INFO("Mission #%d loaded, %u WPs, curr: %d", mission.dataman_id, mission.count, mission.current_seq);
			}

		} else {
			PX4_ERR("reading mission state failed");

			/* initialize mission state in dataman */
			mission.timestamp = hrt_absolute_time();
			mission.dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
			dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
		}

		orb_advert_t mission_pub = orb_advertise(ORB_ID(mission), &mission);
		orb_unadvertise(mission_pub);
	}
}

bool Commander::preflight_check(bool report)
{
	const bool checkGNSS = (arm_requirements & ARM_REQ_GPS_BIT);

	bool success = Preflight::preflightCheck(&mavlink_log_pub, status, status_flags, checkGNSS, report, false,
			hrt_elapsed_time(&commander_boot_timestamp));

	if (success) {
		status_flags.condition_system_sensors_initialized = true;
	}

	return success;
}

void Commander::data_link_check(bool &status_changed)
{
	if (_telemetry_status_sub.updated()) {

		telemetry_status_s telemetry;

		if (_telemetry_status_sub.copy(&telemetry)) {

			// handle different radio types
			switch (telemetry.type) {
			case telemetry_status_s::LINK_TYPE_USB:
				// set (but don't unset) telemetry via USB as active once a MAVLink connection is up
				status_flags.usb_connected = true;
				break;

			case telemetry_status_s::LINK_TYPE_IRIDIUM: {

					iridiumsbd_status_s iridium_status;

					if (_iridiumsbd_status_sub.update(&iridium_status)) {
						_high_latency_datalink_heartbeat = iridium_status.last_heartbeat;

						if (status.high_latency_data_link_lost) {
							if (hrt_elapsed_time(&_high_latency_datalink_lost) > (_param_com_hldl_reg_t.get() * 1_s)) {
								status.high_latency_data_link_lost = false;
								status_changed = true;
							}
						}
					}

					break;
				}
			}

			// handle different remote types
			switch (telemetry.remote_type) {
			case telemetry_status_s::MAV_TYPE_GCS:

				// Recover from data link lost
				if (status.data_link_lost) {
					if (telemetry.heartbeat_time > _datalink_last_heartbeat_gcs) {
						status.data_link_lost = false;
						status_changed = true;

						if (_datalink_last_heartbeat_gcs != 0) {
							mavlink_log_info(&mavlink_log_pub, "Data link regained");
						}
					}
				}

				// Only keep the very last heartbeat timestamp, so we don't get confused
				// by multiple mavlink instances publishing different timestamps.
				if (telemetry.heartbeat_time > _datalink_last_heartbeat_gcs) {
					_datalink_last_heartbeat_gcs = telemetry.heartbeat_time;
				}

				break;

			case telemetry_status_s::MAV_TYPE_ONBOARD_CONTROLLER:

				if (_onboard_controller_lost) {
					if (telemetry.heartbeat_time > _datalink_last_heartbeat_onboard_controller) {
						mavlink_log_info(&mavlink_log_pub, "Onboard controller regained");
						_onboard_controller_lost = false;
						status_changed = true;
					}

				}

				_datalink_last_heartbeat_onboard_controller = telemetry.heartbeat_time;

				if (telemetry.remote_component_id == telemetry_status_s::COMPONENT_ID_OBSTACLE_AVOIDANCE) {
					if (telemetry.heartbeat_time != _datalink_last_heartbeat_avoidance_system) {
						_avoidance_system_status_change = _datalink_last_status_avoidance_system != telemetry.remote_system_status;
					}

					_datalink_last_heartbeat_avoidance_system = telemetry.heartbeat_time;
					_datalink_last_status_avoidance_system = telemetry.remote_system_status;

					if (_avoidance_system_lost) {
						mavlink_log_info(&mavlink_log_pub, "Avoidance system regained");
						status_changed = true;
						_avoidance_system_lost = false;
						status_flags.avoidance_system_valid = true;
					}
				}

				break;
			}
		}
	}


	// GCS data link loss failsafe
	if (!status.data_link_lost) {
		if (_datalink_last_heartbeat_gcs != 0
		    && hrt_elapsed_time(&_datalink_last_heartbeat_gcs) > (_param_com_dl_loss_t.get() * 1_s)) {

			status.data_link_lost = true;
			status.data_link_lost_counter++;

			mavlink_log_critical(&mavlink_log_pub, "Data link lost");

			status_changed = true;
		}
	}

	// ONBOARD CONTROLLER data link loss failsafe (hard coded 5 seconds)
	if ((_datalink_last_heartbeat_onboard_controller > 0)
	    && (hrt_elapsed_time(&_datalink_last_heartbeat_onboard_controller) > 5_s)
	    && !_onboard_controller_lost) {

		mavlink_log_critical(&mavlink_log_pub, "Onboard controller lost");
		_onboard_controller_lost = true;
		status_changed = true;
	}

	// AVOIDANCE SYSTEM state check (only if it is enabled)
	if (status_flags.avoidance_system_required && !_onboard_controller_lost) {

		//if avoidance never started
		if (_datalink_last_heartbeat_avoidance_system == 0
		    && hrt_elapsed_time(&_datalink_last_heartbeat_avoidance_system) > _param_com_oa_boot_t.get() * 1_s) {
			if (!_print_avoidance_msg_once) {
				mavlink_log_critical(&mavlink_log_pub, "Avoidance system not available");
				_print_avoidance_msg_once = true;

			}
		}

		//if heartbeats stop
		if (!_avoidance_system_lost && (_datalink_last_heartbeat_avoidance_system > 0)
		    && (hrt_elapsed_time(&_datalink_last_heartbeat_avoidance_system) > 5_s)) {
			_avoidance_system_lost = true;
			mavlink_log_critical(&mavlink_log_pub, "Avoidance system lost");
			status_flags.avoidance_system_valid = false;
			_print_avoidance_msg_once = false;
		}

		//if status changed
		if (_avoidance_system_status_change) {
			if (_datalink_last_status_avoidance_system == telemetry_status_s::MAV_STATE_BOOT) {
				mavlink_log_info(&mavlink_log_pub, "Avoidance system starting");
			}

			if (_datalink_last_status_avoidance_system == telemetry_status_s::MAV_STATE_ACTIVE) {
				mavlink_log_info(&mavlink_log_pub, "Avoidance system connected");
				status_flags.avoidance_system_valid = true;
			}

			if (_datalink_last_status_avoidance_system == telemetry_status_s::MAV_STATE_CRITICAL) {
				mavlink_log_info(&mavlink_log_pub, "Avoidance system timed out");
			}

			if (_datalink_last_status_avoidance_system == telemetry_status_s::MAV_STATE_FLIGHT_TERMINATION) {
				mavlink_log_critical(&mavlink_log_pub, "Avoidance system rejected");
				status_flags.avoidance_system_valid = false;
				status_changed = true;
			}

			_avoidance_system_status_change = false;
		}
	}


	// high latency data link loss failsafe
	if (_high_latency_datalink_heartbeat > 0
	    && hrt_elapsed_time(&_high_latency_datalink_heartbeat) > (_param_com_hldl_loss_t.get() * 1_s)) {
		_high_latency_datalink_lost = hrt_absolute_time();

		if (!status.high_latency_data_link_lost) {
			status.high_latency_data_link_lost = true;
			mavlink_log_critical(&mavlink_log_pub, "High latency data link lost");
			status_changed = true;
		}
	}
}

void Commander::battery_status_check()
{
	/* update battery status */
	if (_battery_sub.updated()) {
		battery_status_s battery{};

		if (_battery_sub.copy(&battery)) {

			if ((hrt_elapsed_time(&battery.timestamp) < 5_s)
			    && battery.connected
			    && (_battery_warning == battery_status_s::BATTERY_WARNING_NONE)) {

				status_flags.condition_battery_healthy = true;

			} else {
				status_flags.condition_battery_healthy = false;
			}

			// execute battery failsafe if the state has gotten worse
			if (armed.armed) {
				if (battery.warning > _battery_warning) {
					battery_failsafe(&mavlink_log_pub, status, status_flags, &internal_state, battery.warning,
							 (low_battery_action_t)_param_com_low_bat_act.get());
				}
			}

			// Handle shutdown request from emergency battery action
			if (battery.warning != _battery_warning) {

				if ((battery.warning == battery_status_s::BATTERY_WARNING_EMERGENCY) && shutdown_if_allowed()) {
					mavlink_log_critical(&mavlink_log_pub, "Dangerously low battery! Shutting system down");
					px4_usleep(200000);

					int ret_val = px4_shutdown_request(false, false);

					if (ret_val) {
						mavlink_log_critical(&mavlink_log_pub, "System does not support shutdown");

					} else {
						while (1) { px4_usleep(1); }
					}
				}
			}

			// save last value
			_battery_warning = battery.warning;
			_battery_current = battery.current_filtered_a;
		}
	}
}

void Commander::airspeed_use_check()
{
	if (_airspeed_fail_action.get() < 1 || _airspeed_fail_action.get() > 4) {
		// disabled
		return;
	}

	_airspeed_sub.update();
	const airspeed_s &airspeed = _airspeed_sub.get();

	vehicle_acceleration_s accel{};
	_vehicle_acceleration_sub.copy(&accel);

	bool is_fixed_wing = status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	// assume airspeed sensor is good before starting FW flight
	bool valid_flight_condition = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) &&
				      is_fixed_wing && !land_detector.landed;
	bool fault_declared = false;
	bool fault_cleared = false;
	bool bad_number_fail = !PX4_ISFINITE(airspeed.indicated_airspeed_m_s) || !PX4_ISFINITE(airspeed.true_airspeed_m_s);

	if (!valid_flight_condition) {

		_tas_check_fail = false;
		_time_last_tas_pass = hrt_absolute_time();
		_time_last_tas_fail = 0;

		_tas_use_inhibit = false;
		_time_tas_good_declared = hrt_absolute_time();
		_time_tas_bad_declared = 0;

		status.aspd_check_failing = false;
		status.aspd_fault_declared = false;
		status.aspd_use_inhibit = false;
		status.aspd_fail_rtl = false;
		status.arspd_check_level = 0.0f;

		_time_last_airspeed = hrt_absolute_time();
		_time_last_aspd_innov_check = hrt_absolute_time();
		_load_factor_ratio = 0.5f;

	} else {

		// Check normalised innovation levels with requirement for continuous data and use of hysteresis
		// to prevent false triggering.
		float dt_s = float(1e-6 * (double)hrt_elapsed_time(&_time_last_aspd_innov_check));

		if (dt_s < 1.0f) {
			if (_estimator_status_sub.get().tas_test_ratio <= _tas_innov_threshold.get()) {
				// record pass and reset integrator used to trigger
				_time_last_tas_pass = hrt_absolute_time();
				_apsd_innov_integ_state = 0.0f;

			} else {
				// integrate exceedance
				_apsd_innov_integ_state += dt_s * (_estimator_status_sub.get().tas_test_ratio - _tas_innov_threshold.get());
			}

			status.arspd_check_level = _apsd_innov_integ_state;

			if ((_estimator_status_sub.get().vel_test_ratio < 1.0f) && (_estimator_status_sub.get().mag_test_ratio < 1.0f)) {
				// nav velocity data is likely good so airspeed innovations are able to be used
				if ((_tas_innov_integ_threshold.get() > 0.0f) && (_apsd_innov_integ_state > _tas_innov_integ_threshold.get())) {
					_time_last_tas_fail = hrt_absolute_time();
				}
			}

			if (!_tas_check_fail) {
				_tas_check_fail = (hrt_elapsed_time(&_time_last_tas_pass) > TAS_INNOV_FAIL_DELAY);

			} else {
				_tas_check_fail = (hrt_elapsed_time(&_time_last_tas_fail) < TAS_INNOV_FAIL_DELAY);
			}
		}

		_time_last_aspd_innov_check = hrt_absolute_time();


		// The vehicle is flying so use the status of the airspeed innovation check '_tas_check_fail' in
		// addition to a sanity check using airspeed and load factor and a missing sensor data check.

		// Check if sensor data is missing - assume a minimum 5Hz data rate.
		const bool data_missing = (hrt_elapsed_time(&_time_last_airspeed) > 200_ms);

		// Declare data stopped if not received for longer than 1 second
		const bool data_stopped = (hrt_elapsed_time(&_time_last_airspeed) > 1_s);

		_time_last_airspeed = hrt_absolute_time();

		// Check if the airpeed reading is lower than physically possible given the load factor
		bool load_factor_ratio_fail = true;

		if (!bad_number_fail) {
			float max_lift_ratio = fmaxf(airspeed.indicated_airspeed_m_s, 0.7f) / fmaxf(_airspeed_stall.get(), 1.0f);
			max_lift_ratio *= max_lift_ratio;

			_load_factor_ratio = 0.95f * _load_factor_ratio + 0.05f * (fabsf(accel.xyz[2]) / CONSTANTS_ONE_G) / max_lift_ratio;
			_load_factor_ratio = math::constrain(_load_factor_ratio, 0.25f, 2.0f);
			load_factor_ratio_fail = (_load_factor_ratio > 1.1f);
			status.load_factor_ratio = _load_factor_ratio;

			// sanity check independent of stall speed and load factor calculation
			if (airspeed.indicated_airspeed_m_s <= 0.0f) {
				bad_number_fail = true;
			}
		}

		//  Decide if the control loops should be using the airspeed data based on the length of time the
		// airspeed data has been declared bad
		if (_tas_check_fail || load_factor_ratio_fail || data_missing || bad_number_fail) {
			// either load factor or EKF innovation or missing data test failure can declare the airspeed bad
			_time_tas_bad_declared = hrt_absolute_time();
			status.aspd_check_failing = true;

		} else if (!_tas_check_fail && !load_factor_ratio_fail && !data_missing && !bad_number_fail) {
			// All checks must pass to declare airspeed good
			_time_tas_good_declared = hrt_absolute_time();
			status.aspd_check_failing = false;
		}

		if (!_tas_use_inhibit) {
			// A simultaneous load factor and innovaton check fail makes it more likely that a large
			// airspeed measurement fault has developed, so a fault should be declared immediately
			const bool both_checks_failed = (_tas_check_fail && load_factor_ratio_fail);

			// Because the innovation, load factor and data missing checks are subject to short duration false positives
			// a timeout period is applied.
			const bool single_check_fail_timeout = (hrt_elapsed_time(&_time_tas_good_declared) > (_tas_use_stop_delay.get() * 1_s));

			if (data_stopped || both_checks_failed || single_check_fail_timeout || bad_number_fail) {

				_tas_use_inhibit = true;
				fault_declared = true;

				if (data_stopped || data_missing) {
					strcpy(_airspeed_fault_type, "MISSING");

				} else  {
					strcpy(_airspeed_fault_type, "FAULTY ");
				}
			}

		} else if (hrt_elapsed_time(&_time_tas_bad_declared) > (_tas_use_start_delay.get() * 1_s)) {
			_tas_use_inhibit = false;
			fault_cleared = true;
		}
	}

	// Do actions based on value of COM_ASPD_FS_ACT parameter
	status.aspd_fault_declared = false;
	status.aspd_use_inhibit = false;
	status.aspd_fail_rtl = false;

	switch (_airspeed_fail_action.get()) {
	case 4: { // log a message, warn the user, switch to non-airspeed TECS mode, switch to Return mode if not in a pilot controlled mode.
			if (fault_declared) {
				status.aspd_fault_declared = true;
				status.aspd_use_inhibit = true;

				if ((internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL)
				    || (internal_state.main_state == commander_state_s::MAIN_STATE_ACRO)
				    || (internal_state.main_state == commander_state_s::MAIN_STATE_STAB)
				    || (internal_state.main_state == commander_state_s::MAIN_STATE_ALTCTL)
				    || (internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL)
				    || (internal_state.main_state == commander_state_s::MAIN_STATE_RATTITUDE)) {

					// don't RTL if pilot is in control
					mavlink_log_critical(&mavlink_log_pub, "ASPD DATA %s - stopping use", _airspeed_fault_type);

				} else if (hrt_elapsed_time(&_time_tas_good_declared) < (_airspeed_rtl_delay.get() * 1_s)) {
					// Wait for timeout and issue message
					mavlink_log_critical(&mavlink_log_pub, "ASPD DATA %s - stopping use, RTL in %i sec", _airspeed_fault_type,
							     _airspeed_rtl_delay.get());

				} else if (TRANSITION_DENIED != main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags,
						&internal_state)) {

					// Issue critical message even if already in RTL
					status.aspd_fail_rtl = true;

					if (_airspeed_rtl_delay.get() == 0) {
						mavlink_log_critical(&mavlink_log_pub, "ASPD DATA %s - stopping use and returning", _airspeed_fault_type);

					} else {
						mavlink_log_critical(&mavlink_log_pub, "ASPD DATA STILL %s - returning", _airspeed_fault_type);
					}

				} else {
					status.aspd_fail_rtl = true;

					if (_airspeed_rtl_delay.get() == 0) {
						mavlink_log_critical(&mavlink_log_pub, "ASPD DATA %s - stopping use, return failed", _airspeed_fault_type);

					} else {
						mavlink_log_critical(&mavlink_log_pub, "ASPD DATA STILL %s - return failed", _airspeed_fault_type);
					}
				}

			} else if (fault_cleared) {
				mavlink_log_critical(&mavlink_log_pub, "ASPD DATA GOOD - restarting use");
			}

			// Inhibit airspeed use immediately if a bad number
			if (bad_number_fail && !status.aspd_use_inhibit) {
				status.aspd_use_inhibit = true;
			}

			return;
		}

	case 3: { // log a message, warn the user, switch to non-airspeed TECS mode
			if (fault_declared) {
				mavlink_log_critical(&mavlink_log_pub, "ASPD DATA %s  - stopping use", _airspeed_fault_type);
				status.aspd_fault_declared = true;
				status.aspd_use_inhibit = true;

			} else if (fault_cleared) {
				mavlink_log_critical(&mavlink_log_pub, "ASPD DATA GOOD - restarting use");
			}

			// Inhibit airspeed use immediately if a bad number
			if (bad_number_fail && !status.aspd_use_inhibit) {
				status.aspd_use_inhibit = true;
			}

			return;
		}

	case 2: { // log a message, warn the user
			if (fault_declared) {
				mavlink_log_critical(&mavlink_log_pub, "ASPD DATA %s", _airspeed_fault_type);
				status.aspd_fault_declared = true;

			} else if (fault_cleared) {
				mavlink_log_critical(&mavlink_log_pub, "ASPD DATA GOOD");
			}

			// Inhibit airspeed use immediately if a bad number
			if (bad_number_fail && !status.aspd_use_inhibit) {
				status.aspd_use_inhibit = true;
			}

			return;
		}

	case 1: { // log a message
			if (fault_declared) {
				mavlink_log_info(&mavlink_log_pub, "ASPD DATA %s", _airspeed_fault_type);
				status.aspd_fault_declared = true;

			} else if (fault_cleared) {
				mavlink_log_info(&mavlink_log_pub, "ASPD DATA GOOD");
			}

			// Inhibit airspeed use immediately if a bad number
			if (bad_number_fail && !status.aspd_use_inhibit) {
				status.aspd_use_inhibit = true;
			}

			return;
		}

	default:
		// Do nothing
		return;
	}
}

void Commander::estimator_check(bool *status_changed)
{
	// Check if quality checking of position accuracy and consistency is to be performed
	const bool run_quality_checks = !status_flags.circuit_breaker_engaged_posfailure_check;

	_local_position_sub.update();
	_global_position_sub.update();

	const vehicle_local_position_s &lpos = _local_position_sub.get();
	const vehicle_global_position_s &gpos = _global_position_sub.get();

	const bool mag_fault_prev = (_estimator_status_sub.get().control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT));

	if (_estimator_status_sub.update()) {
		const estimator_status_s &estimator_status = _estimator_status_sub.get();

		// Check for a magnetomer fault and notify the user
		const bool mag_fault = (estimator_status.control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT));

		if (!mag_fault_prev && mag_fault) {
			mavlink_log_critical(&mavlink_log_pub, "Stopping compass use! Check calibration on landing");
		}

		// Set the allowable position uncertainty based on combination of flight and estimator state
		// When we are in a operator demanded position control mode and are solely reliant on optical flow, do not check position error because it will gradually increase throughout flight and the operator will compensate for the drift
		const bool reliant_on_opt_flow = ((estimator_status.control_mode_flags & (1 << estimator_status_s::CS_OPT_FLOW))
						  && !(estimator_status.control_mode_flags & (1 << estimator_status_s::CS_GPS))
						  && !(estimator_status.control_mode_flags & (1 << estimator_status_s::CS_EV_POS)));

		const bool operator_controlled_position = (internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL);

		_skip_pos_accuracy_check = reliant_on_opt_flow && operator_controlled_position;

		if (_skip_pos_accuracy_check) {
			_eph_threshold_adj = INFINITY;

		} else {
			_eph_threshold_adj = _param_com_pos_fs_eph.get();
		}

		/* Check estimator status for signs of bad yaw induced post takeoff navigation failure
		 * for a short time interval after takeoff. Fixed wing vehicles can recover using GPS heading,
		 * but rotary wing vehicles cannot so the position and velocity validity needs to be latched
		 * to false after failure to prevent flyaway crashes */
		if (run_quality_checks && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			if (status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
				// reset flags and timer
				_time_at_takeoff = hrt_absolute_time();
				_nav_test_failed = false;
				_nav_test_passed = false;

			} else if (land_detector.landed) {
				// record time of takeoff
				_time_at_takeoff = hrt_absolute_time();

			} else {
				// if nav status is unconfirmed, confirm yaw angle as passed after 30 seconds or achieving 5 m/s of speed
				const bool sufficient_time = (hrt_elapsed_time(&_time_at_takeoff) > 30_s);
				const bool sufficient_speed = (lpos.vx * lpos.vx + lpos.vy * lpos.vy > 25.0f);

				bool innovation_pass = estimator_status.vel_test_ratio < 1.0f && estimator_status.pos_test_ratio < 1.0f;

				if (!_nav_test_failed) {
					if (!_nav_test_passed) {
						// pass if sufficient time or speed
						if (sufficient_time || sufficient_speed) {
							_nav_test_passed = true;
						}

						// record the last time the innovation check passed
						if (innovation_pass) {
							_time_last_innov_pass = hrt_absolute_time();
						}

						// if the innovation test has failed continuously, declare the nav as failed
						if (hrt_elapsed_time(&_time_last_innov_pass) > 1_s) {
							_nav_test_failed = true;
							mavlink_log_emergency(&mavlink_log_pub, "Critical navigation failure! Check sensor calibration");
						}
					}
				}
			}
		}
	}

	/* run global position accuracy checks */
	// Check if quality checking of position accuracy and consistency is to be performed
	if (run_quality_checks) {
		if (_nav_test_failed) {
			status_flags.condition_global_position_valid = false;
			status_flags.condition_local_position_valid = false;
			status_flags.condition_local_velocity_valid = false;

		} else {
			if (!_skip_pos_accuracy_check) {
				// use global position message to determine validity
				check_posvel_validity(true, gpos.eph, _eph_threshold_adj, gpos.timestamp, &_last_gpos_fail_time_us,
						      &_gpos_probation_time_us, &status_flags.condition_global_position_valid, status_changed);
			}

			// use local position message to determine validity
			check_posvel_validity(lpos.xy_valid, lpos.eph, _eph_threshold_adj, lpos.timestamp, &_last_lpos_fail_time_us,
					      &_lpos_probation_time_us, &status_flags.condition_local_position_valid, status_changed);
			check_posvel_validity(lpos.v_xy_valid, lpos.evh, _param_com_vel_fs_evh.get(), lpos.timestamp, &_last_lvel_fail_time_us,
					      &_lvel_probation_time_us, &status_flags.condition_local_velocity_valid, status_changed);
		}
	}

	if ((_last_condition_global_position_valid != status_flags.condition_global_position_valid)
	    && status_flags.condition_global_position_valid) {
		// If global position state changed and is now valid, set respective health flags to true. For now also assume GPS is OK if global pos is OK, but not vice versa.
		set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_AHRS, true, status);
		set_health_flags_present_healthy(subsystem_info_s::SUBSYSTEM_TYPE_GPS, true, true, status);
	}

	check_valid(lpos.timestamp, _param_com_pos_fs_delay.get() * 1_s, lpos.z_valid,
		    &(status_flags.condition_local_altitude_valid), status_changed);
}

void
Commander::offboard_control_update(bool &status_changed)
{
	const offboard_control_mode_s &offboard_control_mode = _offboard_control_mode_sub.get();

	// if this is the first time entering OFFBOARD the subscription may not be active yet
	bool force_update = false;

	if (commander_state_s::MAIN_STATE_OFFBOARD) {
		if (offboard_control_mode.timestamp == 0) {
			_offboard_control_mode_sub.subscribe();
			force_update = true;
		}
	}

	if (_offboard_control_mode_sub.updated() || force_update) {
		const offboard_control_mode_s old = offboard_control_mode;

		if (_offboard_control_mode_sub.update()) {
			if (old.ignore_thrust != offboard_control_mode.ignore_thrust ||
			    old.ignore_attitude != offboard_control_mode.ignore_attitude ||
			    old.ignore_bodyrate_x != offboard_control_mode.ignore_bodyrate_x ||
			    old.ignore_bodyrate_y != offboard_control_mode.ignore_bodyrate_y ||
			    old.ignore_bodyrate_z != offboard_control_mode.ignore_bodyrate_z ||
			    old.ignore_position != offboard_control_mode.ignore_position ||
			    old.ignore_velocity != offboard_control_mode.ignore_velocity ||
			    old.ignore_acceleration_force != offboard_control_mode.ignore_acceleration_force ||
			    old.ignore_alt_hold != offboard_control_mode.ignore_alt_hold) {

				status_changed = true;
			}
		}
	}

	if (offboard_control_mode.timestamp != 0 &&
	    offboard_control_mode.timestamp + OFFBOARD_TIMEOUT > hrt_absolute_time()) {
		if (status_flags.offboard_control_signal_lost) {
			status_flags.offboard_control_signal_lost = false;
			status_flags.offboard_control_loss_timeout = false;
			status_changed = true;
		}

	} else {
		if (!status_flags.offboard_control_signal_lost) {
			status_flags.offboard_control_signal_lost = true;
			status_changed = true;
		}

		/* check timer if offboard was there but now lost */
		if (!status_flags.offboard_control_loss_timeout && offboard_control_mode.timestamp != 0) {
			if (_param_com_of_loss_t.get() < FLT_EPSILON) {
				/* execute loss action immediately */
				status_flags.offboard_control_loss_timeout = true;

			} else {
				/* wait for timeout if set */
				status_flags.offboard_control_loss_timeout = offboard_control_mode.timestamp +
						OFFBOARD_TIMEOUT + _param_com_of_loss_t.get() * 1e6f < hrt_absolute_time();
			}

			if (status_flags.offboard_control_loss_timeout) {
				status_changed = true;
			}
		}
	}

}

void Commander::esc_status_check(const esc_status_s &esc_status)
{
	char esc_fail_msg[50];
	esc_fail_msg[0] = '\0';

	int online_bitmask = (1 << esc_status.esc_count) - 1;

	// Check if ALL the ESCs are online
	if (online_bitmask == esc_status.esc_online_flags) {
		status_flags.condition_escs_error = false;
		_last_esc_online_flags = esc_status.esc_online_flags;

	} else if (_last_esc_online_flags == esc_status.esc_online_flags || esc_status.esc_count == 0)  {

		// Avoid checking the status if the flags are the same or if the mixer has not yet been loaded in the ESC driver

		status_flags.condition_escs_error = true;

	} else if (esc_status.esc_online_flags < _last_esc_online_flags) {

		// Only warn the user when an ESC goes from ONLINE to OFFLINE. This is done to prevent showing Offline ESCs warnings at boot

		for (int index = 0; index < esc_status.esc_count; index++) {
			if ((esc_status.esc_online_flags & (1 << index)) == 0) {
				snprintf(esc_fail_msg + strlen(esc_fail_msg), sizeof(esc_fail_msg) - strlen(esc_fail_msg), "ESC%d ", index + 1);
				esc_fail_msg[sizeof(esc_fail_msg) - 1] = '\0';
			}
		}

		mavlink_log_critical(&mavlink_log_pub, "%soffline", esc_fail_msg);

		_last_esc_online_flags = esc_status.esc_online_flags;
		status_flags.condition_escs_error = true;
	}
}

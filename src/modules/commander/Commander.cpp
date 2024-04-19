/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 */

#include "Commander.hpp"

/* commander module headers */
#include "Arming/ArmAuthorization/ArmAuthorization.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#define DEFINE_GET_PX4_CUSTOM_MODE
#include "px4_custom_mode.h"
#include "ModeUtil/control_mode.hpp"
#include "ModeUtil/conversions.hpp"
#include <lib/modes/ui.hpp>
#include <lib/modes/standard_modes.hpp>

/* PX4 headers */
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>
#include <matrix/math.hpp>

#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/tune_control.h>

typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1,   /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED         = 2,   /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED         = 4,   /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED       = 8,   /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED    = 16,  /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED          = 32,  /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,  /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED         = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
	VEHICLE_MODE_FLAG_ENUM_END             = 129, /*  | */
} VEHICLE_MODE_FLAG;

// TODO: generate
static constexpr bool operator ==(const actuator_armed_s &a, const actuator_armed_s &b)
{
	return (a.armed == b.armed &&
		a.prearmed == b.prearmed &&
		a.ready_to_arm == b.ready_to_arm &&
		a.lockdown == b.lockdown &&
		a.manual_lockdown == b.manual_lockdown &&
		a.force_failsafe == b.force_failsafe &&
		a.in_esc_calibration_mode == b.in_esc_calibration_mode);
}
static_assert(sizeof(actuator_armed_s) == 16, "actuator_armed equality operator review");

#if defined(BOARD_HAS_POWER_CONTROL)
static orb_advert_t tune_control_pub = nullptr;

static void play_power_button_down_tune()
{
	// Override any other tunes because power-off sound should have the priority
	set_tune_override(tune_control_s::TUNE_ID_POWER_OFF);
}

static void stop_tune()
{
	tune_control_s tune_control{};
	tune_control.tune_override = true;
	tune_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);
}

static orb_advert_t power_button_state_pub = nullptr;
static int power_button_state_notification_cb(board_power_button_state_notification_e request)
{
	// Note: this can be called from IRQ handlers, so we publish a message that will be handled
	// on the main thread of commander.
	power_button_state_s button_state{};
	button_state.timestamp = hrt_absolute_time();
	const int ret = PWR_BUTTON_RESPONSE_SHUT_DOWN_PENDING;

	switch (request) {
	case PWR_BUTTON_IDEL:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_IDEL;
		break;

	case PWR_BUTTON_DOWN:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_DOWN;
		play_power_button_down_tune();
		break;

	case PWR_BUTTON_UP:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_UP;
		stop_tune();
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
#endif // BOARD_HAS_POWER_CONTROL

#ifndef CONSTRAINED_FLASH
static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}

static bool wait_for_vehicle_command_reply(const uint32_t cmd,
		uORB::SubscriptionData<vehicle_command_ack_s> &vehicle_command_ack_sub)
{
	hrt_abstime start = hrt_absolute_time();

	while (hrt_absolute_time() - start < 100_ms) {
		if (vehicle_command_ack_sub.update()) {
			if (vehicle_command_ack_sub.get().command == cmd) {
				return vehicle_command_ack_sub.get().result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}

		px4_usleep(10000);
	}

	return false;
}

static bool broadcast_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				      const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				      const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = 0;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = 0;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}
#endif

int Commander::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(argv[0], "calibrate")) {
		if (argc > 1) {
			if (!strcmp(argv[1], "gyro")) {
				// gyro calibration: param1 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 1.f, 0.f, 0.f, 0.f, 0.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "mag")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// magnetometer quick calibration: VEHICLE_CMD_FIXED_MAG_CAL_YAW
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW);

				} else {
					// magnetometer calibration: param2 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 1.f, 0.f, 0.f, 0.0, 0.0, 0.f);
				}

			} else if (!strcmp(argv[1], "baro")) {
				// baro calibration: param3 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 1.f, 0.f, 0.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "accel")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// accelerometer quick calibration: param5 = 3
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 4.0, 0.0, 0.f);

				} else {
					// accelerometer calibration: param5 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 1.0, 0.0, 0.f);
				}

			} else if (!strcmp(argv[1], "level")) {
				// board level calibration: param5 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 2.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "airspeed")) {
				// airspeed calibration: param6 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 2.0, 0.f);

			} else if (!strcmp(argv[1], "esc")) {
				// ESC calibration: param7 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 0.0, 1.f);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
				return 1;
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[0], "check")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_RUN_PREARM_CHECKS);

		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
		PX4_INFO("Preflight check: %s", vehicle_status_sub.get().pre_flight_checks_pass ? "OK" : "FAILED");

		return 0;
	}

	if (!strcmp(argv[0], "arm")) {
		float param2 = 0.f;

		// 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
		if (argc > 1 && !strcmp(argv[1], "-f")) {
			param2 = 21196.f;
		}

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
				     static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
				     param2);

		return 0;
	}

	if (!strcmp(argv[0], "disarm")) {
		float param2 = 0.f;

		// 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
		if (argc > 1 && !strcmp(argv[1], "-f")) {
			param2 = 21196.f;
		}

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
				     static_cast<float>(vehicle_command_s::ARMING_ACTION_DISARM),
				     param2);

		return 0;
	}

	if (!strcmp(argv[0], "takeoff")) {
		// switch to takeoff mode and arm
		uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);

		if (wait_for_vehicle_command_reply(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF, vehicle_command_ack_sub)) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
					     static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
					     0.f);
		}

		return 0;
	}

	if (!strcmp(argv[0], "land")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);

		return 0;
	}

	if (!strcmp(argv[0], "transition")) {
		uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
		vehicle_status_s vehicle_status{};
		vehicle_status_sub.copy(&vehicle_status);
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
				     (float)(vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ?
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW :
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC), 0.0f);

		return 0;
	}

	if (!strcmp(argv[0], "mode")) {
		if (argc > 1) {

			if (!strcmp(argv[1], "manual")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_MANUAL);

			} else if (!strcmp(argv[1], "altctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ALTCTL);

			} else if (!strcmp(argv[1], "posctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);

			} else if (!strcmp(argv[1], "position:slow")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL,
						     PX4_CUSTOM_SUB_MODE_POSCTL_SLOW);

			} else if (!strcmp(argv[1], "auto:mission")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_MISSION);

			} else if (!strcmp(argv[1], "auto:loiter")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);

			} else if (!strcmp(argv[1], "auto:rtl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_RTL);

			} else if (!strcmp(argv[1], "acro")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ACRO);

			} else if (!strcmp(argv[1], "offboard")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);

			} else if (!strcmp(argv[1], "stabilized")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_STABILIZED);

			} else if (!strcmp(argv[1], "auto:takeoff")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF);

			} else if (!strcmp(argv[1], "auto:land")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LAND);

			} else if (!strcmp(argv[1], "auto:precland")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND);

			} else if (!strcmp(argv[1], "ext1")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_EXTERNAL1);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[0], "lockdown")) {

		if (argc < 2) {
			Commander::print_usage("not enough arguments, missing [on, off]");
			return 1;
		}

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION,
						strcmp(argv[1], "off") ? 2.0f : 0.0f /* lockdown */, 0.0f);

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[0], "pair")) {

		// GCS pairing request handled by a companion
		bool ret = broadcast_vehicle_command(vehicle_command_s::VEHICLE_CMD_START_RX_PAIR, 10.f);

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[0], "set_ekf_origin")) {
		if (argc > 3) {

			double latitude  = atof(argv[1]);
			double longitude = atof(argv[2]);
			float  altitude  = atof(argv[3]);

			// Set the ekf NED origin global coordinates.
			bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
							0.f, 0.f, 0.0, 0.0, latitude, longitude, altitude);
			return (ret ? 0 : 1);

		} else {
			PX4_ERR("missing argument");
			return 0;
		}
	}

	if (!strcmp(argv[0], "poweroff")) {

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
						2.0f);

		return (ret ? 0 : 1);
	}


#endif

	return print_usage("unknown command");
}

int Commander::print_status()
{
	PX4_INFO("%s", isArmed() ? "Armed" : "Disarmed");
	PX4_INFO("navigation mode: %s", mode_util::nav_state_names[_vehicle_status.nav_state]);
	PX4_INFO("user intended navigation mode: %s", mode_util::nav_state_names[_vehicle_status.nav_state_user_intention]);
	PX4_INFO("in failsafe: %s", _failsafe.inFailsafe() ? "yes" : "no");
	_mode_management.printStatus();
	perf_print_counter(_loop_perf);
	perf_print_counter(_preflight_check_perf);
	return 0;
}

extern "C" __EXPORT int commander_main(int argc, char *argv[])
{
	return Commander::main(argc, argv);
}

static constexpr const char *arm_disarm_reason_str(arm_disarm_reason_t calling_reason)
{
	switch (calling_reason) {
	case arm_disarm_reason_t::transition_to_standby: return "";

	case arm_disarm_reason_t::rc_stick: return "RC";

	case arm_disarm_reason_t::rc_switch: return "RC (switch)";

	case arm_disarm_reason_t::command_internal: return "internal command";

	case arm_disarm_reason_t::command_external: return "external command";

	case arm_disarm_reason_t::mission_start: return "mission start";

	case arm_disarm_reason_t::auto_disarm_land: return "landing";

	case arm_disarm_reason_t::auto_disarm_preflight: return "auto preflight disarming";

	case arm_disarm_reason_t::kill_switch: return "kill-switch";

	case arm_disarm_reason_t::lockdown: return "lockdown";

	case arm_disarm_reason_t::failure_detector: return "failure detector";

	case arm_disarm_reason_t::shutdown: return "shutdown request";

	case arm_disarm_reason_t::unit_test: return "unit tests";

	case arm_disarm_reason_t::rc_button: return "RC (button)";

	case arm_disarm_reason_t::failsafe: return "failsafe";
	}

	return "";
};

transition_result_t Commander::arm(arm_disarm_reason_t calling_reason, bool run_preflight_checks)
{
	if (isArmed()) {
		return TRANSITION_NOT_CHANGED;
	}

	if (_vehicle_status.calibration_enabled
	    || _vehicle_status.rc_calibration_in_progress
	    || _actuator_armed.in_esc_calibration_mode) {

		mavlink_log_critical(&_mavlink_log_pub, "Arming denied: calibrating\t");
		events::send(events::ID("commander_arm_denied_calibrating"), {events::Log::Critical, events::LogInternal::Info},
			     "Arming denied: calibrating");
		tune_negative(true);
		return TRANSITION_DENIED;
	}

	// allow a grace period for re-arming: preflight checks don't need to pass during that time, for example for accidental in-air disarming
	if (calling_reason == arm_disarm_reason_t::rc_switch
	    && ((_last_disarmed_timestamp != 0) && (hrt_elapsed_time(&_last_disarmed_timestamp) < 5_s))) {

		run_preflight_checks = false;
	}

	if (run_preflight_checks) {
		if (_vehicle_control_mode.flag_control_manual_enabled) {

			if (_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_failsafe_flags.manual_control_signal_lost && _is_throttle_above_center) {

				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: throttle above center\t");
				events::send(events::ID("commander_arm_denied_throttle_center"), {events::Log::Critical, events::LogInternal::Info},
					     "Arming denied: throttle above center");
				tune_negative(true);
				return TRANSITION_DENIED;
			}

			if (!_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_failsafe_flags.manual_control_signal_lost && !_is_throttle_low
			    && _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROVER) {

				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: high throttle\t");
				events::send(events::ID("commander_arm_denied_throttle_high"), {events::Log::Critical, events::LogInternal::Info},
					     "Arming denied: high throttle");
				tune_negative(true);
				return TRANSITION_DENIED;
			}

		} else if (calling_reason == arm_disarm_reason_t::rc_stick
			   || calling_reason == arm_disarm_reason_t::rc_switch
			   || calling_reason == arm_disarm_reason_t::rc_button) {

			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: switch to manual mode first\t");
			events::send(events::ID("commander_arm_denied_not_manual"), {events::Log::Critical, events::LogInternal::Info},
				     "Arming denied: switch to manual mode first");
			tune_negative(true);
			return TRANSITION_DENIED;
		}

		_health_and_arming_checks.update();

		if (!_health_and_arming_checks.canArm(_vehicle_status.nav_state)) {
			tune_negative(true);
			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: Resolve system health failures first\t");
			events::send(events::ID("commander_arm_denied_resolve_failures"), {events::Log::Critical, events::LogInternal::Info},
				     "Arming denied: Resolve system health failures first");
			return TRANSITION_DENIED;
		}
	}

	_vehicle_status.armed_time = hrt_absolute_time();
	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	_vehicle_status.latest_arming_reason = (uint8_t)calling_reason;

	mavlink_log_info(&_mavlink_log_pub, "Armed by %s\t", arm_disarm_reason_str(calling_reason));
	events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_armed_by"), events::Log::Info,
			"Armed by {1}", calling_reason);

	if (_param_com_home_en.get()) {
		_home_position.setHomePosition();
	}

	_status_changed = true;

	return TRANSITION_CHANGED;
}

transition_result_t Commander::disarm(arm_disarm_reason_t calling_reason, bool forced)
{
	if (!isArmed()) {
		return TRANSITION_NOT_CHANGED;
	}

	if (!forced) {
		const bool landed = (_vehicle_land_detected.landed || _vehicle_land_detected.maybe_landed
				     || is_ground_vehicle(_vehicle_status));
		const bool mc_manual_thrust_mode = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
						   && _vehicle_control_mode.flag_control_manual_enabled
						   && !_vehicle_control_mode.flag_control_climb_rate_enabled;
		const bool commanded_by_rc = (calling_reason == arm_disarm_reason_t::rc_stick)
					     || (calling_reason == arm_disarm_reason_t::rc_switch)
					     || (calling_reason == arm_disarm_reason_t::rc_button);

		if (!landed && !(mc_manual_thrust_mode && commanded_by_rc && _param_com_disarm_man.get())) {
			if (calling_reason != arm_disarm_reason_t::rc_stick) {
				mavlink_log_critical(&_mavlink_log_pub, "Disarming denied: not landed\t");
				events::send(events::ID("commander_disarm_denied_not_landed"),
				{events::Log::Critical, events::LogInternal::Info},
				"Disarming denied: not landed");
			}

			return TRANSITION_DENIED;
		}
	}

	_vehicle_status.armed_time = 0;
	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle_status.latest_disarming_reason = (uint8_t)calling_reason;
	_vehicle_status.takeoff_time = 0;

	_have_taken_off_since_arming = false;

	_last_disarmed_timestamp = hrt_absolute_time();

	_user_mode_intention.onDisarm();

	mavlink_log_info(&_mavlink_log_pub, "Disarmed by %s\t", arm_disarm_reason_str(calling_reason));
	events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_disarmed_by"), events::Log::Info,
			"Disarmed by {1}", calling_reason);

	if (_param_com_force_safety.get()) {
		_safety.activateSafety();
	}

	// update flight uuid
	const int32_t flight_uuid = _param_flight_uuid.get() + 1;
	_param_flight_uuid.set(flight_uuid);
	_param_flight_uuid.commit_no_notification();

	_status_changed = true;

	return TRANSITION_CHANGED;
}

Commander::Commander() :
	ModuleParams(nullptr)
{
	_vehicle_land_detected.landed = true;

	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle_status.system_id = 1;
	_vehicle_status.component_id = 1;
	_vehicle_status.system_type = 0;
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;
	_vehicle_status.nav_state = _user_mode_intention.get();
	_vehicle_status.nav_state_user_intention = _user_mode_intention.get();
	_vehicle_status.nav_state_timestamp = hrt_absolute_time();
	_vehicle_status.gcs_connection_lost = true;
	_vehicle_status.power_input_valid = true;

	// default for vtol is rotary wing
	_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

	_param_mav_comp_id = param_find("MAV_COMP_ID");
	_param_mav_sys_id = param_find("MAV_SYS_ID");
	_param_mav_type = param_find("MAV_TYPE");
	_param_rc_map_fltmode = param_find("RC_MAP_FLTMODE");

	updateParameters();
}

Commander::~Commander()
{
	perf_free(_loop_perf);
	perf_free(_preflight_check_perf);
}

bool
Commander::handle_command(const vehicle_command_s &cmd)
{
	/* only handle commands that are meant to be handled by this system and component, or broadcast */
	if (((cmd.target_system != _vehicle_status.system_id) && (cmd.target_system != 0))
	    || ((cmd.target_component != _vehicle_status.component_id) && (cmd.target_component != 0))) {
		return false;
	}

	/* result of the command */
	unsigned cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd.command) {
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION: {

			// Just switch the flight mode here, the navigator takes care of
			// doing something sensible with the coordinates. Its designed
			// to not require navigator and command to receive / process
			// the data at the exact same time.

			const uint32_t change_mode_flags = uint32_t(cmd.param2);
			const bool mode_switch_not_requested = (change_mode_flags & 1) == 0;
			const bool unsupported_bits_set = (change_mode_flags & ~1) != 0;

			if (mode_switch_not_requested || unsupported_bits_set) {
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);

			} else {
				if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, getSourceFromCommand(cmd))) {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE: {

			// Just switch the flight mode here, the navigator takes care of
			// doing something sensible with the coordinates. Its designed
			// to not require navigator and command to receive / process
			// the data at the exact same time.

			if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t)cmd.param1;
			uint8_t custom_main_mode = (uint8_t)cmd.param2;
			uint8_t custom_sub_mode = (uint8_t)cmd.param3;

			uint8_t desired_nav_state = vehicle_status_s::NAVIGATION_STATE_MAX;
			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					switch (custom_sub_mode) {
					default:
					case PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL:
						desired_nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
						break;

					case PX4_CUSTOM_SUB_MODE_POSCTL_SLOW:
						desired_nav_state = vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW;
						break;
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					if (custom_sub_mode > 0) {

						switch (custom_sub_mode) {
						case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND;
							break;

						case PX4_CUSTOM_SUB_MODE_EXTERNAL1...PX4_CUSTOM_SUB_MODE_EXTERNAL8:
							desired_nav_state = vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + (custom_sub_mode - PX4_CUSTOM_SUB_MODE_EXTERNAL1);
							break;

						default:
							main_ret = TRANSITION_DENIED;
							mavlink_log_critical(&_mavlink_log_pub, "Unsupported auto mode\t");
							events::send(events::ID("commander_unsupported_auto_mode"), events::Log::Error,
								     "Unsupported auto mode");
							break;
						}

					} else {
						desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_ACRO;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
				}

			} else {
				/* use base mode */
				if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

				} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
						desired_nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

					} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
						desired_nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;

					} else {
						desired_nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
					}
				}
			}

			if (desired_nav_state != vehicle_status_s::NAVIGATION_STATE_MAX) {
				if (_user_mode_intention.change(desired_nav_state, getSourceFromCommand(cmd))) {
					main_ret = TRANSITION_CHANGED;

				} else {
					if (cmd.from_external && cmd.source_component == 190) { // MAV_COMP_ID_MISSIONPLANNER
						printRejectMode(desired_nav_state);
					}

					main_ret = TRANSITION_DENIED;
				}
			}

			if (main_ret != TRANSITION_DENIED) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE: { // Used from ROS
			uint8_t desired_nav_state = (uint8_t)(cmd.param1 + 0.5f);

			if (_user_mode_intention.change(desired_nav_state, getSourceFromCommand(cmd))) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {

			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters
			const int8_t arming_action = static_cast<int8_t>(lroundf(cmd.param1));

			if (arming_action != vehicle_command_s::ARMING_ACTION_ARM
			    && arming_action != vehicle_command_s::ARMING_ACTION_DISARM) {
				mavlink_log_critical(&_mavlink_log_pub, "Unsupported ARM_DISARM param: %.3f\t", (double)cmd.param1);
				events::send<float>(events::ID("commander_unsupported_arm_disarm_param"), events::Log::Error,
						    "Unsupported ARM_DISARM param: {1:.3}", cmd.param1);

			} else {
				// Arm is forced (checks skipped) when param2 is set to a magic number.
				const bool forced = (static_cast<int>(lroundf(cmd.param2)) == 21196);

				transition_result_t arming_res = TRANSITION_DENIED;
				arm_disarm_reason_t arm_disarm_reason = cmd.from_external ? arm_disarm_reason_t::command_external :
									arm_disarm_reason_t::command_internal;

				if (arming_action == vehicle_command_s::ARMING_ACTION_ARM) {
					arming_res = arm(arm_disarm_reason, cmd.from_external || !forced);

				} else if (arming_action == vehicle_command_s::ARMING_ACTION_DISARM) {
					arming_res = disarm(arm_disarm_reason, forced);

				}

				if (arming_res == TRANSITION_DENIED) {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				} else {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION: {
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;

			if (cmd.param1 > 0.5f) {
				// Trigger real termination.
				if (!isArmed()) {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;

				} else if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_TERMINATION)) {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME: {
			if (_param_com_home_en.get()) {
				bool use_current = cmd.param1 > 0.5f;

				if (use_current) {
					/* use current position */
					if (_home_position.setHomePosition(true)) {
						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					}

				} else {
					float yaw = matrix::wrap_2pi(math::radians(cmd.param4));
					yaw = PX4_ISFINITE(yaw) ? yaw : (float)NAN;
					const double lat = cmd.param5;
					const double lon = cmd.param6;
					const float alt = cmd.param7;

					if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt)) {

						if (_home_position.setManually(lat, lon, alt, yaw)) {

							cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

						} else {
							cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
						}

					} else {
						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
					}
				}

			} else {
				// COM_HOME_EN disabled
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: {
			/* switch to RTL which ends the mission */
			if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, getSourceFromCommand(cmd))) {
				mavlink_log_info(&_mavlink_log_pub, "Returning to launch\t");
				events::send(events::ID("commander_rtl"), events::Log::Info, "Returning to launch");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
			/* ok, home set, use it to take off */
			if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, getSourceFromCommand(cmd))) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF);
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

		/* ok, home set, use it to take off */
		if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF, getSourceFromCommand(cmd))) {
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else {
			printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF);
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
		}

#else
		cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
#endif // CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND: {
			if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, getSourceFromCommand(cmd))) {
				mavlink_log_info(&_mavlink_log_pub, "Landing at current position\t");
				events::send(events::ID("commander_landing_current_pos"), events::Log::Info,
					     "Landing at current position");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND);
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND: {
			if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND, getSourceFromCommand(cmd))) {
				mavlink_log_info(&_mavlink_log_pub, "Precision landing\t");
				events::send(events::ID("commander_landing_prec_land"), events::Log::Info,
					     "Landing using precision landing");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND);
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {

			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;

			// check if current mission and first item are valid
			if (!_failsafe_flags.auto_mission_missing) {

				// requested first mission item valid
				if (PX4_ISFINITE(cmd.param1) && (cmd.param1 >= -1) && (cmd.param1 < _mission_result_sub.get().seq_total)) {

					// switch to AUTO_MISSION and ARM
					if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, getSourceFromCommand(cmd))
					    && (TRANSITION_DENIED != arm(arm_disarm_reason_t::mission_start))) {

						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					}
				}

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Mission start denied! No valid mission\t");
				events::send(events::ID("commander_mission_start_denied_no_mission"), {events::Log::Critical, events::LogInternal::Info},
					     "Mission start denied! No valid mission");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY: {
			// if no high latency telemetry exists send a failed acknowledge
			if (_high_latency_datalink_timestamp < _boot_timestamp) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
				mavlink_log_critical(&_mavlink_log_pub, "Control high latency failed! Telemetry unavailable\t");
				events::send(events::ID("commander_ctrl_high_latency_failed"), {events::Log::Critical, events::LogInternal::Info},
					     "Control high latency failed! Telemetry unavailable");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT: {

			transition_result_t main_ret;

			if (_vehicle_status.in_transition_mode) {
				main_ret = TRANSITION_DENIED;

			} else if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				// for fixed wings the behavior of orbit is the same as loiter
				if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, getSourceFromCommand(cmd))) {
					main_ret = TRANSITION_CHANGED;

				} else {
					main_ret = TRANSITION_DENIED;
				}

			} else {
				// Switch to orbit state and let the orbit task handle the command further
				if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_ORBIT, getSourceFromCommand(cmd))) {
					main_ret = TRANSITION_CHANGED;

				} else {
					main_ret = TRANSITION_DENIED;
				}
			}

			if (main_ret != TRANSITION_DENIED) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				mavlink_log_critical(&_mavlink_log_pub, "Orbit command rejected");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT: {
#ifdef CONFIG_FIGURE_OF_EIGHT

			if (!((_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) || (_vehicle_status.is_vtol))) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
				mavlink_log_critical(&_mavlink_log_pub, "Figure 8 command only available for fixed wing and vtol vehicles.");
				break;
			}

			transition_result_t main_ret = TRANSITION_DENIED;

			if ((_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) &&
			    (!_vehicle_status.in_transition_mode)) {
				if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)) {
					main_ret = TRANSITION_CHANGED;

				} else {
					main_ret = TRANSITION_DENIED;
				}
			}

			if (main_ret != TRANSITION_DENIED) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				mavlink_log_critical(&_mavlink_log_pub, "Figure 8 command rejected, Only available in fixed wing mode.");
			}

#else
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			mavlink_log_critical(&_mavlink_log_pub, "Figure 8 command not supported.");
#endif // CONFIG_FIGURE_OF_EIGHT
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
		cmd_result = handleCommandActuatorTest(cmd);
		break;

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {

			const int param1 = cmd.param1;

			if (param1 == 0) {
				// 0: Do nothing for autopilot
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 1) && !isArmed() && (px4_reboot_request(REBOOT_REQUEST, 400_ms) == 0)) {
				// 1: Reboot autopilot
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_RESET

#if defined(BOARD_HAS_POWER_CONTROL)

			} else if ((param1 == 2) && !isArmed() && (px4_shutdown_request(400_ms) == 0)) {
				// 2: Shutdown autopilot
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // BOARD_HAS_POWER_CONTROL

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 3) && !isArmed() && (px4_reboot_request(REBOOT_TO_BOOTLOADER, 400_ms) == 0)) {
				// 3: Reboot autopilot and keep it in the bootloader until upgraded.
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_RESET

			} else {
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
			}
		}

		break;

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

			if (isArmed() || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {

				if ((int)(cmd.param1) == 1) {
					/* gyro calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::GyroCalibration);

				} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					/* temperature calibration: handled in events module */
					break;

				} else if ((int)(cmd.param2) == 1) {
					/* magnetometer calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::MagCalibration);

				} else if ((int)(cmd.param3) == 1) {
					/* baro calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::BaroCalibration);

				} else if ((int)(cmd.param4) == 1) {
					/* RC calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					/* disable RC control input completely */
					_vehicle_status.rc_calibration_in_progress = true;
					mavlink_log_info(&_mavlink_log_pub, "Calibration: Disabling RC input\t");
					events::send(events::ID("commander_calib_rc_off"), events::Log::Info,
						     "Calibration: Disabling RC input");

				} else if ((int)(cmd.param4) == 2) {
					/* RC trim calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::RCTrimCalibration);

				} else if ((int)(cmd.param5) == 1) {
					/* accelerometer calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibration);

				} else if ((int)(cmd.param5) == 2) {
					// board offset calibration
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::LevelCalibration);

				} else if ((int)(cmd.param5) == 4) {
					// accelerometer quick calibration
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibrationQuick);

				} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
					// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
					/* airspeed calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AirspeedCalibration);

				} else if ((int)(cmd.param7) == 1) {
					/* do esc calibration */
					if (check_battery_disconnected(&_mavlink_log_pub)) {
						answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

						if (_safety.isButtonAvailable() && !_safety.isSafetyOff()) {
							mavlink_log_critical(&_mavlink_log_pub, "ESC calibration denied! Press safety button first\t");
							events::send(events::ID("commander_esc_calibration_denied"), events::Log::Critical,
								     "ESCs calibration denied");

						} else {
							_vehicle_status.calibration_enabled = true;
							_actuator_armed.in_esc_calibration_mode = true;
							_worker_thread.startTask(WorkerThread::Request::ESCCalibration);
						}

					} else {
						answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
					}

				} else if ((int)(cmd.param4) == 0) {
					/* RC calibration ended - have we been in one worth confirming? */
					if (_vehicle_status.rc_calibration_in_progress) {
						/* enable RC control input */
						_vehicle_status.rc_calibration_in_progress = false;
						mavlink_log_info(&_mavlink_log_pub, "Calibration: Restoring RC input\t");
						events::send(events::ID("commander_calib_rc_on"), events::Log::Info,
							     "Calibration: Restoring RC input");
					}

					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW: {
			// Magnetometer quick calibration using world magnetic model and known heading
			if (isArmed() || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
				// parameter 1: Heading   (degrees)
				// parameter 3: Latitude  (degrees)
				// parameter 4: Longitude (degrees)

				// assume vehicle pointing north (0 degrees) if heading isn't specified
				const float heading_radians = PX4_ISFINITE(cmd.param1) ? math::radians(roundf(cmd.param1)) : 0.f;

				float latitude = NAN;
				float longitude = NAN;

				if (PX4_ISFINITE(cmd.param3) && PX4_ISFINITE(cmd.param4)) {
					// invalid if both lat & lon are 0 (current mavlink spec)
					if ((fabsf(cmd.param3) > 0) && (fabsf(cmd.param4) > 0)) {
						latitude = cmd.param3;
						longitude = cmd.param4;
					}
				}

				_vehicle_status.calibration_enabled = true;
				_worker_thread.setMagQuickData(heading_radians, latitude, longitude);
				_worker_thread.startTask(WorkerThread::Request::MagCalibrationQuick);
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

			if (isArmed() || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {

				if (((int)(cmd.param1)) == 0) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamLoadDefault);

				} else if (((int)(cmd.param1)) == 1) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamSaveDefault);

				} else if (((int)(cmd.param1)) == 2) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamResetAllConfig);

				} else if (((int)(cmd.param1)) == 3) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamResetSensorFactory);

				} else if (((int)(cmd.param1)) == 4) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamResetAll);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_DO_SET_STANDARD_MODE: {
			mode_util::StandardMode standard_mode = (mode_util::StandardMode) roundf(cmd.param1);
			uint8_t nav_state = mode_util::getNavStateFromStandardMode(standard_mode);

			if (nav_state == vehicle_status_s::NAVIGATION_STATE_MAX) {
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED);

			} else {
				if (_user_mode_intention.change(nav_state, getSourceFromCommand(cmd))) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_RUN_PREARM_CHECKS:
		_health_and_arming_checks.update(true);
		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR:
		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_2:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_UAVCAN:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST:
	case vehicle_command_s::VEHICLE_CMD_OBLIQUE_SURVEY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_ZOOM:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_FOCUS:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED:
	case vehicle_command_s::VEHICLE_CMD_DO_LAND_START:
	case vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_START:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_STOP:
	case vehicle_command_s::VEHICLE_CMD_NAV_DELAY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
	case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
	case vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE:
	case vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN:
	case vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
	case vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR:
	case vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE:
	case vehicle_command_s::VEHICLE_CMD_DO_WINCH:
	case vehicle_command_s::VEHICLE_CMD_DO_GRIPPER:
		/* ignore commands that are handled by other parts of the system */
		break;

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
		break;
	}

	if (cmd_result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(cmd, cmd_result);
	}

	return true;
}

ModeChangeSource Commander::getSourceFromCommand(const vehicle_command_s &cmd)
{
	return cmd.source_component >= vehicle_command_s::COMPONENT_MODE_EXECUTOR_START ? ModeChangeSource::ModeExecutor :
	       ModeChangeSource::User;
}

void Commander::handleCommandsFromModeExecutors()
{
	if (_vehicle_command_mode_executor_sub.updated()) {
		const unsigned last_generation = _vehicle_command_mode_executor_sub.get_last_generation();
		vehicle_command_s cmd;

		if (_vehicle_command_mode_executor_sub.copy(&cmd)) {
			if (_vehicle_command_mode_executor_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("vehicle_command from executor lost, generation %u -> %u", last_generation,
					_vehicle_command_mode_executor_sub.get_last_generation());
			}

			// For commands from mode executors, we check if it is in charge and then publish it on the official
			// command topic
			const int mode_executor_in_charge = _mode_management.modeExecutorInCharge();

			// source_system is set to the mode executor
			if (cmd.source_component == vehicle_command_s::COMPONENT_MODE_EXECUTOR_START + mode_executor_in_charge) {
				cmd.source_system = _vehicle_status.system_id;
				cmd.timestamp = hrt_absolute_time();
				_vehicle_command_pub.publish(cmd);

			} else {
				cmd.source_system = _vehicle_status.system_id;
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
				PX4_WARN("Got cmd from executor %i not in charge (in charge: %i)", cmd.source_system, mode_executor_in_charge);
			}
		}
	}
}

unsigned Commander::handleCommandActuatorTest(const vehicle_command_s &cmd)
{
	if (isArmed() || (_safety.isButtonAvailable() && !_safety.isSafetyOff())) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	}

	if (_param_com_mot_test_en.get() != 1) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	}

	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = (int)(cmd.param5 + 0.5);

	if (actuator_test.function < 1000) {
		const int first_motor_function = 1; // from MAVLink ACTUATOR_OUTPUT_FUNCTION
		const int first_servo_function = 33;

		if (actuator_test.function >= first_motor_function
		    && actuator_test.function < first_motor_function + actuator_test_s::MAX_NUM_MOTORS) {
			actuator_test.function = actuator_test.function - first_motor_function + actuator_test_s::FUNCTION_MOTOR1;

		} else if (actuator_test.function >= first_servo_function
			   && actuator_test.function < first_servo_function + actuator_test_s::MAX_NUM_SERVOS) {
			actuator_test.function = actuator_test.function - first_servo_function + actuator_test_s::FUNCTION_SERVO1;

		} else {
			return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		}

	} else {
		actuator_test.function -= 1000;
	}

	actuator_test.value = cmd.param1;

	actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
	int timeout_ms = (int)(cmd.param2 * 1000.f + 0.5f);

	if (timeout_ms <= 0) {
		actuator_test.action = actuator_test_s::ACTION_RELEASE_CONTROL;

	} else {
		actuator_test.timeout_ms = timeout_ms;
	}

	// enforce a timeout and a maximum limit
	if (actuator_test.timeout_ms == 0 || actuator_test.timeout_ms > 3000) {
		actuator_test.timeout_ms = 3000;
	}

	_actuator_test_pub.publish(actuator_test);
	return vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
}

void Commander::executeActionRequest(const action_request_s &action_request)
{
	arm_disarm_reason_t arm_disarm_reason{};

	// Silently ignore RC actions during RC calibration
	if (_vehicle_status.rc_calibration_in_progress
	    && (action_request.source == action_request_s::SOURCE_RC_STICK_GESTURE
		|| action_request.source == action_request_s::SOURCE_RC_SWITCH
		|| action_request.source == action_request_s::SOURCE_RC_BUTTON
		|| action_request.source == action_request_s::SOURCE_RC_MODE_SLOT)) {
		return;
	}

	switch (action_request.source) {
	case action_request_s::SOURCE_RC_STICK_GESTURE: arm_disarm_reason = arm_disarm_reason_t::rc_stick; break;

	case action_request_s::SOURCE_RC_SWITCH: arm_disarm_reason = arm_disarm_reason_t::rc_switch; break;

	case action_request_s::SOURCE_RC_BUTTON: arm_disarm_reason = arm_disarm_reason_t::rc_button; break;
	}

	switch (action_request.action) {
	case action_request_s::ACTION_DISARM: disarm(arm_disarm_reason); break;

	case action_request_s::ACTION_ARM: arm(arm_disarm_reason); break;

	case action_request_s::ACTION_TOGGLE_ARMING:
		if (isArmed()) {
			disarm(arm_disarm_reason);

		} else {
			arm(arm_disarm_reason);
		}

		break;

	case action_request_s::ACTION_UNKILL:
		if (_actuator_armed.manual_lockdown) {
			mavlink_log_info(&_mavlink_log_pub, "Kill disengaged\t");
			events::send(events::ID("commander_kill_sw_disengaged"), events::Log::Info, "Kill disengaged");
			_status_changed = true;
			_actuator_armed.manual_lockdown = false;
		}

		break;

	case action_request_s::ACTION_KILL:
		if (!_actuator_armed.manual_lockdown) {
			const char kill_switch_string[] = "Kill engaged\t";
			events::LogLevels log_levels{events::Log::Info};

			if (_vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, kill_switch_string);

			} else {
				mavlink_log_critical(&_mavlink_log_pub, kill_switch_string);
				log_levels.external = events::Log::Critical;
			}

			events::send(events::ID("commander_kill_sw_engaged"), log_levels, "Kill engaged");

			_status_changed = true;
			_actuator_armed.manual_lockdown = true;
		}

		break;

	case action_request_s::ACTION_SWITCH_MODE:

		if (!_user_mode_intention.change(action_request.mode, ModeChangeSource::User, true)) {
			printRejectMode(action_request.mode);
		}

		break;
	}
}


void Commander::updateParameters()
{
	// update parameters from storage
	updateParams();

	int32_t value_int32 = 0;

	// MAV_SYS_ID => vehicle_status.system_id
	if ((_param_mav_sys_id != PARAM_INVALID) && (param_get(_param_mav_sys_id, &value_int32) == PX4_OK)) {
		_vehicle_status.system_id = value_int32;
	}

	// MAV_COMP_ID => vehicle_status.component_id
	if ((_param_mav_comp_id != PARAM_INVALID) && (param_get(_param_mav_comp_id, &value_int32) == PX4_OK)) {
		_vehicle_status.component_id = value_int32;
	}

	// MAV_TYPE -> vehicle_status.system_type
	if ((_param_mav_type != PARAM_INVALID) && (param_get(_param_mav_type, &value_int32) == PX4_OK)) {
		_vehicle_status.system_type = value_int32;
	}


	_vehicle_status.avoidance_system_required = _param_com_obs_avoid.get();

	_auto_disarm_killed.set_hysteresis_time_from(false, _param_com_kill_disarm.get() * 1_s);

	const bool is_rotary = is_rotary_wing(_vehicle_status) || (is_vtol(_vehicle_status)
			       && _vtol_vehicle_status.vehicle_vtol_state != vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	const bool is_fixed = is_fixed_wing(_vehicle_status) || (is_vtol(_vehicle_status)
			      && _vtol_vehicle_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	const bool is_ground = is_ground_vehicle(_vehicle_status);

	/* disable manual override for all systems that rely on electronic stabilization */
	if (is_rotary) {
		_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	} else if (is_fixed) {
		_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	} else if (is_ground) {
		_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
	}

	_vehicle_status.is_vtol = is_vtol(_vehicle_status);
	_vehicle_status.is_vtol_tailsitter = is_vtol_tailsitter(_vehicle_status);

	// _mode_switch_mapped = (RC_MAP_FLTMODE > 0)
	if (_param_rc_map_fltmode != PARAM_INVALID && (param_get(_param_rc_map_fltmode, &value_int32) == PX4_OK)) {
		_mode_switch_mapped = (value_int32 > 0);
	}

}

void Commander::run()
{
	/* initialize */
	led_init();
	buzzer_init();

#if defined(BOARD_HAS_POWER_CONTROL)
	{
		// we need to do an initial publication to make sure uORB allocates the buffer, which cannot happen
		// in IRQ context.
		power_button_state_s button_state{};
		button_state.timestamp = hrt_absolute_time();
		button_state.event = 0xff;
		power_button_state_pub = orb_advertise(ORB_ID(power_button_state), &button_state);

		_power_button_state_sub.copy(&button_state);

		tune_control_s tune_control{};
		button_state.timestamp = hrt_absolute_time();
		tune_control_pub = orb_advertise(ORB_ID(tune_control), &tune_control);
	}

	if (board_register_power_state_notification_cb(power_button_state_notification_cb) != 0) {
		PX4_ERR("Failed to register power notification callback");
	}

#endif // BOARD_HAS_POWER_CONTROL

	_boot_timestamp = hrt_absolute_time();

	arm_auth_init(&_mavlink_log_pub, &_vehicle_status.system_id);

	while (!should_exit()) {

		perf_begin(_loop_perf);

		const actuator_armed_s actuator_armed_prev{_actuator_armed};

		/* update parameters */
		const bool params_updated = _parameter_update_sub.updated();

		if (params_updated) {
			// clear update
			parameter_update_s update;
			_parameter_update_sub.copy(&update);

			updateParameters();

			_status_changed = true;
		}

		/* Update OA parameter */
		_vehicle_status.avoidance_system_required = _param_com_obs_avoid.get();

		handlePowerButtonState();

		systemPowerUpdate();

		landDetectorUpdate();

		safetyButtonUpdate();

		_multicopter_throw_launch.update(isArmed());

		vtolStatusUpdate();

		_home_position.update(_param_com_home_en.get(), !isArmed() && _vehicle_land_detected.landed);

		handleAutoDisarm();

		battery_status_check();

		checkForMissionUpdate();

		manualControlCheck();

		offboardControlCheck();

		// data link checks which update the status
		dataLinkCheck();

		// Check for failure detector status
		if (_failure_detector.update(_vehicle_status, _vehicle_control_mode)) {
			_vehicle_status.failure_detector_status = _failure_detector.getStatus().value;
			_status_changed = true;
		}

		modeManagementUpdate();

		const hrt_abstime now = hrt_absolute_time();

		const bool nav_state_or_failsafe_changed = handleModeIntentionAndFailsafe();

		// Run arming checks @ 10Hz
		if ((now >= _last_health_and_arming_check + 100_ms) || _status_changed || nav_state_or_failsafe_changed) {
			_last_health_and_arming_check = now;

			perf_begin(_preflight_check_perf);
			_health_and_arming_checks.update();
			bool pre_flight_checks_pass = _health_and_arming_checks.canArm(_vehicle_status.nav_state);

			if (_vehicle_status.pre_flight_checks_pass != pre_flight_checks_pass) {
				_vehicle_status.pre_flight_checks_pass = pre_flight_checks_pass;
				_status_changed = true;
			}

			perf_end(_preflight_check_perf);
			checkAndInformReadyForTakeoff();
		}

		// handle commands last, as the system needs to be updated to handle them
		handleCommandsFromModeExecutors();

		if (_vehicle_command_sub.updated()) {
			// got command
			const unsigned last_generation = _vehicle_command_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_vehicle_command_sub.copy(&cmd)) {
				if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("vehicle_command lost, generation %u -> %u", last_generation, _vehicle_command_sub.get_last_generation());
				}

				if (handle_command(cmd)) {
					_status_changed = true;
				}
			}
		}

		if (_action_request_sub.updated()) {
			const unsigned last_generation = _action_request_sub.get_last_generation();
			action_request_s action_request;

			if (_action_request_sub.copy(&action_request)) {
				if (_action_request_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("action_request lost, generation %u -> %u", last_generation, _action_request_sub.get_last_generation());
				}

				executeActionRequest(action_request);
			}
		}

		// update actuator_armed
		_actuator_armed.armed = isArmed();
		_actuator_armed.prearmed = getPrearmState();
		_actuator_armed.ready_to_arm = _vehicle_status.pre_flight_checks_pass || isArmed();
		_actuator_armed.lockdown = ((_vehicle_status.nav_state == _vehicle_status.NAVIGATION_STATE_TERMINATION)
					    || (_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON)
					    || _multicopter_throw_launch.isThrowLaunchInProgress());
		// _actuator_armed.manual_lockdown // action_request_s::ACTION_KILL
		_actuator_armed.force_failsafe = (_vehicle_status.nav_state == _vehicle_status.NAVIGATION_STATE_TERMINATION);
		// _actuator_armed.in_esc_calibration_mode // VEHICLE_CMD_PREFLIGHT_CALIBRATION

		// if force_failsafe or manual_lockdown activated send parachute command
		if ((!actuator_armed_prev.force_failsafe && _actuator_armed.force_failsafe)
		    || (!actuator_armed_prev.manual_lockdown && _actuator_armed.manual_lockdown)
		   ) {
			if (isArmed()) {
				send_parachute_command();
			}
		}

		// publish states (armed, control_mode, vehicle_status, failure_detector_status) at 2 Hz or immediately when changed
		if ((now >= _vehicle_status.timestamp + 500_ms) || _status_changed || nav_state_or_failsafe_changed
		    || !(_actuator_armed == actuator_armed_prev)) {

			// publish actuator_armed first (used by output modules)
			_actuator_armed.timestamp = hrt_absolute_time();
			_actuator_armed_pub.publish(_actuator_armed);

			// update and publish vehicle_control_mode
			updateControlMode();

			// vehicle_status publish (after prearm/preflight updates above)
			_mode_management.getModeStatus(_vehicle_status.valid_nav_states_mask, _vehicle_status.can_set_nav_states_mask);
			_vehicle_status.timestamp = hrt_absolute_time();
			_vehicle_status_pub.publish(_vehicle_status);

			// failure_detector_status publish
			failure_detector_status_s fd_status{};
			fd_status.fd_roll = _failure_detector.getStatusFlags().roll;
			fd_status.fd_pitch = _failure_detector.getStatusFlags().pitch;
			fd_status.fd_alt = _failure_detector.getStatusFlags().alt;
			fd_status.fd_ext = _failure_detector.getStatusFlags().ext;
			fd_status.fd_arm_escs = _failure_detector.getStatusFlags().arm_escs;
			fd_status.fd_battery = _failure_detector.getStatusFlags().battery;
			fd_status.fd_imbalanced_prop = _failure_detector.getStatusFlags().imbalanced_prop;
			fd_status.fd_motor = _failure_detector.getStatusFlags().motor;
			fd_status.imbalanced_prop_metric = _failure_detector.getImbalancedPropMetric();
			fd_status.motor_failure_mask = _failure_detector.getMotorFailures();
			fd_status.timestamp = hrt_absolute_time();
			_failure_detector_status_pub.publish(fd_status);
		}

		checkWorkerThread();

		updateTunes();
		control_status_leds(_status_changed, _battery_warning);

		_status_changed = false;

		arm_auth_update(hrt_absolute_time(), params_updated);

		px4_indicate_external_reset_lockout(LockoutComponent::Commander, isArmed());

		perf_end(_loop_perf);

		// sleep if there are no vehicle_commands or action_requests to process
		if (!_vehicle_command_sub.updated() && !_action_request_sub.updated()) {
			px4_usleep(COMMANDER_MONITORING_INTERVAL);
		}
	}

	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	/* close fds */
	led_deinit();
	buzzer_deinit();
}

void Commander::checkForMissionUpdate()
{
	if (_mission_result_sub.updated()) {
		const mission_result_s &mission_result = _mission_result_sub.get();

		const auto prev_mission_mission_id = mission_result.mission_id;
		_mission_result_sub.update();

		// if mission_result is valid for the current mission
		const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp)
					       && (mission_result.mission_id > 0);

		bool auto_mission_available = mission_result_ok && mission_result.valid;

		if (mission_result_ok) {
			/* Only evaluate mission state if home is set */
			if (!_failsafe_flags.home_position_invalid &&
			    (prev_mission_mission_id != mission_result.mission_id)) {

				if (!auto_mission_available) {
					/* the mission is invalid */
					tune_mission_fail(true);

				} else if (mission_result.warning) {
					/* the mission has a warning */
					tune_mission_warn(true);

				} else {
					/* the mission is valid */
					tune_mission_ok(true);
				}
			}
		}

		if (isArmed() && !_vehicle_land_detected.landed
		    && (mission_result.timestamp >= _vehicle_status.nav_state_timestamp)
		    && mission_result.finished
		    && _mode_management.modeExecutorInCharge() == ModeExecutors::AUTOPILOT_EXECUTOR_ID) {

			if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
			    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF) {
				// Transition mode to loiter or auto-mission after takeoff is completed.
				if ((_param_takeoff_finished_action.get() == 1) && auto_mission_available) {
					_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);

				} else {
					_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
				}

			} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
				// Transition to loiter when the mission is cleared and/or finished, and we are still in mission mode.
				_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
			}
		}
	}
}

bool Commander::getPrearmState() const
{
	if (_vehicle_status.calibration_enabled) {
		return false;
	}

	switch ((PrearmedMode)_param_com_prearm_mode.get()) {
	case PrearmedMode::DISABLED:
		/* skip prearmed state  */
		return false;

	case PrearmedMode::ALWAYS:
		/* safety is not present, go into prearmed
		* (all output drivers should be started / unlocked last in the boot process
		* when the rest of the system is fully initialized)
		*/
		return hrt_elapsed_time(&_boot_timestamp) > 5_s;

	case PrearmedMode::SAFETY_BUTTON:
		if (_safety.isButtonAvailable()) {
			/* safety button is present, go into prearmed if safety is off */
			return _safety.isSafetyOff();
		}

		/* safety button is not present, do not go into prearmed */
		return false;
	}

	return false;
}

void Commander::handlePowerButtonState()
{
#if defined(BOARD_HAS_POWER_CONTROL)

	/* handle power button state */
	if (_power_button_state_sub.updated()) {
		power_button_state_s button_state;

		if (_power_button_state_sub.copy(&button_state)) {
			if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
				if (!isArmed() && (px4_shutdown_request() == 0)) {
					while (1) { px4_usleep(1); }
				}
			}
		}
	}

#endif // BOARD_HAS_POWER_CONTROL
}

void Commander::systemPowerUpdate()
{
	system_power_s system_power;

	if (_system_power_sub.update(&system_power)) {

		if (hrt_elapsed_time(&system_power.timestamp) < 1_s) {
			if (system_power.servo_valid &&
			    !system_power.brick_valid &&
			    !system_power.usb_connected) {
				/* flying only on servo rail, this is unsafe */
				_vehicle_status.power_input_valid = false;

			} else {
				_vehicle_status.power_input_valid = true;
			}
		}
	}
}

void Commander::landDetectorUpdate()
{
	if (_vehicle_land_detected_sub.updated()) {
		const bool was_landed = _vehicle_land_detected.landed;
		_vehicle_land_detected_sub.copy(&_vehicle_land_detected);

		// Only take actions if armed
		if (isArmed()) {
			if (!was_landed && _vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, "Landing detected\t");
				events::send(events::ID("commander_landing_detected"), events::Log::Info, "Landing detected");

			} else if (was_landed && !_vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, "Takeoff detected\t");
				events::send(events::ID("commander_takeoff_detected"), events::Log::Info, "Takeoff detected");
				_vehicle_status.takeoff_time = hrt_absolute_time();
				_have_taken_off_since_arming = true;
			}

			// automatically set or update home position
			if (_param_com_home_en.get()) {
				// set the home position when taking off
				if (!_vehicle_land_detected.landed) {
					if (was_landed) {
						_home_position.setHomePosition();

					} else if (_param_com_home_in_air.get()) {
						_home_position.setInAirHomePosition();
					}
				}
			}
		}
	}
}

void Commander::safetyButtonUpdate()
{
	const bool safety_changed = _safety.safetyButtonHandler();
	_vehicle_status.safety_button_available = _safety.isButtonAvailable();
	_vehicle_status.safety_off = _safety.isSafetyOff();

	if (safety_changed) {
		// Notify the user if the status of the safety button changes
		if (!_safety.isSafetyDisabled()) {
			if (_safety.isSafetyOff()) {
				set_tune(tune_control_s::TUNE_ID_NOTIFY_POSITIVE);

			} else {
				tune_neutral(true);
			}
		}

		_status_changed = true;
	}
}

void Commander::vtolStatusUpdate()
{
	// Make sure that this is only adjusted if vehicle really is of type vtol
	if (_vtol_vehicle_status_sub.update(&_vtol_vehicle_status) && is_vtol(_vehicle_status)) {

		// Check if there has been any change while updating the flags (transition = rotary wing status)
		const auto new_vehicle_type = _vtol_vehicle_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW ?
					      vehicle_status_s::VEHICLE_TYPE_FIXED_WING :
					      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		if (new_vehicle_type != _vehicle_status.vehicle_type) {
			_vehicle_status.vehicle_type = new_vehicle_type;
			_status_changed = true;
		}

		const bool new_in_transition = _vtol_vehicle_status.vehicle_vtol_state ==
					       vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW
					       || _vtol_vehicle_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_MC;

		if (_vehicle_status.in_transition_mode != new_in_transition) {
			_vehicle_status.in_transition_mode = new_in_transition;
			_status_changed = true;
		}

		if (_vehicle_status.in_transition_to_fw != (_vtol_vehicle_status.vehicle_vtol_state ==
				vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW)) {
			_vehicle_status.in_transition_to_fw = (_vtol_vehicle_status.vehicle_vtol_state ==
							       vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW);
			_status_changed = true;
		}

	}
}

void Commander::updateTunes()
{
	// play arming and battery warning tunes
	if (!_arm_tune_played && isArmed()) {

		/* play tune when armed */
		set_tune(tune_control_s::TUNE_ID_ARMING_WARNING);
		_arm_tune_played = true;

	} else if (!_vehicle_status.usb_connected &&
		   (_vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
		   (_battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {

		/* play tune on battery critical */
		set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_FAST);

	} else if ((_vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
		   (_battery_warning == battery_status_s::BATTERY_WARNING_LOW)) {
		/* play tune on battery warning */
		set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_SLOW);

	} else if (_vehicle_status.failsafe && isArmed()) {
		tune_failsafe(true);

	} else if (_multicopter_throw_launch.isReadyToThrow()) {
		set_tune(tune_control_s::TUNE_ID_ARMING_WARNING);

	} else {
		set_tune(tune_control_s::TUNE_ID_STOP);
	}

	/* reset arm_tune_played when disarmed */
	if (!isArmed()) {

		// Notify the user that it is safe to approach the vehicle
		if (_arm_tune_played) {
			tune_neutral(true);
		}

		_arm_tune_played = false;
	}
}

void Commander::checkWorkerThread()
{
	// check if the worker has finished
	if (_worker_thread.hasResult()) {
		int ret = _worker_thread.getResultAndReset();
		_actuator_armed.in_esc_calibration_mode = false;

		if (_vehicle_status.calibration_enabled) { // did we do a calibration?
			_vehicle_status.calibration_enabled = false;

			if (ret == 0) {
				tune_positive(true);

			} else {
				tune_negative(true);
			}
		}
	}
}

void Commander::handleAutoDisarm()
{
	// Auto disarm when landed or kill switch engaged
	if (isArmed()) {

		// Check for auto-disarm on landing or pre-flight
		if (_param_com_disarm_land.get() > 0 || _param_com_disarm_preflight.get() > 0) {

			const bool landed_amid_mission = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
							 && !_mission_result_sub.get().finished;
			const bool auto_disarm_land_enabled = _param_com_disarm_land.get() > 0 && !landed_amid_mission
							      && !_config_overrides.disable_auto_disarm;

			if (auto_disarm_land_enabled && _have_taken_off_since_arming) {
				_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_land.get() * 1_s);
				_auto_disarm_landed.set_state_and_update(_vehicle_land_detected.landed, hrt_absolute_time());

			} else if (_param_com_disarm_preflight.get() > 0 && !_have_taken_off_since_arming) {
				_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);
				_auto_disarm_landed.set_state_and_update(true, hrt_absolute_time());
			}

			if (_auto_disarm_landed.get_state() && !_multicopter_throw_launch.isThrowLaunchInProgress()) {
				if (_have_taken_off_since_arming) {
					disarm(arm_disarm_reason_t::auto_disarm_land);

				} else {
					disarm(arm_disarm_reason_t::auto_disarm_preflight);
				}
			}
		}

		// Auto disarm after 5 seconds if kill switch is engaged
		bool auto_disarm = _actuator_armed.manual_lockdown;

		// auto disarm if locked down to avoid user confusion
		//  skipped in HITL where lockdown is enabled for safety
		if (_vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON) {
			auto_disarm |= _actuator_armed.lockdown;
		}

		//don't disarm if throw launch is in progress
		auto_disarm &= !_multicopter_throw_launch.isThrowLaunchInProgress();

		_auto_disarm_killed.set_state_and_update(auto_disarm, hrt_absolute_time());

		if (_auto_disarm_killed.get_state()) {
			if (_actuator_armed.manual_lockdown) {
				disarm(arm_disarm_reason_t::kill_switch, true);

			} else {
				disarm(arm_disarm_reason_t::lockdown, true);
			}
		}

	} else {
		_auto_disarm_landed.set_state_and_update(false, hrt_absolute_time());
		_auto_disarm_killed.set_state_and_update(false, hrt_absolute_time());
	}
}

bool Commander::handleModeIntentionAndFailsafe()
{
	const uint8_t prev_nav_state = _vehicle_status.nav_state;
	const FailsafeBase::Action prev_failsafe_action = _failsafe.selectedAction();
	const uint8_t prev_failsafe_defer_state = _vehicle_status.failsafe_defer_state;

	FailsafeBase::State state{};
	state.armed = isArmed();
	state.vtol_in_transition_mode = _vehicle_status.in_transition_mode;
	state.mission_finished = _mission_result_sub.get().finished;
	state.user_intended_mode = _user_mode_intention.get();
	state.vehicle_type = _vehicle_status.vehicle_type;

	// There might have been a mode change request without changing the user intended mode.
	// If a failsafe is active we must pass the request along as it might lead to a user-takeover.
	bool mode_change_requested = _user_mode_intention.getHadModeChangeAndClear();

	uint8_t updated_user_intented_mode = _failsafe.update(hrt_absolute_time(), state, mode_change_requested,
					     _failsafe_user_override_request,
					     _failsafe_flags);
	_failsafe_user_override_request = false;

	// Force intended mode if changed by the failsafe state machine
	if (state.user_intended_mode != updated_user_intented_mode) {
		_user_mode_intention.change(updated_user_intented_mode, ModeChangeSource::User, false, true);
		_user_mode_intention.getHadModeChangeAndClear();
	}

	// Handle failsafe action
	_vehicle_status.nav_state_user_intention = _mode_management.getNavStateReplacementIfValid(_user_mode_intention.get(),
			false);
	_vehicle_status.nav_state = _mode_management.getNavStateReplacementIfValid(FailsafeBase::modeFromAction(
					    _failsafe.selectedAction(), _user_mode_intention.get()));
	_vehicle_status.executor_in_charge = _mode_management.modeExecutorInCharge(); // Set this in sync with nav_state

	switch (_failsafe.selectedAction()) {
	case FailsafeBase::Action::Disarm:
		disarm(arm_disarm_reason_t::failsafe, true);
		break;

	case FailsafeBase::Action::Terminate:
		_vehicle_status.nav_state = _vehicle_status.NAVIGATION_STATE_TERMINATION;
		break;

	default:
		break;
	}

	_vehicle_status.failsafe = _failsafe.inFailsafe();
	_vehicle_status.failsafe_and_user_took_over = _failsafe.userTakeoverActive();

	if (prev_nav_state != _vehicle_status.nav_state) {
		_vehicle_status.nav_state_timestamp = hrt_absolute_time();
	}

	_mode_management.updateActiveConfigOverrides(_vehicle_status.nav_state, _config_overrides);

	// Apply failsafe deferring & get the current state
	_failsafe.deferFailsafes(_config_overrides.defer_failsafes, _config_overrides.defer_failsafes_timeout_s);

	if (_failsafe.failsafeDeferred()) {
		_vehicle_status.failsafe_defer_state = vehicle_status_s::FAILSAFE_DEFER_STATE_WOULD_FAILSAFE;

	} else if (_failsafe.getDeferFailsafes()) {
		_vehicle_status.failsafe_defer_state = vehicle_status_s::FAILSAFE_DEFER_STATE_ENABLED;

	} else {
		_vehicle_status.failsafe_defer_state = vehicle_status_s::FAILSAFE_DEFER_STATE_DISABLED;
	}

	return prev_nav_state != _vehicle_status.nav_state ||
	       prev_failsafe_action != _failsafe.selectedAction() ||
	       prev_failsafe_defer_state != _vehicle_status.failsafe_defer_state;
}

void Commander::checkAndInformReadyForTakeoff()
{
#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	static bool ready_for_takeoff_printed = false;

	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ||
	    _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (!ready_for_takeoff_printed &&
		    _health_and_arming_checks.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)) {
			PX4_INFO("%sReady for takeoff!%s", PX4_ANSI_COLOR_GREEN, PX4_ANSI_COLOR_RESET);
			ready_for_takeoff_printed = true;
		}
	}

#endif // CONFIG_ARCH_BOARD_PX4_SITL
}

void Commander::modeManagementUpdate()
{
	ModeManagement::UpdateRequest mode_management_update{};
	_mode_management.update(isArmed(), _vehicle_status.nav_state_user_intention,
				_failsafe.selectedAction() > FailsafeBase::Action::Warn, mode_management_update);

	if (!isArmed() && mode_management_update.change_user_intended_nav_state) {
		_user_mode_intention.change(mode_management_update.user_intended_nav_state);
	}

	if (mode_management_update.control_setpoint_update) {
		_status_changed = true;
	}
}

void Commander::control_status_leds(bool changed, const uint8_t battery_warning)
{
	switch (blink_msg_state()) {
	case 1:
		// blinking LED message, don't touch LEDs
		return;

	case 2:
		// blinking LED message completed, restore normal state
		changed = true;
		break;

	default:
		break;
	}

	const hrt_abstime time_now_us = hrt_absolute_time();

	if (_cpuload_sub.updated()) {
		cpuload_s cpuload;

		if (_cpuload_sub.copy(&cpuload)) {
			const float cpuload_percent = cpuload.load * 100.f;

			bool overload = false;

			// Only check CPU load if it hasn't been disabled
			if (!(_param_com_cpu_max.get() < FLT_EPSILON)) {
				overload = (cpuload_percent > _param_com_cpu_max.get());
			}

			overload = overload || (cpuload.ram_usage > 0.99f);

			if (_overload_start == 0 && overload) {
				_overload_start = time_now_us;

			} else if (!overload) {
				_overload_start = 0;
			}
		}
	}

	const bool overload = (_overload_start != 0);

	// driving the RGB led
	if (changed || _last_overload != overload) {
		uint8_t led_mode = led_control_s::MODE_OFF;
		uint8_t led_color = led_control_s::COLOR_WHITE;
		bool set_normal_color = false;

		uint64_t overload_warn_delay = isArmed() ? 1_ms : 250_ms;

		// set mode
		if (overload && (time_now_us >= _overload_start + overload_warn_delay)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_PURPLE;

		} else if (_multicopter_throw_launch.isReadyToThrow()) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_YELLOW;

		} else if (isArmed()) {
			led_mode = led_control_s::MODE_ON;
			set_normal_color = true;

		} else if (!_vehicle_status.pre_flight_checks_pass) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_RED;

		} else {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;
		}

		if (set_normal_color) {
			// set color
			if (_vehicle_status.failsafe) {
				led_color = led_control_s::COLOR_PURPLE;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_LOW) {
				led_color = led_control_s::COLOR_AMBER;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
				led_color = led_control_s::COLOR_RED;

			} else {
				if (!_failsafe_flags.home_position_invalid && !_failsafe_flags.global_position_invalid) {
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

	_last_overload = overload;

#if !defined(CONFIG_ARCH_LEDS) && defined(BOARD_HAS_CONTROL_STATUS_LEDS)

	if (isArmed()) {
		if (_vehicle_status.failsafe) {
			BOARD_ARMED_LED_OFF();

			if (time_now_us >= _led_armed_state_toggle + 250_ms) {
				_led_armed_state_toggle = time_now_us;
				BOARD_ARMED_STATE_LED_TOGGLE();
			}

		} else {
			BOARD_ARMED_STATE_LED_OFF();

			// armed, solid
			BOARD_ARMED_LED_ON();
		}

	} else if (_vehicle_status.pre_flight_checks_pass) {
		BOARD_ARMED_LED_OFF();

		// ready to arm, blink at 1Hz
		if (time_now_us >= _led_armed_state_toggle + 1_s) {
			_led_armed_state_toggle = time_now_us;
			BOARD_ARMED_STATE_LED_TOGGLE();
		}

	} else {
		BOARD_ARMED_LED_OFF();

		// not ready to arm, blink at 10Hz
		if (time_now_us >= _led_armed_state_toggle + 100_ms) {
			_led_armed_state_toggle = time_now_us;
			BOARD_ARMED_STATE_LED_TOGGLE();
		}
	}

#endif

	// give system warnings on error LED
	if (overload) {
		if (time_now_us >= _led_overload_toggle + 50_ms) {
			_led_overload_toggle = time_now_us;
			BOARD_OVERLOAD_LED_TOGGLE();
		}

	} else {
		BOARD_OVERLOAD_LED_OFF();
	}
}

void Commander::updateControlMode()
{
	_vehicle_control_mode = {};

	mode_util::getVehicleControlMode(_vehicle_status.nav_state,
					 _vehicle_status.vehicle_type, _offboard_control_mode_sub.get(), _vehicle_control_mode);
	_mode_management.updateControlMode(_vehicle_status.nav_state, _vehicle_control_mode);

	_vehicle_control_mode.flag_armed = isArmed();
	_vehicle_control_mode.flag_multicopter_position_control_enabled =
		(_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
		&& (_vehicle_control_mode.flag_control_altitude_enabled
		    || _vehicle_control_mode.flag_control_climb_rate_enabled
		    || _vehicle_control_mode.flag_control_position_enabled
		    || _vehicle_control_mode.flag_control_velocity_enabled
		    || _vehicle_control_mode.flag_control_acceleration_enabled);
	_vehicle_control_mode.timestamp = hrt_absolute_time();
	_vehicle_control_mode_pub.publish(_vehicle_control_mode);
}

void Commander::printRejectMode(uint8_t nav_state)
{
	if (hrt_elapsed_time(&_last_print_mode_reject_time) > 1_s) {

		mavlink_log_critical(&_mavlink_log_pub, "Switching to %s is currently not available\t",
				     mode_util::nav_state_names[nav_state]);
		px4_custom_mode custom_mode = get_px4_custom_mode(nav_state);
		uint32_t mavlink_mode = custom_mode.data;
		/* EVENT
		 * @type append_health_and_arming_messages
		 */
		events::send<uint32_t, events::px4::enums::navigation_mode_t>(events::ID("commander_modeswitch_not_avail"), {events::Log::Critical, events::LogInternal::Info},
				"Switching to mode '{2}' is currently not possible", mavlink_mode, mode_util::navigation_mode(nav_state));

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		tune_negative(isArmed());

		_last_print_mode_reject_time = hrt_absolute_time();
	}
}

void Commander::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	switch (result) {
	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED:
		tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED:
		tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		tune_negative(true);
		break;

	default:
		break;
	}

	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	_vehicle_command_ack_pub.publish(command_ack);
}

int Commander::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("commander",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 40,
				      PX4_STACK_ADJUSTED(3250),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
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
	_vehicle_status.hil_state = vehicle_status_s::HIL_STATE_ON;
}

void Commander::dataLinkCheck()
{
	// high latency data link
	iridiumsbd_status_s iridium_status;

	if (_iridiumsbd_status_sub.update(&iridium_status)) {
		_high_latency_datalink_timestamp = iridium_status.last_at_ok_timestamp;

		if (_vehicle_status.high_latency_data_link_lost &&
		    (_high_latency_datalink_timestamp > _high_latency_datalink_lost) &&
		    (_high_latency_datalink_regained == 0)
		   ) {
			_high_latency_datalink_regained = _high_latency_datalink_timestamp;
		}

		if (_vehicle_status.high_latency_data_link_lost &&
		    (_high_latency_datalink_regained != 0) &&
		    (hrt_elapsed_time(&_high_latency_datalink_regained) > (_param_com_hldl_reg_t.get() * 1_s))
		   ) {
			_vehicle_status.high_latency_data_link_lost = false;
			_status_changed = true;
		}
	}

	for (auto &telemetry_status :  _telemetry_status_subs) {
		telemetry_status_s telemetry;

		if (telemetry_status.update(&telemetry)) {

			// handle different radio types
			switch (telemetry.type) {
			case telemetry_status_s::LINK_TYPE_USB:
				// set (but don't unset) telemetry via USB as active once a MAVLink connection is up
				_vehicle_status.usb_connected = true;
				break;

			case telemetry_status_s::LINK_TYPE_IRIDIUM: {

					if ((_high_latency_datalink_timestamp > 0) &&
					    (hrt_elapsed_time(&_high_latency_datalink_timestamp) > (_param_com_hldl_loss_t.get() * 1_s))) {

						_high_latency_datalink_lost = _high_latency_datalink_timestamp;
						_high_latency_datalink_regained = 0;

						if (!_vehicle_status.high_latency_data_link_lost) {
							_vehicle_status.high_latency_data_link_lost = true;
							mavlink_log_critical(&_mavlink_log_pub, "High latency data link lost\t");
							events::send(events::ID("commander_high_latency_lost"), events::Log::Critical, "High latency data link lost");
							_status_changed = true;
						}
					}

					break;
				}
			}

			if (telemetry.heartbeat_type_gcs) {
				// Initial connection or recovery from data link lost
				if (_vehicle_status.gcs_connection_lost) {
					_vehicle_status.gcs_connection_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_gcs != 0) {
						mavlink_log_info(&_mavlink_log_pub, "GCS connection regained\t");
						events::send(events::ID("commander_dl_regained"), events::Log::Info, "GCS connection regained");
					}
				}

				_datalink_last_heartbeat_gcs = telemetry.timestamp;
			}

			if (telemetry.heartbeat_type_onboard_controller) {
				if (_onboard_controller_lost) {
					_onboard_controller_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_onboard_controller != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Onboard controller regained\t");
						events::send(events::ID("commander_onboard_ctrl_regained"), events::Log::Info, "Onboard controller regained");
					}
				}

				_datalink_last_heartbeat_onboard_controller = telemetry.timestamp;
			}

			if (telemetry.heartbeat_type_parachute) {
				if (_parachute_system_lost) {
					_parachute_system_lost = false;

					if (_datalink_last_heartbeat_parachute_system != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Parachute system regained\t");
						events::send(events::ID("commander_parachute_regained"), events::Log::Info, "Parachute system regained");
					}
				}

				bool healthy = telemetry.parachute_system_healthy;

				_datalink_last_heartbeat_parachute_system = telemetry.timestamp;
				_vehicle_status.parachute_system_present = true;
				_vehicle_status.parachute_system_healthy = healthy;
			}

			if (telemetry.heartbeat_type_open_drone_id) {
				if (_open_drone_id_system_lost) {
					_open_drone_id_system_lost = false;

					if (_datalink_last_heartbeat_open_drone_id_system != 0) {
						mavlink_log_info(&_mavlink_log_pub, "OpenDroneID system regained\t");
						events::send(events::ID("commander_open_drone_id_regained"), events::Log::Info, "OpenDroneID system regained");
					}
				}

				bool healthy = telemetry.open_drone_id_system_healthy;

				_datalink_last_heartbeat_open_drone_id_system = telemetry.timestamp;
				_vehicle_status.open_drone_id_system_present = true;
				_vehicle_status.open_drone_id_system_healthy = healthy;
			}

			if (telemetry.heartbeat_component_obstacle_avoidance) {
				if (_avoidance_system_lost) {
					_avoidance_system_lost = false;
					_status_changed = true;
				}

				_datalink_last_heartbeat_avoidance_system = telemetry.timestamp;
				_vehicle_status.avoidance_system_valid = telemetry.avoidance_system_healthy;
			}
		}
	}


	// GCS data link loss failsafe
	if (!_vehicle_status.gcs_connection_lost) {
		if ((_datalink_last_heartbeat_gcs != 0)
		    && hrt_elapsed_time(&_datalink_last_heartbeat_gcs) > (_param_com_dl_loss_t.get() * 1_s)) {

			_vehicle_status.gcs_connection_lost = true;
			_vehicle_status.gcs_connection_lost_counter++;

			mavlink_log_info(&_mavlink_log_pub, "Connection to ground station lost\t");
			events::send(events::ID("commander_gcs_lost"), {events::Log::Warning, events::LogInternal::Info},
				     "Connection to ground control station lost");

			_status_changed = true;
		}
	}

	// ONBOARD CONTROLLER data link loss failsafe
	if ((_datalink_last_heartbeat_onboard_controller > 0)
	    && (hrt_elapsed_time(&_datalink_last_heartbeat_onboard_controller) > (_param_com_obc_loss_t.get() * 1_s))
	    && !_onboard_controller_lost) {

		mavlink_log_critical(&_mavlink_log_pub, "Connection to mission computer lost\t");
		events::send(events::ID("commander_mission_comp_lost"), events::Log::Critical, "Connection to mission computer lost");
		_onboard_controller_lost = true;
		_status_changed = true;
	}

	// Parachute system
	if ((hrt_elapsed_time(&_datalink_last_heartbeat_parachute_system) > 3_s)
	    && !_parachute_system_lost) {
		mavlink_log_critical(&_mavlink_log_pub, "Parachute system lost");
		_vehicle_status.parachute_system_present = false;
		_vehicle_status.parachute_system_healthy = false;
		_parachute_system_lost = true;
		_status_changed = true;
	}

	// OpenDroneID system
	if ((hrt_elapsed_time(&_datalink_last_heartbeat_open_drone_id_system) > 3_s)
	    && !_open_drone_id_system_lost) {
		mavlink_log_critical(&_mavlink_log_pub, "OpenDroneID system lost");
		events::send(events::ID("commander_open_drone_id_lost"), events::Log::Critical, "OpenDroneID system lost");
		_vehicle_status.open_drone_id_system_present = false;
		_vehicle_status.open_drone_id_system_healthy = false;
		_open_drone_id_system_lost = true;
		_status_changed = true;
	}

	// AVOIDANCE SYSTEM state check (only if it is enabled)
	if (_vehicle_status.avoidance_system_required && !_onboard_controller_lost) {
		// if heartbeats stop
		if (!_avoidance_system_lost && (_datalink_last_heartbeat_avoidance_system > 0)
		    && (hrt_elapsed_time(&_datalink_last_heartbeat_avoidance_system) > 5_s)) {

			_avoidance_system_lost = true;
			_vehicle_status.avoidance_system_valid = false;
		}
	}
}

void Commander::battery_status_check()
{
	// Handle shutdown request from emergency battery action
	if (_battery_warning != _failsafe_flags.battery_warning) {

		if (_failsafe_flags.battery_warning == battery_status_s::BATTERY_WARNING_EMERGENCY) {
#if defined(BOARD_HAS_POWER_CONTROL)

			if (!isArmed() && (px4_shutdown_request(60_s) == 0)) {
				mavlink_log_critical(&_mavlink_log_pub, "Dangerously low battery! Shutting system down in 60 seconds\t");
				events::send(events::ID("commander_low_bat_shutdown"), {events::Log::Emergency, events::LogInternal::Warning},
					     "Dangerously low battery! Shutting system down");

				while (1) { px4_usleep(1); }

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "System does not support shutdown\t");
				/* EVENT
				 * @description Cannot shut down, most likely the system does not support it.
				 */
				events::send(events::ID("commander_low_bat_shutdown_failed"), {events::Log::Emergency, events::LogInternal::Error},
					     "Dangerously low battery! System shut down failed");
			}

#endif // BOARD_HAS_POWER_CONTROL
		}
	}

	_battery_warning = _failsafe_flags.battery_warning;
}

void Commander::manualControlCheck()
{
	manual_control_setpoint_s manual_control_setpoint;
	const bool manual_control_updated = _manual_control_setpoint_sub.update(&manual_control_setpoint);

	if (manual_control_updated && manual_control_setpoint.valid) {

		_is_throttle_above_center = (manual_control_setpoint.throttle > 0.2f);
		_is_throttle_low = (manual_control_setpoint.throttle < -0.8f);

		if (isArmed()) {
			// Abort autonomous mode and switch to position mode if sticks are moved significantly
			// but only if actually in air.
			if (manual_control_setpoint.sticks_moving
			    && !_vehicle_control_mode.flag_control_manual_enabled
			    && (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
			   ) {
				bool override_enabled = false;

				if (_vehicle_control_mode.flag_control_auto_enabled) {
					if (_param_com_rc_override.get() & static_cast<int32_t>(RcOverrideBits::AUTO_MODE_BIT)) {
						override_enabled = true;
					}
				}

				if (_vehicle_control_mode.flag_control_offboard_enabled) {
					if (_param_com_rc_override.get() & static_cast<int32_t>(RcOverrideBits::OFFBOARD_MODE_BIT)) {
						override_enabled = true;
					}
				}

				if (override_enabled) {
					// If no failsafe is active, directly change the mode, otherwise pass the request to the failsafe state machine
					if (_failsafe.selectedAction() <= FailsafeBase::Action::Warn) {
						if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_POSCTL, ModeChangeSource::User, true)) {
							tune_positive(true);
							mavlink_log_info(&_mavlink_log_pub, "Pilot took over using sticks\t");
							events::send(events::ID("commander_rc_override"), events::Log::Info, "Pilot took over using sticks");
						}

					} else {
						_failsafe_user_override_request = true;
					}
				}
			}

		} else {
			const bool is_mavlink = (manual_control_setpoint.data_source > manual_control_setpoint_s::SOURCE_RC);

			// if there's never been a mode change force position control as initial state
			if (!_user_mode_intention.everHadModeChange() && (is_mavlink || !_mode_switch_mapped)) {
				_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_POSCTL, ModeChangeSource::User, false, true);
			}
		}
	}
}

void Commander::offboardControlCheck()
{
	if (_offboard_control_mode_sub.update()) {
		if (_failsafe_flags.offboard_control_signal_lost) {
			// Run arming checks immediately to allow for offboard mode activation
			_status_changed = true;
		}
	}
}

void Commander::send_parachute_command()
{
	vehicle_command_s vcmd{};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
	vcmd.param1 = static_cast<float>(vehicle_command_s::PARACHUTE_ACTION_RELEASE);

	vcmd.source_system = _vehicle_status.system_id;
	vcmd.target_system = _vehicle_status.system_id;
	vcmd.source_component = _vehicle_status.component_id;
	vcmd.target_component = 161; // MAV_COMP_ID_PARACHUTE

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vcmd_pub.publish(vcmd);

	set_tune_override(tune_control_s::TUNE_ID_PARACHUTE_RELEASE);
}

int Commander::print_usage(const char *reason)
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
#ifndef CONSTRAINED_FLASH
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Run sensor calibration");
	PRINT_MODULE_USAGE_ARG("mag|baro|accel|gyro|level|esc|airspeed", "Calibration type", false);
	PRINT_MODULE_USAGE_ARG("quick", "Quick calibration (accel only, not recommended)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("check", "Run preflight checks");
	PRINT_MODULE_USAGE_COMMAND("arm");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Force arming (do not run preflight checks)", true);
	PRINT_MODULE_USAGE_COMMAND("disarm");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Force disarming (disarm in air)", true);
	PRINT_MODULE_USAGE_COMMAND("takeoff");
	PRINT_MODULE_USAGE_COMMAND("land");
	PRINT_MODULE_USAGE_COMMAND_DESCR("transition", "VTOL transition");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode", "Change flight mode");
	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|altctl|posctl|position:slow|auto:mission|auto:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland|ext1",
			"Flight mode", false);
	PRINT_MODULE_USAGE_COMMAND("pair");
	PRINT_MODULE_USAGE_COMMAND("lockdown");
	PRINT_MODULE_USAGE_ARG("on|off", "Turn lockdown on or off", false);
	PRINT_MODULE_USAGE_COMMAND("set_ekf_origin");
	PRINT_MODULE_USAGE_ARG("lat, lon, alt", "Origin Latitude, Longitude, Altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("lat|lon|alt", "Origin latitude longitude altitude");
	PRINT_MODULE_USAGE_COMMAND_DESCR("poweroff", "Power off board (if supported)");
#endif
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 1;
}

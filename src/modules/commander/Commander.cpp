/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
#include "Arming/PreFlightCheck/PreFlightCheck.hpp"
#include "Arming/ArmAuthorization/ArmAuthorization.h"
#include "Arming/HealthFlags/HealthFlags.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#include "px4_custom_mode.h"
#include "state_machine_helper.h"

/* PX4 headers */
#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <navigator/navigation.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <circuit_breaker/circuit_breaker.h>
#include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>
#include <matrix/math.hpp>

#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/tune_control.h>

typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
	VEHICLE_MODE_FLAG_ENUM_END = 129, /*  | */
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
		a.in_esc_calibration_mode == b.in_esc_calibration_mode &&
		a.soft_stop == b.soft_stop);
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
		uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
		vehicle_status_s vehicle_status{};
		vehicle_status_sub.copy(&vehicle_status);

		uORB::Subscription vehicle_status_flags_sub{ORB_ID(vehicle_status_flags)};
		vehicle_status_flags_s vehicle_status_flags{};
		vehicle_status_flags_sub.copy(&vehicle_status_flags);

		uORB::Subscription vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
		vehicle_control_mode_s vehicle_control_mode{};
		vehicle_control_mode_sub.copy(&vehicle_control_mode);

		bool preflight_check_res = PreFlightCheck::preflightCheck(nullptr, vehicle_status, vehicle_status_flags,
					   vehicle_control_mode,
					   true, // report_failures
					   false, // safety_buttton_available not known
					   false); // safety_off not known
		PX4_INFO("Preflight check: %s", preflight_check_res ? "OK" : "FAILED");

		print_health_flags(vehicle_status);

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
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
				     static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
				     0.f);

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
	PX4_INFO("Arm state: %s", _arm_state_machine.getArmStateName());
	PX4_INFO("navigation: %s", nav_state_names[_vehicle_status.nav_state]);
	perf_print_counter(_loop_perf);
	perf_print_counter(_preflight_check_perf);
	return 0;
}

extern "C" __EXPORT int commander_main(int argc, char *argv[])
{
	return Commander::main(argc, argv);
}

bool Commander::shutdown_if_allowed()
{
	return TRANSITION_DENIED != _arm_state_machine.arming_state_transition(_vehicle_status, _vehicle_control_mode,
			_safety.isButtonAvailable(), _safety.isSafetyOff(),
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			_actuator_armed, false /* fRunPreArmChecks */, &_mavlink_log_pub, _vehicle_status_flags,
			arm_disarm_reason_t::shutdown);
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
	}

	return "";
};

using battery_fault_reason_t = events::px4::enums::battery_fault_reason_t;
static_assert(battery_status_s::BATTERY_FAULT_COUNT == (static_cast<uint8_t>(battery_fault_reason_t::_max) + 1)
	      , "Battery fault flags mismatch!");

static constexpr const char *battery_fault_reason_str(battery_fault_reason_t battery_fault_reason)
{
	switch (battery_fault_reason) {
	case battery_fault_reason_t::deep_discharge: return "under voltage";

	case battery_fault_reason_t::voltage_spikes: return "over voltage";

	case battery_fault_reason_t::cell_fail: return "cell fault";

	case battery_fault_reason_t::over_current: return "over current";

	case battery_fault_reason_t::fault_temperature: return "critical temperature";

	case battery_fault_reason_t::under_temperature: return "under temperature";

	case battery_fault_reason_t::incompatible_voltage: return "voltage mismatch";

	case battery_fault_reason_t::incompatible_firmware: return "incompatible firmware";

	case battery_fault_reason_t::incompatible_model: return "incompatible model";

	case battery_fault_reason_t::hardware_fault: return "hardware fault";

	case battery_fault_reason_t::over_temperature: return "near temperature limit";

	}

	return "";
};

using battery_mode_t = events::px4::enums::battery_mode_t;
static_assert(battery_status_s::BATTERY_MODE_COUNT == (static_cast<uint8_t>(battery_mode_t::_max) + 1)
	      , "Battery mode flags mismatch!");
static constexpr const char *battery_mode_str(battery_mode_t battery_mode)
{
	switch (battery_mode) {
	case battery_mode_t::autodischarging: return "auto discharging";

	case battery_mode_t::hotswap: return "hot-swap";

	default: return "unknown";
	}
}

using esc_fault_reason_t = events::px4::enums::esc_fault_reason_t;
static_assert(esc_report_s::ESC_FAILURE_COUNT == (static_cast<uint8_t>(esc_fault_reason_t::_max) + 1)
	      , "ESC fault flags mismatch!");
static constexpr const char *esc_fault_reason_str(esc_fault_reason_t esc_fault_reason)
{
	switch (esc_fault_reason) {
	case esc_fault_reason_t::over_current: return "over current";

	case esc_fault_reason_t::over_voltage: return "over voltage";

	case esc_fault_reason_t::motor_over_temp: return "motor critical temperature";

	case esc_fault_reason_t::over_rpm: return "over RPM";

	case esc_fault_reason_t::inconsistent_cmd: return "control failure";

	case esc_fault_reason_t::motor_stuck: return "motor stall";

	case esc_fault_reason_t::failure_generic: return "hardware failure";

	case esc_fault_reason_t::motor_warn_temp: return "motor over temperature";

	case esc_fault_reason_t::esc_warn_temp: return "over temperature";

	case esc_fault_reason_t::esc_over_temp: return "critical temperature";

	}

	return "";
};



using navigation_mode_t = events::px4::enums::navigation_mode_t;

static inline navigation_mode_t navigation_mode(uint8_t main_state)
{
	switch (main_state) {
	case commander_state_s::MAIN_STATE_MANUAL: return navigation_mode_t::manual;

	case commander_state_s::MAIN_STATE_ALTCTL: return navigation_mode_t::altctl;

	case commander_state_s::MAIN_STATE_POSCTL: return navigation_mode_t::posctl;

	case commander_state_s::MAIN_STATE_AUTO_MISSION: return navigation_mode_t::auto_mission;

	case commander_state_s::MAIN_STATE_AUTO_LOITER: return navigation_mode_t::auto_loiter;

	case commander_state_s::MAIN_STATE_AUTO_RTL: return navigation_mode_t::auto_rtl;

	case commander_state_s::MAIN_STATE_ACRO: return navigation_mode_t::acro;

	case commander_state_s::MAIN_STATE_OFFBOARD: return navigation_mode_t::offboard;

	case commander_state_s::MAIN_STATE_STAB: return navigation_mode_t::stab;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF: return navigation_mode_t::auto_takeoff;

	case commander_state_s::MAIN_STATE_AUTO_LAND: return navigation_mode_t::auto_land;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET: return navigation_mode_t::auto_follow_target;

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND: return navigation_mode_t::auto_precland;

	case commander_state_s::MAIN_STATE_ORBIT: return navigation_mode_t::orbit;

	case commander_state_s::MAIN_STATE_AUTO_VTOL_TAKEOFF: return navigation_mode_t::auto_vtol_takeoff;
	}

	static_assert(commander_state_s::MAIN_STATE_MAX - 1 == (int)navigation_mode_t::auto_vtol_takeoff,
		      "enum definition mismatch");

	return navigation_mode_t::unknown;
}

static constexpr const char *main_state_str(uint8_t main_state)
{
	switch (main_state) {
	case commander_state_s::MAIN_STATE_MANUAL: return "Manual";

	case commander_state_s::MAIN_STATE_ALTCTL: return "Altitude";

	case commander_state_s::MAIN_STATE_POSCTL: return "Position";

	case commander_state_s::MAIN_STATE_AUTO_MISSION: return "Mission";

	case commander_state_s::MAIN_STATE_AUTO_LOITER: return "Hold";

	case commander_state_s::MAIN_STATE_AUTO_RTL: return "RTL";

	case commander_state_s::MAIN_STATE_ACRO: return "Acro";

	case commander_state_s::MAIN_STATE_OFFBOARD: return "Offboard";

	case commander_state_s::MAIN_STATE_STAB: return "Stabilized";

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF: return "Takeoff";

	case commander_state_s::MAIN_STATE_AUTO_LAND: return "Land";

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET: return "Follow target";

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND: return "Precision land";

	case commander_state_s::MAIN_STATE_ORBIT: return "Orbit";

	default: return "Unknown";
	}
}

transition_result_t Commander::arm(arm_disarm_reason_t calling_reason, bool run_preflight_checks)
{
	// allow a grace period for re-arming: preflight checks don't need to pass during that time, for example for accidential in-air disarming
	if (calling_reason == arm_disarm_reason_t::rc_switch
	    && (hrt_elapsed_time(&_last_disarmed_timestamp) < 5_s)) {
		run_preflight_checks = false;
	}

	if (run_preflight_checks && !_arm_state_machine.isArmed()) {
		if (_vehicle_control_mode.flag_control_manual_enabled) {
			if (_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_vehicle_status.rc_signal_lost && _is_throttle_above_center) {
				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: throttle above center\t");
				events::send(events::ID("commander_arm_denied_throttle_center"),
				{events::Log::Critical, events::LogInternal::Info},
				"Arming denied: throttle above center");
				tune_negative(true);
				return TRANSITION_DENIED;
			}

			if (!_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_vehicle_status.rc_signal_lost && !_is_throttle_low
			    && _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROVER) {
				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: high throttle\t");
				events::send(events::ID("commander_arm_denied_throttle_high"),
				{events::Log::Critical, events::LogInternal::Info},
				"Arming denied: high throttle");
				tune_negative(true);
				return TRANSITION_DENIED;
			}

		} else if (calling_reason == arm_disarm_reason_t::rc_stick
			   || calling_reason == arm_disarm_reason_t::rc_switch
			   || calling_reason == arm_disarm_reason_t::rc_button) {
			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: switch to manual mode first\t");
			events::send(events::ID("commander_arm_denied_not_manual"),
			{events::Log::Critical, events::LogInternal::Info},
			"Arming denied: switch to manual mode first");
			tune_negative(true);
			return TRANSITION_DENIED;
		}

		if ((_geofence_result.geofence_action == geofence_result_s::GF_ACTION_RTL)
		    && !_vehicle_status_flags.home_position_valid) {
			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: Geofence RTL requires valid home\t");
			events::send(events::ID("commander_arm_denied_geofence_rtl"),
			{events::Log::Critical, events::LogInternal::Info},
			"Arming denied: Geofence RTL requires valid home");
			tune_negative(true);
			return TRANSITION_DENIED;
		}
	}

	transition_result_t arming_res = _arm_state_machine.arming_state_transition(_vehicle_status, _vehicle_control_mode,
					 _safety.isButtonAvailable(), _safety.isSafetyOff(),
					 vehicle_status_s::ARMING_STATE_ARMED, _actuator_armed, run_preflight_checks,
					 &_mavlink_log_pub, _vehicle_status_flags,
					 calling_reason);

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(&_mavlink_log_pub, "Armed by %s\t", arm_disarm_reason_str(calling_reason));
		events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_armed_by"), events::Log::Info,
				"Armed by {1}", calling_reason);

		_status_changed = true;

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

transition_result_t Commander::disarm(arm_disarm_reason_t calling_reason, bool forced)
{
	if (!forced) {
		const bool landed = (_vehicle_land_detected.landed || _vehicle_land_detected.maybe_landed
				     || is_ground_rover(_vehicle_status));
		const bool mc_manual_thrust_mode = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
						   && _vehicle_control_mode.flag_control_manual_enabled
						   && !_vehicle_control_mode.flag_control_climb_rate_enabled;
		const bool commanded_by_rc = (calling_reason == arm_disarm_reason_t::rc_stick)
					     || (calling_reason == arm_disarm_reason_t::rc_switch)
					     || (calling_reason == arm_disarm_reason_t::rc_button);

		if (!landed && !(mc_manual_thrust_mode && commanded_by_rc)) {
			if (calling_reason != arm_disarm_reason_t::rc_stick) {
				mavlink_log_critical(&_mavlink_log_pub, "Disarming denied! Not landed\t");
				events::send(events::ID("commander_disarming_denied_not_landed"),
				{events::Log::Critical, events::LogInternal::Info},
				"Disarming denied, not landed");
			}

			return TRANSITION_DENIED;
		}
	}

	transition_result_t arming_res = _arm_state_machine.arming_state_transition(_vehicle_status, _vehicle_control_mode,
					 _safety.isButtonAvailable(), _safety.isSafetyOff(),
					 vehicle_status_s::ARMING_STATE_STANDBY, _actuator_armed, false,
					 &_mavlink_log_pub, _vehicle_status_flags,
					 calling_reason);

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(&_mavlink_log_pub, "Disarmed by %s\t", arm_disarm_reason_str(calling_reason));
		events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_disarmed_by"), events::Log::Info,
				"Disarmed by {1}", calling_reason);

		if (_param_com_force_safety.get()) {
			_safety.activateSafety();
		}

		_status_changed = true;

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

Commander::Commander() :
	ModuleParams(nullptr),
	_failure_detector(this)
{
	_vehicle_land_detected.landed = true;

	_vehicle_status.system_id = 1;
	_vehicle_status.component_id = 1;

	_vehicle_status.system_type = 0;
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;

	// We want to accept RC inputs as default
	_vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	_vehicle_status.nav_state_timestamp = hrt_absolute_time();

	/* mark all signals lost as long as they haven't been found */
	_vehicle_status.rc_signal_lost = true;
	_vehicle_status.data_link_lost = true;

	_vehicle_status_flags.offboard_control_signal_lost = true;

	_vehicle_status_flags.power_input_valid = true;

	// default for vtol is rotary wing
	_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

	_vehicle_gps_position_valid.set_hysteresis_time_from(false, GPS_VALID_TIME);
	_vehicle_gps_position_valid.set_hysteresis_time_from(true, 0);

	_param_mav_comp_id = param_find("MAV_COMP_ID");
	_param_mav_sys_id = param_find("MAV_SYS_ID");
	_param_mav_type = param_find("MAV_TYPE");
	_param_rc_map_fltmode = param_find("RC_MAP_FLTMODE");

	updateParameters();

	// run preflight immediately to find all relevant parameters, but don't report
	PreFlightCheck::preflightCheck(&_mavlink_log_pub, _vehicle_status, _vehicle_status_flags, _vehicle_control_mode,
				       false, // report_failures
				       false, // safety_buttton_available not known
				       false); // safety_off not known
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

			// Check if a mode switch had been requested
			if ((((uint32_t)cmd.param2) & 1) > 0) {
				transition_result_t main_ret = main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LOITER,
							       _vehicle_status_flags, _commander_state);

				if ((main_ret != TRANSITION_DENIED)) {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					mavlink_log_critical(&_mavlink_log_pub, "Reposition command rejected\t");
					/* EVENT
					 * @description Check for a valid position estimate
					 */
					events::send(events::ID("commander_reposition_rejected"),
					{events::Log::Error, events::LogInternal::Info},
					"Reposition command rejected");
				}

			} else {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t)cmd.param1;
			uint8_t custom_main_mode = (uint8_t)cmd.param2;
			uint8_t custom_sub_mode = (uint8_t)cmd.param3;

			uint8_t desired_main_state = commander_state_s::MAIN_STATE_MAX;
			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					desired_main_state = commander_state_s::MAIN_STATE_MANUAL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					desired_main_state = commander_state_s::MAIN_STATE_ALTCTL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					desired_main_state = commander_state_s::MAIN_STATE_POSCTL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					if (custom_sub_mode > 0) {

						switch (custom_sub_mode) {
						case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_LOITER;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_RTL;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_TAKEOFF;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_PRECLAND;
							break;

						default:
							main_ret = TRANSITION_DENIED;
							mavlink_log_critical(&_mavlink_log_pub, "Unsupported auto mode\t");
							events::send(events::ID("commander_unsupported_auto_mode"), events::Log::Error,
								     "Unsupported auto mode");
							break;
						}

					} else {
						desired_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					desired_main_state = commander_state_s::MAIN_STATE_ACRO;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
					desired_main_state = commander_state_s::MAIN_STATE_STAB;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					desired_main_state = commander_state_s::MAIN_STATE_OFFBOARD;
				}

			} else {
				/* use base mode */
				if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
					desired_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;

				} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
						desired_main_state = commander_state_s::MAIN_STATE_POSCTL;

					} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
						desired_main_state = commander_state_s::MAIN_STATE_STAB;

					} else {
						desired_main_state = commander_state_s::MAIN_STATE_MANUAL;
					}
				}
			}

			if (desired_main_state != commander_state_s::MAIN_STATE_MAX) {
				main_ret = main_state_transition(_vehicle_status, desired_main_state, _vehicle_status_flags, _commander_state);
			}

			if (main_ret != TRANSITION_DENIED) {
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
				const bool cmd_from_io = (static_cast<int>(roundf(cmd.param3)) == 1234);

				// Flick to in-air restore first if this comes from an onboard system and from IO
				if (!forced && cmd_from_io
				    && (cmd.source_system == _vehicle_status.system_id)
				    && (cmd.source_component == _vehicle_status.component_id)
				    && (arming_action == vehicle_command_s::ARMING_ACTION_ARM)) {
					// TODO: replace with a proper allowed transition
					_arm_state_machine.forceArmState(vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE);
				}

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

					/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
					if ((arming_action == vehicle_command_s::ARMING_ACTION_ARM) && (arming_res == TRANSITION_CHANGED)
					    && (hrt_absolute_time() > (_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL))
					    && (_param_com_home_en.get() && !_home_position_pub.get().manual_home)) {
						set_home_position();
					}
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION: {
			if (cmd.param1 > 1.5f) {
				// Test termination command triggers lockdown but not actual termination.
				if (!_lockdown_triggered) {
					_actuator_armed.lockdown = true;
					_lockdown_triggered = true;
					PX4_WARN("forcing lockdown (motors off)");
				}

			} else if (cmd.param1 > 0.5f) {
				// Trigger real termination.
				if (!_flight_termination_triggered) {
					_actuator_armed.force_failsafe = true;
					_flight_termination_triggered = true;
					PX4_WARN("forcing failsafe (termination)");
					send_parachute_command();
				}

			} else {
				_actuator_armed.force_failsafe = false;
				_actuator_armed.lockdown = false;

				_lockdown_triggered = false;
				_flight_termination_triggered = false;

				PX4_WARN("disabling failsafe and lockdown");
			}

			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME: {
			if (_param_com_home_en.get()) {
				bool use_current = cmd.param1 > 0.5f;

				if (use_current) {
					/* use current position */
					if (set_home_position()) {
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
						const vehicle_local_position_s &local_pos = _local_position_sub.get();

						if (local_pos.xy_global && local_pos.z_global) {
							/* use specified position */
							home_position_s home{};
							home.timestamp = hrt_absolute_time();

							fillGlobalHomePos(home, lat, lon, alt);

							home.manual_home = true;

							// update local projection reference including altitude
							MapProjection ref_pos{local_pos.ref_lat, local_pos.ref_lon};
							float home_x;
							float home_y;
							ref_pos.project(lat, lon, home_x, home_y);
							const float home_z = -(alt - local_pos.ref_alt);
							fillLocalHomePos(home, home_x, home_y, home_z, yaw);

							/* mark home position as set */
							_vehicle_status_flags.home_position_valid = _home_position_pub.update(home);

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
			if (TRANSITION_CHANGED == main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_RTL,
					_vehicle_status_flags,
					_commander_state)) {
				mavlink_log_info(&_mavlink_log_pub, "Returning to launch\t");
				events::send(events::ID("commander_rtl"), events::Log::Info, "Returning to launch");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Return to launch denied\t");
				/* EVENT
				 * @description Check for a valid position estimate
				 */
				events::send(events::ID("commander_rtl_denied"), {events::Log::Critical, events::LogInternal::Info},
					     "Return to launch denied");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
			/* ok, home set, use it to take off */
			if (TRANSITION_CHANGED == main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_TAKEOFF,
					_vehicle_status_flags,
					_commander_state)) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else if (_commander_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Takeoff denied! Please disarm and retry\t");
				/* EVENT
				 * @description Check for a valid position estimate
				 */
				events::send(events::ID("commander_takeoff_denied"), {events::Log::Critical, events::LogInternal::Info},
					     "Takeoff denied! Please disarm and retry");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:

		/* ok, home set, use it to take off */
		if (TRANSITION_CHANGED == main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_VTOL_TAKEOFF,
				_vehicle_status_flags,
				_commander_state)) {
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else if (_commander_state.main_state == commander_state_s::MAIN_STATE_AUTO_VTOL_TAKEOFF) {
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else {
			mavlink_log_critical(&_mavlink_log_pub, "VTOL Takeoff denied! Please disarm and retry");
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
		}

		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND: {
			if (TRANSITION_DENIED != main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LAND,
					_vehicle_status_flags,
					_commander_state)) {
				mavlink_log_info(&_mavlink_log_pub, "Landing at current position\t");
				events::send(events::ID("commander_landing_current_pos"), events::Log::Info,
					     "Landing at current position");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Landing denied! Please land manually\t");
				/* EVENT
				 * @description Check for a valid position estimate
				 */
				events::send(events::ID("commander_landing_current_pos_denied"), {events::Log::Critical, events::LogInternal::Info},
					     "Landing denied! Please land manually");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND: {
			if (TRANSITION_DENIED != main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_PRECLAND,
					_vehicle_status_flags,
					_commander_state)) {
				mavlink_log_info(&_mavlink_log_pub, "Precision landing\t");
				events::send(events::ID("commander_landing_prec_land"), events::Log::Info,
					     "Landing using precision landing");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Precision landing denied! Please land manually\t");
				/* EVENT
				 * @description Check for a valid position estimate
				 */
				events::send(events::ID("commander_landing_prec_land_denied"), {events::Log::Critical, events::LogInternal::Info},
					     "Precision landing denied! Please land manually");
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {

			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;

			// check if current mission and first item are valid
			if (_vehicle_status_flags.auto_mission_available) {

				// requested first mission item valid
				if (PX4_ISFINITE(cmd.param1) && (cmd.param1 >= -1) && (cmd.param1 < _mission_result_sub.get().seq_total)) {

					// switch to AUTO_MISSION and ARM
					if ((TRANSITION_DENIED != main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_MISSION,
							_vehicle_status_flags,
							_commander_state))
					    && (TRANSITION_DENIED != arm(arm_disarm_reason_t::mission_start))) {

						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
						mavlink_log_critical(&_mavlink_log_pub, "Mission start denied\t");
						/* EVENT
						 * @description Check for a valid position estimate
						 */
						events::send(events::ID("commander_mission_start_denied"), {events::Log::Critical, events::LogInternal::Info},
							     "Mission start denied");
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
			if (_high_latency_datalink_heartbeat > _boot_timestamp) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
				mavlink_log_critical(&_mavlink_log_pub, "Control high latency failed! Telemetry unavailable\t");
				events::send(events::ID("commander_ctrl_high_latency_failed"), {events::Log::Critical, events::LogInternal::Info},
					     "Control high latency failed! Telemetry unavailable");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:

		transition_result_t main_ret;

		if (_vehicle_status.in_transition_mode) {
			main_ret = TRANSITION_DENIED;

		} else if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			// for fixed wings the behavior of orbit is the same as loiter
			main_ret = main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LOITER,
							 _vehicle_status_flags, _commander_state);

		} else {
			// Switch to orbit state and let the orbit task handle the command further
			main_ret = main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_ORBIT, _vehicle_status_flags,
							 _commander_state);
		}

		if ((main_ret != TRANSITION_DENIED)) {
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else {
			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			mavlink_log_critical(&_mavlink_log_pub, "Orbit command rejected");
		}

		break;

	case vehicle_command_s::VEHICLE_CMD_DO_MOTOR_TEST:
		cmd_result = handle_command_motor_test(cmd);
		break;

	case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
		cmd_result = handle_command_actuator_test(cmd);
		break;

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {

			const int param1 = cmd.param1;

			if (param1 == 0) {
				// 0: Do nothing for autopilot
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 1) && shutdown_if_allowed() && (px4_reboot_request(false, 400_ms) == 0)) {
				// 1: Reboot autopilot
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_RESET

#if defined(BOARD_HAS_POWER_CONTROL)

			} else if ((param1 == 2) && shutdown_if_allowed() && (px4_shutdown_request(400_ms) == 0)) {
				// 2: Shutdown autopilot
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // BOARD_HAS_POWER_CONTROL

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 3) && shutdown_if_allowed() && (px4_reboot_request(true, 400_ms) == 0)) {
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

			if (_arm_state_machine.isArmed() || _arm_state_machine.isShutdown() || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {

				/* try to go to INIT/PREFLIGHT arming state */
				if (TRANSITION_DENIED == _arm_state_machine.arming_state_transition(_vehicle_status, _vehicle_control_mode,
						_safety.isButtonAvailable(), _safety.isSafetyOff(),
						vehicle_status_s::ARMING_STATE_INIT, _actuator_armed,
						false /* fRunPreArmChecks */, &_mavlink_log_pub, _vehicle_status_flags,
						(cmd.from_external ? arm_disarm_reason_t::command_external : arm_disarm_reason_t::command_internal))
				   ) {

					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
					break;

				}

				if ((int)(cmd.param1) == 1) {
					/* gyro calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::GyroCalibration);

				} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					/* temperature calibration: handled in events module */
					break;

				} else if ((int)(cmd.param2) == 1) {
					/* magnetometer calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::MagCalibration);

				} else if ((int)(cmd.param3) == 1) {
					/* baro calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::BaroCalibration);

				} else if ((int)(cmd.param4) == 1) {
					/* RC calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					/* disable RC control input completely */
					_vehicle_status_flags.rc_calibration_in_progress = true;
					mavlink_log_info(&_mavlink_log_pub, "Calibration: Disabling RC input\t");
					events::send(events::ID("commander_calib_rc_off"), events::Log::Info,
						     "Calibration: Disabling RC input");

				} else if ((int)(cmd.param4) == 2) {
					/* RC trim calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::RCTrimCalibration);

				} else if ((int)(cmd.param5) == 1) {
					/* accelerometer calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibration);
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::LevelCalibration);

				} else if ((int)(cmd.param5) == 4) {
					// accelerometer quick calibration
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibrationQuick);

				} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
					// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
					/* airspeed calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status_flags.calibration_enabled = true;
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
							_vehicle_status_flags.calibration_enabled = true;
							_actuator_armed.in_esc_calibration_mode = true;
							_worker_thread.startTask(WorkerThread::Request::ESCCalibration);
						}

					} else {
						answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
					}

				} else if ((int)(cmd.param4) == 0) {
					/* RC calibration ended - have we been in one worth confirming? */
					if (_vehicle_status_flags.rc_calibration_in_progress) {
						/* enable RC control input */
						_vehicle_status_flags.rc_calibration_in_progress = false;
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
			if (_arm_state_machine.isArmed() || _arm_state_machine.isShutdown() || _worker_thread.isBusy()) {

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

				_vehicle_status_flags.calibration_enabled = true;
				_worker_thread.setMagQuickData(heading_radians, latitude, longitude);
				_worker_thread.startTask(WorkerThread::Request::MagCalibrationQuick);
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

			if (_arm_state_machine.isArmed() || _arm_state_machine.isShutdown() || _worker_thread.isBusy()) {

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
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR:
	case vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE:
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

unsigned
Commander::handle_command_motor_test(const vehicle_command_s &cmd)
{
	if (_arm_state_machine.isArmed() || (_safety.isButtonAvailable() && !_safety.isSafetyOff())) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	}

	if (_param_com_mot_test_en.get() != 1) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	}

	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = (int)(cmd.param1 + 0.5f) - 1;

	int throttle_type = (int)(cmd.param2 + 0.5f);

	if (throttle_type != 0) { // 0: MOTOR_TEST_THROTTLE_PERCENT
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	}

	int motor_count = (int)(cmd.param5 + 0.5);

	if (motor_count > 1) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	}

	test_motor.action = test_motor_s::ACTION_RUN;
	test_motor.value = math::constrain(cmd.param3 / 100.f, 0.f, 1.f);

	if (test_motor.value < FLT_EPSILON) {
		// the message spec is not clear on whether 0 means stop, but it should be closer to what a user expects
		test_motor.value = -1.f;
	}

	test_motor.timeout_ms = (int)(cmd.param4 * 1000.f + 0.5f);

	// enforce a timeout and a maximum limit
	if (test_motor.timeout_ms == 0 || test_motor.timeout_ms > 3000) {
		test_motor.timeout_ms = 3000;
	}

	test_motor.driver_instance = 0; // the mavlink command does not allow to specify the instance, so set to 0 for now
	_test_motor_pub.publish(test_motor);

	return vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
}

unsigned
Commander::handle_command_actuator_test(const vehicle_command_s &cmd)
{
	if (_arm_state_machine.isArmed() || (_safety.isButtonAvailable() && !_safety.isSafetyOff())) {
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
	if (_vehicle_status_flags.rc_calibration_in_progress
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
		if (_arm_state_machine.isArmed()) {
			disarm(arm_disarm_reason);

		} else {
			arm(arm_disarm_reason);
		}

		break;

	case action_request_s::ACTION_UNKILL:
		if (arm_disarm_reason == arm_disarm_reason_t::rc_switch && _actuator_armed.manual_lockdown) {
			mavlink_log_info(&_mavlink_log_pub, "Kill-switch disengaged\t");
			events::send(events::ID("commander_kill_sw_disengaged"), events::Log::Info, "Kill-switch disengaged");
			_status_changed = true;
			_actuator_armed.manual_lockdown = false;
		}

		break;

	case action_request_s::ACTION_KILL:
		if (arm_disarm_reason == arm_disarm_reason_t::rc_switch && !_actuator_armed.manual_lockdown) {
			const char kill_switch_string[] = "Kill-switch engaged\t";
			events::LogLevels log_levels{events::Log::Info};

			if (_vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, kill_switch_string);

			} else {
				mavlink_log_critical(&_mavlink_log_pub, kill_switch_string);
				log_levels.external = events::Log::Critical;
			}

			events::send(events::ID("commander_kill_sw_engaged"), log_levels, "Kill-switch engaged");

			_status_changed = true;
			_actuator_armed.manual_lockdown = true;
			send_parachute_command();
		}

		break;

	case action_request_s::ACTION_SWITCH_MODE:

		// if there's never been a mode change force RC switch as initial state
		if (action_request.source == action_request_s::SOURCE_RC_MODE_SLOT
		    && !_arm_state_machine.isArmed() && (_commander_state.main_state_changes == 0)
		    && (action_request.mode == commander_state_s::MAIN_STATE_ALTCTL
			|| action_request.mode == commander_state_s::MAIN_STATE_POSCTL)) {
			_commander_state.main_state = action_request.mode;
			_commander_state.main_state_changes++;
		}

		int ret = main_state_transition(_vehicle_status, action_request.mode, _vehicle_status_flags, _commander_state);

		if (ret == transition_result_t::TRANSITION_DENIED) {
			print_reject_mode(action_request.mode);
		}

		break;
	}
}

bool
Commander::hasMovedFromCurrentHomeLocation()
{
	float home_dist_xy = -1.f;
	float home_dist_z = -1.f;
	float eph = 0.f;
	float epv = 0.f;

	if (_home_position_pub.get().valid_lpos && _local_position_sub.get().xy_valid && _local_position_sub.get().z_valid) {
		mavlink_wpm_distance_to_point_local(_home_position_pub.get().x, _home_position_pub.get().y, _home_position_pub.get().z,
						    _local_position_sub.get().x, _local_position_sub.get().y, _local_position_sub.get().z,
						    &home_dist_xy, &home_dist_z);

		eph = _local_position_sub.get().eph;
		epv = _local_position_sub.get().epv;

	} else if (_home_position_pub.get().valid_hpos && _home_position_pub.get().valid_alt) {
		if (_vehicle_status_flags.global_position_valid) {
			const vehicle_global_position_s &gpos = _global_position_sub.get();

			get_distance_to_point_global_wgs84(_home_position_pub.get().lat, _home_position_pub.get().lon,
							   _home_position_pub.get().alt,
							   gpos.lat, gpos.lon, gpos.alt,
							   &home_dist_xy, &home_dist_z);

			eph = gpos.eph;
			epv = gpos.epv;

		} else if (_vehicle_status_flags.gps_position_valid) {
			sensor_gps_s gps;
			_vehicle_gps_position_sub.copy(&gps);
			const double lat = static_cast<double>(gps.lat) * 1e-7;
			const double lon = static_cast<double>(gps.lon) * 1e-7;
			const float alt = static_cast<float>(gps.alt) * 1e-3f;

			get_distance_to_point_global_wgs84(_home_position_pub.get().lat, _home_position_pub.get().lon,
							   _home_position_pub.get().alt,
							   lat, lon, alt,
							   &home_dist_xy, &home_dist_z);

			eph = gps.eph;
			epv = gps.epv;
		}
	}

	return (home_dist_xy > eph * 2.f) || (home_dist_z > epv * 2.f);
}

/**
* @brief This function initializes the home position an altitude of the vehicle. This happens first time we get a good GPS fix and each
*		 time the vehicle is armed with a good GPS fix.
**/
bool
Commander::set_home_position()
{
	bool updated = false;
	home_position_s home{};

	if (_vehicle_status_flags.local_position_valid) {
		// Set home position in local coordinates
		const vehicle_local_position_s &lpos = _local_position_sub.get();
		_heading_reset_counter = lpos.heading_reset_counter; // TODO: should not be here

		fillLocalHomePos(home, lpos);
		updated = true;
	}

	if (_vehicle_status_flags.global_position_valid) {
		// Set home using the global position estimate (fused INS/GNSS)
		const vehicle_global_position_s &gpos = _global_position_sub.get();
		fillGlobalHomePos(home, gpos);
		setHomePosValid();
		updated = true;

	} else if (_vehicle_status_flags.gps_position_valid) {
		// Set home using GNSS position
		sensor_gps_s gps_pos;
		_vehicle_gps_position_sub.copy(&gps_pos);
		const double lat = static_cast<double>(gps_pos.lat) * 1e-7;
		const double lon = static_cast<double>(gps_pos.lon) * 1e-7;
		const float alt = static_cast<float>(gps_pos.alt) * 1e-3f;
		fillGlobalHomePos(home, lat, lon, alt);
		setHomePosValid();
		updated = true;

	} else if (_local_position_sub.get().z_global) {
		// handle special case where we are setting only altitude using local position reference
		// This might be overwritten by altitude from global or GNSS altitude
		home.alt = _local_position_sub.get().ref_alt;
		home.valid_alt = true;

		updated = true;
	}

	if (updated) {
		home.timestamp = hrt_absolute_time();
		home.manual_home = false;
		updated = _home_position_pub.update(home);
	}

	return updated;
}

void
Commander::set_in_air_home_position()
{
	home_position_s home{};
	home = _home_position_pub.get();
	const bool global_home_valid = home.valid_hpos && home.valid_alt;
	const bool local_home_valid = home.valid_lpos;

	if (local_home_valid && !global_home_valid) {
		if (_vehicle_status_flags.local_position_valid && _vehicle_status_flags.global_position_valid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS fused) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();
			const vehicle_global_position_s &gpos = _global_position_sub.get();

			MapProjection ref_pos{gpos.lat, gpos.lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const float home_alt = gpos.alt + home.z;
			fillGlobalHomePos(home, home_lat, home_lon, home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			_home_position_pub.update(home);

		} else if (_vehicle_status_flags.local_position_valid && _vehicle_status_flags.gps_position_valid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS raw) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();
			sensor_gps_s gps;
			_vehicle_gps_position_sub.copy(&gps);

			const double lat = static_cast<double>(gps.lat) * 1e-7;
			const double lon = static_cast<double>(gps.lon) * 1e-7;
			const float alt = static_cast<float>(gps.alt) * 1e-3f;

			MapProjection ref_pos{lat, lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const float home_alt = alt + home.z;
			fillGlobalHomePos(home, home_lat, home_lon, home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			_home_position_pub.update(home);
		}

	} else if (!local_home_valid && global_home_valid) {
		const vehicle_local_position_s &lpos = _local_position_sub.get();

		if (_vehicle_status_flags.local_position_valid && lpos.xy_global && lpos.z_global) {
			// Back-compute x, y and z of home position given the global home position
			// and the global reference of the local frame
			MapProjection ref_pos{lpos.ref_lat, lpos.ref_lon};

			float home_x;
			float home_y;
			ref_pos.project(home.lat, home.lon, home_x, home_y);

			const float home_z = -(home.alt - lpos.ref_alt);
			fillLocalHomePos(home, home_x, home_y, home_z, NAN);

			home.timestamp = hrt_absolute_time();
			_home_position_pub.update(home);
		}

	} else if (!local_home_valid && !global_home_valid) {
		// Home position is not known in any frame, set home at current position
		set_home_position();

	} else {
		// nothing to do
	}
}

void
Commander::fillLocalHomePos(home_position_s &home, const vehicle_local_position_s &lpos) const
{
	fillLocalHomePos(home, lpos.x, lpos.y, lpos.z, lpos.heading);
}

void
Commander::fillLocalHomePos(home_position_s &home, float x, float y, float z, float heading) const
{
	home.x = x;
	home.y = y;
	home.z = z;
	home.valid_lpos = true;

	home.yaw = heading;
}

void Commander::fillGlobalHomePos(home_position_s &home, const vehicle_global_position_s &gpos) const
{
	fillGlobalHomePos(home, gpos.lat, gpos.lon, gpos.alt);
}

void Commander::fillGlobalHomePos(home_position_s &home, double lat, double lon, float alt) const
{
	home.lat = lat;
	home.lon = lon;
	home.valid_hpos = true;
	home.alt = alt;
	home.valid_alt = true;
}

void Commander::setHomePosValid()
{
	// play tune first time we initialize HOME
	if (!_vehicle_status_flags.home_position_valid) {
		tune_home_set(true);
	}

	// mark home position as set
	_vehicle_status_flags.home_position_valid = true;
}

void
Commander::updateHomePositionYaw(float yaw)
{
	if (_param_com_home_en.get()) {
		home_position_s home = _home_position_pub.get();

		home.yaw = yaw;
		home.timestamp = hrt_absolute_time();

		_home_position_pub.update(home);
	}
}

void Commander::updateParameters()
{
	// update parameters from storage
	updateParams();

	get_circuit_breaker_params();

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


	_vehicle_status_flags.avoidance_system_required = _param_com_obs_avoid.get();

	_auto_disarm_killed.set_hysteresis_time_from(false, _param_com_kill_disarm.get() * 1_s);
	_offboard_available.set_hysteresis_time_from(true, _param_com_of_loss_t.get() * 1_s);

	const bool is_rotary = is_rotary_wing(_vehicle_status) || (is_vtol(_vehicle_status)
			       && _vtol_vehicle_status.vehicle_vtol_state != vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	const bool is_fixed = is_fixed_wing(_vehicle_status) || (is_vtol(_vehicle_status)
			      && _vtol_vehicle_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	const bool is_ground = is_ground_rover(_vehicle_status);

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

	// CP_DIST: collision preventation enabled if CP_DIST > 0
	if (is_rotary_wing(_vehicle_status) || is_vtol(_vehicle_status)) {
		if (_param_cp_dist == PARAM_INVALID) {
			_param_cp_dist = param_find("CP_DIST");
		}

		float cp_dist = 0;

		if (_param_cp_dist != PARAM_INVALID && (param_get(_param_cp_dist, &cp_dist) == PX4_OK)) {
			_collision_prevention_enabled = cp_dist > 0.f;
		}
	}

	// _mode_switch_mapped = (RC_MAP_FLTMODE > 0)
	if (_param_rc_map_fltmode != PARAM_INVALID && (param_get(_param_rc_map_fltmode, &value_int32) == PX4_OK)) {
		_mode_switch_mapped = (value_int32 > 0);
	}

}

void
Commander::run()
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

	// initially set to failed
	_last_lpos_fail_time_us = _boot_timestamp;
	_last_gpos_fail_time_us = _boot_timestamp;
	_last_lvel_fail_time_us = _boot_timestamp;

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

			/* update parameters */
			if (!_arm_state_machine.isArmed()) {
				updateParameters();

				_status_changed = true;
			}
		}

		/* Update OA parameter */
		_vehicle_status_flags.avoidance_system_required = _param_com_obs_avoid.get();

#if defined(BOARD_HAS_POWER_CONTROL)

		/* handle power button state */
		if (_power_button_state_sub.updated()) {
			power_button_state_s button_state;

			if (_power_button_state_sub.copy(&button_state)) {
				if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
					if (shutdown_if_allowed() && (px4_shutdown_request() == 0)) {
						while (1) { px4_usleep(1); }
					}
				}
			}
		}

#endif // BOARD_HAS_POWER_CONTROL

		offboard_control_update();

		if (_system_power_sub.updated()) {
			system_power_s system_power{};
			_system_power_sub.copy(&system_power);

			if (hrt_elapsed_time(&system_power.timestamp) < 1_s) {
				if (system_power.servo_valid &&
				    !system_power.brick_valid &&
				    !system_power.usb_connected) {
					/* flying only on servo rail, this is unsafe */
					_vehicle_status_flags.power_input_valid = false;

				} else {
					_vehicle_status_flags.power_input_valid = true;
				}

				_system_power_usb_connected = system_power.usb_connected;
			}
		}

		/* Update land detector */
		if (_vehicle_land_detected_sub.updated()) {
			const bool was_landed = _vehicle_land_detected.landed;
			_vehicle_land_detected_sub.copy(&_vehicle_land_detected);

			// Only take actions if armed
			if (_arm_state_machine.isArmed()) {
				if (!was_landed && _vehicle_land_detected.landed) {
					mavlink_log_info(&_mavlink_log_pub, "Landing detected\t");
					events::send(events::ID("commander_landing_detected"), events::Log::Info, "Landing detected");
					_vehicle_status.takeoff_time = 0;

				} else if (was_landed && !_vehicle_land_detected.landed) {
					mavlink_log_info(&_mavlink_log_pub, "Takeoff detected\t");
					events::send(events::ID("commander_takeoff_detected"), events::Log::Info, "Takeoff detected");
					_vehicle_status.takeoff_time = hrt_absolute_time();
					_have_taken_off_since_arming = true;
				}

				// automatically set or update home position
				if (_param_com_home_en.get() && !_home_position_pub.get().manual_home) {
					// set the home position when taking off, but only if we were previously disarmed
					// and at least 500 ms from commander start spent to avoid setting home on in-air restart
					if (!_vehicle_land_detected.landed && (hrt_elapsed_time(&_boot_timestamp) > INAIR_RESTART_HOLDOFF_INTERVAL)) {
						if (was_landed) {
							set_home_position();

						} else if (_param_com_home_in_air.get()
							   && (!_home_position_pub.get().valid_lpos || !_home_position_pub.get().valid_hpos
							       || !_home_position_pub.get().valid_alt)) {
							set_in_air_home_position();
						}
					}
				}
			}
		}

		/* safety button */
		const bool safety_changed = _safety.safetyButtonHandler();
		_vehicle_status.safety_button_available = _safety.isButtonAvailable();
		_vehicle_status.safety_off = _safety.isSafetyOff();

		if (safety_changed) {
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, _safety.isButtonAvailable(), _safety.isSafetyOff(),
					 _safety.isButtonAvailable(), _vehicle_status);

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

		/* update vtol vehicle status*/
		if (_vtol_vehicle_status_sub.updated()) {
			/* vtol status changed */
			_vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);

			/* Make sure that this is only adjusted if vehicle really is of type vtol */
			if (is_vtol(_vehicle_status)) {

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

				if (_vehicle_status_flags.vtol_transition_failure != _vtol_vehicle_status.vtol_transition_failsafe) {
					_vehicle_status_flags.vtol_transition_failure = _vtol_vehicle_status.vtol_transition_failsafe;
					_status_changed = true;
				}

				const bool should_soft_stop = (_vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

				if (_actuator_armed.soft_stop != should_soft_stop) {
					_actuator_armed.soft_stop = should_soft_stop;
					_status_changed = true;
				}
			}
		}

		if (_esc_status_sub.updated()) {
			/* ESCs status changed */
			esc_status_check();

		} else if (_param_escs_checks_required.get() != 0) {

			if (!_vehicle_status_flags.escs_error) {

				if ((_last_esc_status_updated != 0) && (hrt_elapsed_time(&_last_esc_status_updated) > 700_ms)) {
					/* Detect timeout after first telemetry packet received
					 * Some DShot ESCs are unresponsive for ~550ms during their initialization, so we use a timeout higher than that
					 */

					mavlink_log_critical(&_mavlink_log_pub, "ESCs telemetry timeout\t");
					events::send(events::ID("commander_esc_telemetry_timeout"), events::Log::Critical,
						     "ESCs telemetry timeout");
					_vehicle_status_flags.escs_error = true;

				} else if (_last_esc_status_updated == 0 && hrt_elapsed_time(&_boot_timestamp) > 5000_ms) {
					/* Detect if esc telemetry is not connected after reboot */
					mavlink_log_critical(&_mavlink_log_pub, "ESCs telemetry not connected\t");
					events::send(events::ID("commander_esc_telemetry_not_con"), events::Log::Critical,
						     "ESCs telemetry not connected");
					_vehicle_status_flags.escs_error = true;
				}
			}
		}

		estimator_check();

		// Auto disarm when landed or kill switch engaged
		if (_arm_state_machine.isArmed()) {

			// Check for auto-disarm on landing or pre-flight
			if (_param_com_disarm_land.get() > 0 || _param_com_disarm_preflight.get() > 0) {

				const bool landed_amid_mission = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
								 && !_mission_result_sub.get().finished;

				if (_param_com_disarm_land.get() > 0 && _have_taken_off_since_arming && !landed_amid_mission) {
					_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_land.get() * 1_s);
					_auto_disarm_landed.set_state_and_update(_vehicle_land_detected.landed, hrt_absolute_time());

				} else if (_param_com_disarm_preflight.get() > 0 && !_have_taken_off_since_arming) {
					_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);
					_auto_disarm_landed.set_state_and_update(true, hrt_absolute_time());
				}

				if (_auto_disarm_landed.get_state()) {
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

		if (_geofence_warning_action_on
		    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
		    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_LOITER
		    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {

			// reset flag again when we switched out of it
			_geofence_warning_action_on = false;
		}

		if (_battery_status_subs.updated()) {
			battery_status_check();
		}

		/* If in INIT state, try to proceed to STANDBY state */
		if (!_vehicle_status_flags.calibration_enabled && _arm_state_machine.isInit()) {

			_arm_state_machine.arming_state_transition(_vehicle_status, _vehicle_control_mode,
					_safety.isButtonAvailable(), _safety.isSafetyOff(),
					vehicle_status_s::ARMING_STATE_STANDBY, _actuator_armed,
					true /* fRunPreArmChecks */, &_mavlink_log_pub, _vehicle_status_flags,
					arm_disarm_reason_t::transition_to_standby);
		}

		/* start mission result check */
		if (_mission_result_sub.updated()) {
			const mission_result_s &mission_result = _mission_result_sub.get();

			const auto prev_mission_instance_count = mission_result.instance_count;
			_mission_result_sub.update();

			// if mission_result is valid for the current mission
			const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp)
						       && (mission_result.instance_count > 0);

			_vehicle_status_flags.auto_mission_available = mission_result_ok && mission_result.valid;

			if (mission_result_ok) {
				if (_vehicle_status.mission_failure != mission_result.failure) {
					_vehicle_status.mission_failure = mission_result.failure;
					_status_changed = true;

					if (_vehicle_status.mission_failure) {
						// navigator sends out the exact reason
						mavlink_log_critical(&_mavlink_log_pub, "Mission cannot be completed\t");
						events::send(events::ID("commander_mission_cannot_be_completed"), {events::Log::Critical, events::LogInternal::Info},
							     "Mission cannot be completed");
					}
				}

				/* Only evaluate mission state if home is set */
				if (_vehicle_status_flags.home_position_valid &&
				    (prev_mission_instance_count != mission_result.instance_count)) {

					if (!_vehicle_status_flags.auto_mission_available) {
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

			// Transition main state to loiter or auto-mission after takeoff is completed.
			if (_arm_state_machine.isArmed() && !_vehicle_land_detected.landed
			    && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
				_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF)
			    && (mission_result.timestamp >= _vehicle_status.nav_state_timestamp)
			    && mission_result.finished) {

				if ((_param_takeoff_finished_action.get() == 1) && _vehicle_status_flags.auto_mission_available) {
					main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_MISSION, _vehicle_status_flags,
							      _commander_state);

				} else {
					main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _vehicle_status_flags,
							      _commander_state);
				}
			}
		}

		/* start geofence result check */
		if (_geofence_result_sub.update(&_geofence_result)) {
			_vehicle_status.geofence_violated = _geofence_result.geofence_violated;
		}

		const bool in_low_battery_failsafe_delay = _battery_failsafe_timestamp != 0;

		// Geofence actions
		if (_arm_state_machine.isArmed()
		    && (_geofence_result.geofence_action != geofence_result_s::GF_ACTION_NONE)
		    && !in_low_battery_failsafe_delay) {

			// check for geofence violation transition
			if (_geofence_result.geofence_violated && !_geofence_violated_prev) {

				switch (_geofence_result.geofence_action) {
				case (geofence_result_s::GF_ACTION_NONE) : {
						// do nothing
						break;
					}

				case (geofence_result_s::GF_ACTION_WARN) : {
						// do nothing, mavlink critical messages are sent by navigator
						break;
					}

				case (geofence_result_s::GF_ACTION_LOITER) : {
						if (TRANSITION_CHANGED == main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LOITER,
								_vehicle_status_flags,
								_commander_state)) {
							_geofence_loiter_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_RTL) : {
						if (TRANSITION_CHANGED == main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_RTL,
								_vehicle_status_flags,
								_commander_state)) {
							_geofence_rtl_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_LAND) : {
						if (TRANSITION_CHANGED == main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LAND,
								_vehicle_status_flags,
								_commander_state)) {
							_geofence_land_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_TERMINATE) : {
						PX4_WARN("Flight termination because of geofence");

						if (!_flight_termination_triggered && !_lockdown_triggered) {
							_flight_termination_triggered = true;
							mavlink_log_critical(&_mavlink_log_pub, "Geofence violation! Flight terminated\t");
							events::send(events::ID("commander_geofence_termination"), {events::Log::Alert, events::LogInternal::Warning},
								     "Geofence violation! Flight terminated");
							_actuator_armed.force_failsafe = true;
							_status_changed = true;
							send_parachute_command();
						}

						break;
					}
				}
			}

			_geofence_violated_prev = _geofence_result.geofence_violated;

			// reset if no longer in LOITER or if manually switched to LOITER
			const bool in_loiter_mode = _commander_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER;

			if (!in_loiter_mode) {
				_geofence_loiter_on = false;
			}


			// reset if no longer in RTL or if manually switched to RTL
			const bool in_rtl_mode = _commander_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL;

			if (!in_rtl_mode) {
				_geofence_rtl_on = false;
			}

			// reset if no longer in LAND or if manually switched to LAND
			const bool in_land_mode = _commander_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND;

			if (!in_land_mode) {
				_geofence_land_on = false;
			}

			_geofence_warning_action_on = _geofence_warning_action_on || (_geofence_loiter_on || _geofence_rtl_on
						      || _geofence_land_on);

		} else {
			// No geofence checks, reset flags
			_geofence_loiter_on = false;
			_geofence_rtl_on = false;
			_geofence_land_on = false;
			_geofence_warning_action_on = false;
			_geofence_violated_prev = false;
		}

		/* Check for mission flight termination */
		if (_arm_state_machine.isArmed() && _mission_result_sub.get().flight_termination &&
		    !_vehicle_status_flags.circuit_breaker_flight_termination_disabled) {


			if (!_flight_termination_triggered && !_lockdown_triggered) {
				// navigator only requests flight termination on GPS failure
				mavlink_log_critical(&_mavlink_log_pub, "GPS failure: Flight terminated\t");
				events::send(events::ID("commander_mission_termination"), {events::Log::Alert, events::LogInternal::Warning},
					     "GPS failure: Flight terminated");
				_flight_termination_triggered = true;
				_actuator_armed.force_failsafe = true;
				_status_changed = true;
				send_parachute_command();
			}

			if (hrt_elapsed_time(&_last_termination_message_sent) > 4_s) {
				_last_termination_message_sent = hrt_absolute_time();
				mavlink_log_critical(&_mavlink_log_pub, "Flight termination active\t");
				events::send(events::ID("commander_mission_termination_active"), {events::Log::Alert, events::LogInternal::Warning},
					     "Flight termination active");
			}
		}

		manual_control_check();

		// data link checks which update the status
		data_link_check();

		avoidance_check();

		/* check if we are disarmed and there is a better mode to wait in */
		if (!_arm_state_machine.isArmed()) {
			/* if there is no radio control but GPS lock the user might want to fly using
			 * just a tablet. Since the RC will force its mode switch setting on connecting
			 * we can as well just wait in a hold mode which enables tablet control.
			 */
			if (_vehicle_status.rc_signal_lost && (_commander_state.main_state_changes == 0)
			    && _vehicle_status_flags.global_position_valid) {

				main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _vehicle_status_flags,
						      _commander_state);
			}
		}

		/* handle commands last, as the system needs to be updated to handle them */
		if (_vehicle_command_sub.updated()) {
			/* got command */
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

		/* Check for failure detector status */
		if (_failure_detector.update(_vehicle_status, _vehicle_control_mode)) {
			const bool motor_failure_changed = ((_vehicle_status.failure_detector_status & vehicle_status_s::FAILURE_MOTOR) > 0) !=
							   _failure_detector.getStatus().flags.motor;
			_vehicle_status.failure_detector_status = _failure_detector.getStatus().value;
			auto fd_status_flags = _failure_detector.getStatusFlags();
			_status_changed = true;

			if (_arm_state_machine.isArmed()) {
				if (fd_status_flags.arm_escs) {
					// Checks have to pass within the spool up time
					if (hrt_elapsed_time(&_vehicle_status.armed_time) < _param_com_spoolup_time.get() * 1_s) {
						disarm(arm_disarm_reason_t::failure_detector);
						mavlink_log_critical(&_mavlink_log_pub, "ESCs did not respond to arm request\t");
						events::send(events::ID("commander_fd_escs_not_arming"), events::Log::Critical, "ESCs did not respond to arm request");
					}
				}

				if (fd_status_flags.roll || fd_status_flags.pitch || fd_status_flags.alt || fd_status_flags.ext) {
					const bool is_right_after_takeoff = hrt_elapsed_time(&_vehicle_status.takeoff_time) <
									    (1_s * _param_com_lkdown_tko.get());

					if (is_right_after_takeoff && !_lockdown_triggered) {
						// This handles the case where something fails during the early takeoff phase
						_actuator_armed.lockdown = true;
						_lockdown_triggered = true;
						mavlink_log_emergency(&_mavlink_log_pub, "Critical failure detected: lockdown\t");
						/* EVENT
						 * @description
						 * When a critical failure is detected right after takeoff, the system turns off the motors.
						 * Failures include an exceeding tilt angle, altitude failure or an external failure trigger.
						 *
						 * <profile name="dev">
						 * This can be configured with the parameter <param>COM_LKDOWN_TKO</param>.
						 * </profile>
						 */
						events::send(events::ID("commander_fd_lockdown"), {events::Log::Emergency, events::LogInternal::Warning},
							     "Critical failure detected: lockdown");

					} else if (!_vehicle_status_flags.circuit_breaker_flight_termination_disabled &&
						   !_flight_termination_triggered && !_lockdown_triggered) {

						_actuator_armed.force_failsafe = true;
						_flight_termination_triggered = true;
						mavlink_log_emergency(&_mavlink_log_pub, "Critical failure detected: terminate flight\t");
						/* EVENT
						 * @description
						 * Critical failures include an exceeding tilt angle, altitude failure or an external failure trigger.
						 *
						 * <profile name="dev">
						 * Flight termination can be disabled with the parameter <param>CBRK_FLIGHTTERM</param>.
						 * </profile>
						 */
						events::send(events::ID("commander_fd_terminate"), {events::Log::Emergency, events::LogInternal::Warning},
							     "Critical failure detected: terminate flight");
						send_parachute_command();
					}
				}

				if (fd_status_flags.imbalanced_prop
				    && !_imbalanced_propeller_check_triggered) {
					_status_changed = true;
					_imbalanced_propeller_check_triggered = true;
					imbalanced_prop_failsafe(&_mavlink_log_pub, _vehicle_status, _vehicle_status_flags, &_commander_state,
								 (imbalanced_propeller_action_t)_param_com_imb_prop_act.get());
				}
			}

			// One-time actions based on motor failure
			if (motor_failure_changed) {
				if (fd_status_flags.motor) {
					mavlink_log_critical(&_mavlink_log_pub, "Motor failure detected\t");
					events::send(events::ID("commander_motor_failure"), events::Log::Emergency,
						     "Motor failure! Land immediately");
					set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, true, true, false, _vehicle_status);

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Motor recovered\t");
					events::send(events::ID("commander_motor_recovered"), events::Log::Warning,
						     "Motor recovered, landing still advised");
					set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, true, true, true, _vehicle_status);
				}
			}

			if (fd_status_flags.motor) {
				switch ((ActuatorFailureActions)_param_com_actuator_failure_act.get()) {
				case ActuatorFailureActions::AUTO_LOITER:
					mavlink_log_critical(&_mavlink_log_pub, "Loitering due to actuator failure\t");
					events::send(events::ID("commander_act_failure_loiter"), events::Log::Warning,
						     "Loitering due to actuator failure");
					main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_POSCTL, _vehicle_status_flags, _commander_state);
					_status_changed = true;
					break;

				case ActuatorFailureActions::AUTO_LAND:
					mavlink_log_critical(&_mavlink_log_pub, "Landing due to actuator failure\t");
					events::send(events::ID("commander_act_failure_land"), events::Log::Warning,
						     "Landing due to actuator failure");
					main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LAND, _vehicle_status_flags,
							      _commander_state);
					_status_changed = true;
					break;

				case ActuatorFailureActions::AUTO_RTL:
					mavlink_log_critical(&_mavlink_log_pub, "Returning home due to actuator failure\t");
					events::send(events::ID("commander_act_failure_rtl"), events::Log::Warning,
						     "Returning home due to actuator failure");
					main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_RTL, _vehicle_status_flags, _commander_state);
					_status_changed = true;
					break;

				case ActuatorFailureActions::TERMINATE:
					if (!_actuator_armed.manual_lockdown) {
						mavlink_log_critical(&_mavlink_log_pub, "Flight termination due to actuator failure\t");
						events::send(events::ID("commander_act_failure_term"), events::Log::Warning,
							     "Flight termination due to actuator failure");
						_status_changed = true;
						_actuator_armed.manual_lockdown = true;
						send_parachute_command();
					}

					break;

				default:
					// nothing to do here
					break;
				}

			}
		}

		// Check wind speed actions if either high wind warning or high wind RTL is enabled
		if ((_param_com_wind_warn.get() > FLT_EPSILON || _param_com_wind_max.get() > FLT_EPSILON)
		    && !_vehicle_land_detected.landed) {
			checkWindSpeedThresholds();
		}

		_vehicle_status_flags.flight_terminated = _actuator_armed.force_failsafe || _actuator_armed.manual_lockdown;

		/* Get current timestamp */
		const hrt_abstime now = hrt_absolute_time();

		// Trigger RTL if flight time is larger than max flight time specified in COM_FLT_TIME_MAX.
		// The user is not able to override once above threshold, except for triggering Land.
		if (!_vehicle_land_detected.landed
		    && _param_com_flt_time_max.get() > FLT_EPSILON
		    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
		    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND
		    && (now - _vehicle_status.takeoff_time) > (1_s * _param_com_flt_time_max.get())) {
			main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_RTL, _vehicle_status_flags, _commander_state);
			_status_changed = true;
			mavlink_log_critical(&_mavlink_log_pub, "Maximum flight time reached, abort operation and RTL");
			/* EVENT
			* @description
			* Maximal flight time reached, return to launch.
			*/
			events::send(events::ID("commander_max_flight_time_rtl"), {events::Log::Critical, events::LogInternal::Warning},
				     "Maximum flight time reached, abort operation and RTL");
		}

		// automatically set or update home position
		if (_param_com_home_en.get() && !_home_position_pub.get().manual_home) {
			if (!_arm_state_machine.isArmed() && _vehicle_land_detected.landed) {
				const bool can_set_home_lpos_first_time = (!_home_position_pub.get().valid_lpos
						&& _vehicle_status_flags.local_position_valid);
				const bool can_set_home_gpos_first_time = ((!_home_position_pub.get().valid_hpos || !_home_position_pub.get().valid_alt)
						&& (_vehicle_status_flags.global_position_valid || _vehicle_status_flags.gps_position_valid));
				const bool can_set_home_alt_first_time = (!_home_position_pub.get().valid_alt && _local_position_sub.get().z_global);

				if (can_set_home_lpos_first_time
				    || can_set_home_gpos_first_time
				    || can_set_home_alt_first_time
				    || hasMovedFromCurrentHomeLocation()) {
					set_home_position();
				}
			}
		}

		// check for arming state changes
		if (_was_armed != _arm_state_machine.isArmed()) {
			_status_changed = true;
		}

		if (!_was_armed && _arm_state_machine.isArmed() && !_vehicle_land_detected.landed) {
			_have_taken_off_since_arming = true;
		}

		if (_was_armed && !_arm_state_machine.isArmed()) {
			const int32_t flight_uuid = _param_flight_uuid.get() + 1;
			_param_flight_uuid.set(flight_uuid);
			_param_flight_uuid.commit_no_notification();

			_last_disarmed_timestamp = hrt_absolute_time();

			// Switch back to Hold mode after autonomous landing
			if (_vehicle_control_mode.flag_control_auto_enabled) {
				main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LOITER,
						      _vehicle_status_flags, _commander_state);
			}
		}

		if (!_arm_state_machine.isArmed()) {
			/* Reset the flag if disarmed. */
			_have_taken_off_since_arming = false;
			_imbalanced_propeller_check_triggered = false;
		}

		/* now set navigation state according to failsafe and main state */
		bool nav_state_changed = set_nav_state(_vehicle_status,
						       _actuator_armed,
						       _commander_state,
						       &_mavlink_log_pub,
						       static_cast<link_loss_actions_t>(_param_nav_dll_act.get()),
						       _mission_result_sub.get().finished,
						       _mission_result_sub.get().stay_in_failsafe,
						       _vehicle_status_flags,
						       _vehicle_land_detected.landed,
						       static_cast<link_loss_actions_t>(_param_nav_rcl_act.get()),
						       static_cast<offboard_loss_actions_t>(_param_com_obl_act.get()),
						       static_cast<quadchute_actions_t>(_param_com_qc_act.get()),
						       static_cast<offboard_loss_rc_actions_t>(_param_com_obl_rc_act.get()),
						       static_cast<position_nav_loss_actions_t>(_param_com_posctl_navl.get()),
						       _param_com_rcl_act_t.get(),
						       _param_com_rcl_except.get());

		if (nav_state_changed) {
			_vehicle_status.nav_state_timestamp = hrt_absolute_time();
		}

		if (_vehicle_status.failsafe != _failsafe_old) {
			_status_changed = true;

			if (_vehicle_status.failsafe) {
				mavlink_log_info(&_mavlink_log_pub, "Failsafe mode activated\t");
				events::send(events::ID("commander_failsafe_activated"), events::Log::Info, "Failsafe mode activated");

			} else {
				mavlink_log_info(&_mavlink_log_pub, "Failsafe mode deactivated\t");
				events::send(events::ID("commander_failsafe_deactivated"), events::Log::Info, "Failsafe mode deactivated");
			}

			_failsafe_old = _vehicle_status.failsafe;
		}


		// prearm mode
		switch ((PrearmedMode)_param_com_prearm_mode.get()) {
		case PrearmedMode::DISABLED:
			/* skip prearmed state  */
			_actuator_armed.prearmed = false;
			break;

		case PrearmedMode::ALWAYS:
			/* safety is not present, go into prearmed
			* (all output drivers should be started / unlocked last in the boot process
			* when the rest of the system is fully initialized)
			*/
			_actuator_armed.prearmed = (hrt_elapsed_time(&_boot_timestamp) > 5_s);
			break;

		case PrearmedMode::SAFETY_BUTTON:
			if (_safety.isButtonAvailable()) {
				/* safety button is present, go into prearmed if safety is off */
				_actuator_armed.prearmed = _safety.isSafetyOff();

			} else {
				/* safety button is not present, do not go into prearmed */
				_actuator_armed.prearmed = false;
			}

			break;

		default:
			_actuator_armed.prearmed = false;
			break;
		}


		// publish states (armed, control_mode, vehicle_status, commander_state, vehicle_status_flags, failure_detector_status) at 2 Hz or immediately when changed
		if (hrt_elapsed_time(&_vehicle_status.timestamp) >= 500_ms || _status_changed || nav_state_changed
		    || !(_actuator_armed == actuator_armed_prev)) {

			// Evaluate current prearm status (skip during arm -> disarm transition)
			if (!actuator_armed_prev.armed && !_arm_state_machine.isArmed() && !_vehicle_status_flags.calibration_enabled) {
				perf_begin(_preflight_check_perf);
				_vehicle_status_flags.pre_flight_checks_pass = PreFlightCheck::preflightCheck(nullptr, _vehicle_status,
						_vehicle_status_flags,
						_vehicle_control_mode,
						false, // report_failures
						_safety.isButtonAvailable(), _safety.isSafetyOff());
				perf_end(_preflight_check_perf);

				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_PREARM_CHECK, true, true,
						 _vehicle_status_flags.pre_flight_checks_pass, _vehicle_status);
			}

			// publish actuator_armed first (used by output modules)
			_actuator_armed.armed = _arm_state_machine.isArmed();
			_actuator_armed.ready_to_arm = _arm_state_machine.isArmed() || _arm_state_machine.isStandby();
			_actuator_armed.timestamp = hrt_absolute_time();
			_actuator_armed_pub.publish(_actuator_armed);

			// update and publish vehicle_control_mode
			update_control_mode();

			// vehicle_status publish (after prearm/preflight updates above)
			_vehicle_status.arming_state = _arm_state_machine.getArmState();
			_vehicle_status.timestamp = hrt_absolute_time();
			_vehicle_status_pub.publish(_vehicle_status);

			// publish vehicle_status_flags (after prearm/preflight updates above)
			_vehicle_status_flags.timestamp = hrt_absolute_time();
			_vehicle_status_flags_pub.publish(_vehicle_status_flags);

			// commander_state publish internal state for logging purposes
			_commander_state.timestamp = hrt_absolute_time();
			_commander_state_pub.publish(_commander_state);

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

		/* play arming and battery warning tunes */
		if (!_arm_tune_played && _arm_state_machine.isArmed()) {

			/* play tune when armed */
			set_tune(tune_control_s::TUNE_ID_ARMING_WARNING);
			_arm_tune_played = true;

		} else if (!_vehicle_status_flags.usb_connected &&
			   (_vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			/* play tune on battery critical */
			set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_FAST);

		} else if ((_vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_LOW)) {
			/* play tune on battery warning */
			set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_SLOW);

		} else if (_vehicle_status.failsafe && _arm_state_machine.isArmed()) {
			tune_failsafe(true);

		} else {
			set_tune(tune_control_s::TUNE_ID_STOP);
		}

		/* reset arm_tune_played when disarmed */
		if (!_arm_state_machine.isArmed()) {

			// Notify the user that it is safe to approach the vehicle
			if (_arm_tune_played) {
				tune_neutral(true);
			}

			_arm_tune_played = false;
		}

		// check if the worker has finished
		if (_worker_thread.hasResult()) {
			int ret = _worker_thread.getResultAndReset();
			_actuator_armed.in_esc_calibration_mode = false;

			if (_vehicle_status_flags.calibration_enabled) { // did we do a calibration?
				_vehicle_status_flags.calibration_enabled = false;

				if (ret == 0) {
					tune_positive(true);

				} else {
					tune_negative(true);
				}
			}
		}

		control_status_leds(_status_changed, _battery_warning);

		_status_changed = false;

		/* store last position lock state */
		_last_local_altitude_valid = _vehicle_status_flags.local_altitude_valid;
		_last_local_position_valid = _vehicle_status_flags.local_position_valid;
		_last_global_position_valid = _vehicle_status_flags.global_position_valid;

		_was_armed = _arm_state_machine.isArmed();

		arm_auth_update(now, params_updated);

		px4_indicate_external_reset_lockout(LockoutComponent::Commander, _arm_state_machine.isArmed());

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

void
Commander::get_circuit_breaker_params()
{
	_vehicle_status_flags.circuit_breaker_engaged_power_check = circuit_breaker_enabled_by_val(_param_cbrk_supply_chk.get(),
			CBRK_SUPPLY_CHK_KEY);
	_vehicle_status_flags.circuit_breaker_engaged_usb_check = circuit_breaker_enabled_by_val(_param_cbrk_usb_chk.get(),
			CBRK_USB_CHK_KEY);
	_vehicle_status_flags.circuit_breaker_engaged_airspd_check = circuit_breaker_enabled_by_val(
				_param_cbrk_airspd_chk.get(),
				CBRK_AIRSPD_CHK_KEY);
	_vehicle_status_flags.circuit_breaker_flight_termination_disabled = circuit_breaker_enabled_by_val(
				_param_cbrk_flightterm.get(),
				CBRK_FLIGHTTERM_KEY);
	_vehicle_status_flags.circuit_breaker_vtol_fw_arming_check = circuit_breaker_enabled_by_val(
				_param_cbrk_vtolarming.get(),
				CBRK_VTOLARMING_KEY);
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

			bool overload = (cpuload.load > 0.95f) || (cpuload.ram_usage > 0.98f);

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

		uint64_t overload_warn_delay = _arm_state_machine.isArmed() ? 1_ms : 250_ms;

		// set mode
		if (overload && (time_now_us >= _overload_start + overload_warn_delay)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_PURPLE;

		} else if (_arm_state_machine.isArmed()) {
			led_mode = led_control_s::MODE_ON;
			set_normal_color = true;

		} else if (!_vehicle_status_flags.pre_flight_checks_pass) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_RED;

		} else if (_arm_state_machine.isStandby()) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (_arm_state_machine.isInit()) {
			// if in init status it should not be in the error state
			led_mode = led_control_s::MODE_OFF;

		} else {
			// STANDBY_ERROR and other states
			led_mode = led_control_s::MODE_BLINK_NORMAL;
			led_color = led_control_s::COLOR_RED;
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
				if (_vehicle_status_flags.home_position_valid && _vehicle_status_flags.global_position_valid) {
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

	if (_arm_state_machine.isArmed()) {
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

	} else if (_arm_state_machine.isStandby()) {
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

bool Commander::check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				      const hrt_abstime &data_timestamp_us, hrt_abstime &last_fail_time_us,
				      const bool was_valid)
{
	bool valid = was_valid;
	const bool data_stale = ((hrt_elapsed_time(&data_timestamp_us) > _param_com_pos_fs_delay.get() * 1_s)
				 || (data_timestamp_us == 0));
	const float req_accuracy = (was_valid ? required_accuracy * 2.5f : required_accuracy);
	const bool level_check_pass = data_valid && !data_stale && (data_accuracy < req_accuracy);

	// Check accuracy with hysteresis in both test level and time
	if (level_check_pass) {
		if (!was_valid) {
			// check if probation period has elapsed
			if (hrt_elapsed_time(&last_fail_time_us) > 1_s) {
				valid = true;
			}
		}

	} else {
		// level check failed
		if (was_valid) {
			// FAILURE! no longer valid
			valid = false;
		}

		last_fail_time_us = hrt_absolute_time();
	}

	if (was_valid != valid) {
		_status_changed = true;
	}

	return valid;
}

void
Commander::update_control_mode()
{
	_vehicle_control_mode = {};

	/* set vehicle_control_mode according to set_navigation_state */
	_vehicle_control_mode.flag_armed = _arm_state_machine.isArmed();

	switch (_vehicle_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = stabilization_required();
		_vehicle_control_mode.flag_control_attitude_enabled = stabilization_required();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = true;
		_vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		_vehicle_control_mode.flag_control_auto_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = true;
		_vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		_vehicle_control_mode.flag_control_auto_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		_vehicle_control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		_vehicle_control_mode.flag_control_offboard_enabled = true;

		if (_offboard_control_mode_sub.get().position) {
			_vehicle_control_mode.flag_control_position_enabled = true;
			_vehicle_control_mode.flag_control_velocity_enabled = true;
			_vehicle_control_mode.flag_control_altitude_enabled = true;
			_vehicle_control_mode.flag_control_climb_rate_enabled = true;
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().velocity) {
			_vehicle_control_mode.flag_control_velocity_enabled = true;
			_vehicle_control_mode.flag_control_altitude_enabled = true;
			_vehicle_control_mode.flag_control_climb_rate_enabled = true;
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().acceleration) {
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().attitude) {
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().body_rate) {
			_vehicle_control_mode.flag_control_rates_enabled = true;
		}

		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:

	// Follow Target supports RC adjustment, so disable auto control mode to disable
	// the Flight Task from exiting itself when RC stick movement is detected.
	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		_vehicle_control_mode.flag_control_manual_enabled = false;
		_vehicle_control_mode.flag_control_auto_enabled = false;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = true;
		_vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	default:
		break;
	}

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

bool
Commander::stabilization_required()
{
	return _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
}

void
Commander::print_reject_mode(uint8_t main_state)
{
	if (hrt_elapsed_time(&_last_print_mode_reject_time) > 1_s) {

		mavlink_log_critical(&_mavlink_log_pub, "Switching to %s is currently not available\t", main_state_str(main_state));
		/* EVENT
		 * @description Check for a valid position estimate
		 */
		events::send<events::px4::enums::navigation_mode_t>(events::ID("commander_modeswitch_not_avail"), {events::Log::Critical, events::LogInternal::Info},
				"Switching to mode '{1}' is currently not possible", navigation_mode(main_state));

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		tune_negative(_arm_state_machine.isArmed());

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
				      3250,
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

void Commander::data_link_check()
{
	for (auto &telemetry_status :  _telemetry_status_subs) {
		telemetry_status_s telemetry;

		if (telemetry_status.update(&telemetry)) {

			// handle different radio types
			switch (telemetry.type) {
			case telemetry_status_s::LINK_TYPE_USB:
				// set (but don't unset) telemetry via USB as active once a MAVLink connection is up
				_vehicle_status_flags.usb_connected = true;
				break;

			case telemetry_status_s::LINK_TYPE_IRIDIUM: {
					iridiumsbd_status_s iridium_status;

					if (_iridiumsbd_status_sub.update(&iridium_status)) {
						_high_latency_datalink_heartbeat = iridium_status.last_heartbeat;

						if (_vehicle_status.high_latency_data_link_lost) {
							if (hrt_elapsed_time(&_high_latency_datalink_lost) > (_param_com_hldl_reg_t.get() * 1_s)) {
								_vehicle_status.high_latency_data_link_lost = false;
								_status_changed = true;
							}
						}

						const bool present = true;
						const bool enabled = true;
						const bool ok = (iridium_status.last_heartbeat > 0); // maybe at some point here an additional check should be made

						set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_SATCOM, present, enabled, ok, _vehicle_status);
					}

					break;
				}
			}

			if (telemetry.heartbeat_type_gcs) {
				// Initial connection or recovery from data link lost
				if (_vehicle_status.data_link_lost) {
					_vehicle_status.data_link_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_gcs != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Data link regained\t");
						events::send(events::ID("commander_dl_regained"), events::Log::Info, "Data link regained");
					}

					if (!_arm_state_machine.isArmed() && !_vehicle_status_flags.calibration_enabled) {
						// make sure to report preflight check failures to a connecting GCS
						PreFlightCheck::preflightCheck(&_mavlink_log_pub, _vehicle_status, _vehicle_status_flags, _vehicle_control_mode,
									       true, // report_failures
									       _safety.isButtonAvailable(), _safety.isSafetyOff());
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
				_vehicle_status_flags.parachute_system_present = true;
				_vehicle_status_flags.parachute_system_healthy = healthy;
				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_PARACHUTE, true, true, healthy, _vehicle_status);
			}

			if (telemetry.heartbeat_component_obstacle_avoidance) {
				if (_avoidance_system_lost) {
					_avoidance_system_lost = false;
					_status_changed = true;
				}

				_datalink_last_heartbeat_avoidance_system = telemetry.timestamp;
				_vehicle_status_flags.avoidance_system_valid = telemetry.avoidance_system_healthy;
			}
		}
	}


	// GCS data link loss failsafe
	if (!_vehicle_status.data_link_lost) {
		if ((_datalink_last_heartbeat_gcs != 0)
		    && hrt_elapsed_time(&_datalink_last_heartbeat_gcs) > (_param_com_dl_loss_t.get() * 1_s)) {

			_vehicle_status.data_link_lost = true;
			_vehicle_status.data_link_lost_counter++;

			mavlink_log_info(&_mavlink_log_pub, "Connection to ground station lost\t");
			events::send(events::ID("commander_gcs_lost"), {events::Log::Warning, events::LogInternal::Info},
				     "Connection to ground station lost");

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
		_vehicle_status_flags.parachute_system_present = false;
		_vehicle_status_flags.parachute_system_healthy = false;
		_parachute_system_lost = true;
		_status_changed = true;
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_PARACHUTE, false, true, false, _vehicle_status);
	}

	// AVOIDANCE SYSTEM state check (only if it is enabled)
	if (_vehicle_status_flags.avoidance_system_required && !_onboard_controller_lost) {
		// if heartbeats stop
		if (!_avoidance_system_lost && (_datalink_last_heartbeat_avoidance_system > 0)
		    && (hrt_elapsed_time(&_datalink_last_heartbeat_avoidance_system) > 5_s)) {

			_avoidance_system_lost = true;
			_vehicle_status_flags.avoidance_system_valid = false;
		}
	}

	// high latency data link loss failsafe
	if (_high_latency_datalink_heartbeat > 0
	    && hrt_elapsed_time(&_high_latency_datalink_heartbeat) > (_param_com_hldl_loss_t.get() * 1_s)) {
		_high_latency_datalink_lost = hrt_absolute_time();

		if (!_vehicle_status.high_latency_data_link_lost) {
			_vehicle_status.high_latency_data_link_lost = true;
			mavlink_log_critical(&_mavlink_log_pub, "High latency data link lost\t");
			events::send(events::ID("commander_high_latency_lost"), events::Log::Critical, "High latency data link lost");
			_status_changed = true;
		}
	}
}

void Commander::avoidance_check()
{
	for (auto &dist_sens_sub : _distance_sensor_subs) {
		distance_sensor_s distance_sensor;

		if (dist_sens_sub.update(&distance_sensor)) {
			if ((distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {

				_valid_distance_sensor_time_us = distance_sensor.timestamp;
			}
		}
	}

	const bool distance_sensor_valid = hrt_elapsed_time(&_valid_distance_sensor_time_us) < 500_ms;
	const bool cp_healthy = _vehicle_status_flags.avoidance_system_valid || distance_sensor_valid;

	const bool sensor_oa_present = cp_healthy || _vehicle_status_flags.avoidance_system_required
				       || _collision_prevention_enabled;

	const bool auto_mode = _vehicle_control_mode.flag_control_auto_enabled;
	const bool pos_ctl_mode = (_vehicle_control_mode.flag_control_manual_enabled
				   && _vehicle_control_mode.flag_control_position_enabled);

	const bool sensor_oa_enabled = ((auto_mode && _vehicle_status_flags.avoidance_system_required) || (pos_ctl_mode
					&& _collision_prevention_enabled));
	const bool sensor_oa_healthy = ((auto_mode && _vehicle_status_flags.avoidance_system_valid) || (pos_ctl_mode
					&& cp_healthy));

	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_OBSTACLE_AVOIDANCE, sensor_oa_present, sensor_oa_enabled,
			 sensor_oa_healthy, _vehicle_status);
}

void Commander::battery_status_check()
{
	size_t battery_required_count = 0;
	bool battery_has_fault = false;
	// There are possibly multiple batteries, and we can't know which ones serve which purpose. So the safest
	// option is to check if ANY of them have a warning, and specifically find which one has the most
	// urgent warning.
	uint8_t worst_warning = battery_status_s::BATTERY_WARNING_NONE;
	// To make sure that all connected batteries are being regularly reported, we check which one has the
	// oldest timestamp.
	hrt_abstime oldest_update = hrt_absolute_time();
	float worst_battery_time_s{NAN};

	for (auto &battery_sub : _battery_status_subs) {
		int index = battery_sub.get_instance();
		battery_status_s battery;

		if (!battery_sub.copy(&battery)) {
			continue;
		}

		if (battery.is_required) {
			battery_required_count++;
		}

		if (_arm_state_machine.isArmed()) {

			if (_last_connected_batteries[index] && !battery.connected) {
				mavlink_log_critical(&_mavlink_log_pub, "Battery %d disconnected. Land now! \t", index + 1);
				events::send<uint8_t>(events::ID("commander_battery_disconnected"), {events::Log::Emergency, events::LogInternal::Warning},
						      "Battery {1} disconnected. Land now!", index + 1);
				// trigger a battery failsafe action if a battery disconnects in flight
				worst_warning = battery_status_s::BATTERY_WARNING_CRITICAL;
			}

			if ((battery.mode > 0) && (battery.mode != _last_battery_mode[index])) {

				mavlink_log_critical(&_mavlink_log_pub, "Battery %d is in %s mode! \t", index + 1,
						     battery_mode_str(static_cast<battery_mode_t>(battery.mode)));
				events::send<uint8_t, events::px4::enums::battery_mode_t>(events::ID("commander_battery_mode"), {events::Log::Critical, events::LogInternal::Warning},
						"Battery {1} mode: {2}. Land now!", index + 1, static_cast<battery_mode_t>(battery.mode));
			}
		}

		_last_connected_batteries.set(index, battery.connected);
		_last_battery_mode[index] = battery.mode;

		if (battery.connected) {
			if (battery.warning > worst_warning) {
				worst_warning = battery.warning;
			}

			if (battery.timestamp < oldest_update) {
				oldest_update = battery.timestamp;
			}

			if (battery.faults > 0) {
				// MAVLink supported faults, can be checked on the ground station
				battery_has_fault = true;

				if (battery.faults != _last_battery_fault[index] || battery.custom_faults != _last_battery_custom_fault[index]) {
					for (uint8_t fault_index = 0; fault_index <= static_cast<uint8_t>(battery_fault_reason_t::_max);
					     fault_index++) {
						if (battery.faults & (1 << fault_index)) {
							mavlink_log_emergency(&_mavlink_log_pub, "Battery %d: %s. %s \t", index + 1,
									      battery_fault_reason_str(static_cast<battery_fault_reason_t>(fault_index)),
									      _arm_state_machine.isArmed() ? "Land now!" : "");

							events::px4::enums::suggested_action_t action = _arm_state_machine.isArmed() ?
									events::px4::enums::suggested_action_t::land :
									events::px4::enums::suggested_action_t::none;

							/* EVENT
							 * @description
							 * The battery reported a failure which might be dangerous to fly.
							 * Manufacturer error code: {4}
							 */
							events::send<uint8_t, battery_fault_reason_t, events::px4::enums::suggested_action_t, uint32_t>
							(events::ID("commander_battery_fault"), {events::Log::Emergency, events::LogInternal::Warning},
							 "Battery {1}: {2}. {3}", index + 1, static_cast<battery_fault_reason_t>(fault_index), action, battery.custom_faults);
						}
					}
				}
			}

			_last_battery_fault[index] = battery.faults;
			_last_battery_custom_fault[index] = battery.custom_faults;

			if (PX4_ISFINITE(battery.time_remaining_s)
			    && (!PX4_ISFINITE(worst_battery_time_s)
				|| (PX4_ISFINITE(worst_battery_time_s) && (battery.time_remaining_s < worst_battery_time_s)))) {
				worst_battery_time_s = battery.time_remaining_s;
			}

		}
	}

	rtl_time_estimate_s rtl_time_estimate{};

	// Compare estimate of RTL time to estimate of remaining flight time
	if (_rtl_time_estimate_sub.copy(&rtl_time_estimate)
	    && (hrt_absolute_time() - rtl_time_estimate.timestamp) < 2_s
	    && rtl_time_estimate.valid
	    && _arm_state_machine.isArmed()
	    && !_vehicle_land_detected.ground_contact // not in any landing stage
	    && !_rtl_time_actions_done
	    && PX4_ISFINITE(worst_battery_time_s)
	    && rtl_time_estimate.safe_time_estimate >= worst_battery_time_s
	    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
	    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {
		// Try to trigger RTL
		if (main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_RTL, _vehicle_status_flags,
					  _commander_state) == TRANSITION_CHANGED) {
			mavlink_log_emergency(&_mavlink_log_pub, "Remaining flight time low, returning to land\t");
			events::send(events::ID("commander_remaining_flight_time_rtl"), {events::Log::Critical, events::LogInternal::Info},
				     "Remaining flight time low, returning to land");

		} else {
			mavlink_log_emergency(&_mavlink_log_pub, "Remaining flight time low, land now!\t");
			events::send(events::ID("commander_remaining_flight_time_land"), {events::Log::Critical, events::LogInternal::Info},
				     "Remaining flight time low, land now!");
		}

		_rtl_time_actions_done = true;
	}

	bool battery_warning_level_increased_while_armed = false;
	bool update_internal_battery_state = false;

	if (_arm_state_machine.isArmed()) {
		if (worst_warning > _battery_warning) {
			battery_warning_level_increased_while_armed = true;
			update_internal_battery_state = true;
		}

	} else {
		if (_battery_warning != worst_warning) {
			update_internal_battery_state = true;
		}
	}

	if (update_internal_battery_state) {
		_battery_warning = worst_warning;
	}

	_vehicle_status_flags.battery_healthy =
		// All connected batteries are regularly being published
		(hrt_elapsed_time(&oldest_update) < 5_s)
		// There is at least one connected battery (in any slot)
		&& (_last_connected_batteries.count() >= battery_required_count)
		// No currently-connected batteries have any warning
		&& (_battery_warning == battery_status_s::BATTERY_WARNING_NONE)
		// No currently-connected batteries have any fault
		&& (!battery_has_fault);

	// execute battery failsafe if the state has gotten worse while we are armed
	if (battery_warning_level_increased_while_armed) {
		uint8_t failsafe_action = get_battery_failsafe_action(_commander_state, _battery_warning,
					  (low_battery_action_t)_param_com_low_bat_act.get());

		warn_user_about_battery(&_mavlink_log_pub, _battery_warning,
					failsafe_action, _param_com_bat_act_t.get(),
					main_state_str(failsafe_action), navigation_mode(failsafe_action));
		_battery_failsafe_timestamp = hrt_absolute_time();

		// Switch to loiter to wait for the reaction delay
		if (_param_com_bat_act_t.get() > 0.f
		    && failsafe_action != commander_state_s::MAIN_STATE_MAX) {
			main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _vehicle_status_flags,
					      _commander_state);
		}
	}

	if (_battery_failsafe_timestamp != 0
	    && hrt_elapsed_time(&_battery_failsafe_timestamp) > _param_com_bat_act_t.get() * 1_s
	    && (_commander_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER
		|| _vehicle_control_mode.flag_control_auto_enabled)) {
		_battery_failsafe_timestamp = 0;
		uint8_t failsafe_action = get_battery_failsafe_action(_commander_state, _battery_warning,
					  (low_battery_action_t)_param_com_low_bat_act.get());

		if (failsafe_action != commander_state_s::MAIN_STATE_MAX) {
			_commander_state.main_state = failsafe_action;
			_commander_state.main_state_changes++;
			_commander_state.timestamp = hrt_absolute_time();
		}
	}

	// Handle shutdown request from emergency battery action
	if (update_internal_battery_state) {

		if (_battery_warning == battery_status_s::BATTERY_WARNING_EMERGENCY) {
#if defined(BOARD_HAS_POWER_CONTROL)

			if (shutdown_if_allowed() && (px4_shutdown_request(60_s) == 0)) {
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
}

void Commander::estimator_check()
{
	// Check if quality checking of position accuracy and consistency is to be performed
	const bool run_quality_checks = true;

	_local_position_sub.update();
	_global_position_sub.update();

	const vehicle_local_position_s &lpos = _local_position_sub.get();

	if (lpos.heading_reset_counter != _heading_reset_counter) {
		if (_vehicle_status_flags.home_position_valid) {
			updateHomePositionYaw(_home_position_pub.get().yaw + lpos.delta_heading);
		}

		_heading_reset_counter = lpos.heading_reset_counter;
	}

	const bool mag_fault_prev = _estimator_status_flags_sub.get().cs_mag_fault;
	const bool gnss_heading_fault_prev = _estimator_status_flags_sub.get().cs_gps_yaw_fault;

	// use primary estimator_status
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			if (estimator_selector_status.primary_instance != _estimator_status_sub.get_instance()) {
				_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
				_estimator_status_flags_sub.ChangeInstance(estimator_selector_status.primary_instance);
			}
		}
	}

	if (_estimator_status_flags_sub.update()) {
		const estimator_status_flags_s &estimator_status_flags = _estimator_status_flags_sub.get();

		_vehicle_status_flags.dead_reckoning = estimator_status_flags.cs_wind_dead_reckoning
						       || estimator_status_flags.cs_inertial_dead_reckoning;

		if (!(estimator_status_flags.cs_inertial_dead_reckoning || estimator_status_flags.cs_wind_dead_reckoning)) {
			// position requirements (update if not dead reckoning)
			bool gps             = estimator_status_flags.cs_gps;
			bool optical_flow    = estimator_status_flags.cs_opt_flow;
			bool vision_position = estimator_status_flags.cs_ev_pos;

			_vehicle_status_flags.position_reliant_on_gps             =  gps && !optical_flow && !vision_position;
			_vehicle_status_flags.position_reliant_on_optical_flow    = !gps &&  optical_flow && !vision_position;
			_vehicle_status_flags.position_reliant_on_vision_position = !gps && !optical_flow &&  vision_position;
		}

		// Check for a magnetomer fault and notify the user
		if (!mag_fault_prev && estimator_status_flags.cs_mag_fault) {
			mavlink_log_critical(&_mavlink_log_pub, "Compass needs calibration - Land now!\t");
			events::send(events::ID("commander_stopping_mag_use"), events::Log::Critical,
				     "Stopping compass use! Land now and calibrate the compass");
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MAG, true, true, false, _vehicle_status);
		}

		if (!gnss_heading_fault_prev && estimator_status_flags.cs_gps_yaw_fault) {
			mavlink_log_critical(&_mavlink_log_pub, "GNSS heading not reliable - Land now!\t");
			events::send(events::ID("commander_stopping_gnss_heading_use"), events::Log::Critical,
				     "GNSS heading not reliable. Land now!");
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_GPS, true, true, false, _vehicle_status);
		}
	}


	/* Check estimator status for signs of bad yaw induced post takeoff navigation failure
	* for a short time interval after takeoff.
	* Most of the time, the drone can recover from a bad initial yaw using GPS-inertial
	* heading estimation (yaw emergency estimator) or GPS heading (fixed wings only), but
	* if this does not fix the issue we need to stop using a position controlled
	* mode to prevent flyaway crashes.
	*/
	bool pre_flt_fail_innov_heading = false;
	bool pre_flt_fail_innov_vel_horiz = false;

	if (_estimator_status_sub.updated()) {

		estimator_status_s estimator_status;

		if (_estimator_status_sub.copy(&estimator_status)) {

			pre_flt_fail_innov_heading = estimator_status.pre_flt_fail_innov_heading;
			pre_flt_fail_innov_vel_horiz = estimator_status.pre_flt_fail_innov_vel_horiz;

			if (run_quality_checks && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

				if (!_arm_state_machine.isArmed()) {
					_nav_test_failed = false;
					_nav_test_passed = false;

				} else {
					if (!_nav_test_passed) {
						// Both test ratios need to pass/fail together to change the nav test status
						const bool innovation_pass = (estimator_status.vel_test_ratio < 1.f) && (estimator_status.pos_test_ratio < 1.f)
									     && (estimator_status.vel_test_ratio > FLT_EPSILON) && (estimator_status.pos_test_ratio > FLT_EPSILON);

						const bool innovation_fail = (estimator_status.vel_test_ratio >= 1.f) && (estimator_status.pos_test_ratio >= 1.f);

						if (innovation_pass) {
							_time_last_innov_pass = hrt_absolute_time();

							// if nav status is unconfirmed, confirm yaw angle as passed after 30 seconds or achieving 5 m/s of speed
							const bool sufficient_time = (_vehicle_status.takeoff_time != 0)
										     && (hrt_elapsed_time(&_vehicle_status.takeoff_time) > 30_s);
							const bool sufficient_speed = matrix::Vector2f(lpos.vx, lpos.vy).longerThan(5.f);

							// Even if the test already failed, allow it to pass if it did not fail during the last 10 seconds
							if (hrt_elapsed_time(&_time_last_innov_fail) > 10_s
							    && (sufficient_time || sufficient_speed)) {
								_nav_test_passed = true;
								_nav_test_failed = false;
							}

						} else if (innovation_fail) {
							_time_last_innov_fail = hrt_absolute_time();

							if (!_nav_test_failed && hrt_elapsed_time(&_time_last_innov_pass) > 2_s) {
								// if the innovation test has failed continuously, declare the nav as failed
								_nav_test_failed = true;
								mavlink_log_emergency(&_mavlink_log_pub, "Navigation failure! Land and recalibrate sensors\t");
								events::send(events::ID("commander_navigation_failure"), events::Log::Emergency,
									     "Navigation failure! Land and recalibrate the sensors");
							}
						}
					}
				}
			}
		}
	}

	// run position and velocity accuracy checks
	// Check if quality checking of position accuracy and consistency is to be performed
	if (run_quality_checks) {
		float lpos_eph_threshold_adj = _param_com_pos_fs_eph.get();

		// relax local position eph threshold in operator controlled position mode
		if (_commander_state.main_state == commander_state_s::MAIN_STATE_POSCTL &&
		    ((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
		     || (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL))) {

			// Set the allowable position uncertainty based on combination of flight and estimator state
			// When we are in a operator demanded position control mode and are solely reliant on optical flow,
			// do not check position error because it will gradually increase throughout flight and the operator will compensate for the drift
			if (_vehicle_status_flags.position_reliant_on_optical_flow) {
				lpos_eph_threshold_adj = INFINITY;
			}
		}

		bool xy_valid = lpos.xy_valid && !_nav_test_failed;
		bool v_xy_valid = lpos.v_xy_valid && !_nav_test_failed;

		if (!_arm_state_machine.isArmed()) {
			if (pre_flt_fail_innov_heading || pre_flt_fail_innov_vel_horiz) {
				xy_valid = false;
			}

			if (pre_flt_fail_innov_vel_horiz) {
				v_xy_valid = false;
			}
		}

		const vehicle_global_position_s &gpos = _global_position_sub.get();

		_vehicle_status_flags.global_position_valid =
			check_posvel_validity(xy_valid, gpos.eph, _param_com_pos_fs_eph.get(), gpos.timestamp,
					      _last_gpos_fail_time_us, _vehicle_status_flags.global_position_valid);

		_vehicle_status_flags.local_position_valid =
			check_posvel_validity(xy_valid, lpos.eph, lpos_eph_threshold_adj, lpos.timestamp,
					      _last_lpos_fail_time_us, _vehicle_status_flags.local_position_valid);

		_vehicle_status_flags.local_velocity_valid =
			check_posvel_validity(v_xy_valid, lpos.evh, _param_com_vel_fs_evh.get(), lpos.timestamp,
					      _last_lvel_fail_time_us, _vehicle_status_flags.local_velocity_valid);
	}


	// altitude
	_vehicle_status_flags.local_altitude_valid = lpos.z_valid
			&& (hrt_elapsed_time(&lpos.timestamp) < (_param_com_pos_fs_delay.get() * 1_s));


	// attitude
	vehicle_attitude_s attitude{};
	_vehicle_attitude_sub.copy(&attitude);
	const matrix::Quatf q{attitude.q};
	const bool no_element_larger_than_one = (fabsf(q(0)) <= 1.f)
						&& (fabsf(q(1)) <= 1.f)
						&& (fabsf(q(2)) <= 1.f)
						&& (fabsf(q(3)) <= 1.f);
	const bool norm_in_tolerance = (fabsf(1.f - q.norm()) <= 1e-6f);

	const bool attitude_valid = (hrt_elapsed_time(&attitude.timestamp) < 1_s)
				    && norm_in_tolerance && no_element_larger_than_one;

	if (_vehicle_status_flags.attitude_valid && !attitude_valid) {
		PX4_ERR("attitude estimate no longer valid");
	}

	_vehicle_status_flags.attitude_valid = attitude_valid;


	// angular velocity
	vehicle_angular_velocity_s angular_velocity{};
	_vehicle_angular_velocity_sub.copy(&angular_velocity);
	const bool condition_angular_velocity_time_valid = (angular_velocity.timestamp != 0)
			&& (hrt_elapsed_time(&angular_velocity.timestamp) < 1_s);
	const bool condition_angular_velocity_finite = PX4_ISFINITE(angular_velocity.xyz[0])
			&& PX4_ISFINITE(angular_velocity.xyz[1]) && PX4_ISFINITE(angular_velocity.xyz[2]);
	const bool angular_velocity_valid = condition_angular_velocity_time_valid
					    && condition_angular_velocity_finite;

	if (_vehicle_status_flags.angular_velocity_valid && !angular_velocity_valid) {
		const char err_str[] {"angular velocity no longer valid"};

		if (!condition_angular_velocity_time_valid) {
			PX4_ERR("%s (timeout)", err_str);

		} else if (!condition_angular_velocity_finite) {
			PX4_ERR("%s (non-finite values)", err_str);
		}
	}

	_vehicle_status_flags.angular_velocity_valid = angular_velocity_valid;


	// gps
	const bool condition_gps_position_was_valid = _vehicle_status_flags.gps_position_valid;

	if (_vehicle_gps_position_sub.updated()) {
		sensor_gps_s vehicle_gps_position;

		if (_vehicle_gps_position_sub.copy(&vehicle_gps_position)) {

			bool time = (vehicle_gps_position.timestamp != 0) && (hrt_elapsed_time(&vehicle_gps_position.timestamp) < 1_s);

			bool fix = vehicle_gps_position.fix_type >= 2;
			bool eph = vehicle_gps_position.eph < _param_com_pos_fs_eph.get();
			bool epv = vehicle_gps_position.epv < _param_com_pos_fs_epv.get();
			bool evh = vehicle_gps_position.s_variance_m_s < _param_com_vel_fs_evh.get();
			bool no_jamming = (vehicle_gps_position.jamming_state != sensor_gps_s::JAMMING_STATE_CRITICAL);

			_vehicle_gps_position_valid.set_state_and_update(time && fix && eph && epv && evh && no_jamming, hrt_absolute_time());
			_vehicle_status_flags.gps_position_valid = _vehicle_gps_position_valid.get_state();

			_vehicle_gps_position_timestamp_last = vehicle_gps_position.timestamp;

			if (condition_gps_position_was_valid) {
				if (vehicle_gps_position.jamming_state == sensor_gps_s::JAMMING_STATE_CRITICAL) {
					mavlink_log_warning(&_mavlink_log_pub, "GPS jamming state critical\t");
					events::send(events::ID("commander_gps_jamming_critical"), {events::Log::Critical, events::LogInternal::Info},
						     "GPS jamming state critical");
				}
			}
		}

	} else {
		const hrt_abstime now_us = hrt_absolute_time();

		if (now_us > _vehicle_gps_position_timestamp_last + GPS_VALID_TIME) {
			_vehicle_gps_position_valid.set_state_and_update(false, now_us);
			_vehicle_status_flags.gps_position_valid = false;
		}
	}

	if (condition_gps_position_was_valid && !_vehicle_status_flags.gps_position_valid) {
		PX4_DEBUG("GPS no longer valid");

		// report GPS failure if flying and the global position estimate is still valid
		if (!_vehicle_land_detected.landed && _vehicle_status_flags.global_position_valid) {
			mavlink_log_warning(&_mavlink_log_pub, "GPS no longer valid\t");
			events::send(events::ID("commander_gps_lost"), {events::Log::Critical, events::LogInternal::Info},
				     "GPS no longer valid");
		}
	}
}

void Commander::manual_control_check()
{
	manual_control_setpoint_s manual_control_setpoint;
	const bool manual_control_updated = _manual_control_setpoint_sub.update(&manual_control_setpoint);

	if (manual_control_updated && manual_control_setpoint.valid) {

		if (!_vehicle_status_flags.rc_signal_found_once) {
			_vehicle_status_flags.rc_signal_found_once = true;

		} else if (_vehicle_status.rc_signal_lost) {
			if (_last_valid_manual_control_setpoint > 0) {
				float elapsed = hrt_elapsed_time(&_last_valid_manual_control_setpoint) * 1e-6f;
				mavlink_log_info(&_mavlink_log_pub, "Manual control regained after %.1fs\t", (double)elapsed);
				events::send<float>(events::ID("commander_rc_regained"), events::Log::Info,
						    "Manual control regained after {1:.1} s", elapsed);
			}
		}

		if (_vehicle_status.rc_signal_lost) {
			_vehicle_status.rc_signal_lost = false;
			_status_changed = true;
		}

		_last_valid_manual_control_setpoint = manual_control_setpoint.timestamp;
		_is_throttle_above_center = (manual_control_setpoint.z > 0.6f);
		_is_throttle_low = (manual_control_setpoint.z < 0.1f);

		const bool is_mavlink = (manual_control_setpoint.data_source > manual_control_setpoint_s::SOURCE_RC);

		if (is_mavlink) {
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, true, _vehicle_status);

		} else {
			// if not mavlink also report valid RC calibration for health
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, _vehicle_status_flags.rc_calibration_valid,
					 _vehicle_status);
		}

		if (_arm_state_machine.isArmed()) {
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

				const bool in_low_battery_failsafe_delay = (_battery_failsafe_timestamp != 0);

				if (override_enabled && !in_low_battery_failsafe_delay && !_geofence_warning_action_on) {

					const transition_result_t posctl_result =
						main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_POSCTL, _vehicle_status_flags, _commander_state);

					if (posctl_result == TRANSITION_CHANGED) {
						tune_positive(true);
						mavlink_log_info(&_mavlink_log_pub, "Pilot took over position control using sticks\t");
						events::send(events::ID("commander_rc_override_pos"), events::Log::Info,
							     "Pilot took over position control using sticks");
						_status_changed = true;

					} else if (posctl_result == TRANSITION_DENIED) {
						// If transition to POSCTL was denied, then we can try again with ALTCTL.
						const transition_result_t altctl_result =
							main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_ALTCTL, _vehicle_status_flags, _commander_state);

						if (altctl_result == TRANSITION_CHANGED) {
							tune_positive(true);
							mavlink_log_info(&_mavlink_log_pub, "Pilot took over altitude control using sticks\t");
							events::send(events::ID("commander_rc_override_alt"), events::Log::Info,
								     "Pilot took over altitude control using sticks");
							_status_changed = true;
						}
					}
				}
			}

		} else {
			// disarmed
			// if there's never been a mode change force position control as initial state
			if (_commander_state.main_state_changes == 0) {
				if (is_mavlink || !_mode_switch_mapped) {
					_commander_state.main_state = commander_state_s::MAIN_STATE_POSCTL;
					_commander_state.main_state_changes++;
				}
			}
		}

	} else if ((manual_control_updated && !manual_control_setpoint.valid)
		   || hrt_elapsed_time(&_last_valid_manual_control_setpoint) > _param_com_rc_loss_t.get() * 1_s) {

		// prohibit stick use in case of reported invalidity or data timeout
		if (!_vehicle_status.rc_signal_lost) {
			_vehicle_status.rc_signal_lost = true;
			_status_changed = true;

			mavlink_log_critical(&_mavlink_log_pub, "Manual control lost\t");
			events::send(events::ID("commander_rc_lost"), {events::Log::Critical, events::LogInternal::Info},
				     "Manual control lost");

			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, false, _vehicle_status);
		}
	}
}

void
Commander::offboard_control_update()
{
	bool offboard_available = false;

	if (_offboard_control_mode_sub.updated()) {
		const offboard_control_mode_s old = _offboard_control_mode_sub.get();

		if (_offboard_control_mode_sub.update()) {
			const offboard_control_mode_s &ocm = _offboard_control_mode_sub.get();

			if (old.position != ocm.position ||
			    old.velocity != ocm.velocity ||
			    old.acceleration != ocm.acceleration ||
			    old.attitude != ocm.attitude ||
			    old.body_rate != ocm.body_rate ||
			    old.actuator != ocm.actuator) {

				_status_changed = true;
			}

			if (ocm.position || ocm.velocity || ocm.acceleration || ocm.attitude || ocm.body_rate || ocm.actuator) {
				offboard_available = true;
			}
		}
	}

	if (_offboard_control_mode_sub.get().position && !_vehicle_status_flags.local_position_valid) {
		offboard_available = false;

	} else if (_offboard_control_mode_sub.get().velocity && !_vehicle_status_flags.local_velocity_valid) {
		offboard_available = false;

	} else if (_offboard_control_mode_sub.get().acceleration && !_vehicle_status_flags.local_velocity_valid) {
		// OFFBOARD acceleration handled by position controller
		offboard_available = false;
	}

	_offboard_available.set_state_and_update(offboard_available, hrt_absolute_time());

	const bool offboard_lost = !_offboard_available.get_state();

	if (_vehicle_status_flags.offboard_control_signal_lost != offboard_lost) {
		_vehicle_status_flags.offboard_control_signal_lost = offboard_lost;
		_status_changed = true;
	}
}

void Commander::esc_status_check()
{
	esc_status_s esc_status{};

	_esc_status_sub.copy(&esc_status);

	if (esc_status.esc_count > 0) {

		char esc_fail_msg[50];
		esc_fail_msg[0] = '\0';

		int online_bitmask = (1 << esc_status.esc_count) - 1;

		// Check if ALL the ESCs are online
		if (online_bitmask == esc_status.esc_online_flags) {

			_vehicle_status_flags.escs_error = false;
			_last_esc_online_flags = esc_status.esc_online_flags;

		} else if (_last_esc_online_flags == esc_status.esc_online_flags)  {

			// Avoid checking the status if the flags are the same or if the mixer has not yet been loaded in the ESC driver

			_vehicle_status_flags.escs_error = true;

		} else if (esc_status.esc_online_flags < _last_esc_online_flags) {

			// Only warn the user when an ESC goes from ONLINE to OFFLINE. This is done to prevent showing Offline ESCs warnings at boot

			for (int index = 0; index < esc_status.esc_count; index++) {
				if ((esc_status.esc_online_flags & (1 << index)) == 0) {
					snprintf(esc_fail_msg + strlen(esc_fail_msg), sizeof(esc_fail_msg) - strlen(esc_fail_msg), "ESC%d ", index + 1);
					esc_fail_msg[sizeof(esc_fail_msg) - 1] = '\0';
					events::px4::enums::suggested_action_t action = _arm_state_machine.isArmed() ?
							events::px4::enums::suggested_action_t::land :
							events::px4::enums::suggested_action_t::none;
					// TODO: use esc_status.esc[index].actuator_function as index after SYS_CTRL_ALLOC becomes default
					events::send<uint8_t, events::px4::enums::suggested_action_t>(events::ID("commander_esc_offline"),
							events::Log::Critical, "ESC{1} offline. {2}", index + 1, action);
				}
			}

			mavlink_log_critical(&_mavlink_log_pub, "%soffline. %s\t", esc_fail_msg,
					     _arm_state_machine.isArmed() ? "Land now!" : "");

			_last_esc_online_flags = esc_status.esc_online_flags;
			_vehicle_status_flags.escs_error = true;
		}

		_vehicle_status_flags.escs_failure = false;

		for (int index = 0; index < esc_status.esc_count; index++) {

			_vehicle_status_flags.escs_failure |= esc_status.esc[index].failures > 0;

			if (esc_status.esc[index].failures != _last_esc_failure[index]) {

				for (uint8_t fault_index = 0; fault_index <= static_cast<uint8_t>(esc_fault_reason_t::_max);
				     fault_index++) {
					if (esc_status.esc[index].failures & (1 << fault_index)) {

						esc_fault_reason_t fault_reason_index = static_cast<esc_fault_reason_t>(fault_index);

						const char *user_action = nullptr;
						events::px4::enums::suggested_action_t action = events::px4::enums::suggested_action_t::none;

						if (fault_reason_index == esc_fault_reason_t::motor_warn_temp
						    || fault_reason_index == esc_fault_reason_t::esc_warn_temp) {
							user_action = "Reduce throttle";
							action = events::px4::enums::suggested_action_t::reduce_throttle;

						} else {
							user_action = "Land now!";
							action = events::px4::enums::suggested_action_t::land;
						}

						mavlink_log_emergency(&_mavlink_log_pub, "ESC%d: %s. %s \t", index + 1,
								      esc_fault_reason_str(fault_reason_index), _arm_state_machine.isArmed() ? user_action : "");

						events::send<uint8_t, events::px4::enums::esc_fault_reason_t, events::px4::enums::suggested_action_t>
						(events::ID("commander_esc_fault"), {events::Log::Emergency, events::LogInternal::Warning},
						 "ESC {1}: {2}. {3}", index + 1, fault_reason_index, action);
					}
				}
			}

			_last_esc_failure[index] = esc_status.esc[index].failures;
		}

	}

	_last_esc_status_updated = esc_status.timestamp;
}

void Commander::send_parachute_command()
{
	vehicle_command_s vcmd{};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
	vcmd.param1 = static_cast<float>(vehicle_command_s::PARACHUTE_ACTION_RELEASE);

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = 161; // MAV_COMP_ID_PARACHUTE

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vcmd_pub.publish(vcmd);

	set_tune_override(tune_control_s::TUNE_ID_PARACHUTE_RELEASE);
}

void Commander::checkWindSpeedThresholds()
{
	wind_s wind_estimate;

	if (_wind_sub.update(&wind_estimate)) {
		const matrix::Vector2f wind(wind_estimate.windspeed_north, wind_estimate.windspeed_east);

		// publish a warning if it's the first since in air or 60s have passed since the last warning
		const bool warning_timeout_passed = _last_wind_warning == 0 || hrt_elapsed_time(&_last_wind_warning) > 60_s;

		if (_param_com_wind_max.get() > FLT_EPSILON
		    && wind.longerThan(_param_com_wind_max.get())
		    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
		    && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {

			main_state_transition(_vehicle_status, commander_state_s::MAIN_STATE_AUTO_RTL, _vehicle_status_flags, _commander_state);
			_status_changed = true;
			mavlink_log_critical(&_mavlink_log_pub, "Wind speeds above limit, abort operation and RTL (%.1f m/s)\t",
					     (double)wind.norm());

			events::send<float>(events::ID("commander_high_wind_rtl"),
			{events::Log::Warning, events::LogInternal::Info},
			"Wind speeds above limit, abort operation and RTL ({1:.1m/s})", wind.norm());

		} else if (_param_com_wind_warn.get() > FLT_EPSILON
			   && wind.longerThan(_param_com_wind_warn.get())
			   && warning_timeout_passed
			   && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
			   && _commander_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {

			mavlink_log_critical(&_mavlink_log_pub, "High wind speed detected (%.1f m/s), landing advised\t", (double)wind.norm());

			events::send<float>(events::ID("commander_high_wind_warning"),
			{events::Log::Warning, events::LogInternal::Info},
			"High wind speed detected ({1:.1m/s}), landing advised", wind.norm());
			_last_wind_warning = hrt_absolute_time();
		}
	}
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
	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|altctl|posctl|auto:mission|auto:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland",
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

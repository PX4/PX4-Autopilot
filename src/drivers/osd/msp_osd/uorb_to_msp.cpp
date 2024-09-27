/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/* uorb_to_msp.cpp
 *
 * Implementation file for UORB -> MSP conversion functions.
 */

// includes for mathematical manipulation
#include <math.h>
#include <matrix/math.hpp>
#include <lib/geo/geo.h>

// clock access
#include <px4_platform_common/defines.h>
using namespace time_literals;

#include "msp_osd.hpp"

#include "uorb_to_msp.hpp"

msp_battery_state_t MspOsd::construct_BATTERY_STATE(const battery_status_s& battery_status) {

	// initialize result
	msp_battery_state_t battery_state = {0};

	// MSP_BATTERY_STATE
	battery_state.amperage = battery_status.current_a; // not used? 	(int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); // send current in 0.01 A steps, range is -320A to 320A
	battery_state.batteryVoltage = (uint16_t)(battery_status.voltage_v * 100.0f);		// Should be constrained to uint16

	battery_state.mAhDrawn = battery_status.discharged_mah ; // OK
	battery_state.batteryCellCount = battery_status.cell_count;								 // 0 indicates battery not detected.
	battery_state.batteryCapacity = battery_status.capacity; // not used?

	// Voltage Text Display color 0==white, 1==red
	if (battery_status.voltage_v < _param_batt_low_v.get()) {
		battery_state.batteryState = 1;

	} else {
		battery_state.batteryState = 0;
	}

	battery_state.legacyBatteryVoltage = uint8_t(battery_status.voltage_v * 10);
	return battery_state;
}

msp_name_t MspOsd::construct_display_message(const struct vehicle_status_s& vehicle_status,
				     const struct vehicle_attitude_s& vehicle_attitude,
				     const struct log_message_s& log_message,
				     const struct esc_status_s& esc_status,
				     const struct parameter_selector_s& parameter_selector,
				     const int log_level,
				     MessageDisplay &display)
{
	// initialize result
	msp_name_t display_message {0};

	const auto now = hrt_absolute_time();
	set_arm_status_string(now, vehicle_status, display);
	set_flight_mode_string(now, vehicle_status, esc_status, parameter_selector, display);
	set_warning_string(now, log_message, log_level, display);
	set_heading_string(now, vehicle_attitude, display);

	// update message and return
	display.get(display_message.craft_name, hrt_absolute_time());
	return display_message;
}

void MspOsd::set_arm_status_string(const hrt_abstime& now, const struct vehicle_status_s& vehicle_status, MessageDisplay& display)
{
	// update arming state if current
	if (vehicle_status.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::ARMING, "???");
		return;
	}

	// display armed / disarmed
	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		display.set(MessageDisplayType::ARMING, "ARM");
	else
		display.set(MessageDisplayType::ARMING, "DSRM");
}

void MspOsd::set_flight_mode_string(const hrt_abstime& now, const struct vehicle_status_s& vehicle_status, const struct esc_status_s& esc_status, const struct parameter_selector_s& parameter_selector, MessageDisplay& display)
{
	// return if flight mode is stale or if the parameter selector state has not changed
	static uint32_t param_selector_instance = 0;
	if (vehicle_status.timestamp < (now - 1_s) && (parameter_selector.instance <= param_selector_instance)) {
		display.set(MessageDisplayType::FLIGHT_MODE, "???");
		return;
	}

	// FIXME: Could instead display it as a warning that scrolls?
	// But once we are out of turtle mode the message should immediately go away.... and currently that doesn't happen

	// If the ESC is in turtle mode show that as the mode
	// FIXME: should be removed once turtle mode is a proper mode  in PX4
	bool turtle_mode_enabled = false;
	for (auto report : esc_status.esc) {
		// Turtle mode is enabled on one esc assume it is set all other relevant escs
		if (report.esc_state == esc_report_s::ESC_STATE_TURTLE_MODE) {
			turtle_mode_enabled = true;
		}
	}

	if (turtle_mode_enabled) {
		display.set(MessageDisplayType::FLIGHT_MODE, "TURTLE");
		return;
	}

	// If the parameter selector module is being used overide the flight mode string
	// with the short string from the parameter selector
	if (parameter_selector.state > 0) {
		display.set(MessageDisplayType::FLIGHT_MODE, parameter_selector.short_string);
		return;
	}

	// display flight mode
	switch (vehicle_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		display.set(MessageDisplayType::FLIGHT_MODE, "MANUAL");
		break;
	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		display.set(MessageDisplayType::FLIGHT_MODE, "ALTCTL");
		break;
	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		display.set(MessageDisplayType::FLIGHT_MODE, "POSCTL");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_MISSION");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_LOITER");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_RTL");
		break;
	case vehicle_status_s::NAVIGATION_STATE_UNUSED:
		display.set(MessageDisplayType::FLIGHT_MODE, "UNUSED");
		break;
	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		display.set(MessageDisplayType::FLIGHT_MODE, "ACRO");
		break;
	case vehicle_status_s::NAVIGATION_STATE_UNUSED1:
		display.set(MessageDisplayType::FLIGHT_MODE, "UNUSED1");
		break;
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		display.set(MessageDisplayType::FLIGHT_MODE, "DESCEND");
		break;
	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		display.set(MessageDisplayType::FLIGHT_MODE, "TERMINATION");
		break;
	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		display.set(MessageDisplayType::FLIGHT_MODE, "OFFBOARD");
		break;
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		display.set(MessageDisplayType::FLIGHT_MODE, "STAB");
		break;
	case vehicle_status_s::NAVIGATION_STATE_UNUSED2:
		display.set(MessageDisplayType::FLIGHT_MODE, "UNUSED2");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_TAKEOFF");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_LAND");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_FOLLOW_TARGET");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_PRECLAND");
		break;
	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		display.set(MessageDisplayType::FLIGHT_MODE, "ORBIT");
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_VTOL_TAKEOFF");
		break;
	case vehicle_status_s::NAVIGATION_STATE_MAX:
		display.set(MessageDisplayType::FLIGHT_MODE, "MAX");
		break;
	default:
		display.set(MessageDisplayType::FLIGHT_MODE, "???");
	}
}

void MspOsd::set_warning_string(const hrt_abstime& now, const struct log_message_s& log_message, const int log_level, MessageDisplay& display)
{
	static uint64_t last_warning_stamp {0};
	// display, if updated
	if (log_message.severity <= log_level) {
		display.set(MessageDisplayType::WARNING, log_message.subtext);
		last_warning_stamp = now;
	} else if (now - last_warning_stamp > 10_s) {
		// clear warning after timeout
		display.set(MessageDisplayType::WARNING, "");
		last_warning_stamp = now;
	}
}

void MspOsd::set_heading_string(const hrt_abstime& now, const struct vehicle_attitude_s& vehicle_attitude, MessageDisplay& display)
{
	if (vehicle_attitude.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::HEADING, "N?");
		return;
	}

	// Get the current yaw angle and convert to YAW between 0 to 360 deg
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	const auto vehicle_yaw = matrix::wrap(math::degrees(euler_attitude.psi()), 0.f, 360.f);

	// Reset North (12 O'Clock) to the current yaw angle if told by the user
	const uint8_t osd_hg_rst_chan = _param_osd_hdg_rst_chan.get();
	if ((osd_hg_rst_chan != 0)  && _manual_control_setpoint_sub.updated()) {
		manual_control_setpoint_s manual_control_setpoint{};
		_manual_control_setpoint_sub.copy(&manual_control_setpoint);

		float channel_setpoint = 0;
		if (osd_hg_rst_chan == MSP_OSD_AUX1) {
			channel_setpoint = manual_control_setpoint.aux1;

		} else if (osd_hg_rst_chan == MSP_OSD_AUX2) {
			channel_setpoint = manual_control_setpoint.aux2;
		}

		// Next ADD hysteresis for 0.3 seconds hold
		if (channel_setpoint > MSP_OSD_HEADING_RESET_THRESHOLD)
		{
			_osd_heading_origin = vehicle_yaw;
		}
	}

	const auto yaw_recentered = matrix::wrap(vehicle_yaw - _osd_heading_origin, 0.f, 360.f);

	const bool show_heading_as_clock_directions = _param_osd_disp_opts.get() & (1u << DisplayOptionIndex::SHOW_HEADING_AS_CLOCK_DIR);
	if (show_heading_as_clock_directions) {
		uint8_t clock_direction = ceilf(floorf(yaw_recentered / 15.0f) / 2.0f);
		// Display North, as 12 O'Clock
		if (clock_direction == 0)
		{
			clock_direction = 12;
		}

		char buffer[3];
		snprintf(buffer, sizeof(buffer), "%u", clock_direction);
		display.set(MessageDisplayType::HEADING, buffer);
	} else {
		// display north direction
		if (yaw_recentered <= 22.5f)
			display.set(MessageDisplayType::HEADING, "N");
		else if(yaw_recentered <= 67.5f)
			display.set(MessageDisplayType::HEADING, "NE");
		else if(yaw_recentered <= 112.5f)
			display.set(MessageDisplayType::HEADING, "E");
		else if(yaw_recentered <= 157.5f)
			display.set(MessageDisplayType::HEADING, "SE");
		else if(yaw_recentered <= 202.5f)
			display.set(MessageDisplayType::HEADING, "S");
		else if(yaw_recentered <= 247.5f)
			display.set(MessageDisplayType::HEADING, "SW");
		else if(yaw_recentered <= 292.5f)
			display.set(MessageDisplayType::HEADING, "W");
		else if(yaw_recentered <= 337.5f)
			display.set(MessageDisplayType::HEADING, "NW");
		else if(yaw_recentered <= 360.0f)
			display.set(MessageDisplayType::HEADING, "N");
	}

}

namespace msp_osd {

msp_fc_variant_t construct_FC_VARIANT() {
	// initialize result
	msp_fc_variant_t variant{};

	memcpy(variant.flightControlIdentifier, "BTFL", sizeof(variant.flightControlIdentifier));
	return variant;
}

msp_status_BF_t construct_STATUS(const vehicle_status_s &vehicle_status)
{
	// initialize result
	msp_status_BF_t status_BF = {0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status_BF.flight_mode_flags |= ARM_ACRO_BF;

		switch (vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:
			status_BF.flight_mode_flags |= STAB_BF;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			status_BF.flight_mode_flags |= RESC_BF;
			break;

		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			status_BF.flight_mode_flags |= FS_BF;
			break;

		default:
			status_BF.flight_mode_flags |= 0;
			break;
		}
	}

	status_BF.arming_disable_flags_count = 1;
	status_BF.arming_disable_flags  = !(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	return status_BF;
}

msp_analog_t construct_ANALOG(const battery_status_s &battery_status, const input_rc_s &input_rc)
{
	// initialize result
	msp_analog_t analog {0};

	analog.vbat = battery_status.voltage_v * 10; // bottom right... v * 10
	analog.rssi = (uint16_t)((input_rc.link_quality * 1023.0f) / 100.0f);
	analog.amperage = battery_status.current_a * 100; // main amperage
	analog.mAhDrawn = battery_status.discharged_mah; // unused
	return analog;
}

msp_raw_gps_t construct_RAW_GPS(const sensor_gps_s &vehicle_gps_position,
				const airspeed_validated_s &airspeed_validated)
{
	// initialize result
	msp_raw_gps_t raw_gps {0};

	if (vehicle_gps_position.fix_type >= 2) {
		raw_gps.lat = vehicle_gps_position.lat;
		raw_gps.lon = vehicle_gps_position.lon;
		raw_gps.alt =  vehicle_gps_position.alt / 10;

		float course = math::degrees(vehicle_gps_position.cog_rad);

		if (course < 0) {
			course += 360.0f;
		}

		raw_gps.groundCourse = course * 100.0f; // centidegrees

	} else {
		raw_gps.lat = 0;
		raw_gps.lon = 0;
		raw_gps.alt = 0;
		raw_gps.groundCourse = 0; // centidegrees
	}

	raw_gps.groundCourse = 0; // centidegrees

	if (vehicle_gps_position.fix_type == 0
	    || vehicle_gps_position.fix_type == 1) {
		raw_gps.fixType = MSP_GPS_NO_FIX;

	} else if (vehicle_gps_position.fix_type == 2) {
		raw_gps.fixType = MSP_GPS_FIX_2D;

	} else if (vehicle_gps_position.fix_type >= 3 && vehicle_gps_position.fix_type <= 5) {
		raw_gps.fixType = MSP_GPS_FIX_3D;

	} else {
		raw_gps.fixType = MSP_GPS_NO_FIX;
	}

	//raw_gps.hdop = vehicle_gps_position_struct.hdop
	raw_gps.numSat = vehicle_gps_position.satellites_used;

	if (airspeed_validated.airspeed_sensor_measurement_valid
	    && PX4_ISFINITE(airspeed_validated.indicated_airspeed_m_s)
	    && airspeed_validated.indicated_airspeed_m_s > 0) {
		raw_gps.groundSpeed = airspeed_validated.indicated_airspeed_m_s * 100;

	} else {
		raw_gps.groundSpeed = 0;
	}

	return raw_gps;
}

msp_comp_gps_t construct_COMP_GPS(const home_position_s &home_position,
				  const estimator_status_s &estimator_status,
				  const vehicle_global_position_s &vehicle_global_position,
				  const bool heartbeat)
{
	// initialize result
	msp_comp_gps_t comp_gps {0};

	// Calculate distance and direction to home
	if (home_position.valid_hpos
	    && home_position.valid_lpos
	    && estimator_status.solution_status_flags & (1 << 4)) {
		float bearing_to_home = math::degrees(get_bearing_to_next_waypoint(vehicle_global_position.lat,
						      vehicle_global_position.lon,
						      home_position.lat, home_position.lon));

		if (bearing_to_home < 0) {
			bearing_to_home += 360.0f;
		}

		float distance_to_home = get_distance_to_next_waypoint(vehicle_global_position.lat,
					 vehicle_global_position.lon,
					 home_position.lat, home_position.lon);

		comp_gps.distanceToHome = (int16_t)distance_to_home; // meters
		comp_gps.directionToHome = bearing_to_home;

	} else {
		comp_gps.distanceToHome = 0; // meters
		comp_gps.directionToHome = 0;
	}

	comp_gps.heartbeat = heartbeat;
	return comp_gps;
}

msp_attitude_t construct_ATTITUDE(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_attitude_t attitude {0};

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	attitude.pitch = math::degrees(euler_attitude.theta()) * 10;
	attitude.roll = math::degrees(euler_attitude.phi()) * 10;
	//attitude.yaw = math::degrees(euler_attitude.psi()) * 10;

	float yaw_fixed = math::degrees(euler_attitude.psi());

	if (yaw_fixed < 0) {
		yaw_fixed += 360.0f;
	}

	attitude.yaw = yaw_fixed;

	//attitude.yaw = 360;

	return attitude;
}

msp_altitude_t construct_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
				  const estimator_status_s &estimator_status,
				  const vehicle_local_position_s &vehicle_local_position)
{
	// initialize result
	msp_altitude_t altitude {0};

	if (vehicle_gps_position.fix_type >= 2) {
		altitude.estimatedActualPosition = vehicle_gps_position.alt / 10;

	} else {
		altitude.estimatedActualPosition = 0;
	}

	if (estimator_status.solution_status_flags & (1 << 5)) {
		altitude.estimatedActualVelocity = -vehicle_local_position.vz * 10; //m/s to cm/s

	} else {
		altitude.estimatedActualVelocity = 0;
	}

	return altitude;
}

msp_esc_sensor_data_dji_t construct_ESC_SENSOR_DATA()
{
	// initialize result
	msp_esc_sensor_data_dji_t esc_sensor_data {0};

	esc_sensor_data.rpm = 0;
	esc_sensor_data.temperature = 50;

	return esc_sensor_data;
}

} // namespace msp_osd

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
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/modes/ui.hpp>
#include <matrix/math.hpp>

// clock access
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
using namespace time_literals;

#include "uorb_to_msp.hpp"
#include <drivers/drv_hrt.h>
namespace msp_osd
{
typedef enum {
	MSP_DP_HEARTBEAT = 0,         // Release the display after clearing and updating
	MSP_DP_RELEASE = 1,         // Release the display after clearing and updating
	MSP_DP_CLEAR_SCREEN = 2,    // Clear the display
	MSP_DP_WRITE_STRING = 3,    // Write a string at given coordinates
	MSP_DP_DRAW_SCREEN = 4,     // Trigger a screen draw
	MSP_DP_OPTIONS = 5,         // Not used by Betaflight. Reserved by Ardupilot and INAV
	MSP_DP_SYS = 6,             // Display system element displayportSystemElement_e at given coordinates
	MSP_DP_COUNT,
} displayportMspSubCommand;

msp_name_t construct_display_message(const vehicle_status_s &vehicle_status,
				     const vehicle_attitude_s &vehicle_attitude,
				     const log_message_s &log_message,
				     const int log_level,
				     MessageDisplay &display)
{
	// initialize result
	msp_name_t display_message {0};

	const auto now = hrt_absolute_time();
	static uint64_t last_warning_stamp {0};

	// update arming state, flight mode, and warnings, if current
	if (vehicle_status.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::ARMING, "???");
		display.set(MessageDisplayType::FLIGHT_MODE, "???");

	} else {
		// display armed / disarmed
		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			display.set(MessageDisplayType::ARMING, "ARM");

		} else {
			display.set(MessageDisplayType::ARMING, "DSRM");
		}

		// display flight mode
		display.set(MessageDisplayType::FLIGHT_MODE, mode_util::nav_state_names[vehicle_status.nav_state]);
	}

	// display, if updated
	if (log_message.severity <= log_level) {
		display.set(MessageDisplayType::WARNING, log_message.text);
		last_warning_stamp = now;

	} else if (now - last_warning_stamp > 30_s) {
		// clear warning after timeout
		display.set(MessageDisplayType::WARNING, "");
		last_warning_stamp = now;
	}

	// update heading, if relatively recent
	if (vehicle_attitude.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::HEADING, "N?");

	} else {
		// convert to YAW
		matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
		const auto yaw = math::degrees(euler_attitude.psi());

		// display north direction
		if (yaw <= 22.5f) {
			display.set(MessageDisplayType::HEADING, "N");

		} else if (yaw <= 67.5f) {
			display.set(MessageDisplayType::HEADING, "NE");

		} else if (yaw <= 112.5f) {
			display.set(MessageDisplayType::HEADING, "E");

		} else if (yaw <= 157.5f) {
			display.set(MessageDisplayType::HEADING, "SE");

		} else if (yaw <= 202.5f) {
			display.set(MessageDisplayType::HEADING, "S");

		} else if (yaw <= 247.5f) {
			display.set(MessageDisplayType::HEADING, "SW");

		} else if (yaw <= 292.5f) {
			display.set(MessageDisplayType::HEADING, "W");

		} else if (yaw <= 337.5f) {
			display.set(MessageDisplayType::HEADING, "NW");

		} else if (yaw <= 360.0f) {
			display.set(MessageDisplayType::HEADING, "N");
		}
	}

	// update message and return
	display.get(display_message.craft_name, hrt_absolute_time());
	return display_message;
}

msp_fc_variant_t construct_FC_VARIANT()
{
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

msp_rendor_rssi_t construct_rendor_RSSI(const input_rc_s &input_rc)
// print the rssi of the rc as a percentage in the top left corner
{
	msp_rendor_rssi_t rssi;
	rssi.screenYPosition = 0x02;
	rssi.screenXPosition = 0x02;

	snprintf(&rssi.str[0], sizeof(rssi.str), "%3d", input_rc.link_quality);
	rssi.str[3] = '%';

	return rssi;
}

msp_battery_state_t construct_BATTERY_STATE(const battery_status_s &battery_status)
{
	// initialize result
	msp_battery_state_t battery_state = {0};

	// MSP_BATTERY_STATE
	battery_state.amperage = battery_status.current_a * 100.0f; // Used for power element
	battery_state.batteryVoltage = (uint16_t)((battery_status.voltage_v / battery_status.cell_count) * 400.0f);  // OK
	battery_state.mAhDrawn = battery_status.discharged_mah ; // OK
	battery_state.batteryCellCount = battery_status.cell_count;
	battery_state.batteryCapacity = battery_status.capacity; // not used?

	// Voltage color 0==white, 1==red
	if (battery_status.voltage_v < 14.4f) {
		battery_state.batteryState = 1;

	} else {
		battery_state.batteryState = 0;
	}

	battery_state.legacyBatteryVoltage = battery_status.voltage_v * 10;
	return battery_state;
}

msp_rendor_battery_state_t construct_rendor_BATTERY_STATE(const battery_status_s &battery_status)
{
	// initialize result
	msp_rendor_battery_state_t battery_state = {0};

	battery_state.subCommand = MSP_DP_WRITE_STRING; // 3 write string. fixed
	battery_state.screenYPosition = 0x04;
	battery_state.screenXPosition = 0x02;
	battery_state.iconAttrs = 0x00;

	float sigle_cell_v = battery_status.voltage_v / battery_status.cell_count;

	if (sigle_cell_v > 4.0f) {
		battery_state.iconIndex = 0x91; // Full battery Icon

	} else if ((sigle_cell_v <= 4.0f) && (sigle_cell_v > 3.5f)) {
		battery_state.iconIndex = 0x93; // Half battery Icon

	} else if ((sigle_cell_v <= 3.5f) && (sigle_cell_v > 3.2f)) {
		battery_state.iconIndex = 0x95; // Empty battery Icon

	} else {
		battery_state.iconIndex = 0x96; // Dead battery Icon
	}

	snprintf(&battery_state.str[0], sizeof(battery_state.str), "%.1fV", (double)sigle_cell_v);
	return battery_state;
}



msp_raw_gps_t construct_RAW_GPS(const sensor_gps_s &vehicle_gps_position,
				const airspeed_validated_s &airspeed_validated)
{
	// initialize result
	msp_raw_gps_t raw_gps {0};

	if (vehicle_gps_position.fix_type >= 2) {
		raw_gps.lat = static_cast<int32_t>(vehicle_gps_position.latitude_deg * 1e7);
		raw_gps.lon = static_cast<int32_t>(vehicle_gps_position.longitude_deg * 1e7);
		raw_gps.alt = static_cast<int16_t>(vehicle_gps_position.altitude_msl_m * 100.0);

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

	if (airspeed_validated.airspeed_source >= airspeed_validated_s::GROUND_MINUS_WIND
	    && PX4_ISFINITE(airspeed_validated.indicated_airspeed_m_s)
	    && airspeed_validated.indicated_airspeed_m_s > 0.f) {
		raw_gps.groundSpeed = airspeed_validated.indicated_airspeed_m_s * 100;

	} else {
		raw_gps.groundSpeed = 0;
	}

	return raw_gps;
}

msp_rendor_latitude_t construct_rendor_GPS_LAT(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_latitude_t lat;

	lat.screenYPosition = 0x0A;
	lat.screenXPosition = 0x29;

	if (vehicle_gps_position.fix_type >= 2) {
		snprintf(&lat.str[0], sizeof(lat.str), "%.6f", vehicle_gps_position.latitude_deg);

	} else {
		snprintf(&lat.str[0], sizeof(lat.str), "%.6f", 0.0);
	}

	return lat;
}

msp_rendor_longitude_t construct_rendor_GPS_LON(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_longitude_t lon;

	lon.screenYPosition = 0x09;
	lon.screenXPosition = 0x29;

	if (vehicle_gps_position.fix_type >= 2) {
		snprintf(&lon.str[0], sizeof(lon.str), "%.6f", vehicle_gps_position.longitude_deg);

	} else {
		snprintf(&lon.str[0], sizeof(lon.str), "%.6f", -0.0);
	}

	return lon;
}

msp_rendor_satellites_used_t construct_rendor_GPS_NUM(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_satellites_used_t num;

	num.screenYPosition = 0x08;
	num.screenXPosition = 0x30;

	memset(&num.str[0], 0, sizeof(num.str));
	snprintf(&num.str[0], sizeof(num.str), "%d", vehicle_gps_position.satellites_used);

	return num;
}


msp_comp_gps_t construct_COMP_GPS(const home_position_s &home_position,
				  const vehicle_global_position_s &vehicle_global_position,
				  const bool heartbeat)
{
	// initialize result
	msp_comp_gps_t comp_gps {0};

	// Calculate distance and direction to home
	if (home_position.valid_hpos
	    && home_position.valid_lpos
	    && (hrt_elapsed_time(&vehicle_global_position.timestamp) < 1_s)) {

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

msp_rendor_distanceToHome_t construct_rendor_distanceToHome(const home_position_s &home_position,
		const vehicle_global_position_s &vehicle_global_position)
{
	msp_rendor_distanceToHome_t distance;

	distance.screenYPosition = 0x08;
	distance.screenXPosition = 0x02;

	int16_t dist_i = 0;

	if (home_position.valid_hpos
	    && home_position.valid_lpos
	    && (hrt_elapsed_time(&vehicle_global_position.timestamp) < 1_s)) {

		float distance_to_home = get_distance_to_next_waypoint(vehicle_global_position.lat,
					 vehicle_global_position.lon,
					 home_position.lat, home_position.lon);

		dist_i = (int16_t)distance_to_home; // meters

	}

	memset(&distance.str[0], 0, sizeof(distance.str));
	snprintf(&distance.str[0], sizeof(distance.str), "%d", dist_i); // 65536

	return distance;
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

msp_rendor_pitch_t  construct_rendor_PITCH(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_rendor_pitch_t pit;

	pit.screenYPosition = 0x0D;
	pit.screenXPosition = 0x30;

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	double pitch_deg = (double)math::degrees(euler_attitude.theta());
	// attitude.roll = math::degrees(euler_attitude.phi()) * 10;

	memset(&pit.str[0], 0, sizeof(pit.str));
	snprintf(&pit.str[0], sizeof(pit.str), "%.1f", pitch_deg);

	return pit;
}

msp_rendor_roll_t  construct_rendor_ROLL(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_rendor_roll_t roll;

	roll.screenYPosition = 0x0E;
	roll.screenXPosition = 0x30;

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	// double pitch = (double)math::degrees(euler_attitude.theta());
	double roll_deg = (double)math::degrees(euler_attitude.phi());

	memset(&roll.str[0], 0, sizeof(roll.str));
	snprintf(&roll.str[0], sizeof(roll.str), "%.1f", roll_deg);

	return roll;
}


msp_altitude_t construct_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
				  const vehicle_local_position_s &vehicle_local_position)
{
	// initialize result
	msp_altitude_t altitude {0};

	if (vehicle_gps_position.fix_type >= 2) {
		altitude.estimatedActualPosition = static_cast<int32_t>(vehicle_gps_position.altitude_msl_m * 100.0);	// cm

	} else {
		altitude.estimatedActualPosition = 0;
	}

	if (vehicle_local_position.v_z_valid) {
		altitude.estimatedActualVelocity = -vehicle_local_position.vz * 100; //m/s to cm/s

	} else {
		altitude.estimatedActualVelocity = 0;
	}

	return altitude;
}

msp_rendor_altitude_t construct_Rendor_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
		const vehicle_local_position_s &vehicle_local_position)
{
	msp_rendor_altitude_t altitude;

	altitude.screenYPosition = 0x10;
	altitude.screenXPosition = 0x02;

	double alt;

	if (vehicle_gps_position.fix_type >= 2) {
		alt = vehicle_gps_position.altitude_msl_m;

	} else {
		alt = (double)(vehicle_local_position.z * -1.0f);
	}

	memset(&altitude.str[0], 0, sizeof(altitude.str));
	snprintf(&altitude.str[0], sizeof(altitude.str), "%.1f", alt);

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

msp_rc_t construct_MSP_RC(const input_rc_s &input_rc)
{
	// initialize result
	msp_rc_t rc;

	rc.channelValue[0] = input_rc.values[0]; // roll
	rc.channelValue[1] = input_rc.values[1]; // pitch
	rc.channelValue[2] = input_rc.values[3]; // yaw
	rc.channelValue[3] = input_rc.values[2]; // Throttle
	return rc;
}

msp_status_t construct_MSP_STATUS(const vehicle_status_s &vehicle_status)
{	
	// initialize result
	msp_status_t status{0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status.flightModeFlags |= (1 << MSP_MODE_ARM);
	}

	return status;
}



msp_rendor_battery_state_t construct_rendor_BATTERY_FULL_VOLTAGE(const battery_status_s &battery_status)
{
	// initialize result
	msp_rendor_battery_state_t battery_state = {0};

	battery_state.subCommand = MSP_DP_WRITE_STRING; // 3 write string. fixed
	battery_state.screenYPosition = 0x05; // Position below single cell voltage
	battery_state.screenXPosition = 0x02;
	battery_state.iconAttrs = 0x00;
	battery_state.iconIndex = 0x91; // Full battery icon (same as single cell full battery)(145 dec)

	// Display full battery voltage (e.g., "14.8V" for 4S battery)
	snprintf(&battery_state.str[0], sizeof(battery_state.str), "%.1fV", (double)battery_status.voltage_v);
	return battery_state;
}


msp_rendor_distance_sensor_t construct_rendor_DISTANCE_SENSOR(const estimator_aid_source1d_s &estimator_aid_src_rng_hgt)
{
	msp_rendor_distance_sensor_t distance = {0}; // Initialize all fields to zero

	distance.subCommand = MSP_DP_WRITE_STRING; // 0x03 Write string
	distance.screenYPosition = 0x07;
	distance.screenXPosition = 0x02;
	distance.iconAttrs = 0x00;
	distance.iconIndex = 0xB1; // Use altitude icon (similar to distance/height measurement)

	// Use the observation from the estimator aid source (processed range finder height)

	uint64_t time_diff = hrt_absolute_time() - estimator_aid_src_rng_hgt.time_last_fuse;
	if ( time_diff > 1000_ms ) {
		memset(&distance.str[0], 0, sizeof(distance.str));
		snprintf(&distance.str[0], sizeof(distance.str), "N.A");
	}
	else {
		float dist = estimator_aid_src_rng_hgt.observation;
		if (PX4_ISFINITE(dist) && estimator_aid_src_rng_hgt.fused) {
			memset(&distance.str[0], 0, sizeof(distance.str));
			snprintf(&distance.str[0], sizeof(distance.str), "%.2f", static_cast<double>(dist));
		}
		else {
			memset(&distance.str[0], 0, sizeof(distance.str));
			snprintf(&distance.str[0], sizeof(distance.str), "N.A");
		}
	}
	return distance;
}


msp_baro_altitude_t construct_rendor_BARO_ALT(const estimator_aid_source1d_s &__orb_estimator_aid_src_baro_hgt)
{
	msp_baro_altitude_t baro_altitude = {0}; // Initialize all fields to zero

	baro_altitude.subCommand = MSP_DP_WRITE_STRING; // 0x03 Write string
	baro_altitude.screenYPosition = 0x06;
	baro_altitude.screenXPosition = 0x02;
	baro_altitude.iconAttrs = 0x00;
	baro_altitude.iconIndex = 0x7F; // Use altitude icon

	float alt = __orb_estimator_aid_src_baro_hgt.observation;
	if (PX4_ISFINITE(alt) && __orb_estimator_aid_src_baro_hgt.fused) {
		memset(&baro_altitude.str[0], 0, sizeof(baro_altitude.str));
		snprintf(&baro_altitude.str[0], sizeof(baro_altitude.str), "%.2f", static_cast<double>(alt));
	}
	else {
		memset(&baro_altitude.str[0], 0, sizeof(baro_altitude.str));
		snprintf(&baro_altitude.str[0], sizeof(baro_altitude.str), "N.A");
	}
	return baro_altitude;
}



msp_rendor_formic_ring_t construct_rendor_FORMIC_RING(const dds_flag_s &dds_flag)
{
	msp_rendor_formic_ring_t formic_ring{};

	formic_ring.subCommand = MSP_DP_WRITE_STRING; // 0x03 subcommand write string. fixed
	formic_ring.screenYPosition = 0x02;
	formic_ring.screenXPosition = 0x01;
	formic_ring.iconAttrs = 0x00;
	formic_ring.iconIndex = 0x00; // no icon

	if (dds_flag.dds_connected) {
		snprintf(&formic_ring.str[0], sizeof(formic_ring.str), "FORMIC:V");
	} else {
		snprintf(&formic_ring.str[0], sizeof(formic_ring.str), "FORMIC:X");
	}

	return formic_ring;
}

msp_rendor_formic_crosshairs_t construct_rendor_FORMIC_CROSSHAIRS(int osd_format)
{
	msp_rendor_formic_crosshairs_t crosshairs = {0}; // Initialize all fields to zero

	crosshairs.subCommand = MSP_DP_WRITE_STRING; // 0x06 Display system element
	crosshairs.screenYPosition = 0x09; // Center vertically (10 = 21/2 rounded down)
	crosshairs.screenXPosition = 0x19; // Center horizontally (29 = 59/2 rounded down)
	crosshairs.systemElement = 0x00; // Crosshairs system element ID (0x00 = CROSSHAIR)
	
	if (osd_format == 0) {
		crosshairs.iconIndex = MCP_ARDUPILOT(MSP_ICON_AUTOCONFIG_CROSSHAIRS); // ardupilotconfig
	} else {
		crosshairs.iconIndex = MSP_ICON_AUTOCONFIG_CROSSHAIRS; // autoconfig
	}

	return crosshairs;
}


msp_rendor_total_arm_time_t construct_rendor_TOTAL_ACTIVATED_TIME(const vehicle_status_s &vehicle_status)
// print the time from system start to the current time in mm:ss format
{
	msp_rendor_total_arm_time_t render_total_arm_time = {}; // Initialize all fields to zero

	render_total_arm_time.subCommand = MSP_DP_WRITE_STRING; // 0x03 Write string
	render_total_arm_time.screenYPosition = 0x08;
	render_total_arm_time.screenXPosition = 0x02;
	render_total_arm_time.iconAttrs = 0x00;
	render_total_arm_time.iconIndex = MCP_TIMER_ICON; // Timer/clock icon (common Betaflight timer icon index)

	// Convert microseconds to mm:ss format
	// timestamp is in microseconds since system start
	uint64_t total_seconds = vehicle_status.timestamp / 1000000ULL; // Convert microseconds to seconds
	int minutes = total_seconds / 60;
	int seconds = total_seconds % 60;

	// Format as "mm:ss" (e.g., "05:23" for 5 minutes 23 seconds)
	memset(&render_total_arm_time.str[0], 0, sizeof(render_total_arm_time.str));
	snprintf(&render_total_arm_time.str[0], sizeof(render_total_arm_time.str), "%02d:%02d", minutes, seconds);

	return render_total_arm_time;
}


msp_rendor_total_arm_time_t construct_rendor_TOTAL_ARM_TIME(const total_arm_time_s &total_arm_time)
{
	msp_rendor_total_arm_time_t render_total_arm_time = {}; // Initialize all fields to zero

	render_total_arm_time.subCommand = MSP_DP_WRITE_STRING; // 0x03 Write string
	render_total_arm_time.screenYPosition = 0x09;
	render_total_arm_time.screenXPosition = 0x02;
	render_total_arm_time.iconAttrs = 0x00;
	render_total_arm_time.iconIndex = MCP_TIMER_ICON; // Timer/clock icon (common Betaflight timer icon index)

	uint64_t total_seconds = total_arm_time.total_arm_time_ms / 1000;
	int minutes = total_seconds / 60;
	int seconds = total_seconds % 60;

	// Format as "mm:ss" (e.g., "05:23" for 5 minutes 23 seconds)
	memset(&render_total_arm_time.str[0], 0, sizeof(render_total_arm_time.str));
	snprintf(&render_total_arm_time.str[0], sizeof(render_total_arm_time.str), "%02d:%02d", minutes, seconds);

	return render_total_arm_time;
}


msp_rendor_formic_vision_quality_t construct_rendor_FORMIC_VISION_QUALITY(const vehicle_odometry_s &vehicle_vision_odometry)
{
	msp_rendor_formic_vision_quality_t vision_quality = {};
	vision_quality.subCommand = MSP_DP_WRITE_STRING; 
	vision_quality.screenYPosition = 0x0A;
	vision_quality.screenXPosition = 0x02;

	const uint64_t VISION_TIMEOUT_US = 1000000; // 1 second
	const uint64_t data_age = hrt_absolute_time() - vehicle_vision_odometry.timestamp;

	// 1. Group all validity conditions into one clean, readable boolean
	bool is_valid = (vehicle_vision_odometry.timestamp != 0) &&  // Has it ever received data?
			(data_age <= VISION_TIMEOUT_US) &&           // Is the data fresh?
			(vehicle_vision_odometry.quality != -1);     // Is the quality value valid?

	// 2. Clear the string buffer exactly like in the TOTAL_ARM_TIME function
	memset(&vision_quality.str[0], 0, sizeof(vision_quality.str));

	// 3. Format the string using the same &...str[0] syntax
	if (is_valid) {
		snprintf(&vision_quality.str[0], sizeof(vision_quality.str), "VQ:%d", vehicle_vision_odometry.quality);
	} else {
		snprintf(&vision_quality.str[0], sizeof(vision_quality.str), "VQ:N/A");
	}

	return vision_quality;
}

} // namespace msp_osd

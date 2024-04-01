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

#include "uorb_to_msp_dp.hpp"

namespace msp_dp_osd
{

void log_msg_to_upper(char* string){
	// Convert string to uppercase, otherwise it will try to print symbols instead
	for(size_t i=0; i<strlen(string);++i){
		string[i] = (string[i] >= 'a' && string[i] <= 'z') ? string[i] - 'a' + 'A' : string[i];
	}
}

msp_name_t construct_display_message(const vehicle_status_s &vehicle_status,
				     const vehicle_attitude_s &vehicle_attitude,
				     const log_message_s &log_message,
				     const int log_level,
				     msp_osd::MessageDisplay &display)
{
	// initialize result
	msp_name_t display_message {0};

	const auto now = hrt_absolute_time();
	static uint64_t last_warning_stamp {0};

	// update arming state, flight mode, and warnings, if current
	if (vehicle_status.timestamp < (now - 1_s)) {
		display.set(msp_osd::MessageDisplayType::ARMING, "???");
		display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "???");

	} else {
		// display armed / disarmed
		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			display.set(msp_osd::MessageDisplayType::ARMING, "ARM");

		} else {
			display.set(msp_osd::MessageDisplayType::ARMING, "DSRM");
		}

		// display flight mode
		switch (vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "MANUAL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "ALTCTL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "POSCTL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_MISSION");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_LOITER");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_RTL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_UNUSED:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "UNUSED");
			break;

		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "ACRO");
			break;

		case vehicle_status_s::NAVIGATION_STATE_UNUSED1:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "UNUSED1");
			break;

		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "DESCEND");
			break;

		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "TERMINATION");
			break;

		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "OFFBOARD");
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "STAB");
			break;

		case vehicle_status_s::NAVIGATION_STATE_UNUSED2:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "UNUSED2");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_TAKEOFF");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_LAND");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_FOLLOW_TARGET");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_PRECLAND");
			break;

		case vehicle_status_s::NAVIGATION_STATE_ORBIT:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "ORBIT");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "AUTO_VTOL_TAKEOFF");
			break;

		case vehicle_status_s::NAVIGATION_STATE_MAX:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "MAX");
			break;

		default:
			display.set(msp_osd::MessageDisplayType::FLIGHT_MODE, "???");
		}
	}

	// display, if updated
	if (log_message.severity <= log_level) {
		char* log_msg = (char*)log_message.text;
		log_msg_to_upper(log_msg);
		display.set(msp_osd::MessageDisplayType::WARNING, log_msg);
		last_warning_stamp = now;

	} else if (now - last_warning_stamp > 30_s) {
		// clear warning after timeout
		display.set(msp_osd::MessageDisplayType::WARNING, "");
		last_warning_stamp = now;
	}

	// update heading, if relatively recent
	if (vehicle_attitude.timestamp < (now - 1_s)) {
		display.set(msp_osd::MessageDisplayType::HEADING, "N?");

	} else {
		// convert to YAW
		matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
		const auto yaw = math::degrees(euler_attitude.psi());

		// display north direction
		if (yaw <= 22.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "N");

		} else if (yaw <= 67.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "NE");

		} else if (yaw <= 112.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "E");

		} else if (yaw <= 157.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "SE");

		} else if (yaw <= 202.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "S");

		} else if (yaw <= 247.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "SW");

		} else if (yaw <= 292.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "W");

		} else if (yaw <= 337.5f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "NW");

		} else if (yaw <= 360.0f) {
			display.set(msp_osd::MessageDisplayType::HEADING, "N");
		}
	}

	// update message and return
	display.get(display_message.craft_name, hrt_absolute_time());
	return display_message;
}

// Construct VTX configuration
msp_dp_vtx_config_t construct_vtx_config(uint8_t band=5, uint8_t channel=1){
	msp_dp_vtx_config_t vtx_config {0};

	vtx_config.protocol = 5; 		// MSP
	vtx_config.band 	= band; 	// BAND (R=5, F=4, E=3) 
	vtx_config.channel 	= channel; 	// CHANNEL (R=1-8, F=1,2,4, E=1)
	vtx_config.power 	= 1; 		// POWER LEVEL 1 -> 25mW (0mW, 25mW, 200mW)
	vtx_config.pit	 	= 0; 		// PIT MODE OFF
	vtx_config.freq 	= 0x161A;	// 5658 MHz

	return vtx_config;
}

// Vehicle status message (arm state)
msp_dp_status_t construct_status(const vehicle_status_s &vehicle_status){
	msp_dp_status_t status = {0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status.armed = 0x01;
	}

	status.arming_disable_flags_count = 1;
	status.arming_disable_flags  = !(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	return status;
}

// RC channels message (used for goggle menus)
msp_rc_t construct_RC(const input_rc_s &input_rc,  const msp_dp_rc_sticks_t &sticks){
	/*
	HDZERO expects:
	CH 1 - Roll
	CH 2 - Pitch
	CH 3 - Yaw
	CH 4 - Throttle
	*/
	msp_rc_t msp_rc{0};
	
	for (int i=0; i < MSP_MAX_SUPPORTED_CHANNELS; ++i){
		msp_rc.channelValue[i] = input_rc.values[i];
	}
	uint16_t throttle{msp_rc.channelValue[sticks.throttle-1]};	
	uint16_t roll{msp_rc.channelValue[sticks.roll-1]};			
	uint16_t pitch{msp_rc.channelValue[sticks.pitch-1]};		
	uint16_t yaw{msp_rc.channelValue[sticks.yaw-1]};			
	msp_rc.channelValue[0] = roll;
	msp_rc.channelValue[1] = pitch;	
	msp_rc.channelValue[2] = yaw;		
	msp_rc.channelValue[3] = throttle;	
	return msp_rc;
}

// OSD setup message
msp_dp_canvas_t construct_OSD_canvas(uint8_t row, uint8_t col){
	msp_dp_canvas_t msp_canvas{0};

	// HD 5018
	if (row > 49) row = 49;
	if (col > 17) col = 17;
	msp_canvas.row_max = row;
	msp_canvas.col_max = col;
	return msp_canvas;
}

// Construct a heartbeat command
displayportMspCommand_e construct_OSD_heartbeat(){
	return MSP_DP_HEARTBEAT;
}

// Construct a release command
displayportMspCommand_e construct_OSD_release(){
	return MSP_DP_RELEASE;
}

// Construct a clear command
displayportMspCommand_e construct_OSD_clear(){
	return MSP_DP_CLEAR_SCREEN;
}

// Construct a write command into an output buffer given location, string, and # bytes to write 
// WARNING: If input string has lowercase chars, they may be interpreted as symbols!
uint8_t construct_OSD_write(uint8_t col, uint8_t row, bool blink, const char *string, uint8_t *output, uint8_t len)
{
	msp_dp_cmd_t msp_osd_dp_cmd;
	int str_len = strlen(string);
    if (str_len > MSP_OSD_MAX_STRING_LENGTH) str_len = MSP_OSD_MAX_STRING_LENGTH;
	msp_osd_dp_cmd.subcmd = (uint8_t)MSP_DP_WRITE_STRING;
	msp_osd_dp_cmd.row = row;
	msp_osd_dp_cmd.col = col;
	msp_osd_dp_cmd.attr = blink ? msp_osd_dp_cmd.attr | DISPLAYPORT_MSP_ATTR_BLINK : 0;	// Blink doesn't work with HDZero Freestyle V2 VTX
	memcpy(output, &msp_osd_dp_cmd, sizeof(msp_osd_dp_cmd));
	memcpy(&output[MSP_OSD_DP_WRITE_PAYLOAD], string, str_len);
	return 0;
}

// Construct a draw command
displayportMspCommand_e construct_OSD_draw(){
	return MSP_DP_DRAW_SCREEN;
}

// Construct a config command
msp_dp_config_t construct_OSD_config(resolutionType_e resolution, uint8_t fontType){
	msp_dp_config_t msp_osd_dp_config;
	msp_osd_dp_config.subcmd     = MSP_DP_CONFIG;
	msp_osd_dp_config.fontType   = fontType;
	msp_osd_dp_config.resolution = resolution;
	return msp_osd_dp_config;
}

// Construct Flight mode message
const char* construct_flight_mode(const vehicle_status_s &vehicle_status){
	const char* flight_mode;
	switch (vehicle_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		flight_mode = "MANUAL"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		flight_mode = "ALTCTL"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		flight_mode = "POSCTL"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		flight_mode = "AUTO_MISSION"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		flight_mode = "AUTO_LOITER"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		flight_mode = "AUTO_RTL"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_UNUSED:
		flight_mode = "UNUSED"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		flight_mode = "ACRO"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_UNUSED1:
		flight_mode = "UNUSED1"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		flight_mode = "DESCEND"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		flight_mode = "TERMINATION"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		flight_mode = "OFFBOARD"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		flight_mode = "STAB"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_UNUSED2:
		flight_mode = "UNUSED2"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		flight_mode = "AUTO_TAKEOFF"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		flight_mode = "AUTO_LAND"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		flight_mode = "AUTO_FOLLOW_TARGET"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		flight_mode = "AUTO_PRECLAND"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		flight_mode = "ORBIT"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		flight_mode = "AUTO_VTOL_TAKEOFF"; 
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		flight_mode = "MAX"; 
		break;

	default:
		flight_mode = "???"; 
		break;
	}
	return flight_mode;
}

// Generate bearing symbol from value
uint8_t get_symbol_from_bearing(float bearing){
	uint8_t bearing_sybmol{SYM_ARROW_NORTH};
	// NORTH
	if((bearing >= 0.0f && bearing <= 11.25f) || (bearing > 348.75f && bearing <= 360.0f))
	{
		bearing_sybmol = SYM_ARROW_NORTH;	
	}
	// NORTH-NORTH-EAST
	else if((bearing > 11.25f && bearing <= 33.75f))
	{
		bearing_sybmol = SYM_ARROW_8;	
	}
	// NORTH-EAST
	else if((bearing > 33.75f && bearing <= 56.25f))
	{
		bearing_sybmol = SYM_ARROW_7;	
	}
	// EAST-NORTH-EAST
	else if((bearing > 56.25f && bearing <= 78.75f))
	{
		bearing_sybmol = SYM_ARROW_6;	
	}
	// EAST
	else if((bearing > 78.75f && bearing <= 101.25f))
	{
		bearing_sybmol = SYM_ARROW_EAST;	
	}
	// EAST-SOUTH-EAST
	else if((bearing > 101.25f && bearing <= 123.75f))
	{
		bearing_sybmol = SYM_ARROW_4;	
	}
	// SOUTH-EAST
	else if((bearing > 123.75f && bearing <= 146.25f))
	{
		bearing_sybmol = SYM_ARROW_3;	
	}
	// SOUTH-SOUTH-EAST
	else if((bearing > 146.25f && bearing <= 168.75f))
	{
		bearing_sybmol = SYM_ARROW_2;	
	}
	// SOUTH
	else if((bearing > 168.75f && bearing <= 191.25f))
	{
		bearing_sybmol = SYM_ARROW_SOUTH;	
	}
	// SOUTH-SOUTH-WEST
	else if((bearing > 191.25f && bearing <= 213.75f))
	{
		bearing_sybmol = SYM_ARROW_16;	
	}
	// SOUTH-WEST
	else if((bearing > 213.75f && bearing <= 236.25f))
	{
		bearing_sybmol = SYM_ARROW_15;	
	}
	// WEST-SOUTH-WEST
	else if((bearing > 236.25f && bearing <= 258.75f))
	{
		bearing_sybmol = SYM_ARROW_14;	
	}
	// WEST
	else if((bearing > 258.75f && bearing <= 281.25f))
	{
		bearing_sybmol = SYM_ARROW_WEST;	
	}
	// WEST-NORTH-WEST
	else if((bearing > 281.25f && bearing <= 303.75f))
	{
		bearing_sybmol = SYM_ARROW_12;	
	}
	// NORTH-WEST
	else if((bearing > 303.75f && bearing <= 326.25f))
	{
		bearing_sybmol = SYM_ARROW_11;	
	}
	// NORTH-NORTH-WEST
	else if((bearing > 326.25f && bearing <= 348.75f))
	{
		bearing_sybmol = SYM_ARROW_10;	
	}
	return bearing_sybmol;
}

} // namespace msp_dp_osd

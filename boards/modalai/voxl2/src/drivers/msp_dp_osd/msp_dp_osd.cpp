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

/* Notes:
 *  - Currently there's a lot of wasted processing here if certain displays are enabled.
 *    A relatively low-hanging fruit would be figuring out which display elements require
 *    information from what UORB topics and disable if the information isn't displayed.
 * 	(this is complicated by the fact that it's not a one-to-one mapping...)
 */

#include "msp_dp_osd.hpp"

#include "msp_dp_defines.h"
#include <drivers/osd/msp_osd/msp_defines.h>

#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <string>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>

#include <lib/geo/geo.h>

#include "MspDPV1.hpp"

bool clear{true};

MspDPOsd::MspDPOsd(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
	_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);

	_warning.set_period(_param_osd_scroll_rate.get() * 1000ULL);
	_warning.set_dwell(_param_osd_dwell_time.get() * 1000ULL);
	_warning.set(MessageDisplayType::FLIGHT_MODE, "");
	_warning.set(MessageDisplayType::ARMING, "");
	_warning.set(MessageDisplayType::HEADING, "");

	// back up device name for connection later
	strcpy(_device, device);

	PX4_INFO("MSP DP OSD running on %s", _device);
}

MspDPOsd::~MspDPOsd()
{
	if(_msp_fd) close(_msp_fd);
}

bool MspDPOsd::init()
{
	// Initialize remote OSD data
	for (int i = 0; i < MAX_REMOTE_OSD_FIELDS; i++) {
		_remote_osd[i].row = -1;
		_remote_osd[i].col = -1;
	}

	_vehicle_mavlink_tunnel_sub.registerCallback();

	ScheduleOnInterval(100_ms);

	return true;
}


void MspDPOsd::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_update();
	}

	// Check if any remote OSD updates
	if (_remote_enable && _vehicle_mavlink_tunnel_sub.updated()) {
		mavlink_tunnel_s remote_osd{};
		while (_vehicle_mavlink_tunnel_sub.update(&remote_osd)) {
			if (remote_osd.payload_type == 0xfff0) {
				int row = (int) remote_osd.payload[0];
				int col = (int) remote_osd.payload[1];
				char *osd_string = (char*) &remote_osd.payload[2];
				// PX4_INFO("Remote OSD: %u %u %s", row, col, osd_string);
				for (int i = 0; i < MAX_REMOTE_OSD_FIELDS; i++) {
					// See if entry already exists for this string
					if ((row == _remote_osd[i].row) && (col == _remote_osd[i].col)) {
						// PX4_INFO("Updating entry %d", i);
						strncpy(_remote_osd[i].string, osd_string, MAX_REMOTE_OSD_STRING_LEN);
						break;
					}
					// If we get to unallocated entries then this is a new remote string
					if (_remote_osd[i].row == -1) {
						// PX4_INFO("Adding entry %d", i);
						_remote_osd[i].row = row;
						_remote_osd[i].col = col;
						strncpy(_remote_osd[i].string, osd_string, MAX_REMOTE_OSD_STRING_LEN);
						break;
					}
				}
			}
		}
		// Updates are signaled via callback and happen outside of standard
		// OSD refresh cycles so just return here and normal cycle processing
		// will handle the refresh.
		return;
	}

	// perform first time initialization, if needed
	if (!_is_initialized) {
		struct termios t;
		_msp_fd = open(_device, O_RDWR | O_NONBLOCK);

		if (_msp_fd < 0) {
			_performance_data.initialization_problems = true;
			return;
		}

		tcgetattr(_msp_fd, &t);
		cfsetspeed(&t, B115200);
		t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
		t.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		t.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
		t.c_oflag = 0;
		tcsetattr(_msp_fd, TCSANOW, &t);

		_msp = MspDPV1(_msp_fd);

		_is_initialized = true;

		// Clear old info on OSD
		const auto clear_osd_msg = msp_dp_osd::construct_OSD_clear();
		this->Send(MSP_CMD_DISPLAYPORT, &clear_osd_msg, MSP_DIRECTION_REPLY);

		// Send OSD Canvas size
		const auto osd_canvas_msg = msp_dp_osd::construct_OSD_canvas(row_max[resolution], column_max[resolution]);
		this->Send(MSP_SET_OSD_CANVAS, &osd_canvas_msg, MSP_DIRECTION_REPLY);
	}

	const auto osd_config_msg = msp_dp_osd::construct_OSD_config(this->resolution, this->fontType);
	this->Send(MSP_CMD_DISPLAYPORT, &osd_config_msg, MSP_DIRECTION_REPLY);

	// Heartbeat
    // a) ensure display is not released by remote OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
	const auto heartbeat_msg = msp_dp_osd::construct_OSD_heartbeat();
	this->Send(MSP_CMD_DISPLAYPORT, &heartbeat_msg, MSP_DIRECTION_REPLY);

	// Clear screen
	if (clear){
		const auto clear_osd_msg = msp_dp_osd::construct_OSD_clear();
		this->Send(MSP_CMD_DISPLAYPORT, &clear_osd_msg, MSP_DIRECTION_REPLY);
	}

	// FC VARIANT
	const auto fc_variant_msg = msp_osd::construct_FC_VARIANT();
	this->Send(MSP_FC_VARIANT, &fc_variant_msg, MSP_DIRECTION_REPLY);

	// VEHICLE STATUS / Ready Message / ERROR MESSAGES / FLIGHT MODE
	{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		// Vehicle Status
		const auto status_msg = msp_dp_osd::construct_status(vehicle_status);
		this->Send(MSP_STATUS, &status_msg, MSP_DIRECTION_REPLY);

		// Ready or Not Ready message
		// Do not show when ARMED
		if(vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED
			&& _parameters.ready_row != -1 && _parameters.ready_col != -1)
		{
			const char* ready_msg = "";

			if (vehicle_status.pre_flight_checks_pass) {
				ready_msg = "READY";
			} else {
				ready_msg = "NOT READY";
			}

			uint8_t ready_output[sizeof(msp_dp_cmd_t) + sizeof(ready_msg)+1]{0};	// size of output buffer is size of OSD display port command struct and the buffer you want shown on OSD
			msp_dp_osd::construct_OSD_write(_parameters.ready_col, _parameters.ready_row, false, ready_msg, ready_output, sizeof(ready_output));
			this->Send(MSP_CMD_DISPLAYPORT, &ready_output, MSP_DIRECTION_REPLY);
		}

		if (_parameters.status_col != -1 && _parameters.status_row > 0){
			log_message_s log_message{};
			_log_message_sub.copy(&log_message);

			const auto warning_msg = construct_warning_message(
								log_message,
								_param_osd_log_level.get(),
								_warning);
			uint8_t warning_msg_output[sizeof(msp_dp_cmd_t) + sizeof(warning_msg.craft_name)+1]{0};

			msp_dp_osd::construct_OSD_write(_parameters.status_col, _parameters.status_row - 1, false, warning_msg.craft_name, warning_msg_output, sizeof(warning_msg_output));
			this->Send(MSP_CMD_DISPLAYPORT, &warning_msg_output, MSP_DIRECTION_REPLY);
		}

		// STATUS MESSAGE -> BOTTOM-MIDDLE MIDDLE (PX4 error messages)
		// FLIGHT MODE | ARMING | HEADING ....  WILL PRINT PX4 ERROR MESSAGES FOR 30 SECONDS THEN RESET to above format
		if (_parameters.status_col != -1 && _parameters.status_row != -1){
			vehicle_attitude_s vehicle_attitude{};
			_vehicle_attitude_sub.copy(&vehicle_attitude);
			log_message_s log_message{};
			_log_message_sub.copy(&log_message);

			esc_status_s esc_status{};
			_esc_status_sub.copy(&esc_status);

			parameter_selector_s parameter_selector{};
			_parameter_selector_sub.copy(&parameter_selector);

			const auto display_msg = construct_display_message(
								vehicle_status,
								vehicle_attitude,
								log_message,
								esc_status,
								parameter_selector,
								_param_osd_log_level.get(),
								_display);
			uint8_t display_msg_output[sizeof(msp_dp_cmd_t) + sizeof(display_msg.craft_name)+1]{0};

			msp_dp_osd::construct_OSD_write(_parameters.status_col, _parameters.status_row, false, display_msg.craft_name, display_msg_output, sizeof(display_msg_output));	// display_msg max size (w/o warning) is 15
			this->Send(MSP_CMD_DISPLAYPORT, &display_msg_output, MSP_DIRECTION_REPLY);
		}

		// Flight Mode -> BOTTOM-MIDDLE BOTTOM
		if (_parameters.flight_mode_col != -1 && _parameters.flight_mode_row != -1){
			const auto flight_mode = msp_dp_osd::construct_flight_mode(vehicle_status);
			uint8_t flight_mode_output[sizeof(msp_dp_cmd_t) + sizeof(flight_mode)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.flight_mode_col, _parameters.flight_mode_row, false, flight_mode, flight_mode_output, sizeof(flight_mode_output));
			this->Send(MSP_CMD_DISPLAYPORT, &flight_mode_output, MSP_DIRECTION_REPLY);
		}
	}

	// RC CHANNELS / RSSI
	{
		input_rc_s input_rc{};
		_input_rc_sub.copy(&input_rc);

		// Send RC channel values
		const auto rc_msg = msp_dp_osd::construct_RC(input_rc, this->_sticks);
		this->Send(MSP_RC, &rc_msg, MSP_DIRECTION_REPLY);

		// Send RSSI
		if (_parameters.rssi_col != -1 && _parameters.rssi_row != -1){
			char rssi[5];
			snprintf(rssi, sizeof(rssi), "%c%d", SYM_RSSI, (int)input_rc.rssi_dbm);
			uint8_t rssi_output[sizeof(msp_dp_cmd_t) + sizeof(rssi)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.rssi_col, _parameters.rssi_row, false, rssi, rssi_output, sizeof(rssi_output));
			this->Send(MSP_CMD_DISPLAYPORT, &rssi_output, MSP_DIRECTION_REPLY);
		}
	}

	// BATTERY / CURRENT DRAW
	{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		// Full battery voltage
		if(_parameters.battery_col != -1 && _parameters.battery_row != -1){
			char batt[8];
			// uint8_t battery_symbol = SYM_BATT_FULL;	// Could probably make this change based on battery level...
			snprintf(batt, sizeof(batt), "%c%.2f%c", SYM_BATT_FULL, static_cast<double>(battery_status.voltage_v), SYM_VOLT);
			uint8_t battery_output[sizeof(msp_dp_cmd_t) + sizeof(batt)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.battery_col, _parameters.battery_row, false, batt, battery_output, sizeof(battery_output));
			this->Send(MSP_CMD_DISPLAYPORT, &battery_output, MSP_DIRECTION_REPLY);
		}

		// Per cell battery voltage
		if(_parameters.cell_battery_col != -1 && _parameters.cell_battery_row != -1){
			char batt_cell[7];
			// uint8_t battery_symbol = SYM_BATT_FULL;	// Could probably make this change based on battery level...
			snprintf(batt_cell, sizeof(batt_cell), "%c%.2f%c", SYM_BATT_FULL, static_cast<double>(battery_status.voltage_v/battery_status.cell_count), SYM_VOLT);
			uint8_t batt_cell_output[sizeof(msp_dp_cmd_t) + sizeof(batt_cell)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.cell_battery_col, _parameters.cell_battery_row, false, batt_cell, batt_cell_output, sizeof(batt_cell_output));
			this->Send(MSP_CMD_DISPLAYPORT, &batt_cell_output, MSP_DIRECTION_REPLY);
		}

		// Current draw
		if(_parameters.current_draw_col != -1 && _parameters.current_draw_row != -1){
			char current_draw[8];
			snprintf(current_draw, sizeof(current_draw), "%.3f%c", static_cast<double>(battery_status.current_filtered_a), SYM_AMP);
			uint8_t current_draw_output[sizeof(msp_dp_cmd_t) + sizeof(current_draw)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.current_draw_col, _parameters.current_draw_row, false, current_draw, current_draw_output, sizeof(current_draw_output));
			this->Send(MSP_CMD_DISPLAYPORT, &current_draw_output, MSP_DIRECTION_REPLY);
		}
	}

	// MSP_BATTERY_STATE
	// This is required to fill the DJI Battery Indicator in the right hand corner
	{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		const auto msg = construct_BATTERY_STATE(battery_status);
		this->Send(MSP_BATTERY_STATE, &msg);
	}

	// GPS LAT/LONG
	{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		// GPS Longitude
		if(_parameters.longitude_col != -1 && _parameters.longitude_row != -1){
			char longitude[11];
			snprintf(longitude, sizeof(longitude), "%c%.7f", SYM_LON, static_cast<double>(vehicle_gps_position.lon)*1e-7);
			uint8_t longitude_output[sizeof(msp_dp_cmd_t) + sizeof(longitude)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.longitude_col, _parameters.longitude_row, false, longitude, longitude_output, sizeof(longitude_output));
		}

		// GPS Latitude
		if(_parameters.latitude_col != -1 && _parameters.latitude_row != -1){
			char latitude[11];
			snprintf(latitude, sizeof(latitude), "%c%.7f", SYM_LAT, static_cast<double>(vehicle_gps_position.lat)*1e-7);
			uint8_t latitude_output[sizeof(msp_dp_cmd_t) + sizeof(latitude)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.latitude_col, _parameters.latitude_row, false, latitude, latitude_output, sizeof(latitude_output));
			this->Send(MSP_CMD_DISPLAYPORT, &latitude_output, MSP_DIRECTION_REPLY);
		}
	}

	// DIR/DIST TO HOME/HEADING ANGLE
	{
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		estimator_status_s estimator_status{};
		_estimator_status_sub.copy(&estimator_status);
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);
		int16_t distance_to_home{0};
		int16_t bearing_to_home{SYM_ARROW_NORTH};

		// Calculate distance and direction to home
		if (home_position.valid_hpos && home_position.valid_lpos && estimator_status.solution_status_flags & (1 << 4)) {
			float bearing_to_home_f = math::degrees(get_bearing_to_next_waypoint(vehicle_global_position.lat,
								vehicle_global_position.lon,
								home_position.lat, home_position.lon));

			if (bearing_to_home < 0) {
				bearing_to_home += 360.0f;
			}

			float distance_to_home_f = get_distance_to_next_waypoint(vehicle_global_position.lat,
						vehicle_global_position.lon,
						home_position.lat, home_position.lon);

			distance_to_home = (int16_t)distance_to_home_f; // meters
			bearing_to_home = msp_dp_osd::get_symbol_from_bearing(bearing_to_home_f);
		}

		// Direction/Distance to home
		if(_parameters.to_home_col != -1 && _parameters.to_home_row != -1){
			char to_home[8];
			snprintf(to_home, sizeof(to_home), "%c%c%i%c", bearing_to_home, SYM_HOMEFLAG, distance_to_home, SYM_M);
			uint8_t to_home_output[sizeof(msp_dp_cmd_t) + sizeof(to_home)+1]{0};
			msp_dp_osd::construct_OSD_write(_parameters.to_home_col, _parameters.to_home_row, false, to_home, to_home_output, sizeof(to_home_output));
			this->Send(MSP_CMD_DISPLAYPORT, &to_home_output, MSP_DIRECTION_REPLY);
		}

		// Heading Angle
		if(_param_osd_hdg_col.get() != -1 && _param_osd_hdg_row.get() != -1){
			char heading[5];
			snprintf(heading, sizeof(heading), "%c%i", bearing_to_home, (int16_t)vehicle_gps_position.heading);
			uint8_t heading_output[sizeof(msp_dp_cmd_t) + sizeof(heading)+1]{0};
			msp_dp_osd::construct_OSD_write(_param_osd_hdg_col.get(), _param_osd_hdg_row.get(), false, heading, heading_output, sizeof(heading_output));
			this->Send(MSP_CMD_DISPLAYPORT, &heading_output, MSP_DIRECTION_REPLY);
		}
	}

	// CROSSHAIRS
	if(_param_osd_ch_col.get() != -1 && _param_osd_ch_row.get() != 1){
		char crosshair[4] = {SYM_AH_CENTER_LINE, SYM_AH_CENTER, SYM_AH_CENTER_LINE_RIGHT, '\0'};
		uint8_t crosshair_output[sizeof(msp_dp_cmd_t) + sizeof(crosshair)]{0};
		msp_dp_osd::construct_OSD_write(_param_osd_ch_col.get(), _param_osd_ch_row.get(), false, crosshair, crosshair_output, sizeof(crosshair_output));
		this->Send(MSP_CMD_DISPLAYPORT, &crosshair_output, MSP_DIRECTION_REPLY);
	}

	// Remote OSD input
	if (_remote_enable){
		for (int i = 0; i < MAX_REMOTE_OSD_FIELDS; i++) {
			// Look for initialized OSD strings
			if (_remote_osd[i].row != -1) {
				uint8_t remote_osd_output[sizeof(msp_dp_cmd_t) + strlen(_remote_osd[i].string)]{0};
				msp_dp_osd::construct_OSD_write(_remote_osd[i].col, _remote_osd[i].row, false, _remote_osd[i].string, remote_osd_output, sizeof(remote_osd_output));
				this->Send(MSP_CMD_DISPLAYPORT, &remote_osd_output, MSP_DIRECTION_REPLY);
			} else {
				break;
			}
		}
	}

	// DRAW whole screen
	displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
	this->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
}

void MspDPOsd::Send(const unsigned int message_type, const void *payload, mspDirection_e direction)
{
	if (_msp.Send(message_type, payload, direction)) {
		_performance_data.successful_sends++;

	} else {
		_performance_data.unsuccessful_sends++;
	}
}

void MspDPOsd::parameters_update()
{
	int32_t band_t{0};
	int32_t channel_t{0};
	int32_t remote_enable_t{0};

	// update our display rate and dwell time
	_display.set_period(hrt_abstime(_param_osd_scroll_rate.get() * 1000ULL));
	_display.set_dwell(hrt_abstime(_param_osd_dwell_time.get() * 1000ULL));
	_warning.set_period(hrt_abstime(_param_osd_scroll_rate.get() * 1000ULL));
	_warning.set_dwell(hrt_abstime(_param_osd_dwell_time.get() * 1000ULL));

	// Get DisplayPort based positions
	param_get(param_find("OSD_RSSI_COL"),  	&_parameters.rssi_col);
	param_get(param_find("OSD_RSSI_ROW"),  	&_parameters.rssi_row);

	param_get(param_find("OSD_CURR_COL"),  	&_parameters.current_draw_col);
	param_get(param_find("OSD_CURR_ROW"),  	&_parameters.current_draw_row);

	param_get(param_find("OSD_BATT_COL"),  	&_parameters.battery_col);
	param_get(param_find("OSD_BATT_ROW"),  	&_parameters.battery_row);
	param_get(param_find("OSD_CBATT_COL"), 	&_parameters.cell_battery_col);
	param_get(param_find("OSD_CBATT_ROW"), 	&_parameters.cell_battery_row);

	param_get(param_find("OSD_RDY_COL"),  	&_parameters.ready_col);
	param_get(param_find("OSD_RDY_ROW"),  	&_parameters.ready_row);
	param_get(param_find("OSD_STATUS_COL"), &_parameters.status_col);
	param_get(param_find("OSD_STATUS_ROW"), &_parameters.status_row);
	param_get(param_find("OSD_FM_COL"),  	&_parameters.flight_mode_col);
	param_get(param_find("OSD_FM_ROW"),  	&_parameters.flight_mode_row);

	param_get(param_find("OSD_LAT_COL"),  	&_parameters.latitude_col);
	param_get(param_find("OSD_LAT_ROW"),  	&_parameters.latitude_row);
	param_get(param_find("OSD_LONG_COL"), 	&_parameters.longitude_col);
	param_get(param_find("OSD_LONG_ROW"), 	&_parameters.longitude_row);

	param_get(param_find("OSD_HOME_COL"), 	&_parameters.to_home_col);
	param_get(param_find("OSD_HOME_ROW"), 	&_parameters.to_home_row);

	// param_get(param_find("OSD_CH_COL"), 	&_parameters.crosshair_col);
	// param_get(param_find("OSD_CH_ROW"), 	&_parameters.crosshair_row);

	// param_get(param_find("OSD_HDG_COL"), 	&_parameters.heading_col);
	// param_get(param_find("OSD_HDG_ROW"), 	&_parameters.heading_row);

	// param_get(param_find("OSD_CH_COL"), 	&_param_osd_ch_col.get());
	// param_get(param_find("OSD_CH_ROW"), 	&_param_osd_ch_row.get());

	// param_get(param_find("OSD_HDG_COL"), 	&_param_osd_hdg_col.get());
	// param_get(param_find("OSD_HDG_ROW"), 	&_param_osd_hdg_row.get());

	param_get(param_find("OSD_BATT_LOW_V"), 	&_parameters.osd_batt_low_v);
	param_get(param_find("OSD_DISPLAY_OPTS"), 	&_parameters.osd_display_opts);
	param_get(param_find("OSD_HDG_RST_CHAN"), 	&_parameters.osd_hdg_rst_chan);

	param_get(param_find("OSD_CHANNEL"), 	&channel_t);
	param_get(param_find("OSD_BAND"),    	&band_t);
	param_get(param_find("OSD_REMOTE"),  	&remote_enable_t);

	param_get(param_find("RC_MAP_THROTTLE"), &_sticks.throttle);
	param_get(param_find("RC_MAP_ROLL"),     &_sticks.roll);
	param_get(param_find("RC_MAP_PITCH"),    &_sticks.pitch);
	param_get(param_find("RC_MAP_YAW"),    	 &_sticks.yaw);

	this->_band = (uint8_t)band_t;
	this->_channel = (uint8_t)channel_t;
	this->_remote_enable = (bool) remote_enable_t;
}

int MspDPOsd::task_spawn(int argc, char *argv[])
{
	// initialize device
	const char *device = nullptr;
	bool error_flag = false;

	// loop through input arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device) {
		PX4_ERR("Missing device");
		return PX4_ERROR;
	}

	MspDPOsd *instance = new MspDPOsd(device);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MspDPOsd::print_status()
{
	char current_band;
	uint8_t band = get_instance()->_band;
	if (band == 5){
		current_band = 'R';
	} else if (band == 4){
		current_band = 'F';
	} else if (band == 3){
		current_band = 'E';
	} else {
		current_band = '?';
	}
	PX4_INFO("Running on %s", _device);
	PX4_INFO("\tinitialized: %d", _is_initialized);
	PX4_INFO("\tinitialization issues: %d", _performance_data.initialization_problems);
	PX4_INFO("\tscroll rate: %d", static_cast<int>(_param_osd_scroll_rate.get()));
	PX4_INFO("\tsuccessful sends: %lu", _performance_data.successful_sends);
	PX4_INFO("\tunsuccessful sends: %lu", _performance_data.unsuccessful_sends);
	PX4_INFO("\tBand: %c", current_band);
	PX4_INFO("\tChannel: %u", get_instance()->_channel);


	// print current display string
	char msg[FULL_MSG_BUFFER];
	_display.get(msg, hrt_absolute_time());
	PX4_INFO("Current message: \n\t%s", msg);

	// print current warning string
	_warning.get(msg, hrt_absolute_time());
	PX4_INFO("Current warning: \n\t%s", msg);

	return 0;
}

// Ex: "msp_dp_osd -s TEST -h 5 -v 5 write_string" -> Write "TEST" at row 5/column 5
// NOTE: To disable screen clearing, use "msp_dp_osd toggle_clear" command 
int MspDPOsd::custom_command(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	int row{0};
	int col{0};
	size_t msg_len{0};
	char cmd_band[1]{'R'};	// Default to Raceband
	uint8_t cmd_channel{0};	// Channel 1 (R1)
	uint8_t cmd_power{1};	// Default 25mW
	int cmd_fontType{0};
	int cmd_resolution{0};
	char cmd_string[get_instance()->column_max[get_instance()->resolution]]{0};
	const char* resolutions[4] = {"SD_3016", "HD_5018", "HD_3016", "HD_5320"};
	const char *myoptarg = nullptr;
	const char *verb = argv[argc - 1];
	PX4_INFO("Executing the following command: %s", verb);

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;
	}

	while ((ch = px4_getopt(argc, argv, "v:h:f:r:s:b:c:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'v':	// row 
			row = atoi(myoptarg); // 0 min, 17 max, HD_5018 TRUNCATES MSG
			if (row < 0) row = 0;
			if (row > get_instance()->row_max[get_instance()->resolution]) row = 17;
			PX4_INFO("Got Row: %i", row);
			break;

		case 'h':	// column
			col = atoi(myoptarg);	// 0 min, 49 max, HD_5018 TRUNCATES MSG
			if (col < 0) col = 0;
			if (col > get_instance()->column_max[get_instance()->resolution]) col = 49;
			PX4_INFO("Got Col: %i", col);
			break;

		case 'f':
			cmd_fontType = atoi(myoptarg);
			if (cmd_fontType < 0 || cmd_fontType > 3){
				print_usage("Invalid font type, must be 0-3.");
				return 0;
			}
			PX4_INFO("Got fontType: %i", cmd_fontType);
			break;

		case 'r':
			cmd_resolution = atoi(myoptarg);
			if (cmd_resolution < 0 || cmd_resolution > 3){
				print_usage("Invalid resolution, must be 0-3.");
				return 0;
			}
			PX4_INFO("Got Resolution: %s", resolutions[cmd_resolution]);
			break;

		case 's':
			msg_len = strlen(myoptarg);
			if (msg_len > MSP_OSD_MAX_STRING_LENGTH){
				PX4_WARN("String length (%lu) too long, max string length: %i. Message may be truncated.", msg_len, MSP_OSD_MAX_STRING_LENGTH);
				msg_len = MSP_OSD_MAX_STRING_LENGTH;
			}
			PX4_INFO("Got string: %s, Length: %lu", myoptarg, msg_len);
			strncpy(cmd_string, myoptarg, msg_len + 1);
			break;

		case 'b':
			strncpy(cmd_band, myoptarg, strlen(myoptarg) + 1);
			PX4_INFO("Got Band: %s", cmd_band);
			break;

		case 'c':
			cmd_channel = atoi(myoptarg);
			PX4_INFO("Got channel: %u", cmd_channel);
			break;

		case 'p':
			cmd_power = atoi(myoptarg);
			if (cmd_power < 1 || cmd_power > 3){
				PX4_ERR("VTX commanded power must be 1-3. Received: %i", cmd_power);
				return 0;
			}
			cmd_power--;
			PX4_INFO("Got Power: %i", cmd_power);
			break;

		default:
			print_usage("Unknown command, parsing flags");
			return 0;
		}
	}

	// Write string for testing placement on canvas
	if(!strcmp(verb,"write")){
		PX4_INFO("");
		PX4_INFO("Sending WRITE CMD");
		char line[2]={SYM_BATT_FULL,'\0'};	// 30 char max
		uint8_t output[sizeof(msp_dp_cmd_t) + sizeof(line)]{0};
		msp_dp_osd::construct_OSD_write(col, row, false, line, output, sizeof(output));
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &output, MSP_DIRECTION_REPLY);
		PX4_INFO("");
		PX4_INFO("Sending DRAW CMD");
		displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Write string
	if(!strcmp(verb,"write_string")){
		PX4_INFO("");
		PX4_INFO("Sending WRITE STRING CMD");
		if (cmd_string[MSP_OSD_MAX_STRING_LENGTH-1] != '\0') cmd_string[MSP_OSD_MAX_STRING_LENGTH-1] = '\0';

		// Convert string to uppercase, otherwise it will try to print symbols instead
		for(size_t i=0; i<strlen(cmd_string);++i){
			cmd_string[i] = (cmd_string[i] >= 'a' && cmd_string[i] <= 'z') ? cmd_string[i] - 'a' + 'A' : cmd_string[i];
		}

		const char* const_cmd_string = cmd_string;
		uint8_t output[sizeof(msp_dp_cmd_t) + strlen(const_cmd_string)+1]{0};
		PX4_INFO("Output String: %s\tSize of output: %lu", const_cmd_string, sizeof(output));
		msp_dp_osd::construct_OSD_write(col, row, false, const_cmd_string, output, sizeof(output));
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &output, MSP_DIRECTION_REPLY);
		PX4_INFO("");
		PX4_INFO("Sending DRAW CMD");
		displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Turn auto clear OSD on/off
	if(!strcmp(verb,"toggle_clear")){
		PX4_INFO("");
		PX4_INFO("Sending TOGGLING CLEAR");
		clear = !clear;
		return 0;
	}

	// Release OSD
	if(!strcmp(verb,"release")){
		PX4_INFO("");
		PX4_INFO("Sending RELEASE CMD");
		const auto msg = msp_dp_osd::construct_OSD_release();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Config OSD resolution, font
	if(!strcmp(verb,"osd_config")){
		PX4_INFO("");
		PX4_INFO("Sending OSD CONFIG CMD");
		const auto msg = msp_dp_osd::construct_OSD_config((resolutionType_e)cmd_resolution, cmd_fontType);
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &msg, MSP_DIRECTION_REPLY);
		get_instance()->resolution = (resolutionType_e)cmd_resolution;
		get_instance()->fontType = cmd_fontType;
		return 0;
	}

	// Config OSD Canvas size
	if(!strcmp(verb,"osd_canvas")){
		PX4_INFO("");
		PX4_INFO("Sending OSD CANVAS CMD");
		const auto msg = msp_dp_osd::construct_OSD_canvas(get_instance()->row_max[get_instance()->resolution], get_instance()->column_max[get_instance()->resolution]);
		get_instance()->Send(MSP_SET_OSD_CANVAS, &msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Config VTX Band and Channel settings
	if(!strcmp(verb,"vtx")){
		/* Fields:
		protocol: 5 -> MSP
		band:	  5 -> RC Band R (E,F,R)
		channel:  1 -> Channel (Ex: R1, R2, F1, etc)
		power:	  1 -> 0 (0mw), 1 (25mW), 2 (200mW)
		pit:	  0 -> Pit mode off
		freq:	  0x161A -> 5658 MHz
		*/

		// Convert string to uppercase
		for(size_t i=0; i<strlen(cmd_band);++i){
			cmd_band[i] = (cmd_band[i] >= 'a' && cmd_band[i] <= 'z') ? cmd_band[i] - 'a' + 'A' : cmd_band[i];
		}

		uint8_t protocol{5};
		uint8_t band{get_instance()->_band};
		uint8_t channel{get_instance()->_channel};
		uint8_t power{get_instance()->_power};
		uint8_t pit{0};
		uint16_t freq{get_instance()->_frequency};

		if(*cmd_band == 'R'){
			band = 5;
		}else if (*cmd_band == 'F'){
			band = 4;
		} else if (*cmd_band == 'E'){
			band = 3;
		} else {
			band = 5;
			*cmd_band = 'R';
		}

		if(cmd_channel != 0){
			channel = cmd_channel;
		}

		PX4_INFO("");
		PX4_INFO("Sending VTX BAND/CHANNEL CONFIG");
		PX4_INFO("\tBand:    %u", band);
		PX4_INFO("\tChannel: %u", channel);
		PX4_INFO("\tPower:   %u", power);
		PX4_INFO("\tFreq:    %u", freq);
		const msp_dp_vtx_config_t vtx_config_msg = {
			protocol,
			band,
			channel,
			power,
			pit,
			freq
		};
		get_instance()->Send(MSP_VTX_CONFIG, &vtx_config_msg, MSP_DIRECTION_REPLY);
		get_instance()->_band = band;
		get_instance()->_channel = channel;
		get_instance()->_frequency = freq;
		return 0;
	}

	// Config VTX Power settings
	if(!strcmp(verb,"power")){
		uint8_t protocol{5};
		uint8_t band{get_instance()->_band};
		uint8_t channel{get_instance()->_channel};
		uint8_t power{(uint8_t)cmd_power};
		uint8_t pit{0};
		uint16_t freq{get_instance()->_frequency};

		PX4_INFO("");
		PX4_INFO("Sending VTX POWER CONFIG");
		PX4_INFO("\tBand:    %u", band);
		PX4_INFO("\tChannel: %u", channel);
		PX4_INFO("\tPower:   %u", power);
		PX4_INFO("\tFreq:    %u", freq);
		const msp_dp_vtx_config_t vtx_config_msg = {
			protocol,
			band,
			channel,
			power,
			pit,
			freq
		};
		get_instance()->Send(MSP_VTX_CONFIG, &vtx_config_msg, MSP_DIRECTION_REPLY);
		get_instance()->_power = power;
		return 0;
	}

	PX4_WARN("Unknown command: %s", verb);
	return 0;
}

int MspDPOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MSP DisplayPort telemetry streamer

### Implementation
Converts uORB messages to MSP telemetry packets

### Examples
CLI usage example:
$ msp_dp_osd

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("msp_dp_osd", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyHS1", "/dev/ttyHS1", "UART port", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("clear", "DisplayPort command: Clear the OSD.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("release", "DisplayPort command: Clears the display and allows local rendering on the display device based on telemetry information etc.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write_string", "DisplayPort command: Write string to OSD at given location");
	PRINT_MODULE_USAGE_PARAM_INT('h', 0, 0, 19, "Line/Row to write the string on", false);
	PRINT_MODULE_USAGE_PARAM_INT('v', 0, 0, 52, "Column to write the string on", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("osd_config", "DisplayPort command: Set OSD font type and resolution.");
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 3, "Resolution to set OSD to.", false);
	PRINT_MODULE_USAGE_PARAM_INT('f', 0, 0, 3, "Font type to use for OSD.", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("vtx", "Set VTX channel and/or band.");
	PRINT_MODULE_USAGE_PARAM_INT('c', 1, 1, 7, "Channel to use for VTX.", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 5, 3, 5, "Band to use for VTX.", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("power", "Set VTX power.");
	PRINT_MODULE_USAGE_PARAM_INT('p', 1, 0, 2, "Set VTX power. Recommended to set this through OSD menu.", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int msp_dp_osd_main(int argc, char *argv[])
{
	return MspDPOsd::main(argc, argv);
}

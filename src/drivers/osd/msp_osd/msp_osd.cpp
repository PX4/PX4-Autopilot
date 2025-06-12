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

#include "msp_osd.hpp"

#include "msp_defines.h"

#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <ctype.h>

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

#include "MspV1.hpp"

//OSD elements positions
//in betaflight configurator set OSD elements to your desired positions and in CLI type "set osd" to retreieve the numbers.
//234 -> not visible. Horizontally 2048-2074(spacing 1), vertically 2048-2528(spacing 32). 26 characters X 15 lines

// Currently working elements positions (hardcoded)

/* center col

Speed Power Alt
Rssi cell_voltage mah
craft name

*/

// Left
const uint16_t osd_gps_lat_pos = 2048;
const uint16_t osd_gps_lon_pos = 2080;
const uint16_t osd_gps_sats_pos = 2112;

// Center
// Top
const uint16_t osd_disarmed_pos = 2125;
const uint16_t osd_home_dir_pos = 2093;
const uint16_t osd_home_dist_pos = 2095;

// Bottom row 1
const uint16_t osd_gps_speed_pos = 2413;
const uint16_t osd_power_pos = 2415;
const uint16_t osd_altitude_pos = 2416;

// Bottom Row 2
const uint16_t osd_rssi_value_pos = 2445;
const uint16_t osd_avg_cell_voltage_pos = 2446;
const uint16_t osd_mah_drawn_pos = 2449;

// Bottom Row 3
const uint16_t osd_craft_name_pos = 2480;
const uint16_t osd_crosshairs_pos = 2319;

// Right
const uint16_t osd_main_batt_voltage_pos = 2073;
const uint16_t osd_current_draw_pos = 2103;


const uint16_t osd_numerical_vario_pos = LOCATION_HIDDEN;

#define OSD_GRID_COL_MAX (59) // From betaflight-configurator OSD tab
#define OSD_GRID_ROW_MAX (21) // From betaflight-configurator OSD tab

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


MspOsd::MspOsd(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
	_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);

	// back up device name for connection later
	strcpy(_device, device);

	// _is_initialized = true;
	PX4_INFO("MSP OSD running on %s", _device);
}

MspOsd::~MspOsd()
{
}

bool MspOsd::init()
{
	ScheduleOnInterval(100_ms);

	return true;
}

void MspOsd::SendConfig()
{
	msp_osd_config_t msp_osd_config;

	msp_osd_config.units = 0;
	msp_osd_config.osd_item_count = 56;
	msp_osd_config.osd_stat_count = 24;
	msp_osd_config.osd_timer_count = 2;
	msp_osd_config.osd_warning_count = 16;              // 16
	msp_osd_config.osd_profile_count = 1;              // 1
	msp_osd_config.osdprofileindex = 1;                // 1
	msp_osd_config.overlay_radio_mode = 0;             //  0

	// display conditional elements
	msp_osd_config.osd_craft_name_pos = enabled(SymbolIndex::CRAFT_NAME) ? osd_craft_name_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_disarmed_pos = enabled(SymbolIndex::DISARMED) ? osd_disarmed_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_lat_pos = enabled(SymbolIndex::GPS_LAT) ? osd_gps_lat_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_lon_pos = enabled(SymbolIndex::GPS_LON) ? osd_gps_lon_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_sats_pos = enabled(SymbolIndex::GPS_SATS) ? osd_gps_sats_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_speed_pos = enabled(SymbolIndex::GPS_SPEED) ? osd_gps_speed_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_home_dist_pos = enabled(SymbolIndex::HOME_DIST) ? osd_home_dist_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_home_dir_pos = enabled(SymbolIndex::HOME_DIR) ? osd_home_dir_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_main_batt_voltage_pos = enabled(SymbolIndex::MAIN_BATT_VOLTAGE) ? osd_main_batt_voltage_pos :
			LOCATION_HIDDEN;
	msp_osd_config.osd_current_draw_pos = enabled(SymbolIndex::CURRENT_DRAW) ? osd_current_draw_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_mah_drawn_pos = enabled(SymbolIndex::MAH_DRAWN) ? osd_mah_drawn_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_rssi_value_pos = enabled(SymbolIndex::RSSI_VALUE) ? osd_rssi_value_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_altitude_pos = enabled(SymbolIndex::ALTITUDE) ? osd_altitude_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_numerical_vario_pos = enabled(SymbolIndex::NUMERICAL_VARIO) ? osd_numerical_vario_pos :
			LOCATION_HIDDEN;

	msp_osd_config.osd_power_pos = enabled(SymbolIndex::POWER) ? osd_power_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_avg_cell_voltage_pos = enabled(SymbolIndex::AVG_CELL_VOLTAGE) ? osd_avg_cell_voltage_pos :
			LOCATION_HIDDEN;

	// the location of our crosshairs can change
	msp_osd_config.osd_crosshairs_pos = LOCATION_HIDDEN;

	if (enabled(SymbolIndex::CROSSHAIRS)) {
		msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos - 32 * _param_osd_ch_height.get();
	}

	// possibly available, but not currently used
	msp_osd_config.osd_flymode_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_tmp_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_horizon_sidebars_pos = 		LOCATION_HIDDEN;

	// Not implemented or not available
	msp_osd_config.osd_artificial_horizon_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_1_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_2_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_throttle_pos_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_vtx_channel_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_yaw_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pidrate_profile_pos =		LOCATION_HIDDEN;
	msp_osd_config.osd_warnings_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_debug_pos = 				LOCATION_HIDDEN;
	msp_osd_config.osd_main_batt_usage_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_numerical_heading_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_compass_bar_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_remaining_time_estimate_pos = 	LOCATION_HIDDEN;
	msp_osd_config.osd_rtc_datetime_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_adjustment_range_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_core_temperature_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_anti_gravity_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_g_force_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_motor_diag_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_log_status_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flip_arrow_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_link_quality_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flight_dist_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_left_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_right_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_display_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_freq_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rate_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_pid_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_profile_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rssi_dbm_value_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_rc_channels_pos = 			LOCATION_HIDDEN;

	_msp.Send(MSP_OSD_CONFIG, &msp_osd_config);
}

// extract it to MSPOSD_BF_Run() and MSPOSD_DJIFPV_Run() for compatibility?
void MspOsd::Run()
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
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		parameters_update();
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

		_msp = MspV1(_msp_fd);

		_is_initialized = true;
	}

	if (change_channel) {
		msp_get_vtx_config_t vtx_set_config{0};
		vtx_set_config.low_power_disarm = vtx_config.low_power_disarm;
		vtx_set_config.pit_mode = vtx_config.pit_mode;
		vtx_set_config.vtx_type = VTXDEV_MSP;
		vtx_set_config.band = vtx_config.user_band;
		vtx_set_config.channel = vtx_config.user_channel;
		this->Send(MSP_GET_VTX_CONFIG, &vtx_set_config, sizeof(msp_get_vtx_config_t));
		change_channel = false;
	}

	this->Receive();

	if (!has_vtx_config) {
		this->Send(MSP_GET_VTX_CONFIG, nullptr, 0);
	}

	// avoid premature pessimization; if skip processing if we're effectively disabled
	if (_param_osd_symbols.get() == 0) {
		return;
	}

	uint8_t subcmd = MSP_DP_HEARTBEAT;
	this->Send(MSP_CMD_DISPLAYPORT, &subcmd, 1);

	subcmd = MSP_DP_CLEAR_SCREEN;
	this->Send(MSP_CMD_DISPLAYPORT, &subcmd, 1);

	// update display message
	{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		log_message_s log_message{};
		_log_message_sub.copy(&log_message);
		// TODO re-wirte this function?
		const auto display_message = msp_osd::construct_display_message(
						     vehicle_status,
						     vehicle_attitude,
						     log_message,
						     _param_osd_log_level.get(),
						     _display);

		char msg[sizeof(msp_name_t) + 5] = {0};
		int index = 0;
		msg[index++] = MSP_DP_WRITE_STRING;
		msg[index++] = 0x02; // row position
		msg[index++] = 0x14; // colum position
		msg[index++] = 0;		// Icon attr
		msg[index++] = 0x03; // Icon index >
		memcpy(&msg[index++], &display_message, sizeof(msp_name_t));
		this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msg));
	}

	// MSP_FC_VARIANT
	{
		const auto msg = msp_osd::construct_FC_VARIANT();
		this->Send(MSP_FC_VARIANT, &msg, sizeof(msg));
	}

	// MSP_ANALOG
	{
		if (enabled(SymbolIndex::RSSI_VALUE)) {
			input_rc_s input_rc{};
			_input_rc_sub.copy(&input_rc);
			const auto msg = msp_osd::construct_rendor_RSSI(input_rc);
			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_rssi_t));
		}
	}

	// MSP_BATTERY_STATE
	{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		const auto msg_original = msp_osd::construct_BATTERY_STATE(battery_status);
		this->Send(MSP_BATTERY_STATE, &msg_original);

		const auto msg = msp_osd::construct_rendor_BATTERY_STATE(battery_status);
		this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_battery_state_t));

	}

	// MSP_RAW_GPS
	{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		if (enabled(SymbolIndex::GPS_LAT)) {
			const auto msg = msp_osd::construct_rendor_GPS_LAT(vehicle_gps_position);
			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_latitude_t));
		}

		if (enabled(SymbolIndex::GPS_LON)) {
			const auto msg = msp_osd::construct_rendor_GPS_LON(vehicle_gps_position);
			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_longitude_t));
		}

		if (enabled(SymbolIndex::GPS_SATS)) {
			const auto msg = msp_osd::construct_rendor_GPS_NUM(vehicle_gps_position);
			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_satellites_used_t));
		}
	}

	// MSP_COMP_GPS
	{
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);

		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);

		if (enabled(SymbolIndex::HOME_DIST)) {
			const auto msg =  msp_osd::construct_rendor_distanceToHome(home_position, vehicle_global_position);

			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_distanceToHome_t));
		}
	}

	// MSP_ATTITUDE
	{
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		{
			const auto msg = msp_osd::construct_rendor_PITCH(vehicle_attitude);
			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_pitch_t));
		}
		{
			const auto msg = msp_osd::construct_rendor_ROLL(vehicle_attitude);
			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_roll_t));
		}
	}


	// MSP_ALTITUDE
	{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);

		if (enabled(SymbolIndex::ALTITUDE)) {
			const auto msg = msp_osd::construct_Rendor_ALTITUDE(vehicle_gps_position, vehicle_local_position);

			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_altitude_t));
		}
	}

	// MSP_MOTOR_TELEMETRY
	{

	}

	// MSP_RC
	{
		if (_param_osd_rc_stick.get() == 1) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
				input_rc_s input_rc{};
				_input_rc_sub.copy(&input_rc);
				const auto msg = msp_osd::construct_MSP_RC(input_rc);
				this->Send(MSP_RC, &msg, sizeof(msp_rc_t));
			}
		}

	}

	// MSP_STATUS
	{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		const auto msg = msp_osd::construct_MSP_STATUS(vehicle_status);
		this->Send(MSP_STATUS, &msg, sizeof(msp_status_t));
	}

	subcmd = MSP_DP_DRAW_SCREEN;
	this->Send(MSP_CMD_DISPLAYPORT, &subcmd, 1);
}

void MspOsd::Send(const unsigned int message_type, const void *payload)
{
	if (_msp.Send(message_type, payload)) {
		_performance_data.successful_sends++;

	} else {
		_performance_data.unsuccessful_sends++;
	}
}
void MspOsd::Send(const unsigned int message_type, const void *payload, int32_t payload_size)
{
	if (_msp.Send(message_type, payload, payload_size)) {
		_performance_data.successful_sends++;

	} else {
		_performance_data.unsuccessful_sends++;
	}
}

void MspOsd::Receive()
{
	uint8_t packet[255];
	uint8_t message_id;
	int ret;

	while ((ret = _msp.Receive(packet, &message_id)) != -EWOULDBLOCK) {
		if (ret >= 0) {
			switch (message_id) {

			case MSP_SET_VTX_CONFIG: {
					if (ret == 0xF) {
						memcpy((void *)&vtx_config, packet, sizeof(vtx_config));
						has_vtx_config = true;
					}

					break;
				}

			case MSP_SET_VTXTABLE_BAND: {
					msp_set_vtxtable_band_t *band_info = (msp_set_vtxtable_band_t *)&packet[0];

					// Only supported fixed name lenght and < 8 channels for now
					if (band_info->band <= BAND_COUNT && band_info->band_name_length == 8 && band_info->channel_count <= 8) {
						memcpy((void *)&vtx_bands[band_info->band - 1], packet, sizeof(msp_set_vtxtable_band_t));

						if (has_vtx_config && band_info->band == vtx_config.band_count) {
							has_vtx_bands = true;
						}
					}

					break;
				}

			case MSP_SET_VTXTABLE_POWERLEVEL: {
					if ((packet[0] - 1) < POWER_LEVEL_COUNT) {
						memcpy((void *)&power_levels[packet[0] - 1], packet, sizeof(msp_set_vtxtable_powerlevel_t));
						has_power_config = true;
					}

					break;
				}

			default:
				break;

			}
		}
	}

}

void MspOsd::parameters_update()
{
	// update our display rate and dwell time
	_display.set_period(hrt_abstime(_param_osd_scroll_rate.get() * 1000ULL));
	_display.set_dwell(hrt_abstime(_param_osd_dwell_time.get() * 1000ULL));
}

bool MspOsd::enabled(const SymbolIndex &symbol)
{
	return _param_osd_symbols.get() & (1u << symbol);
}

int MspOsd::task_spawn(int argc, char *argv[])
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

	MspOsd *instance = new MspOsd(device);

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

int MspOsd::print_status()
{
	PX4_INFO("Running on %s", _device);
	PX4_INFO("\tinitialized: %d", _is_initialized);
	PX4_INFO("\tinitialization issues: %d", _performance_data.initialization_problems);
	PX4_INFO("\tscroll rate: %d", static_cast<int>(_param_osd_scroll_rate.get()));
	PX4_INFO("\tsuccessful sends: %lu", _performance_data.successful_sends);
	PX4_INFO("\tunsuccessful sends: %lu", _performance_data.unsuccessful_sends);

	// print current display string
	char msg[FULL_MSG_BUFFER];
	_display.get(msg, hrt_absolute_time());
	PX4_INFO("Current message: \n\t%s", msg);

	if (has_vtx_config) {
		PX4_INFO("=== VTX Configuration ===");

		if (has_vtx_bands) {
			PX4_INFO("Channel: %c%u", vtx_bands[vtx_config.user_band - 1].band_letter, vtx_config.user_channel);

		} else {
			PX4_INFO("Band: %u", vtx_config.user_band);
			PX4_INFO("Channel: %u", vtx_config.user_channel);
		}

		PX4_INFO("Frequency: %u MHz", vtx_config.user_freq);

		if (has_power_config && (vtx_config.power_level - 1) < POWER_LEVEL_COUNT) {
			PX4_INFO("Transmit power: %.*s mW", power_levels[vtx_config.power_level - 1].power_label_length,
				 power_levels[vtx_config.power_level - 1].power_label_name);

		} else {
			PX4_INFO("Power Level: %u/%u", vtx_config.power_level,  vtx_config.power_count);

		}

		PX4_INFO("PIT Mode: %s", vtx_config.pit_mode ? "On" : "Off");

		const char *disarm_modes[] = {
			"Off",
			"Always",
			"Until First Arm"
		};

		if (vtx_config.low_power_disarm < 3) {
			PX4_INFO("Low Power Disarm: %s", disarm_modes[vtx_config.low_power_disarm]);

		} else {
			PX4_INFO("Low Power Disarm: Unknown (%u)", vtx_config.low_power_disarm);
		}

		PX4_INFO("PIT Frequency: %u MHz", vtx_config.pit_freq);

	} else {
		PX4_INFO("No VTX Configuration available, can't do channel switching");
	}

	return 0;
}

int MspOsd::set_channel(char *new_channel)
{
	char band_letter = toupper(new_channel[0]);

	if (!has_vtx_bands) {
		return -2;
	}

	for (int i = 0; i < BAND_COUNT; i++) {
		if (vtx_bands[i].band != 0) {
			if (band_letter == toupper(vtx_bands[i].band_letter)) {
				int channel = atoi(&new_channel[1]);

				if (channel > 0 && channel <= vtx_config.channel_count && vtx_bands[i].frequency[channel - 1] != 0) {
					vtx_config.user_band = vtx_bands[i].band;
					vtx_config.user_channel = channel;
					vtx_config.user_freq = vtx_bands[i].frequency[channel - 1];
					change_channel = true;
					return 0;
				}

			}
		}
	}

	return -1;
}

int MspOsd::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp("channel", argv[0]) == 0) {
		if (argc == 1) {
			PX4_INFO("Please provide a channel");

		} else if (is_running() && _object.load()) {
			MspOsd *object = _object.load();
			int ret = object->set_channel(argv[1]);

			if (ret == -1) {
				PX4_INFO("Channel not found");

			} else if (ret == -2) {
				PX4_INFO("No VTX Channel table available");
			}

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;
}

int MspOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MSP telemetry streamer

### Implementation
Converts uORB messages to MSP telemetry packets

### Examples
CLI usage example:
$ msp_osd

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("msp_osd", "driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("channel", "Change VTX channel");

	return 0;
}

extern "C" __EXPORT int msp_osd_main(int argc, char *argv[])
{
	return MspOsd::main(argc, argv);
}

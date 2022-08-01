/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "msp_osd.hpp"

#include "msp_defines.h"

#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>

#include <lib/geo/geo.h>

#include "MspV1.hpp"

//OSD elements positions
//in betaflight configurator set OSD elements to your desired positions and in CLI type "set osd" to retreieve the numbers.
//234 -> not visible. Horizontally 2048-2074(spacing 1), vertically 2048-2528(spacing 32). 26 characters X 15 lines

// Currently working elements

// Left
const uint16_t osd_gps_lat_pos = 2048;
const uint16_t osd_gps_lon_pos = 2080;
uint16_t osd_gps_sats_pos = 2112;
const uint16_t osd_rssi_value_pos = 2176;
const uint16_t osd_flymode_pos = 234;
const uint16_t osd_esc_tmp_pos = 234;

// Center
const uint16_t osd_home_dir_pos = 2093;
const uint16_t osd_craft_name_pos = 2543;
const uint16_t osd_horizon_sidebars_pos = 234;
const uint16_t osd_disarmed_pos = 2125;

// Right
const uint16_t osd_main_batt_voltage_pos = 2073;
const uint16_t osd_current_draw_pos = 2103;
const uint16_t osd_mah_drawn_pos = 2138;
const uint16_t osd_altitude_pos = 2233;
const uint16_t osd_numerical_vario_pos = 2267;
const uint16_t osd_gps_speed_pos = 2299;
const uint16_t osd_home_dist_pos = 2331;
const uint16_t osd_power_pos = 234;

// Disabled
const uint16_t osd_pitch_angle_pos = 234;
const uint16_t osd_roll_angle_pos = 234;
const uint16_t osd_crosshairs_pos = 234;
const uint16_t osd_avg_cell_voltage_pos = 234;

// Not implemented or not available
const uint16_t osd_throttle_pos_pos = 234;
const uint16_t osd_vtx_channel_pos = 234;
const uint16_t osd_roll_pids_pos = 234;
const uint16_t osd_pitch_pids_pos = 234;
const uint16_t osd_yaw_pids_pos = 234;
const uint16_t osd_pidrate_profile_pos = 234;
const uint16_t osd_warnings_pos = 234;
const uint16_t osd_debug_pos = 234;
const uint16_t osd_artificial_horizon_pos = 234;
const uint16_t osd_item_timer_1_pos = 234;
const uint16_t osd_item_timer_2_pos = 234;
const uint16_t osd_main_batt_usage_pos = 234;
const uint16_t osd_numerical_heading_pos = 234;
const uint16_t osd_compass_bar_pos = 234;
const uint16_t osd_esc_rpm_pos = 234;
const uint16_t osd_remaining_time_estimate_pos = 234;
const uint16_t osd_rtc_datetime_pos = 234;
const uint16_t osd_adjustment_range_pos = 234;
const uint16_t osd_core_temperature_pos = 234;
const uint16_t osd_anti_gravity_pos = 234;
const uint16_t osd_g_force_pos = 234;
const uint16_t osd_motor_diag_pos = 234;
const uint16_t osd_log_status_pos = 234;
const uint16_t osd_flip_arrow_pos = 234;
const uint16_t osd_link_quality_pos = 234;
const uint16_t osd_flight_dist_pos = 234;
const uint16_t osd_stick_overlay_left_pos = 234;
const uint16_t osd_stick_overlay_right_pos = 234;
const uint16_t osd_display_name_pos = 234;
const uint16_t osd_esc_rpm_freq_pos = 234;
const uint16_t osd_rate_profile_name_pos = 234;
const uint16_t osd_pid_profile_name_pos = 234;
const uint16_t osd_profile_name_pos = 234;
const uint16_t osd_rssi_dbm_value_pos = 234;
const uint16_t osd_rc_channels_pos = 234;

MspOsd::MspOsd() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_msp(NULL),
	_display(/*display_chars=*/12, /*update_period_ms*/250)
{
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

	msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
	msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
	msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
	msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
	msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
	msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
	msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
	msp_osd_config.osd_flymode_pos = osd_flymode_pos;
	msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
	msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
	msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
	msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
	msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
	msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
	msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
	msp_osd_config.osd_altitude_pos = osd_altitude_pos;
	msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
	msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
	msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
	msp_osd_config.osd_power_pos = osd_power_pos;
	msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
	msp_osd_config.osd_warnings_pos = osd_warnings_pos;
	msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
	msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
	msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
	msp_osd_config.osd_debug_pos = osd_debug_pos;
	msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
	msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
	msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
	msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
	msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
	msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
	msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
	msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
	msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
	msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
	msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
	msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
	msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
	msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
	msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
	msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
	msp_osd_config.osd_g_force_pos = osd_g_force_pos;
	msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
	msp_osd_config.osd_log_status_pos = osd_log_status_pos;
	msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
	msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
	msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
	msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
	msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
	msp_osd_config.osd_display_name_pos = osd_display_name_pos;
	msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
	msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
	msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
	msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
	msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
	msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

	_msp.Send(MSP_OSD_CONFIG, &msp_osd_config);
}

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
	}

	if (!_is_initialized) {

		struct termios t;

		_msp_fd = open(_port, O_RDWR | O_NONBLOCK);

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

	msp_name_t name = {0};
	msp_fc_variant_t variant = {0};

	// update UORB topics
	//power_monitor_sub.update(&power_monitor_struct);
	_battery_status_sub.update(&_battery_status_struct);
	_vehicle_status_sub.update(&_vehicle_status_struct);
	_vehicle_gps_position_sub.update(&_vehicle_gps_position_struct);
	_airspeed_validated_sub.update(&_airspeed_validated_struct);
	_vehicle_air_data_sub.update(&_vehicle_air_data_struct);
	_home_position_sub.update(&_home_position_struct);
	_vehicle_global_position_sub.update(&_vehicle_global_position_struct);
	_vehicle_local_position_sub.update(&_vehicle_local_position_struct);
	_vehicle_attitude_sub.update(&_vehicle_attitude_struct);
	_estimator_status_sub.update(&_estimator_status_struct);
	_input_rc_sub.update(&_input_rc_struct);


	memcpy(variant.flightControlIdentifier, "BTFL", sizeof(variant.flightControlIdentifier));
	_msp.Send(MSP_FC_VARIANT, &variant) ? _performance_data.successful_sends++ : _performance_data.unsuccessful_sends++;

	// MSP_NAME
	snprintf(name.craft_name, sizeof(name.craft_name), "> %i", _x);
	name.craft_name[14] = '\0';
	_msp.Send(MSP_NAME, &name) ? _performance_data.successful_sends++ : _performance_data.unsuccessful_sends++;

	// MSP_STATUS
	if (true) {
		const auto msg = msp_osd::construct_STATUS(
			_vehicle_status_struct);
		this->Send(MSP_STATUS, &msg);
	}

	// MSP_ANALOG
	if (true) {
		const auto msg = msp_osd::construct_ANALOG(
			_battery_status_struct,
			_input_rc_struct);
		this->Send(MSP_ANALOG, &msg);
	}

	// MSP_BATTERY_STATE
	if (true) {
		const auto msg = msp_osd::construct_BATTERY_STATE(
			_battery_status_struct);
		this->Send(MSP_BATTERY_STATE, &msg);
	}

	// MSP_RAW_GPS
	if (true) {
		const auto msg = msp_osd::construct_RAW_GPS(
			_vehicle_gps_position_struct,
			_airspeed_validated_struct);
		this->Send(MSP_RAW_GPS, &msg);
	}

	// MSP_COMP_GPS
	if (true) {
		// update heartbeat
		_heartbeat = !_heartbeat;

		// construct and send message
		const auto msg = msp_osd::construct_COMP_GPS(
			_home_position_struct,
			_estimator_status_struct,
			_vehicle_global_position_struct,
			_heartbeat);
		this->Send(MSP_COMP_GPS, &msg);
	}

	// MSP_ATTITUDE
	if (true) {
		const auto msg = msp_osd::construct_ATTITUDE(
			_vehicle_attitude_struct);
		this->Send(MSP_ATTITUDE, &msg);
	}

	// MSP_ALTITUDE
	if (true) {
		// construct and send message
		const auto msg = msp_osd::construct_ALTITUDE(
			_vehicle_gps_position_struct,
			_estimator_status_struct,
			_vehicle_local_position_struct);
		this->Send(MSP_ALTITUDE, &msg);
	}

	// MSP_MOTOR_TELEMETRY
	if (true) {
		const auto msg = msp_osd::construct_ESC_SENSOR_DATA();
		this->Send(MSP_ESC_SENSOR_DATA, &msg);
	}

	// send full configuration
	SendConfig();
}

void MspOsd::Send(const unsigned int message_type, const void *payload)
{
	if (_msp.Send(message_type, payload))
		_performance_data.successful_sends++;
	else
		_performance_data.unsuccessful_sends++;
}

int MspOsd::task_spawn(int argc, char *argv[])
{
	MspOsd *instance = new MspOsd();

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
	PX4_INFO("Running on port %s", _port);
	PX4_INFO("\tinitialized: %d", _is_initialized);
	PX4_INFO("\tinitialization issues: %d", _performance_data.initialization_problems);
	PX4_INFO("\tsuccessful sends: %lu", _performance_data.successful_sends);
	PX4_INFO("\tunsuccessful sends: %lu", _performance_data.unsuccessful_sends);

	return 0;
}

int MspOsd::custom_command(int argc, char *argv[])
{
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
Msp OSD!

### Implementation
Does the things for the DJI Air Unit OSD

### Examples
CLI usage example:
$ msp_osd

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "msp_osd");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int msp_osd_main(int argc, char *argv[])
{
	return MspOsd::main(argc, argv);
}

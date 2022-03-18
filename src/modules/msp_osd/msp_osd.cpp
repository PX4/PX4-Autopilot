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

#include "MspV1.hpp"

const char arduPlaneModeStr[24][15]  =  {
	"MANUAL",
	"CIRCLE",
	"STABILIZE",
	"TRAINING",
	"ACRO",
	"FLY BY WIRE A",
	"FLY BY WIRE B",
	"CRUISE",
	"AUTOTUNE",
	"",
	"AUTO",
	"RTL",
	"LOITER",
	"TAKEOFF",
	"AVOID_ADSB",
	"GUIDED",
	"INITIALISING",
	"QSTABILIZE",
	"QHOVER",
	"QLOITER",
	"QLAND",
	"QRTL",
	"QAUTOTUNE",
	"QACRO"
};


//OSD elements positions
//in betaflight configurator set OSD elements to your desired positions and in CLI type "set osd" to retreieve the numbers.
//234 -> not visible. Horizontally 2048-2074(spacing 1), vertically 2048-2528(spacing 32). 26 characters X 15 lines

//currently working elements
const uint16_t osd_rssi_value_pos = 2179;
const uint16_t osd_avg_cell_voltage_pos = 2072;
const uint16_t osd_gps_lon_pos = 2113;
const uint16_t osd_gps_lat_pos = 2081;
const uint16_t osd_flymode_pos = 234;
const uint16_t osd_craft_name_pos = 2543;
const uint16_t osd_current_draw_pos = 2102;
const uint16_t osd_mah_drawn_pos = 2136;
uint16_t osd_gps_sats_pos = 2465;
const uint16_t osd_main_batt_voltage_pos = 234;
const uint16_t osd_pitch_angle_pos = 234;
const uint16_t osd_roll_angle_pos = 234;
const uint16_t osd_crosshairs_pos = 234;
const uint16_t osd_numerical_vario_pos = 2532;
const uint16_t osd_gps_speed_pos = 2394;
const uint16_t osd_altitude_pos = 2455;
const uint16_t osd_home_dir_pos = 2095;
const uint16_t osd_home_dist_pos = 2426;

//not implemented or not available
const uint16_t osd_throttle_pos_pos = 234;
const uint16_t osd_vtx_channel_pos = 234;
const uint16_t osd_roll_pids_pos = 234;
const uint16_t osd_pitch_pids_pos = 234;
const uint16_t osd_yaw_pids_pos = 234;
const uint16_t osd_power_pos = 234;
const uint16_t osd_pidrate_profile_pos = 234;
const uint16_t osd_warnings_pos = 234;
const uint16_t osd_debug_pos = 234;
const uint16_t osd_artificial_horizon_pos = 234;
const uint16_t osd_horizon_sidebars_pos = 234;
const uint16_t osd_item_timer_1_pos = 234;
const uint16_t osd_item_timer_2_pos = 234;
const uint16_t osd_main_batt_usage_pos = 234;
const uint16_t osd_disarmed_pos = 234;
const uint16_t osd_numerical_heading_pos = 234;
const uint16_t osd_compass_bar_pos = 234;
const uint16_t osd_esc_tmp_pos = 234;
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

int MspOsd::print_status()
{
	PX4_INFO("Running");

	return 0;
}

int MspOsd::custom_command(int argc, char *argv[])
{
	return 0;
}

int MspOsd::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
	  SCHED_DEFAULT,
	  SCHED_PRIORITY_DEFAULT,
	  1024,
	  (px4_main_t)&run_trampoline,
	  (char *const *)argv);

	if (_task_id < 0)
	{
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MspOsd *MspOsd::instantiate(int argc, char *argv[])
{
	MspOsd *instance = new MspOsd();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

MspOsd::MspOsd()
	: ModuleParams(nullptr),
	_msp(NULL)
{
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

void MspOsd::SendTelemetry()
{
	//msp_battery_state_t battery_state = {0};
	//msp_name_t name = {0};
	msp_status_BF_t status_BF = {0};
	msp_analog_t analog = {0};
	/*msp_raw_gps_t raw_gps = {0};
	msp_comp_gps_t comp_gps = {0};
	msp_attitude_t attitude = {0};
	msp_altitude_t altitude = {0};*/
	
    //MSP_NAME
    //memcpy(name.craft_name, craftname, sizeof(craftname));
    //msp.send(MSP_NAME, &name, sizeof(name));

    //MSP_STATUS
    status_BF.flightModeFlags = ARM_ACRO_BF;
    _msp.Send(MSP_STATUS, &status_BF);

    //MSP_ANALOG
    analog.vbat = 24;
    analog.rssi = 100;
    analog.amperage = 20;
    analog.mAhDrawn = 20;
    _msp.Send(MSP_ANALOG, &analog);

}

void MspOsd::run()
{
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;
	
	int fd = -1;
	struct termios t;

	fd = open("/dev/ttyS2", O_RDWR | O_NONBLOCK);
	tcgetattr(fd, &t);
	cfsetspeed(&t, 57600);
	t.c_cflag &= ~(CSTOPB | PARENB);
	tcsetattr(fd, TCSANOW, &t);

	parameters_update(true);
	
	_msp = MspV1(fd);
	
	

	while (!should_exit())
	{
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 100);

		if (pret == 0)
		{
			// Timeout: let the loop run anyway, don't do `continue` here
		}
		else if (pret < 0)
		{
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		}
		else if (fds[0].revents & POLLIN)
		{
			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
		}
		
		SendConfig();

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
}

void MspOsd::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
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

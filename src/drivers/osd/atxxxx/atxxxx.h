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

#pragma once

/**
 * @file atxxxx.h
 * @author Daniele Pettenuzzo
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */
#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <lib/osd/MessageDisplay.hpp>
#include <lib/osd/OsdTelemetry.hpp>
#include <parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#define OSD_SPI_BUS_SPEED (2000000L) /*  2 MHz  */

#define DIR_READ(a) ((a) | (1 << 7))
#define DIR_WRITE(a) ((a) & 0x7f)

#define OSD_CHARS_PER_ROW	30
#define OSD_NUM_ROWS_PAL	16
#define OSD_NUM_ROWS_NTSC	13
static constexpr uint8_t OSD_VM0 {0x00};
static constexpr uint8_t OSD_DMM{0x04};
static constexpr uint8_t OSD_DMAH{0x05};
static constexpr uint8_t OSD_DMAL{0x06};
static constexpr uint8_t OSD_DMDI{0x07};

static constexpr uint8_t OSD_VM0_PAL{1 << 6};
static constexpr uint8_t OSD_VM0_ENABLE_DISPLAY{1 << 3};
static constexpr uint8_t OSD_VM0_SOFTWARE_RESET{1 << 1};
static constexpr uint8_t OSD_VM0_DISABLE_VIDEO_BUFFER{1 << 0};
static constexpr uint8_t OSD_VM0_CONFIGURATION_MASK{OSD_VM0_PAL | OSD_VM0_ENABLE_DISPLAY |
	OSD_VM0_SOFTWARE_RESET | OSD_VM0_DISABLE_VIDEO_BUFFER};

extern "C" __EXPORT int atxxxx_main(int argc, char *argv[]);

class OSDatxxxx : public device::SPI, public ModuleParams, public I2CSPIDriver<OSDatxxxx>
{
public:
	OSDatxxxx(const I2CSPIDriverConfig &config);
	virtual ~OSDatxxxx() = default;

	static void print_usage();

	int init() override;

	void RunImpl();

protected:
	int probe() override;

private:
	int start();

	int reset();

	int init_osd();

	int readRegister(unsigned reg, uint8_t *data, unsigned count);
	int writeRegister(unsigned reg, uint8_t data);

	int write_character_to_screen(uint8_t c, uint8_t pos_x, uint8_t pos_y);
	int add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y);
	int add_string_to_screen(const char *str, uint8_t pos_x, uint8_t pos_y, int width);
	int flush_screen(int max_updates);

	int add_battery_voltage(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y);
	int add_consumed_mah(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y);
	int add_altitude(const vehicle_local_position_s &local_position, uint8_t pos_x, uint8_t pos_y);
	int add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y);
	bool enabled(osd::Symbol symbol) const;

	int enable_screen();
	int disable_screen();

	int update_screen();

	osd::MessageDisplay _display{};
	osd::Telemetry _telemetry{};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};
	uint8_t _screen[OSD_CHARS_PER_ROW * OSD_NUM_ROWS_PAL] {};
	uint8_t _displayed_screen[OSD_CHARS_PER_ROW * OSD_NUM_ROWS_PAL] {};
	int _flush_position{};
	bool _keep_running{false};
	bool _spi_initialized{false};
	bool _initialized{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_ATXXXX_CFG>) _param_osd_atxxxx_cfg,
		(ParamInt<px4::params::OSD_SYMBOLS>) _param_osd_symbols,
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::OSD_LOG_LEVEL>) _param_osd_log_level,
		(ParamInt<px4::params::OSD_SCROLL_RATE>) _param_osd_scroll_rate,
		(ParamInt<px4::params::OSD_DWELL_TIME>) _param_osd_dwell_time,
		(ParamInt<px4::params::OSD_BAT_VOLT_X>) _param_osd_bat_volt_x,
		(ParamInt<px4::params::OSD_BAT_VOLT_Y>) _param_osd_bat_volt_y,
		(ParamInt<px4::params::OSD_MAH_X>) _param_osd_mah_x,
		(ParamInt<px4::params::OSD_MAH_Y>) _param_osd_mah_y,
		(ParamInt<px4::params::OSD_CELL_V_X>) _param_osd_cell_v_x,
		(ParamInt<px4::params::OSD_CELL_V_Y>) _param_osd_cell_v_y,
		(ParamInt<px4::params::OSD_SYSID_X>) _param_osd_sysid_x,
		(ParamInt<px4::params::OSD_SYSID_Y>) _param_osd_sysid_y,
		(ParamInt<px4::params::OSD_MISSION_X>) _param_osd_mission_x,
		(ParamInt<px4::params::OSD_MISSION_Y>) _param_osd_mission_y,
		(ParamInt<px4::params::OSD_MAV_STATE_X>) _param_osd_mav_state_x,
		(ParamInt<px4::params::OSD_MAV_STATE_Y>) _param_osd_mav_state_y,
		(ParamInt<px4::params::OSD_RSSI_X>) _param_osd_rssi_x,
		(ParamInt<px4::params::OSD_RSSI_Y>) _param_osd_rssi_y,
		(ParamInt<px4::params::OSD_LQ_X>) _param_osd_lq_x,
		(ParamInt<px4::params::OSD_LQ_Y>) _param_osd_lq_y,
		(ParamInt<px4::params::OSD_GPS_SAT_X>) _param_osd_gps_sat_x,
		(ParamInt<px4::params::OSD_GPS_SAT_Y>) _param_osd_gps_sat_y,
		(ParamInt<px4::params::OSD_GPS_SPD_X>) _param_osd_gps_spd_x,
		(ParamInt<px4::params::OSD_GPS_SPD_Y>) _param_osd_gps_spd_y,
		(ParamInt<px4::params::OSD_GPS_INFO_X>) _param_osd_gps_info_x,
		(ParamInt<px4::params::OSD_GPS_INFO_Y>) _param_osd_gps_info_y,
		(ParamInt<px4::params::OSD_ALT_X>) _param_osd_alt_x,
		(ParamInt<px4::params::OSD_ALT_Y>) _param_osd_alt_y,
		(ParamInt<px4::params::OSD_HOME_DIR_X>) _param_osd_home_dir_x,
		(ParamInt<px4::params::OSD_HOME_DIR_Y>) _param_osd_home_dir_y,
		(ParamInt<px4::params::OSD_HOME_DST_X>) _param_osd_home_dst_x,
		(ParamInt<px4::params::OSD_HOME_DST_Y>) _param_osd_home_dst_y,
		(ParamInt<px4::params::OSD_AH_X>) _param_osd_ah_x,
		(ParamInt<px4::params::OSD_AH_Y>) _param_osd_ah_y,
		(ParamInt<px4::params::OSD_MODE_X>) _param_osd_mode_x,
		(ParamInt<px4::params::OSD_MODE_Y>) _param_osd_mode_y,
		(ParamInt<px4::params::OSD_FTIME_X>) _param_osd_ftime_x,
		(ParamInt<px4::params::OSD_FTIME_Y>) _param_osd_ftime_y,
		(ParamInt<px4::params::OSD_STATUS_X>) _param_osd_status_x,
		(ParamInt<px4::params::OSD_STATUS_Y>) _param_osd_status_y,
		(ParamInt<px4::params::OSD_ARM_X>) _param_osd_arm_x,
		(ParamInt<px4::params::OSD_ARM_Y>) _param_osd_arm_y,
		(ParamInt<px4::params::OSD_HEAD_X>) _param_osd_head_x,
		(ParamInt<px4::params::OSD_HEAD_Y>) _param_osd_head_y,
		(ParamInt<px4::params::OSD_CROSS_X>) _param_osd_cross_x,
		(ParamInt<px4::params::OSD_CROSS_Y>) _param_osd_cross_y,
		(ParamInt<px4::params::OSD_CURRENT_X>) _param_osd_current_x,
		(ParamInt<px4::params::OSD_CURRENT_Y>) _param_osd_current_y,
		(ParamInt<px4::params::OSD_POWER_X>) _param_osd_power_x,
		(ParamInt<px4::params::OSD_POWER_Y>) _param_osd_power_y,
		(ParamInt<px4::params::OSD_THROT_X>) _param_osd_throt_x,
		(ParamInt<px4::params::OSD_THROT_Y>) _param_osd_throt_y,
		(ParamInt<px4::params::OSD_VARIO_X>) _param_osd_vario_x,
		(ParamInt<px4::params::OSD_VARIO_Y>) _param_osd_vario_y,
		(ParamInt<px4::params::OSD_PITCH_X>) _param_osd_pitch_x,
		(ParamInt<px4::params::OSD_PITCH_Y>) _param_osd_pitch_y,
		(ParamInt<px4::params::OSD_ROLL_X>) _param_osd_roll_x,
		(ParamInt<px4::params::OSD_ROLL_Y>) _param_osd_roll_y,
		(ParamInt<px4::params::OSD_GPS_LAT_X>) _param_osd_gps_lat_x,
		(ParamInt<px4::params::OSD_GPS_LAT_Y>) _param_osd_gps_lat_y,
		(ParamInt<px4::params::OSD_GPS_LON_X>) _param_osd_gps_lon_x,
		(ParamInt<px4::params::OSD_GPS_LON_Y>) _param_osd_gps_lon_y,
		(ParamInt<px4::params::OSD_VTX_INFO_X>) _param_osd_vtx_info_x,
		(ParamInt<px4::params::OSD_VTX_INFO_Y>) _param_osd_vtx_info_y,
		(ParamInt<px4::params::OSD_VTX_FREQ_X>) _param_osd_vtx_freq_x,
		(ParamInt<px4::params::OSD_VTX_FREQ_Y>) _param_osd_vtx_freq_y,
		(ParamInt<px4::params::OSD_VTX_POWER_X>) _param_osd_vtx_power_x,
		(ParamInt<px4::params::OSD_VTX_POWER_Y>) _param_osd_vtx_power_y,
		(ParamInt<px4::params::OSD_CAM_VFOV>) _param_osd_cam_vfov,
		(ParamInt<px4::params::OSD_CAM_HFOV>) _param_osd_cam_hfov,
		(ParamInt<px4::params::OSD_CAM_UPT>) _param_osd_cam_upt
	)
};

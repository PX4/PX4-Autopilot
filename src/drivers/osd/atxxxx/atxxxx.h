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
	void add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y);
	void add_string_to_screen(const char *str, uint8_t pos_x, uint8_t pos_y, int width);
	int flush_screen(int max_updates);

	void add_battery_voltage(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y);
	void add_consumed_mah(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y);
	void add_altitude(const vehicle_local_position_s &local_position, uint8_t pos_x, uint8_t pos_y);
	void add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y);
	bool enabled(osd::Symbol symbol) const;

	int enable_screen();
	int disable_screen();

	int update_screen();

	osd::MessageDisplay _display{};
	osd::Telemetry _telemetry{};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};
	uint8_t _screen[OSD_CHARS_PER_ROW * OSD_NUM_ROWS_PAL] {};
	uint8_t _displayed_screen[OSD_CHARS_PER_ROW * OSD_NUM_ROWS_PAL] {};
	int _num_rows{OSD_NUM_ROWS_NTSC};	// rows of the active video standard, the smaller of the two until configured
	int _flush_position{};
	bool _keep_running{false};
	bool _spi_initialized{false};
	bool _initialized{false};


	// Element positions are looked up by name instead of being declared as individual
	// ParamInt members: one loop costs far less flash than 62 generated param_find calls.
	enum PositionParam : uint8_t {
		POS_BAT_VOLT_X,
		POS_BAT_VOLT_Y,
		POS_MAH_X,
		POS_MAH_Y,
		POS_CELL_V_X,
		POS_CELL_V_Y,
		POS_SYSID_X,
		POS_SYSID_Y,
		POS_MISSION_X,
		POS_MISSION_Y,
		POS_MAV_STATE_X,
		POS_MAV_STATE_Y,
		POS_RSSI_X,
		POS_RSSI_Y,
		POS_LQ_X,
		POS_LQ_Y,
		POS_GPS_SAT_X,
		POS_GPS_SAT_Y,
		POS_GPS_SPD_X,
		POS_GPS_SPD_Y,
		POS_GPS_INFO_X,
		POS_GPS_INFO_Y,
		POS_ALT_X,
		POS_ALT_Y,
		POS_HOME_DST_X,
		POS_HOME_DST_Y,
		POS_AH_X,
		POS_AH_Y,
		POS_MODE_X,
		POS_MODE_Y,
		POS_FTIME_X,
		POS_FTIME_Y,
		POS_STATUS_X,
		POS_STATUS_Y,
		POS_ARM_X,
		POS_ARM_Y,
		POS_HEAD_X,
		POS_HEAD_Y,
		POS_CROSS_X,
		POS_CROSS_Y,
		POS_CURRENT_X,
		POS_CURRENT_Y,
		POS_POWER_X,
		POS_POWER_Y,
		POS_THROT_X,
		POS_THROT_Y,
		POS_VARIO_X,
		POS_VARIO_Y,
		POS_PITCH_X,
		POS_PITCH_Y,
		POS_ROLL_X,
		POS_ROLL_Y,
		POS_GPS_LAT_X,
		POS_GPS_LAT_Y,
		POS_GPS_LON_X,
		POS_GPS_LON_Y,
		POS_VTX_INFO_X,
		POS_VTX_INFO_Y,
		POS_VTX_FREQ_X,
		POS_VTX_FREQ_Y,
		POS_VTX_POWER_X,
		POS_VTX_POWER_Y,
		POS_COUNT
	};

	void add_element_to_screen(const char *str, PositionParam element, int width);
	void add_centered_element_to_screen(const char *str, PositionParam element, int width);
	void mark_position_params_used();
	void update_position_params();
	int position(PositionParam p) const { return _position[p]; }

	int32_t _position[POS_COUNT] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_ATXXXX_CFG>) _param_osd_atxxxx_cfg,
		(ParamInt<px4::params::OSD_SYMBOLS>) _param_osd_symbols,
		(ParamInt<px4::params::OSD_LOG_LEVEL>) _param_osd_log_level,
		(ParamInt<px4::params::OSD_SCROLL_RATE>) _param_osd_scroll_rate,
		(ParamInt<px4::params::OSD_DWELL_TIME>) _param_osd_dwell_time,
		(ParamInt<px4::params::OSD_CAM_VFOV>) _param_osd_cam_vfov,
		(ParamInt<px4::params::OSD_CAM_HFOV>) _param_osd_cam_hfov,
		(ParamInt<px4::params::OSD_CAM_UPT>) _param_osd_cam_upt
	)
};

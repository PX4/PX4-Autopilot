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

/**
 * @file atxxxx.cpp
 * @author Daniele Pettenuzzo
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 * Driver for the ATXXXX chip (e.g. MAX7456) on the omnibus f4 fcu connected via SPI.
 */

#include "atxxxx.h"
#include "symbols.h"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <ctype.h>
#include <string.h>

using namespace time_literals;

static constexpr uint32_t OSD_UPDATE_RATE{100_ms};	// 10 Hz
static constexpr int OSD_MAX_UPDATES_PER_CYCLE{10};
static constexpr uint32_t OSD_RETRY_INTERVAL{500_ms};


static constexpr param_t POSITION_PARAM[] {
	static_cast<param_t>(px4::params::OSD_BAT_VOLT_X),
	static_cast<param_t>(px4::params::OSD_BAT_VOLT_Y),
	static_cast<param_t>(px4::params::OSD_MAH_X),
	static_cast<param_t>(px4::params::OSD_MAH_Y),
	static_cast<param_t>(px4::params::OSD_CELL_V_X),
	static_cast<param_t>(px4::params::OSD_CELL_V_Y),
	static_cast<param_t>(px4::params::OSD_SYSID_X),
	static_cast<param_t>(px4::params::OSD_SYSID_Y),
	static_cast<param_t>(px4::params::OSD_MISSION_X),
	static_cast<param_t>(px4::params::OSD_MISSION_Y),
	static_cast<param_t>(px4::params::OSD_MAV_STATE_X),
	static_cast<param_t>(px4::params::OSD_MAV_STATE_Y),
	static_cast<param_t>(px4::params::OSD_RSSI_X),
	static_cast<param_t>(px4::params::OSD_RSSI_Y),
	static_cast<param_t>(px4::params::OSD_LQ_X),
	static_cast<param_t>(px4::params::OSD_LQ_Y),
	static_cast<param_t>(px4::params::OSD_GPS_SAT_X),
	static_cast<param_t>(px4::params::OSD_GPS_SAT_Y),
	static_cast<param_t>(px4::params::OSD_GPS_SPD_X),
	static_cast<param_t>(px4::params::OSD_GPS_SPD_Y),
	static_cast<param_t>(px4::params::OSD_GPS_INFO_X),
	static_cast<param_t>(px4::params::OSD_GPS_INFO_Y),
	static_cast<param_t>(px4::params::OSD_ALT_X),
	static_cast<param_t>(px4::params::OSD_ALT_Y),
	static_cast<param_t>(px4::params::OSD_HOME_DST_X),
	static_cast<param_t>(px4::params::OSD_HOME_DST_Y),
	static_cast<param_t>(px4::params::OSD_AH_X),
	static_cast<param_t>(px4::params::OSD_AH_Y),
	static_cast<param_t>(px4::params::OSD_MODE_X),
	static_cast<param_t>(px4::params::OSD_MODE_Y),
	static_cast<param_t>(px4::params::OSD_FTIME_X),
	static_cast<param_t>(px4::params::OSD_FTIME_Y),
	static_cast<param_t>(px4::params::OSD_STATUS_X),
	static_cast<param_t>(px4::params::OSD_STATUS_Y),
	static_cast<param_t>(px4::params::OSD_ARM_X),
	static_cast<param_t>(px4::params::OSD_ARM_Y),
	static_cast<param_t>(px4::params::OSD_HEAD_X),
	static_cast<param_t>(px4::params::OSD_HEAD_Y),
	static_cast<param_t>(px4::params::OSD_CROSS_X),
	static_cast<param_t>(px4::params::OSD_CROSS_Y),
	static_cast<param_t>(px4::params::OSD_CURRENT_X),
	static_cast<param_t>(px4::params::OSD_CURRENT_Y),
	static_cast<param_t>(px4::params::OSD_POWER_X),
	static_cast<param_t>(px4::params::OSD_POWER_Y),
	static_cast<param_t>(px4::params::OSD_THROT_X),
	static_cast<param_t>(px4::params::OSD_THROT_Y),
	static_cast<param_t>(px4::params::OSD_VARIO_X),
	static_cast<param_t>(px4::params::OSD_VARIO_Y),
	static_cast<param_t>(px4::params::OSD_PITCH_X),
	static_cast<param_t>(px4::params::OSD_PITCH_Y),
	static_cast<param_t>(px4::params::OSD_ROLL_X),
	static_cast<param_t>(px4::params::OSD_ROLL_Y),
	static_cast<param_t>(px4::params::OSD_GPS_LAT_X),
	static_cast<param_t>(px4::params::OSD_GPS_LAT_Y),
	static_cast<param_t>(px4::params::OSD_GPS_LON_X),
	static_cast<param_t>(px4::params::OSD_GPS_LON_Y),
	static_cast<param_t>(px4::params::OSD_VTX_INFO_X),
	static_cast<param_t>(px4::params::OSD_VTX_INFO_Y),
	static_cast<param_t>(px4::params::OSD_VTX_FREQ_X),
	static_cast<param_t>(px4::params::OSD_VTX_FREQ_Y),
	static_cast<param_t>(px4::params::OSD_VTX_POWER_X),
	static_cast<param_t>(px4::params::OSD_VTX_POWER_Y),
};

void
OSDatxxxx::mark_position_params_used()
{
	static_assert(sizeof(POSITION_PARAM) / sizeof(POSITION_PARAM[0]) == POS_COUNT,
		      "position parameter table does not match PositionParam");

	// mark them used so they are still reported to a GCS, as ParamInt members would
	for (unsigned i = 0; i < POS_COUNT; ++i) {
		param_set_used(POSITION_PARAM[i]);
	}
}

void
OSDatxxxx::update_position_params()
{
	for (unsigned i = 0; i < POS_COUNT; ++i) {
		param_get(POSITION_PARAM[i], &_position[i]);
	}
}

OSDatxxxx::OSDatxxxx(const I2CSPIDriverConfig &config) :
	SPI(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_keep_running(config.keep_running)
{
	mark_position_params_used();
	update_position_params();
	_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
	_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);
}

int
OSDatxxxx::init()
{
	int ret = PX4_OK;

	if (!_spi_initialized) {
		ret = SPI::init();

		if (ret == PX4_OK) {
			_spi_initialized = true;
		}
	}

	if (ret == PX4_OK) {
		ret = reset();
	}

	if (ret == PX4_OK) {
		ret = init_osd();
	}

	if (ret == PX4_OK) {
		memset(_screen, ' ', sizeof(_screen));
		memset(_displayed_screen, 0xff, sizeof(_displayed_screen));
		ret = flush_screen(sizeof(_screen));
	}

	if (ret == PX4_OK) {
		ret = enable_screen();
	}

	if (ret != PX4_OK) {
		if (_keep_running) {
			ScheduleDelayed(OSD_RETRY_INTERVAL);
			return PX4_OK;
		}

		return ret;
	}

	_initialized = true;
	return start();
}

int
OSDatxxxx::start()
{
	ScheduleOnInterval(OSD_UPDATE_RATE, 10000);

	return PX4_OK;
}

int
OSDatxxxx::probe()
{
	uint8_t data = 0;
	int ret = writeRegister(OSD_VM0, OSD_VM0_DISABLE_VIDEO_BUFFER);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = readRegister(OSD_VM0, &data, 1);

	if (ret != PX4_OK || data != OSD_VM0_DISABLE_VIDEO_BUFFER) {
		PX4_DEBUG("probe failed: transfer %d, VM0 0x%02x", ret, data);
		return ret != PX4_OK ? ret : PX4_ERROR;
	}

	return PX4_OK;
}

int
OSDatxxxx::init_osd()
{
	uint8_t data = 0;

	_num_rows = _param_osd_atxxxx_cfg.get() == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL;

	if (_param_osd_atxxxx_cfg.get() == 2) {
		data |= OSD_VM0_PAL;
	}

	int ret = writeRegister(OSD_VM0, data);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = writeRegister(OSD_DMM, 0);

	if (ret != PX4_OK) {
		return ret;
	}

	return PX4_OK;
}

int
OSDatxxxx::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
	uint8_t cmd[5] {}; // read up to 4 bytes

	cmd[0] = DIR_READ(reg);

	int ret = transfer(&cmd[0], &cmd[0], count + 1);

	if (ret != PX4_OK) {
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	memcpy(&data[0], &cmd[1], count);

	return ret;
}

int
OSDatxxxx::writeRegister(unsigned reg, uint8_t data)
{
	uint8_t cmd[2] {}; // write 1 byte

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	int ret = transfer(&cmd[0], nullptr, 2);

	if (OK != ret) {
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	return ret;
}

int
OSDatxxxx::write_character_to_screen(uint8_t c, uint8_t pos_x, uint8_t pos_y)
{
	const uint16_t position = OSD_CHARS_PER_ROW * pos_y + pos_x;
	int ret = writeRegister(OSD_DMAH, position > 0xff ? 0x01 : 0x00);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = writeRegister(OSD_DMAL, static_cast<uint8_t>(position));

	if (ret != PX4_OK) {
		return ret;
	}

	return writeRegister(OSD_DMDI, c);
}

void
OSDatxxxx::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{
	// characters outside the active screen are not drawn: the PAL buffer is
	// always allocated, but NTSC only shows the first OSD_NUM_ROWS_NTSC rows
	if (pos_x >= OSD_CHARS_PER_ROW || pos_y >= _num_rows) {
		return;
	}

	_screen[OSD_CHARS_PER_ROW * pos_y + pos_x] = static_cast<uint8_t>(c);
}

void
OSDatxxxx::add_element_to_screen(const char *str, PositionParam element, int width)
{
	// the X entry is followed by its Y entry, so one index locates both
	add_string_to_screen(str, _position[element], _position[element + 1], width);
}

void
OSDatxxxx::add_centered_element_to_screen(const char *str, PositionParam element, int width)
{
	// these elements hold variable-length text, so their X entry is the centre
	// column rather than the left edge. The start column is clamped while still
	// signed: add_string_to_screen() takes an unsigned column, and a negative one
	// would wrap far past the row and drop or misplace the text.
	const int len = math::min(static_cast<int>(strlen(str)), width);
	const int pos_x = math::max(0, position(element) - len / 2);
	add_string_to_screen(str, pos_x, _position[element + 1], width);
}

void
OSDatxxxx::add_string_to_screen(const char *str, uint8_t pos_x, uint8_t pos_y, int width)
{
	// width only clips the string: update_screen() blanks the whole buffer before
	// drawing, so padding the remainder would erase elements sharing the row
	for (int i = 0; i < width && str[i] != '\0'; ++i) {
		add_character_to_screen(str[i], pos_x + i, pos_y);
	}
}

int
OSDatxxxx::flush_screen(int max_updates)
{
	const int screen_size = OSD_CHARS_PER_ROW * _num_rows;
	int total_updates = 0;

	while (total_updates < max_updates) {
		int position = -1;

		for (int offset = 0; offset < screen_size; ++offset) {
			const int candidate = (_flush_position + offset) % screen_size;

			if (_screen[candidate] != _displayed_screen[candidate]) {
				position = candidate;
				break;
			}
		}

		if (position < 0) {
			break;
		}

		const int ret = write_character_to_screen(_screen[position], position % OSD_CHARS_PER_ROW,
				position / OSD_CHARS_PER_ROW);

		if (ret != PX4_OK) {
			return ret;
		}

		_displayed_screen[position] = _screen[position];
		_flush_position = (position + 1) % screen_size;
		++total_updates;
	}

	return PX4_OK;
}

void
OSDatxxxx::add_battery_voltage(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
	char batt_symbol = OSD_SYMBOL_BATT_EMPTY;
	const float remaining = PX4_ISFINITE(battery.remaining) ? battery.remaining : -1.f;
	const float voltage_v = PX4_ISFINITE(battery.voltage_v) ? battery.voltage_v : 0.f;

	if (remaining >= 0.875f) {
		batt_symbol = OSD_SYMBOL_BATT_FULL;

	} else if (remaining >= 0.625f) {
		batt_symbol = OSD_SYMBOL_BATT_5;

	} else if (remaining >= 0.375f) {
		batt_symbol = OSD_SYMBOL_BATT_4;

	} else if (remaining >= 0.25f) {
		batt_symbol = OSD_SYMBOL_BATT_3;

	} else if (remaining >= 0.125f) {
		batt_symbol = OSD_SYMBOL_BATT_2;

	} else if (remaining >= 0.f) {
		batt_symbol = OSD_SYMBOL_BATT_1;
	}

	snprintf(buf, sizeof(buf), "%c%5.2f", batt_symbol, (double)voltage_v);
	buf[sizeof(buf) - 1] = '\0';

	buf[6] = 'V';
	buf[7] = '\0';

	add_string_to_screen(buf, pos_x, pos_y, 7);
}

void
OSDatxxxx::add_consumed_mah(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y)
{
	char buf[7];
	const float discharged_mah = PX4_ISFINITE(battery.discharged_mah) ? battery.discharged_mah : 0.f;

	snprintf(buf, sizeof(buf), "%5d", (int)discharged_mah);
	buf[5] = OSD_SYMBOL_MAH;
	buf[6] = '\0';

	add_string_to_screen(buf, pos_x, pos_y, 6);
}

void
OSDatxxxx::add_altitude(const vehicle_local_position_s &local_position, uint8_t pos_x, uint8_t pos_y)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%c%5.1f%c", OSD_SYMBOL_ARROW_NORTH, (double) - local_position.z, OSD_SYMBOL_M);
	buf[sizeof(buf) - 1] = '\0';

	add_string_to_screen(buf, pos_x, pos_y, 9);
}

void
OSDatxxxx::add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];

	const int total_seconds = static_cast<int>(flight_time);
	const int minutes = total_seconds / 60;
	const int seconds = total_seconds % 60;

	snprintf(buf, sizeof(buf), "%c%02d:%02d", OSD_SYMBOL_FLIGHT_TIME, minutes, seconds);
	buf[sizeof(buf) - 1] = '\0';

	add_string_to_screen(buf, pos_x, pos_y, 6);
}

int
OSDatxxxx::enable_screen()
{
	uint8_t data = 0;
	int ret = readRegister(OSD_VM0, &data, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	return writeRegister(OSD_VM0, data | OSD_VM0_ENABLE_DISPLAY);
}

int
OSDatxxxx::disable_screen()
{
	uint8_t data = 0;
	int ret = readRegister(OSD_VM0, &data, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	return writeRegister(OSD_VM0, data & ~OSD_VM0_ENABLE_DISPLAY);
}

int
OSDatxxxx::update_screen()
{
	const osd::TelemetryData &telemetry = _telemetry.data();
	char buf[16] {};
	const int horizon_x = position(POS_AH_X);
	const int horizon_y = position(POS_AH_Y);
	const battery_status_s battery = telemetry.battery_valid ? telemetry.battery : battery_status_s{};
	const float roll_rad = telemetry.attitude_valid ? telemetry.roll_rad : 0.f;
	const float pitch_rad = telemetry.attitude_valid ? telemetry.pitch_rad : 0.f;
	const float yaw_rad = telemetry.attitude_valid ? telemetry.yaw_rad : 0.f;
	memset(_screen, ' ', sizeof(_screen));

	if (enabled(osd::Symbol::ArtificialHorizon)) {
		float roll = -matrix::wrap_pi(roll_rad);

		if (roll > M_PI_2_F) {
			roll -= M_PI_F;

		} else if (roll < -M_PI_2_F) {
			roll += M_PI_F;
		}

		const float camera_pitch = pitch_rad + math::radians(static_cast<float>(_param_osd_cam_upt.get()));
		const float half_vertical_fov = math::radians(static_cast<float>(_param_osd_cam_vfov.get())) * 0.5f;
		const float half_horizontal_fov = math::radians(static_cast<float>(_param_osd_cam_hfov.get())) * 0.5f;

		if (fabsf(camera_pitch) < M_PI_2_F) {
			const float screen_height_subrows = _num_rows * 9.f;
			const float pitch_subrows = tanf(camera_pitch) / tanf(half_vertical_fov) * screen_height_subrows * 0.5f;
			const float subrows_per_column = screen_height_subrows / OSD_CHARS_PER_ROW *
							 tanf(half_horizontal_fov) / tanf(half_vertical_fov);
			const float sin_roll = sinf(roll);
			const float cos_roll = cosf(roll);

			if (fabsf(cos_roll) >= fabsf(sin_roll)) {
				for (int x = -4; x <= 4; ++x) {
					const int subrow = lroundf(pitch_subrows + sin_roll / cos_roll * x * subrows_per_column);
					const int row_offset = floorf(static_cast<float>(subrow) / 9.f);
					const int glyph_offset = subrow - row_offset * 9;

					if (horizon_y + row_offset >= 0 && horizon_y + row_offset < _num_rows) {
						add_character_to_screen(OSD_SYMBOL_AH_BAR9_0 + glyph_offset, horizon_x + x,
									horizon_y + row_offset);
					}
				}

			} else {
				for (int y = -horizon_y; y < _num_rows - horizon_y; ++y) {
					const int x = lroundf((y * 9.f - pitch_subrows) * cos_roll /
							      (sin_roll * subrows_per_column));

					if (x >= -4 && x <= 4) {
						add_character_to_screen('|', horizon_x + x, horizon_y + y);
					}
				}
			}
		}
	}

	if (enabled(osd::Symbol::Crosshairs)) {
		const int crosshair_x = position(POS_CROSS_X);
		const int crosshair_y = position(POS_CROSS_Y);
		add_character_to_screen(OSD_SYMBOL_AH_CENTER, crosshair_x, crosshair_y);
	}

	if (enabled(osd::Symbol::MainBatteryVoltage)) {
		add_battery_voltage(battery, position(POS_BAT_VOLT_X), position(POS_BAT_VOLT_Y));
	}

	if (enabled(osd::Symbol::MahDrawn)) {
		add_consumed_mah(battery, position(POS_MAH_X), position(POS_MAH_Y));
	}

	if (enabled(osd::Symbol::AverageCellVoltage)) {
		const float cell_voltage = battery.cell_count > 0 ? battery.voltage_v / battery.cell_count : 0.f;
		snprintf(buf, sizeof(buf), "%c%4.2fV", OSD_SYMBOL_BATT_EMPTY, (double)cell_voltage);
		add_element_to_screen(buf, POS_CELL_V_X, 7);
	}

	if (enabled(osd::Symbol::CurrentDraw)) {
		const float current_a = PX4_ISFINITE(battery.current_a) ? battery.current_a : 0.f;
		snprintf(buf, sizeof(buf), "%c%4.1fA", OSD_SYMBOL_AMP, (double)current_a);
		add_element_to_screen(buf, POS_CURRENT_X, 6);
	}

	if (enabled(osd::Symbol::Power)) {
		const float current_a = PX4_ISFINITE(battery.current_a) ? battery.current_a : 0.f;
		snprintf(buf, sizeof(buf), "%4.0fW", (double)(battery.voltage_v * current_a));
		add_element_to_screen(buf, POS_POWER_X, 5);
	}

	if (enabled(osd::Symbol::SystemId)) {
		// vehicle_status carries the MAVLink system ID, so the driver does not depend on
		// MAV_SYS_ID: that parameter only exists on boards that build the mavlink module
		snprintf(buf, sizeof(buf), "S%03d", telemetry.status.system_id);
		add_element_to_screen(buf, POS_SYSID_X, 4);
	}

	if (enabled(osd::Symbol::MavState)) {
		const char *mav_state = "MAVINI";

		if (telemetry.actuator_armed.termination || telemetry.actuator_armed.kill ||
		    (telemetry.actuator_armed.lockdown && telemetry.status.hil_state == vehicle_status_s::HIL_STATE_OFF)) {
			mav_state = "MAVTRM";

		} else if (telemetry.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			mav_state = telemetry.status.failsafe ? "MAVCRT" : "MAVACT";

		} else if (telemetry.status.calibration_enabled || telemetry.status.rc_calibration_in_progress ||
			   telemetry.actuator_armed.in_esc_calibration_mode) {
			mav_state = "MAVCAL";

		} else if (telemetry.status.pre_flight_checks_pass) {
			mav_state = "MAVSTB";
		}

		add_centered_element_to_screen(mav_state, POS_MAV_STATE_X, 6);
	}

	if (enabled(osd::Symbol::Rssi)) {
		const bool input_rc_valid = telemetry.input_rc.timestamp != 0;

		if (input_rc_valid && PX4_ISFINITE(telemetry.input_rc.rssi_dbm)) {
			snprintf(buf, sizeof(buf), "%c%4.0f", OSD_SYMBOL_RSSI, (double)telemetry.input_rc.rssi_dbm);

		} else {
			int rssi = input_rc_valid && telemetry.input_rc.rssi >= 0
				   && telemetry.input_rc.rssi <= input_rc_s::RSSI_MAX ? telemetry.input_rc.rssi : 0;

			if (rssi == 0 && telemetry.radio_status.timestamp != 0) {
				const int radio_rssi = telemetry.radio_status.remote_rssi != 0 ? telemetry.radio_status.remote_rssi :
						       telemetry.radio_status.rssi;
				rssi = math::constrain(radio_rssi, 0, 254);
			}

			snprintf(buf, sizeof(buf), "%c%3d", OSD_SYMBOL_RSSI, rssi);
		}

		add_element_to_screen(buf, POS_RSSI_X, 5);
	}

	if (enabled(osd::Symbol::LinkQuality)) {
		const int link_quality = telemetry.input_rc.timestamp != 0 && telemetry.input_rc.link_quality >= 0
					 ? telemetry.input_rc.link_quality : 0;
		snprintf(buf, sizeof(buf), "LQ%3d", link_quality);
		add_element_to_screen(buf, POS_LQ_X, 5);
	}

	if (enabled(osd::Symbol::GpsSatellites)) {
		const int satellites = telemetry.gps.timestamp != 0 ? telemetry.gps.satellites_used : 0;
		snprintf(buf, sizeof(buf), "%c%c%2d", OSD_SYMBOL_SAT_L, OSD_SYMBOL_SAT_R, satellites);
		add_element_to_screen(buf, POS_GPS_SAT_X, 4);
	}

	if (enabled(osd::Symbol::GpsSpeed)) {
		const float speed = telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && PX4_ISFINITE(telemetry.gps.vel_m_s)
				    ? telemetry.gps.vel_m_s * 3.6f : 0.f;
		snprintf(buf, sizeof(buf), "SPD%3.0f", (double)speed);
		add_element_to_screen(buf, POS_GPS_SPD_X, 6);
	}

	if (enabled(osd::Symbol::GpsInfo)) {
		const char *fix_type = telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_3D ? "3D" :
				       (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D ? "2D" : "NO");
		const float hdop = PX4_ISFINITE(telemetry.gps.hdop) ? telemetry.gps.hdop : 0.f;
		const float vdop = PX4_ISFINITE(telemetry.gps.vdop) ? telemetry.gps.vdop : 0.f;
		const float eph = PX4_ISFINITE(telemetry.gps.eph) ? telemetry.gps.eph : 0.f;
		const float pdop = sqrtf(hdop * hdop + vdop * vdop);
		snprintf(buf, sizeof(buf), "%s D%3.1f E%3.1f", fix_type, (double)pdop, (double)eph);
		add_element_to_screen(buf, POS_GPS_INFO_X, 12);
	}

	if (enabled(osd::Symbol::GpsLatitude)) {
		const double latitude = telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && PX4_ISFINITE(telemetry.gps.latitude_deg)
					? telemetry.gps.latitude_deg : 0.;
		snprintf(buf, sizeof(buf), "LAT%+.5f", latitude);
		add_element_to_screen(buf, POS_GPS_LAT_X, 13);
	}

	if (enabled(osd::Symbol::GpsLongitude)) {
		const double longitude = telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && PX4_ISFINITE(telemetry.gps.longitude_deg)
					 ? telemetry.gps.longitude_deg : 0.;
		snprintf(buf, sizeof(buf), "LON%+.5f", longitude);
		add_element_to_screen(buf, POS_GPS_LON_X, 14);
	}

	if (enabled(osd::Symbol::Altitude)) {
		vehicle_local_position_s local_position{};
		local_position.z = telemetry.local_position.z_valid && PX4_ISFINITE(telemetry.local_position.z)
				   ? telemetry.local_position.z : 0.f;
		add_altitude(local_position, position(POS_ALT_X), position(POS_ALT_Y));
	}

	if (enabled(osd::Symbol::NumericalVario)) {
		const float vertical_speed = telemetry.local_position.v_z_valid && PX4_ISFINITE(telemetry.local_position.vz)
					     ? -telemetry.local_position.vz : 0.f;
		snprintf(buf, sizeof(buf), "V%+4.1f", (double)vertical_speed);
		add_element_to_screen(buf, POS_VARIO_X, 7);
	}

	if (enabled(osd::Symbol::PitchAngle)) {
		snprintf(buf, sizeof(buf), "P%+4.0f", (double)math::degrees(pitch_rad));
		add_element_to_screen(buf, POS_PITCH_X, 6);
	}

	if (enabled(osd::Symbol::RollAngle)) {
		snprintf(buf, sizeof(buf), "R%+4.0f", (double)math::degrees(roll_rad));
		add_element_to_screen(buf, POS_ROLL_X, 6);
	}

	if (enabled(osd::Symbol::MissionState)) {
		const mission_s &mission = telemetry.mission;
		const mission_result_s &result = telemetry.mission_result;
		const bool stored = mission.timestamp != 0 && mission.count > 0;
		// the feasibility check runs only once home, global position and the geofence are
		// ready, and stamps the mission it ran against. An id mismatch means the uploaded
		// mission has not been checked yet, which is not the same as it having been rejected
		const bool checked = result.timestamp != 0 && result.mission_id == mission.mission_id;

		if (!stored) {
			strncpy(buf, "MISNONE", sizeof(buf));

		} else if (!checked) {
			strncpy(buf, "MISWAIT", sizeof(buf));

		} else if (result.failure) {
			strncpy(buf, "MISFAIL", sizeof(buf));

		} else if (!result.valid) {
			strncpy(buf, "MISINVAL", sizeof(buf));

		} else if (result.finished) {
			strncpy(buf, "MISDONE", sizeof(buf));

		} else if (result.warning) {
			strncpy(buf, "MISWARN", sizeof(buf));

		} else {
			snprintf(buf, sizeof(buf), "MIS%02u/%02u",
				 static_cast<unsigned>(math::min(result.seq_current, static_cast<uint16_t>(99))),
				 static_cast<unsigned>(math::min(result.seq_total, static_cast<uint16_t>(99))));
		}

		add_element_to_screen(buf, POS_MISSION_X, 8);
	}

	if (enabled(osd::Symbol::HomeDistance)) {
		const float home_distance = telemetry.home_valid ? telemetry.home_distance_m : 0.f;
		snprintf(buf, sizeof(buf), "%4.0f%c", (double)home_distance, OSD_SYMBOL_M);
		add_element_to_screen(buf, POS_HOME_DST_X, 6);
	}

	const char *flight_mode = _telemetry.flight_mode();

	if (telemetry.status.timestamp == 0) {
		flight_mode = "NO STATUS";
	}

	if (enabled(osd::Symbol::FlightMode)) {
		strncpy(buf, flight_mode, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = '\0';

		for (int i = 0; buf[i] != '\0'; ++i) {
			buf[i] = toupper(static_cast<unsigned char>(buf[i]));
		}

		add_centered_element_to_screen(buf, POS_MODE_X, 14);
	}

	if (enabled(osd::Symbol::Disarmed)) {
		const char *arming_state = telemetry.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
					   ? "ARMED" : "DISARMED";
		add_centered_element_to_screen(arming_state, POS_ARM_X, 8);
	}

	if (enabled(osd::Symbol::Heading)) {
		const int heading_deg = static_cast<int>(lroundf(math::degrees(yaw_rad))) % 360;
		snprintf(buf, sizeof(buf), "%c%03d", OSD_SYMBOL_ARROW_NORTH, heading_deg);
		add_element_to_screen(buf, POS_HEAD_X, 4);
	}

	if (enabled(osd::Symbol::Throttle)) {
		const bool manual_control_valid = telemetry.manual_control.valid && telemetry.manual_control.timestamp != 0 &&
						  hrt_elapsed_time(&telemetry.manual_control.timestamp) < 1_s &&
						  PX4_ISFINITE(telemetry.manual_control.throttle);
		const float throttle = manual_control_valid
				       ? math::constrain((telemetry.manual_control.throttle + 1.f) * 50.f, 0.f, 100.f) : 0.f;
		snprintf(buf, sizeof(buf), "T%3.0f", (double)throttle);
		add_element_to_screen(buf, POS_THROT_X, 4);
	}

#if defined(CONFIG_DRIVERS_VTX)

	if (enabled(osd::Symbol::VtxInfo)) {
		if (telemetry.vtx.timestamp != 0 && telemetry.vtx.band >= 0 && telemetry.vtx.channel >= 0 &&
		    telemetry.vtx.power_level >= 0 && telemetry.vtx.band_letter != 0) {
			snprintf(buf, sizeof(buf), "VTX %c:%d:%d", telemetry.vtx.band_letter, telemetry.vtx.channel + 1,
				 telemetry.vtx.power_level + 1);

		} else {
			strncpy(buf, "VTX -:0:0", sizeof(buf));
		}

		add_element_to_screen(buf, POS_VTX_INFO_X, 11);
	}

	if (enabled(osd::Symbol::VtxFrequency)) {
		const uint16_t frequency = telemetry.vtx.timestamp != 0 ? telemetry.vtx.frequency : 0;
		snprintf(buf, sizeof(buf), "VTF: %huM", frequency);
		add_element_to_screen(buf, POS_VTX_FREQ_X, 11);
	}

	if (enabled(osd::Symbol::VtxPower)) {
		if (telemetry.vtx.timestamp != 0 && telemetry.vtx.power_level >= 0 && telemetry.vtx.power_label[0] != 0) {
			snprintf(buf, sizeof(buf), "VTW: %.*sMW", (int)sizeof(telemetry.vtx.power_label),
				 reinterpret_cast<const char *>(telemetry.vtx.power_label));

		} else {
			snprintf(buf, sizeof(buf), "VTW: 0MW");
		}

		add_element_to_screen(buf, POS_VTX_POWER_X, 12);
	}

#endif

	if (enabled(osd::Symbol::FlightTime)) {
		add_flighttime(_telemetry.flight_time_s(), position(POS_FTIME_X), position(POS_FTIME_Y));
	}

	_telemetry.update_message_display(_param_osd_log_level.get(), _display);
	char message[FULL_MSG_BUFFER] {};
	_display.get(message, hrt_absolute_time());

	if (enabled(osd::Symbol::StatusMessage)) {
		if (message[0] == '\0') {
			const failsafe_flags_s &failsafe_flags = telemetry.failsafe_flags;

			if (telemetry.status.timestamp == 0) {
				strncpy(message, "NO STATUS", sizeof(message));

			} else if (telemetry.status.failsafe) {
				strncpy(message, "FAILSAFE", sizeof(message));

			} else if (failsafe_flags.fd_critical_failure ||
				   failsafe_flags.fd_esc_arming_failure ||
				   failsafe_flags.fd_imbalanced_prop ||
				   failsafe_flags.fd_motor_failure ||
				   failsafe_flags.fd_alt_loss) {
				strncpy(message, "FAILURE", sizeof(message));

			} else if (failsafe_flags.battery_warning != 0 || failsafe_flags.battery_unhealthy) {
				strncpy(message, "BATT WARN", sizeof(message));

			} else if (failsafe_flags.manual_control_signal_lost) {
				strncpy(message, "NO RC", sizeof(message));

			} else if (failsafe_flags.attitude_invalid) {
				strncpy(message, "NO ATT", sizeof(message));

			} else if (failsafe_flags.local_position_invalid) {
				strncpy(message, "NO POS", sizeof(message));

			} else if (failsafe_flags.global_position_invalid) {
				strncpy(message, "NO GPS", sizeof(message));

			} else if (failsafe_flags.home_position_invalid) {
				strncpy(message, "NO HOME", sizeof(message));

			} else if (telemetry.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				strncpy(message, "ARMED", sizeof(message));

			} else if (telemetry.status.pre_flight_checks_pass) {
				strncpy(message, "READY", sizeof(message));

			} else {
				strncpy(message, "NOT READY", sizeof(message));
			}
		}

		add_centered_element_to_screen(message, POS_STATUS_X, FULL_MSG_LENGTH);
	}

	return flush_screen(OSD_MAX_UPDATES_PER_CYCLE);
}

bool
OSDatxxxx::enabled(osd::Symbol symbol) const
{
	return _param_osd_symbols.get() & (1u << static_cast<uint8_t>(symbol));
}

int
OSDatxxxx::reset()
{
	int ret = writeRegister(OSD_VM0, OSD_VM0_SOFTWARE_RESET);
	usleep(100);

	return ret;
}

void
OSDatxxxx::RunImpl()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		init();
		return;
	}

	uint8_t vm0 = 0;
	const uint8_t expected_vm0 = OSD_VM0_ENABLE_DISPLAY |
				     (_param_osd_atxxxx_cfg.get() == 2 ? OSD_VM0_PAL : 0);
	const int ret = readRegister(OSD_VM0, &vm0, 1);

	if (ret != PX4_OK || (vm0 & OSD_VM0_CONFIGURATION_MASK) != expected_vm0) {
		_initialized = false;
		ScheduleDelayed(OSD_RETRY_INTERVAL);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		update_position_params();
		_num_rows = _param_osd_atxxxx_cfg.get() == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL;
		_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
		_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);
	}

	_telemetry.update();
	update_screen();
}

void
OSDatxxxx::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
OSD driver for the ATXXXX chip that is mounted on the OmnibusF4SD board for example.

It can be enabled with the OSD_ATXXXX_CFG parameter.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("atxxxx", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
atxxxx_main(int argc, char *argv[])
{
	using ThisDriver = OSDatxxxx;
	BusCLIArguments cli{false, true};
	cli.spi_mode = SPIDEV_MODE0;
	cli.default_spi_frequency = OSD_SPI_BUS_SPEED;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_OSD_DEVTYPE_ATXXXX);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

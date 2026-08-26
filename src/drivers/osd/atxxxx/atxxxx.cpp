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
static constexpr int OSD_MAX_UPDATES_PER_CYCLE{20};
static constexpr uint32_t OSD_RETRY_INTERVAL{500_ms};



OSDatxxxx::OSDatxxxx(const I2CSPIDriverConfig &config) :
	SPI(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_keep_running(config.keep_running)
{
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
OSDatxxxx::add_centered_string_to_screen(const char *str, uint8_t pos_x, uint8_t pos_y, int width)
{
	// these elements hold variable-length text, so their X entry is the centre
	// column rather than the left edge. The start column is clamped while still
	// signed: add_string_to_screen() takes an unsigned column, and a negative one
	// would wrap far past the row and drop or misplace the text.
	const int len = math::min(static_cast<int>(strlen(str)), width);
	add_string_to_screen(str, math::max(0, pos_x - len / 2), pos_y, width);
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
	char batt_symbol = '?';
	const float remaining = PX4_ISFINITE(battery.remaining) ? battery.remaining : -1.f;
	const float voltage_v = battery.voltage_v;

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

	snprintf(buf, sizeof(buf), "%5d", (int)battery.discharged_mah);
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

	add_string_to_screen(buf, pos_x, pos_y, 7);
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
	const int horizon_x = _param_osd_ah_x.get();
	const int horizon_y = _param_osd_ah_y.get();
	const battery_status_s &battery = telemetry.battery;
	const float roll_rad = telemetry.attitude_valid ? telemetry.roll_rad : 0.f;
	const float pitch_rad = telemetry.attitude_valid ? telemetry.pitch_rad : 0.f;
	const float yaw_rad = telemetry.attitude_valid ? telemetry.yaw_rad : 0.f;
	memset(_screen, ' ', sizeof(_screen));

	if (enabled(osd::Symbol::ArtificialHorizon) && telemetry.attitude_valid) {
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
		// OSD_CH_POS_* are offsets from the centre of the screen, so the row they
		// land on follows the active video standard rather than a fixed default
		const int crosshair_x = OSD_CHARS_PER_ROW / 2 + _param_osd_ch_pos_hor.get();
		const int crosshair_y = _num_rows / 2 - _param_osd_ch_pos_ver.get();

		if (crosshair_x >= 0 && crosshair_x < OSD_CHARS_PER_ROW && crosshair_y >= 0 && crosshair_y < _num_rows) {
			add_character_to_screen(OSD_SYMBOL_AH_CENTER, crosshair_x, crosshair_y);
		}
	}

	if (enabled(osd::Symbol::BatteryVoltage)) {
		if (_param_osd_volt_mode.get() == 0) {
			if (telemetry.battery_valid) {
				add_battery_voltage(battery, _param_osd_volt_x.get(), _param_osd_volt_y.get());

			} else {
				snprintf(buf, sizeof(buf), "%c--.--V", OSD_SYMBOL_BATT_EMPTY);
				add_string_to_screen(buf, _param_osd_volt_x.get(), _param_osd_volt_y.get(), 7);
			}

		} else {
			if (telemetry.battery_valid && battery.cell_count > 0) {
				const float cell_voltage = battery.voltage_v / battery.cell_count;
				snprintf(buf, sizeof(buf), "%c%4.2fV", OSD_SYMBOL_BATT_EMPTY, (double)cell_voltage);

			} else {
				snprintf(buf, sizeof(buf), "%c--.--V", OSD_SYMBOL_BATT_EMPTY);
			}

			add_string_to_screen(buf, _param_osd_volt_x.get(), _param_osd_volt_y.get(), 7);
		}
	}

	if (enabled(osd::Symbol::MahDrawn)) {
		if (telemetry.battery_valid && PX4_ISFINITE(battery.discharged_mah) && battery.discharged_mah != -1.f) {
			add_consumed_mah(battery, _param_osd_mah_x.get(), _param_osd_mah_y.get());

		} else {
			snprintf(buf, sizeof(buf), "-----%c", OSD_SYMBOL_MAH);
			add_string_to_screen(buf, _param_osd_mah_x.get(), _param_osd_mah_y.get(), 6);
		}
	}

	if (enabled(osd::Symbol::PowerDraw)) {
		const bool current_valid = telemetry.battery_valid && PX4_ISFINITE(battery.current_a) && battery.current_a != -1.f;

		if (_param_osd_pwr_mode.get() == 0) {
			if (current_valid) {
				snprintf(buf, sizeof(buf), "%c%4.1fA", OSD_SYMBOL_AMP, (double)battery.current_a);

			} else {
				snprintf(buf, sizeof(buf), "%c--.-A", OSD_SYMBOL_AMP);
			}

		} else {
			if (current_valid) {
				snprintf(buf, sizeof(buf), "%4.0fW", (double)(battery.voltage_v * battery.current_a));

			} else {
				strncpy(buf, "-----W", sizeof(buf));
			}
		}

		add_string_to_screen(buf, _param_osd_pwr_x.get(), _param_osd_pwr_y.get(), 6);
	}

	if (enabled(osd::Symbol::SystemId)) {
		if (telemetry.status.timestamp == 0) {
			switch (_param_osd_id_mode.get()) {
			case 1:
				strncpy(buf, "S---", sizeof(buf));
				break;

			case 2:
				strncpy(buf, "---", sizeof(buf));
				break;

			default:
				strncpy(buf, "S--- ---", sizeof(buf));
				break;
			}

		} else {
			const char *mav_state = "INI";

			if (telemetry.actuator_armed.termination || telemetry.actuator_armed.kill ||
			    (telemetry.actuator_armed.lockdown && telemetry.status.hil_state == vehicle_status_s::HIL_STATE_OFF)) {
				mav_state = "TRM";

			} else if (telemetry.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				mav_state = telemetry.status.failsafe ? "CRT" : "ACT";

			} else if (telemetry.status.calibration_enabled || telemetry.status.rc_calibration_in_progress ||
				   telemetry.actuator_armed.in_esc_calibration_mode) {
				mav_state = "CAL";

			} else if (telemetry.status.pre_flight_checks_pass) {
				mav_state = "STB";
			}

			// vehicle_status carries the MAVLink system ID, so the driver does not depend on
			// MAV_SYS_ID: that parameter only exists on boards that build the mavlink module
			switch (_param_osd_id_mode.get()) {
			case 1:
				snprintf(buf, sizeof(buf), "S%03d", telemetry.status.system_id);
				break;

			case 2:
				snprintf(buf, sizeof(buf), "%s", mav_state);
				break;

			default:
				snprintf(buf, sizeof(buf), "S%03d %s", telemetry.status.system_id, mav_state);
				break;
			}
		}

		add_string_to_screen(buf, _param_osd_id_x.get(), _param_osd_id_y.get(), 8);
	}

	if (enabled(osd::Symbol::Rssi)) {
		const bool input_rc_seen = telemetry.input_rc.timestamp != 0;
		const bool input_rc_valid = input_rc_seen && hrt_elapsed_time(&telemetry.input_rc.timestamp) < 1_s;
		bool rssi_available = false;
		bool rssi_dbm_available = false;
		float rssi_dbm = 0.f;
		int rssi = 0;

		if (input_rc_valid) {
			if (telemetry.input_rc.rc_lost || telemetry.input_rc.rc_failsafe) {
				rssi = 0;
				rssi_available = true;

			} else if (PX4_ISFINITE(telemetry.input_rc.rssi_dbm)) {
				rssi_dbm = telemetry.input_rc.rssi_dbm;
				rssi_dbm_available = true;
				rssi_available = true;

			} else if (telemetry.input_rc.rssi >= 0 && telemetry.input_rc.rssi <= input_rc_s::RSSI_MAX) {
				rssi = telemetry.input_rc.rssi;
				rssi_available = true;
			}
		}

		const bool radio_status_valid = telemetry.radio_status.timestamp != 0 &&
						hrt_elapsed_time(&telemetry.radio_status.timestamp) < 2_s;

		// A previously active RC topic going stale is itself a link problem. Do not
		// mask that with an unrelated telemetry-radio RSSI value.
		if (!rssi_available && (!input_rc_seen || input_rc_valid) && radio_status_valid) {
			const int radio_rssi = telemetry.radio_status.remote_rssi < 255 ? telemetry.radio_status.remote_rssi :
					       telemetry.radio_status.rssi;

			if (radio_rssi < 255) {
				rssi = lroundf(radio_rssi * 100.f / 254.f);
				rssi_available = true;
			}
		}

		if (rssi_dbm_available) {
			snprintf(buf, sizeof(buf), "%c%4.0f", OSD_SYMBOL_RSSI, (double)rssi_dbm);

		} else if (rssi_available) {
			snprintf(buf, sizeof(buf), "%c%3d", OSD_SYMBOL_RSSI, rssi);

		} else {
			snprintf(buf, sizeof(buf), "%c---", OSD_SYMBOL_RSSI);
		}

		add_string_to_screen(buf, _param_osd_rssi_x.get(), _param_osd_rssi_y.get(), 5);
	}

	if (enabled(osd::Symbol::LinkQuality)) {
		const bool input_rc_valid = telemetry.input_rc.timestamp != 0 &&
					    hrt_elapsed_time(&telemetry.input_rc.timestamp) < 1_s;

		if (input_rc_valid && (telemetry.input_rc.rc_lost || telemetry.input_rc.rc_failsafe)) {
			strncpy(buf, "LQ  0", sizeof(buf));

		} else if (input_rc_valid && telemetry.input_rc.link_quality >= 0) {
			snprintf(buf, sizeof(buf), "LQ%3d", telemetry.input_rc.link_quality);

		} else {
			strncpy(buf, "LQ---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_lq_x.get(), _param_osd_lq_y.get(), 5);
	}

	const bool gps_valid = telemetry.gps.timestamp != 0 &&
			       hrt_elapsed_time(&telemetry.gps.timestamp) < 2_s;

	if (enabled(osd::Symbol::GpsSatellites)) {
		if (gps_valid) {
			snprintf(buf, sizeof(buf), "%c%c%2d", OSD_SYMBOL_SAT_L, OSD_SYMBOL_SAT_R, telemetry.gps.satellites_used);

		} else {
			snprintf(buf, sizeof(buf), "%c%c--", OSD_SYMBOL_SAT_L, OSD_SYMBOL_SAT_R);
		}

		add_string_to_screen(buf, _param_osd_gps_sat_x.get(), _param_osd_gps_sat_y.get(), 4);
	}

	if (enabled(osd::Symbol::GpsSpeed)) {
		if (gps_valid && telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && telemetry.gps.vel_ned_valid &&
		    PX4_ISFINITE(telemetry.gps.vel_m_s)) {
			snprintf(buf, sizeof(buf), "SPD%3.0f", (double)(telemetry.gps.vel_m_s * 3.6f));

		} else {
			strncpy(buf, "SPD---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_gps_spd_x.get(), _param_osd_gps_spd_y.get(), 6);
	}

	if (enabled(osd::Symbol::GpsLatitude)) {
		if (gps_valid && telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && PX4_ISFINITE(telemetry.gps.latitude_deg)) {
			snprintf(buf, sizeof(buf), "LAT%+.5f", telemetry.gps.latitude_deg);

		} else {
			strncpy(buf, "LAT---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_gps_lat_x.get(), _param_osd_gps_lat_y.get(), 13);
	}

	if (enabled(osd::Symbol::GpsLongitude)) {
		if (gps_valid && telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && PX4_ISFINITE(telemetry.gps.longitude_deg)) {
			snprintf(buf, sizeof(buf), "LON%+.5f", telemetry.gps.longitude_deg);

		} else {
			strncpy(buf, "LON---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_gps_lon_x.get(), _param_osd_gps_lon_y.get(), 14);
	}

	if (enabled(osd::Symbol::GpsInfo)) {
		if (gps_valid) {
			const char *fix_type = telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_3D ? "3D" :
					       (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D ? "2D" : "NO");

			if (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && PX4_ISFINITE(telemetry.gps.hdop) &&
			    PX4_ISFINITE(telemetry.gps.vdop) && PX4_ISFINITE(telemetry.gps.eph)) {
				const float pdop = sqrtf(telemetry.gps.hdop * telemetry.gps.hdop +
						 telemetry.gps.vdop * telemetry.gps.vdop);
				snprintf(buf, sizeof(buf), "%s D%3.1f E%3.1f", fix_type, (double)pdop, (double)telemetry.gps.eph);

			} else {
				snprintf(buf, sizeof(buf), "%s D--- E---", fix_type);
			}

		} else {
			strncpy(buf, "-- D--- E---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_gps_info_x.get(), _param_osd_gps_info_y.get(), 12);
	}

	const bool local_position_valid = telemetry.local_position.timestamp != 0 &&
					  hrt_elapsed_time(&telemetry.local_position.timestamp) < 1_s;

	if (enabled(osd::Symbol::Altitude)) {
		if (local_position_valid && telemetry.local_position.z_valid && PX4_ISFINITE(telemetry.local_position.z)) {
			add_altitude(telemetry.local_position, _param_osd_alt_x.get(), _param_osd_alt_y.get());

		} else {
			snprintf(buf, sizeof(buf), "%c----%c", OSD_SYMBOL_ARROW_NORTH, OSD_SYMBOL_M);
			add_string_to_screen(buf, _param_osd_alt_x.get(), _param_osd_alt_y.get(), 9);
		}
	}

	if (enabled(osd::Symbol::NumericalVario)) {
		if (local_position_valid && telemetry.local_position.v_z_valid && PX4_ISFINITE(telemetry.local_position.vz)) {
			snprintf(buf, sizeof(buf), "V%+4.1f", (double)-telemetry.local_position.vz);

		} else {
			strncpy(buf, "V---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_vario_x.get(), _param_osd_vario_y.get(), 7);
	}

	if (enabled(osd::Symbol::Attitude)) {
		if (telemetry.attitude_valid) {
			snprintf(buf, sizeof(buf), "P%+4.0f R%+4.0f", (double)math::degrees(pitch_rad), (double)math::degrees(roll_rad));

		} else {
			strncpy(buf, "P--- R---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_att_x.get(), _param_osd_att_y.get(), 11);
	}

	if (enabled(osd::Symbol::MissionState)) {
		const mission_s &mission = telemetry.mission;
		const mission_result_s &result = telemetry.mission_result;
		const bool stored = mission.timestamp != 0 && mission.count > 0;
		// The feasibility result applies to the mission, geofence and home position
		// it was generated against. A mismatch means the current mission is waiting
		// for a new feasibility check, not that it was rejected.
		const bool checked = result.timestamp != 0 &&
				     result.mission_id == mission.mission_id &&
				     result.geofence_id == mission.geofence_id &&
				     result.home_position_counter == telemetry.home.update_count;

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

		add_string_to_screen(buf, _param_osd_mission_x.get(), _param_osd_mission_y.get(), 8);
	}

	if (enabled(osd::Symbol::HomeDistance)) {
		if (telemetry.home_valid) {
			snprintf(buf, sizeof(buf), "%4.0f%c", (double)telemetry.home_distance_m, OSD_SYMBOL_M);

		} else {
			snprintf(buf, sizeof(buf), "----%c", OSD_SYMBOL_M);
		}

		add_string_to_screen(buf, _param_osd_home_dst_x.get(), _param_osd_home_dst_y.get(), 6);
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

		add_centered_string_to_screen(buf, _param_osd_mode_x.get(), _param_osd_mode_y.get(), 14);
	}

	if (enabled(osd::Symbol::Disarmed)) {
		const char *arming_state = "NO STATUS";

		if (telemetry.status.timestamp != 0) {
			arming_state = telemetry.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? "ARMED" : "DISARMED";
		}

		add_centered_string_to_screen(arming_state, _param_osd_arm_x.get(), _param_osd_arm_y.get(), 9);
	}

	if (enabled(osd::Symbol::Heading)) {
		if (telemetry.attitude_valid) {
			const int heading_deg = static_cast<int>(lroundf(math::degrees(yaw_rad))) % 360;
			snprintf(buf, sizeof(buf), "%c%03d", OSD_SYMBOL_ARROW_NORTH, heading_deg);

		} else {
			snprintf(buf, sizeof(buf), "%c---", OSD_SYMBOL_ARROW_NORTH);
		}

		add_string_to_screen(buf, _param_osd_head_x.get(), _param_osd_head_y.get(), 4);
	}

	if (enabled(osd::Symbol::Throttle)) {
		const bool manual_control_valid = telemetry.manual_control.valid && telemetry.manual_control.timestamp != 0 &&
						  hrt_elapsed_time(&telemetry.manual_control.timestamp) < 1_s &&
						  PX4_ISFINITE(telemetry.manual_control.throttle);

		if (manual_control_valid) {
			const float throttle = math::constrain((telemetry.manual_control.throttle + 1.f) * 50.f, 0.f, 100.f);
			snprintf(buf, sizeof(buf), "T%3.0f", (double)throttle);

		} else {
			strncpy(buf, "T---", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_throt_x.get(), _param_osd_throt_y.get(), 4);
	}

#if defined(CONFIG_DRIVERS_VTX)

	if (enabled(osd::Symbol::VtxStatus)) {
		const bool vtx_valid = telemetry.vtx.timestamp != 0;

		if (_param_osd_vtx_mode.get() == 0) {
			if (vtx_valid && telemetry.vtx.band >= 0 && telemetry.vtx.channel >= 0 &&
			    telemetry.vtx.power_level >= 0 && telemetry.vtx.band_letter != 0) {
				snprintf(buf, sizeof(buf), "VTX %c:%d:%d", telemetry.vtx.band_letter, telemetry.vtx.channel + 1,
					 telemetry.vtx.power_level + 1);

			} else {
				strncpy(buf, "VTX -:-:-", sizeof(buf));
			}

		} else if (vtx_valid && telemetry.vtx.power_label[0] != 0 && telemetry.vtx.frequency > 0) {
			snprintf(buf, sizeof(buf), "%huM %.*sMW", telemetry.vtx.frequency, (int)sizeof(telemetry.vtx.power_label),
				 reinterpret_cast<const char *>(telemetry.vtx.power_label));

		} else {
			strncpy(buf, "----M ---MW", sizeof(buf));
		}

		add_string_to_screen(buf, _param_osd_vtx_x.get(), _param_osd_vtx_y.get(), 12);
	}

#endif

	if (enabled(osd::Symbol::FlightTime)) {
		if (telemetry.status.timestamp == 0 ||
		    (telemetry.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED && telemetry.status.armed_time == 0)) {
			snprintf(buf, sizeof(buf), "%c--:--", OSD_SYMBOL_FLIGHT_TIME);
			add_string_to_screen(buf, _param_osd_ftime_x.get(), _param_osd_ftime_y.get(), 7);

		} else {
			add_flighttime(_telemetry.flight_time_s(), _param_osd_ftime_x.get(), _param_osd_ftime_y.get());
		}
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

			} else if (!telemetry.status.pre_flight_checks_pass
				   && telemetry.status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
				strncpy(message, "NOT READY", sizeof(message));
			}

			// falling through leaves the row clear: an all-clear string would sit there
			// permanently, and the arming element already reports armed state
		}

		add_centered_string_to_screen(message, _param_osd_status_x.get(), _param_osd_status_y.get(), FULL_MSG_LENGTH);
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

/****************************************************************************
 *
 *   Copyright (c) 2017-2024 PX4 Development Team. All rights reserved.
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

#include "AUAV_Differential.hpp"
#include <parameters/param.h>
#include <cctype>

AUAV_Differential::AUAV_Differential(const I2CSPIDriverConfig &config) :
	AUAV(config)
{
	/* Initialize cal_range from parameter as fallback value */
	int32_t hw_model = 0;
	param_get(param_find("SENS_EN_AUAVX"), &hw_model);

	switch (hw_model) {
	case 1: /* AUAV L05D (+- 5 inH20) */
		_cal_range = 10;
		break;

	case 2: /* AUAV L10D (+- 10 inH20) */
		_cal_range = 20;
		break;

	case 3: /* AUAV L30D (+- 30 inH20) */
		_cal_range = 60;
		break;

	default:
		_cal_range = 10; /* Default fallback */
		break;
	}
}

void AUAV_Differential::print_status()
{
	AUAV::print_status();
	PX4_INFO("cal range: %" PRId32, _cal_range);
}

void AUAV_Differential::publish_pressure(const float pressure_p, const float temperature_c,
		const hrt_abstime timestamp_sample)
{
	differential_pressure_s differential_pressure{};
	differential_pressure.timestamp = hrt_absolute_time();
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.device_id = get_device_id();
	differential_pressure.differential_pressure_pa = pressure_p;
	int32_t differential_press_rev = 0;
	param_get(param_find("SENS_DPRES_REV"), &differential_press_rev);

	//If differential pressure reverse param set, swap positive and negative
	if (differential_press_rev == 1) {
		differential_pressure.differential_pressure_pa = -1.0f * pressure_p;
	}

	differential_pressure.temperature = temperature_c;
	differential_pressure.error_count = perf_event_count(_comms_errors);
	_differential_pressure_pub.publish(differential_pressure);
}

int64_t AUAV_Differential::get_conversion_interval() const
{
	return DIFF_CONVERSION_INTERVAL;
}

AUAV::calib_eeprom_addr_t AUAV_Differential::get_calib_eeprom_addr() const
{
	return calib_eeprom_addr_t {
		EEPROM_DIFF_AHW,
		EEPROM_DIFF_ALW,
		EEPROM_DIFF_BHW,
		EEPROM_DIFF_BLW,
		EEPROM_DIFF_CHW,
		EEPROM_DIFF_CLW,
		EEPROM_DIFF_DHW,
		EEPROM_DIFF_DLW,
		EEPROM_DIFF_TC50,
		EEPROM_DIFF_ES
	};
}

float AUAV_Differential::process_pressure_dig(const float pressure_dig) const
{
	const float pressure_in_h = 1.25f * ((pressure_dig - (1 << 23)) / (1 << 24)) * static_cast<float>(_cal_range);
	return pressure_in_h * INH_TO_PA;
}

int AUAV_Differential::read_factory_data()
{
	/* The differential sensor needs the cal_range from the absolute sensor's EEPROM.
	 * Temporarily switch to the absolute sensor's I2C address to read it, then switch back. */

	uint8_t original_address = get_device_address();
	set_device_address(I2C_ADDRESS_ABSOLUTE);

	/* Read the calibration range from the absolute sensor's EEPROM */
	uint16_t factory_data = 0;
	int status = read_calibration_eeprom(EEPROM_ABS_CAL_RNG, factory_data);

	set_device_address(original_address);

	if (status != PX4_OK) {
		return status;
	}

	/* If EEPROM data is 0, stop trying (sensor does not have factory data) */
	if (factory_data == 0) {
		PX4_INFO("Differential: cal range data is 0, using fallback value");
		return PX4_OK;
	}

	/* Decode the two bytes as ASCII characters */
	uint8_t char_high = (factory_data >> 8) & 0xFF;
	uint8_t char_low = factory_data & 0xFF;

	/* Validate that both characters are ASCII digits (0-9) */
	if (!isdigit(char_high) || !isdigit(char_low)) {
		return PX4_ERROR;
	}

	int32_t sensor_type = ((char_high - '0') * 10) + (char_low - '0');

	/* Check if the detected sensor type is valid */
	if (sensor_type != AUAV_LD_05 && sensor_type != AUAV_LD_10 && sensor_type != AUAV_LD_30) {
		return PX4_ERROR;
	}

	_cal_range = sensor_type * 2;
	PX4_INFO("Differential: read cal range %" PRId32 " from absolute sensor EEPROM", _cal_range);

	return PX4_OK;
}

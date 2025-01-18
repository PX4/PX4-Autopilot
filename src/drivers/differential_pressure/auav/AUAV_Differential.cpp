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

AUAV_Differential::AUAV_Differential(const I2CSPIDriverConfig &config) :
	AUAV(config)
{
}

void AUAV_Differential::publish_pressure(const float pressure_p, const float temperature_c,
		const hrt_abstime timestamp_sample)
{
	differential_pressure_s differential_pressure{};
	differential_pressure.timestamp = hrt_absolute_time();
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.device_id = get_device_id();
	differential_pressure.differential_pressure_pa = pressure_p;
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
	const float pressure_in_h = 1.25f * ((pressure_dig - (1 << 23)) / (1 << 24)) * _cal_range;
	return pressure_in_h * INH_TO_PA;
}

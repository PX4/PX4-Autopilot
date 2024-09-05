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

#include "AUAV_Absolute.hpp"

AUAV_Absolute::AUAV_Absolute(const I2CSPIDriverConfig &config) :
	AUAV(config)
{
}

AUAV_Absolute::~AUAV_Absolute()
{
}

void AUAV_Absolute::publish_pressure(float pressure_p, float temperature_c, hrt_abstime timestamp_sample)
{
	sensor_baro_s sensor_baro{};
	sensor_baro.timestamp_sample = timestamp_sample;
	sensor_baro.device_id = get_device_id();
	sensor_baro.pressure = pressure_p;
	sensor_baro.temperature = temperature_c;
	sensor_baro.error_count = perf_event_count(_comms_errors);
	sensor_baro.timestamp = hrt_absolute_time();
	_sensor_baro_pub.publish(sensor_baro);
}

int64_t AUAV_Absolute::get_conversion_interval()
{
	return ABS_CONVERSION_INTERVAL;
}

AUAV::calib_eeprom_addr_t AUAV_Absolute::get_calib_eeprom_addr()
{
	return calib_eeprom_addr_t {
		EEPROM_ABS_AHW,
		EEPROM_ABS_ALW,
		EEPROM_ABS_BHW,
		EEPROM_ABS_BLW,
		EEPROM_ABS_CHW,
		EEPROM_ABS_CLW,
		EEPROM_ABS_DHW,
		EEPROM_ABS_DLW,
		EEPROM_ABS_TC50,
		EEPROM_ABS_ES
	};
}

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

#pragma once

#include "AUAV.hpp"
#include <uORB/topics/sensor_baro.h>

/* AUAV EEPROM addresses for absolute channel */
static constexpr uint8_t EEPROM_ABS_AHW 	= 0x2F;
static constexpr uint8_t EEPROM_ABS_ALW 	= 0x30;
static constexpr uint8_t EEPROM_ABS_BHW 	= 0x31;
static constexpr uint8_t EEPROM_ABS_BLW 	= 0x32;
static constexpr uint8_t EEPROM_ABS_CHW 	= 0x33;
static constexpr uint8_t EEPROM_ABS_CLW 	= 0x34;
static constexpr uint8_t EEPROM_ABS_DHW 	= 0x35;
static constexpr uint8_t EEPROM_ABS_DLW 	= 0x36;
static constexpr uint8_t EEPROM_ABS_TC50 	= 0x37;
static constexpr uint8_t EEPROM_ABS_ES		= 0x38;

/* Measurement rate is 50Hz */
static constexpr unsigned ABS_MEAS_RATE = 50;
static constexpr int64_t ABS_CONVERSION_INTERVAL = (1000000 / ABS_MEAS_RATE); /* microseconds */

/* Conversions */
static constexpr float MBAR_TO_PA = 100.0f;

class AUAV_Absolute : public AUAV
{
public:
	explicit AUAV_Absolute(const I2CSPIDriverConfig &config);
	~AUAV_Absolute() = default;

private:
	void publish_pressure(const float pressure_p, const float temperature_c, const hrt_abstime timestamp_sample) override;
	int64_t get_conversion_interval() const override;
	calib_eeprom_addr_t get_calib_eeprom_addr() const override;
	float process_pressure_dig(const float pressure_dig) const override;

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
};

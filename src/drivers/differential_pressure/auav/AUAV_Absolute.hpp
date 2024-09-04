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

/* Measurement rate is 100Hz */
static constexpr unsigned MEAS_RATE = 100;
static constexpr int64_t CONVERSION_INTERVAL = (1000000 / MEAS_RATE); /* microseconds */

class AUAV_Absolute : public AUAV
{
public:
	AUAV_Absolute(const I2CSPIDriverConfig &config);
	~AUAV_Absolute() override;

	void RunImpl() override;
	void print_status() override;

private:
	void handle_state_read_calibdata();
	void handle_state_request_measurement();
	void handle_state_gather_measurement();

	float correct_pressure(uint32_t pressure, uint32_t temperature);

	void publish_pressure(float pressure_p, float temperature_c, hrt_abstime timestamp_sample);

	struct calib_data_raw_t {
		uint16_t a_hw;
		uint16_t a_lw;
		uint16_t b_hw;
		uint16_t b_lw;
		uint16_t c_hw;
		uint16_t c_lw;
		uint16_t d_hw;
		uint16_t d_lw;
		uint16_t tc50;
		uint16_t es;
	};

	struct calib_data_t {
		float a;
		float b;
		float c;
		float d;
		float es;
		float tc50h;
		float tc50l;
	};

	calib_data_t _calib_data {};
	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
};

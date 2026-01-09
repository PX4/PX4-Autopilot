/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/sensor_temp.h>
#include <uORB/PublicationMulti.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

#define TMP102_TEMP_REG 0x00
#define TMP102_CONFIG_REG 0x01
#define TMP102_TLOW_REG 0x02
#define TMP102_THIGH_REG 0x03

constexpr uint16_t DEFAULT_CONFIG_REG = 0x60A0; // 12-bit resolution, comparator mode, active low, 4Hz

class TMP102 : public device::I2C, public I2CSPIDriver<TMP102>
{
public:
	TMP102(const I2CSPIDriverConfig &config);
	~TMP102() override;

	int init() override;
	int probe() override;
	void RunImpl();
	static void print_usage();

protected:
	void print_status();

private:
	uORB::PublicationMulti<sensor_temp_s> _sensor_temp_pub{ORB_ID(sensor_temp)};
	perf_counter_t		_cycle_perf;
	perf_counter_t		_comms_errors;

	uint16_t curr_PR = 0;

	float read_temperature();
	int read_reg(uint8_t address, uint16_t &data);
};

/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

class MCP9808 : public device::I2C, public I2CSPIDriver<MCP9808>
{
public:
	MCP9808(const I2CSPIDriverConfig &config);
	~MCP9808() override;

	int init() override;
	int probe() override;
	void RunImpl();
	static void print_usage();

protected:
	void print_status() override;
	void exit_and_cleanup() override;

private:
	enum class Register : uint8_t {
		CONFIG = 0x01,
		AMBIENT_TEMP = 0x05,
		MANUF_ID = 0x06,
		DEVICE_ID = 0x07,
		RESOLUTION = 0x08
	};

	uORB::PublicationMulti<sensor_temp_s> _to_sensor_temp{ORB_ID(sensor_temp)};
	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;


	sensor_temp_s _sensor_temp{};


	int read_reg(Register address, uint16_t &data);
	int write_reg(Register address, uint16_t value);
	float read_temperature();

	uint32_t measurement_time = 0;

};

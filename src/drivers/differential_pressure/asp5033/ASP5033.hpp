/****************************************************************************
 *
 * Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file ASP5033.hpp
 *
 * Driver for ASP5033 connected via I2C.
 *
 * Supported sensors:
 *
 *    - ASP5033
 *
 * Interface application notes:
 *
 *
 *@author Denislav Petrov <denislavamitoba@gmail.com>
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

#include <px4_platform_common/module.h>
#include <lib/systemlib/mavlink_log.h>




/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */


/* Configuration Constants */
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x6D; /* 0x6D 0xE4 */
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface


#define REG_CMD_ASP5033                0x30
#define REG_PRESS_DATA_ASP5033         0X06
#define REG_TEMP_DATA_ASP5033          0X09
#define CMD_MEASURE_ASP5033            0X0A
#define REG_WHOAMI_DEFAULT_ID_ASP5033  0X00
#define REG_WHOAMI_RECHECK_ID_ASP5033  0X66
#define REG_ID_ASP5033                 0x01
#define REG_ID_SET_ASP5033             0xa4

using namespace time_literals;


class ASP5033 : public device::I2C, public I2CSPIDriver<ASP5033>
{
public:
	ASP5033(const I2CSPIDriverConfig &config);
	~ASP5033() override;

	static void print_usage();
	void print_status() override;


	void RunImpl();

	int init() override;



	float press_sum;
	uint32_t press_count;


private:

	float _pressure = 0.f;
	float _temperature = 0.f;
	float _pressure_prev = 0.f;
	float _temperaute_prev = 0.f;

	int probe() override;

	int measure();
	int collect();
	int sensor_id_check();

	bool get_differential_pressure();
	hrt_abstime last_sample_time = hrt_absolute_time();
	orb_advert_t 	_mavlink_log_pub {nullptr}; //log send to


	uint32_t _measure_interval{CONVERSION_INTERVAL};
	uint32_t _conversion_interval{CONVERSION_INTERVAL};

	bool _sensor_ok{false};
	bool _collect_phase{false};

	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
	perf_counter_t _fault_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": fault detected")};
};


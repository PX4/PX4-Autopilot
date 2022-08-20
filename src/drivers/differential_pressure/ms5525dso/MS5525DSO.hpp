/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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

#include <math.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

/* The MS5525DSODSO address is 111011Cx, where C is the complementary value of the pin CSB */
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x76;

/* Measurement rate is 100Hz */
static constexpr unsigned MEAS_RATE = 100;
static constexpr int64_t CONVERSION_INTERVAL = (1000000 / MEAS_RATE); /* microseconds */

class MS5525DSO : public device::I2C, public I2CSPIDriver<MS5525DSO>
{
public:
	MS5525DSO(const I2CSPIDriverConfig &config);
	~MS5525DSO() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	int probe() override;

	bool init_ms5525dso();

	uint8_t prom_crc4(uint16_t n_prom[]) const;

	int measure();
	int collect();

	static constexpr uint8_t CMD_RESET = 0x1E; // ADC reset command
	static constexpr uint8_t CMD_ADC_READ = 0x00; // ADC read command

	static constexpr uint8_t CMD_PROM_START = 0xA0; // Prom read command (first)

	// D1 - pressure convert commands
	// Convert D1 (OSR=256)  0x40
	// Convert D1 (OSR=512)  0x42
	// Convert D1 (OSR=1024) 0x44
	// Convert D1 (OSR=2048) 0x46
	// Convert D1 (OSR=4096) 0x48
	static constexpr uint8_t CMD_CONVERT_PRES = 0x48;

	// D2 - temperature convert commands
	// Convert D2 (OSR=256)  0x50
	// Convert D2 (OSR=512)  0x52
	// Convert D2 (OSR=1024) 0x54
	// Convert D2 (OSR=2048) 0x56
	// Convert D2 (OSR=4096) 0x58
	static constexpr uint8_t CMD_CONVERT_TEMP = 0x58;

	uint8_t _current_cmd{CMD_CONVERT_PRES};

	unsigned _pressure_count{0};

	// Qx Coefficients Matrix by Pressure Range
	//  5525DSO-pp001DS (Pmin = -1, Pmax = 1)
	static constexpr uint8_t Q1 = 15;
	static constexpr uint8_t Q2 = 17;
	static constexpr uint8_t Q3 = 7;
	static constexpr uint8_t Q4 = 5;
	static constexpr uint8_t Q5 = 7;
	static constexpr uint8_t Q6 = 21;

	// calibration coefficients from prom
	uint16_t C1{0};
	uint16_t C2{0};
	uint16_t C3{0};
	uint16_t C4{0};
	uint16_t C5{0};
	uint16_t C6{0};

	int64_t Tref{0};

	// last readings for D1 (uncompensated pressure) and D2 (uncompensated temperature)
	uint32_t D1{0};
	uint32_t D2{0};

	bool _inited{false};

	uint32_t _measure_interval{CONVERSION_INTERVAL};
	uint32_t _conversion_interval{CONVERSION_INTERVAL};

	bool _sensor_ok{false};
	bool _collect_phase{false};

	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
};

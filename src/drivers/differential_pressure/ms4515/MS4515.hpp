/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 *
 * Driver for the MEAS Spec series MS4515 connected via I2C.
 *
 * Supported sensors:
 *
 *    - MS4515DO
 *
 * Interface application notes:
 *
 *    - Interfacing to MEAS Digital Pressure Modules (http://www.meas-spec.com/downloads/Interfacing_to_MEAS_Digital_Pressure_Modules.pdf)
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x46;

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */

class MS4515 : public device::I2C, public I2CSPIDriver<MS4515>
{
public:
	MS4515(const I2CSPIDriverConfig &config);
	~MS4515() override;

	static void print_usage();

	void RunImpl();

	int init() override;

private:
	int probe() override;

	int measure();
	int collect();

	uint32_t _measure_interval{CONVERSION_INTERVAL};
	uint32_t _conversion_interval{CONVERSION_INTERVAL};

	int16_t _dp_raw_prev{0};
	int16_t _dT_raw_prev{0};

	bool _sensor_ok{false};
	bool _collect_phase{false};

	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
};

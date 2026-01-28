/****************************************************************************
 *
 *   Copyright (c) 2017-2026 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/defines.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

using namespace time_literals;

/* Measurement rate is 100Hz */
static constexpr unsigned DIFF_MEAS_RATE = 100;
static constexpr int64_t DIFF_CONVERSION_INTERVAL = (1000000 / DIFF_MEAS_RATE); /* microseconds */
/* reading too fast can yield all zero data -> incorrect sensor reading */
static_assert(DIFF_CONVERSION_INTERVAL >= 7000, "Conversion interval is too fast");

static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x28;
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface

class DLVR : public device::I2C, public I2CSPIDriver<DLVR>
{
public:
	explicit DLVR(const I2CSPIDriverConfig &config);
	~DLVR();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, const int runtime_instance);
	static void print_usage();

	virtual void RunImpl();
	void print_status() override;
	int init() override;

private:
	void publish_pressure(const float pressure_p, const float temperature_c,
			      const hrt_abstime timestamp_sample);
	int64_t get_conversion_interval() const;
	void initiate_a_sample();
	void gather_measurement();

	float process_pressure_raw(const float pressure_dig) const;
	float process_temperature_raw(const float temperature_raw) const;

	// DLVR L10D-values as default
	float _cal_range{20.0f};
	float _offset_out{8192.f};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};

private:
	int probe() override;
};

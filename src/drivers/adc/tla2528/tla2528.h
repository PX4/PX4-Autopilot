/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/device/i2c.h>
#include <uORB/topics/adc_report.h>
#include <uORB/PublicationMulti.hpp>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

class TLA2528 : public device::I2C, public I2CSPIDriver<TLA2528>, public ModuleParams
{
public:
	TLA2528(const I2CSPIDriverConfig &config);
	~TLA2528() override;

	static void print_usage();
	int init() override;
	void RunImpl();
	int probe() override;

private:
	static const hrt_abstime SAMPLE_INTERVAL{10_ms};
	static constexpr int NUM_CHANNELS = 8;

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;

	uORB::PublicationMulti<adc_report_s> _adc_report_pub{ORB_ID(adc_report)};
	adc_report_s _adc_report{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ADC_TLA2528_REFV>) _adc_tla2528_refv
	)

	int init_reset();
	int poll_reset();
	int configure();
	int init_calibrate();
	int poll_calibrate();
	void adc_get();
	void exit_and_cleanup() override;

	enum class STATE : uint8_t {
		RESET,
		CONFIGURE,
		CALIBRATE,
		WORK
	};
	STATE _state{STATE::RESET};
};

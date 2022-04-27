/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file paa3905.hpp
 *
 * Driver for the Pixart paa3905 optical flow sensors connected via SPI.
 */

#pragma once

#include "PixArt_PAA3905_Registers.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/spi.h>
#include <conversion/rotation.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/optical_flow.h>

using namespace time_literals;
using namespace PixArt_PAA3905;

#define DIR_WRITE(a) ((a) | (1 << 7))
#define DIR_READ(a) ((a) & 0x7f)

class PAA3905 : public device::SPI, public I2CSPIDriver<PAA3905>
{
public:
	PAA3905(const I2CSPIDriverConfig &config);
	virtual ~PAA3905();

	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();

private:
	void exit_and_cleanup() override;

	int probe() override;

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	uint8_t	RegisterRead(uint8_t reg, int retries = 2);
	void RegisterWrite(uint8_t reg, uint8_t data);
	bool RegisterWriteVerified(uint8_t reg, uint8_t data, int retries = 1);

	void EnableLed();

	void StandardDetectionSetting();
	void EnhancedDetectionMode();
	void ModeAuto012();

	void CheckMode();

	void Configure();

	void ResetAccumulatedData();

	uORB::PublicationMulti<optical_flow_s> _optical_flow_pub{ORB_ID(optical_flow)};

	perf_counter_t	_sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t	_comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};
	perf_counter_t	_false_motion_perf{perf_alloc(PC_COUNT, MODULE_NAME": false motion report")};
	perf_counter_t	_register_write_fail_perf{perf_alloc(PC_COUNT, MODULE_NAME": verified register write failed")};

	static constexpr uint64_t COLLECT_TIME{15000}; // 15 milliseconds, optical flow data publish rate

	const spi_drdy_gpio_t _drdy_gpio;

	uint64_t _previous_collect_timestamp{0};
	uint64_t _flow_dt_sum_usec{0};
	uint8_t _flow_sample_counter{0};
	uint16_t _flow_quality_sum{0};

	matrix::Dcmf	_rotation;

	int             _discard_reading{3};

	int		_flow_sum_x{0};
	int		_flow_sum_y{0};

	Mode		_mode{Mode::LowLight};

	uint32_t _scheduled_interval_us{SAMPLE_INTERVAL_MODE_1};

	int _valid_count{0};

	bool _data_ready_interrupt_enabled{false};

	hrt_abstime _last_good_publish{0};
	hrt_abstime _last_reset{0};
};

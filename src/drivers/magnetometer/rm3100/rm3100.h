/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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
 * @file rm3100.h
 *
 * Shared defines for the RM3100 driver.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

/**
 * RM3100 internal constants and data structures.
 */

#define RM3100_INTERVAL			13000	// 13000 Microseconds, corresponds to ~75 Hz (TMRC 0x95)
#define UTESLA_TO_GAUSS			100.0f
#define RM3100_SENSITIVITY		75.0f

#define ADDR_POLL		0x00
#define ADDR_CMM		0x01
#define ADDR_CCX		0x04
#define ADDR_CCY		0x06
#define ADDR_CCZ		0x08
#define ADDR_TMRC		0x0B
#define ADDR_MX			0x24
#define ADDR_MY			0x27
#define ADDR_MZ			0x2A
#define ADDR_BIST		0x33
#define ADDR_STATUS		0x34
#define ADDR_HSHAKE		0x35
#define ADDR_REVID		0x36

#define CCX_DEFAULT_MSB		0x00
#define CCX_DEFAULT_LSB		0xC8
#define CCY_DEFAULT_MSB		CCX_DEFAULT_MSB
#define CCY_DEFAULT_LSB		CCX_DEFAULT_LSB
#define CCZ_DEFAULT_MSB		CCX_DEFAULT_MSB
#define CCZ_DEFAULT_LSB		CCX_DEFAULT_LSB
#define CMM_DEFAULT		0b0111'0001 // continuous mode
#define CONTINUOUS_MODE		(1 << 0)
#define POLLING_MODE		(0 << 0)
#define TMRC_DEFAULT		0x95 // ~13 ms, ~75 Hz
#define BIST_SELFTEST		0b1000'1111
#define BIST_DEFAULT		0x00
#define BIST_XYZ_OK		((1 << 4) | (1 << 5) | (1 << 6))
#define BIST_STE		(1 << 7)
#define BIST_DUR_USEC		(2*RM3100_INTERVAL)
#define HSHAKE_DEFAULT		(0x0B)
#define HSHAKE_NO_DRDY_CLEAR	(0x08)
#define STATUS_DRDY		(1 << 7)
#define POLL_XYZ		0x70

#define RM3100_REVID		0x22

/* interface factories */
extern device::Device *RM3100_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);
extern device::Device *RM3100_I2C_interface(int bus, int bus_frequency);

#define RM3100_ADDRESS		0x20

class RM3100 : public I2CSPIDriver<RM3100>
{
public:
	RM3100(device::Device *interface, const I2CSPIDriverConfig &config);
	virtual ~RM3100();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int init();

	void print_status() override;

	/**
	 * Configures the device with default register values.
	 */
	int set_default_register_values();

	void RunImpl();

private:
	/**
	 * Run sensor self-test
	 *
	 * @return 0 if self-test is ok, 1 else
	 */
	int self_test();

	/**
	* Converts int24_t stored in 32-bit container to int32_t
	*/
	void convert_signed(int32_t *n);

	PX4Magnetometer _px4_mag;

	device::Device *_interface{nullptr};

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _range_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": range error")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};

	int32_t _raw_data_prev[3] {};

	int _failure_count{0};

};

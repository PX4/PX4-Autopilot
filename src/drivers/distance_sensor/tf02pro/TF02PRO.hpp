/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file TF02PRO.hpp
 *
 * Driver for the SRF02 sonar range finder adapted from the Maxbotix sonar range finder driver (srf02).
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

/* Configuration Constants */
#define TF02PRO_BASEADDR			0x10 	// 7-bit address. 8-bit address is 0x20.

/* Device limits */
#define TF02PRO_MIN_DISTANCE 			(0.10f)
#define TF02PRO_MAX_DISTANCE 			(35.00f)

#define TF02PRO_CONVERSION_INTERVAL 		100000	// 100 ms for one sensor.

class TF02PRO : public device::I2C, public I2CSPIDriver<TF02PRO>
{
public:
	TF02PRO(const I2CSPIDriverConfig &config);
	~TF02PRO() override;

	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:

	void start();
	int collect();
	int measure();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 * @param address The I2C bus address to probe.
	 * @return True if the device is present.
	 */
	int probe_address(uint8_t address);

	PX4Rangefinder _px4_rangefinder;

	int _interval{TF02PRO_CONVERSION_INTERVAL};

	bool _collect_phase{false};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED,  MODULE_NAME": read")};
};

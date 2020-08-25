/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file tfmini_s.cpp
 * @author Luis Rodrigues
 *
 * Driver for the TFmini-s range finders connected via I2C.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>

using namespace time_literals;

/* Configuration Constants */
#define TFMINI_S_D_ADDR                 0x10 // 7-bit address.
#define TFMINI_S_F_ADDR                 0x11 // 7-bit address.
#define TFMINI_S_L_ADDR                 0x12 // 7-bit address.
#define TFMINI_S_R_ADDR                 0x13 // 7-bit address.
#define TFMINI_S_U_ADDR                 0x14 // 7-bit address.
#define TFMINI_S_B_ADDR                 0x15 // 7-bit address.

/* TERARANGER Registers addresses */
#define TFMINI_S_MAX_DISTANCE             (12.00f)
#define TFMINI_S_MIN_DISTANCE             (0.10f)

#define TFMINI_S_MEASUREMENT_INTERVAL         1_ms

class TFmini_s : public device::I2C, public px4::ScheduledWorkItem
{
public:
        TFmini_s(const int bus, const int address = TFMINI_S_D_ADDR,
		   const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
        ~TFmini_s() override;

	virtual int init() override;
	void print_info();

protected:

	virtual int probe() override;

private:

	void start();
	void stop();
	int collect();
	int measure();

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address The I2C bus address to probe.
	* @return True if the device is present.
	*/
	int probe_address(const uint8_t address);

	void Run() override;

	PX4Rangefinder _px4_rangefinder;

	bool _collect_phase{false};
        uint8_t TFMINI_S_ADDR{0x10};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comm_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};

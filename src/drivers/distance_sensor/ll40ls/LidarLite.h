/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file LidarLite.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 *
 * Generic interface driver for the PulsedLight Lidar-Lite range finders.
 */
#pragma once

#include <drivers/device/device.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>

using namespace time_literals;

// Device limits
static constexpr float LL40LS_MIN_DISTANCE{0.05f};
static constexpr float LL40LS_MAX_DISTANCE{25.00f};
static constexpr float LL40LS_MAX_DISTANCE_V2{35.00f};

// Normal conversion wait time.
static constexpr uint32_t LL40LS_CONVERSION_INTERVAL{50_ms};

// Maximum time to wait for a conversion to complete.
static constexpr uint32_t LL40LS_CONVERSION_TIMEOUT{100_ms};

class LidarLite
{
public:
	LidarLite(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~LidarLite();

	virtual int init() = 0;
	virtual void start() = 0;
	virtual void stop() = 0;

	/**
	 * @brief Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * @brief print registers to console.
	 */
	virtual void print_registers() {};

protected:

	uint32_t get_measure_interval() const { return _measure_interval; };

	virtual int collect() = 0;

	virtual int measure() = 0;

	virtual int reset_sensor() { return PX4_ERROR; };

	PX4Rangefinder	_px4_rangefinder;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "ll40ls: comms errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "ll40ls: read")};
	perf_counter_t _sensor_resets{perf_alloc(PC_COUNT, "ll40ls: resets")};
	perf_counter_t _sensor_zero_resets{perf_alloc(PC_COUNT, "ll40ls: zero resets")};

private:

	uint32_t  _measure_interval{LL40LS_CONVERSION_INTERVAL};
};

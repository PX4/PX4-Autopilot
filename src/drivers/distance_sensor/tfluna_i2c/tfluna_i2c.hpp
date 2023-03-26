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
 * @file tfluna_i2c.hpp
 *
 * @author Muhammad Rizky Millennianno <muhammad.rizky912@ui.ac.id>
 * @author Vishwakarma Aerial Dexterity Group Universitas Indonesia <rotary.auav@gmail.com>
 *
 * Driver for the Benewake TF-Luna lidar range finder.
 * Default I2C address 0x66 is used.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/device.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>

/* Configuration Constants */
#define TFLUNA_BASEADDR							0x10 // default I2C address
#define TFLUNA_ENABLEDISABLE				0x25 // 0-disable, 1-enable
#define TFLUNA_SIGNAL_LOW						0x02
#define TFLUNA_SIGNAL_HIGH					0x03
#define TFLUNA_SOFT_RESET						0x21
#define TFLUNA_HARD_RESET						0x29
#define TFLUNA_SAVE_SETTINGS				0x20
#define TFLUNA_DIST_LOW							0x00
#define TFLUNA_DIST_HIGH						0x01
#define TFLUNA_TEMP_LOW							0x04
#define TFLUNA_TEMP_HIGH						0x05
#define TFLUNA_FPS_LOW							0x26 
#define TFLUNA_FPS_HIGH							0x27 
#define TFLUNA_TRIG_MODE						0x23 // 0-continous, 1-trigger
#define TFLUNA_TRIGGER							0x24
#define TFLUNA_CONTINOUS						0x00

static constexpr uint8_t TFLUNA_BASE_SAMPLE_HZ[2] 	= {0x64, 0x00}; // 100 Hz sampling speed

static constexpr uint16_t UNDEREXPOSE_SENSOR_THRESHOLD 	= 100;
static constexpr uint16_t OVEREXPOSE_SENSOR_THRESHOLD 	= 0xFFFF;
static constexpr float OVERHEAT_SENSOR_THRESHOLD	= 70.f;
static constexpr int TFLUNA_BASE_SAMPLE_MS 					= int(1000000/TFLUNA_BASE_SAMPLE_HZ[0]); // sampling speed in ms

using namespace time_literals;

class TFLuna : public device::I2C, public I2CSPIDriver<TFLuna>
{
public:
	TFLuna(const I2CSPIDriverConfig &config);

	~TFLuna() override;

	static void print_usage();
	
	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status() override;

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

	int init() override;

private:
	int probe() override;

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	int reset_sensor();

	int read_register(const uint8_t& reg_address, uint8_t& value);
	int write_register(const uint8_t& reg_address, const uint8_t& value);

	int _consecutive_errors{0};

	PX4Rangefinder	_px4_rangefinder;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};

/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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
 * @file VL53L0X.hpp
 *
 * Driver for the ST VL53L0X ToF Sensor connected via I2C.
 */

#pragma once

#include <stdlib.h>
#include <string.h>

#include <containers/Array.hpp>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/distance_sensor.h>

/* Configuration Constants */
#define VL53LXX_BASEADDR                                0x29
#define VL53LXX_DEVICE_PATH                             "/dev/vl53lxx"

/* VL53LXX Registers addresses */
#define VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG            0x89
#define MSRC_CONFIG_CONTROL_REG                         0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG 0x44
#define SYSTEM_SEQUENCE_CONFIG_REG                      0x01
#define DYNAMIC_SPAD_REF_EN_START_OFFSET_REG            0x4F
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG         0x4E
#define GLOBAL_CONFIG_REF_EN_START_SELECT_REG           0xB6
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG            0xB0
#define SYSTEM_INTERRUPT_CONFIG_GPIO_REG                0x0A
#define SYSTEM_SEQUENCE_CONFIG_REG                      0x01
#define SYSRANGE_START_REG                              0x00
#define RESULT_INTERRUPT_STATUS_REG                     0x13
#define SYSTEM_INTERRUPT_CLEAR_REG                      0x0B
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG            0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH_REG                     0x84
#define SYSTEM_INTERRUPT_CLEAR_REG                      0x0B
#define RESULT_RANGE_STATUS_REG                         0x14
#define VL53LXX_RA_IDENTIFICATION_MODEL_ID              0xC0
#define VL53LXX_IDENTIFICATION_MODEL_ID                 0xEEAA

#define VL53LXX_US                                      1000    // 1ms
#define VL53LXX_SAMPLE_RATE                             50000   // 50ms

#define VL53LXX_MAX_RANGING_DISTANCE                    2.0f
#define VL53LXX_MIN_RANGING_DISTANCE                    0.0f

#define VL53LXX_BUS_CLOCK                               400000 // 400kHz bus clock speed

class VL53LXX : public device::I2C, public I2CSPIDriver<VL53LXX>
{
public:
	VL53LXX(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency,
		int address = VL53LXX_BASEADDR);

	virtual ~VL53LXX();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status() override;

	virtual ssize_t read(device::file_t *file_pointer, char *buffer, size_t buflen);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

private:
	virtual int probe() override;

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Sends an i2c measure command to the sensor.
	 */
	int measure();

	int readRegister(const uint8_t reg_address, uint8_t &value);
	int readRegisterMulti(const uint8_t reg_address, uint8_t *value, const uint8_t length);

	int writeRegister(const uint8_t reg_address, const uint8_t value);
	int writeRegisterMulti(const uint8_t reg_address, const uint8_t *value, const uint8_t length);

	int sensorInit();
	int sensorTuning();
	int singleRefCalibration(const uint8_t byte);
	int spadCalculations();

	bool _collect_phase{false};
	bool _measurement_started{false};
	bool _new_measurement{true};

	int _measure_interval{VL53LXX_SAMPLE_RATE};

	uint8_t _rotation{0};
	uint8_t _stop_variable{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "vl53lxx_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "vl53lxx_read")};
};

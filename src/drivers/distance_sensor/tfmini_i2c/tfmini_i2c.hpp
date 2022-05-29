/****************************************************************************
 *
 *   Copyright (c) 2017-2019, 2021 PX4 Development Team. All rights reserved.
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
 * @file tfmini_i2c.cpp
 *
 * Driver for the Benewake TFmini laser rangefinder series
 */

#pragma once

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>


class TFMINI_I2C : public device::I2C, public ModuleParams, public I2CSPIDriver<TFMINI_I2C>
{
public:
	TFMINI_I2C(const I2CSPIDriverConfig &config);
	virtual ~TFMINI_I2C();

	static void print_usage();

	int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status() override;

	/**
	 * Sets a new device address.
	 * @param address The new sensor address to be set: 1-254 even addresses only.
	 * @return Returns PX4_OK iff successful, PX4_ERROR otherwise.
	 */
	int set_address(const uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

protected:
	void custom_method(const BusCLIArguments &cli) override;
	int32_t getTFi2cParam(uint8_t index,uint8_t type);

private:

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Sends an i2c measure command to start the next sonar ping.
	 */
	int measure();
	int tfi2c_transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);
	int check_checksum(uint8_t *arr, int pkt_len);

	int set_param();

	enum tf_param_type {
		TF_PARAM_ADDR = 0,
		TF_PARAM_ROT,
		TF_PARAM_MAXD,
		TF_PARAM_MIND
	};

	static constexpr uint8_t RANGE_FINDER_MAX_SENSORS = 6;

	int _measure_interval{0};	// Initialize the measure interval for a single sensor.

	int _sensor_index{0};	// Initialize counter for cycling i2c adresses to zero.

	int _sensor_count{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_error{perf_alloc(PC_ELAPSED, "tfmini_i2c_comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, "tfmini_i2c_sample_perf")};

    	uint8_t crc_clc;
	struct sensor_param_s
	{
		int32_t max_range;
		int32_t min_range;
		uint8_t addresses;
		uint8_t rotations;
	};
	sensor_param_s sensor_param_value[RANGE_FINDER_MAX_SENSORS];
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_TFI2C>)   _p_sensor_enabled

	);
    float avoid_distance;
    int entra_avoid_area{0};
    uint8_t get_a_data;
};

/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file DistanceSensor.cpp
 * Distance Sensor driver base class.
 */

#pragma once

#include <errno.h>
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

#include <arch/board/board.h>
#include <board_config.h>
#include <containers/Array.hpp>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_cli.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_workqueue.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/uORB.h>


using namespace time_literals;

/* Configuration Constants */
#define DEFAULT_BAUD_RATE         B115200      // Baudrate 115200.
#define DEFAULT_MEASURE_INTERVAL  100_ms       // 100ms default sensor measurement interval.
#define DEFAULT_SERIAL_PORT       "/dev/ttyS3" // Default UART port.

class DistanceSensor : public px4::ScheduledWorkItem
{
public:
	/**  Default Constructor */
	DistanceSensor();

	/** Virtual destructor */
	virtual ~DistanceSensor();

	/**
	 * Iniitalizes general settings, subscribes to the distance sensor topic, calls
	 * dev_init() to performs device specific initializations, and starts the driver.
	 */
	virtual int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	virtual void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	virtual void stop();

protected:

	/**
	 * Performs device specific initialization actions.
	 */
	virtual int dev_init() = 0;

	/**
	 * Performs device specific stop actions.
	 */
	virtual void dev_stop() = 0;

	/**
	 * Gets reported sensor distance data.
	 */
	virtual float get_distance() = 0;

	/**
	 * Gets the sensor field of view.
	 */
	virtual float get_field_of_view();

	/**
	 * Gets the sensor specific product or device ID.
	 */
	virtual uint8_t get_sensor_id();

	/**
	 * Gets the sensor orientation.
	 */
	virtual uint8_t get_sensor_orientation();

	/**
	 * Gets the sensor variance.
	 */
	virtual uint8_t get_sensor_variance();

	/**
	 * Sets the sensor orientation.
	 */
	virtual void set_sensor_orientation(const uint8_t orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/**
	 * Generic method to be overloaded for device specific measure actions.
	 */
	virtual int measure();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = DEFAULT_BAUD_RATE);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	const char *_serial_port{nullptr};

	int _file_descriptor{-1};
	int _measure_interval{DEFAULT_MEASURE_INTERVAL};
	int _orb_class_instance{-1};

	uint8_t _orientation{0};
	uint8_t _variance{0};

	float _distance_m{0.0f};
	float _field_of_view{0.0f};
	float _max_distance{1000.0f};
	float _min_distance{0.0f};

	bool _collect_phase{false};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "distance_sensor_comms_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "distance_sensor_sample_perf")};

private:
	/**
	 * Calls dev_collect for sensor specific data collection, populates
	 * the distance_sensor_s struct and publishes the uORB topic.
	 */
	int collect();

	orb_advert_t _distance_sensor_topic{nullptr};
};

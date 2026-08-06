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

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

#define JRE30_MAX_DISTANCE      50.0f
#define JRE30_MIN_DISTANCE      0.10f
#define JRE30_MEASURE_INTERVAL  25_ms // 40Hz
#define READING_LEN             16

typedef union {
	struct {
		bool     gain : 1;
		bool     distance_disable : 1;
		uint16_t rfu : 14;
	} bits;
	uint16_t value;
} jre_status_union_t;

struct __attribute__((__packed__)) reading_msg {
	uint8_t  header_high;
	uint8_t  header_low;
	uint8_t  version;
	uint8_t  frame_count;
	uint8_t  distance_high;
	uint8_t  distance_low;
	uint8_t  rfu[4];
	uint8_t  radio_strength_high;
	uint8_t  radio_strength_low;
	uint8_t  status_high;
	uint8_t  status_low;
	uint16_t crc;  // little endian
};

class JRE30 : public px4::ScheduledWorkItem
{
public:
	JRE30(const char *serial_port, const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~JRE30() override;

	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

private:

	/**
	 * Reads the data measrurement from serial UART.
	 */
	int collect();

	/**
	 * Sends a data request message to the sensor.
	 */
	int measure();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B460800);

	void Run() override;

	uint16_t crc16_signature(const uint8_t *buf, uint32_t length, uint16_t crc, uint16_t final_xor);

	const char *_serial_port{nullptr};

	PX4Rangefinder _px4_rangefinder;

	int _file_descriptor;

	uint8_t            _buffer[sizeof(reading_msg)];
	uint8_t            _buffer_len;
	uint16_t           _crc16_calc;
	jre_status_union_t _status;
	uint16_t           _radio_strength;

	hrt_abstime _measurement_time;

	perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME": comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};
};

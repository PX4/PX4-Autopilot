/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file cm8jl65.cpp
 * @author Claudio Micheli <claudio@auterion.com>
 *
 * Driver for the Lanbao PSK-CM8JL65-CC5 distance sensor.
 * Make sure to disable MAVLINK messages (MAV_0_CONFIG PARAMETER)
 * on the serial port you connect the sensor,i.e TELEM2.
 *
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/mavlink_log.h>

using namespace time_literals;

/* Configuration Constants */
static constexpr uint32_t CM8JL65_MEASURE_INTERVAL{50_ms};	// 50ms default sensor conversion time.

/* Frame start delimiter */
static constexpr unsigned char START_FRAME_DIGIT1{0xA5};
static constexpr unsigned char START_FRAME_DIGIT2{0x5A};

/**
 * Frame format definition
 *   1B     1B      1B              1B            2B
 * | 0xA5 | 0x5A | distance-MSB | distance-LSB | crc-16 |
 *
 * Frame data saved for CRC calculation
 */
static constexpr uint8_t DISTANCE_MSB_POS{2};
static constexpr uint8_t DISTANCE_LSB_POS{3};
static constexpr uint8_t PARSER_BUF_LENGTH{4};

class CM8JL65 : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	CM8JL65(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~CM8JL65() override;

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:
	orb_advert_t _mavlink_log_pub = nullptr;

	enum class PARSE_STATE {
		WAITING_FRAME = 0,
		DIGIT_1,
		DIGIT_2,
		MSB_DATA,
		LSB_DATA,
		CHECKSUM
	};

	/**
	 * Calculates the 16 byte crc value for the data frame.
	 * @param data_frame The data frame to compute a checksum for.
	 * @param crc16_length The length of the data frame.
	 */
	uint16_t crc16_calc(const unsigned char *data_frame, uint8_t crc16_length);

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	int data_parser(const uint8_t check_byte, uint8_t parserbuf[PARSER_BUF_LENGTH], PARSE_STATE &state,
			uint16_t &crc16, int &distance);

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	void stop();


	PX4Rangefinder	_px4_rangefinder;

	char _port[20] {};

	unsigned char _frame_data[PARSER_BUF_LENGTH] {START_FRAME_DIGIT1, START_FRAME_DIGIT2, 0, 0};

	int _file_descriptor{-1};

	uint8_t _linebuf[25] {};

	uint16_t _crc16{0};

	PARSE_STATE _parse_state{PARSE_STATE::WAITING_FRAME};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};

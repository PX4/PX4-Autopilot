/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file sony_asdt1.hpp
 * @author Andrew Brahim <brahim@ascendengineer.com>
 * @author Apoorv Thapliyal
 *
 * Driver for the Sony ASDT1 lidar
 */

#pragma once


#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/obstacle_distance.h>

#include <matrix/matrix/math.hpp>
#include "matrix/Vector2.hpp"
#include <mathlib/mathlib.h>

#define MPDATASIZE 576

using namespace device;
using namespace time_literals;
using matrix::Vector3f;
using matrix::Vector;

class AS_DT1 : public px4::ScheduledWorkItem
{
public:
	AS_DT1(const char *device);
	virtual ~AS_DT1();

	int init();

	void print_info();

	int writeCommand(const uint8_t *data, size_t length);

private:
	enum class ParserState {
		FindBegin,
		ReadPayload,
		FindEnd,
	};

	int collect();
	int process_frame(const uint8_t *frame, size_t length);

	void Run() override;

	void start();
	void stop();

	obstacle_distance_s 			_obstacle_distance{};
	uORB::Publication<obstacle_distance_s>	_obstacle_distance_pub{ORB_ID(obstacle_distance)};	/**< obstacle_distance publication */

	static constexpr uint8_t 	BIN_COUNT = sizeof(obstacle_distance_s::distances) / sizeof(
				obstacle_distance_s::distances[0]);
	static constexpr size_t		READ_BUFFER_SIZE{1024};
	static constexpr uint8_t		MAX_READS_PER_COLLECT{17};
	static constexpr size_t		ASDT1_RAW_FRAME_SIZE{8640};
	static constexpr size_t		ASDT1_MAX_BACKLOG{ASDT1_RAW_FRAME_SIZE * 2};


	Serial _uart{}; ///< UART interface to ASDT1
	char _device[20] {}; ///< device / serial port path

	matrix::Vector<int, MPDATASIZE> xyDataTbl;
	static constexpr int kCONVERSIONINTERVAL{9_ms};

	unsigned int _linebuf_index{0};
	uint8_t _frame_buffer[ASDT1_RAW_FRAME_SIZE] {};
	size_t _frame_buffer_len{0};
	uint8_t _latest_frame[ASDT1_RAW_FRAME_SIZE] {};
	size_t _latest_frame_len{0};
	bool _have_latest_frame{false};
	ParserState _parser_state{ParserState::FindBegin};
	size_t _begin_match_index{0};
	size_t _end_match_index{0};

	hrt_abstime _last_read{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	unsigned int _baud = 921600; // ASDT1 default baud rate

	float range_min = 0.0f;    // in mm
	float range_max = 50000.0f; // in mm
};

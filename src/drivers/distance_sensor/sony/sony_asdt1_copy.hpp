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
 * @file sony_asdt1_copy.hpp
 *
 * Experimental AS-DT1 driver copy using binz Z-only output and index-based
 * obstacle_distance binning.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/obstacle_distance.h>

using namespace time_literals;

class AS_DT1_COPY : public px4::ScheduledWorkItem
{
public:
	AS_DT1_COPY(const char *device);
	virtual ~AS_DT1_COPY();

	int init();
	void print_info();

private:
	enum class ParserState {
		FindBegin,
		ReadPayload,
		FindEnd,
	};

	int start();
	void stop();
	void Run() override;

	int collect();
	bool parse_byte(uint8_t byte);
	int process_frame(const uint8_t *frame, size_t length);
	int write_command_padded(const char *command);

	static int32_t decode_20bit_signed(uint32_t raw);
	static uint32_t decode_20bit_raw(const uint8_t *data, size_t sample_index);
	static uint16_t z_raw_to_distance_cm(int32_t z_raw);
	static int sample_to_layout_index(size_t sample_index);
	static bool layout_index_to_row_col(int layout_index, int &row, int &col);
	static int col_to_obstacle_bin(int col);

	obstacle_distance_s _obstacle_distance{};
	uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{ORB_ID(obstacle_distance)};

	static constexpr uint8_t BIN_COUNT = sizeof(obstacle_distance_s::distances) / sizeof(
				obstacle_distance_s::distances[0]);

	static constexpr size_t READ_BUFFER_SIZE{512};
	static constexpr uint8_t MAX_READS_PER_COLLECT{8};
	static constexpr size_t ASDT1_MAX_SAMPLE_COUNT{576};
	static constexpr size_t ASDT1_SHORT_SAMPLE_COUNT{288};
	static constexpr size_t ASDT1_BINZ_FRAME_SIZE{1440};
	static constexpr size_t ASDT1_BINZ_SHORT_FRAME_SIZE{720};
	static constexpr size_t ASDT1_MAX_BACKLOG{ASDT1_BINZ_FRAME_SIZE * 3};

	static constexpr int ASDT1_ROWS{24};
	static constexpr int ASDT1_COLS{24};

	// First-pass vertical crop. This keeps the middle band of the AS-DT1 image
	// for 2D obstacle avoidance and ignores upper/lower points.
	static constexpr int MIN_USED_ROW{8};
	static constexpr int MAX_USED_ROW{15};

	static constexpr float HORIZONTAL_FOV_DEG{35.0f};
	static constexpr float LEFT_EDGE_DEG{-HORIZONTAL_FOV_DEG / 2.0f};
	static constexpr float OBSTACLE_INCREMENT_DEG{5.0f};

	Serial _uart{};
	char _device[20]{};

	uint8_t _frame_buffer[ASDT1_BINZ_FRAME_SIZE]{};
	size_t _frame_buffer_len{0};
	uint8_t _latest_frame[ASDT1_BINZ_FRAME_SIZE]{};
	size_t _latest_frame_len{0};
	bool _have_latest_frame{false};

	ParserState _parser_state{ParserState::FindBegin};
	size_t _begin_match_index{0};
	size_t _end_match_index{0};
	size_t _candidate_frame_len{0};

	hrt_abstime _last_read{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": copy com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": copy read")};

	unsigned int _baud{921600};
};

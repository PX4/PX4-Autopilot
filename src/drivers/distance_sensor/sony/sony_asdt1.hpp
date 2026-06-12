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
#include <uORB/Publication.hpp>
#include <uORB/topics/obstacle_distance.h>

#include <mathlib/mathlib.h>

using namespace device;
using namespace time_literals;

class AS_DT1 : public px4::ScheduledWorkItem
{
public:
	AS_DT1(const char *device);
	virtual ~AS_DT1();

	int init();

	void print_info();

	int writeCommand(const uint8_t *data, size_t length);

#ifdef UNIT_TEST
public:
#else
private:
#endif
	enum class ParserState {
		FindBegin,
		ReadPayload,
		FindEnd,
	};

	int collect();
	bool parse_byte(uint8_t byte);
	int process_frame(const uint8_t *frame, size_t length);
	int writeCommandPadded(const char *command);
	void readAndLogCommandResponse(const char *label, hrt_abstime timeout);
	void capture_debug_byte(uint8_t byte);
	void print_debug_buffer(const char *label, const uint8_t *buffer, size_t length);

	static int32_t decode_20bit_signed(uint32_t raw);
	static uint32_t decode_20bit_raw(const uint8_t *data, size_t sample_index);
	static uint16_t z_raw_to_distance_cm(uint32_t z_raw);
	static int sample_to_layout_index(size_t sample_index);
	static bool layout_index_to_row_col(int layout_index, int &row, int &col);
	static int col_to_obstacle_bin(int col);

	void Run() override;

	int start();
	void stop();

	obstacle_distance_s 			_obstacle_distance{};
	uORB::Publication<obstacle_distance_s>	_obstacle_distance_pub{ORB_ID(obstacle_distance)};	/**< obstacle_distance publication */

	static constexpr uint8_t 	BIN_COUNT = sizeof(obstacle_distance_s::distances) / sizeof(
				obstacle_distance_s::distances[0]);
	static constexpr size_t		READ_BUFFER_SIZE{512};
	static constexpr uint8_t		MAX_READS_PER_COLLECT{8};
	static constexpr size_t		ASDT1_MAX_SAMPLE_COUNT{576};
	static constexpr size_t		ASDT1_SHORT_SAMPLE_COUNT{288};
	static constexpr size_t		ASDT1_BINZ_FRAME_SIZE{1440};
	static constexpr size_t		ASDT1_BINZ_SHORT_FRAME_SIZE{720};
	static constexpr size_t		ASDT1_FRAME_BUFFER_SIZE{ASDT1_BINZ_FRAME_SIZE + 8};
	static constexpr size_t		ASDT1_MAX_BACKLOG{ASDT1_BINZ_FRAME_SIZE * 3};
	static constexpr size_t		DEBUG_RX_CAPTURE_SIZE{64};

	static constexpr int		ASDT1_COLS{24};
	static constexpr int		MIN_USED_ROW{8};
	static constexpr int		MAX_USED_ROW{15};
	static constexpr float		HORIZONTAL_FOV_DEG{35.0f};
	static constexpr float		LEFT_EDGE_DEG{-HORIZONTAL_FOV_DEG / 2.0f};
	static constexpr float		OBSTACLE_INCREMENT_DEG{5.0f};

	Serial _uart{}; ///< UART interface to ASDT1
	char _device[20] {}; ///< device / serial port path

	uint8_t _frame_buffer[ASDT1_FRAME_BUFFER_SIZE] {};
	size_t _frame_buffer_len{0};
	uint8_t _latest_frame[ASDT1_BINZ_FRAME_SIZE] {};
	size_t _latest_frame_len{0};
	bool _have_latest_frame{false};
	ParserState _parser_state{ParserState::FindBegin};
	size_t _begin_match_index{0};
	size_t _end_match_index{0};
	size_t _candidate_frame_len{0};
	size_t _payload_end_probe_match_index{0};

	hrt_abstime _last_read{0};
	uint64_t _bytes_rx_total{0};
	ssize_t _last_bytes_available{0};
	ssize_t _last_bytes_read{0};
	size_t _last_frame_processed_len{0};
	size_t _last_sample_count{0};
	uint8_t _last_valid_bins{0};
	uint16_t _last_closest_distance{UINT16_MAX};
	uint8_t _first_rx[DEBUG_RX_CAPTURE_SIZE] {};
	size_t _first_rx_len{0};
	uint8_t _last_rx[DEBUG_RX_CAPTURE_SIZE] {};
	size_t _last_rx_len{0};
	size_t _last_rx_pos{0};
	uint8_t _last_begin_probe[DEBUG_RX_CAPTURE_SIZE] {};
	size_t _last_begin_probe_len{0};
	size_t _last_begin_probe_remaining{0};
	uint64_t _rx_b_count{0};
	uint64_t _rx_e_count{0};
	uint64_t _rx_cr_count{0};
	uint64_t _rx_lf_count{0};
	uint64_t _rx_prompt_count{0};
	size_t _last_end_payload_len{0};
	size_t _last_end_probe_payload_len{0};
	size_t _last_rejected_candidate_len{0};
	uint64_t _payload_len_720_count{0};
	uint64_t _payload_len_1440_count{0};
	uint64_t _payload_len_unexpected_count{0};
	uint64_t _end_mismatch_count{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _bytes_rx{perf_alloc(PC_COUNT, MODULE_NAME": bytes_rx")};
	perf_counter_t _no_data{perf_alloc(PC_COUNT, MODULE_NAME": no_data")};
	perf_counter_t _frames_rx{perf_alloc(PC_COUNT, MODULE_NAME": frames_rx")};
	perf_counter_t _frames_pub{perf_alloc(PC_COUNT, MODULE_NAME": frames_pub")};
	perf_counter_t _parser_resets{perf_alloc(PC_COUNT, MODULE_NAME": parser_reset")};
	perf_counter_t _backlog_flushes{perf_alloc(PC_COUNT, MODULE_NAME": backlog_flush")};

	unsigned int _baud = 115200;

};

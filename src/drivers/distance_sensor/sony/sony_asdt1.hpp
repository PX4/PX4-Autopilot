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
 *
 * Minimal AS-DT1 serial write probe.
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/obstacle_distance.h>

#include <mathlib/mathlib.h>

#include <stdint.h>
#include <sys/types.h>

class AS_DT1 : public px4::ScheduledWorkItem
{
public:
	AS_DT1(const char *device, bool one_shot, bool flshow_only);
	~AS_DT1() override;

	int init();
	void print_info();

private:
	enum class ParserState {
		FindBegin,
		ReadPayload,
		ReadEnd,
	};

	enum class StartupState {
		SendFormat,
		WaitFormatPrompt,
		SendFsync,
		Streaming,
		WaitFirstFrame,
		SyncDrain,
		SendPromptSync,
		WaitPromptSync,
		DrainAfterSync,
		SendStop,
		WaitStopPrompt,
		SendMode,
		SendReboot,
		ModeSyncDrain,
		ModeSendPromptSync,
		ModeWaitPromptSync,
		ModeDrainAfterSync,
		ModeSendStop,
		ModeWaitStopPrompt,
		WaitModePrompt,
		BaudProbeDrain,
		BaudProbeSendPromptSync,
		BaudProbeWaitResponse,
		SendBaud,
		WaitBaudPrompt,
	};

	void start();
	void stop();
	void Run() override;

	int open_port();
	void close_port();
	int write_start_command();
	int write_command_padded(const char *command);
	int write_prompt_sync();
	ssize_t drain_input();
	ssize_t read_startup_response();
	ssize_t read_baud_probe_response();
	void record_read(const uint8_t *buffer, ssize_t bytes_read);
	void reset_parser();
	void reset_startup_prompt();
	void set_startup_deadline(hrt_abstime timeout_us);
	bool startup_deadline_elapsed() const;
	const char *startup_state_name() const;
	int read_and_print_response(const char *label);
	int read_once();
	bool parse_byte(uint8_t byte);
	int process_frame(const uint8_t *frame, size_t length);

	static uint32_t decode_20bit_raw(const uint8_t *data, size_t sample_index);
	static uint16_t z_raw_to_distance_cm(uint32_t z_raw);
	static int sample_to_layout_index(size_t sample_index);
	static int sample_to_layout_index(size_t sample_index, int32_t mode);
	static bool short_frame_mode(int32_t mode);
	static size_t sample_count_for_mode(int32_t mode);
	static size_t frame_size_for_mode(int32_t mode);
	static int min_used_row_for_mode(int32_t mode);
	static int max_used_row_for_mode(int32_t mode);
	static bool layout_index_to_row_col(int layout_index, int &row, int &col);
	static int col_to_obstacle_bin(int col);
	static uint16_t max_distance_for_mode(int32_t mode);
	static const char *mode_command_for_param(int32_t mode);

	static constexpr size_t COMMAND_BUFFER_SIZE{32};
	static constexpr size_t READ_BUFFER_SIZE{512};
	static constexpr size_t LAST_READ_CAPTURE_SIZE{64};
	static constexpr uint8_t READ_DRAIN_LIMIT{8};
	static constexpr int FORMAT_SETTLE_INTERVAL{200000};
	static constexpr int REBOOT_SETTLE_INTERVAL{3000000};
	static constexpr int PROMPT_SYNC_INTERVAL{100000};
	static constexpr int PROMPT_SYNC_SETTLE_INTERVAL{500000};
	static constexpr int COMMAND_RESPONSE_TIMEOUT{1000000};
	static constexpr int FIRST_FRAME_TIMEOUT{2000000};
	static constexpr unsigned int ASDT1_DESIRED_BAUD{921600};
	static constexpr unsigned int ASDT1_FALLBACK_BAUD{115200};
	static constexpr uint8_t PROMPT_SYNC_ATTEMPT_LIMIT{16};
	static constexpr uint8_t BAUD_PROBE_ATTEMPT_LIMIT{5};
	static constexpr uint8_t STARTUP_COMMAND_RETRY_LIMIT{3};
	static constexpr uint8_t STARTUP_DRAIN_READ_LIMIT{8};
	static constexpr uint8_t FLSHOW_RETRY_LIMIT{5};
	static constexpr uint8_t BIN_COUNT = sizeof(obstacle_distance_s::distances) / sizeof(
				obstacle_distance_s::distances[0]);
	static constexpr size_t ASDT1_MAX_SAMPLE_COUNT{576};
	static constexpr size_t ASDT1_SHORT_SAMPLE_COUNT{288};
	static constexpr size_t ASDT1_BINZ_FRAME_SIZE{1440};
	static constexpr size_t ASDT1_BINZ_SHORT_FRAME_SIZE{720};
	static constexpr size_t ASDT1_FRAME_BUFFER_SIZE{ASDT1_BINZ_FRAME_SIZE};
	static constexpr uint16_t MIN_VALID_DISTANCE_CM{30};
	static constexpr int ASDT1_COLS{24};
	static constexpr int MIN_USED_ROW{8};
	static constexpr int MAX_USED_ROW{15};
	static constexpr int MIN_USED_SHORT_ROW{4};
	static constexpr int MAX_USED_SHORT_ROW{7};
	static constexpr float HORIZONTAL_FOV_DEG{35.0f};
	static constexpr float LEFT_EDGE_DEG{-HORIZONTAL_FOV_DEG / 2.0f};
	static constexpr float OBSTACLE_INCREMENT_DEG{5.0f};

	obstacle_distance_s _obstacle_distance{};
	uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{ORB_ID(obstacle_distance)};

	int _fd{-1};
	int _interval{2000};
	char _device[20]{};
	bool _one_shot{false};
	bool _flshow_only{false};
	int32_t _mode{0};
	unsigned int _baud{ASDT1_DESIRED_BAUD};
	char _last_command[COMMAND_BUFFER_SIZE]{};
	size_t _last_command_len{0};
	ssize_t _last_write{-1};
	uint64_t _read_attempts{0};
	uint64_t _bytes_read_total{0};
	uint64_t _no_data_reads{0};
	uint64_t _read_errors{0};
	hrt_abstime _last_read{0};
	ssize_t _last_read_size{0};
	uint8_t _last_read_bytes[LAST_READ_CAPTURE_SIZE]{};
	size_t _last_read_bytes_len{0};
	uint8_t _frame_buffer[ASDT1_FRAME_BUFFER_SIZE]{};
	size_t _frame_buffer_len{0};
	ParserState _parser_state{ParserState::FindBegin};
	StartupState _startup_state{StartupState::SyncDrain};
	bool _startup_prompt_seen{false};
	size_t _startup_prompt_match_index{0};
	bool _startup_begin_seen{false};
	size_t _startup_begin_match_index{0};
	hrt_abstime _startup_deadline{0};
	uint8_t _startup_sync_attempts{0};
	uint8_t _startup_stop_attempts{0};
	uint8_t _startup_format_attempts{0};
	uint64_t _startup_fsync_attempts{0};
	uint64_t _startup_frame_wait_baseline{0};
	uint64_t _startup_mode_attempts{0};
	uint64_t _startup_baud_set_attempts{0};
	uint64_t _startup_baud_probe_attempts{0};
	uint64_t _startup_reboot_attempts{0};
	bool _startup_baud_probe_done{false};
	bool _startup_mode_preamble_done{false};
	uint64_t _startup_prompt_timeouts{0};
	uint64_t _startup_discarded_bytes{0};
	size_t _begin_match_index{0};
	size_t _end_match_index{0};
	uint64_t _frames_rx{0};
	uint64_t _frames_pub{0};
	uint64_t _parser_resets{0};
	uint64_t _end_marker_failures{0};
	uint64_t _below_min_distance_samples{0};
	size_t _last_frame_processed_len{0};
	size_t _last_sample_count{0};
	uint8_t _last_valid_bins{0};
	uint16_t _last_closest_distance{UINT16_MAX};
};

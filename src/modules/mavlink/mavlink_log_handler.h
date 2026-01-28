/****************************************************************************
 *
 *   Copyright (c) 2014-2024 PX4 Development Team. All rights reserved.
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

#include <perf/perf_counter.h>
#include "mavlink_bridge_header.h"

class Mavlink;

class MavlinkLogHandler
{
public:
	MavlinkLogHandler(Mavlink &mavlink);
	~MavlinkLogHandler();

	void send();
	void handle_message(const mavlink_message_t *msg);

private:
	struct LogEntry {
		uint16_t id{0xffff};
		uint32_t time_utc{};
		uint32_t size_bytes{};
		FILE *fp{nullptr};
		char filepath[60];
		uint32_t offset{};
	};

	struct LogEntryRequest {
		uint16_t id{0xffff};
		uint32_t start_offset{};
		uint32_t byte_count{};
	};

	struct LogListRequest {
		uint16_t first_id{0};
		uint16_t last_id{0};
		uint16_t current_id{0};
		int file_index{0};
	};

	enum class LogHandlerState {
		Idle,
		Listing,
		SendingData
	};

	// mavlink message handlers
	void handle_log_request_list(const mavlink_message_t *msg);
	void handle_log_request_data(const mavlink_message_t *msg);
	void handle_log_request_end(const mavlink_message_t *msg);
	void handle_log_erase(const mavlink_message_t *msg);

	// state functions
	void state_idle();
	void state_listing();
	void state_sending_data();

	// Log request list
	bool create_log_list_file();
	void write_entries_to_file(FILE *f, const char *dir);
	void send_log_entry(uint32_t size, uint32_t time_utc);

	// Log request data
	bool log_entry_from_id(uint16_t log_id, LogEntry *entry);

	// Log erase
	void delete_all_logs(const char *dir);


private:
	LogHandlerState _state{LogHandlerState::Idle};
	Mavlink &_mavlink;

	// Log list
	LogListRequest _list_request{};
	int _num_logs{0};
	bool _logs_listed{false};

	// Log data
	LogEntry 			_current_entry{};
	LogEntryRequest 	_entry_request{};
	bool 				_file_send_finished{};

	perf_counter_t _create_file_elapsed{perf_alloc(PC_ELAPSED, MODULE_NAME": create file")};
	perf_counter_t _listing_elapsed{perf_alloc(PC_ELAPSED, MODULE_NAME": listing")};
};

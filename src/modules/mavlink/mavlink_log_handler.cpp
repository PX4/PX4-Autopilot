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

#include "mavlink_log_handler.h"
#include "mavlink_main.h"
#include <dirent.h>
#include <sys/stat.h>

static constexpr int MAX_BYTES_BURST = 256 * 1024;
static const char *kLogListFilePath = PX4_STORAGEDIR "/logdata.txt";
static const char *kLogListFilePathTemp = PX4_STORAGEDIR "/$log$.txt";
static const char *kLogDir = PX4_STORAGEDIR "/log";

#ifdef __PX4_NUTTX
#define PX4LOG_REGULAR_FILE DTYPE_FILE
#define PX4LOG_DIRECTORY    DTYPE_DIRECTORY
#define PX4_MAX_FILEPATH 	CONFIG_PATH_MAX
#else
#ifndef PATH_MAX
#define PATH_MAX 1024 // maximum on macOS
#endif
#define PX4LOG_REGULAR_FILE DT_REG
#define PX4LOG_DIRECTORY    DT_DIR
#define PX4_MAX_FILEPATH 	PATH_MAX
#endif

MavlinkLogHandler::MavlinkLogHandler(Mavlink &mavlink)
	: _mavlink(mavlink)
{}

MavlinkLogHandler::~MavlinkLogHandler()
{
	perf_free(_create_file_elapsed);
	perf_free(_listing_elapsed);

	if (_current_entry.fp) {
		fclose(_current_entry.fp);
	}

	unlink(kLogListFilePath);
	unlink(kLogListFilePathTemp);
}

void MavlinkLogHandler::send()
{
	switch (_state) {
	case LogHandlerState::Idle: {
			state_idle();
			break;
		}

	case LogHandlerState::Listing: {
			state_listing();
			break;
		}

	case LogHandlerState::SendingData: {
			state_sending_data();
			break;
		}
	}
}

void MavlinkLogHandler::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		handle_log_request_list(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		handle_log_request_data(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_END:
		handle_log_request_end(msg);
		break;

	case MAVLINK_MSG_ID_LOG_ERASE:
		handle_log_erase(msg);
		break;
	}
}

void MavlinkLogHandler::state_idle()
{
	if (_current_entry.fp && _file_send_finished) {
		fclose(_current_entry.fp);
		_current_entry.fp = nullptr;

		_current_entry.id = 0xffff;
		_current_entry.time_utc = 0;
		_current_entry.size_bytes = 0;
		_current_entry.filepath[0] = 0;
		_current_entry.offset = 0;

		_entry_request.id = 0xffff;
		_entry_request.start_offset = 0;
		_entry_request.byte_count = 0;
	}
}

void MavlinkLogHandler::state_listing()
{
	static constexpr uint32_t MAVLINK_PACKET_SIZE = MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_LOG_ENTRY_LEN;

	if (_mavlink.get_free_tx_buf() <= MAVLINK_PACKET_SIZE) {
		return;
	}

	DIR *dp = opendir(kLogDir);

	if (!dp) {
		PX4_DEBUG("No logs available");
		return;
	}

	FILE *fp = fopen(kLogListFilePath, "r");

	if (!fp) {
		PX4_DEBUG("Failed to open log list file");
		closedir(dp);
		return;
	}

	fseek(fp, _list_request.file_index, SEEK_SET);

	size_t bytes_sent = 0;

	char line[PX4_MAX_FILEPATH];

	perf_begin(_listing_elapsed);

	while (fgets(line, sizeof(line), fp)) {

		if (_list_request.current_id < _list_request.first_id) {
			_list_request.current_id++;
			continue;
		}

		// We can send!
		uint32_t size_bytes = 0;
		uint32_t time_utc = 0;
		char filepath[PX4_MAX_FILEPATH];

		// If parsed lined successfully, send the entry
		if (sscanf(line, "%" PRIu32 " %" PRIu32 " %s", &time_utc, &size_bytes, filepath) != 3) {
			PX4_DEBUG("sscanf failed");
			continue;
		}

		send_log_entry(time_utc, size_bytes);
		bytes_sent += sizeof(mavlink_log_entry_t);
		_list_request.current_id++;

		// Yield if we've exceed mavlink burst or buffer limit
		if (_mavlink.get_free_tx_buf() <= MAVLINK_PACKET_SIZE || bytes_sent >= MAX_BYTES_BURST) {
			_list_request.file_index = ftell(fp);
			fclose(fp);
			closedir(dp);
			perf_end(_listing_elapsed);
			return;
		}
	}

	perf_end(_listing_elapsed);

	fclose(fp);
	closedir(dp);

	_list_request.current_id = 0;
	_list_request.file_index = 0;
	_state = LogHandlerState::Idle;
}

void MavlinkLogHandler::state_sending_data()
{
	static constexpr uint32_t MAVLINK_PACKET_SIZE = MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_LOG_DATA_LEN;
	size_t bytes_sent = 0;

	while (_mavlink.get_free_tx_buf() > MAVLINK_PACKET_SIZE && bytes_sent < MAX_BYTES_BURST) {

		// Only seek if we need to
		long int offset = _current_entry.offset - ftell(_current_entry.fp);

		if (offset && fseek(_current_entry.fp, offset, SEEK_CUR)) {
			fclose(_current_entry.fp);
			_current_entry.fp = nullptr;
			PX4_DEBUG("seek error");
			_state = LogHandlerState::Idle;
		}

		// Prepare mavlink message
		mavlink_log_data_t msg;

		if (_current_entry.offset >= _current_entry.size_bytes) {
			PX4_DEBUG("Entry offset equal to file size");
			_state = LogHandlerState::Idle;
			return;
		}

		size_t bytes_to_read = _current_entry.size_bytes - _current_entry.offset;

		if (bytes_to_read > sizeof(msg.data)) {
			bytes_to_read = sizeof(msg.data);
		}

		msg.count = fread(msg.data, 1, bytes_to_read, _current_entry.fp);
		msg.id = _current_entry.id;
		msg.ofs = _current_entry.offset;

		mavlink_msg_log_data_send_struct(_mavlink.get_channel(), &msg);

		bytes_sent += MAVLINK_PACKET_SIZE;
		_current_entry.offset += msg.count;

		bool chunk_finished = _current_entry.offset >= (_entry_request.byte_count + _entry_request.start_offset);
		_file_send_finished = _current_entry.offset >= _current_entry.size_bytes;

		if (chunk_finished || _file_send_finished) {
			_state = LogHandlerState::Idle;
			return;
		}
	}
}

void MavlinkLogHandler::handle_log_request_list(const mavlink_message_t *msg)
{
	mavlink_log_request_list_t request;
	mavlink_msg_log_request_list_decode(msg, &request);

	if (!create_log_list_file()) {
		return;
	}

	_list_request.first_id = request.start;
	_list_request.last_id = request.end == 0xffff ? _num_logs : request.end;
	_list_request.current_id = 0;
	_list_request.file_index = 0;
	_logs_listed = true;
	_state = LogHandlerState::Listing;
}

void MavlinkLogHandler::handle_log_request_data(const mavlink_message_t *msg)
{
	if (!_logs_listed) {
		PX4_DEBUG("Logs not yet listed");
		_state = LogHandlerState::Idle;
		return;
	}

	mavlink_log_request_data_t request;
	mavlink_msg_log_request_data_decode(msg, &request);

	if (request.id >= _num_logs) {
		PX4_DEBUG("Requested log %" PRIu16 " but we only have %u", request.id, _num_logs);
		_state = LogHandlerState::Idle;
		return;
	}

	// Handle switching to new request ID
	if (request.id != _current_entry.id) {
		// Close the old file
		if (_current_entry.fp) {
			fclose(_current_entry.fp);
			_current_entry.fp = nullptr;
		}

		LogEntry entry = {};

		if (!log_entry_from_id(request.id, &entry)) {
			PX4_DEBUG("Log file ID %u does not exist", request.id);
			_state = LogHandlerState::Idle;
			return;
		}

		entry.fp = fopen(entry.filepath, "rb");
		entry.offset = request.ofs;

		if (!entry.fp) {
			PX4_DEBUG("Failed to open file %s", entry.filepath);
			return;
		}

		_current_entry = entry;
	}

	// Stop if offset request is larger than file
	if (request.ofs >= _current_entry.size_bytes) {
		PX4_DEBUG("Request offset %" PRIu32 "greater than file size %" PRIu32, request.ofs,
			  _current_entry.size_bytes);
		_state = LogHandlerState::Idle;
		return;
	}

	_entry_request.id = request.id;
	_entry_request.start_offset = request.ofs;
	_entry_request.byte_count = request.count;
	// Set the offset of the current entry to the requested offset
	_current_entry.offset = _entry_request.start_offset;
	_file_send_finished = false;
	_state = LogHandlerState::SendingData;
}

void MavlinkLogHandler::handle_log_request_end(const mavlink_message_t *msg)
{
	_state = LogHandlerState::Idle;
}

void MavlinkLogHandler::handle_log_erase(const mavlink_message_t *msg)
{
	if (_current_entry.fp) {
		fclose(_current_entry.fp);
		_current_entry.fp = nullptr;
	}

	_state = LogHandlerState::Idle;
	unlink(kLogListFilePath);
	unlink(kLogListFilePathTemp);

	delete_all_logs(kLogDir);
}

bool MavlinkLogHandler::create_log_list_file()
{
	perf_begin(_create_file_elapsed);

	// clean up old file
	unlink(kLogListFilePath);
	_num_logs = 0;

	DIR *dp = opendir(kLogDir);

	if (!dp) {
		PX4_DEBUG("No logs available");
		return false;
	}

	FILE *temp_fp = fopen(kLogListFilePathTemp, "w");

	if (!temp_fp) {
		PX4_DEBUG("Failed to create temp file");
		closedir(dp);
		return false;
	}

	struct dirent *result = nullptr;

	// Iterate over the log/ directory which contains subdirectories formatted: yyyy-mm-dd
	while (1) {
		result = readdir(dp);

		if (!result) {
			// Reached end of directory
			break;
		}

		if (result->d_type != PX4LOG_DIRECTORY) {
			// Skip stray files
			continue;
		}

		// Skip the '.' and '..' entries
		if (strcmp(result->d_name, ".") == 0 || strcmp(result->d_name, "..") == 0) {
			continue;
		}

		// Open up the sub directory
		char dirpath[PX4_MAX_FILEPATH];
		int ret = snprintf(dirpath, sizeof(dirpath), "%s/%s", kLogDir, result->d_name);

		bool path_is_ok = (ret > 0) && (ret < (int)sizeof(dirpath));

		if (!path_is_ok) {
			PX4_DEBUG("Log subdir path error: %s", dirpath);
			continue;
		}

		// Iterate over files inside the subdir and write them to the file
		write_entries_to_file(temp_fp, dirpath);
	}

	fclose(temp_fp);
	closedir(dp);

	// Rename temp file to data file
	if (rename(kLogListFilePathTemp, kLogListFilePath)) {
		PX4_DEBUG("Failed to rename temp file");
		return false;
	}

	perf_end(_create_file_elapsed);

	return true;
}

void MavlinkLogHandler::write_entries_to_file(FILE *fp, const char *dir)
{
	DIR *dp = opendir(dir);
	struct dirent *result = nullptr;

	while (1) {
		result = readdir(dp);

		if (!result) {
			// Reached end of directory
			break;
		}

		if (result->d_type != PX4LOG_REGULAR_FILE) {
			// Skip non files
			continue;
		}

		char filepath[PX4_MAX_FILEPATH];
		int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir, result->d_name);
		bool path_is_ok = (ret > 0) && (ret < (int)sizeof(filepath));

		if (!path_is_ok) {
			PX4_DEBUG("Log subdir path error: %s", filepath);
			continue;
		}

		struct stat filestat;

		if (stat(filepath, &filestat) != 0) {
			PX4_DEBUG("stat() failed: %s", filepath);
			continue;
		}

		// Write to file using format:
		// [ time ] [ size_bytes ] [ filepath ]
		fprintf(fp, "%u %u %s\n", unsigned(filestat.st_mtime), unsigned(filestat.st_size), filepath);
		_num_logs++;
	}

	closedir(dp);
}

void MavlinkLogHandler::send_log_entry(uint32_t time_utc, uint32_t size_bytes)
{
	mavlink_log_entry_t msg;
	msg.time_utc     = time_utc;
	msg.size         = size_bytes;
	msg.id           = _list_request.current_id;
	msg.num_logs     = _num_logs;
	msg.last_log_num = _list_request.last_id;
	mavlink_msg_log_entry_send_struct(_mavlink.get_channel(), &msg);
}

bool MavlinkLogHandler::log_entry_from_id(uint16_t log_id, LogEntry *entry)
{
	DIR *dp = opendir(kLogDir);

	if (!dp) {
		PX4_INFO("No logs available");
		return false;
	}

	FILE *fp = fopen(kLogListFilePath, "r");

	if (!fp) {
		PX4_DEBUG("Failed to open %s", kLogListFilePath);
		closedir(dp);
		return false;
	}

	bool found_entry = false;
	uint16_t current_id = 0;
	char line[PX4_MAX_FILEPATH];

	while (fgets(line, sizeof(line), fp)) {

		if (current_id != log_id) {
			current_id++;
			continue;
		}

		if (sscanf(line, "%" PRIu32 " %" PRIu32 " %s", &(entry->time_utc), &(entry->size_bytes), entry->filepath) != 3) {
			PX4_DEBUG("sscanf failed");
			continue;
		}

		entry->id = log_id;
		found_entry = true;
		break;
	}

	fclose(fp);
	closedir(dp);

	return found_entry;
}

void MavlinkLogHandler::delete_all_logs(const char *dir)
{
	//-- Open log directory
	DIR *dp = opendir(dir);

	if (dp == nullptr) {
		return;
	}

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		// no more entries?
		if (result == nullptr) {
			break;
		}

		if (result->d_type == PX4LOG_DIRECTORY && result->d_name[0] != '.') {
			char filepath[PX4_MAX_FILEPATH];
			int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(filepath));

			if (path_is_ok) {
				delete_all_logs(filepath);

				if (rmdir(filepath)) {
					PX4_DEBUG("Error removing %s", filepath);
				}
			}
		}

		if (result->d_type == PX4LOG_REGULAR_FILE) {
			char filepath[PX4_MAX_FILEPATH];
			int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(filepath));

			if (path_is_ok) {
				if (unlink(filepath)) {
					PX4_DEBUG("Error unlinking %s", filepath);
				}
			}
		}
	}

	closedir(dp);
}

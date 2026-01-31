/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include "HardfaultStream.hpp"

using namespace time_literals;

namespace hardfault_stream
{

HardfaultStream::HardfaultStream() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

HardfaultStream::~HardfaultStream()
{
	ScheduleClear();

	if (_hardfault_file != nullptr) {
		fclose(_hardfault_file);
	}
}

int HardfaultStream::task_spawn(int argc, char *argv[])
{
	HardfaultStream *obj = new HardfaultStream();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void HardfaultStream::start()
{
	ScheduleOnInterval(150_ms);
}

bool HardfaultStream::mavlink_gcs_up()
{
	for (auto &telemetry_status : _telemetry_status_subs) {
		telemetry_status_s telemetry;

		if (telemetry_status.update(&telemetry)) {
			if (telemetry.heartbeat_type_gcs) {
				return true;
			}
		}
	}

	return false;
}

void HardfaultStream::search_hardfault_file()
{
	DIR *dp = opendir(PX4_STORAGEDIR);

	if (dp != nullptr) {

		struct dirent *result;
		struct stat st;
		time_t latest_mtime = 0;

		while ((result = readdir(dp))) {
			// Check for pattern fault_*.log
			if (strncmp("fault_", result->d_name, 6) == 0 && strcmp(result->d_name + strlen(result->d_name) - 4, ".log") == 0) {
				char current_file_path[CONFIG_PATH_MAX + 1];
				snprintf(current_file_path, sizeof(current_file_path), "%s/%s", PX4_STORAGEDIR, result->d_name);

				if (stat(current_file_path, &st) == 0) {
					if (st.st_mtime >= latest_mtime) {
						latest_mtime = st.st_mtime;
						strncpy(_hardfault_file_path, current_file_path, sizeof(_hardfault_file_path));
						_hardfault_file_path[sizeof(_hardfault_file_path) - 1] = '\0';
						_hardfault_file_present = true;
					}
				}
			}
		}
	}

	closedir(dp);
}

void HardfaultStream::stream_hardfault()
{
	if (_hardfault_file == nullptr) {
		_hardfault_file = fopen(_hardfault_file_path, "rb");

		if (_hardfault_file == nullptr) {
			PX4_ERR("Can't open hardfault log: %s", _hardfault_file_path);
			_state = State::RequestStop;
			return;
		}

		PX4_INFO("Streaming hardfault log: %s", _hardfault_file_path);
	}

	static constexpr int chunk_size = sizeof(mavlink_log_s::text) - 1;
	uint8_t chunk[chunk_size];
	size_t bytes_read = fread(chunk, 1, chunk_size, _hardfault_file);

	if (bytes_read > 0) {
		mavlink_vasprintf(_MSG_PRIO_CRITICAL, &_mavlink_log_pub, "%.*s", bytes_read, chunk);

	} else {
		fclose(_hardfault_file);
		_hardfault_file = nullptr;
		_stream_finished = true;
	}
}

void HardfaultStream::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	switch (_state) {
	case State::SearchFile:
		search_hardfault_file();
		_state = State::WaitMavlink;
		ScheduleNow();
		break;

	case State::WaitMavlink:
		if (mavlink_gcs_up()) {
			_state = State::StreamFile;
			ScheduleNow();
		}

		break;

	case State::StreamFile:
		if (!_hardfault_file_present || _stream_finished) {
			_state = State::RequestStop;

		} else {
			stream_hardfault();
		}

		break;

	case State::RequestStop:
		request_stop();
		_state = State::WaitStop;

	case State::WaitStop:
		break;
	}
}

int HardfaultStream::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process that streams the latest hardfault via MAVLink.

The module is especially useful when it is necessary to quickly push a hard fault to the ground station.
This is useful in cases where the drone experiences a hard fault during flight.
It ensures that some data is retained in case the permanent storage is destroyed during a crash.

To reliably stream, it is necessary to send the STATUSTEXT message via MAVLink at a
high enough frequency. The recommended frequency is 10 Hz or higher.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hardfault_stream", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int hardfault_stream_main(int argc, char *argv[])
{
	return HardfaultStream::main(argc, argv);
}

} // namespace hardfault_stream

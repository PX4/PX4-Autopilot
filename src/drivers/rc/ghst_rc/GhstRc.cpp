/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "GhstRc.hpp"

#include <poll.h>
#include <termios.h>

using namespace time_literals;

GhstRc::GhstRc(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

GhstRc::~GhstRc()
{
	delete _ghst_telemetry;

	perf_free(_cycle_interval_perf);
	perf_free(_publish_interval_perf);
}

int GhstRc::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device == nullptr) {
		PX4_ERR("valid device required");
		return PX4_ERROR;
	}

	GhstRc *instance = new GhstRc(device);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}

void GhstRc::Run()
{
	if (should_exit()) {
		ScheduleClear();
		::close(_rc_fd);
		exit_and_cleanup();
		return;
	}

	if (_rc_fd < 0) {
		_rc_fd = ::open(_device, O_RDWR | O_NONBLOCK);
	}

	// poll with 3 second timeout
	pollfd fds[1];
	fds[0].fd = _rc_fd;
	fds[0].events = POLLIN;
	::poll(fds, 1, 3000);

	perf_count(_cycle_interval_perf);

	const hrt_abstime cycle_timestamp = hrt_absolute_time();

	// read all available data from the serial RC input UART
	int new_bytes = ::read(_rc_fd, &_rcs_buf[0], RC_MAX_BUFFER_SIZE);

	if (new_bytes > 0) {
		_bytes_rx += new_bytes;
	}

	if (_rc_scan_begin == 0) {
		_rc_scan_begin = cycle_timestamp;
		// Configure serial port for GHST
		ghst_config(_rc_fd);
	}

	if (_rc_locked || (cycle_timestamp - _rc_scan_begin < 300_ms)) {

	}

	// parse new data
	if (new_bytes > 0) {
		uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
		uint16_t raw_rc_count = 0;
		int8_t ghst_rssi = -1;
		bool rc_updated = ghst_parse(cycle_timestamp, &_rcs_buf[0], new_bytes, &raw_rc_values[0], &ghst_rssi, &raw_rc_count,
					     input_rc_s::RC_INPUT_MAX_CHANNELS);

		if (rc_updated) {
			if (!_rc_locked) {
				_rc_locked = true;
				PX4_INFO("RC input locked");
			}

			// we have a new GHST frame. Publish it.
			input_rc_s input_rc{};
			input_rc.timestamp_last_signal = cycle_timestamp;
			input_rc.channel_count = math::max(raw_rc_count, (uint16_t)input_rc_s::RC_INPUT_MAX_CHANNELS);
			input_rc.rssi = ghst_rssi;
			input_rc.rc_lost = (raw_rc_count == 0);
			input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_GHST;

			for (int i = 0; i < input_rc.channel_count; i++) {
				input_rc.values[i] = raw_rc_values[i];
			}

			input_rc.timestamp = hrt_absolute_time();
			_input_rc_pub.publish(input_rc);
			perf_count(_publish_interval_perf);

			if (!_ghst_telemetry) {
				_ghst_telemetry = new GHSTTelemetry(_rc_fd);
			}

			if (_ghst_telemetry) {
				_ghst_telemetry->update(cycle_timestamp);
			}
		}
	}

	ScheduleDelayed(4_ms);
}

int GhstRc::print_status()
{
	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	PX4_INFO("RC state: %s", _rc_locked ? "found" : "searching for signal");

	if (_rc_locked) {
		PX4_INFO("Telemetry: %s", _ghst_telemetry ? "yes" : "no");
	}

	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_publish_interval_perf);

	return 0;
}

int GhstRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GhstRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the GHST RC input parsing.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ghst_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ghst_rc_main(int argc, char *argv[])
{
	return GhstRc::main(argc, argv);
}

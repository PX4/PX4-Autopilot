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

#include <lib/rc/common_rc.h>
#include "ghst_rc.hpp"
#include <lib/rc/ghst.hpp>

#include <px4_log.h>

#include <drivers/device/qurt/uart.h>

#include <termios.h>
#include <fcntl.h>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

uint32_t GhstRc::baudrate = GHST_BAUDRATE;

GhstRc::GhstRc(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(GHST_RC_DEFAULT_PORT))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

GhstRc::~GhstRc()
{
	perf_free(_cycle_interval_perf);
	perf_free(_publish_interval_perf);
}

int GhstRc::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case 'b':
			baudrate = atoi(myoptarg);
			PX4_INFO("Setting GHST baudrate to %u", baudrate);
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
		return PX4_ERROR;
	}

	if (!device_name) {
		PX4_ERR("Valid device required");
		return PX4_ERROR;
	}

	GhstRc *instance = new GhstRc(device_name);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}

void GhstRc::fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[GHST_MAX_NUM_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count_local;

	if (_rc_in.channel_count > GHST_MAX_NUM_CHANNELS) {
		_rc_in.channel_count = GHST_MAX_NUM_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}

		// once filled, reset values back to default
		_raw_rc_values[i] = UINT16_MAX;
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {
		_rc_in.rssi = 255;

	} else {
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

void GhstRc::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_rc_fd = -1;
		exit_and_cleanup();
		return;
	}

	if (_rc_fd < 0) {
		_rc_fd = qurt_uart_open(_device, baudrate);

		if (_rc_fd < 0) {
			PX4_ERR("Error opening port: %s", _device);
			return;
		}

		if (_rc_fd >= 0) {
			_is_singlewire = true;

			// Configure serial port for GHST
			ghst_config(_rc_fd);
		}

		_rc_in.rssi_dbm = NAN;
		_rc_in.link_quality = -1;

	}

	const hrt_abstime time_now_us = hrt_absolute_time();
	const hrt_abstime cycle_timestamp = time_now_us;
	perf_count_interval(_cycle_interval_perf, time_now_us);

	// Read all available data from the serial RC input UART
	int new_bytes = qurt_uart_read(_rc_fd, (char *) &_rcs_buf[0], RC_MAX_BUFFER_SIZE, 500);

	if (new_bytes > 0) {
		_bytes_rx += new_bytes;
		ghstLinkStatistics_t link_stats = { .rssi_pct = -1, .rssi_dbm = NAN, .link_quality = 0 };
		bool rc_updated = ghst_parse(cycle_timestamp, &_rcs_buf[0], new_bytes, &_raw_rc_values[0], &link_stats,
					     &_raw_rc_count, GHST_MAX_NUM_CHANNELS);

		if (rc_updated) {
			_last_packet_seen = time_now_us;
			// we have a new GHST frame. Publish it.
			_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_GHST;
			fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp, false, false, 0, link_stats.rssi_pct);

			// ghst telemetry works on fmu-v5
			// on other Pixhawk (-related) boards we cannot write to the RC UART
			// another option is to use a different UART port
#ifdef BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

			if (!_ghst_telemetry) {
				_ghst_telemetry = new GHSTTelemetry(_rcs_fd);
			}

			if (_ghst_telemetry) {
				_ghst_telemetry->update(cycle_timestamp);
			}

#endif /* BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT */
		}
	}

	// If no communication
	if (time_now_us - _last_packet_seen > 100_ms) {
		// Invalidate link statistics
		_rc_in.rssi_dbm = NAN;
		_rc_in.link_quality = -1;
	}

	// If we have not gotten RC updates specifically
	if (time_now_us - _rc_in.timestamp_last_signal > 50_ms) {
		_rc_in.rc_lost = 1;
		_rc_in.rc_failsafe = 1;
		_rc_in.rssi_dbm = NAN;
		_rc_in.link_quality = -1;

	} else {
		_rc_in.rc_lost = 0;
		_rc_in.rc_failsafe = 0;
	}

	_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_GHST;
	_rc_in.timestamp = hrt_absolute_time();
	_input_rc_pub.publish(_rc_in);

	perf_count(_publish_interval_perf);

	ScheduleDelayed(4_ms);
}

int GhstRc::print_status()
{
	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	if (_is_singlewire) {
		PX4_INFO("Telemetry disabled: Singlewire RC port");
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
This module parses the GHST RC uplink protocol and can generate GHST downlink telemetry data

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

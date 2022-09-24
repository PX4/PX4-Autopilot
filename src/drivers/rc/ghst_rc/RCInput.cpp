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

#include "RCInput.hpp"

#include <termios.h>

using namespace time_literals;

RcGhst::RcGhst(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	// initialize raw_rc values and count
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		_raw_rc_values[i] = UINT16_MAX;
	}

	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

RcGhst::~RcGhst()
{
	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int RcGhst::init()
{
	_rcs_fd = ::open(_device, O_RDWR | O_NONBLOCK);

	if (_rcs_fd >= 0) {
		ghst_config(_rcs_fd);
	}

	if (_rcs_fd < 0) {
		return -errno;
	}

	if (board_rc_swap_rxtx(_device)) {
#if defined(TIOCSSWAP)
		ioctl(_rcs_fd, TIOCSSWAP, SER_SWAP_ENABLED);
#endif // TIOCSSWAP
	}

	rc_io_invert(false);

	return 0;
}

int RcGhst::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;
#if defined(RC_SERIAL_PORT)
	device_name = RC_SERIAL_PORT;
#endif // RC_SERIAL_PORT

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
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

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		RcGhst *instance = new RcGhst(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(_current_update_interval);

		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

void RcGhst::fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count_local;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
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

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

void RcGhst::rc_io_invert(bool invert)
{
	// First check if the board provides a board-specific inversion method (e.g. via GPIO),
	// and if not use an IOCTL
	if (!board_rc_invert_input(_device, invert)) {
#if defined(TIOCSINVERT)

		if (invert) {
			ioctl(_rcs_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX);

		} else {
			ioctl(_rcs_fd, TIOCSINVERT, 0);
		}

#endif // TIOCSINVERT
	}
}

void RcGhst::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}

	} else {

		perf_begin(_cycle_perf);

		const hrt_abstime cycle_timestamp = hrt_absolute_time();

		bool rc_updated = false;

		// This block scans for a supported serial RC input and locks onto the first one found
		// Scan for 500 msec, then switch protocol
		constexpr hrt_abstime rc_scan_max = 500_ms;

		// TODO: needs work (poll _rcs_fd)
		// int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
		// then update priority to SCHED_PRIORITY_FAST_DRIVER
		// read all available data from the serial RC input UART

		// read all available data from the serial RC input UART
		int newBytes = ::read(_rcs_fd, &_rcs_buf[0], RC_MAX_BUFFER_SIZE);

		if (newBytes > 0) {
			_bytes_rx += newBytes;
		}

		const bool rc_scan_locked = _rc_scan_locked;

		if (_rc_scan_begin == 0) {
			_rc_scan_begin = cycle_timestamp;
			// Configure serial port for GHST
			ghst_config(_rcs_fd);

			// flush serial buffer and any existing buffered data
			tcflush(_rcs_fd, TCIOFLUSH);
			memset(_rcs_buf, 0, sizeof(_rcs_buf));

		} else if (_rc_scan_locked
			   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			// parse new data
			if (newBytes > 0) {
				int8_t ghst_rssi = -1;
				rc_updated = ghst_parse(cycle_timestamp, &_rcs_buf[0], newBytes, &_raw_rc_values[0], &ghst_rssi,
							&_raw_rc_count, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new GHST frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_GHST;
					fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp, false, false, 0, ghst_rssi);

					_rc_scan_locked = true;

					// telemetry
					if ((cycle_timestamp - _last_update) > (1_s / (UPDATE_RATE_HZ * NUM_DATA_TYPES))) {

						switch (_next_type) {
						case 0U:
							send_battery_status();
							break;

						case 1U:
							send_gps1_status();
							break;

						case 2U:
							send_gps2_status();
							break;

						default:
							break;
						}

						_last_update = cycle_timestamp;
						_next_type = (_next_type + 1U) % NUM_DATA_TYPES;
					}
				}
			}
		}

		perf_end(_cycle_perf);

		if (rc_updated) {
			perf_count(_publish_interval_perf);

			_to_input_rc.publish(_rc_in);

		} else if (!rc_updated && (hrt_elapsed_time(&_rc_in.timestamp_last_signal) > 1_s)) {
			_rc_scan_locked = false;
		}

		if (!rc_scan_locked && _rc_scan_locked) {
			PX4_INFO("RC input locked");
		}
	}
}

bool RcGhst::send_battery_status()
{
	battery_status_s battery_status;

	if (_battery_status_sub.update(&battery_status)) {

		float voltage_in_10mV = battery_status.voltage_filtered_v * FACTOR_VOLTS_TO_10MV;
		float current_in_10mA = battery_status.current_filtered_a * FACTOR_AMPS_TO_10MA;
		float fuel_in_10mAh = battery_status.discharged_mah * FACTOR_MAH_TO_10MAH;
		return ghst_send_telemetry_battery_status(_rcs_fd,
				static_cast<uint16_t>(voltage_in_10mV),
				static_cast<uint16_t>(current_in_10mA),
				static_cast<uint16_t>(fuel_in_10mAh));
	}

	return false;
}

bool RcGhst::send_gps1_status()
{
	sensor_gps_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		int32_t latitude = vehicle_gps_position.lat;         // 1e-7 degrees
		int32_t longitude = vehicle_gps_position.lon;        // 1e-7 degrees
		uint16_t altitude = vehicle_gps_position.alt / 1000; // mm -> m

		return ghst_send_telemetry_gps1_status(_rcs_fd, latitude, longitude, altitude);
	}

	return false;
}

bool RcGhst::send_gps2_status()
{
	sensor_gps_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {

		uint16_t ground_speed = (uint16_t)(vehicle_gps_position.vel_d_m_s / 3.6f * 10.f);
		uint16_t ground_course = (uint16_t)(math::degrees(vehicle_gps_position.cog_rad) * 100.f);
		uint8_t num_sats = vehicle_gps_position.satellites_used;

		// TBD: Can these be computed in a RC telemetry driver?
		uint16_t home_dist = 0;
		uint16_t home_dir = 0;
		uint8_t flags = 0;

		return ghst_send_telemetry_gps2_status(_rcs_fd, ground_speed, ground_course, num_sats, home_dist, home_dir, flags);
	}

	return false;
}

int RcGhst::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = RcGhst::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RcGhst::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);

	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	PX4_INFO("RC state: %s", _rc_scan_locked ? "found" : "searching for signal");
	// PX4_INFO("GHST Telemetry: %s", _ghst_telemetry ? "yes" : "no");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);

	if (hrt_elapsed_time(&_rc_in.timestamp) < 1_s) {
		print_message(ORB_ID(input_rc), _rc_in);
	}

	return 0;
}

int RcGhst::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the RC input parsing and auto-selecting the method. Supported methods are:
- GHST

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_ghst", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rc_ghst_main(int argc, char *argv[])
{
	return RcGhst::main(argc, argv);
}

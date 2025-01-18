/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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

#include "SbusRc.hpp"

#include <termios.h>

SbusRc::SbusRc(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

SbusRc::~SbusRc()
{
	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int SbusRc::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;
	bool silent = false;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			silent = false;
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
		SbusRc *instance = new SbusRc(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(_current_update_interval);

		return PX4_OK;

	} else if (silent) {
		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_ERR("valid device required");
		}
	}

	return PX4_ERROR;
}

void SbusRc::Run()
{
	if (should_exit()) {

		close(_rcs_fd);

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	const hrt_abstime cycle_timestamp = hrt_absolute_time();

	bool rc_updated = false;

	constexpr hrt_abstime rc_scan_max = 3_s;

	// read all available data from the serial RC input UART
	uint8_t rcs_buf[SBUS_BUFFER_SIZE] {};
	int newBytes = ::read(_rcs_fd, &rcs_buf[0], SBUS_BUFFER_SIZE);

	if (newBytes > 0) {
		_bytes_rx += newBytes;
	}

	const bool rc_scan_locked = _rc_scan_locked;

	if (_rc_scan_begin == 0) {
		_rc_scan_begin = cycle_timestamp;

		// Configure serial port
		if (_rcs_fd < 0) {
			_rcs_fd = open(_device, O_RDWR | O_NONBLOCK);
		}

		sbus_config(_rcs_fd, board_rc_singlewire(_device));

		// First check if the board provides a board-specific inversion method (e.g. via GPIO),
		// and if not use an IOCTL
		if (!board_rc_invert_input(_device, true)) {
#if defined(TIOCSINVERT)
			ioctl(_rcs_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX);
#endif // TIOCSINVERT
		}

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);

	} else if (_rc_scan_locked
		   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

		if (newBytes > 0) {
			uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
			uint16_t raw_rc_count = 0;
			bool sbus_failsafe = false;
			bool sbus_frame_drop = false;
			unsigned sbus_frame_drops = 0;

			if (sbus_parse(cycle_timestamp, &rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count,
				       &sbus_failsafe, &sbus_frame_drop, &sbus_frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS)
			   ) {
				// we have a new SBUS frame. Publish it.
				input_rc_s input_rc{};
				input_rc.timestamp_last_signal = cycle_timestamp;
				input_rc.channel_count = math::constrain(raw_rc_count, (uint16_t)0, (uint16_t)input_rc_s::RC_INPUT_MAX_CHANNELS);
				input_rc.rssi = -1;
				input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;

				unsigned valid_chans = 0;

				for (unsigned i = 0; i < input_rc.channel_count; i++) {
					input_rc.values[i] = raw_rc_values[i];

					if (raw_rc_values[i] != UINT16_MAX) {
						valid_chans++;
					}
				}

				input_rc.channel_count = valid_chans;

				if ((_param_rc_rssi_pwm_chan.get() > 0) && (_param_rc_rssi_pwm_chan.get() < input_rc.channel_count)) {
					const int32_t rssi_pwm_chan = _param_rc_rssi_pwm_chan.get();
					const int32_t rssi_pwm_min = _param_rc_rssi_pwm_min.get();
					const int32_t rssi_pwm_max = _param_rc_rssi_pwm_max.get();

					// get RSSI from input channel
					int rc_rssi = ((input_rc.values[rssi_pwm_chan - 1] - rssi_pwm_min) * 100) / (rssi_pwm_max - rssi_pwm_min);
					input_rc.rssi = math::constrain(rc_rssi, 0, 100);
				}

				if (valid_chans == 0) {
					input_rc.rssi = 0;
				}

				input_rc.rc_failsafe = sbus_failsafe;
				input_rc.rc_lost = (valid_chans == 0);
				input_rc.rc_lost_frame_count = sbus_frame_drops;

				input_rc.link_quality = -1;
				input_rc.rssi_dbm = NAN;

				input_rc.timestamp = hrt_absolute_time();
				_input_rc_pub.publish(input_rc);
				perf_count(_publish_interval_perf);

				_timestamp_last_signal = cycle_timestamp;
				rc_updated = true;

				if (valid_chans > 0) {
					_rc_scan_locked = true;
				}
			}
		}

	} else {
		_rc_scan_begin = 0;
		_rc_scan_locked = false;

		close(_rcs_fd);
		_rcs_fd = -1;
	}

	if (!rc_updated && (hrt_elapsed_time(&_timestamp_last_signal) > 1_s)) {
		_rc_scan_locked = false;
	}

	if (!rc_scan_locked && _rc_scan_locked) {
		PX4_INFO("RC input locked");
	}

	perf_end(_cycle_perf);
}

int SbusRc::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = SbusRc::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int SbusRc::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);

	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	PX4_INFO("RC state: %s", _rc_scan_locked ? "found" : "searching for signal");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);

	return 0;
}

int SbusRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does SBUS RC input parsing.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sbus_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sbus_rc_main(int argc, char *argv[])
{
	return SbusRc::main(argc, argv);
}

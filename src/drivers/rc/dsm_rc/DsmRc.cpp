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

#include "DsmRc.hpp"

#include <uORB/topics/vehicle_command_ack.h>

#include <termios.h>

using namespace time_literals;

DsmRc::DsmRc(const char *device) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

DsmRc::~DsmRc()
{
#if defined(SPEKTRUM_POWER_PASSIVE)
	// Disable power controls for Spektrum receiver
	SPEKTRUM_POWER_PASSIVE();
#endif // SPEKTRUM_POWER_PASSIVE

	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int DsmRc::task_spawn(int argc, char *argv[])
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
		DsmRc *instance = new DsmRc(device_name);

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

void DsmRc::Run()
{
	if (should_exit()) {

		close(_rcs_fd);

		dsm_deinit();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	const hrt_abstime cycle_timestamp = hrt_absolute_time();


	/* vehicle command */
	vehicle_command_s vcmd;

	if (_vehicle_cmd_sub.update(&vcmd)) {
		// Check for a pairing command
		if (vcmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) {

			uint8_t cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
#if defined(SPEKTRUM_POWER)

			if (!_rc_scan_locked && !_armed) {
				if ((int)vcmd.param1 == 0) {
					// DSM binding command
					int dsm_bind_mode = (int)vcmd.param2;

					int dsm_bind_pulses = 0;

					if (dsm_bind_mode == 0) {
						dsm_bind_pulses = DSM2_BIND_PULSES;

					} else if (dsm_bind_mode == 1) {
						dsm_bind_pulses = DSMX_BIND_PULSES;

					} else {
						dsm_bind_pulses = DSMX8_BIND_PULSES;
					}

					bind_spektrum(dsm_bind_pulses);

					cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
				}

			} else {
				cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

#endif // SPEKTRUM_POWER

			// publish acknowledgement
			vehicle_command_ack_s command_ack{};
			command_ack.command = vcmd.command;
			command_ack.result = cmd_ret;
			command_ack.target_system = vcmd.source_system;
			command_ack.target_component = vcmd.source_component;
			command_ack.timestamp = hrt_absolute_time();
			uORB::Publication<vehicle_command_ack_s> vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
			vehicle_command_ack_pub.publish(command_ack);
		}
	}

	bool rc_updated = false;

	constexpr hrt_abstime rc_scan_max = 3_s;

	// read all available data from the serial RC input UART
	uint8_t rcs_buf[DSM_BUFFER_SIZE] {};
	int newBytes = ::read(_rcs_fd, &rcs_buf[0], DSM_BUFFER_SIZE);

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

		dsm_config(_rcs_fd);

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);

	} else if (_rc_scan_locked
		   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

		if (newBytes > 0) {
			uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
			uint16_t raw_rc_count = 0;

			int8_t dsm_rssi = 0;
			bool dsm_11_bit = false;
			unsigned frame_drops = 0;

			if (dsm_parse(cycle_timestamp, &rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count, &dsm_11_bit, &frame_drops,
				      &dsm_rssi, input_rc_s::RC_INPUT_MAX_CHANNELS)
			   ) {
				// we have a new DSM frame. Publish it.
				input_rc_s input_rc{};
				input_rc.timestamp_last_signal = cycle_timestamp;
				input_rc.channel_count = math::constrain(raw_rc_count, (uint16_t)0, (uint16_t)input_rc_s::RC_INPUT_MAX_CHANNELS);
				input_rc.rssi = dsm_rssi;
				input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;

				unsigned valid_chans = 0;

				for (unsigned i = 0; i < input_rc.channel_count; i++) {
					input_rc.values[i] = raw_rc_values[i];

					if (raw_rc_values[i] != UINT16_MAX) {
						valid_chans++;
					}
				}

				input_rc.channel_count = valid_chans;

				if (valid_chans == 0) {
					input_rc.rssi = 0;
				}

				input_rc.rc_lost = (valid_chans == 0);
				input_rc.rc_lost_frame_count = frame_drops;

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

	if (!rc_updated && !_armed && (hrt_elapsed_time(&_timestamp_last_signal) > 1_s)) {
		_rc_scan_locked = false;
	}

	if (!rc_scan_locked && _rc_scan_locked) {
		PX4_INFO("RC input locked");
	}

	perf_end(_cycle_perf);
}

#if defined(SPEKTRUM_POWER)
bool DsmRc::bind_spektrum(int arg) const
{
	int ret = PX4_ERROR;

	/* specify 11ms DSMX. RX will automatically fall back to 22ms or DSM2 if necessary */

	/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
	PX4_INFO("DSM_BIND_START: DSM%s RX", (arg == 0) ? "2" : ((arg == 1) ? "-X" : "-X8"));

	if (arg == DSM2_BIND_PULSES ||
	    arg == DSMX_BIND_PULSES ||
	    arg == DSMX8_BIND_PULSES) {

		dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);

		dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);
		usleep(500000);

		dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
		usleep(72000);

		irqstate_t flags = px4_enter_critical_section();
		dsm_bind(DSM_CMD_BIND_SEND_PULSES, arg);
		px4_leave_critical_section(flags);

		usleep(50000);

		dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

		ret = OK;

	} else {
		PX4_ERR("DSM bind failed");
		ret = -EINVAL;
	}

	return (ret == PX4_OK);
}
#endif /* SPEKTRUM_POWER */

int DsmRc::custom_command(int argc, char *argv[])
{
#if defined(SPEKTRUM_POWER)
	const char *verb = argv[0];

	if (!strcmp(verb, "bind")) {
		uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
		vehicle_command_s vcmd{};
		vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
		vcmd.timestamp = hrt_absolute_time();
		vehicle_command_pub.publish(vcmd);
		return 0;
	}

#endif // SPEKTRUM_POWER

	if (!is_running()) {
		int ret = DsmRc::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int DsmRc::print_status()
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

int DsmRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does Spektrum DSM RC input parsing.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dsm_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

#if defined(SPEKTRUM_POWER)
	PRINT_MODULE_USAGE_COMMAND_DESCR("bind", "Send a DSM bind command (module must be running)");
#endif /* SPEKTRUM_POWER */

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dsm_rc_main(int argc, char *argv[])
{
	return DsmRc::main(argc, argv);
}

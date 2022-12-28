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

#include <uORB/topics/vehicle_command_ack.h>

#include <termios.h>

using namespace time_literals;

RcDsm::RcDsm(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

RcDsm::~RcDsm()
{
#if defined(RC_SERIAL_PORT) && defined(SPEKTRUM_POWER_PASSIVE)

	if ((strcmp(RC_SERIAL_PORT, _device) == 0)) {
		// Disable power controls for Spektrum receiver
		SPEKTRUM_POWER_PASSIVE();
	}

#endif // RC_SERIAL_PORT && SPEKTRUM_POWER_PASSIVE

	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int RcDsm::init()
{


	return 0;
}

int RcDsm::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

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
		RcDsm *instance = new RcDsm(device_name);

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

void RcDsm::Run()
{
	if (should_exit()) {

		if (_rcs_fd >= 0) {
			close(_rcs_fd);
			_rcs_fd = -1;
		}

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

#if defined(RC_SERIAL_PORT)

			if ((strcmp(RC_SERIAL_PORT, _device) == 0)) {
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

						// specify 11ms DSMX. RX will automatically fall back to 22ms or DSM2 if necessary
						if (dsm_bind_pulses == DSM2_BIND_PULSES ||
						    dsm_bind_pulses == DSMX_BIND_PULSES ||
						    dsm_bind_pulses == DSMX8_BIND_PULSES) {

							/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
							PX4_INFO("DSM_BIND_START: DSM%s RX", (dsm_bind_pulses == 0) ? "2" : ((dsm_bind_pulses == 1) ? "-X" : "-X8"));

							dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);

							dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);
							usleep(500000);

							dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
							usleep(72000);

							irqstate_t flags = px4_enter_critical_section();
							dsm_bind(DSM_CMD_BIND_SEND_PULSES, dsm_bind_pulses);
							px4_leave_critical_section(flags);

							usleep(50000);

							dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

							cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

						} else {
							cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
							PX4_ERR("DSM bind failed");
						}
					}

				} else {
					cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

#endif // SPEKTRUM_POWER
			}

#endif // RC_SERIAL_PORT

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

	if (!_rc_scan_locked && (_rc_scan_begin == 0)) {
		_rc_scan_begin = cycle_timestamp;

		// Configure serial port for DSM
#if defined(RC_SERIAL_PORT)

		if ((strcmp(RC_SERIAL_PORT, _device) == 0)) {
# if defined(SPEKTRUM_POWER_CONFIG)
			// Enable power controls for Spektrum receiver
			SPEKTRUM_POWER_CONFIG();
# endif // SPEKTRUM_POWER_CONFIG
# if defined(SPEKTRUM_POWER)
			// enable power on DSM connector
			SPEKTRUM_POWER(true);
# endif // SPEKTRUM_POWER
		}

#endif // RC_SERIAL_PORT

		if (_rcs_fd >= 0) {
			close(_rcs_fd);
			_rcs_fd = -1;
		}

		if (_rcs_fd < 0) {
			// dsm_init sets some file static variables and returns a file descriptor
			// it also powers on the radio if needed
			_rcs_fd = open(_device, O_RDWR | O_NONBLOCK);

			if (board_rc_swap_rxtx(_device)) {
#if defined(TIOCSSWAP)
				ioctl(_rcs_fd, TIOCSSWAP, SER_SWAP_ENABLED);
#endif // TIOCSSWAP
			}

			// First check if the board provides a board-specific inversion method (e.g. via GPIO),
			// and if not use an IOCTL
			if (!board_rc_invert_input(_device, false)) {
#if defined(TIOCSINVERT)
				ioctl(_rcs_fd, TIOCSINVERT, 0);
#endif // TIOCSINVERT
			}
		}

		if (_rcs_fd >= 0) {
			// 115200bps, no parity, one stop bit
			struct termios t;
			tcgetattr(_rcs_fd, &t);
			cfsetspeed(&t, 115200);
			t.c_cflag &= ~(CSTOPB | PARENB);
			tcsetattr(_rcs_fd, TCSANOW, &t);

			// initialise the decoder
			dsm_proto_init();
		}
	}

	if (_rcs_fd >= 0) {
		// TODO: needs work (poll _rcs_fd)
		// int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
		// then update priority to SCHED_PRIORITY_FAST_DRIVER
		// read all available data from the serial RC input UART
		const hrt_abstime timestamp_last_signal = hrt_absolute_time();

		static constexpr size_t RC_MAX_BUFFER_SIZE{sizeof(dsm_decode_t)};
		uint8_t rcs_buf[RC_MAX_BUFFER_SIZE] {};
		int new_bytes = ::read(_rcs_fd, &rcs_buf[0], RC_MAX_BUFFER_SIZE);

		if (new_bytes > 0) {
			_bytes_rx += new_bytes;
		}

		if (new_bytes > 0) {
			uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS];

			for (auto &rc_channel : raw_rc_values) {
				rc_channel = UINT16_MAX;
			}

			uint16_t raw_rc_count = 0;
			bool dsm_11_bit = false;
			unsigned frame_drops = 0;
			int8_t dsm_rssi = -1;

			// parse new data
			const bool rc_updated = dsm_parse(cycle_timestamp, &rcs_buf[0], new_bytes, &raw_rc_values[0], &raw_rc_count,
							  &dsm_11_bit, &frame_drops, &dsm_rssi, input_rc_s::RC_INPUT_MAX_CHANNELS);

			if (rc_updated) {

				if (!_rc_scan_locked) {
					_rc_scan_locked = true;
					PX4_INFO("RC input locked");
				}

				input_rc_s input_rc{};
				input_rc.timestamp_last_signal = timestamp_last_signal;
				input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;

				unsigned valid_chans = 0;

				for (unsigned i = 0; i < math::min(raw_rc_count, (uint16_t)input_rc_s::RC_INPUT_MAX_CHANNELS); i++) {
					input_rc.values[i] = raw_rc_values[i];

					if (raw_rc_values[i] != UINT16_MAX) {
						valid_chans++;
					}
				}

				input_rc.channel_count = valid_chans;
				input_rc.rssi = -1;
				input_rc.link_quality = -1;
				input_rc.rssi_dbm = NAN;

				if ((valid_chans == 0) && (dsm_rssi >= 0) && (dsm_rssi <= 100)) {
					input_rc.rssi = dsm_rssi;
				}

				input_rc.rc_lost = (valid_chans == 0);
				input_rc.rc_lost_frame_count = frame_drops;
				input_rc.rc_total_frame_count = 0; // TODO:
				input_rc.timestamp = hrt_absolute_time();
				_input_rc_pub.publish(input_rc);

				perf_count_interval(_publish_interval_perf, timestamp_last_signal);
				_last_valid_rc = timestamp_last_signal;
			}
		}
	}

	perf_end(_cycle_perf);

	if (!_armed && (hrt_elapsed_time(&_last_valid_rc) > 5_s)) {
		_rc_scan_locked = false;
		_rc_scan_begin = 0;
	}
}

int RcDsm::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "bind")) {
		if (is_running()) {
			uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
			vehicle_command_s vcmd{};
			vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
			vcmd.timestamp = hrt_absolute_time();
			vehicle_command_pub.publish(vcmd);
			return 0;

		} else {
			return -1;
		}
	}

	/* start the FMU if not running */
	if (!is_running()) {
		int ret = RcDsm::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RcDsm::print_status()
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

int RcDsm::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the RC input parsing and auto-selecting the method. Supported methods are:
- DSM

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_dsm", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", false);

#if defined(SPEKTRUM_POWER)
	PRINT_MODULE_USAGE_COMMAND_DESCR("bind", "Send a DSM bind command (module must be running)");
#endif /* SPEKTRUM_POWER */

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int spektrum_dsm_main(int argc, char *argv[])
{
	return RcDsm::main(argc, argv);
}

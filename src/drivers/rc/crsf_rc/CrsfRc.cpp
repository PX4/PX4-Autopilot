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

#include "CrsfRc.hpp"

#include <poll.h>
#include <termios.h>

using namespace time_literals;

CrsfRc::CrsfRc(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

CrsfRc::~CrsfRc()
{
	perf_free(_cycle_interval_perf);
	perf_free(_publish_interval_perf);
}

int CrsfRc::task_spawn(int argc, char *argv[])
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
		CrsfRc *instance = new CrsfRc(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleNow();

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

void CrsfRc::Run()
{
	if (should_exit()) {
		ScheduleClear();
		::close(_rc_fd);
		_rc_fd = -1;
		exit_and_cleanup();
		return;
	}

	if (_rc_fd < 0) {
		_rc_fd = ::open(_device, O_RDWR | O_NONBLOCK);

		if (_rc_fd >= 0) {
			// no parity, one stop bit
			struct termios t {};
			tcgetattr(_rc_fd, &t);
			cfsetspeed(&t, CRSF_BAUDRATE);
			t.c_cflag &= ~(CSTOPB | PARENB);
			tcsetattr(_rc_fd, TCSANOW, &t);
		}
	}

	// poll with 3 second timeout
	pollfd fds[1];
	fds[0].fd = _rc_fd;
	fds[0].events = POLLIN;
	int ret = ::poll(fds, 1, 3000);

	if (ret < 0) {
		PX4_ERR("poll error");
		// try again with delay
		ScheduleDelayed(500_ms);
		return;
	}

	const hrt_abstime time_now_us = hrt_absolute_time();
	perf_count_interval(_cycle_interval_perf, time_now_us);

	// read all available data from the serial RC input UART
	int new_bytes = ::read(_rc_fd, &_rcs_buf[0], RC_MAX_BUFFER_SIZE);

	if (new_bytes > 0) {
		_bytes_rx += new_bytes;

		// parse new data
		uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
		uint16_t raw_rc_count = 0;
		bool rc_updated = crsf::crsf_parse(time_now_us, &_rcs_buf[0], new_bytes, &raw_rc_values[0], &raw_rc_count,
						   input_rc_s::RC_INPUT_MAX_CHANNELS);

		if (rc_updated) {
			if (!_rc_locked) {
				_rc_locked = true;
				PX4_INFO("RC input locked");
			}

			// we have a new CRSF frame. Publish it.
			input_rc_s input_rc{};
			input_rc.timestamp_last_signal = time_now_us;
			input_rc.channel_count = math::max(raw_rc_count, (uint16_t)input_rc_s::RC_INPUT_MAX_CHANNELS);
			input_rc.rssi = -1; // TODO
			input_rc.rc_lost = (raw_rc_count == 0);
			input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF;

			for (int i = 0; i < input_rc.channel_count; i++) {
				input_rc.values[i] = raw_rc_values[i];
			}

			input_rc.timestamp = hrt_absolute_time();
			_input_rc_pub.publish(input_rc);
			perf_count(_publish_interval_perf);
			_rc_valid = input_rc.timestamp;

			if (_param_rc_crsf_tel_en.get() && (input_rc.timestamp > _telemetry_update_last + 100_ms)) {
				switch (_next_type) {
				case 0:
					battery_status_s battery_status;

					if (_battery_status_sub.update(&battery_status)) {
						uint16_t voltage = battery_status.voltage_filtered_v * 10;
						uint16_t current = battery_status.current_filtered_a * 10;
						int fuel = battery_status.discharged_mah;
						uint8_t remaining = battery_status.remaining * 100;
						crsf::crsf_send_telemetry_battery(_rc_fd, voltage, current, fuel, remaining);
					}

					break;

				case 1:
					sensor_gps_s vehicle_gps_position;

					if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
						int32_t latitude = vehicle_gps_position.lat;
						int32_t longitude = vehicle_gps_position.lon;
						uint16_t groundspeed = vehicle_gps_position.vel_d_m_s / 3.6f * 10.f;
						uint16_t gps_heading = math::degrees(vehicle_gps_position.cog_rad) * 100.f;
						uint16_t altitude = vehicle_gps_position.alt + 1000;
						uint8_t num_satellites = vehicle_gps_position.satellites_used;
						crsf::crsf_send_telemetry_gps(_rc_fd, latitude, longitude, groundspeed, gps_heading, altitude, num_satellites);
					}

					break;

				case 2:
					vehicle_attitude_s vehicle_attitude;

					if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
						matrix::Eulerf attitude = matrix::Quatf(vehicle_attitude.q);
						int16_t pitch = attitude(1) * 1e4f;
						int16_t roll = attitude(0) * 1e4f;
						int16_t yaw = attitude(2) * 1e4f;
						crsf::crsf_send_telemetry_attitude(_rc_fd, pitch, roll, yaw);
					}

					break;

				case 3:
					vehicle_status_s vehicle_status;

					if (_vehicle_status_sub.update(&vehicle_status)) {
						const char *flight_mode = "(unknown)";

						switch (vehicle_status.nav_state) {
						case vehicle_status_s::NAVIGATION_STATE_MANUAL:
							flight_mode = "Manual";
							break;

						case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
							flight_mode = "Altitude";
							break;

						case vehicle_status_s::NAVIGATION_STATE_POSCTL:
							flight_mode = "Position";
							break;

						case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
							flight_mode = "Return";
							break;

						case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
							flight_mode = "Mission";
							break;

						case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
						case vehicle_status_s::NAVIGATION_STATE_DESCEND:
						case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
						case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
						case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
						case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
							flight_mode = "Auto";
							break;

						case vehicle_status_s::NAVIGATION_STATE_ACRO:
							flight_mode = "Acro";
							break;

						case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
							flight_mode = "Terminate";
							break;

						case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
							flight_mode = "Offboard";
							break;

						case vehicle_status_s::NAVIGATION_STATE_STAB:
							flight_mode = "Stabilized";
							break;
						}

						crsf::crsf_send_telemetry_flight_mode(_rc_fd, flight_mode);
					}

					break;
				}

				_telemetry_update_last = input_rc.timestamp;
				_next_type = (_next_type + 1) % num_data_types;
			}
		}
	}

	if (!_rc_locked && (hrt_elapsed_time(&_rc_valid) > 5_s)) {
		::close(_rc_fd);
		_rc_fd = -1;
		_rc_locked = false;
		ScheduleDelayed(200_ms);

	} else {
		ScheduleDelayed(4_ms);
	}
}

int CrsfRc::print_status()
{
	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	PX4_INFO("RC state: %s", _rc_locked ? "found" : "searching for signal");

	if (_rc_locked) {
		PX4_INFO("Telemetry: %s", _param_rc_crsf_tel_en.get() ? "yes" : "no");
	}

	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_publish_interval_perf);

	return 0;
}

int CrsfRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CrsfRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the CRSF RC input parsing.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("crsf_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int crsf_rc_main(int argc, char *argv[])
{
	return CrsfRc::main(argc, argv);
}

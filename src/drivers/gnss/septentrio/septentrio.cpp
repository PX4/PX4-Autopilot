/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * @file septentrio.cpp
 *
 * Septentrio GNSS receiver driver
 *
 * @author Matej Franceskin <Matej.Franceskin@gmail.com>
 * @author <a href="https://github.com/SeppeG">Seppe Geuens</a>
 * @author Thomas Frans
*/

#include "septentrio.h"

#include <cmath>
#include <ctime>
#include <string.h>
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <uORB/topics/gps_inject_data.h>

#include "util.h"

using namespace time_literals;

#ifndef GPS_READ_BUFFER_SIZE
#define GPS_READ_BUFFER_SIZE 150 ///< Buffer size for `read()` call
#endif

// TODO: This functionality is not available on the Snapdragon yet.
#ifdef __PX4_QURT
#define NO_MKTIME
#endif

#define SBF_CONFIG_TIMEOUT      1000    ///< Timeout for waiting on ACK in ms
#define SBF_PACKET_TIMEOUT      2       ///< If no data during this delay (in ms) assume that full update received
#define MSG_SIZE                100     ///< Size of the message to be sent to the receiver
#define TIMEOUT_5HZ             500     ///< Timeout time in mS, 1000 mS (1Hz) + 300 mS delta for error
#define RATE_MEASUREMENT_PERIOD 5_s     ///< Rate at which bandwith measurements are taken
// TODO: This number seems wrong. It's also not clear why an ULL is created and casted to UL (time_t).
#define GPS_EPOCH_SECS ((time_t)1234567890ULL)

// Trace macros (disable for production builds)
#define SBF_TRACE_PARSER(...)   {/*PX4_INFO(__VA_ARGS__);*/} ///< decoding progress in parse_char()
#define SBF_TRACE_RXMSG(...)    {/*PX4_INFO(__VA_ARGS__);*/} ///< Rx msgs in payload_rx_done()

// Warning macros (disable to save memory)
#define SBF_WARN(...)           {PX4_WARN(__VA_ARGS__);} ///< Module debug warnings
#define SBF_INFO(...)           {PX4_INFO(__VA_ARGS__);} ///< Module debug info
#define SBF_DEBUG(...)          {/*PX4_INFO(__VA_ARGS__);*/} ///< Module debug tracing

// Commands
#define SBF_FORCE_INPUT "SSSSSSSSSS\n"  /**< Force input on the connected port */

#define SBF_CONFIG_RESET_HOT "ExeResetReceiver, soft, none\n"

#define SBF_CONFIG_RESET_WARM "ExeResetReceiver, soft, PVTData\n"

#define SBF_CONFIG_RESET_COLD "ExeResetReceiver, hard, SatData\n"

#define SBF_CONFIG "setSBFOutput, Stream1, %s, PVTGeodetic+VelCovGeodetic+DOP+AttEuler+AttCovEuler, msec100\n" /**< Configure the correct blocks for GPS positioning and heading */

#define SBF_CONFIG_BAUDRATE "setCOMSettings, %s, baud%lu\n"

#define SBF_CONFIG_RESET "setSBFOutput, Stream1, %s, none, off\n"

#define SBF_CONFIG_RECEIVER_DYNAMICS "setReceiverDynamics, %s, UAV\n"

#define SBF_CONFIG_ATTITUDE_OFFSET "setAttitudeOffset, %.3f, %.3f\n"

#define SBF_DATA_IO "setDataInOut, %s, Auto, %s\n"

#define SBF_ATTITUDE_SOURCE "setGNSSAttitude, %s\n"
#define SBF_ATTITUDE_SOURCE_MOVING_BASE "MovingBase"

static constexpr int SEP_SET_CLOCK_DRIFT_TIME_S{5}; ///< RTC drift time when time synchronization is needed (in seconds)

px4::atomic<SeptentrioGPS *> SeptentrioGPS::_secondary_instance{nullptr};

SeptentrioGPS::SeptentrioGPS(const char *device_path, SeptentrioInstance instance, uint32_t baud_rate) :
	Device(MODULE_NAME),
	_instance(instance),
	_baud_rate(baud_rate)
{
	int32_t mode = 0;

	strncpy(_port, device_path, sizeof(_port) - 1);
	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	_report_gps_pos.heading = NAN;
	_report_gps_pos.heading_offset = NAN;

	int32_t enable_sat_info = 0;
	param_get(param_find("SEP_SAT_INFO"), &enable_sat_info);

	// Create satellite info data object if requested
	if (enable_sat_info) {
		_sat_info = new GPSSatelliteInfo();
		_p_report_sat_info = &_sat_info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}

	param_get(param_find("SEP_MODE"), &mode);

	switch (mode) {
	case 0:
	default:
		_mode = SeptentrioMode::Default;
		break;
	case 1:
		_mode = SeptentrioMode::RoverWithMovingBase;
		break;
	}
}

SeptentrioGPS::~SeptentrioGPS()
{
	SeptentrioGPS *secondary_instance = _secondary_instance.load();

	if (_instance == SeptentrioInstance::Main && secondary_instance) {
		secondary_instance->request_stop();

		// Wait for exit
		uint32_t i = 0;

		do {
			px4_usleep(20000);
			++i;
		} while (_secondary_instance.load() && i < 100);
	}

	delete _dump_from_device;
	delete _dump_to_device;
	delete _rtcm_parsing;
	delete _sat_info;
}

int SeptentrioGPS::print_status()
{
	SeptentrioGPS *secondary_instance = _secondary_instance.load();

	switch (_instance) {
	case SeptentrioInstance::Main:
		PX4_INFO("Main GPS");
		break;

	case SeptentrioInstance::Secondary:
		PX4_INFO("");
		PX4_INFO("Secondary GPS");

	default:
		break;
	}

	PX4_INFO("status: %s, port: %s, baud rate: %lu", _healthy ? "OK" : "NOT OK", _port, _baud_rate);
	PX4_INFO("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");
	PX4_INFO("rate reading: \t\t%6i B/s", _rate_reading);

	if (_report_gps_pos.timestamp != 0) {
		PX4_INFO("rate position: \t\t%6.2f Hz", (double)get_position_update_rate());
		PX4_INFO("rate velocity: \t\t%6.2f Hz", (double)get_velocity_update_rate());

		PX4_INFO("rate publication:\t\t%6.2f Hz", (double)_rate);
		PX4_INFO("rate RTCM injection:\t%6.2f Hz", (double)_rate_rtcm_injection);

		print_message(ORB_ID(sensor_gps), _report_gps_pos);
	}

	if (_instance == SeptentrioInstance::Main && secondary_instance) {
		secondary_instance->print_status();
	}

	return 0;
}

void SeptentrioGPS::run()
{
	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;
	param_t handle = param_find("SEP_YAW_OFFS");
	float heading_offset = 0.f;

	if (handle != PARAM_INVALID) {
		param_get(handle, &heading_offset);
		heading_offset = matrix::wrap_pi(math::radians(heading_offset));
	}

	if (initialize_communication_dump() == PX4_ERROR) {
		SBF_WARN("GPS communication logging could not be initialized");
	}

	// Set up the communication, configure the receiver and start processing data until we have to exit.
	while (!should_exit()) {
		_uart.setPort(_port);

		if (!_uart.open()) {
			PX4_ERR("Error opening serial device %s", _port);
			continue;
		}

		decode_init();

		// TODO: Not sure whether this is still correct when drivers are separate modules.
		set_device_type(DRV_GPS_DEVTYPE_SBF);

		if (_dump_communication_mode == SeptentrioDumpCommMode::RTCM) {
			_output_mode = SeptentrioGPSOutputMode::GPSAndRTCM;

		} else {
			_output_mode = SeptentrioGPSOutputMode::GPS;
		}

		// If configuration is successful, start processing messages.
		if (configure(heading_offset) == 0) {

			PX4_INFO("Automatic configuration finished");

			// Reset report
			memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));
			_report_gps_pos.heading = NAN;
			_report_gps_pos.heading_offset = heading_offset;

			// Read data from the receiver and publish it until an error occurs.
			int helper_ret;
			unsigned receive_timeout = TIMEOUT_5HZ;

			while ((helper_ret = receive(receive_timeout)) > 0 && !should_exit()) {

				if (helper_ret & 1) {
					publish();

					last_rate_count++;
				}

				if (_p_report_sat_info && (helper_ret & 2)) {
					publish_satellite_info();
				}

				reset_if_scheduled();

				// Measure update rate every 5 seconds
				if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
					float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
					_rate = last_rate_count / dt;
					_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
					_rate_reading = _num_bytes_read / dt;
					last_rate_measurement = hrt_absolute_time();
					last_rate_count = 0;
					_last_rate_rtcm_injection_count = 0;
					_num_bytes_read = 0;
					store_update_rates();
					reset_update_rates();
				}

				if (!_healthy) {
					_healthy = true;
				}
			}

			if (_healthy) {
				_healthy = false;
				_rate = 0.0f;
				_rate_rtcm_injection = 0.0f;
			}
		}

		_uart.close();
	}

	PX4_INFO("exiting");
}

int SeptentrioGPS::task_spawn(int argc, char *argv[])
{
	return task_spawn(argc, argv, SeptentrioInstance::Main);
}

int SeptentrioGPS::task_spawn(int argc, char *argv[], SeptentrioInstance instance)
{
	px4_main_t entry_point;
	static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2040);

	if (instance == SeptentrioInstance::Main) {
		entry_point = (px4_main_t)&run_trampoline;

	} else {
		entry_point = (px4_main_t)&run_trampoline_secondary;
	}

	px4_task_t task_id = px4_task_spawn_cmd("septentrio",
						SCHED_DEFAULT,
						SCHED_PRIORITY_SLOW_DRIVER,
						TASK_STACK_SIZE,
						entry_point,
						(char *const *)argv);

	if (task_id < 0) {
		// `_task_id` of module that hasn't been started before or has been stopped should already be -1.
		// This is just to make sure.
		_task_id = -1;
		return -errno;
	}

	if (instance == SeptentrioInstance::Main) {
		_task_id = task_id;
	}

	return 0;
}

int SeptentrioGPS::run_trampoline_secondary(int argc, char *argv[])
{
	// Get rid of the task name (first argument)
	argc -= 1;
	argv += 1;

	SeptentrioGPS *gps = instantiate(argc, argv, SeptentrioInstance::Secondary);

	if (gps) {
		_secondary_instance.store(gps);
		gps->run();
		_secondary_instance.store(nullptr);
		delete gps;
	}

	return 0;
}

SeptentrioGPS *SeptentrioGPS::instantiate(int argc, char *argv[])
{
	return instantiate(argc, argv, SeptentrioInstance::Main);
}

SeptentrioGPS *SeptentrioGPS::instantiate(int argc, char *argv[], SeptentrioInstance instance)
{
	const char *device_path = nullptr;
	int baud_rate = 0;
	const char *device_path_secondary = nullptr;
	int baud_rate_secondary = 0;
	SeptentrioGPS *gps = nullptr;
	bool error_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:e:b:g:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baud_rate) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;
		case 'g':
			if (px4_get_parameter_value(myoptarg, baud_rate_secondary) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;
		case 'd':
			device_path = myoptarg;
			break;

		case 'e':
			device_path_secondary = myoptarg;
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
		return nullptr;
	}

	if (instance == SeptentrioInstance::Main) {
		if (device::Serial::validatePort(device_path)) {
			gps = new SeptentrioGPS(device_path, instance, baud_rate);

		} else {
			PX4_ERR("invalid device (-d) %s", device_path ? device_path : "");
		}

		if (gps && device_path_secondary) {
			task_spawn(argc, argv, SeptentrioInstance::Secondary);

			// Wait until the secondary instance is running
			uint32_t i = 0;

			do {
				px4_usleep(2500);
			} while (!_secondary_instance.load() && ++i < 400);

			if (i == 400) {
				PX4_ERR("Timed out while waiting for second instance to start");
			}
		}

	} else {
		if (device::Serial::validatePort(device_path_secondary)) {
			gps = new SeptentrioGPS(device_path_secondary, instance, baud_rate_secondary);

		} else {
			PX4_ERR("Invalid secondary device (-e) %s", device_path_secondary ? device_path_secondary : "");
		}
	}

	return gps;
}

// Called from outside driver thread.
// Return 0 on success, -1 otherwise.
int SeptentrioGPS::custom_command(int argc, char *argv[])
{
	bool handled = false;
	SeptentrioGPS *driver_instance;

	if (!is_running()) {
		PX4_INFO("not running");
		return -1;
	}

	driver_instance = get_instance();

	if (argc == 2 && !strcmp(argv[0], "reset")) {

		if (!strcmp(argv[1], "hot")) {
			handled = true;
			driver_instance->schedule_reset(SeptentrioGPSResetType::Hot);

		} else if (!strcmp(argv[1], "cold")) {
			handled = true;
			driver_instance->schedule_reset(SeptentrioGPSResetType::Cold);

		} else if (!strcmp(argv[1], "warm")) {
			handled = true;
			driver_instance->schedule_reset(SeptentrioGPSResetType::Warm);
		}
	}

	if (handled) {
		PX4_INFO("Resetting GPS - %s", argv[1]);
		return 0;
	}

	return (handled) ? 0 : print_usage("unknown command");
}

int SeptentrioGPS::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPS driver module that handles the communication with Septentrio devices and publishes the position via uORB.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published on
the second uORB topic instance. It can be used for logging and heading computation.

### Examples

Starting 2 GPS devices (main one on /dev/ttyS3, secondary on /dev/ttyS4)
$ septentrio start -d /dev/ttyS3 -e /dev/ttyS4

Initiate warm restart of GPS device
$ gps reset warm
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("septentrio", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "Primary Septentrio receiver", true);
	PRINT_MODULE_USAGE_PARAM_STRING('e', "/dev/ttyS4", "<file:dev>", "Secondary Septentrio receiver", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset connected receiver");
	PRINT_MODULE_USAGE_ARG("cold|warm|hot", "Specify reset type", false);

	return 0;
}

int SeptentrioGPS::reset(SeptentrioGPSResetType type)
{
	bool res = false;

	send_message(SBF_FORCE_INPUT);

	// The receiver can't receive commands right after forcing input.
	px4_usleep(500000);

	switch (type) {
	case SeptentrioGPSResetType::Hot:
		res = send_message_and_wait_for_ack(SBF_CONFIG_RESET_HOT, SBF_CONFIG_TIMEOUT);
		break;

	case SeptentrioGPSResetType::Warm:
		res = send_message_and_wait_for_ack(SBF_CONFIG_RESET_WARM, SBF_CONFIG_TIMEOUT);
		break;

	case SeptentrioGPSResetType::Cold:
		res = send_message_and_wait_for_ack(SBF_CONFIG_RESET_COLD, SBF_CONFIG_TIMEOUT);
		break;

	default:
		break;
	}

	if (res) {
		return PX4_OK;
	} else {
		return PX4_ERROR;
	}
}

float SeptentrioGPS::get_position_update_rate()
{
	return _rate_lat_lon;
}

float SeptentrioGPS::get_velocity_update_rate()
{
	return _rate_vel;
}

void SeptentrioGPS::schedule_reset(SeptentrioGPSResetType reset_type)
{
	SeptentrioGPS *secondary_instance = _secondary_instance.load();

	_scheduled_reset.store((int)reset_type);

	if (_instance == SeptentrioInstance::Main && secondary_instance) {
		secondary_instance->schedule_reset(reset_type);
	}
}

uint32_t SeptentrioGPS::detect_receiver_baud_rate(bool force_input) {
	// Baud rates we expect the receiver to be running at.
	const uint32_t expected_baud_rates[] = {115200, 230400, 57600, 460800, 500000, 576000, 1200, 2400, 4800, 9600, 19200, 38400, 921600, 1000000, 1500000};
	// So we can restore the port to its original state.
	const uint32_t original_baud_rate = _uart.getBaudrate();

	for (uint i = 0; i < sizeof(expected_baud_rates)/sizeof(expected_baud_rates[0]); i++) {
		if (set_baudrate(expected_baud_rates[i]) != PX4_OK)
			return 0;

		SBF_INFO("`_uart` now running at %lu", _uart.getBaudrate());

		px4_usleep(20000);

		if (force_input) {
			send_message(SBF_FORCE_INPUT);
			px4_usleep(20000);
		}

		if (send_message_and_wait_for_ack("getEchoMessage\n", SBF_CONFIG_TIMEOUT)) {
			_uart.setBaudrate(original_baud_rate);
			PX4_INFO("Detected baud rate: %lu", expected_baud_rates[i]);
			return expected_baud_rates[i];
		}

		px4_usleep(20000);
	}

	_uart.setBaudrate(original_baud_rate);
	return 0;
}

int SeptentrioGPS::detect_serial_port(char* const port_name) {
	// Read buffer to get the COM port
	char buf[GPS_READ_BUFFER_SIZE];
	size_t buffer_offset = 0;   // The offset into the string where the next data should be read to.
	hrt_abstime time_started = hrt_absolute_time();
	bool response_detected = false;

	// Receiver prints prompt after forcing input.
	send_message(SBF_FORCE_INPUT);

	do {
		// Read at most the amount of available bytes in the buffer after the current offset, -1 because we need '\0' at the end for a valid string.
		int read_result = read(reinterpret_cast<uint8_t *>(buf) + buffer_offset, sizeof(buf) - buffer_offset - 1, SBF_CONFIG_TIMEOUT);

		if (read_result < 0) {
			SBF_WARN("SBF read error");
			return PX4_ERROR;
		}

		// Sanitize the data so it doesn't contain any `0` values.
		for (size_t i = buffer_offset; i < buffer_offset + read_result; i++) {
			if (buf[i] == 0) {
				buf[i] = 1;
			}
		}

		buffer_offset += read_result;

		// Make sure the current buffer is a valid string.
		buf[buffer_offset] = '\0';

		char* port_name_address = strstr(buf, ">");

		// Check if we found a port candidate.
		if (buffer_offset > 4 && port_name_address != nullptr) {
			size_t port_name_offset = reinterpret_cast<size_t>(port_name_address) - reinterpret_cast<size_t>(buf) - 4;
			for (size_t i = 0; i < 4; i++) {
				port_name[i] = buf[port_name_offset + i];
			}
			// NOTE: This limits the ports to serial and USB ports only. Otherwise the detection doesn't work correctly.
			if (strstr(port_name, "COM") != nullptr || strstr(port_name, "USB") != nullptr) {
				response_detected = true;
				break;
			}
		}

		if (buffer_offset + 1 >= sizeof(buf)) {
			// Copy the last 3 bytes such that a half port isn't lost.
			for (int i = 0; i < 4; i++) {
				buf[i] = buf[sizeof(buf) - 4 + i];
			}
			buffer_offset = 3;
		}
	} while (time_started + 5 * 1000 * SBF_CONFIG_TIMEOUT > hrt_absolute_time());

	if (!response_detected) {
		SBF_WARN("No valid serial port detected");
		return PX4_ERROR;
	} else {
		PX4_INFO("Serial port found: %s", port_name);
		return PX4_OK;
	}
}

int SeptentrioGPS::configure(float heading_offset)
{
	uint32_t detected_receiver_baud_rate = 0;
	char com_port[5] {};
	float pitch_offset = 0.f;
	char msg[MSG_SIZE];

	_configured = false;

	// Passively detect receiver baud rate.
	detected_receiver_baud_rate = detect_receiver_baud_rate(true);

	if (detected_receiver_baud_rate == 0) {
		SBF_INFO("CONFIG: failed baud detection");
		return PX4_ERROR;
	}

	// Set same baud rate on our end.
	if (set_baudrate(detected_receiver_baud_rate) != PX4_OK) {
		SBF_INFO("CONFIG: failed local baud rate setting");
		return PX4_ERROR;
	}

	// Passively detect receiver port.
	if (detect_serial_port(com_port) != PX4_OK) {
		SBF_INFO("CONFIG: failed port detection");
		return PX4_ERROR;
	}

	// If user requested specific baud rate, set it now. Otherwise keep detected baud rate.
	if (strstr(com_port, "COM") != nullptr && _baud_rate != 0) {
		snprintf(msg, sizeof(msg), SBF_CONFIG_BAUDRATE, com_port, _baud_rate);

		if (!send_message(msg)) {
			SBF_INFO("CONFIG: failed baud rate command write");
			return PX4_ERROR;
		}

		px4_usleep(20000);

		if (!send_message(msg)) {
			SBF_INFO("CONFIG: failed baud rate command write");
			return PX4_ERROR;
		}

		px4_usleep(20000);

		if (set_baudrate(_baud_rate) != PX4_OK) {
			SBF_INFO("CONFIG: failed local baud rate setting");
			return PX4_ERROR;
		}

		if (!send_message_and_wait_for_ack("getEchoMessage\n", SBF_CONFIG_TIMEOUT)) {
			SBF_INFO("CONFIG: failed baud rate setting");
			return PX4_ERROR;
		}
	}

	// Delete all sbf outputs on current COM port to remove clutter data
	snprintf(msg, sizeof(msg), SBF_CONFIG_RESET, com_port);

	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		SBF_INFO("CONFIG: failed delete output");
		return PX4_ERROR; // connection and/or baudrate detection failed
	}

	// Define/inquire the type of data that the receiver should accept/send on a given connection descriptor
	snprintf(msg, sizeof(msg), SBF_DATA_IO, com_port, "SBF");

	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		return PX4_ERROR;
	}

	param_t handle = param_find("SEP_PITCH_OFFS");

	if (handle != PARAM_INVALID) {
		param_get(handle, &pitch_offset);
	}

	// Specify the offsets that the receiver applies to the computed attitude angles.
	snprintf(msg, sizeof(msg), SBF_CONFIG_ATTITUDE_OFFSET, (double)(heading_offset * 180 / M_PI_F), (double)pitch_offset);

	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		return PX4_ERROR;
	}

	snprintf(msg, sizeof(msg), SBF_CONFIG_RECEIVER_DYNAMICS, "high");
	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		return PX4_ERROR;
	}

	// Output a set of SBF blocks on a given connection at a regular interval.
	snprintf(msg, sizeof(msg), SBF_CONFIG, com_port);
	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		PX4_ERR("Failed to configure SBF");
		return PX4_ERROR;
	}

	if (_mode == SeptentrioMode::RoverWithMovingBase) {
		if (_instance == SeptentrioInstance::Main) {
			snprintf(msg, sizeof(msg), SBF_ATTITUDE_SOURCE, SBF_ATTITUDE_SOURCE_MOVING_BASE);
			if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
				SBF_INFO("CONFIG: failed to set moving base");
				return PX4_ERROR;
			}
		} else {
			snprintf(msg, sizeof(msg), SBF_DATA_IO, com_port, "+RTCMv3");
			if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
				SBF_INFO("CONFIG: failed to set data io");
				return PX4_ERROR;
			}
		}
	}

	_configured = true;

	return PX4_OK;
}

int SeptentrioGPS::parse_char(const uint8_t byte)
{
	int ret = 0;

	switch (_decode_state) {

	// Expecting Sync1
	case SBF_DECODE_SYNC1:
		if (byte == SBF_SYNC1) { // Sync1 found --> expecting Sync2
			SBF_TRACE_PARSER("A");
			payload_rx_add(byte); // add a payload byte
			_decode_state = SBF_DECODE_SYNC2;

		} else if (byte == RTCM3_PREAMBLE && _rtcm_parsing) {
			SBF_TRACE_PARSER("RTCM");
			_decode_state = SBF_DECODE_RTCM3;
			_rtcm_parsing->addByte(byte);
		}

		break;

	// Expecting Sync2
	case SBF_DECODE_SYNC2:
		if (byte == SBF_SYNC2) { // Sync2 found --> expecting CRC
			SBF_TRACE_PARSER("B");
			payload_rx_add(byte); // add a payload byte
			_decode_state = SBF_DECODE_PAYLOAD;

		} else { // Sync1 not followed by Sync2: reset parser
			decode_init();
		}

		break;

	// Expecting payload
	case SBF_DECODE_PAYLOAD: SBF_TRACE_PARSER(".");

		ret = payload_rx_add(byte); // add a payload byte

		if (ret < 0) {
			// payload not handled, discard message
			ret = 0;
			decode_init();

		} else if (ret > 0) {
			ret = payload_rx_done(); // finish payload processing
			decode_init();

		} else {
			// expecting more payload, stay in state SBF_DECODE_PAYLOAD
			ret = 0;

		}

		break;

	case SBF_DECODE_RTCM3:
		if (_rtcm_parsing->addByte(byte)) {
			SBF_DEBUG("got RTCM message with length %i", (int) _rtcm_parsing->messageLength());
			got_rtcm_message(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
			decode_init();
		}

		break;

	default:
		break;
	}

	return ret;
}

int SeptentrioGPS::payload_rx_add(const uint8_t byte)
{
	int ret = 0;
	uint8_t *p_buf = reinterpret_cast<uint8_t *>(&_buf);

	p_buf[_rx_payload_index++] = byte;

	if ((_rx_payload_index > 7 && _rx_payload_index >= _buf.length) || _rx_payload_index >= sizeof(_buf)) {
		ret = 1; // payload received completely
	}

	return ret;
}

int SeptentrioGPS::payload_rx_done()
{
	int ret = 0;

	if (_buf.length <= 4 ||
	    _buf.length > _rx_payload_index ||
	    _buf.crc16 != sep_crc16(reinterpret_cast<uint8_t *>(&_buf) + 4, _buf.length - 4)) {
		return 0;
	}

	// Handle message
	switch (_buf.msg_id) {
	case SBF_ID_PVTGeodetic: SBF_TRACE_RXMSG("Rx PVTGeodetic");
		_msg_status |= 1;

		if (_buf.payload_pvt_geodetic.mode_type < 1) {
			_report_gps_pos.fix_type = 1;

		} else if (_buf.payload_pvt_geodetic.mode_type == 6) {
			_report_gps_pos.fix_type = 4;

		} else if (_buf.payload_pvt_geodetic.mode_type == 5 || _buf.payload_pvt_geodetic.mode_type == 8) {
			_report_gps_pos.fix_type = 5;

		} else if (_buf.payload_pvt_geodetic.mode_type == 4 || _buf.payload_pvt_geodetic.mode_type == 7) {
			_report_gps_pos.fix_type = 6;

		} else {
			_report_gps_pos.fix_type = 3;
		}

		// Check fix and error code
		_report_gps_pos.vel_ned_valid = _report_gps_pos.fix_type > 1 && _buf.payload_pvt_geodetic.error == 0;

		// Check boundaries and invalidate GPS velocities
		// We're not just checking for the do-not-use value (-2*10^10) but for any value beyond the specified max values
		if (fabsf(_buf.payload_pvt_geodetic.vn) > 600.0f || fabsf(_buf.payload_pvt_geodetic.ve) > 600.0f ||
		    fabsf(_buf.payload_pvt_geodetic.vu) > 600.0f) {
			_report_gps_pos.vel_ned_valid = false;
		}

		// Check boundaries and invalidate position
		// We're not just checking for the do-not-use value (-2*10^10) but for any value beyond the specified max values
		if (fabs(_buf.payload_pvt_geodetic.latitude) > (double)(M_PI_F / 2.0f) ||
		    fabs(_buf.payload_pvt_geodetic.longitude) > (double) M_PI_F ||
		    fabs(_buf.payload_pvt_geodetic.height) > SBF_PVTGEODETIC_DNU ||
		    fabsf(_buf.payload_pvt_geodetic.undulation) > (float) SBF_PVTGEODETIC_DNU) {
			_report_gps_pos.fix_type = 0;
		}

		if (_buf.payload_pvt_geodetic.nr_sv < 255) {  // 255 = do not use value
			_report_gps_pos.satellites_used = _buf.payload_pvt_geodetic.nr_sv;

			if (_p_report_sat_info) {
				// Only fill in the satellite count for now (we could use the ChannelStatus message for the
				// other data, but it's really large: >800B)
				_p_report_sat_info->timestamp = hrt_absolute_time();
				_p_report_sat_info->count = _report_gps_pos.satellites_used;
				ret = 2;
			}

		} else {
			_report_gps_pos.satellites_used = 0;
		}

		_report_gps_pos.latitude_deg = _buf.payload_pvt_geodetic.latitude * M_RAD_TO_DEG;
		_report_gps_pos.longitude_deg = _buf.payload_pvt_geodetic.longitude * M_RAD_TO_DEG;
		_report_gps_pos.altitude_ellipsoid_m = _buf.payload_pvt_geodetic.height;
		_report_gps_pos.altitude_msl_m = _buf.payload_pvt_geodetic.height - static_cast<double>
						(_buf.payload_pvt_geodetic.undulation);

		/* H and V accuracy are reported in 2DRMS, but based off the uBlox reporting we expect RMS.
		 * Devide by 100 from cm to m and in addition divide by 2 to get RMS. */
		_report_gps_pos.eph = static_cast<float>(_buf.payload_pvt_geodetic.h_accuracy) / 200.0f;
		_report_gps_pos.epv = static_cast<float>(_buf.payload_pvt_geodetic.v_accuracy) / 200.0f;

		_report_gps_pos.vel_n_m_s = static_cast<float>(_buf.payload_pvt_geodetic.vn);
		_report_gps_pos.vel_e_m_s = static_cast<float>(_buf.payload_pvt_geodetic.ve);
		_report_gps_pos.vel_d_m_s = -1.0f * static_cast<float>(_buf.payload_pvt_geodetic.vu);
		_report_gps_pos.vel_m_s = sqrtf(_report_gps_pos.vel_n_m_s * _report_gps_pos.vel_n_m_s +
					       _report_gps_pos.vel_e_m_s * _report_gps_pos.vel_e_m_s);

		_report_gps_pos.cog_rad = static_cast<float>(_buf.payload_pvt_geodetic.cog) * M_DEG_TO_RAD_F;
		_report_gps_pos.c_variance_rad = 1.0f * M_DEG_TO_RAD_F;

		// _buf.payload_pvt_geodetic.cog is set to -2*10^10 for velocities below 0.1m/s
		if (_buf.payload_pvt_geodetic.cog > 360.0f) {
			_buf.payload_pvt_geodetic.cog = NAN;
		}

		_report_gps_pos.time_utc_usec = 0;
#ifndef NO_MKTIME
		struct tm timeinfo;
		time_t epoch;

		// Convert to unix timestamp
		memset(&timeinfo, 0, sizeof(timeinfo));

		timeinfo.tm_year = 1980 - 1900;
		timeinfo.tm_mon = 0;
		timeinfo.tm_mday = 6 + _buf.WNc * 7;
		timeinfo.tm_hour = 0;
		timeinfo.tm_min = 0;
		timeinfo.tm_sec = _buf.TOW / 1000;

		epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.

			timespec ts;
			memset(&ts, 0, sizeof(ts));
			ts.tv_sec = epoch;
			ts.tv_nsec = (_buf.TOW % 1000) * 1000 * 1000;
			set_clock(ts);

			_report_gps_pos.time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_report_gps_pos.time_utc_usec += (_buf.TOW % 1000) * 1000;
		}

#endif
		_report_gps_pos.timestamp = hrt_absolute_time();
		_last_timestamp_time = _report_gps_pos.timestamp;
		_rate_count_vel++;
		_rate_count_lat_lon++;
		// NOTE: Isn't this just `ret |= (_msg_status == 7)`?
		ret |= (_msg_status == 7) ? 1 : 0;
		break;

	case SBF_ID_VelCovGeodetic: SBF_TRACE_RXMSG("Rx VelCovGeodetic");
		_msg_status |= 2;
		_report_gps_pos.s_variance_m_s = _buf.payload_vel_col_geodetic.cov_ve_ve;

		if (_report_gps_pos.s_variance_m_s < _buf.payload_vel_col_geodetic.cov_vn_vn) {
			_report_gps_pos.s_variance_m_s = _buf.payload_vel_col_geodetic.cov_vn_vn;
		}

		if (_report_gps_pos.s_variance_m_s < _buf.payload_vel_col_geodetic.cov_vu_vu) {
			_report_gps_pos.s_variance_m_s = _buf.payload_vel_col_geodetic.cov_vu_vu;
		}

		//SBF_DEBUG("VelCovGeodetic handled");
		break;

	case SBF_ID_DOP: SBF_TRACE_RXMSG("Rx DOP");
		_msg_status |= 4;
		_report_gps_pos.hdop = _buf.payload_dop.hDOP * 0.01f;
		_report_gps_pos.vdop = _buf.payload_dop.vDOP * 0.01f;
		//SBF_DEBUG("DOP handled");
		break;

	case SBF_ID_AttEuler: SBF_TRACE_RXMSG("Rx AttEuler");

		if (!_buf.payload_att_euler.error_not_requested) {

			int error_aux1 = _buf.payload_att_euler.error_aux1;
			int error_aux2 = _buf.payload_att_euler.error_aux2;

			// SBF_DEBUG("Mode: %u", _buf.payload_att_euler.mode)
			if (error_aux1 == 0 && error_aux2 == 0) {
				float heading = _buf.payload_att_euler.heading;
				heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]


				if (heading > M_PI_F) {
					heading -= 2.f * M_PI_F; // final range is [-pi, pi]
				}

				_report_gps_pos.heading = heading;
				// SBF_DEBUG("Heading: %.3f rad", (double) _report_gps_pos.heading)
				//SBF_DEBUG("AttEuler handled");

			} else if (error_aux1 != 0) {
				//SBF_DEBUG("Error code for Main-Aux1 baseline: Not enough measurements");
			} else if (error_aux2 != 0) {
				//SBF_DEBUG("Error code for Main-Aux2 baseline: Not enough measurements");
			}
		} else {
			//SBF_DEBUG("AttEuler: attitude not requested by user");
		}


		break;

	case SBF_ID_AttCovEuler: SBF_TRACE_RXMSG("Rx AttCovEuler");

		if (!_buf.payload_att_cov_euler.error_not_requested) {
			int error_aux1 = _buf.payload_att_cov_euler.error_aux1;
			int error_aux2 = _buf.payload_att_cov_euler.error_aux2;

			if (error_aux1 == 0 && error_aux2 == 0) {
				float heading_acc = _buf.payload_att_cov_euler.cov_headhead;
				heading_acc *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
				_report_gps_pos.heading_accuracy = heading_acc;
				// SBF_DEBUG("Heading-Accuracy: %.3f rad", (double) _report_gps_pos.heading_accuracy)
				//SBF_DEBUG("AttCovEuler handled");

			} else if (error_aux1 != 0) {
				//SBF_DEBUG("Error code for Main-Aux1 baseline: %u: Not enough measurements", error_aux1);
			} else if (error_aux2 != 0) {
				//SBF_DEBUG("Error code for Main-Aux2 baseline: %u: Not enough measurements", error_aux2);
			}
		} else {
			//SBF_DEBUG("AttCovEuler: attitude not requested by user");
		}

		break;

	default:
		break;
	}

	if (ret > 0) {
		// NOTE: Isn't this always 0?
		_report_gps_pos.timestamp_time_relative = static_cast<int32_t>(_last_timestamp_time - _report_gps_pos.timestamp);
	}

	if (ret == 1) {
		_msg_status &= ~1;
	}

	return ret;
}

void SeptentrioGPS::decode_init()
{
	_decode_state = SBF_DECODE_SYNC1;
	_rx_payload_index = 0;

	if (_output_mode == SeptentrioGPSOutputMode::GPSAndRTCM || _mode != SeptentrioMode::Default) {
		if (!_rtcm_parsing) {
			_rtcm_parsing = new RTCMParsing();
		}

		if (_rtcm_parsing) {
			_rtcm_parsing->reset();
		}
	}
}

bool SeptentrioGPS::send_message(const char *msg)
{
	SBF_DEBUG("Send MSG: %s", msg);
	int length = strlen(msg);

	return (write(reinterpret_cast<const uint8_t*>(msg), length) == length);
}

bool SeptentrioGPS::send_message_and_wait_for_ack(const char *msg, const int timeout)
{
	if (!send_message(msg)) {
		return false;
	}

	// Wait for acknowledge
	// For all valid set -, get - and exe -commands, the first line of the reply is an exact copy
	// of the command as entered by the user, preceded with "$R:"
	char buf[GPS_READ_BUFFER_SIZE];
	size_t buffer_offset = 0;   // The offset into the string where the next data should be read to.
	hrt_abstime time_started = hrt_absolute_time();

	do {
		// Read at most the amount of available bytes in the buffer after the current offset, -1 because we need '\0' at the end for a valid string.
		int read_result = read(reinterpret_cast<uint8_t *>(buf) + buffer_offset, sizeof(buf) - buffer_offset - 1, 50);

		if (read_result < 0) {
			SBF_WARN("SBF read error");
			return false;
		}

		// Sanitize the data so it doesn't contain any `0` values.
		for (size_t i = buffer_offset; i < buffer_offset + read_result; i++) {
			if (buf[i] == 0) {
				buf[i] = 1;
			}
		}

		buffer_offset += read_result;

		// Make sure the current buffer is a valid string.
		buf[buffer_offset] = '\0';

		if (strstr(buf, "$R: ") != nullptr) {
			SBF_DEBUG("Response: received");
			return true;
		}

		if (buffer_offset + 1 >= sizeof(buf)) {
			// Copy the last 3 bytes such that a half response isn't lost.
			for (int i = 0; i < 4; i++) {
				buf[i] = buf[sizeof(buf) - 4 + i];
			}
			buffer_offset = 3;
		}
	} while (time_started + 1000 * timeout > hrt_absolute_time());

	SBF_DEBUG("Response: timeout");
	return false;
}

int SeptentrioGPS::receive(unsigned timeout)
{
	int ret = 0;
	int handled = 0;
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	if (!_configured) {
		px4_usleep(timeout * 1000);
		return 0;
	}

	// timeout additional to poll
	hrt_abstime time_started = hrt_absolute_time();

	while (true) {
		// Wait for only SBF_PACKET_TIMEOUT if something already received.
		ret = read(buf, sizeof(buf), handled ? SBF_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			// Something went wrong when polling or reading.
			SBF_WARN("ubx poll_or_read err");
			return -1;

		} else {
			SBF_DEBUG("Read %d bytes (receive)", ret);

			// Pass received bytes to the packet decoder.
			for (int i = 0; i < ret; i++) {
				handled |= parse_char(buf[i]);
				SBF_DEBUG("parsed %d: 0x%x", i, buf[i]);
			}
		}

		if (handled > 0) {
			return handled;
		}

		// abort after timeout if no useful packets received
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			SBF_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int SeptentrioGPS::read(uint8_t *buf, size_t buf_length, int timeout)
{
	int num_read = poll_or_read(buf, buf_length, timeout);

	if (num_read > 0) {
		dump_gps_data(buf, (size_t)num_read, SeptentrioDumpCommMode::Full, false);
	}

	return num_read;
}

int SeptentrioGPS::poll_or_read(uint8_t *buf, size_t buf_length, int timeout)
{
	const size_t character_count = 32; // minimum bytes that we want to read
	const int max_timeout = 50;
	int timeout_adjusted = math::min(max_timeout, timeout);

	handle_inject_data_topic();

	return _uart.readAtLeast(buf, buf_length, math::min(character_count, buf_length), timeout_adjusted);
}

int SeptentrioGPS::write(const uint8_t* buf, size_t buf_length)
{
	dump_gps_data(buf, buf_length, SeptentrioDumpCommMode::Full, true);

	return _uart.write(buf, buf_length);
}

int SeptentrioGPS::initialize_communication_dump()
{
	param_t handle = param_find("SEP_DUMP_COMM");
	int32_t param_dump_comm;

	if (handle == PARAM_INVALID || param_get(handle, &param_dump_comm) != 0) {
		return PX4_ERROR;
	}

	// Check whether dumping is disabled.
	if (param_dump_comm < 1 || param_dump_comm > 2) {
		return PX4_ERROR;
	}

	_dump_from_device = new gps_dump_s();
	_dump_to_device = new gps_dump_s();

	if (!_dump_from_device || !_dump_to_device) {
		PX4_ERR("failed to allocated dump data");
		return PX4_ERROR;
	}

	memset(_dump_to_device, 0, sizeof(gps_dump_s));
	memset(_dump_from_device, 0, sizeof(gps_dump_s));

	// Make sure to use a large enough queue size, so that we don't lose
	// messages. You may also want to increase the logger rate for that.
	_dump_communication_pub.advertise();

	_dump_communication_mode = (SeptentrioDumpCommMode)param_dump_comm;

	return PX4_OK;
}

void SeptentrioGPS::reset_if_scheduled()
{
	SeptentrioGPSResetType reset_type = (SeptentrioGPSResetType)_scheduled_reset.load();

	if (reset_type != SeptentrioGPSResetType::None) {
		_scheduled_reset.store((int)SeptentrioGPSResetType::None);
		int res = reset(reset_type);

		if (res == PX4_OK) {
			PX4_INFO("Reset succeeded.");
		} else {
			PX4_INFO("Reset failed.");
		}
	}
}

int SeptentrioGPS::set_baudrate(uint32_t baud)
{
	if (_uart.setBaudrate(baud)) {
		return PX4_OK;
	} else {
		return PX4_ERROR;
	}
}

void SeptentrioGPS::handle_inject_data_topic()
{
	bool received_new_data = false;
	gps_inject_data_s msg;

	// If there has not been a valid RTCM message for a while, try to switch to a different RTCM link
	if ((hrt_absolute_time() - _last_rtcm_injection_time) > 5_s) {

		for (int instance = 0; instance < _orb_inject_data_sub.size(); instance++) {
			const bool exists = _orb_inject_data_sub[instance].advertised();

			if (exists) {
				if (_orb_inject_data_sub[instance].copy(&msg)) {
					if ((hrt_absolute_time() - msg.timestamp) < 5_s) {
						// Remember that we already copied a message from this instance
						received_new_data = true;
						_selected_rtcm_instance = instance;
						break;
					}
				}
			}
		}
	}

	// Limit maximum number of GPS injections to 8 since usually
	// GPS injections should consist of 1-4 packets (GPS, Glonass, BeiDou, Galileo).
	// Looking at 8 packets thus guarantees, that at least a full injection
	// data set is evaluated.
	// Moving Base requires a higher rate, so we allow up to 8 packets.
	const size_t max_num_injections = gps_inject_data_s::ORB_QUEUE_LENGTH;
	size_t num_injections = 0;

	do {
		if (received_new_data) {
			num_injections++;

			// Prevent injection of RTCM data from self
			// The driver only supports using the secondary receiver as moving base, so this check is enough
			if (_mode != SeptentrioMode::Default && _instance == SeptentrioInstance::Main) {
				// Write the message to the gps device. Note that the message could be fragmented.
				// But as we don't write anywhere else to the device during operation, we don't
				// need to assemble the message first.
				write(msg.data, msg.len);

				++_last_rate_rtcm_injection_count;
				_last_rtcm_injection_time = hrt_absolute_time();
			}
		}

		received_new_data = _orb_inject_data_sub[_selected_rtcm_instance].update(&msg);

	} while (received_new_data && num_injections < max_num_injections);
}

void SeptentrioGPS::publish()
{
	_report_gps_pos.device_id = get_device_id();
	_report_gps_pos.selected_rtcm_instance = _selected_rtcm_instance;
	_report_gps_pos.rtcm_injection_rate = _rate_rtcm_injection;

	_report_gps_pos_pub.publish(_report_gps_pos);

	// Heading/yaw data can be updated at a lower rate than the other navigation data.
	// The uORB message definition requires this data to be set to a NAN if no new valid data is available.
	_report_gps_pos.heading = NAN;

	if (_report_gps_pos.spoofing_state != _spoofing_state) {

		if (_report_gps_pos.spoofing_state > sensor_gps_s::SPOOFING_STATE_NONE) {
			PX4_WARN("GPS spoofing detected! (state: %d)", _report_gps_pos.spoofing_state);
		}

		_spoofing_state = _report_gps_pos.spoofing_state;
	}

	if (_report_gps_pos.jamming_state != _jamming_state) {

		if (_report_gps_pos.jamming_state > sensor_gps_s::JAMMING_STATE_WARNING) {
			PX4_WARN("GPS jamming detected! (state: %d) (indicator: %d)", _report_gps_pos.jamming_state,
					(uint8_t)_report_gps_pos.jamming_indicator);
		}

		_jamming_state = _report_gps_pos.jamming_state;
	}
}

void SeptentrioGPS::publish_satellite_info()
{
	if (_p_report_sat_info != nullptr) {
		_report_sat_info_pub.publish(*_p_report_sat_info);
	}
}

void SeptentrioGPS::publish_rtcm_corrections(uint8_t *data, size_t len)
{
	gps_inject_data_s gps_inject_data{};

	gps_inject_data.timestamp = hrt_absolute_time();
	gps_inject_data.device_id = get_device_id();

	size_t capacity = (sizeof(gps_inject_data.data) / sizeof(gps_inject_data.data[0]));

	if (len > capacity) {
		gps_inject_data.flags = 1; //LSB: 1=fragmented

	} else {
		gps_inject_data.flags = 0;
	}

	size_t written = 0;

	while (written < len) {

		gps_inject_data.len = len - written;

		if (gps_inject_data.len > capacity) {
			gps_inject_data.len = capacity;
		}

		memcpy(gps_inject_data.data, &data[written], gps_inject_data.len);

		_gps_inject_data_pub.publish(gps_inject_data);

		written = written + gps_inject_data.len;
	}
}

void SeptentrioGPS::dump_gps_data(const uint8_t *data, size_t len, SeptentrioDumpCommMode mode, bool msg_to_gps_device)
{
	gps_dump_s *dump_data  = msg_to_gps_device ? _dump_to_device : _dump_from_device;

	if (_dump_communication_mode != mode || !dump_data) {
		return;
	}

	dump_data->instance = (uint8_t)_instance;

	while (len > 0) {
		size_t write_len = len;

		if (write_len > sizeof(dump_data->data) - dump_data->len) {
			write_len = sizeof(dump_data->data) - dump_data->len;
		}

		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= sizeof(dump_data->data)) {
			if (msg_to_gps_device) {
				dump_data->len |= 1 << 7;
			}

			dump_data->timestamp = hrt_absolute_time();
			_dump_communication_pub.publish(*dump_data);
			dump_data->len = 0;
		}
	}
}

void SeptentrioGPS::got_rtcm_message(uint8_t *data, size_t len)
{
	publish_rtcm_corrections(data, len);
	dump_gps_data(data, len, SeptentrioDumpCommMode::RTCM, false);
}

void SeptentrioGPS::store_update_rates()
{
	_rate_vel = _rate_count_vel / (((float)(hrt_absolute_time() - _interval_rate_start)) / 1000000.0f);
	_rate_lat_lon = _rate_count_lat_lon / (((float)(hrt_absolute_time() - _interval_rate_start)) / 1000000.0f);
}

void SeptentrioGPS::reset_update_rates()
{
	_rate_count_vel = 0;
	_rate_count_lat_lon = 0;
	_interval_rate_start = hrt_absolute_time();
}

void SeptentrioGPS::set_clock(timespec rtc_gps_time)
{
	timespec rtc_system_time;
	px4_clock_gettime(CLOCK_REALTIME, &rtc_system_time);
	int drift_time = abs(rtc_system_time.tv_sec - rtc_gps_time.tv_sec);

	// As of 2021 setting the time on Nuttx temporarily pauses interrupts so only set the time if it is very wrong.
	if (drift_time >= SEP_SET_CLOCK_DRIFT_TIME_S) {
		// TODO: clock slewing of the RTC for small time differences
		px4_clock_settime(CLOCK_REALTIME, &rtc_gps_time);
	}
}

extern "C" __EXPORT int septentrio_main(int argc, char *argv[])
{
	return SeptentrioGPS::main(argc, argv);
}

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
#include <cstdint>
#include <ctime>
#include <string.h>
#include <termios.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/sensor_gps.h>

#include "util.h"
#include "sbf/messages.h"

using namespace device;

namespace septentrio
{

/**
 * RTC drift time when time synchronization is needed (in seconds).
*/
constexpr int k_max_allowed_clock_drift = 5;

/**
 * If silence from the receiver for this time (ms), assume full data received.
*/
constexpr int k_receiver_read_timeout = 2;

/**
 * The maximum allowed time for reading from the receiver.
 */
constexpr int k_max_receiver_read_timeout = 50;

/**
 * Minimum amount of bytes we try to read at one time from the receiver.
*/
constexpr size_t k_min_receiver_read_bytes = 32;

/**
 * The baud rate of Septentrio receivers with factory default configuration.
*/
constexpr uint32_t k_septentrio_receiver_default_baud_rate = 115200;

constexpr uint8_t k_max_command_size            = 120;
constexpr uint16_t k_timeout_5hz                = 500;
constexpr uint32_t k_read_buffer_size           = 150;
constexpr time_t k_gps_epoch_secs               = 1234567890ULL; // TODO: This seems wrong

// Septentrio receiver commands
// - erst: exeResetReceiver
// - sso: setSBFOutput
// - ssu: setSatelliteUsage
// - scs: setCOMSettings
// - srd: setReceiverDynamics
// - sto: setAttitudeOffset
// - sdio: setDataInOut
// - gecm: getEchoMessage
// - sga: setGNSSAttitude
constexpr const char *k_command_force_input = "SSSSSSSSSS\n";
constexpr const char *k_command_reset_hot = "erst,soft,none\n";
constexpr const char *k_command_reset_warm = "erst,soft,PVTData\n";
constexpr const char *k_command_reset_cold = "erst,hard,SatData\n";
constexpr const char *k_command_sbf_output_pvt =
	"sso,Stream%lu,%s,PVTGeodetic+VelCovGeodetic+DOP+AttEuler+AttCovEuler+EndOfPVT+ReceiverStatus,%s\n";
constexpr const char *k_command_set_sbf_output =
	"sso,Stream%lu,%s,%s%s,%s\n";
constexpr const char *k_command_clear_sbf = "sso,Stream%lu,%s,none,off\n";
constexpr const char *k_command_set_baud_rate =
	"scs,%s,baud%lu\n"; // The receiver sends the reply at the new baud rate!
constexpr const char *k_command_set_dynamics = "srd,%s,UAV\n";
constexpr const char *k_command_set_attitude_offset = "sto,%.3f,%.3f\n";
constexpr const char *k_command_set_data_io = "sdio,%s,Auto,%s\n";
constexpr const char *k_command_set_satellite_usage = "ssu,%s\n";
constexpr const char *k_command_ping = "gecm\n";
constexpr const char *k_command_set_gnss_attitude = "sga,%s\n";

constexpr const char *k_gnss_attitude_source_moving_base = "MovingBase";
constexpr const char *k_gnss_attitude_source_multi_antenna = "MultiAntenna";

constexpr const char *k_frequency_0_1hz  = "sec10";
constexpr const char *k_frequency_0_2hz  = "sec5";
constexpr const char *k_frequency_0_5hz  = "sec2";
constexpr const char *k_frequency_1_0hz  = "sec1";
constexpr const char *k_frequency_2_0hz  = "msec500";
constexpr const char *k_frequency_5_0hz  = "msec200";
constexpr const char *k_frequency_10_0hz = "msec100";
constexpr const char *k_frequency_20_0hz = "msec50";
constexpr const char *k_frequency_25_0hz = "msec40";
constexpr const char *k_frequency_50_0hz = "msec20";

px4::atomic<SeptentrioDriver *> SeptentrioDriver::_secondary_instance {nullptr};
uint32_t SeptentrioDriver::k_supported_baud_rates[] {0, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1500000};
uint32_t SeptentrioDriver::k_default_baud_rate {230400};
orb_advert_t SeptentrioDriver::k_mavlink_log_pub {nullptr};

SeptentrioDriver::SeptentrioDriver(const char *device_path, Instance instance, uint32_t baud_rate) :
	Device(MODULE_NAME),
	_instance(instance),
	_chosen_baud_rate(baud_rate)
{
	strncpy(_port, device_path, sizeof(_port) - 1);
	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	int32_t enable_sat_info {0};
	get_parameter("SEP_SAT_INFO", &enable_sat_info);

	if (enable_sat_info) {
		_message_satellite_info = new satellite_info_s();
	}

	get_parameter("SEP_YAW_OFFS", &_heading_offset);
	get_parameter("SEP_PITCH_OFFS", &_pitch_offset);

	int32_t dump_mode {0};
	get_parameter("SEP_DUMP_COMM", &dump_mode);
	DumpMode mode = static_cast<DumpMode>(dump_mode);

	if (mode != DumpMode::Disabled) {
		initialize_communication_dump(mode);
	}

	int32_t receiver_stream_main {k_default_main_stream};
	get_parameter("SEP_STREAM_MAIN", &receiver_stream_main);
	_receiver_stream_main = receiver_stream_main;
	int32_t receiver_stream_log {k_default_log_stream};
	get_parameter("SEP_STREAM_LOG", &receiver_stream_log);
	_receiver_stream_log = receiver_stream_log;

	if (_receiver_stream_log == _receiver_stream_main) {
		mavlink_log_warning(&k_mavlink_log_pub, "Septentrio: Logging stream should be different from main stream");
	}

	int32_t automatic_configuration {true};
	get_parameter("SEP_AUTO_CONFIG", &automatic_configuration);
	_automatic_configuration = static_cast<bool>(automatic_configuration);

	get_parameter("SEP_CONST_USAGE", &_receiver_constellation_usage);

	int32_t logging_frequency {static_cast<int32_t>(ReceiverLogFrequency::Hz1_0)};
	get_parameter("SEP_LOG_HZ", &logging_frequency);
	_receiver_logging_frequency = static_cast<ReceiverLogFrequency>(logging_frequency);
	int32_t logging_level {static_cast<int32_t>(ReceiverLogLevel::Default)};
	get_parameter("SEP_LOG_LEVEL", &logging_level);
	_receiver_logging_level = static_cast<ReceiverLogLevel>(logging_level);
	int32_t logging_overwrite {false};
	get_parameter("SEP_LOG_FORCE", &logging_overwrite);
	_receiver_logging_overwrite = logging_overwrite;
	int32_t receiver_setup {static_cast<int32_t>(ReceiverSetup::Default)};
	get_parameter("SEP_HARDW_SETUP", &receiver_setup);
	_receiver_setup = static_cast<ReceiverSetup>(receiver_setup);
	int32_t sbf_output_frequency {static_cast<int32_t>(SBFOutputFrequency::Hz5_0)};
	get_parameter("SEP_OUTP_HZ", &sbf_output_frequency);
	_sbf_output_frequency = static_cast<SBFOutputFrequency>(sbf_output_frequency);

	if (_instance == Instance::Secondary && _receiver_setup == ReceiverSetup::MovingBase) {
		_rtcm_decoder = new rtcm::Decoder();
	}

	set_device_type(DRV_GPS_DEVTYPE_SBF);

	reset_gps_state_message();
}

SeptentrioDriver::~SeptentrioDriver()
{
	if (_instance == Instance::Main) {
		if (await_second_instance_shutdown() == PX4_ERROR) {
			SEP_ERR("Secondary instance shutdown timed out");
		}
	}

	if (_message_data_from_receiver) {
		delete _message_data_from_receiver;
	}

	if (_message_data_to_receiver) {
		delete _message_data_to_receiver;
	}

	if (_message_satellite_info) {
		delete _message_satellite_info;
	}

	if (_rtcm_decoder) {
		delete _rtcm_decoder;
	}
}

int SeptentrioDriver::print_status()
{
	SeptentrioDriver *secondary_instance = _secondary_instance.load();

	switch (_instance) {
	case Instance::Main:
		PX4_INFO("Main GPS");
		break;

	case Instance::Secondary:
		PX4_INFO("");
		PX4_INFO("Secondary GPS");
		break;
	}

	PX4_INFO("health: %s, port: %s, baud rate: %lu", is_healthy() ? "OK" : "NOT OK", _port, _uart.getBaudrate());
	PX4_INFO("controller -> receiver data rate: %lu B/s", output_data_rate());
	PX4_INFO("receiver -> controller data rate: %lu B/s", input_data_rate());
	PX4_INFO("sat info: %s", (_message_satellite_info != nullptr) ? "enabled" : "disabled");

	if (first_gps_uorb_message_created() && _state == State::ReceivingData) {
		PX4_INFO("rate RTCM injection: %6.2f Hz", static_cast<double>(rtcm_injection_frequency()));
		print_message(ORB_ID(sensor_gps), _message_gps_state);
	}

	if (_instance == Instance::Main && secondary_instance) {
		secondary_instance->print_status();
	}

	return 0;
}

void SeptentrioDriver::run()
{
	while (!should_exit()) {
		switch (_state) {
		case State::OpeningSerialPort: {
				_uart.setPort(_port);

				if (_uart.open()) {
					_state = State::DetectingBaudRate;

				} else {
					// Failed to open port, so wait a bit before trying again.
					px4_usleep(200000);
				}

				break;
			}

		case State::DetectingBaudRate: {
				static uint32_t expected_baud_rates[] = {k_septentrio_receiver_default_baud_rate, 115200, 230400, 57600, 460800, 500000, 576000, 38400, 921600, 1000000, 1500000};
				expected_baud_rates[0] = _chosen_baud_rate != 0 ? _chosen_baud_rate : k_septentrio_receiver_default_baud_rate;

				if (detect_receiver_baud_rate(expected_baud_rates[_current_baud_rate_index], true)) {
					if (set_baudrate(expected_baud_rates[_current_baud_rate_index]) == PX4_OK) {
						_state = State::ConfiguringDevice;

					} else {
						SEP_ERR("Setting local baud rate failed");
					}

				} else {
					_current_baud_rate_index++;

					if (_current_baud_rate_index == sizeof(expected_baud_rates) / sizeof(expected_baud_rates[0])) {
						_current_baud_rate_index = 0;
					}
				}

				break;
			}

		case State::ConfiguringDevice: {
				ConfigureResult result = configure();

				if (!(static_cast<int32_t>(result) & static_cast<int32_t>(ConfigureResult::FailedCompletely))) {
					if (static_cast<int32_t>(result) & static_cast<int32_t>(ConfigureResult::NoLogging)) {
						mavlink_log_warning(&k_mavlink_log_pub, "Septentrio: Failed to configure receiver internal logging");
					}

					SEP_INFO("Automatic configuration finished");
					_state = State::ReceivingData;

				} else {
					_state = State::DetectingBaudRate;
				}

				break;
			}

		case State::ReceivingData: {
				int receive_result {0};

				receive_result = receive(k_timeout_5hz);

				if (receive_result == -1) {
					_state = State::DetectingBaudRate;
				}

				if (_message_satellite_info && (receive_result & 2)) {
					publish_satellite_info();
				}

				break;
			}

		}

		reset_if_scheduled();

		handle_inject_data_topic();

		if (update_monitoring_interval_ended()) {
			start_update_monitoring_interval();
		}
	}

}

int SeptentrioDriver::task_spawn(int argc, char *argv[])
{
	return task_spawn(argc, argv, Instance::Main);
}

int SeptentrioDriver::task_spawn(int argc, char *argv[], Instance instance)
{
	px4_main_t entry_point;
	static constexpr int k_task_stack_size = PX4_STACK_ADJUSTED(2048);

	if (instance == Instance::Main) {
		entry_point = &run_trampoline;

	} else {
		entry_point = &run_trampoline_secondary;
	}

	px4_task_t task_id = px4_task_spawn_cmd("septentrio",
						SCHED_DEFAULT,
						SCHED_PRIORITY_SLOW_DRIVER,
						k_task_stack_size,
						entry_point,
						(char *const *)argv);

	if (task_id < 0) {
		// `_task_id` of module that hasn't been started before or has been stopped should already be -1.
		// This is just to make sure.
		_task_id = -1;
		return -errno;
	}

	if (instance == Instance::Main) {
		_task_id = task_id;
	}

	return 0;
}

int SeptentrioDriver::run_trampoline_secondary(int argc, char *argv[])
{
	// Get rid of the task name (first argument)
	argc -= 1;
	argv += 1;

	SeptentrioDriver *gps = instantiate(argc, argv, Instance::Secondary);

	if (gps) {
		_secondary_instance.store(gps);
		gps->run();
		_secondary_instance.store(nullptr);
		delete gps;

	} else {
		return -1;
	}

	return 0;
}

SeptentrioDriver *SeptentrioDriver::instantiate(int argc, char *argv[])
{
	return instantiate(argc, argv, Instance::Main);
}

SeptentrioDriver *SeptentrioDriver::instantiate(int argc, char *argv[], Instance instance)
{
	ModuleArguments arguments {};
	SeptentrioDriver *gps {nullptr};

	if (parse_cli_arguments(argc, argv, arguments) == PX4_ERROR) {
		return nullptr;
	}

	if (arguments.device_path_main && arguments.device_path_secondary
	    && strcmp(arguments.device_path_main, arguments.device_path_secondary) == 0) {
		mavlink_log_critical(&k_mavlink_log_pub, "Septentrio: Device paths must be different");
		return nullptr;
	}

	bool valid_chosen_baud_rate {false};

	for (uint8_t i = 0; i < sizeof(k_supported_baud_rates) / sizeof(k_supported_baud_rates[0]); i++) {
		switch (instance) {
		case Instance::Main:
			if (arguments.baud_rate_main == static_cast<int>(k_supported_baud_rates[i])) {
				valid_chosen_baud_rate = true;
			}

			break;

		case Instance::Secondary:
			if (arguments.baud_rate_secondary == static_cast<int>(k_supported_baud_rates[i])) {
				valid_chosen_baud_rate = true;
			}

			break;
		}
	}

	if (!valid_chosen_baud_rate) {
		mavlink_log_critical(&k_mavlink_log_pub, "Septentrio: Baud rate %d is unsupported, falling back to default %lu",
				     instance == Instance::Main ? arguments.baud_rate_main : arguments.baud_rate_secondary, k_default_baud_rate);
	}

	if (instance == Instance::Main) {
		if (Serial::validatePort(arguments.device_path_main)) {
			gps = new SeptentrioDriver(arguments.device_path_main, instance,
						   valid_chosen_baud_rate ? arguments.baud_rate_main : k_default_baud_rate);

		} else {
			PX4_ERR("Invalid device (-d) %s", arguments.device_path_main ? arguments.device_path_main : "");
		}

		if (gps && arguments.device_path_secondary) {
			task_spawn(argc, argv, Instance::Secondary);

			if (await_second_instance_startup() == PX4_ERROR) {
				return nullptr;
			}
		}

	} else {
		if (Serial::validatePort(arguments.device_path_secondary)) {
			gps = new SeptentrioDriver(arguments.device_path_secondary, instance,
						   valid_chosen_baud_rate ? arguments.baud_rate_secondary : k_default_baud_rate);

		} else {
			PX4_ERR("Invalid secondary device (-e) %s", arguments.device_path_secondary ? arguments.device_path_secondary : "");
		}
	}

	return gps;
}

// Called from outside driver thread.
// Return 0 on success, -1 otherwise.
int SeptentrioDriver::custom_command(int argc, char *argv[])
{
	bool handled = false;
	const char *failure_reason {"unknown command"};
	SeptentrioDriver *driver_instance;

	if (!is_running()) {
		PX4_INFO("not running");
		return -1;
	}

	driver_instance = get_instance();

	if (argc >= 1 && strcmp(argv[0], "reset") == 0) {
		if (argc == 2) {
			ReceiverResetType type{ReceiverResetType::None};

			if (strcmp(argv[1], "hot") == 0) {
				type = ReceiverResetType::Hot;

			} else if (strcmp(argv[1], "warm") == 0) {
				type = ReceiverResetType::Warm;

			} else if (strcmp(argv[1], "cold") == 0) {
				type = ReceiverResetType::Cold;

			} else {
				failure_reason = "unknown reset type";
			}

			if (type != ReceiverResetType::None) {
				driver_instance->schedule_reset(type);
				handled = true;
			}

		} else {
			failure_reason = "incorrect usage of reset command";
		}
	}

	return handled ? 0 : print_usage(failure_reason);
}

int SeptentrioDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for Septentrio GNSS receivers.
It can automatically configure them and make their output available for the rest of the system.
A secondary receiver is supported for redundancy, logging and dual-receiver heading.
Septentrio receiver baud rates from 57600 to 1500000 are supported.
If others are used, the driver will use 230400 and give a warning.

### Examples

Use one receiver on port `/dev/ttyS0` and automatically configure it to use baud rate 230400:
$ septentrio start -d /dev/ttyS0 -b 230400

Use two receivers, the primary on port `/dev/ttyS3` and the secondary on `/dev/ttyS4`,
detect baud rate automatically and preserve them:
$ septentrio start -d /dev/ttyS3 -e /dev/ttyS4

Perform warm reset of the receivers:
$ gps reset warm
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("septentrio", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Primary receiver port", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 57600, 1500000, "Primary receiver baud rate", true);
	PRINT_MODULE_USAGE_PARAM_STRING('e', nullptr, "<file:dev>", "Secondary receiver port", true);
	PRINT_MODULE_USAGE_PARAM_INT('g', 0, 57600, 1500000, "Secondary receiver baud rate", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset connected receiver");
	PRINT_MODULE_USAGE_ARG("cold|warm|hot", "Specify reset type", false);

	return 0;
}

int SeptentrioDriver::reset(ReceiverResetType type)
{
	bool res = false;

	force_input();

	switch (type) {
	case ReceiverResetType::Hot:
		res = send_message_and_wait_for_ack(k_command_reset_hot, k_receiver_ack_timeout_fast);
		break;

	case ReceiverResetType::Warm:
		res = send_message_and_wait_for_ack(k_command_reset_warm, k_receiver_ack_timeout_fast);
		break;

	case ReceiverResetType::Cold:
		res = send_message_and_wait_for_ack(k_command_reset_cold, k_receiver_ack_timeout_fast);
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

int SeptentrioDriver::force_input()
{
	ssize_t written = write(reinterpret_cast<const uint8_t*>(k_command_force_input), strlen(k_command_force_input));

	if (written < 0) {
		return PX4_ERROR;
	} else {
		// The receiver can't receive input right after forcing input. From testing, the duration seems to be 1 ms, so wait 10 ms to be sure.
		px4_usleep(10000);
		return PX4_OK;
	}
}

int SeptentrioDriver::parse_cli_arguments(int argc, char *argv[], ModuleArguments& arguments)
{
	int ch{'\0'};
	int myoptind{1};
	const char *myoptarg{nullptr};

	while ((ch = px4_getopt(argc, argv, "d:e:b:g:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, arguments.baud_rate_main) != 0) {
				PX4_ERR("Baud rate parsing failed");
				return PX4_ERROR;
			}
			break;
		case 'g':
			if (px4_get_parameter_value(myoptarg, arguments.baud_rate_secondary) != 0) {
				PX4_ERR("Baud rate parsing failed");
				return PX4_ERROR;
			}
			break;
		case 'd':
			arguments.device_path_main = myoptarg;
			break;

		case 'e':
			arguments.device_path_secondary = myoptarg;
			break;

		case '?':
			return PX4_ERROR;

		default:
			PX4_WARN("unrecognized flag");
			return PX4_ERROR;
			break;
		}
	}

	return PX4_OK;
}

int SeptentrioDriver::await_second_instance_startup()
{
	uint32_t i = 0;

	do {
		px4_usleep(2500);
	} while (!_secondary_instance.load() && ++i < 400);

	if (!_secondary_instance.load()) {
		SEP_ERR("Timed out while waiting for second instance to start");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int SeptentrioDriver::await_second_instance_shutdown()
{
	if (_instance == Instance::Secondary) {
		return PX4_OK;
	}

	SeptentrioDriver* secondary_instance = _secondary_instance.load();

	if (secondary_instance) {
		secondary_instance->request_stop();

		uint32_t i {0};

		// Give the secondary instance 2 seconds at most to properly shut down.
		while (_secondary_instance.load() && i < 100) {
			px4_usleep(20000);

			++i;
		}

		return _secondary_instance.load() ? PX4_ERROR : PX4_OK;
	} else {
		return PX4_OK;
	}
}

void SeptentrioDriver::schedule_reset(ReceiverResetType reset_type)
{
	SeptentrioDriver *secondary_instance = _secondary_instance.load();

	_scheduled_reset.store((int)reset_type);

	if (_instance == Instance::Main && secondary_instance) {
		secondary_instance->schedule_reset(reset_type);
	}
}

bool SeptentrioDriver::detect_receiver_baud_rate(const uint32_t &baud_rate, bool forced_input) {
	if (set_baudrate(baud_rate) != PX4_OK) {
		return false;
	}

	if (forced_input) {
		force_input();
	}

	// Make sure that any weird data is "flushed" in the receiver.
	(void)send_message("\n");

	if (send_message_and_wait_for_ack(k_command_ping, k_receiver_ack_timeout_fast)) {
		SEP_INFO("Detected baud rate: %lu", baud_rate);
		return true;
	}

	return false;
}

int SeptentrioDriver::detect_serial_port(char* const port_name) {
	// Read buffer to get the COM port
	char buf[k_read_buffer_size];
	size_t buffer_offset = 0;   // The offset into the string where the next data should be read to.
	hrt_abstime timeout_time = hrt_absolute_time() + 5 * 1000 * k_receiver_ack_timeout_fast;
	bool response_detected = false;

	// Receiver prints prompt after a message.
	if (!send_message(k_command_ping)) {
		return PX4_ERROR;
	}

	do {
		// Read at most the amount of available bytes in the buffer after the current offset, -1 because we need '\0' at the end for a valid string.
		int read_result = read(reinterpret_cast<uint8_t *>(buf) + buffer_offset, sizeof(buf) - buffer_offset - 1, k_receiver_ack_timeout_fast);

		if (read_result < 0) {
			SEP_WARN("SBF read error");
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
	} while (timeout_time > hrt_absolute_time());

	if (!response_detected) {
		SEP_WARN("No valid serial port detected");
		return PX4_ERROR;
	} else {
		SEP_INFO("Serial port found: %s", port_name);
		return PX4_OK;
	}
}

SeptentrioDriver::ConfigureResult SeptentrioDriver::configure()
{
	char msg[k_max_command_size] {};
	char com_port[5] {};
	ConfigureResult result {ConfigureResult::OK};

	// Passively detect receiver port.
	if (detect_serial_port(com_port) != PX4_OK) {
		SEP_WARN("CONFIG: failed port detection");
		return ConfigureResult::FailedCompletely;
	}

	// We should definitely match baud rates and detect used port, but don't do other configuration if not requested.
	// This will force input on the receiver. That shouldn't be a problem as it's on our own connection.
	if (!_automatic_configuration) {
		return ConfigureResult::OK;
	}

	// If user requested specific baud rate, set it now. Otherwise keep detected baud rate.
	if (strstr(com_port, "COM") != nullptr && _chosen_baud_rate != 0) {
		snprintf(msg, sizeof(msg), k_command_set_baud_rate, com_port, _chosen_baud_rate);

		if (!send_message(msg)) {
			SEP_WARN("CONFIG: baud rate command write error");
			return ConfigureResult::FailedCompletely;
		}

		// When sending a command and setting the baud rate right after, the controller could send the command at the new baud rate.
		// From testing this could take some time.
		px4_usleep(2000000);

		if (set_baudrate(_chosen_baud_rate) != PX4_OK) {
			SEP_WARN("CONFIG: failed local baud rate setting");
			return ConfigureResult::FailedCompletely;
		}

		if (!send_message_and_wait_for_ack(k_command_ping, k_receiver_ack_timeout_fast)) {
			SEP_WARN("CONFIG: failed remote baud rate setting");
			return ConfigureResult::FailedCompletely;
		}
	}

	// Delete all sbf outputs on current COM port to remove clutter data
	snprintf(msg, sizeof(msg), k_command_clear_sbf, _receiver_stream_main, com_port);

	if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
		SEP_WARN("CONFIG: failed delete output");
		return ConfigureResult::FailedCompletely; // connection and/or baudrate detection failed
	}

	// Set up the satellites used in PVT computation.
	if (_receiver_constellation_usage != static_cast<int32_t>(SatelliteUsage::Default)) {
		char requested_satellites[40] {};
		if (_receiver_constellation_usage & static_cast<int32_t>(SatelliteUsage::GPS)) {
			strcat(requested_satellites, "GPS+");
		}
		if (_receiver_constellation_usage & static_cast<int32_t>(SatelliteUsage::GLONASS)) {
			strcat(requested_satellites, "GLONASS+");
		}
		if (_receiver_constellation_usage & static_cast<int32_t>(SatelliteUsage::Galileo)) {
			strcat(requested_satellites, "GALILEO+");
		}
		if (_receiver_constellation_usage & static_cast<int32_t>(SatelliteUsage::SBAS)) {
			strcat(requested_satellites, "SBAS+");
		}
		if (_receiver_constellation_usage & static_cast<int32_t>(SatelliteUsage::BeiDou)) {
			strcat(requested_satellites, "BEIDOU+");
		}
		// Make sure to remove the trailing '+' if any.
		requested_satellites[math::max(static_cast<int>(strlen(requested_satellites)) - 1, 0)] = '\0';
		snprintf(msg, sizeof(msg), k_command_set_satellite_usage, requested_satellites);
		// Use a longer timeout as the `setSatelliteUsage` command acknowledges a bit slower on mosaic-H-based receivers.
		if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_slow)) {
			SEP_WARN("CONFIG: Failed to configure constellation usage");
			return ConfigureResult::FailedCompletely;
		}
	}

	// Internal logging on the receiver.
	if (_receiver_logging_frequency != ReceiverLogFrequency::Disabled && _receiver_stream_log != _receiver_stream_main) {
		const char *frequency {nullptr};
		const char *level {nullptr};

		switch (_receiver_logging_frequency) {
		case ReceiverLogFrequency::Hz0_1:
			frequency = k_frequency_0_1hz;
			break;
		case ReceiverLogFrequency::Hz0_2:
			frequency = k_frequency_0_2hz;
			break;
		case ReceiverLogFrequency::Hz0_5:
			frequency = k_frequency_0_5hz;
			break;
		case ReceiverLogFrequency::Hz1_0:
		default:
			frequency = k_frequency_1_0hz;
			break;
		case ReceiverLogFrequency::Hz2_0:
			frequency = k_frequency_2_0hz;
			break;
		case ReceiverLogFrequency::Hz5_0:
			frequency = k_frequency_5_0hz;
			break;
		case ReceiverLogFrequency::Hz10_0:
			frequency = k_frequency_10_0hz;
			break;
		case ReceiverLogFrequency::Hz20_0:
			frequency = k_frequency_20_0hz;
			break;
		case ReceiverLogFrequency::Hz25_0:
			frequency = k_frequency_25_0hz;
			break;
		case ReceiverLogFrequency::Hz50_0:
			frequency = k_frequency_50_0hz;
			break;
		}

		switch (_receiver_logging_level) {
		case ReceiverLogLevel::Lite:
			level = "Comment+ReceiverStatus";
			break;
		case ReceiverLogLevel::Basic:
			level = "Comment+ReceiverStatus+PostProcess+Event";
			break;
		case ReceiverLogLevel::Default:
		default:
			level = "Comment+ReceiverStatus+PostProcess+Event+Support";
			break;
		case ReceiverLogLevel::Full:
			level = "Comment+ReceiverStatus+PostProcess+Event+Support+BBSamples";
			break;
		}

		snprintf(msg, sizeof(msg), k_command_set_sbf_output, _receiver_stream_log, "DSK1", _receiver_logging_overwrite ? "" : "+", level, frequency);
		if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
			result = static_cast<ConfigureResult>(static_cast<int32_t>(result) | static_cast<int32_t>(ConfigureResult::NoLogging));
		}
	} else if (_receiver_stream_log == _receiver_stream_main) {
		result = static_cast<ConfigureResult>(static_cast<int32_t>(result) | static_cast<int32_t>(ConfigureResult::NoLogging));
	}

	// Define/inquire the type of data that the receiver should accept/send on a given connection descriptor
	snprintf(msg, sizeof(msg), k_command_set_data_io, com_port, "SBF");

	if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
		return ConfigureResult::FailedCompletely;
	}

	// Specify the offsets that the receiver applies to the computed attitude angles.
	snprintf(msg, sizeof(msg), k_command_set_attitude_offset, static_cast<double>(_heading_offset), static_cast<double>(_pitch_offset));

	if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
		return ConfigureResult::FailedCompletely;
	}

	snprintf(msg, sizeof(msg), k_command_set_dynamics, "high");
	if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
		return ConfigureResult::FailedCompletely;
	}

	const char *sbf_frequency {k_frequency_10_0hz};
	switch (_sbf_output_frequency) {
	case SBFOutputFrequency::Hz5_0:
		sbf_frequency = k_frequency_5_0hz;
		break;
	case SBFOutputFrequency::Hz10_0:
		sbf_frequency = k_frequency_10_0hz;
		break;
	case SBFOutputFrequency::Hz20_0:
		sbf_frequency = k_frequency_20_0hz;
		break;
	case SBFOutputFrequency::Hz25_0:
		sbf_frequency = k_frequency_25_0hz;
		break;
	}

	// Output a set of SBF blocks on a given connection at a regular interval.
	snprintf(msg, sizeof(msg), k_command_sbf_output_pvt, _receiver_stream_main, com_port, sbf_frequency);
	if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
		SEP_WARN("CONFIG: Failed to configure SBF");
		return ConfigureResult::FailedCompletely;
	}

	if (_receiver_setup == ReceiverSetup::MovingBase) {
		if (_instance == Instance::Secondary) {
			snprintf(msg, sizeof(msg), k_command_set_data_io, com_port, "+RTCMv3");
			if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
				SEP_WARN("CONFIG: Failed to configure RTCM output");
			}
		} else {
			snprintf(msg, sizeof(msg), k_command_set_gnss_attitude, k_gnss_attitude_source_moving_base);
			if (!send_message_and_wait_for_ack(msg, k_receiver_ack_timeout_fast)) {
				SEP_WARN("CONFIG: Failed to configure attitude source");
			}
		}
	} else {
		snprintf(msg, sizeof(msg), k_command_set_gnss_attitude, k_gnss_attitude_source_multi_antenna);
		// This fails on single-antenna receivers, which is fine. Therefore don't check for acknowledgement.
		if (!send_message(msg)) {
			SEP_WARN("CONFIG: Failed to configure attitude source");
			return ConfigureResult::FailedCompletely;
		}
	}

	return result;
}

int SeptentrioDriver::parse_char(const uint8_t byte)
{
	int result = 0;

	switch (_active_decoder) {
	case DecodingStatus::Searching:
		if (_sbf_decoder.add_byte(byte) != sbf::Decoder::State::SearchingSync1) {
			_active_decoder = DecodingStatus::SBF;
		} else if (_rtcm_decoder && _rtcm_decoder->add_byte(byte) != rtcm::Decoder::State::SearchingPreamble) {
			_active_decoder = DecodingStatus::RTCMv3;
		}
		break;
	case DecodingStatus::SBF:
		if (_sbf_decoder.add_byte(byte) == sbf::Decoder::State::Done) {
			if (process_message() == PX4_OK) {
				result = 1;
			}
			_sbf_decoder.reset();
			_active_decoder = DecodingStatus::Searching;
		}
		break;
	case DecodingStatus::RTCMv3:
		if (_rtcm_decoder->add_byte(byte) == rtcm::Decoder::State::Done) {
			if (process_message() == PX4_OK) {
				result = 1;
			}
			_rtcm_decoder->reset();
			_active_decoder = DecodingStatus::Searching;
		}
		break;
	}

	return result;
}

int SeptentrioDriver::process_message()
{
	int result = PX4_ERROR;

	switch (_active_decoder) {
	case DecodingStatus::Searching: {
		SEP_ERR("Can't process incomplete message!");
		result = PX4_ERROR;
		break;
	}
	case DecodingStatus::SBF: {
		using namespace sbf;

		switch (_sbf_decoder.id()) {
		case BlockID::Invalid: {
			SEP_TRACE_PARSING("Tried to process invalid SBF message");
			break;
		}
		case BlockID::DOP: {
			SEP_TRACE_PARSING("Processing DOP SBF message");
			_current_interval_messages.dop = true;

			DOP dop;

			if (_sbf_decoder.parse(&dop) == PX4_OK) {
				_message_gps_state.hdop = dop.h_dop * 0.01f;
				_message_gps_state.vdop = dop.v_dop * 0.01f;
				result = PX4_OK;
			}

			break;
		}
		case BlockID::PVTGeodetic: {
			using ModeType = PVTGeodetic::ModeType;
			using Error = PVTGeodetic::Error;

			SEP_TRACE_PARSING("Processing PVTGeodetic SBF message");
			_current_interval_messages.pvt_geodetic = true;

			Header header;
			PVTGeodetic pvt_geodetic;

			if (_sbf_decoder.parse(&header) == PX4_OK && _sbf_decoder.parse(&pvt_geodetic) == PX4_OK) {
				switch (pvt_geodetic.mode_type) {
				case ModeType::NoPVT:
					_message_gps_state.fix_type = sensor_gps_s::FIX_TYPE_NONE;
					break;
				case ModeType::PVTWithSBAS:
					_message_gps_state.fix_type = sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL;
					break;
				case ModeType::RTKFloat:
				case ModeType::MovingBaseRTKFloat:
					_message_gps_state.fix_type = sensor_gps_s::FIX_TYPE_RTK_FLOAT;
					break;
				case ModeType::RTKFixed:
				case ModeType::MovingBaseRTKFixed:
					_message_gps_state.fix_type = sensor_gps_s::FIX_TYPE_RTK_FIXED;
					break;
				default:
					_message_gps_state.fix_type = sensor_gps_s::FIX_TYPE_3D;
					break;
				}

				// Check boundaries and invalidate GPS velocities
				if (pvt_geodetic.vn <= k_dnu_f4_value || pvt_geodetic.ve <= k_dnu_f4_value || pvt_geodetic.vu <= k_dnu_f4_value) {
					_message_gps_state.vel_ned_valid = false;
				}

				if (pvt_geodetic.latitude > k_dnu_f8_value && pvt_geodetic.longitude > k_dnu_f8_value && pvt_geodetic.height > k_dnu_f8_value && pvt_geodetic.undulation > k_dnu_f4_value) {
					_message_gps_state.latitude_deg = pvt_geodetic.latitude * M_RAD_TO_DEG;
					_message_gps_state.longitude_deg = pvt_geodetic.longitude * M_RAD_TO_DEG;
					_message_gps_state.altitude_msl_m = pvt_geodetic.height - static_cast<double>(pvt_geodetic.undulation);
					_message_gps_state.altitude_ellipsoid_m = pvt_geodetic.height;
				} else {
					_message_gps_state.fix_type = sensor_gps_s::FIX_TYPE_NONE;
				}

				if (pvt_geodetic.nr_sv != PVTGeodetic::k_dnu_nr_sv) {
					_message_gps_state.satellites_used = pvt_geodetic.nr_sv;

					if (_message_satellite_info) {
						// Only fill in the satellite count for now (we could use the ChannelStatus message for the
						// other data, but it's really large: >800B)
						_message_satellite_info->timestamp = hrt_absolute_time();
						_message_satellite_info->count = _message_gps_state.satellites_used;
					}

				} else {
					_message_gps_state.satellites_used = 0;
				}

				/* H and V accuracy are reported in 2DRMS, but based off the u-blox reporting we expect RMS.
				 * Divide by 100 from cm to m and in addition divide by 2 to get RMS. */
				_message_gps_state.eph = static_cast<float>(pvt_geodetic.h_accuracy) / 200.0f;
				_message_gps_state.epv = static_cast<float>(pvt_geodetic.v_accuracy) / 200.0f;

				// Check fix and error code
				_message_gps_state.vel_ned_valid = _message_gps_state.fix_type > sensor_gps_s::FIX_TYPE_NONE && pvt_geodetic.error == Error::None;
				_message_gps_state.vel_n_m_s = pvt_geodetic.vn;
				_message_gps_state.vel_e_m_s = pvt_geodetic.ve;
				_message_gps_state.vel_d_m_s = -1.0f * pvt_geodetic.vu;
				_message_gps_state.vel_m_s = sqrtf(_message_gps_state.vel_n_m_s * _message_gps_state.vel_n_m_s +
							_message_gps_state.vel_e_m_s * _message_gps_state.vel_e_m_s);

				if (pvt_geodetic.cog > k_dnu_f4_value) {
					_message_gps_state.cog_rad = pvt_geodetic.cog * M_DEG_TO_RAD_F;
				}
				_message_gps_state.c_variance_rad = M_DEG_TO_RAD_F;

				_message_gps_state.time_utc_usec = 0;
#ifndef __PX4_QURT // NOTE: Functionality isn't available on Snapdragon yet.
				if (_time_synced) {
					struct tm timeinfo;
					time_t epoch;

					// Convert to unix timestamp
					memset(&timeinfo, 0, sizeof(timeinfo));

					timeinfo.tm_year = 1980 - 1900;
					timeinfo.tm_mon = 0;
					timeinfo.tm_mday = 6 + header.wnc * 7;
					timeinfo.tm_hour = 0;
					timeinfo.tm_min = 0;
					timeinfo.tm_sec = header.tow / 1000;

					epoch = mktime(&timeinfo);

					if (epoch > k_gps_epoch_secs) {
						// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
						// and control its drift. Since we rely on the HRT for our monotonic
						// clock, updating it from time to time is safe.

						timespec ts;
						memset(&ts, 0, sizeof(ts));
						ts.tv_sec = epoch;
						ts.tv_nsec = (header.tow % 1000) * 1000 * 1000;
						set_clock(ts);

						_message_gps_state.time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
						_message_gps_state.time_utc_usec += (header.tow % 1000) * 1000;
					}
				}

#endif
				_message_gps_state.timestamp = hrt_absolute_time();
				result = PX4_OK;
			}

			break;
		}

		case BlockID::ReceiverStatus: {
			SEP_TRACE_PARSING("Processing ReceiverStatus SBF message");

			ReceiverStatus receiver_status;

			if (_sbf_decoder.parse(&receiver_status) == PX4_OK) {
				_message_gps_state.rtcm_msg_used = receiver_status.rx_state_diff_corr_in ? sensor_gps_s::RTCM_MSG_USED_USED : sensor_gps_s::RTCM_MSG_USED_NOT_USED;
				_time_synced = receiver_status.rx_state_wn_set && receiver_status.rx_state_tow_set;
			}

			break;
		}
		case BlockID::QualityInd: {
			SEP_TRACE_PARSING("Processing QualityInd SBF message");
			break;
		}
		case BlockID::RFStatus: {
			SEP_TRACE_PARSING("Processing RFStatus SBF message");
			break;
		}
		case BlockID::GALAuthStatus: {
			SEP_TRACE_PARSING("Processing GALAuthStatus SBF message");
			break;
		}
		case BlockID::EndOfPVT: {
			SEP_TRACE_PARSING("Processing EndOfPVT SBF message");

			// EndOfPVT guarantees that all PVT blocks for this epoch have been sent, so it's safe to assume the uORB message contains all required data.
			publish();
			break;
		}
		case BlockID::VelCovGeodetic: {
			SEP_TRACE_PARSING("Processing VelCovGeodetic SBF message");
			_current_interval_messages.vel_cov_geodetic = true;

			VelCovGeodetic vel_cov_geodetic;

			if (_sbf_decoder.parse(&vel_cov_geodetic) == PX4_OK) {
				if (vel_cov_geodetic.cov_ve_ve > k_dnu_f4_value && vel_cov_geodetic.cov_vn_vn > k_dnu_f4_value && vel_cov_geodetic.cov_vu_vu > k_dnu_f4_value) {
					_message_gps_state.s_variance_m_s = math::max(math::max(vel_cov_geodetic.cov_ve_ve, vel_cov_geodetic.cov_vn_vn), vel_cov_geodetic.cov_vu_vu);
				}
			}

			break;
		}
		case BlockID::GEOIonoDelay: {
			SEP_TRACE_PARSING("Processing GEOIonoDelay SBF message");
			break;
		}
		case BlockID::AttEuler: {
			using Error = AttEuler::Error;

			SEP_TRACE_PARSING("Processing AttEuler SBF message");
			_current_interval_messages.att_euler = true;

			AttEuler att_euler;

			if (_sbf_decoder.parse(&att_euler) == PX4_OK &&
			    !att_euler.error_not_requested &&
			    att_euler.error_aux1 == Error::None &&
			    att_euler.error_aux2 == Error::None &&
			    att_euler.heading > k_dnu_f4_value) {
				float heading = att_euler.heading * M_PI_F / 180.0f; // Range of degrees to range of radians in [0, 2PI].

				// Ensure range is in [-PI, PI].
				if (heading > M_PI_F) {
					heading -= 2.f * M_PI_F;
				}

				_message_gps_state.heading = heading;
			}

			break;
		}
		case BlockID::AttCovEuler: {
			using Error = AttCovEuler::Error;

			SEP_TRACE_PARSING("Processing AttCovEuler SBF message");
			_current_interval_messages.att_cov_euler = true;

			AttCovEuler att_cov_euler;

			if (_sbf_decoder.parse(&att_cov_euler) == PX4_OK &&
			    !att_cov_euler.error_not_requested &&
			    att_cov_euler.error_aux1 == Error::None &&
			    att_cov_euler.error_aux2 == Error::None &&
			    att_cov_euler.cov_headhead > k_dnu_f4_value) {
				_message_gps_state.heading_accuracy = att_cov_euler.cov_headhead * M_PI_F / 180.0f; // Convert range of degrees to range of radians in [0, 2PI]
			}

			break;
		}
		}

		break;
	}
	case DecodingStatus::RTCMv3: {
		SEP_TRACE_PARSING("Processing RTCMv3 message");
		publish_rtcm_corrections(_rtcm_decoder->message(), _rtcm_decoder->received_bytes());
		break;
	}
	}

	return result;
}

bool SeptentrioDriver::send_message(const char *msg)
{
	PX4_DEBUG("Send MSG: %s", msg);
	int length = strlen(msg);

	return (write(reinterpret_cast<const uint8_t*>(msg), length) == length);
}

bool SeptentrioDriver::send_message_and_wait_for_ack(const char *msg, const int timeout)
{
	if (!send_message(msg)) {
		return false;
	}

	// Wait for acknowledge
	// For all valid set -, get - and exe -commands, the first line of the reply is an exact copy
	// of the command as entered by the user, preceded with "$R: "
	char buf[k_read_buffer_size];
	char expected_response[k_max_command_size+4];
	snprintf(expected_response, sizeof(expected_response), "$R: %s", msg);
	uint16_t response_check_character = 0;
	// Length of the message without the newline but including the preceding response part "$R: "
	size_t response_len = strlen(msg) + 3;
	hrt_abstime timeout_time = hrt_absolute_time() + 1000 * timeout;

	do {
		int read_result = read(reinterpret_cast<uint8_t*>(buf), sizeof(buf), 50);

		if (read_result < 0) {
			SEP_WARN("SBF read error");
			return false;
		}

		for (int i = 0; i < read_result; i++) {
			if (response_check_character == response_len) {
				// We encountered the complete response
				return true;
			} else if (expected_response[response_check_character] == buf[i]) {
				++response_check_character;
			} else if (buf[i] == '$') {
				// Special case makes sure we don't miss start of new response if that happened to be the character we weren't expecting next (e.g., `$R: ge$R: gecm`)
				response_check_character = 1;
			} else {
				response_check_character = 0;
			}
		}
	} while (timeout_time > hrt_absolute_time());

	SEP_WARN("Response: timeout");
	return false;
}

int SeptentrioDriver::receive(unsigned timeout)
{
	int ret = 0;
	int handled = 0;
	uint8_t buf[k_read_buffer_size];

	// timeout additional to poll
	hrt_abstime time_started = hrt_absolute_time();

	while (true) {
		// Wait for only `k_receiver_read_timeout` if something already received.
		ret = read(buf, sizeof(buf), handled ? k_receiver_read_timeout : timeout);

		if (ret < 0) {
			// Something went wrong when polling or reading.
			SEP_WARN("poll_or_read err");
			return -1;

		} else {
			// Pass received bytes to the packet decoder.
			for (int i = 0; i < ret; i++) {
				handled |= parse_char(buf[i]);
			}
		}

		if (handled > 0) {
			return handled;
		}

		// abort after timeout if no useful packets received
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			PX4_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int SeptentrioDriver::read(uint8_t *buf, size_t buf_length, int timeout)
{
	int num_read = poll_or_read(buf, buf_length, timeout);

	if (num_read > 0) {
		_current_interval_bytes_read += num_read;
		if (should_dump_incoming()) {
			dump_gps_data(buf, num_read, DataDirection::FromReceiver);
		}
	}

	return num_read;
}

int SeptentrioDriver::poll_or_read(uint8_t *buf, size_t buf_length, int timeout)
{
	int read_timeout = math::min(k_max_receiver_read_timeout, timeout);

	return _uart.readAtLeast(buf, buf_length, math::min(k_min_receiver_read_bytes, buf_length), read_timeout);
}

int SeptentrioDriver::write(const uint8_t* buf, size_t buf_length)
{
	ssize_t write_result = _uart.write(buf, buf_length);

	if (write_result >= 0) {
		_current_interval_bytes_written += write_result;
		if (should_dump_outgoing()) {
			dump_gps_data(buf, write_result, DataDirection::ToReceiver);
		}
	}

	return write_result;
}

int SeptentrioDriver::initialize_communication_dump(DumpMode mode)
{
	if (mode == DumpMode::FromReceiver || mode == DumpMode::Both) {
		_message_data_from_receiver = new gps_dump_s();

		if (!_message_data_from_receiver) {
			SEP_ERR("Failed to allocate incoming dump buffer");
			return PX4_ERROR;
		}

		memset(_message_data_from_receiver, 0, sizeof(*_message_data_from_receiver));
	}
	if (mode == DumpMode::ToReceiver || mode == DumpMode::Both) {
		_message_data_to_receiver = new gps_dump_s();

		if (!_message_data_to_receiver) {
			SEP_ERR("failed to allocated dump data");
			return PX4_ERROR;
		}

		memset(_message_data_to_receiver, 0, sizeof(*_message_data_to_receiver));
	}

	if (mode != DumpMode::Disabled) {
		_gps_dump_pub.advertise();
	}

	return PX4_OK;
}

void SeptentrioDriver::reset_if_scheduled()
{
	ReceiverResetType reset_type = (ReceiverResetType)_scheduled_reset.load();

	if (reset_type != ReceiverResetType::None) {
		_scheduled_reset.store((int)ReceiverResetType::None);
		int res = reset(reset_type);

		if (res == PX4_OK) {
			SEP_INFO("Reset succeeded.");
		} else {
			SEP_INFO("Reset failed.");
		}
	}
}

int SeptentrioDriver::set_baudrate(uint32_t baud)
{
	if (_uart.setBaudrate(baud)) {
		SEP_INFO("baud controller: %lu", baud);
		return PX4_OK;
	} else {
		return PX4_ERROR;
	}
}

void SeptentrioDriver::handle_inject_data_topic()
{
	// We don't want to call copy again further down if we have already done a copy in the selection process.
	bool already_copied = false;
	gps_inject_data_s msg;

	// If there has not been a valid RTCM message for a while, try to switch to a different RTCM link
	if ((hrt_absolute_time() - _last_rtcm_injection_time) > 5_s) {

		for (int instance = 0; instance < _gps_inject_data_sub.size(); instance++) {
			const bool exists = _gps_inject_data_sub[instance].advertised();

			if (exists) {
				if (_gps_inject_data_sub[instance].copy(&msg)) {
					if ((hrt_absolute_time() - msg.timestamp) < 5_s) {
						// Remember that we already did a copy on this instance.
						already_copied = true;
						_selected_rtcm_instance = instance;
						break;
					}
				}
			}
		}
	}

	bool updated = already_copied;

	// Limit maximum number of GPS injections to 8 since usually
	// GPS injections should consist of 1-4 packets (GPS, GLONASS, BeiDou, Galileo).
	// Looking at 8 packets thus guarantees, that at least a full injection
	// data set is evaluated.
	// Moving Base requires a higher rate, so we allow up to 8 packets.
	const size_t max_num_injections = gps_inject_data_s::ORB_QUEUE_LENGTH;
	size_t num_injections = 0;

	do {
		if (updated) {
			num_injections++;

			// Prevent injection of data from self or from ground if moving base and this is rover.
			if ((_instance == Instance::Secondary && msg.device_id != get_device_id()) || (_instance == Instance::Main && msg.device_id == get_device_id()) || _receiver_setup != ReceiverSetup::MovingBase) {
				/* Write the message to the gps device. Note that the message could be fragmented.
				* But as we don't write anywhere else to the device during operation, we don't
				* need to assemble the message first.
				*/
				write(msg.data, msg.len);

				++_current_interval_rtcm_injections;
				_last_rtcm_injection_time = hrt_absolute_time();
			}
		}

		updated = _gps_inject_data_sub[_selected_rtcm_instance].update(&msg);

	} while (updated && num_injections < max_num_injections);
}

void SeptentrioDriver::publish()
{
	_message_gps_state.device_id = get_device_id();
	_message_gps_state.selected_rtcm_instance = _selected_rtcm_instance;
	_message_gps_state.rtcm_injection_rate = rtcm_injection_frequency();

	_sensor_gps_pub.publish(_message_gps_state);

	if (_message_gps_state.spoofing_state != _spoofing_state) {

		if (_message_gps_state.spoofing_state > sensor_gps_s::SPOOFING_STATE_NONE) {
			SEP_WARN("GPS spoofing detected! (state: %d)", _message_gps_state.spoofing_state);
		}

		_spoofing_state = _message_gps_state.spoofing_state;
	}

	if (_message_gps_state.jamming_state != _jamming_state) {

		if (_message_gps_state.jamming_state > sensor_gps_s::JAMMING_STATE_WARNING) {
			SEP_WARN("GPS jamming detected! (state: %d) (indicator: %d)", _message_gps_state.jamming_state,
					(uint8_t)_message_gps_state.jamming_indicator);
		}

		_jamming_state = _message_gps_state.jamming_state;
	}
}

void SeptentrioDriver::publish_satellite_info()
{
	if (_message_satellite_info) {
		_satellite_info_pub.publish(*_message_satellite_info);
	}
}

bool SeptentrioDriver::first_gps_uorb_message_created() const
{
	return _message_gps_state.timestamp != 0;
}

void SeptentrioDriver::publish_rtcm_corrections(uint8_t *data, size_t len)
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

void SeptentrioDriver::dump_gps_data(const uint8_t *data, size_t len, DataDirection data_direction)
{
	gps_dump_s *dump_data = data_direction == DataDirection::FromReceiver ? _message_data_from_receiver : _message_data_to_receiver;
	dump_data->instance = _instance == Instance::Main ? 0 : 1;

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
			if (data_direction == DataDirection::ToReceiver) {
				dump_data->len |= 1 << 7;
			}

			dump_data->timestamp = hrt_absolute_time();
			_gps_dump_pub.publish(*dump_data);
			dump_data->len = 0;
		}
	}
}

bool SeptentrioDriver::should_dump_incoming() const
{
	return _message_data_from_receiver != 0;
}

bool SeptentrioDriver::should_dump_outgoing() const
{
	return _message_data_to_receiver != 0;
}

void SeptentrioDriver::start_update_monitoring_interval()
{
	PX4_DEBUG("Update monitoring interval started");
	_last_interval_rtcm_injections = _current_interval_rtcm_injections;
	_last_interval_bytes_written = _current_interval_bytes_written;
	_last_interval_bytes_read = _current_interval_bytes_read;
	_last_interval_messages = _current_interval_messages;
	_current_interval_rtcm_injections = 0;
	_current_interval_bytes_written = 0;
	_current_interval_bytes_read = 0;
	_current_interval_messages = MessageTracker {};
	_current_interval_start_time = hrt_absolute_time();
}

bool SeptentrioDriver::update_monitoring_interval_ended() const
{
	return current_monitoring_interval_duration() > k_update_monitoring_interval_duration;
}

hrt_abstime SeptentrioDriver::current_monitoring_interval_duration() const
{
	return hrt_absolute_time() - _current_interval_start_time;
}

float SeptentrioDriver::rtcm_injection_frequency() const
{
	return _last_interval_rtcm_injections / us_to_s(static_cast<uint64_t>(k_update_monitoring_interval_duration));
}

uint32_t SeptentrioDriver::output_data_rate() const
{
	return static_cast<uint32_t>(_last_interval_bytes_written / us_to_s(static_cast<uint64_t>(k_update_monitoring_interval_duration)));
}

uint32_t SeptentrioDriver::input_data_rate() const
{
	return static_cast<uint32_t>(_last_interval_bytes_read / us_to_s(static_cast<uint64_t>(k_update_monitoring_interval_duration)));
}

bool SeptentrioDriver::receiver_configuration_healthy() const
{
	return _last_interval_messages.dop &&
               _last_interval_messages.pvt_geodetic &&
               _last_interval_messages.vel_cov_geodetic &&
               _last_interval_messages.att_euler &&
               _last_interval_messages.att_cov_euler;
}

float SeptentrioDriver::us_to_s(uint64_t us)
{
	return static_cast<float>(us) / 1000000.0f;
}

bool SeptentrioDriver::clock_needs_update(timespec real_time)
{
	timespec rtc_system_time;

	px4_clock_gettime(CLOCK_REALTIME, &rtc_system_time);
	int drift_time = abs(rtc_system_time.tv_sec - real_time.tv_sec);

	return drift_time >= k_max_allowed_clock_drift;
}

void SeptentrioDriver::set_clock(timespec rtc_gps_time)
{
	if (clock_needs_update(rtc_gps_time)) {
		px4_clock_settime(CLOCK_REALTIME, &rtc_gps_time);
	}
}

bool SeptentrioDriver::is_healthy() const
{
	if (_state == State::ReceivingData && receiver_configuration_healthy()) {
		return true;
	}

	return false;
}

void SeptentrioDriver::reset_gps_state_message()
{
	memset(&_message_gps_state, 0, sizeof(_message_gps_state));
	_message_gps_state.heading = NAN;
	_message_gps_state.heading_offset = matrix::wrap_pi(math::radians(_heading_offset));
}

uint32_t SeptentrioDriver::get_parameter(const char *name, int32_t *value)
{
	return _get_parameter(name, value);
}

uint32_t SeptentrioDriver::get_parameter(const char *name, float *value)
{
	return _get_parameter(name, value);
}

} // namespace septentrio

extern "C" __EXPORT int septentrio_main(int argc, char *argv[])
{
	return septentrio::SeptentrioDriver::main(argc, argv);
}

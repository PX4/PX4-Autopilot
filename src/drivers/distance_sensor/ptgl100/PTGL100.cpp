/*
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "PTGL100.hpp"

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace time_literals;

static constexpr hrt_abstime POLL_INTERVAL = 5_ms;
static constexpr hrt_abstime RETRY_INTERVAL = 1_s;
static constexpr hrt_abstime TX_TIMEOUT = 100_ms;

ModuleBase::Descriptor PTGL100::desc{task_spawn, custom_command, print_usage};

PTGL100::PTGL100(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_rangefinder(0, rotation)
{
	strncpy(_port, port, sizeof(_port) - 1);

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	id.devid_s.devtype = DRV_DIST_DEVTYPE_PTGL100;

	// Use the complete trailing port number (ttyS10 as well as ttyS1).
	size_t suffix = strlen(_port);

	while (suffix > 0 && _port[suffix - 1] >= '0' && _port[suffix - 1] <= '9') {
		--suffix;
	}

	const unsigned long bus = strtoul(&_port[suffix], nullptr, 10);

	if (bus <= 31) { // DeviceId has a five-bit bus field.
		id.devid_s.bus = bus;
	}

	_rangefinder.set_device_id(id.devid);
	_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_rangefinder.set_min_distance(ptgl100::MIN_DISTANCE_M);
	_rangefinder.set_max_distance(ptgl100::MAX_DISTANCE_M);
	_rangefinder.set_mode(distance_sensor_s::MODE_ENABLED);
	// Receiver FoV is unspecified; leave it unknown rather than using beam divergence.
}

PTGL100::~PTGL100()
{
	ScheduleClear();
	close_port();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
	perf_free(_checksum_errors);
	perf_free(_unknown_commands);
	perf_free(_measurement_failures);
	perf_free(_start_commands);
	perf_free(_samples);
}

int PTGL100::open_port()
{
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		return -errno;
	}

	termios config{};

	if (tcgetattr(_fd, &config) != 0) {
		const int error = errno;
		close_port();
		return -error;
	}

	// Raw binary 460800 baud, 8N1, no software or hardware flow control.
	config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK);
	config.c_oflag &= ~OPOST;
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	config.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
	config.c_cflag |= CS8 | CLOCAL | CREAD;
	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 0;

	if (cfsetispeed(&config, B460800) != 0 || cfsetospeed(&config, B460800) != 0
	    || tcsetattr(_fd, TCSANOW, &config) != 0 || tcflush(_fd, TCIOFLUSH) != 0) {
		const int error = errno;
		close_port();
		return -error;
	}

	_parser.reset();
	_last_start_command = 0;
	_last_measurement_reply = 0;
	return PX4_OK;
}

void PTGL100::close_port()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	_tx_offset = ptgl100::FRAME_SIZE;
	_parser.reset();
}

int PTGL100::init()
{
	_last_open_attempt = hrt_absolute_time();
	const int result = open_port();

	if (result != PX4_OK) {
		PX4_ERR("cannot configure %s: %d", _port, result);
		return result;
	}

	ScheduleOnInterval(POLL_INTERVAL);
	return PX4_OK;
}

void PTGL100::start_measurement(hrt_abstime now)
{
	_parser.reset();
	ptgl100::make_command(ptgl100::CONTINUOUS_MEASUREMENT, _tx_buffer);
	_tx_offset = 0;
	_last_start_command = now;
	perf_count(_start_commands);
}

void PTGL100::transmit()
{
	if (_tx_offset == ptgl100::FRAME_SIZE) {
		return;
	}

	const ssize_t written = ::write(_fd, _tx_buffer + _tx_offset, ptgl100::FRAME_SIZE - _tx_offset);

	if (written > 0) {
		_tx_offset += written;

	} else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
		perf_count(_comms_errors);
		close_port();
		return;
	}

	if (_tx_offset < ptgl100::FRAME_SIZE && hrt_elapsed_time(&_last_start_command) >= TX_TIMEOUT) {
		perf_count(_comms_errors);
		close_port();
	}
}

void PTGL100::stop_measurement()
{
	if (_fd >= 0) {
		uint8_t command[ptgl100::FRAME_SIZE];
		ptgl100::make_command(ptgl100::STOP_MEASUREMENT, command);

		// Best effort only: do not block a shared work queue waiting for an ACK.
		if (::write(_fd, command, sizeof(command)) != static_cast<ssize_t>(sizeof(command))) {
			perf_count(_comms_errors);
		}
	}
}

void PTGL100::handle_reply(const ptgl100::Reply &reply, hrt_abstime now)
{
	if (reply.is_measurement()) {
		_last_measurement_reply = now;
		_last_measurement_status = reply.status;

		if (reply.status == ptgl100::STATUS_SUCCESS) {
			_last_distance = ptgl100::distance_m(reply.distance_raw);
			_last_sample = now;
			_rangefinder.update(now, _last_distance);
			perf_count(_samples);

		} else {
			// The distance bytes are undefined on failure. Do not refresh an old sample.
			perf_count(_measurement_failures);
		}

	} else if (reply.command == ptgl100::SELF_TEST) {
		_self_test_received = true;
		_self_test_status = reply.status;
		_self_test_error = reply.error_code;
		// The module has restarted. The normal one-second retry will resume ranging.
		_last_measurement_reply = 0;

	} else if (reply.command == ptgl100::STOP_MEASUREMENT && reply.status == ptgl100::STATUS_SUCCESS) {
		_last_measurement_reply = 0;
	}
}

void PTGL100::collect()
{
	uint8_t buffer[64];

	// Bound work per invocation even when the serial port is flooded with noise.
	static constexpr unsigned MAX_READS = 4;

	for (unsigned read_count = 0; read_count < MAX_READS; ++read_count) {
		const ssize_t count = ::read(_fd, buffer, sizeof(buffer));

		if (count <= 0) {
			if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
				perf_count(_comms_errors);
				close_port();
			}

			return;
		}

		const hrt_abstime now = hrt_absolute_time();

		for (ssize_t i = 0; i < count; ++i) {
			ptgl100::Reply reply{};

			switch (_parser.parse(buffer[i], reply)) {
			case ptgl100::Parser::Result::Reply:
				handle_reply(reply, now);
				break;

			case ptgl100::Parser::Result::BadChecksum:
				perf_count(_checksum_errors);
				break;

			case ptgl100::Parser::Result::UnknownCommand:
				perf_count(_unknown_commands);
				break;

			case ptgl100::Parser::Result::Incomplete:
				break;
			}
		}
	}
}

void PTGL100::Run()
{
	if (should_exit()) {
		ScheduleClear();
		stop_measurement();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);
	const hrt_abstime now = hrt_absolute_time();

	if (_fd < 0 && now - _last_open_attempt >= RETRY_INTERVAL) {
		_last_open_attempt = now;

		if (open_port() != PX4_OK) {
			perf_count(_comms_errors);
		}
	}

	if (_fd >= 0) {
		collect();

		if (_fd >= 0) {
			if ((_last_start_command == 0 || hrt_elapsed_time(&_last_start_command) >= RETRY_INTERVAL)
			    && (_last_measurement_reply == 0 || hrt_elapsed_time(&_last_measurement_reply) >= RETRY_INTERVAL)) {
				start_measurement(hrt_absolute_time());
			}

			transmit();
		}
	}

	perf_end(_cycle_perf);
}

int PTGL100::print_status()
{
	PX4_INFO("port: %s, 460800 baud, 8N1 (%s)", _port, _fd >= 0 ? "open" : "reconnecting");
	PX4_INFO("distance_sensor instance: %d, device id: %" PRIu32, _rangefinder.get_instance(), _rangefinder.get_device_id());

	if (_last_sample != 0) {
		PX4_INFO("distance: %.3f m, sample age: %.3f s", (double)_last_distance,
			 (double)hrt_elapsed_time(&_last_sample) / 1e6);

	} else {
		PX4_INFO("no successful measurement yet");
	}

	PX4_INFO("last measurement status: %u", _last_measurement_status);

	if (_self_test_received) {
		PX4_INFO("self test status: %u, error: 0x%02x", _self_test_status, _self_test_error);
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_checksum_errors);
	perf_print_counter(_unknown_commands);
	perf_print_counter(_measurement_failures);
	perf_print_counter(_start_commands);
	perf_print_counter(_samples);
	return PX4_OK;
}

int PTGL100::task_spawn(int argc, char *argv[])
{
	const char *port = nullptr;
	int rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	int option_index = 1;
	const char *option_argument = nullptr;
	int option;

	while ((option = px4_getopt(argc, argv, "d:R:", &option_index, &option_argument)) != EOF) {
		switch (option) {
		case 'd':
			port = option_argument;
			break;

		case 'R':
			if (px4_get_parameter_value(option_argument, rotation) != 0 || rotation < 0 || rotation > 25) {
				PX4_ERR("invalid rotation");
				return PX4_ERROR;
			}

			break;

		default:
			print_usage("invalid argument");
			return PX4_ERROR;
		}
	}

	if (port == nullptr || port[0] == '\0' || strlen(port) >= PORT_NAME_SIZE || option_index != argc) {
		print_usage("specify a serial device with -d");
		return PX4_ERROR;
	}

	PTGL100 *instance = new PTGL100(port, rotation);

	if (instance != nullptr) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int PTGL100::custom_command(int argc, char *argv[])
{
	print_usage("unknown command");
	return PX4_ERROR;
}

int PTGL100::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Serial driver for the JRT PTGL-100 laser rangefinder (460800 baud, 8N1).
Set SENS_PTGL100_CFG to an unused serial port and reboot to start automatically.
SENS_PTGL100_ROT selects orientation (downward by default).

Publishes distance_sensor for MAVLink DISTANCE_SENSOR and the default ULog profile.
Use `listener distance_sensor` to view data, and `ptgl100 status` for its instance.
Uses the revised 0x68/0x69 protocol with 24-bit distances in millimetres.

### Examples
$ ptgl100 start -d /dev/ttyS1
$ ptgl100 status
$ ptgl100 stop
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("ptgl100", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int ptgl100_main(int argc, char *argv[])
{
	return ModuleBase::main(PTGL100::desc, argc, argv);
}

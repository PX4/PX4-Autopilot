/****************************************************************************
 *
 *   Copyright (C) 2017  Intel Corporation. All rights reserved.
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

#include <px4_config.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_module.h>

#include <stdlib.h>
#include <string.h>
#include <termios.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>

#include <perf/perf_counter.h>

#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

#define DEVICE_PATH                    "/dev/LeddarOne"
#define LEDDAR_ONE_DEFAULT_SERIAL_PORT "/dev/ttyS3"

#define MAX_DISTANCE            40.0f
#define MIN_DISTANCE            0.01f

#define SENSOR_READING_FREQ     10.0f
#define READING_USEC_PERIOD     (unsigned long)(1000000.0f / SENSOR_READING_FREQ)
#define OVERSAMPLE              6
#define WORK_USEC_INTERVAL      READING_USEC_PERIOD / OVERSAMPLE
#define COLLECT_USEC_TIMEOUT    READING_USEC_PERIOD / (OVERSAMPLE / 2)

/* 0.5sec */
#define PROBE_USEC_TIMEOUT      500000_us

#define MODBUS_SLAVE_ADDRESS    0x01
#define MODBUS_READING_FUNCTION 0x04
#define READING_START_ADDR      0x14
#define READING_LEN             0xA

static const uint8_t request_reading_msg[] = {
	MODBUS_SLAVE_ADDRESS,
	MODBUS_READING_FUNCTION,
	0, /* starting addr high byte */
	READING_START_ADDR,
	0, /* number of bytes to read high byte */
	READING_LEN,
	0x30, /* CRC low */
	0x09 /* CRC high */
};

struct __attribute__((__packed__)) reading_msg {
	uint8_t slave_addr;
	uint8_t function;
	uint8_t len;
	uint8_t low_timestamp_high_byte;
	uint8_t low_timestamp_low_byte;
	uint8_t high_timestamp_high_byte;
	uint8_t high_timestamp_low_byte;
	uint8_t temp_high;
	uint8_t temp_low;
	uint8_t num_detections_high_byte;
	uint8_t num_detections_low_byte;
	uint8_t first_dist_high_byte;
	uint8_t first_dist_low_byte;
	uint8_t first_amplitude_high_byte;
	uint8_t first_amplitude_low_byte;
	uint8_t second_dist_high_byte;
	uint8_t second_dist_low_byte;
	uint8_t second_amplitude_high_byte;
	uint8_t second_amplitude_low_byte;
	uint8_t third_dist_high_byte;
	uint8_t third_dist_low_byte;
	uint8_t third_amplitude_high_byte;
	uint8_t third_amplitude_low_byte;
	uint16_t crc; /* little-endian */
};

class LeddarOne : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	LeddarOne(const char *device_path, const char *serial_port, uint8_t rotation);
	virtual ~LeddarOne();

	virtual int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

private:

	/**
	 * Calculates the 16 byte crc value for the data frame.
	 * @param data_frame The data frame to compute a checksum for.
	 * @param crc16_length The length of the data frame.
	 */
	uint16_t crc16_calc(const unsigned char *data_frame, uint8_t crc16_length);

	/**
	 * Reads the data measrurement from serial UART.
	 */
	int collect();

	int measure();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	void publish(uint16_t distance_cm);

	bool request();

	void Run() override;

	const char *_serial_port;

	bool _collect_phase{false};

	int _file_descriptor{-1};

	uint8_t _buffer[sizeof(struct reading_msg)];
	uint8_t _buffer_len{0};
	uint8_t _rotation;

	hrt_abstime _timeout_usec{0};

	perf_counter_t _collect_timeout_perf{perf_alloc(PC_COUNT, "leddar_one_collect_timeout")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "leddar_one_comms_errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "leddar_one_sample")};

	orb_advert_t _topic{nullptr};
};

LeddarOne::LeddarOne(const char *device_path, const char *serial_port, uint8_t rotation):
	CDev(device_path),
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_rotation(rotation)
{
	_serial_port = strdup(serial_port);
}

LeddarOne::~LeddarOne()
{
	stop();

	free((char *)_serial_port);

	if (_topic) {
		orb_unadvertise(_topic);
	}

	perf_free(_collect_timeout_perf);
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

uint16_t
LeddarOne::crc16_calc(const unsigned char *data_frame, uint8_t crc16_length)
{
	uint16_t crc = 0xFFFF;

	for (uint8_t i = 0; i < crc16_length; i++) {
		crc ^= data_frame[i];

		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xA001;

			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

int
LeddarOne::collect()
{
	if (!_collect_phase) {
		return measure();
	}

	perf_end(_sample_perf);

	const hrt_abstime time_now = hrt_absolute_time();

	struct reading_msg *msg;

	int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, sizeof(_buffer) - _buffer_len);

	if (bytes_read < 1) {
		if (time_now > _timeout_usec) {
			_collect_phase = true;
			_timeout_usec = 0;
		}

		perf_count(_collect_timeout_perf);
		return PX4_ERROR;
	}

	_buffer_len += bytes_read;

	if (_buffer_len < sizeof(struct reading_msg)) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	msg = (struct reading_msg *)_buffer;

	if (msg->slave_addr != MODBUS_SLAVE_ADDRESS || msg->function != MODBUS_READING_FUNCTION) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	const uint16_t crc16 = crc16_calc(_buffer, _buffer_len - 2);

	if (crc16 != msg->crc) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	// NOTE: little-endian support only.
	publish(msg->first_dist_high_byte << 8 | msg->first_dist_low_byte);

	_collect_phase = false;
	_timeout_usec = time_now + READING_USEC_PERIOD;

	perf_end(_sample_perf);
	return PX4_OK;
}

int
LeddarOne::measure()
{
	const hrt_abstime time_now = hrt_absolute_time();

	if (time_now > _timeout_usec) {
		if (request()) {
			_buffer_len = 0;
			_timeout_usec = time_now + COLLECT_USEC_TIMEOUT;
			_collect_phase = true;
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

void
LeddarOne::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

int
LeddarOne::init()
{
	if (CDev::init()) {
		PX4_ERR("Unable to initialize character device\n");
		return -1;
	}

	if (open_serial_port()) {
		return PX4_ERROR;
	}

	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime timeout_usec = time_now + PROBE_USEC_TIMEOUT;

	while (time_now < timeout_usec) {
		if (measure() == PX4_OK) {
			PX4_INFO("LeddarOne initialized");
			return PX4_OK;
		}

		px4_usleep(1000);
		time_now = hrt_absolute_time();
	}

	PX4_ERR("No readings from LeddarOne");
	return PX4_ERROR;
}

int
LeddarOne::open_serial_port(const speed_t speed)
{
	// File descriptor already initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_serial_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	if (tcgetattr(_file_descriptor, &uart_config)) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		::close(_file_descriptor);
		_file_descriptor = -1;
		return PX4_ERROR;
	}

	// Clear: data bit size, two stop bits, parity, hardware flow control.
	uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

	// Set: 8 data bits, enable receiver, ignore modem status lines.
	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Turn off output processing.
	uart_config.c_oflag = 0;

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void
LeddarOne::print_info()
{

	perf_print_counter(_collect_timeout_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
	PX4_INFO("measure interval:  %u msec", static_cast<uint16_t>(WORK_USEC_INTERVAL) / 1000);
}

void
LeddarOne::publish(uint16_t distance_mm)
{
	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = _rotation;
	report.current_distance = ((float)distance_mm / 1000.0f);
	report.min_distance = MIN_DISTANCE;
	report.max_distance = MAX_DISTANCE;
	report.variance = 0.0f;
	report.signal_quality = -1;
	report.id = 0;

	if (_topic == nullptr) {
		_topic = orb_advertise(ORB_ID(distance_sensor), &report);

	} else {
		orb_publish(ORB_ID(distance_sensor), _topic, &report);
	}
}

bool
LeddarOne::request()
{
	/* flush anything in RX buffer */
	tcflush(_file_descriptor, TCIFLUSH);

	int r = ::write(_file_descriptor, request_reading_msg, sizeof(request_reading_msg));
	return r == sizeof(request_reading_msg);
}

void
LeddarOne::start()
{
	/*
	 * file descriptor can only be accessed by the process that opened it
	 * so closing here and it will be opened from the High priority kernel
	 * process
	 */
	::close(_file_descriptor);
	_file_descriptor = -1;

	ScheduleOnInterval(WORK_USEC_INTERVAL);
}

void
LeddarOne::stop()
{
	if (_file_descriptor > -1) {
		::close(_file_descriptor);
	}

	ScheduleClear();
}



/**
 * Local functions in support of the shell command.
 */
namespace leddar_one
{

LeddarOne *g_dev;

int start(const char *port, const uint8_t rotation);
int status();
int stop();
int test();
int usage();

int start(const char *port, const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	g_dev = new LeddarOne(DEVICE_PATH, port, rotation);

	if (g_dev == nullptr) {
		delete g_dev;
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	g_dev->start();
	PX4_INFO("driver started");
	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int test()
{
	int fd = open(DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Unable to open %s", DEVICE_PATH);
		return PX4_ERROR;
	}

	distance_sensor_s report;
	ssize_t sz = ::read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("No sample available in %s", DEVICE_PATH);
		return PX4_ERROR;
	}

	print_message(report);

	close(fd);

	PX4_INFO("PASS");
	return PX4_OK;
}

int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the LeddarOne LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_LEDDAR1_CFG parameter.

Setup/usage information: https://docs.px4.io/en/sensor/leddar_one.html

### Examples

Attempt to start driver on a specified serial device.
$ leddar_one start -d /dev/ttyS1
Stop driver
$ leddar_one stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("leddar_one", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");
	return PX4_OK;

}

} // namespace

extern "C" __EXPORT int leddar_one_main(int argc, char *argv[])
{

	const char *myoptarg = nullptr;

	int ch = 0;
	int myoptind = 1;

	const char *port = LEDDAR_ONE_DEFAULT_SERIAL_PORT;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "d:r", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			port = myoptarg;
			break;

		case 'r':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
			return leddar_one::usage();
		}
	}

	if (myoptind >= argc) {
		return leddar_one::usage();
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return leddar_one::start(port, rotation);

	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return leddar_one::status();
	}

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
		return leddar_one::stop();

	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return leddar_one::test();

	}

	// Print driver usage information.
	return leddar_one::usage();
}

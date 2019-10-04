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


#include <stdlib.h>
#include <string.h>
#include <termios.h>

#include <arch/board/board.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

#define DEVICE_PATH                     "/dev/LeddarOne"
#define LEDDAR_ONE_DEFAULT_SERIAL_PORT  "/dev/ttyS3"

#define LEDDAR_ONE_FIELD_OF_VIEW        (0.105f) // 6 deg cone angle.

#define LEDDAR_ONE_MAX_DISTANCE         40.0f
#define LEDDAR_ONE_MIN_DISTANCE         0.01f

#define LEDDAR_ONE_MEASURE_INTERVAL     100_ms // 10Hz

#define MODBUS_SLAVE_ADDRESS            0x01
#define MODBUS_READING_FUNCTION         0x04
#define READING_START_ADDR              0x14
#define READING_LEN                     0xA

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
	LeddarOne(const char *device_path,
		  const char *serial_port = LEDDAR_ONE_DEFAULT_SERIAL_PORT,
		  const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
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
	uint16_t crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length);

	/**
	 * Reads the data measrurement from serial UART.
	 */
	int collect();

	/**
	 * Sends a data request message to the sensor.
	 */
	int measure();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	void Run() override;

	const char *_serial_port{nullptr};

	PX4Rangefinder _px4_rangefinder;

	int _file_descriptor{-1};
	int _orb_class_instance{-1};

	uint8_t _buffer[sizeof(reading_msg)];
	uint8_t _buffer_len{0};

	hrt_abstime _measurement_time{0};

	perf_counter_t _comms_error{perf_alloc(PC_COUNT, "leddar_one_comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "leddar_one_sample")};
};

LeddarOne::LeddarOne(const char *device_path, const char *serial_port, uint8_t device_orientation):
	CDev(device_path),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_rangefinder(0 /* device id not yet used */, ORB_PRIO_DEFAULT, device_orientation)
{
	_serial_port = strdup(serial_port);

	_px4_rangefinder.set_device_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_px4_rangefinder.set_max_distance(LEDDAR_ONE_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(LEDDAR_ONE_MIN_DISTANCE);
	_px4_rangefinder.set_fov(LEDDAR_ONE_FIELD_OF_VIEW);
	_px4_rangefinder.set_orientation(device_orientation);
}

LeddarOne::~LeddarOne()
{
	stop();
	free((char *)_serial_port);
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

uint16_t
LeddarOne::crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length)
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
	perf_begin(_sample_perf);

	const int buffer_size = sizeof(_buffer);
	const int message_size = sizeof(reading_msg);

	int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, buffer_size - _buffer_len);

	if (bytes_read < 1) {
		// Trigger a new measurement.
		return measure();
	}

	_buffer_len += bytes_read;

	if (_buffer_len < message_size) {
		// Return on next scheduled cycle to collect remaining data.
		return PX4_OK;
	}

	reading_msg *msg {nullptr};
	msg = (reading_msg *)_buffer;

	if (msg->slave_addr != MODBUS_SLAVE_ADDRESS ||
	    msg->function != MODBUS_READING_FUNCTION) {
		PX4_ERR("slave address or function read error");
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return measure();
	}

	const uint16_t crc16 = crc16_calc(_buffer, buffer_size - 2);

	if (crc16 != msg->crc) {
		PX4_ERR("crc error");
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return measure();
	}

	// NOTE: little-endian support only.
	uint16_t distance_mm = (msg->first_dist_high_byte << 8 | msg->first_dist_low_byte);
	float distance_m = static_cast<float>(distance_mm) / 1000.0f;

	// @TODO - implement a meaningful signal quality value.
	int8_t signal_quality = -1;

	_px4_rangefinder.update(_measurement_time, distance_m, signal_quality);

	perf_end(_sample_perf);

	// Trigger the next measurement.
	return measure();;
}

int
LeddarOne::init()
{
	if (CDev::init()) {
		PX4_ERR("Unable to initialize device");
		return PX4_ERROR;
	}

	if (open_serial_port() != PX4_OK) {
		return PX4_ERROR;
	}

	hrt_abstime time_now = hrt_absolute_time();

	const hrt_abstime timeout_usec = time_now + 500000_us; // 0.5sec

	while (time_now < timeout_usec) {
		if (measure() == PX4_OK) {
			px4_usleep(LEDDAR_ONE_MEASURE_INTERVAL);

			if (collect() == PX4_OK) {
				// The file descriptor can only be accessed by the process that opened it,
				// so closing here allows the port to be opened from scheduled work queue.
				stop();
				return PX4_OK;
			}
		}

		px4_usleep(1000);
		time_now = hrt_absolute_time();
	}

	PX4_ERR("No readings from LeddarOne");
	return PX4_ERROR;
}

int
LeddarOne::measure()
{
	// Flush the receive buffer.
	tcflush(_file_descriptor, TCIFLUSH);

	int num_bytes = ::write(_file_descriptor, request_reading_msg, sizeof(request_reading_msg));

	if (num_bytes != sizeof(request_reading_msg)) {
		PX4_INFO("measurement error: %i, errno: %i", num_bytes, errno);
		return PX4_ERROR;
	}

	_measurement_time = hrt_absolute_time();
	_buffer_len = 0;
	return PX4_OK;
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

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

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

	// Flush the hardware buffers.
	tcflush(_file_descriptor, TCIOFLUSH);

	PX4_INFO("opened UART port %s", _serial_port);
	return PX4_OK;
}

void
LeddarOne::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
	PX4_INFO("measure interval:  %u msec", static_cast<uint16_t>(LEDDAR_ONE_MEASURE_INTERVAL));
}

void
LeddarOne::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void
LeddarOne::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(LEDDAR_ONE_MEASURE_INTERVAL, LEDDAR_ONE_MEASURE_INTERVAL);
	PX4_INFO("driver started");
}

void
LeddarOne::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;

	// Clear the work queue schedule.
	ScheduleClear();
}


/**
 * Local functions in support of the shell command.
 */
namespace leddar_one
{

LeddarOne *g_dev;

int start(const char *port = LEDDAR_ONE_DEFAULT_SERIAL_PORT,
	  const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int test(const char *port = LEDDAR_ONE_DEFAULT_SERIAL_PORT);
int usage();

int start(const char *port, const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	g_dev = new LeddarOne(DEVICE_PATH, port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		delete g_dev;
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	g_dev->start();
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
int
test(const char *port)
{
	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	int fd = ::open(port, flags);

	if (fd < 0) {
		PX4_ERR("Unable to open %s", port);
		return PX4_ERROR;
	}

	ssize_t num_bytes = ::write(fd, request_reading_msg, sizeof(request_reading_msg));

	if (num_bytes != sizeof(request_reading_msg)) {
		PX4_INFO("serial port write failed: %i, errno: %i", num_bytes, errno);
		return PX4_ERROR;
	}

	px4_usleep(LEDDAR_ONE_MEASURE_INTERVAL);

	uint8_t buffer[sizeof(reading_msg)];

	num_bytes = ::read(fd, buffer, sizeof(reading_msg));

	if (num_bytes != sizeof(reading_msg)) {
		PX4_ERR("Data not available at %s", port);
		return PX4_ERROR;
	}

	close(fd);

	PX4_INFO("PASS");
	return PX4_OK;
}

int
usage()
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
		return leddar_one::test(port);
	}

	// Print driver usage information.
	return leddar_one::usage();
}

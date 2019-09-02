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

#include <lib/drivers/distance_sensor/DistanceSensor.h>

using namespace time_literals;

#define DEVICE_PATH                    "/dev/LeddarOne"

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

class LeddarOne : public DistanceSensor, public cdev::CDev
{
public:
	LeddarOne(const char *device_path,
		  const char *serial_port = DEFAULT_SERIAL_PORT,
		  const uint8_t orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	virtual ~LeddarOne();

	/**
	 * Performs device specific initializations.
	 */
	int dev_init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:

	/**
	 * Calculates the 16 byte crc value for the data frame.
	 * @param data_frame The data frame to compute a checksum for.
	 * @param crc16_length The length of the data frame.
	 */
	uint16_t crc16_calc(const unsigned char *data_frame, uint8_t crc16_length);

	/**
	 * Performs device specific stop actions.
	 */
	void dev_stop();

	/**
	 * Reads distance sensor data from serial UART, calls the data_parser and validates the checksum.
	 */
	float get_distance();

	int measure();

	bool measure_request();

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

LeddarOne::LeddarOne(const char *device_path, const char *serial_port, const uint8_t orientation):
	CDev(device_path)
{
	_max_distance     = MAX_DISTANCE;
	_min_distance     = MIN_DISTANCE;
	_measure_interval = WORK_USEC_INTERVAL;
	_orientation      = orientation;
	_serial_port      = strdup(serial_port);
}

LeddarOne::~LeddarOne()
{
	// Device specific stop behaviors.
	dev_stop();
	free((char *)_serial_port);
	perf_free(_collect_timeout_perf);
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

float
LeddarOne::get_distance()
{
	// Ensure the serial port is open.
	if (open_serial_port() == PX4_ERROR) {
		return -1.f;
	}

	float distance_m = -1.0f;

	const hrt_abstime time_now = hrt_absolute_time();

	int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, sizeof(_buffer) - _buffer_len);

	if (bytes_read < 1) {
		if (time_now > _timeout_usec) {
			_collect_phase = true;
			_timeout_usec = 0;
		}

		perf_count(_collect_timeout_perf);
		return distance_m;
	}

	_buffer_len += bytes_read;

	if (_buffer_len < sizeof(struct reading_msg)) {
		perf_count(_comms_errors);
		return distance_m;
	}

	reading_msg *msg;
	msg = (struct reading_msg *)_buffer;

	if (msg->slave_addr != MODBUS_SLAVE_ADDRESS || msg->function != MODBUS_READING_FUNCTION) {
		perf_count(_comms_errors);
		return distance_m;
	}

	const uint16_t crc16 = crc16_calc(_buffer, _buffer_len - 2);

	if (crc16 != msg->crc) {
		perf_count(_comms_errors);
		return distance_m;
	}

	_collect_phase = false;
	_timeout_usec = time_now + READING_USEC_PERIOD;

	// NOTE: little-endian support only.
	int distance_mm = (msg->first_dist_high_byte << 8 | msg->first_dist_low_byte);
	distance_m = static_cast<float>(distance_mm) / 1000.0f;

	return distance_m;
}

int
LeddarOne::dev_init()
{
	if (CDev::init()) {
		PX4_ERR("Unable to initialize character device\n");
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

void
LeddarOne::dev_stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;
}

int
LeddarOne::measure()
{
	const hrt_abstime time_now = hrt_absolute_time();

	if (time_now > _timeout_usec) {
		if (measure_request()) {
			_buffer_len = 0;
			_timeout_usec = time_now + COLLECT_USEC_TIMEOUT;
			_collect_phase = true;
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

bool
LeddarOne::measure_request()
{
	/* flush anything in RX buffer */
	tcflush(_file_descriptor, TCIFLUSH);

	int r = ::write(_file_descriptor, request_reading_msg, sizeof(request_reading_msg));
	return r == sizeof(request_reading_msg);
}

void
LeddarOne::print_info()
{
	perf_print_counter(_collect_timeout_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
	PX4_INFO("measure interval:  %u msec", _measure_interval / 1000);
}


/**
 * Local functions in support of the shell command.
 */
namespace leddar_one
{

LeddarOne *g_dev;

int reset(const char *port = DEFAULT_SERIAL_PORT);
int start(const char *port = DEFAULT_SERIAL_PORT,
	  const uint8_t orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int test(const char *port = DEFAULT_SERIAL_PORT);
int usage();

/**
 * Reset the driver.
 */
int
reset(const char *port)
{
	if (stop() == PX4_OK) {
		return start(port);
	}

	return PX4_ERROR;
}

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
	if (g_dev->DistanceSensor::init() != PX4_OK) {
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
int
test(const char *port)
{
	int fd = open(port, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Unable to open %s", port);
		return PX4_ERROR;
	}

	distance_sensor_s report;
	ssize_t bytes_read = read(fd, &report, sizeof(report));

	if (bytes_read != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	close(fd);
	print_message(report);
	return PX4_OK;
}

int usage()
{
	PX4_INFO("usage: leddar_one command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int leddar_one_main(int argc, char *argv[])
{

	const char *myoptarg = nullptr;

	int ch = 0;
	int myoptind = 1;

	const char *port = DEFAULT_SERIAL_PORT;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "d:R", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			port = myoptarg;
			break;

		case 'R':
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

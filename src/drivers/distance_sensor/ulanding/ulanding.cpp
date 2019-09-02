/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file ulanding.cpp
 * @author Jessica Stockham <jessica@aerotenna.com>
 * @author Roman Bapst <roman@uaventure.com>
 *
 * Driver for the uLanding radar from Aerotenna
 */

#include <lib/drivers/distance_sensor/DistanceSensor.h>
#include <mathlib/mathlib.h>

using namespace time_literals;

#define ULANDING_MEASURE_INTERVAL       10_ms
#define ULANDING_MAX_DISTANCE           50.0f
#define ULANDING_MIN_DISTANCE           0.315f
#define ULANDING_VERSION                1

#if defined(__PX4_POSIX_OCPOC)
#define ULANDING_DEFAULT_PORT   "/dev/ttyS6"    // Default uLanding port on OCPOC.
#else
#define ULANDING_DEFAULT_PORT   "/dev/ttyS2"    // Default serial port on Pixhawk (TELEM2), baudrate 115200
#endif

#if ULANDING_VERSION == 1
#define ULANDING_PACKET_HDR        254
#define ULANDING_BUFFER_LENGTH     18
#else
#define ULANDING_PACKET_HDR        72
#define ULANDING_BUFFER_LENGTH     9
#endif

/**
 * Assume standard deviation to be equal to sensor resolution.
 * Static bench tests have shown that the sensor ouput does
 * not vary if the unit is not moved.
 */
#define ULANDING_SENSOR_RESOLUTION      0.045f * 0.045f


class ULanding : public DistanceSensor, public cdev::CDev
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	ULanding(const char *port = ULANDING_DEFAULT_PORT,
		 const uint8_t orietation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~ULanding();

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	virtual int dev_init() override;

private:

	/**
	 * Performs device specific stop actions.
	 */
	void dev_stop();

	/**
	 * Reads distance sensor data from serial UART, calls the data_parser and validates the checksum.
	 */
	float get_distance();

	uint8_t _buffer[ULANDING_BUFFER_LENGTH] {};
};

ULanding::ULanding(const char *serial_port, const uint8_t orientation) :
	CDev(RANGE_FINDER_BASE_DEVICE_PATH)
{
	_collect_phase    = true;
	_max_distance     = ULANDING_MAX_DISTANCE;
	_min_distance     = ULANDING_MIN_DISTANCE;
	_measure_interval = ULANDING_MEASURE_INTERVAL;
	_orientation      = orientation;
	_serial_port      = strdup(serial_port);
	_variance         = ULANDING_SENSOR_RESOLUTION * ULANDING_SENSOR_RESOLUTION;
}

ULanding::~ULanding()
{
	// Device specific stop behaviors.
	dev_stop();
	free((char *)_serial_port);
}

int
ULanding::dev_init()
{
	// Intitialize the character device.
	if (CDev::init() == OK) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

void
ULanding::dev_stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;
}

float
ULanding::get_distance()
{
	// Ensure the serial port is open.
	open_serial_port();

	int bytes_processed = 0;
	int distance_cm = -1;
	int index = 0;

	bool checksum_passed = false;

	float distance_m = -1.0f;

	// Read from the sensor UART buffer.
	int bytes_read = ::read(_file_descriptor, &_buffer[0], sizeof(_buffer));

	if (bytes_read > 0) {
		index = bytes_read - 6;

		while (index >= 0 && !checksum_passed) {
			if (_buffer[index] == ULANDING_PACKET_HDR) {
				bytes_processed = index;

				while (bytes_processed < bytes_read && !checksum_passed) {
					if (ULANDING_VERSION == 1) {
						uint8_t checksum_value = (_buffer[index + 1] + _buffer[index + 2] + _buffer[index + 3] + _buffer[index + 4]) & 0xFF;
						uint8_t checksum_byte = _buffer[index + 5];

						if (checksum_value == checksum_byte) {
							checksum_passed = true;
							distance_cm = (_buffer[index + 3] << 8) | _buffer[index + 2];
							distance_m = static_cast<float>(distance_cm) / 100.f;
						}

					} else {
						checksum_passed = true;
						distance_cm = (_buffer[index + 1] & 0x7F);
						distance_cm += ((_buffer[index + 2] & 0x7F) << 7);
						distance_m = static_cast<float>(distance_cm) * ULANDING_SENSOR_RESOLUTION;
						break;
					}

					bytes_processed++;
				}
			}

			index--;
		}
	}

	if (!checksum_passed) {
		return -EAGAIN;
	}

	distance_m = math::constrain(distance_m, ULANDING_MIN_DISTANCE, ULANDING_MAX_DISTANCE);
	return distance_m;
}

namespace ulanding
{
ULanding *g_dev;

int reset(const char *port = ULANDING_DEFAULT_PORT);
int start(const char *port = ULANDING_DEFAULT_PORT,
	  const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int test(const char *port = ULANDING_DEFAULT_PORT);
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

/**
 * Start the driver.
 */
int
start(const char *port, const uint8_t orientation)
{
	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new ULanding(port, orientation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->DistanceSensor::init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

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

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
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
		PX4_ERR("%s open failed (try 'cm8jl65 start' if the driver is not running", port);
		return PX4_ERROR;
	}

	// Perform a simple demand read.
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

int
usage()
{
	PX4_INFO("usage: ulanding command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace ulanding


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int ulanding_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = ULANDING_DEFAULT_PORT;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return ulanding::usage();
		}
	}

	if (myoptind >= argc) {
		return ulanding::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return ulanding::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return ulanding::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return ulanding::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return ulanding::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return ulanding::test();
	}

	return ulanding::usage();
}

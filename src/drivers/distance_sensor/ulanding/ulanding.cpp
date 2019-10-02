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

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/uORB.h>


using namespace time_literals;

#define ULANDING_MEASURE_INTERVAL       10_ms
#define ULANDING_MAX_DISTANCE	        50.0f
#define ULANDING_MIN_DISTANCE	        0.315f
#define ULANDING_VERSION	        1

#if defined(__PX4_POSIX_OCPOC)
#define RADAR_DEFAULT_PORT      "/dev/ttyS6"	// Default uLanding port on OCPOC.
#else
#define RADAR_DEFAULT_PORT      "/dev/ttyS2"	// Default serial port on Pixhawk (TELEM2), baudrate 115200
#endif

#if ULANDING_VERSION == 1
#define ULANDING_PACKET_HDR     254
#define ULANDING_BUFFER_LENGTH  18
#else
#define ULANDING_PACKET_HDR     72
#define ULANDING_BUFFER_LENGTH  9
#endif

/**
 * Assume standard deviation to be equal to sensor resolution.
 * Static bench tests have shown that the sensor ouput does
 * not vary if the unit is not moved.
 */
#define SENS_VARIANCE           0.045f * 0.045f


class Radar : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	Radar(const char *port = RADAR_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~Radar();

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	virtual int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	void stop();

	char _port[20];

	int _file_descriptor{-1};
	int _orb_class_instance{-1};

	unsigned int _head{0};
	unsigned int _tail{0};

	uint8_t _buffer[ULANDING_BUFFER_LENGTH] {};
	uint8_t _rotation;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "radar_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "radar_read")};

	orb_advert_t _distance_sensor_topic{nullptr};
};

Radar::Radar(const char *port, uint8_t rotation) :
	CDev(RANGE_FINDER0_DEVICE_PATH),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_rotation(rotation)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

Radar::~Radar()
{
	// Ensure we are truly inactive.
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
Radar::collect()
{
	perf_begin(_sample_perf);

	int bytes_processed = 0;
	int distance_cm = -1;
	int index = 0;

	float distance_m = -1.0f;

	bool checksum_passed = false;

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
						distance_m = static_cast<float>(distance_cm) * 0.045f;
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

	distance_sensor_s report;
	report.current_distance = distance_m;
	report.id               = 0;	// TODO: set proper ID.
	report.max_distance     = ULANDING_MAX_DISTANCE;
	report.min_distance     = ULANDING_MIN_DISTANCE;
	report.orientation      = _rotation;
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
	report.variance         = SENS_VARIANCE;

	// Publish the new report.
	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

	// Notify anyone waiting for data.
	poll_notify(POLLIN);
	perf_end(_sample_perf);

	return PX4_OK;
}

int
Radar::init()
{
	// Intitialize the character device.
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	// Get a publish handle on the range finder topic.
	distance_sensor_s ds_report = {};
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	start();
	return PX4_OK;
}

int
Radar::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_file_descriptor)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// No line processing - echo off, echo newline off, canonical mode off, extended input processing off, signal chars off
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

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

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void
Radar::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("ulanding max distance %.2f m", static_cast<double>(ULANDING_MAX_DISTANCE));
	PX4_INFO("ulanding min distance %.2f m", static_cast<double>(ULANDING_MIN_DISTANCE));
	PX4_INFO("ulanding update rate: %.2f Hz", static_cast<double>(ULANDING_MEASURE_INTERVAL));
}

void
Radar::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void
Radar::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(ULANDING_MEASURE_INTERVAL, 0);
	PX4_INFO("driver started");
}

void
Radar::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);

	// Clear the work queue schedule.
	ScheduleClear();
}

namespace radar
{
Radar	*g_dev;

int reset(const char *port = RADAR_DEFAULT_PORT);
int start(const char *port = RADAR_DEFAULT_PORT,
	  const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int test(const char *port = RADAR_DEFAULT_PORT);
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
start(const char *port, const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new Radar(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
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
		PX4_ERR("%s open failed (try 'radar start' if the driver is not running", port);
		return PX4_ERROR;
	}

	// Perform a simple demand read.
	distance_sensor_s report;
	ssize_t bytes_read = read(fd, &report, sizeof(report));

	if (bytes_read != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);
	return PX4_OK;
}

int
usage()
{
	PX4_INFO("usage: radar command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace radar


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int ulanding_radar_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = RADAR_DEFAULT_PORT;
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
			return radar::usage();
		}
	}

	if (myoptind >= argc) {
		return radar::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return radar::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return radar::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return radar::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return radar::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return radar::test();
	}

	return radar::usage();
}

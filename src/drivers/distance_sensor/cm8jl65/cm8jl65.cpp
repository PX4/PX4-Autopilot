/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file cm8jl65.cpp
 * @author Claudio Micheli <claudio@auterion.com>
 *
 * Driver for the Lanbao PSK-CM8JL65-CC5 distance sensor.
 * Make sure to disable MAVLINK messages (MAV_0_CONFIG PARAMETER)
 * on the serial port you connect the sensor,i.e TELEM1.
 *
 */

#include <poll.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
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
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include "cm8jl65_parser.h"


/* Configuration Constants */
#define CM8JL65_TAKE_RANGE_REG		'd'
#define CM8JL65_DEFAULT_PORT		"/dev/ttyS2"	// Default serial port on Pixhawk (TELEM2), baudrate 115200
#define CM8JL65_CONVERSION_INTERVAL 	50*1000UL	// 50ms default sensor conversion time.


class CM8JL65 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	CM8JL65(const char *port = CM8JL65_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~CM8JL65() override;

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	virtual int  init() override;

	virtual int  ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:

	/**
	 * Reads data from serial UART and places it into a buffer
	 */
	int collect();

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

	char _port[20] {};

	unsigned char _frame_data[PARSER_BUF_LENGTH] {START_FRAME_DIGIT1, START_FRAME_DIGIT2, 0, 0};

	int _class_instance{-1};
	int _conversion_interval{CM8JL65_CONVERSION_INTERVAL};
	int _fd{-1};
	int _orb_class_instance{-1};

	uint8_t _cycle_counter{0};
	uint8_t _linebuf[25] {};
	uint8_t _rotation{0};

	uint16_t _crc16{0};

	float _max_distance{9.0f};
	float _min_distance{0.10f};

	ringbuffer::RingBuffer *_reports{nullptr};

	CM8JL65_PARSE_STATE _parse_state{STATE0_WAITING_FRAME};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "cm8jl65_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "cm8jl65_read")};
};


CM8JL65::CM8JL65(const char *port, uint8_t rotation) :
	CDev(RANGE_FINDER0_DEVICE_PATH),
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_rotation(rotation)
{
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';
}

CM8JL65::~CM8JL65()
{
	// Ensure we are truly inactive.
	stop();

	// Free any existing reports.
	if (_reports != nullptr) {
		delete _reports;
	}

	// Unregister the device class name.
	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
CM8JL65::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

		if (_reports == nullptr) {
			PX4_ERR("alloc failed");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		/* get a publish handle on the range finder topic */
		struct distance_sensor_s ds_report = {};

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			PX4_ERR("failed to create distance_sensor object");
		}

	} while (0);

	return ret;
}

int
CM8JL65::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					start();
					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {

					/* convert hz to tick interval via microseconds */
					int interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < _conversion_interval) {
						return -EINVAL;
					}

					start();


					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

int
CM8JL65::collect()
{
	int index = 0;
	int _distance_mm = -1;
	bool crc_valid = false;

	perf_begin(_sample_perf);

	// Read from the sensor UART buffer.
	int bytes_read = ::read(_fd, &_linebuf[0], sizeof(_linebuf));
	int bytes_processed = 0;

	if (bytes_read < 0) {
		PX4_DEBUG("read err: %d \n", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

	} else if (bytes_read > 0) {
		//     printf("Bytes read: %d \n",bytes_read);
		index = bytes_read - 6 ;

		while ((index >= 0) && (!crc_valid)) {
			if (_linebuf[index] == START_FRAME_DIGIT1) {
				bytes_processed = index;

				while ((bytes_processed < bytes_read) && (!crc_valid)) {
					//    printf("In the cycle, processing  byte %d, 0x%02X \n",bytes_processed, _linebuf[bytes_processed]);
					if (OK == cm8jl65_parser(_linebuf[bytes_processed], _frame_data, _parse_state, _crc16, _distance_mm)) {
						crc_valid = true;
					}

					bytes_processed++;
				}

				_parse_state = STATE0_WAITING_FRAME;
			}

			index--;
		}
	}

	if (!crc_valid) {
		return -EAGAIN;
	}

	bytes_read = OK;

	//printf("val (int): %d, raw: 0x%08X, valid: %s \n", _distance_mm, _frame_data, ((crc_valid) ? "OK" : "NO"));

	struct distance_sensor_s report;
	report.current_distance = static_cast<float>(_distance_mm) / 1000.0f;
	report.id               = 0;	// TODO: set proper ID.
	report.max_distance     = _max_distance;
	report.min_distance     = _min_distance;
	report.orientation      = _rotation;
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.variance         = 0.0f;

	// Publish the new report.
	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

	_reports->force(&report);

	// Notify anyone waiting for data.
	poll_notify(POLLIN);
	perf_end(_sample_perf);

	return PX4_OK;
}

void
CM8JL65::start()
{
	PX4_INFO("driver started");

	// Flush the report ring buffer.
	_reports->flush();

	// Schedule a cycle to start things.
	ScheduleNow();
}

void
CM8JL65::stop()
{
	ScheduleClear();
}

void
CM8JL65::Run()
{
	// File descriptor initialized?
	if (_fd < 0) {
		// Open fd.
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		// Fill the struct for the new configuration.
		tcgetattr(_fd, &uart_config);

		// Clear ONLCR flag (which appends a CR for every LF).
		uart_config.c_oflag &= ~ONLCR;

		// No parity, one stop bit.
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

		unsigned speed = B115200;

		// Set baud rate.
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}
	}

	// Perform collection.
	int collect_ret = collect();

	if (collect_ret == -EAGAIN) {
		_cycle_counter++;
	}

	// Schedule a fresh cycle call when a complete packet has been received.
	ScheduleDelayed(_conversion_interval);
	_cycle_counter = 0;
}

void
CM8JL65::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace cm8jl65
{

CM8JL65	*g_dev;

int reset();
int start(const char *port, const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int test();
int usage();

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	return PX4_OK;
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
	g_dev = new CM8JL65(port, rotation);

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

	// Set the poll rate to default, starts automatic data collection.
	int fd = open(RANGE_FINDER0_DEVICE_PATH, 0);

	if (fd < 0) {
		PX4_ERR("Opening device '%s' failed", port);
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("failed to set baudrate %d", B115200);
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
		return PX4_OK;
	}

	return PX4_ERROR;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	int fd = open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'cm8jl65 start' if the driver is not running", RANGE_FINDER0_DEVICE_PATH);
		return PX4_ERROR;
	}

	// Perform a simple demand read.
	struct distance_sensor_s report;
	ssize_t sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	// Start the sensor polling at 2 Hz rate.
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}

	// Read the sensor 5x and report each value.
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		// Wait for data to be ready.
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out");
			break;
		}

		// Now go get it.
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("read failed: got %zi vs exp. %zu", sz, sizeof(report));
			break;
		}

		print_message(report);
	}

	// Reset the sensor polling to the default rate.
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("ioctl SENSORIOCSPOLLRATE failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
usage()
{
	PX4_INFO("usage: cm8jl65 command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace cm8jl65


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int cm8jl65_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = CM8JL65_DEFAULT_PORT;
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
			return cm8jl65::usage();
		}
	}

	if (myoptind >= argc) {
		return cm8jl65::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return cm8jl65::reset();
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return cm8jl65::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return cm8jl65::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return cm8jl65::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return cm8jl65::test();
	}

	return cm8jl65::usage();
}

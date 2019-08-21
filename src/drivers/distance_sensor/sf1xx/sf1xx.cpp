/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file sf1xx.cpp
 *
 * @author ecmnet <ecm@gmx.de>
 * @author Vasily Evseenko <svpcom@gmail.com>
 *
 * Driver for the Lightware SF1xx lidar range finder series.
 * Default I2C address 0x66 is used.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_module.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#define SF1XX_BUS_DEFAULT	PX4_I2C_BUS_EXPANSION
#define SF1XX_BASEADDR		0x66
#define SF1XX_DEVICE_PATH	"/dev/sf1xx"


class SF1XX : public device::I2C, public px4::ScheduledWorkItem
{
public:
	SF1XX(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING, int bus = SF1XX_BUS_DEFAULT,
	      int address = SF1XX_BASEADDR);

	virtual ~SF1XX() override;

	int init() override;

	ssize_t read(device::file_t *filp, char *buffer, size_t buflen) override;
	int ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

protected:
	int probe() override;

private:
	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address The I2C bus address to probe.
	* @return True if the device is present.
	*/
	int probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults SF1XX_MIN_DISTANCE
	* and SF1XX_MAX_DISTANCE
	*/
	void set_minimum_distance(float min);
	void set_maximum_distance(float max);
	float get_minimum_distance();
	float get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void Run() override;
	int measure();
	int collect();

	bool _sensor_ok{false};

	int _class_instance{-1};
	int _conversion_interval{-1};
	int _measure_interval{0};
	int _orb_class_instance{-1};

	float _max_distance{-1.0f};
	float _min_distance{-1.0f};

	uint8_t _rotation{0};

	ringbuffer::RingBuffer  *_reports{nullptr};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "sf1xx_read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "sf1xx_com_err")};
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sf1xx_main(int argc, char *argv[]);

SF1XX::SF1XX(uint8_t rotation, int bus, int address) :
	I2C("SF1XX", SF1XX_DEVICE_PATH, bus, address, 400000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_rotation(rotation)
{
}

SF1XX::~SF1XX()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
SF1XX::init()
{
	int ret = PX4_ERROR;
	int hw_model;
	param_get(param_find("SENS_EN_SF1XX"), &hw_model);

	switch (hw_model) {
	case 0:
		PX4_WARN("disabled.");
		return ret;

	case 1:  /* SF10/a (25m 32Hz) */
		_min_distance = 0.01f;
		_max_distance = 25.0f;
		_conversion_interval = 31250;
		break;

	case 2:  /* SF10/b (50m 32Hz) */
		_min_distance = 0.01f;
		_max_distance = 50.0f;
		_conversion_interval = 31250;
		break;

	case 3:  /* SF10/c (100m 16Hz) */
		_min_distance = 0.01f;
		_max_distance = 100.0f;
		_conversion_interval = 62500;
		break;

	case 4:
		/* SF11/c (120m 20Hz) */
		_min_distance = 0.01f;
		_max_distance = 120.0f;
		_conversion_interval = 50000;
		break;

	case 5:
		/* SF20/LW20 (100m 48-388Hz) */
		_min_distance = 0.001f;
		_max_distance = 100.0f;
		_conversion_interval = 20834;
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return ret;
	}

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	set_device_address(SF1XX_BASEADDR);

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	// Select altitude register
	int ret2 = measure();

	if (ret2 == 0) {
		ret = OK;
		_sensor_ok = true;
		PX4_INFO("(%dm %dHz) with address %d found", (int)_max_distance,
			 (int)(1e6f / _conversion_interval), SF1XX_BASEADDR);
	}

	return ret;
}

int
SF1XX::probe()
{
	return measure();
}

void
SF1XX::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
SF1XX::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
SF1XX::get_minimum_distance()
{
	return _min_distance;
}

float
SF1XX::get_maximum_distance()
{
	return _max_distance;
}

int
SF1XX::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_interval = (_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* convert hz to tick interval via microseconds */
					int interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < _conversion_interval) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
SF1XX::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		px4_usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
SF1XX::measure()
{
	int ret;

	/*
	 * Send the command '0' -- read altitude
	 */

	uint8_t cmd = 0;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
SF1XX::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};
	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = float(distance_cm) * 1e-2f;

	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = _rotation;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.variance = 0.0f;
	report.signal_quality = -1;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
SF1XX::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* set register to '0' */
	measure();

	/* schedule a cycle to start things */
	ScheduleDelayed(_conversion_interval);
}

void
SF1XX::stop()
{
	ScheduleClear();
}

void
SF1XX::Run()
{
	/* Collect results */
	if (OK != collect()) {
		PX4_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_conversion_interval);
}

void
SF1XX::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace sf1xx
{

SF1XX	*g_dev;

int 	start(uint8_t rotation);
int 	start_bus(uint8_t rotation, int i2c_bus);
int 	stop();
int 	test();
int 	reset();
int 	info();

/**
 *
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start(uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(rotation, i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(uint8_t rotation, int i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new SF1XX(rotation, i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(SF1XX_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		px4_close(fd);
		goto fail;
	}

	px4_close(fd);
	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
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

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = px4_open(SF1XX_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'sf1xx start' if the driver is not running)", SF1XX_DEVICE_PATH);
		return PX4_ERROR;
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			return PX4_ERROR;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			return PX4_ERROR;
		}

		print_message(report);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default poll rate");
		return PX4_ERROR;
	}

	px4_close(fd);

	PX4_INFO("PASS");
	return PX4_OK;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = px4_open(SF1XX_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
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

	px4_close(fd);

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

} /* namespace */


static void
sf1xx_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for Lightware SFxx series LIDAR rangefinders: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/en/sensor/sfxx_lidar.html

### Examples

Attempt to start driver on any bus (start on bus where first sensor found).
$ sf1xx start -a
Stop driver
$ sf1xx stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sf1xx", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset","Reset driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info","Print driver information");

}

int
sf1xx_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	bool start_all = false;

	int i2c_bus = SF1XX_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return sf1xx::start(rotation);

		} else {
			return sf1xx::start_bus(rotation, i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return sf1xx::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return sf1xx::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return sf1xx::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return sf1xx::info();
	}

out_error:
	sf1xx_usage();
	return PX4_ERROR;
}

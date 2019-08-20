/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file teraranger.cpp
 * @author Luis Rodrigues
 *
 * Driver for the TeraRanger One range finders connected via I2C.
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
#define TERARANGER_BUS_DEFAULT           PX4_I2C_BUS_EXPANSION
#define TRONE_BASEADDR      0x30 /* 7-bit address */
#define TREVO_BASEADDR      0x31 /* 7-bit address */
#define TERARANGER_DEVICE_PATH   	"/dev/teraranger"

/* TERARANGER Registers addresses */

#define TERARANGER_MEASURE_REG	0x00		/* Measure range register */
#define TERARANGER_WHO_AM_I_REG  0x01        /* Who am I test register */
#define TERARANGER_WHO_AM_I_REG_VAL 0xA1


/* Device limits */
#define TRONE_MIN_DISTANCE (0.20f)
#define TRONE_MAX_DISTANCE (14.00f)
#define TREVO_60M_MIN_DISTANCE (0.50f)
#define TREVO_60M_MAX_DISTANCE (60.0f)
#define TREVO_600HZ_MIN_DISTANCE (0.75f)
#define TREVO_600HZ_MAX_DISTANCE (8.0f)

#define TERARANGER_CONVERSION_INTERVAL 50000 /* 50ms */

class TERARANGER : public device::I2C, public px4::ScheduledWorkItem
{
public:
	TERARANGER(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING,
		   int bus = TERARANGER_BUS_DEFAULT, int address = TRONE_BASEADDR);
	virtual ~TERARANGER();

	virtual int 		init() override;

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen) override;
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe() override;

private:
	uint8_t _rotation;
	float				_min_distance;
	float				_max_distance;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	uint8_t				_valid;
	int				_measure_interval;
	bool				_collect_phase;
	int				_class_instance;
	int				_orb_class_instance;

	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults TRONE_MIN_DISTANCE
	* and TRONE_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void					Run() override;
	int					measure();
	int					collect();

};

static const uint8_t crc_table[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
	0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
	0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
	0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
	0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
	0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
	0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
	0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
	0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
	0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
	0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
	0xfa, 0xfd, 0xf4, 0xf3
};

static uint8_t crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

	while (len--) {
		i = (crc ^ *p++) & 0xFF;
		crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
	}

	return crc & 0xFF;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int teraranger_main(int argc, char *argv[]);

TERARANGER::TERARANGER(uint8_t rotation, int bus, int address) :
	I2C("TERARANGER", TERARANGER_DEVICE_PATH, bus, address, 100000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_rotation(rotation),
	_min_distance(-1.0f),
	_max_distance(-1.0f),
	_reports(nullptr),
	_sensor_ok(false),
	_valid(0),
	_measure_interval(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "tr1_read")),
	_comms_errors(perf_alloc(PC_COUNT, "tr1_com_err"))
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
}

TERARANGER::~TERARANGER()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
TERARANGER::init()
{
	int ret = PX4_ERROR;
	int hw_model;
	param_get(param_find("SENS_EN_TRANGER"), &hw_model);

	switch (hw_model) {
	case 0: /* Disabled */
		PX4_WARN("Disabled");
		return ret;

	case 1: /* Autodetect */
		/* Assume TROne */
		set_device_address(TRONE_BASEADDR);

		if (I2C::init() != OK) {
			set_device_address(TREVO_BASEADDR);

			if (I2C::init() != OK) {
				goto out;

			} else {
				_min_distance = TREVO_60M_MIN_DISTANCE;
				_max_distance = TREVO_60M_MAX_DISTANCE;
			}

		} else {
			_min_distance = TRONE_MIN_DISTANCE;
			_max_distance = TRONE_MAX_DISTANCE;
		}

		break;

	case 2: /* TROne */
		set_device_address(TRONE_BASEADDR);

		if (I2C::init() != OK) {
			goto out;
		}

		_min_distance = TRONE_MIN_DISTANCE;
		_max_distance = TRONE_MAX_DISTANCE;
		break;

	case 3: /* TREvo60m */
		set_device_address(TREVO_BASEADDR);

		/* do I2C init (and probe) first */
		if (I2C::init() != OK) {
			goto out;
		}

		_min_distance = TREVO_60M_MIN_DISTANCE;
		_max_distance = TREVO_60M_MAX_DISTANCE;
		break;

	case 4: /* TREvo600Hz */
		set_device_address(TREVO_BASEADDR);

		/* do I2C init (and probe) first */
		if (I2C::init() != OK) {
			goto out;
		}

		_min_distance = TREVO_600HZ_MIN_DISTANCE;
		_max_distance = TREVO_600HZ_MAX_DISTANCE;
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* get a publish handle on the range finder topic */
		struct distance_sensor_s ds_report;
		measure();
		_reports->get(&ds_report);

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_LOW);

		if (_distance_sensor_topic == nullptr) {
			PX4_ERR("failed to create distance_sensor object");
		}
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;
out:
	return ret;
}

int
TERARANGER::probe()
{
	uint8_t who_am_i = 0;

	const uint8_t cmd = TERARANGER_WHO_AM_I_REG;

	// can't use a single transfer as Teraranger needs a bit of time for internal processing
	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		if (transfer(nullptr, 0, &who_am_i, 1) == OK && who_am_i == TERARANGER_WHO_AM_I_REG_VAL) {
			return measure();
		}
	}

	PX4_DEBUG("WHO_AM_I byte mismatch 0x%02x should be 0x%02x\n",
		  (unsigned)who_am_i,
		  TERARANGER_WHO_AM_I_REG_VAL);

	// not found on any address
	return -EIO;
}

void
TERARANGER::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
TERARANGER::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
TERARANGER::get_minimum_distance()
{
	return _min_distance;
}

float
TERARANGER::get_maximum_distance()
{
	return _max_distance;
}

int
TERARANGER::ioctl(device::file_t *filp, int cmd, unsigned long arg)
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
					_measure_interval = (TERARANGER_CONVERSION_INTERVAL);

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
					if (interval < (TERARANGER_CONVERSION_INTERVAL)) {
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
TERARANGER::read(device::file_t *filp, char *buffer, size_t buflen)
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
		px4_usleep(TERARANGER_CONVERSION_INTERVAL);

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
TERARANGER::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	const uint8_t cmd = TERARANGER_MEASURE_REG;
	ret = transfer(&cmd, sizeof(cmd), nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
TERARANGER::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[3] = {0, 0, 0};

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 3);

	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_mm = (val[0] << 8) | val[1];
	float distance_m = float(distance_mm) *  1e-3f;
	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	/* there is no enum item for a combined LASER and ULTRASOUND which it should be */
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = _rotation;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.variance = 0.0f;
	report.signal_quality = -1;
	/* TODO: set proper ID */
	report.id = 0;

	// This validation check can be used later
	_valid = crc8(val, 2) == val[2] && (float)report.current_distance > report.min_distance
		 && (float)report.current_distance < report.max_distance ? 1 : 0;

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
TERARANGER::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
TERARANGER::stop()
{
	ScheduleClear();
}

void
TERARANGER::Run()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > TERARANGER_CONVERSION_INTERVAL) {
			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - TERARANGER_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(TERARANGER_CONVERSION_INTERVAL);
}

void
TERARANGER::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace teraranger
{

TERARANGER	*g_dev;

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
	g_dev = new TERARANGER(rotation, i2c_bus);


	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(TERARANGER_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	px4_close(fd);
	return PX4_OK;

fail:

	if (fd >= 0) {
		px4_close(fd);
	}

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

	int fd = px4_open(TERARANGER_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'teraranger start' if the driver is not running)", TERARANGER_DEVICE_PATH);
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

	/* read the sensor 50x and report each value */
	for (unsigned i = 0; i < 50; i++) {
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
	int fd = px4_open(TERARANGER_DEVICE_PATH, O_RDONLY);

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

} // namespace


static void
teraranger_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for TeraRanger rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_TRANGER.

Setup/usage information: https://docs.px4.io/en/sensor/rangefinders.html#teraranger-rangefinders

### Examples

Start driver on any bus (start on bus where first sensor found).
$ teraranger start -a
Start driver on specified bus
$ teraranger start -b 1
Stop driver
$ teraranger stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("teraranger", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses (first one found)", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset","Reset driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info","Print driver information");



}

int
teraranger_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	bool start_all = false;

	int i2c_bus = TERARANGER_BUS_DEFAULT;

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
			return teraranger::start(rotation);

		} else {
			return teraranger::start_bus(rotation, i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return teraranger::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return teraranger::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return teraranger::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return teraranger::info();
	}

out_error:
	teraranger_usage();
	return PX4_ERROR;
}

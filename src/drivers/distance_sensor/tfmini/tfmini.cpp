/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file tfmini.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 * @author Christoph Tobler <christoph@px4.io>
 *
 * Driver for the Benewake TFmini laser rangefinder series
 */

#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

#include "tfmini_parser.h"

/* Configuration Constants */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class TFMINI : public device::CDev
{
public:
	TFMINI(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~TFMINI();

	virtual int init();

	virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

private:
	char                     _port[20];
	uint8_t                  _rotation;
	float                    _min_distance;
	float                    _max_distance;
	int                      _conversion_interval;
	work_s                   _work;
	ringbuffer::RingBuffer  *_reports;
	int                      _measure_ticks;
	bool                     _collect_phase;
	int                      _fd;
	char                     _linebuf[10];
	unsigned                 _linebuf_index;
	enum TFMINI_PARSE_STATE  _parse_state;

	hrt_abstime              _last_read;

	int                      _class_instance;
	int                      _orb_class_instance;

	orb_advert_t             _distance_sensor_topic;

	unsigned                 _consecutive_fail_count;

	perf_counter_t           _sample_perf;
	perf_counter_t           _comms_errors;

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
	* range to be brought in at all, otherwise it will use the defaults TFMINI_MIN_DISTANCE
	* and TFMINI_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int				collect();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int tfmini_main(int argc, char *argv[]);

TFMINI::TFMINI(const char *port, uint8_t rotation) :
	CDev("tfmini", RANGE_FINDER0_DEVICE_PATH),
	_rotation(rotation),
	_min_distance(0.30f),
	_max_distance(12.0f),
	_conversion_interval(10000),
	_reports(nullptr),
	_measure_ticks(0),
	_collect_phase(false),
	_fd(-1),
	_linebuf_index(0),
	_parse_state(TFMINI_PARSE_STATE0_UNSYNC),
	_last_read(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "tfmini_read")),
	_comms_errors(perf_alloc(PC_COUNT, "tfmini_com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// disable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

TFMINI::~TFMINI()
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

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
TFMINI::init()
{
	int32_t hw_model;
	param_get(param_find("SENS_EN_TFMINI"), &hw_model);

	switch (hw_model) {
	case 0:
		DEVICE_LOG("disabled.");
		return 0;

	case 1: /* TFMINI (12m, 100 Hz)*/
		_min_distance = 0.3f;
		_max_distance = 12.0f;
		_conversion_interval =	10000;
		break;

	default:
		DEVICE_LOG("invalid HW model %d.", hw_model);
		return -1;
	}

	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_SYNC);

		if (_fd < 0) {
			warnx("Error opening fd");
			return -1;
		}

		/*baudrate 115200, 8 bits, no parity, 1 stop bit */
		unsigned speed = B115200;

		struct termios uart_config;

		int termios_state;

		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			warnx("ERR CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			warnx("ERR CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			warnx("ERR baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;         /* 8-bit characters */
		uart_config.c_cflag &= ~PARENB;     /* no parity bit */
		uart_config.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
		uart_config.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

		/* setup for non-canonical mode */
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		/* fetch bytes as they become available */
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			warnx("FAIL: laser fd");
			ret = -1;
			break;
		}

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

		if (_reports == nullptr) {
			warnx("mem err");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		/* get a publish handle on the range finder topic */
		struct distance_sensor_s ds_report = {};

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
		}

	} while (0);

	/* close the fd */
	::close(_fd);
	_fd = -1;

	return ret;
}

void
TFMINI::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
TFMINI::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
TFMINI::get_minimum_distance()
{
	return _min_distance;
}

float
TFMINI::get_maximum_distance()
{
	return _max_distance;
}

int
TFMINI::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {

					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			ATOMIC_ENTER;

			if (!_reports->resize(arg)) {
				ATOMIC_LEAVE;
				return -ENOMEM;
			}

			ATOMIC_LEAVE;

			return OK;
		}

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
TFMINI::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

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

		/* wait for it to complete */
		usleep(_conversion_interval);

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
TFMINI::collect()
{
	int	ret;

	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	uint64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	/* read from the sensor (uart buffer) */
	ret = ::read(_fd, &readbuf[0], readlen);

	if (ret < 0) {
		DEVICE_DEBUG("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_conversion_interval * 2)) {
			return ret;

		} else {
			return -EAGAIN;
		}

	} else if (ret == 0) {
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();

	float distance_m = -1.0f;
	bool valid = false;

	for (int i = 0; i < ret; i++) {
		if (OK == tfmini_parser(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &distance_m)) {
			valid = true;
		}
	}

	if (!valid) {
		return -EAGAIN;
	}

	DEVICE_DEBUG("val (float): %8.4f, raw: %s, valid: %s", (double)distance_m, _linebuf, ((valid) ? "OK" : "NO"));

	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = _rotation;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it */
	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
TFMINI::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&TFMINI::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_RANGEFINDER;

	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);


	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);

	}
}

void
TFMINI::stop()
{
	work_cancel(HPWORK, &_work);
}

void
TFMINI::cycle_trampoline(void *arg)
{
	TFMINI *dev = (TFMINI *)arg;

	dev->cycle();
}

void
TFMINI::cycle()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_SYNC);
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&TFMINI::cycle_trampoline,
				   this,
				   USEC2TICK(1042 * 8));
			return;
		}

		if (OK != collect_ret) {

			/* we know the sensor needs about four seconds to initialize */
			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
				DEVICE_LOG("collection error #%u", _consecutive_fail_count);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			start();
			return;

		} else {
			/* apparently success */
			_consecutive_fail_count = 0;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(_conversion_interval)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&TFMINI::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(_conversion_interval));

			return;
		}
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&TFMINI::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));
}

void
TFMINI::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %d ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace tfmini
{

TFMINI	*g_dev;

void start(const char *port, uint8_t rotation);
void stop();
void test();
void reset();
void info();
void usage();

/**
 * Start the driver.
 */
void
start(const char *port, uint8_t rotation)
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new TFMINI(port, rotation);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		warnx("Opening device '%s' failed");
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		warnx("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		warnx("driver stopped");

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct distance_sensor_s report;
	ssize_t sz;

	int fd = px4_open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'tfmini start' if the driver is not running", RANGE_FINDER0_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = px4_read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* start the sensor polling at 2 Hz rate */
	if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		px4_pollfd_struct_t fds{};

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			warnx("timed out");
			break;
		}

		/* now go get it */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warnx("read failed: got %d vs exp. %d", sz, sizeof(report));
			break;
		}

		print_message(report);
	}

	/* reset the sensor polling to the default rate */
	if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "ERR: DEF RATE");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = px4_open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a little info on how to use the driver.
 */
void
usage()
{
	printf("usage:\n");
	printf("tfmini start -d <device path> -R (optional) <rotation>:\n");
}

} // namespace

int
tfmini_main(int argc, char *argv[])
{
	// check for optional arguments
	int ch;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = "";
	int myoptind = 1;
	const char *myoptarg = nullptr;


	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting distance sensor orientation to %d", (int)rotation);
			break;

		case 'd':
			device_path = myoptarg;
			PX4_INFO("Using device path '%s'", device_path);
			break;

		default:
			PX4_WARN("Unknown option!");
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			tfmini::start(device_path, rotation);

		} else {
			PX4_WARN("Please specify device path!");
			tfmini::usage();
			return PX4_ERROR;
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		tfmini::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		tfmini::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		tfmini::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[1], "status")) {
		tfmini::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}

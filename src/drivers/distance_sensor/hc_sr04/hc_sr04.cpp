/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file hc_sr04.cpp
 *
 * Driver for the hc_sr04 sonar range finders .
 */

#include <px4_config.h>
#include <px4_workqueue.h>
#include <drivers/device/device.h>
#include <px4_defines.h>
#include <containers/Array.hpp>

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

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#define SR04_MAX_RANGEFINDERS 6
#define SR04_ID_BASE	 0x10

/* Configuration Constants */
#define SR04_DEVICE_PATH	"/dev/hc_sr04"

/* Device limits */
#define SR04_MIN_DISTANCE 	(0.10f)
#define SR04_MAX_DISTANCE 	(4.00f)

#define SR04_CONVERSION_INTERVAL 	100000 /* 100ms for one sonar */


class HC_SR04 : public cdev::CDev
{
public:
	HC_SR04();
	virtual ~HC_SR04();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();
	void                            interrupt(unsigned time);

protected:
	virtual int			probe();

private:
	float				_min_distance;
	float				_max_distance;
	ringbuffer::RingBuffer	*_reports;
	bool				_sensor_ok;
	int					_measure_interval;
	bool				_collect_phase;
	int					_class_instance;
	int					_orb_class_instance;

	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int					_cycling_rate;	/* */
	uint8_t				_index_counter;	/* temporary sonar i2c address */

	px4::Array<float, 6>
	_latest_sonar_measurements; /* vector to store latest sonar measurements in before writing to report */
	unsigned 		_sonars{6};
	struct GPIOConfig {
		uint32_t        trig_port;
		uint32_t        echo_port;
		uint32_t        alt;
	};
	static const GPIOConfig _gpio_tab[];
	unsigned 		_raising_time;
	unsigned 		_falling_time;
	unsigned 		_status;
	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
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
	* range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
	* and MB12XX_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				Run() override;
	int					measure();
	int					collect();

};

const HC_SR04::GPIOConfig HC_SR04::_gpio_tab[] = {
	{GPIO_GPIO6_OUTPUT,      GPIO_GPIO7_INPUT,       0},
	{GPIO_GPIO6_OUTPUT,      GPIO_GPIO8_INPUT,       0},
	{GPIO_GPIO6_OUTPUT,      GPIO_GPIO9_INPUT,       0},
	{GPIO_GPIO6_OUTPUT,      GPIO_GPIO10_INPUT,       0},
	{GPIO_GPIO6_OUTPUT,      GPIO_GPIO11_INPUT,       0},
	{GPIO_GPIO6_OUTPUT,      GPIO_GPIO12_INPUT,       0}
};

/*
 * Driver 'main' command.
 */
extern "C"  __EXPORT int hc_sr04_main(int argc, char *argv[]);
static int sonar_isr(int irq, void *context);

HC_SR04::HC_SR04() :
	CDev(SR04_DEVICE_PATH, 0),
	_min_distance(SR04_MIN_DISTANCE),
	_max_distance(SR04_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_interval(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "hc_sr04_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hc_sr04_comms_errors")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0), 	/* initialising temp sonar i2c address to zero */
	_sonars(sonars),
	_raising_time(0),
	_falling_time(0),
	_status(0)

{
}

HC_SR04::~HC_SR04()
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

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
HC_SR04::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	/* init echo port : */
	for (unsigned i = 0; i <= _sonars; i++) {
		px4_arch_configgpio(_gpio_tab[i].trig_port);
		px4_arch_gpiowrite(_gpio_tab[i].trig_port, false);
		px4_arch_configgpio(_gpio_tab[i].echo_port);
		_latest_sonar_measurements.push_back(0);
	}

	usleep(200000); /* wait for 200ms; */

	_cycling_rate = SR04_CONVERSION_INTERVAL;

	/* show the connected sonars in terminal */
	PX4_DEBUG("Number of sonars set: %d", _sonars);

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

int
HC_SR04::probe()
{
	return (OK);
}

void
HC_SR04::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
HC_SR04::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
HC_SR04::get_minimum_distance()
{
	return _min_distance;
}

float
HC_SR04::get_maximum_distance()
{
	return _max_distance;
}
void
HC_SR04::interrupt(unsigned time)
{
	if (_status == 0) {
		_raising_time = time;
		_status++;
		return;

	} else if (_status == 1) {
		_falling_time = time;
		_status++;
		return;
	}

	return;
}

int
HC_SR04::ioctl(device::file_t *filp, int cmd, unsigned long arg)
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
					_measure_interval = (_cycling_rate);

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
					if (interval < (_cycling_rate)) {
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
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
HC_SR04::read(device::file_t *filp, char *buffer, size_t buflen)
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
		usleep(_cycling_rate * 2);

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
HC_SR04::measure()
{

	int ret;
	/*
	 * Send a plus begin a measurement.
	 */
	px4_arch_gpiowrite(_gpio_tab[_cycle_counter].trig_port, true);
	usleep(10);  // 10us
	px4_arch_gpiowrite(_gpio_tab[_cycle_counter].trig_port, false);

	px4_arch_gpiosetevent(_gpio_tab[_cycle_counter].echo_port, true, true, false, sonar_isr);
	_status = 0;
	ret = OK;

	return ret;
}

int
HC_SR04::collect()
{
	int	ret = -EIO;
#if 0
	perf_begin(_sample_perf);

	/* read from the sensor */
	if (_status != 2) {
		PX4_DEBUG("erro sonar %d ,status=%d", _cycle_counter, _status);
		px4_arch_gpiosetevent(_gpio_tab[_cycle_counter].echo_port, true, true, false, nullptr);
		perf_end(_sample_perf);
		return (ret);
	}

	unsigned  distance_time = _falling_time - _raising_time ;

	float si_units = (distance_time * 0.000170f) ; /* meter */
	struct distance_sensor_s report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	/* if only one sonar, write it to the original distance parameter so that it's still used as altitude sonar */
	if (_sonars == 1) {
		report.distance = si_units;

		for (unsigned i = 0; i < (SRF02_MAX_RANGEFINDERS); i++) {
			report.id[i] = 0;
			report.distance_vector[i] = 0;
		}

		report.id[0] = SR04_ID_BASE;
		report.distance_vector[0] = si_units; //  将测量值填入向量中，适应test()的要求
		report.just_updated = 1;

	} else {
		/* for multiple sonars connected */

		_latest_sonar_measurements[_cycle_counter] = si_units;
		report.just_updated = 0;

		for (unsigned i = 0; i < SRF02_MAX_RANGEFINDERS; i++) {
			if (i < _sonars) {
				report.distance_vector[i] = _latest_sonar_measurements[i];
				report.id[i] = SR04_ID_BASE + i;
				report.just_updated++;

			} else {
				report.distance_vector[i] = 0;
				report.id[i] = 0;
			}

		}

		report.distance =  _latest_sonar_measurements[0]; //
	}

	report.minimum_distance = get_minimum_distance();
	report.maximum_distance = get_maximum_distance();
	report.valid = si_units > get_minimum_distance() && si_units < get_maximum_distance() ? 1 : 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	px4_arch_gpiosetevent(_gpio_tab[_cycle_counter].echo_port, true, true, false, nullptr); /* close interrupt */
	perf_end(_sample_perf);
#endif
	return ret;
}

void
HC_SR04::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	measure();  /* begin measure */

	/* schedule a cycle to start things */
	ScheduleDelayed(_cycling_rate);
}

void
HC_SR04::stop()
{
	ScheduleClear();
}

void
HC_SR04::Run()
{
	/*_circle_count 计录当前sonar　*/
	/* perform collection */
	if (OK != collect()) {
		PX4_DEBUG("collection error");
	}

	/* change to next sonar */
	_cycle_counter = _cycle_counter + 1;

	if (_cycle_counter >= _sonars) {
		_cycle_counter = 0;
	}

	/* 测量next sonar */
	if (OK != measure()) {
		PX4_DEBUG("measure error sonar adress %d", _cycle_counter);
	}

	ScheduleDelayed(_cycling_rate);
}

void
HC_SR04::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u \n", _measure_interval);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace  hc_sr04
{

HC_SR04	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new HC_SR04();

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SR04_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
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
		delete g_dev;
		g_dev = nullptr;

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
#if 0
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = open(SR04_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'hc_sr04 start' if the driver is not running", SR04_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		print_message(report);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
#endif
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SR04_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
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

} /* namespace */

static int sonar_isr(int irq, void *context)
{
	unsigned time = hrt_absolute_time();
	/* ack the interrupts we just read */

	if (hc_sr04::g_dev != nullptr) {
		hc_sr04::g_dev->interrupt(time);
	}

	return OK;
}



int
hc_sr04_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		hc_sr04::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		hc_sr04::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		hc_sr04::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		hc_sr04::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		hc_sr04::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}

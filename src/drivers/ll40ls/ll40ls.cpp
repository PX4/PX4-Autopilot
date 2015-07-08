/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file ll40ls.cpp
 * @author Allyson Kreft
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via I2C.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

/* Configuration Constants */
#define LL40LS_BUS 			PX4_I2C_BUS_EXPANSION
#define LL40LS_BASEADDR 	0x62 /* 7-bit address */
#define LL40LS_BASEADDR_OLD 	0x42 /* previous 7-bit address */
#define LL40LS_DEVICE_PATH_INT 	"/dev/ll40ls_int"
#define LL40LS_DEVICE_PATH_EXT 	"/dev/ll40ls_ext"

/* LL40LS Registers addresses */

#define LL40LS_MEASURE_REG		0x00		/* Measure range register */
#define LL40LS_MSRREG_RESET	    0x00		/* reset to power on defaults */
#define LL40LS_MSRREG_ACQUIRE	    0x04		/* Value to initiate a measurement, varies based on sensor revision */
#define LL40LS_MAX_ACQ_COUNT_REG       0x02		/* maximum acquisition count register */
#define LL40LS_DISTHIGH_REG		0x8F		/* High byte of distance register, auto increment */
#define LL40LS_WHO_AM_I_REG         0x11
#define LL40LS_WHO_AM_I_REG_VAL         0xCA
#define LL40LS_SIGNAL_STRENGTH_REG  0x5b

/* Device limits */
#define LL40LS_MIN_DISTANCE (0.00f)
#define LL40LS_MAX_DISTANCE (60.00f)

// normal conversion wait time
#define LL40LS_CONVERSION_INTERVAL 50*1000UL /* 50ms */

// maximum time to wait for a conversion to complete.
#define LL40LS_CONVERSION_TIMEOUT 100*1000UL /* 100ms */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class LL40LS : public device::I2C
{
public:
	LL40LS(int bus, const char *path, int address = LL40LS_BASEADDR);
	virtual ~LL40LS();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

	/**
	 * print registers to console
	 */
	void					print_registers();

protected:
	virtual int			probe();
	virtual int			read_reg(uint8_t reg, uint8_t &val);

private:
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	RingBuffer			*_reports;
	bool				_sensor_ok;
	unsigned			_measure_ticks;
	bool				_collect_phase;
	int					_class_instance;

	orb_advert_t		_range_finder_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	perf_counter_t		_sensor_resets;
	perf_counter_t		_sensor_zero_resets;
	uint16_t		_last_distance;
	uint16_t		_zero_counter;
	uint64_t		_acquire_time_usec;
	volatile bool		_pause_measurements;

	/**< the bus the device is connected to */
	int			_bus;

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
	* range to be brought in at all, otherwise it will use the defaults LL40LS_MIN_DISTANCE
	* and LL40LS_MAX_DISTANCE
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
	int					measure();
	int					collect();
	int					reset_sensor();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void		cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ll40ls_main(int argc, char *argv[]);

LL40LS::LL40LS(int bus, const char *path, int address) :
	I2C("LL40LS", path, bus, address, 100000),
	_min_distance(LL40LS_MIN_DISTANCE),
	_max_distance(LL40LS_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_range_finder_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "ll40ls_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ll40ls_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "ll40ls_buffer_overflows")),
	_sensor_resets(perf_alloc(PC_COUNT, "ll40ls_resets")),
	_sensor_zero_resets(perf_alloc(PC_COUNT, "ll40ls_zero_resets")),
	_last_distance(0),
	_zero_counter(0),
	_pause_measurements(false),
	_bus(bus)
{
	// up the retries since the device misses the first measure attempts
	_retries = 3;

	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

LL40LS::~LL40LS()
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
	perf_free(_buffer_overflows);
	perf_free(_sensor_resets);
	perf_free(_sensor_zero_resets);
}

int
LL40LS::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(range_finder_report));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* get a publish handle on the range finder topic */
		struct range_finder_report rf_report;
		measure();
		_reports->get(&rf_report);
		_range_finder_topic = orb_advertise(ORB_ID(sensor_range_finder), &rf_report);

		if (_range_finder_topic < 0) {
			debug("failed to create sensor_range_finder object. Did you start uOrb?");
		}
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;
out:
	return ret;
}

int
LL40LS::read_reg(uint8_t reg, uint8_t &val)
{
	return transfer(&reg, 1, &val, 1);
}

int
LL40LS::probe()
{
	// cope with both old and new I2C bus address
	const uint8_t addresses[2] = {LL40LS_BASEADDR, LL40LS_BASEADDR_OLD};

	// more retries for detection
	_retries = 10;

	for (uint8_t i=0; i<sizeof(addresses); i++) {
		uint8_t who_am_i=0, max_acq_count=0;

		// set the I2C bus address
		set_address(addresses[i]);

		/* register 2 defaults to 0x80. If this matches it is
		   almost certainly a ll40ls */
		if (read_reg(LL40LS_MAX_ACQ_COUNT_REG, max_acq_count) == OK && max_acq_count == 0x80) {
			// very likely to be a ll40ls. This is the
			// default max acquisition counter
			goto ok;
		}

		if (read_reg(LL40LS_WHO_AM_I_REG, who_am_i) == OK && who_am_i == LL40LS_WHO_AM_I_REG_VAL) {
			// it is responding correctly to a
			// WHO_AM_I. This works with older sensors (pre-production)
			goto ok;
		}

		debug("probe failed reg11=0x%02x reg2=0x%02x\n",
		      (unsigned)who_am_i, 
		      (unsigned)max_acq_count);
	}

	// not found on any address
	return -EIO;

ok:
	_retries = 3;

	// reset the sensor to ensure it is in a known state with
	// correct settings
	return reset_sensor();
}

void
LL40LS::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
LL40LS::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
LL40LS::get_minimum_distance()
{
	return _min_distance;
}

float
LL40LS::get_maximum_distance()
{
	return _max_distance;
}

int
LL40LS::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(LL40LS_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(LL40LS_CONVERSION_INTERVAL)) {
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

			irqstate_t flags = irqsave();

			if (!_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		reset_sensor();
		return OK;

	case RANGEFINDERIOCSETMINIUMDISTANCE: {
			set_minimum_distance(*(float *)arg);
			return 0;
		}
		break;

	case RANGEFINDERIOCSETMAXIUMDISTANCE: {
			set_maximum_distance(*(float *)arg);
			return 0;
		}
		break;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
LL40LS::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct range_finder_report);
	struct range_finder_report *rbuf = reinterpret_cast<struct range_finder_report *>(buffer);
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

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(LL40LS_CONVERSION_INTERVAL);

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
LL40LS::measure()
{
	int ret;

	if (_pause_measurements) {
		// we are in print_registers() and need to avoid
		// acquisition to keep the I2C peripheral on the
		// sensor active
		return OK;
	}

	/*
	 * Send the command to begin a measurement.
	 */
	const uint8_t cmd[2] = { LL40LS_MEASURE_REG, LL40LS_MSRREG_ACQUIRE };
	ret = transfer(cmd, sizeof(cmd), nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		if (perf_event_count(_comms_errors) % 10 == 0) {
			perf_count(_sensor_resets);
			reset_sensor();
		}
		return ret;
	}

	// remember when we sent the acquire so we can know when the
	// acquisition has timed out
	_acquire_time_usec = hrt_absolute_time();
	ret = OK;

	return ret;
}

/*
  reset the sensor to power on defaults
 */
int
LL40LS::reset_sensor()
{
	const uint8_t cmd[2] = { LL40LS_MEASURE_REG, LL40LS_MSRREG_RESET };
	int ret = transfer(cmd, sizeof(cmd), nullptr, 0);
	return ret;
}

/*
  dump sensor registers for debugging
 */
void
LL40LS::print_registers()
{
	_pause_measurements = true;
	printf("ll40ls registers\n");
	// wait for a while to ensure the lidar is in a ready state
	usleep(50000);
	for (uint8_t reg=0; reg<=0x67; reg++) {
		uint8_t val = 0;
		int ret = transfer(&reg, 1, &val, 1);
		if (ret != OK) {
			printf("%02x:XX ",(unsigned)reg);
		} else {
			printf("%02x:%02x ",(unsigned)reg, (unsigned)val);
		}
		if (reg % 16 == 15) {
			printf("\n");
		}
	}
	printf("\n");
	_pause_measurements = false;
}

int
LL40LS::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	// read the high and low byte distance registers
	uint8_t distance_reg = LL40LS_DISTHIGH_REG;
	ret = transfer(&distance_reg, 1, &val[0], sizeof(val));

	if (ret < 0) {
		if (hrt_absolute_time() - _acquire_time_usec > LL40LS_CONVERSION_TIMEOUT) {
			/*
			  NACKs from the sensor are expected when we
			  read before it is ready, so only consider it
			  an error if more than 100ms has elapsed.
			 */
			debug("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			if (perf_event_count(_comms_errors) % 10 == 0) {
				perf_count(_sensor_resets);
				reset_sensor();
			}
		}
		perf_end(_sample_perf);
		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		return ret;
	}

	uint16_t distance = (val[0] << 8) | val[1];
	float si_units = distance * 0.01f; /* cm to m */
	struct range_finder_report report;

	if (distance == 0) {
		_zero_counter++;
		if (_zero_counter == 20) {
			/* we have had 20 zeros in a row - reset the
			   sensor. This is a known bad state of the
			   sensor where it returns 16 bits of zero for
			   the distance with a trailing NACK, and
			   keeps doing that even when the target comes
			   into a valid range.
			*/
			_zero_counter = 0;
			perf_end(_sample_perf);
			perf_count(_sensor_zero_resets);
			return reset_sensor();
		}
	} else {
		_zero_counter = 0;
	}

	_last_distance = distance;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.distance = si_units;
	report.minimum_distance = get_minimum_distance();
	report.maximum_distance = get_maximum_distance();
	if (si_units > get_minimum_distance() && si_units < get_maximum_distance()) {
		report.valid = 1;
	}
	else {
		report.valid = 0;
	}

	/* publish it, if we are the primary */
	if (_range_finder_topic >= 0) {
		orb_publish(ORB_ID(sensor_range_finder), _range_finder_topic, &report);
	}

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
LL40LS::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LL40LS::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		subsystem_info_s::SUBSYSTEM_TYPE_RANGEFINDER
	};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
LL40LS::stop()
{
	work_cancel(HPWORK, &_work);
}

void
LL40LS::cycle_trampoline(void *arg)
{
	LL40LS *dev = (LL40LS *)arg;

	dev->cycle();
}

void
LL40LS::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* try a collection */
		if (OK != collect()) {
			debug("collection error");
			/* if we've been waiting more than 200ms then
			   send a new acquire */
			if (hrt_absolute_time() - _acquire_time_usec > LL40LS_CONVERSION_TIMEOUT*2) {
				_collect_phase = false;
			}
		} else {
			/* next phase is measurement */
			_collect_phase = false;

			/*
			 * Is there a collect->measure gap?
			 */
			if (_measure_ticks > USEC2TICK(LL40LS_CONVERSION_INTERVAL)) {
				
				/* schedule a fresh cycle call when we are ready to measure again */
				work_queue(HPWORK,
					   &_work,
					   (worker_t)&LL40LS::cycle_trampoline,
					   this,
					   _measure_ticks - USEC2TICK(LL40LS_CONVERSION_INTERVAL));
				
				return;
			}
		}
	}

	if (_collect_phase == false) {
		/* measurement phase */
		if (OK != measure()) {
			debug("measure error");
		} else {
			/* next phase is collection. Don't switch to
			   collection phase until we have a successful
			   acquire request I2C transfer */
			_collect_phase = true;
		}
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&LL40LS::cycle_trampoline,
		   this,
		   USEC2TICK(LL40LS_CONVERSION_INTERVAL));
}

void
LL40LS::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	perf_print_counter(_sensor_resets);
	perf_print_counter(_sensor_zero_resets);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
	printf("distance: %ucm (0x%04x)\n",
	       (unsigned)_last_distance, (unsigned)_last_distance);
}

/**
 * Local functions in support of the shell command.
 */
namespace ll40ls
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

LL40LS	*g_dev_int;
LL40LS	*g_dev_ext;

void	start(int bus);
void	stop(int bus);
void	test(int bus);
void	reset(int bus);
void	info(int bus);
void    regdump(int bus);
void	usage();

/**
 * Start the driver.
 */
void
start(int bus)
{
	/* create the driver, attempt expansion bus first */
	if (bus == -1 || bus == PX4_I2C_BUS_EXPANSION) {
		if (g_dev_ext != nullptr)
			errx(0, "already started external");
		g_dev_ext = new LL40LS(PX4_I2C_BUS_EXPANSION, LL40LS_DEVICE_PATH_EXT);
		if (g_dev_ext != nullptr && OK != g_dev_ext->init()) {
			delete g_dev_ext;
			g_dev_ext = nullptr;
			if (bus == PX4_I2C_BUS_EXPANSION) {
				goto fail;
			}
		}
	}

#ifdef PX4_I2C_BUS_ONBOARD
	/* if this failed, attempt onboard sensor */
	if (bus == -1 || bus == PX4_I2C_BUS_ONBOARD) {
		if (g_dev_int != nullptr)
			errx(0, "already started internal");
		g_dev_int = new LL40LS(PX4_I2C_BUS_ONBOARD, LL40LS_DEVICE_PATH_INT);
		if (g_dev_int != nullptr && OK != g_dev_int->init()) {
			/* tear down the failing onboard instance */
			delete g_dev_int;
			g_dev_int = nullptr;

			if (bus == PX4_I2C_BUS_ONBOARD) {
				goto fail;
			}
		}
		if (g_dev_int == nullptr && bus == PX4_I2C_BUS_ONBOARD) {
			goto fail;
		}
	}
#endif

	/* set the poll rate to default, starts automatic data collection */
	if (g_dev_int != nullptr) {
		int fd = open(LL40LS_DEVICE_PATH_INT, O_RDONLY);
                if (fd == -1) {
                    goto fail;
                }
		int ret = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
		close(fd);
		if (ret < 0) {
			goto fail;
		}
        }

	if (g_dev_ext != nullptr) {
		int fd = open(LL40LS_DEVICE_PATH_EXT, O_RDONLY);
                if (fd == -1) {
                    goto fail;
                }
		int ret = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
		close(fd);
		if (ret < 0) {
			goto fail;
		}
        }

	exit(0);

fail:
	if (g_dev_int != nullptr && (bus == -1 || bus == PX4_I2C_BUS_ONBOARD)) {
		delete g_dev_int;
		g_dev_int = nullptr;
	}
	if (g_dev_ext != nullptr && (bus == -1 || bus == PX4_I2C_BUS_EXPANSION)) {
		delete g_dev_ext;
		g_dev_ext = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop(int bus)
{
	LL40LS **g_dev = (bus == PX4_I2C_BUS_ONBOARD?&g_dev_int:&g_dev_ext);
	if (*g_dev != nullptr) {
		delete *g_dev;
		*g_dev = nullptr;

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
test(int bus)
{
	struct range_finder_report report;
	ssize_t sz;
	int ret;
	const char *path = (bus==PX4_I2C_BUS_ONBOARD?LL40LS_DEVICE_PATH_INT:LL40LS_DEVICE_PATH_EXT);

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'll40ls start' if the driver is not running", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.distance);
	warnx("time:        %lld", report.timestamp);

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

		warnx("periodic read %u", i);
		warnx("measurement: %0.3f", (double)report.distance);
		warnx("time:        %lld", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(int bus)
{
	const char *path = (bus==PX4_I2C_BUS_ONBOARD?LL40LS_DEVICE_PATH_INT:LL40LS_DEVICE_PATH_EXT);
	int fd = open(path, O_RDONLY);

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
info(int bus)
{
	LL40LS *g_dev = (bus == PX4_I2C_BUS_ONBOARD?g_dev_int:g_dev_ext);
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Dump registers
 */
void
regdump(int bus)
{
	LL40LS *g_dev = (bus == PX4_I2C_BUS_ONBOARD?g_dev_int:g_dev_ext);
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'stop', 'info', 'test', 'reset', 'info' or 'regdump'");
	warnx("options:");
	warnx("    -X only external bus");
#ifdef PX4_I2C_BUS_ONBOARD
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
ll40ls_main(int argc, char *argv[])
{
	int ch;
	int bus = -1;

	while ((ch = getopt(argc, argv, "XI")) != EOF) {
		switch (ch) {
#ifdef PX4_I2C_BUS_ONBOARD
		case 'I':
			bus = PX4_I2C_BUS_ONBOARD;
			break;
#endif
		case 'X':
			bus = PX4_I2C_BUS_EXPANSION;
			break;
		default:
			ll40ls::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ll40ls::start(bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		ll40ls::stop(bus);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		ll40ls::test(bus);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		ll40ls::reset(bus);
	}

	/*
	 * dump registers
	 */
	if (!strcmp(verb, "regdump")) {
		ll40ls::regdump(bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		ll40ls::info(bus);
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info' or 'regdump'");
}

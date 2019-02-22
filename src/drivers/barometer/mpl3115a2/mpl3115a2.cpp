/****************************************************************************
 *
 *   Copyright (c) 2017-2018 PX4 Development Team. All rights reserved.
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
 * @file mpl3115a2.cpp
 *
 * Driver for the MPL3115A2 barometric pressure sensor connected via I2C.
 */

#include <lib/cdev/CDev.hpp>
#include <drivers/device/Device.hpp>
#include <px4_config.h>
#include <px4_log.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>

#include <nuttx/arch.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include "mpl3115a2.h"



enum MPL3115A2_BUS {
	MPL3115A2_BUS_ALL = 0,
	MPL3115A2_BUS_I2C_INTERNAL,
	MPL3115A2_BUS_I2C_EXTERNAL,
};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * MPL3115A2 internal constants and data structures.
 */

#define MPL3115A2_CONVERSION_INTERVAL	10000	/* microseconds */
#define MPL3115A2_OSR                   2       /* Over Sample rate of 4 18MS Minimum time between data samples */
#define MPL3115A2_CTRL_TRIGGER          (CTRL_REG1_OST | CTRL_REG1_OS(MPL3115A2_OSR))
#define MPL3115A2_BARO_DEVICE_PATH_EXT  "/dev/mpl3115a2_ext"
#define MPL3115A2_BARO_DEVICE_PATH_INT  "/dev/mpl3115a2_int"

class MPL3115A2 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	MPL3115A2(device::Device *interface, const char *path);
	~MPL3115A2();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	device::Device			*_interface;

	unsigned		_measure_interval;

	ringbuffer::RingBuffer	*_reports;
	bool			_collect_phase;

	/*  */
	float			_P;
	float			_T;

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @param delay_us the number of microseconds before executing the next cycle
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start(unsigned delay_us = 1);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			Run() override;

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	virtual int		measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int mpl3115a2_main(int argc, char *argv[]);

MPL3115A2::MPL3115A2(device::Device *interface, const char *path) :
	CDev(path),
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_measure_interval(0),
	_reports(nullptr),
	_collect_phase(false),
	_P(0),
	_T(0),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpl3115a2_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "mpl3115a2_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "mpl3115a2_com_err"))
{
	_interface->set_device_type(DRV_BARO_DEVTYPE_MPL3115A2);
}

MPL3115A2::~MPL3115A2()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(get_devname(), _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
MPL3115A2::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		PX4_DEBUG("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));

	if (_reports == nullptr) {
		PX4_DEBUG("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	sensor_baro_s brp;
	_reports->flush();

	while (true) {

		/* First reading */
		do  {
			ret = measure();

			if (ret == -EAGAIN) {
				usleep(500);
			}
		} while (ret == -EAGAIN);

		if (ret != OK) {
			ret = -EIO;
			break;
		}

		/* First reading */
		do  {
			ret = collect();

			if (ret == -EAGAIN) {
				usleep(500);
			}
		} while (ret == -EAGAIN);

		if (ret != OK) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		_reports->get(&brp);

		// DEVICE_LOG("altitude (%u) = %.2f", _device_type, (double)brp.altitude);

		/* ensure correct devid */
		brp.device_id = _interface->get_device_id();

		ret = OK;

		_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
						  &_orb_class_instance, (_interface->external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

		if (_baro_topic == nullptr) {
			warnx("failed to create sensor_baro publication");
		}

		break;
	}

out:
	return ret;
}

ssize_t
MPL3115A2::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_baro_s);
	sensor_baro_s *brp = reinterpret_cast<sensor_baro_s *>(buffer);
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
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* First reading */
		do  {
			ret = measure();

			if (ret == -EAGAIN) {
				usleep(1000);
			}
		} while (ret == -EAGAIN);

		if (ret != OK) {
			ret = -EIO;
			break;
		}

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(brp)) {
			ret = sizeof(*brp);
		}

	} while (0);

	return ret;
}

int
MPL3115A2::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_interval = (MPL3115A2_CONVERSION_INTERVAL);

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
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < (MPL3115A2_CONVERSION_INTERVAL)) {
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

	case SENSORIOCRESET: {
			/*
			 * Since we are initialized, we do not need to do anything, since the
			 * PROM is correctly read and the part does not need to be configured.
			 */
			unsigned int dummy;
			_interface->ioctl(IOCTL_RESET, dummy);
			return OK;
		}

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return CDev::ioctl(filp, cmd, arg);
}

void
MPL3115A2::start(unsigned delay_us)
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleDelayed(delay_us);
}

void
MPL3115A2::stop()
{
	ScheduleClear();
}

void
MPL3115A2::Run()
{
	int ret;
	unsigned dummy;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret == -EIO) {
			/* issue a reset command to the sensor */
			_interface->ioctl(IOCTL_RESET, dummy);
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MPL3115A2 datasheet
			 */
			start(2800);
			return;
		}

		if (ret == -EAGAIN) {

			/* Ready read it on next cycle */
			ScheduleDelayed(MPL3115A2_CONVERSION_INTERVAL);

			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* Look for a ready condition */

	ret = measure();

	if (ret == -EIO) {
		/* issue a reset command to the sensor */
		_interface->ioctl(IOCTL_RESET, dummy);
		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is measurement */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(MPL3115A2_CONVERSION_INTERVAL);
}

int
MPL3115A2::measure()
{
	int ret;

	perf_begin(_measure_perf);

	/*
	 * Send the command to read the ADC for P and T.
	 */
	unsigned addr = (MPL3115A2_CTRL_REG1 << 8) | MPL3115A2_CTRL_TRIGGER;
	ret = _interface->ioctl(IOCTL_MEASURE, addr);

	if (ret == -EIO) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return PX4_OK;
}

int
MPL3115A2::collect()
{
	int ret;
	MPL3115A2_data_t reading;
	uint8_t ctrl;

	perf_begin(_sample_perf);

	ret = _interface->read(MPL3115A2_CTRL_REG1, (void *)&ctrl, 1);

	if (ret == -EIO) {
		perf_end(_sample_perf);
		return ret;
	}

	if (ctrl & CTRL_REG1_OST) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	sensor_baro_s report;

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();

	report.error_count = perf_event_count(_comms_errors);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	ret = _interface->read(OUT_P_MSB, (void *)&reading, sizeof(reading));

	if (ret == -EIO) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	_T = (float) reading.temperature.b[1] + ((float)(reading.temperature.b[0]) / 16.0f);
	_P = (float)(reading.pressure.q >> 8) + ((float)(reading.pressure.b[0]) / 4.0f);

	report.temperature = _T;
	report.pressure = _P / 100.0f;		/* convert to millibar */

	/* return device ID */
	report.device_id = _interface->get_device_id();

	/* publish it */
	if (_baro_topic != nullptr) {
		/* publish it */
		orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

void
MPL3115A2::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);
	_reports->print_info("report queue");

	sensor_baro_s brp = {};
	_reports->get(&brp);
	print_message(brp);
}

/**
 * Local functions in support of the shell command.
 */
namespace mpl3115a2
{

/*
  list of supported bus configurations
 */
struct mpl3115a2_bus_option {
	enum MPL3115A2_BUS busid;
	const char *devpath;
	MPL3115A2_constructor interface_constructor;
	uint8_t busnum;
	MPL3115A2 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ MPL3115A2_BUS_SPI_EXTERNAL, "/dev/mpl3115a2_spi_ext", &MPL3115A2_spi_interface, PX4_SPI_BUS_EXT, NULL },
#endif
#ifdef PX4_SPIDEV_BARO
	{ MPL3115A2_BUS_SPI_INTERNAL, "/dev/mpl3115a2_spi_int", &MPL3115A2_spi_interface, PX4_SPI_BUS_BARO, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ MPL3115A2_BUS_I2C_INTERNAL, "/dev/mpl3115a2_int", &MPL3115A2_i2c_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION
	{ MPL3115A2_BUS_I2C_EXTERNAL, "/dev/mpl3115a2_ext", &MPL3115A2_i2c_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ MPL3115A2_BUS_I2C_EXTERNAL, "/dev/mpl3115a2_ext1", &MPL3115A2_i2c_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ MPL3115A2_BUS_I2C_EXTERNAL, "/dev/mpl3115a2_ext2", &MPL3115A2_i2c_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct mpl3115a2_bus_option &bus);
struct mpl3115a2_bus_option &find_bus(enum MPL3115A2_BUS busid);
void	start(enum MPL3115A2_BUS busid);
void	test(enum MPL3115A2_BUS busid);
void	reset(enum MPL3115A2_BUS busid);
void	info();
void	usage();

/**
 * Start the driver.
 */
bool
start_bus(struct mpl3115a2_bus_option &bus)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new MPL3115A2(interface, bus.devpath);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		errx(1, "can't open baro device");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "failed setting default poll rate");
	}

	close(fd);
	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum MPL3115A2_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MPL3115A2_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MPL3115A2_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started = started | start_bus(bus_options[i]);
	}

	if (!started) {
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}


/**
 * find a bus structure for a busid
 */
struct mpl3115a2_bus_option &find_bus(enum MPL3115A2_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MPL3115A2_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum MPL3115A2_BUS busid)
{
	struct mpl3115a2_bus_option &bus = find_bus(busid);
	sensor_baro_s report;
	ssize_t sz;
	int ret;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'mpl3115a2 start' if the driver is not running)");
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

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

	close(fd);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(enum MPL3115A2_BUS busid)
{
	struct mpl3115a2_bus_option &bus = find_bus(busid);
	int fd;

	fd = open(bus.devpath, O_RDONLY);

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
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct mpl3115a2_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			warnx("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (intternal I2C bus)");

}

} // namespace

int
mpl3115a2_main(int argc, char *argv[])
{
	enum MPL3115A2_BUS busid = MPL3115A2_BUS_ALL;
	int myoptind = 1;
	int ch;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "XI", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MPL3115A2_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MPL3115A2_BUS_I2C_INTERNAL;
			break;

		default:
			mpl3115a2::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		mpl3115a2::usage();
		exit(0);
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		mpl3115a2::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		mpl3115a2::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mpl3115a2::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		mpl3115a2::info();
	}

	mpl3115a2::usage();
	exit(1);
}

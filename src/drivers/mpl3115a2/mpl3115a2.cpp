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
 * @file mpl3115a2.cpp
 *
 * FIXME!: This is stubberd out driver for the NXP MPL3115A2
 * it has bogus code in it and is just being used to verify
 * the i2C buss and WHOAMI
 *
 * Driver for the MPL3115A2 barometric pressure sensor connected via I2C.
 */

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
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <platforms/px4_getopt.h>

#include "mpl3115a2.h"



enum MPL3115A2_BUS {
	MPL3115A2_BUS_ALL = 0,
	MPL3115A2_BUS_I2C_INTERNAL,
	MPL3115A2_BUS_I2C_EXTERNAL,
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * MPL3115A2 internal constants and data structures.
 */

#define MPL3115A2_CONVERSION_INTERVAL	10000	/* microseconds */
#define MPL3115A2_OSR                   0       /* Over Sample rate of 1 6MS Minimum time between data samples */
#define MPL3115A2_CTRL_TRIGGER          (CTRL_REG1_OST | CTRL_REG1_OS(MPL3115A2_OSR))
#define MPL3115A2_BARO_DEVICE_PATH_EXT  "/dev/mpl3115a2_ext"
#define MPL3115A2_BARO_DEVICE_PATH_INT  "/dev/mpl3115a2_int"

class MPL3115A2 : public device::CDev
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
	Device			*_interface;

	struct work_s		_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer	*_reports;
	bool			_collect_phase;

	/*  */
	float			_P;
	float			_T;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @param delay_ticks the number of queue ticks before executing the next cycle
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle(unsigned delay_ticks = 1);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();

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
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

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
	CDev("MPL3115A2", path),
	_interface(interface),
	_measure_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_P(0),
	_T(0),
	_msl_pressure(101325),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpl3115a2_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "mpl3115a2_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "mpl3115a2_com_err"))
{
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_MPL3115A2;
}

MPL3115A2::~MPL3115A2()
{
	/* make sure we are truly inactive */
	stop_cycle();

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
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));

	if (_reports == nullptr) {
		DEVICE_DEBUG("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	struct baro_report brp;
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
		brp.device_id = _device_id.devid;

		ret = OK;

		_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
						  &_orb_class_instance, (external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

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
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
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

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_measure_ticks = 0;
				return OK;

			/* external signalling not supported */
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
					_measure_ticks = USEC2TICK(MPL3115A2_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
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
					if (ticks < USEC2TICK(MPL3115A2_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
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

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);
			return OK;
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

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return CDev::ioctl(filp, cmd, arg);
}

void
MPL3115A2::start_cycle(unsigned delay_ticks)
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MPL3115A2::cycle_trampoline, this, delay_ticks);
}

void
MPL3115A2::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void
MPL3115A2::cycle_trampoline(void *arg)
{
	MPL3115A2 *dev = reinterpret_cast<MPL3115A2 *>(arg);

	dev->cycle();
}

void
MPL3115A2::cycle()
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
			start_cycle(USEC2TICK(2800));
			return;
		}


		if (ret == -EAGAIN) {

			/* Ready read it on next cycle */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&MPL3115A2::cycle_trampoline,
				   this,
				   USEC2TICK(MPL3115A2_CONVERSION_INTERVAL));
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
		start_cycle();
		return;
	}

	/* next phase is measurement */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MPL3115A2::cycle_trampoline,
		   this,
		   USEC2TICK(MPL3115A2_CONVERSION_INTERVAL));
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

	struct baro_report report;

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
	report.device_id = _device_id.devid;

	/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

	/*
	 * PERFORMANCE HINT:
	 *
	 * The single precision calculation is 50 microseconds faster than the double
	 * precision variant. It is however not obvious if double precision is required.
	 * Pending more inspection and tests, we'll leave the double precision variant active.
	 *
	 * Measurements:
	 * 	double precision: mpl3115a2_read: 992 events, 258641us elapsed, min 202us max 305us
	 *	single precision: mpl3115a2_read: 963 events, 208066us elapsed, min 202us max 241us
	 */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = _msl_pressure / 1000.0;

	/* measured pressure in kPa */
	double p = (double) _P / 1000.0;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	report.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

	/* publish it */
	if (!(_pub_blocked) && _baro_topic != nullptr) {
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
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
	printf("device:         mpl3115a2\n");
	printf("P:              %.3f\n", (double)_P);
	printf("T:              %.3f\n", (double)_T);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));
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
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct mpl3115a2_bus_option &bus);
struct mpl3115a2_bus_option &find_bus(enum MPL3115A2_BUS busid);
void	start(enum MPL3115A2_BUS busid);
void	test(enum MPL3115A2_BUS busid);
void	reset(enum MPL3115A2_BUS busid);
void	info();
void	calibrate(unsigned altitude, enum MPL3115A2_BUS busid);
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
	struct baro_report report;
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

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.pressure);
	warnx("altitude:    %11.4f", (double)report.altitude);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

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
		warnx("pressure:    %10.4f", (double)report.pressure);
		warnx("altitude:    %11.4f", (double)report.altitude);
		warnx("temperature: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
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

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude, enum MPL3115A2_BUS busid)
{
	struct mpl3115a2_bus_option &bus = find_bus(busid);
	struct baro_report report;
	float	pressure;
	float	p1;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'mpl3115a2 start' if the driver is not running)");
	}

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX)) {
		errx(1, "failed to set poll rate");
	}

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "sensor read failed");
		}

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK) {
		err(1, "BAROIOCSMSLPRESSURE");
	}

	close(fd);
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'calibrate'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (intternal I2C bus)");

}

} // namespace

int
mpl3115a2_main(int argc, char *argv[])
{
	enum MPL3115A2_BUS busid = MPL3115A2_BUS_ALL;
	int ch;
	int myoptind = 1;
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

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(verb, "calibrate")) {
		if (argc < 2) {
			errx(1, "missing altitude");
		}

		long altitude = strtol(argv[myoptind + 1], nullptr, 10);

		mpl3115a2::calibrate(altitude, busid);
	}

	mpl3115a2::usage();
	exit(1);
}

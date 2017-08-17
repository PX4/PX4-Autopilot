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
 * @file lps25h.cpp
 *
 * Driver for the LPS25H barometer connected via I2C or SPI.
 */

#include <px4_config.h>

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

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>

#include "lps25h.h"

/*
 * LPS25H internal constants and data structures.
 */

/* Max measurement rate is 25Hz */
#define LPS25H_CONVERSION_INTERVAL	(1000000 / 25)	/* microseconds */

#define ADDR_REF_P_XL		0x08
#define ADDR_REF_P_L		0x09
#define ADDR_REF_P_H		0x0A
#define ADDR_WHO_AM_I		0x0F
#define ADDR_RES_CONF		0x10
#define ADDR_CTRL_REG1		0x20
#define ADDR_CTRL_REG2		0x21
#define ADDR_CTRL_REG3		0x22
#define ADDR_CTRL_REG4		0x23
#define ADDR_INT_CFG		0x24
#define ADDR_INT_SOURCE		0x25

#define ADDR_STATUS_REG		0x27
#define ADDR_P_OUT_XL		0x28
#define ADDR_P_OUT_L		0x29
#define ADDR_P_OUT_H		0x2A
#define ADDR_TEMP_OUT_L		0x2B
#define ADDR_TEMP_OUT_H		0x2C

#define ADDR_FIFO_CTRL		0x2E
#define ADDR_FIFO_STATUS	0x2F
#define ADDR_THS_P_L		0x30
#define ADDR_THS_P_H		0x31

#define ADDR_RPDS_L		0x39
#define ADDR_RPDS_H		0x3A

/* Data sheet is ambigious if AVGT or AVGP is first */
#define RES_CONF_AVGT_8		0x00
#define RES_CONF_AVGT_32	0x01
#define RES_CONF_AVGT_128	0x02
#define RES_CONF_AVGT_512	0x03
#define RES_CONF_AVGP_8		0x00
#define RES_CONF_AVGP_32	0x04
#define RES_CONF_AVGP_128	0x08
#define RES_CONF_AVGP_512	0x0C

#define CTRL_REG1_SIM		(1 << 0)
#define CTRL_REG1_RESET_AZ	(1 << 1)
#define CTRL_REG1_BDU		(1 << 2)
#define CTRL_REG1_DIFF_EN	(1 << 3)
#define CTRL_REG1_PD		(1 << 7)
#define CTRL_REG1_ODR_SINGLE	(0 << 4)
#define CTRL_REG1_ODR_1HZ	(1 << 4)
#define CTRL_REG1_ODR_7HZ	(2 << 4)
#define CTRL_REG1_ODR_12HZ5	(3 << 4)
#define CTRL_REG1_ODR_25HZ	(4 << 4)

#define CTRL_REG2_ONE_SHOT	(1 << 0)
#define CTRL_REG2_AUTO_ZERO	(1 << 1)
#define CTRL_REG2_SWRESET	(1 << 2)
#define CTRL_REG2_FIFO_MEAN_DEC	(1 << 4)
#define CTRL_REG2_WTM_EN	(1 << 5)
#define CTRL_REG2_FIFO_EN	(1 << 6)
#define CTRL_REG2_BOOT		(1 << 7)

#define CTRL_REG3_INT1_S_DATA	0x0
#define CTRL_REG3_INT1_S_P_HIGH	0x1
#define CTRL_REG3_INT1_S_P_LOW	0x2
#define CTRL_REG3_INT1_S_P_LIM	0x3
#define CTRL_REG3_PP_OD		(1 << 6)
#define CTRL_REG3_INT_H_L	(1 << 7)

#define CTRL_REG4_P1_DRDY	(1 << 0)
#define CTRL_REG4_P1_OVERRUN	(1 << 1)
#define CTRL_REG4_P1_WTM	(1 << 2)
#define CTRL_REG4_P1_EMPTY	(1 << 3)

#define INTERRUPT_CFG_PH_E	(1 << 0)
#define INTERRUPT_CFG_PL_E	(1 << 1)
#define INTERRUPT_CFG_LIR	(1 << 2)

#define INT_SOURCE_PH		(1 << 0)
#define INT_SOURCE_PL		(1 << 1)
#define INT_SOURCE_IA		(1 << 2)

#define STATUS_REG_T_DA		(1 << 0)
#define STATUS_REG_P_DA		(1 << 1)
#define STATUS_REG_T_OR		(1 << 4)
#define STATUS_REG_P_OR		(1 << 5)

#define FIFO_CTRL_WTM_FMEAN_2	0x01
#define FIFO_CTRL_WTM_FMEAN_4	0x03
#define FIFO_CTRL_WTM_FMEAN_8	0x07
#define FIFO_CTRL_WTM_FMEAN_16	0x0F
#define FIFO_CTRL_WTM_FMEAN_32	0x1F
#define FIFO_CTRL_F_MODE_BYPASS	(0x0 << 5)
#define FIFO_CTRL_F_MODE_FIFO	(0x1 << 5)
#define FIFO_CTRL_F_MODE_STREAM	(0x2 << 5)
#define FIFO_CTRL_F_MODE_SFIFO	(0x3 << 5)
#define FIFO_CTRL_F_MODE_BSTRM	(0x4 << 5)
#define FIFO_CTRL_F_MODE_FMEAN	(0x6 << 5)
#define FIFO_CTRL_F_MODE_BFIFO	(0x7 << 5)

#define FIFO_STATUS_EMPTY	(1 << 5)
#define FIFO_STATUS_FULL	(1 << 6)
#define FIFO_STATUS_WTM		(1 << 7)

enum LPS25H_BUS {
	LPS25H_BUS_ALL = 0,
	LPS25H_BUS_I2C_INTERNAL,
	LPS25H_BUS_I2C_EXTERNAL,
	LPS25H_BUS_SPI
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class LPS25H : public device::CDev
{
public:
	LPS25H(device::Device *interface, const char *path);
	virtual ~LPS25H();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

private:
	work_s			_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer	*_reports;
	bool			_collect_phase;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	struct baro_report	_last_report;           /**< used for info() */

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(struct file *filp, unsigned enable);

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
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/* this class has pointer data members, do not allow copying it */
	LPS25H(const LPS25H &);
	LPS25H operator=(const LPS25H &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int lps25h_main(int argc, char *argv[]);


LPS25H::LPS25H(device::Device *interface, const char *path) :
	CDev("LPS25H", path),
	_interface(interface),
	_work{},
	_measure_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_msl_pressure(101325),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "lps25h_read")),
	_comms_errors(perf_alloc(PC_COUNT, "lps25h_comms_errors")),
	_last_report{0}
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_LPS25H;

	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

LPS25H::~LPS25H()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(BARO_BASE_DEVICE_PATH, _class_instance);
	}

	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
LPS25H::init()
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

	if (reset() != OK) {
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	ret = OK;

out:
	return ret;
}

ssize_t
LPS25H::read(struct file *filp, char *buffer, size_t buflen)
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
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(LPS25H_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(brp)) {
			ret = sizeof(struct baro_report);
		}
	} while (0);

	return ret;
}

int
LPS25H::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

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
					_measure_ticks = USEC2TICK(LPS25H_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(LPS25H_CONVERSION_INTERVAL)) {
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

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);
			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		return reset();

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	case DEVIOCGDEVICEID:
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
LPS25H::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LPS25H::cycle_trampoline, this, 1);
}

void
LPS25H::stop()
{
	work_cancel(HPWORK, &_work);
}

int
LPS25H::reset()
{
	int ret = 0;

	// Power on
	ret = write_reg(ADDR_CTRL_REG1, CTRL_REG1_PD);

	usleep(1000);

	// Reset
	ret = write_reg(ADDR_CTRL_REG2, CTRL_REG2_BOOT | CTRL_REG2_SWRESET);

	usleep(5000);

	// Power on
	ret = write_reg(ADDR_CTRL_REG1, CTRL_REG1_PD);

	usleep(1000);

	return ret;
}

void
LPS25H::cycle_trampoline(void *arg)
{
	LPS25H *dev = reinterpret_cast<LPS25H *>(arg);

	dev->cycle();
}

void
LPS25H::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(LPS25H_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&LPS25H::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(LPS25H_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&LPS25H::cycle_trampoline,
		   this,
		   USEC2TICK(LPS25H_CONVERSION_INTERVAL));
}

int
LPS25H::measure()
{
	int ret;

	/*
	 * Send the command to begin a 16-bit measurement.
	 */
	ret = write_reg(ADDR_CTRL_REG2, CTRL_REG2_ONE_SHOT);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
LPS25H::collect()
{
#pragma pack(push, 1)
	struct {
		uint8_t		status;
		uint8_t		p_xl, p_l, p_h;
		int16_t		t;
	} report;
#pragma pack(pop)

	int	ret;

	perf_begin(_sample_perf);
	struct baro_report new_report;
	bool sensor_is_onboard = false;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);

	/*
	 * @note  We could read the status register 1 here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device : MSB enables register address auto-increment */
	ret = _interface->read(ADDR_STATUS_REG | (1 << 7), (uint8_t *)&report, sizeof(report));

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* get measurements from the device */
	new_report.temperature = 42.5 + (report.t / 480);

	/* raw pressure */
	uint32_t raw = report.p_xl + (report.p_l << 8) + (report.p_h << 16);

	/* Pressure and MSL in mBar */
	double p = raw / 4096.0;
	double msl = _msl_pressure / 100.0;

	double alt = (1.0 - pow(p / msl, 0.190263)) * 44330.8;

	new_report.pressure = p;
	new_report.altitude = alt;

	/* get device ID */
	new_report.device_id = _device_id.devid;

	if (!(_pub_blocked)) {

		if (_baro_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_baro), _baro_topic, &new_report);

		} else {
			_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &new_report,
							  &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

			if (_baro_topic == nullptr) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	_reports->force(&new_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);


	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

int
LPS25H::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
LPS25H::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void
LPS25H::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("pressure    %.2f\n", (double)_last_report.pressure);
	printf("altitude:    %.2f\n", (double)_last_report.altitude);
	printf("temperature %.2f\n", (double)_last_report.temperature);

	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace lps25h
{

/*
  list of supported bus configurations
 */
struct lps25h_bus_option {
	enum LPS25H_BUS busid;
	const char *devpath;
	LPS25H_constructor interface_constructor;
	uint8_t busnum;
	LPS25H	*dev;
} bus_options[] = {
	{ LPS25H_BUS_I2C_EXTERNAL, "/dev/lps25h_ext", &LPS25H_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_ONBOARD
	{ LPS25H_BUS_I2C_INTERNAL, "/dev/lps25h_int", &LPS25H_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_HMC
	{ LPS25H_BUS_SPI, "/dev/lps25h_spi", &LPS25H_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void	start(enum LPS25H_BUS busid);
bool	start_bus(struct lps25h_bus_option &bus);
struct lps25h_bus_option &find_bus(enum LPS25H_BUS busid);
void	test(enum LPS25H_BUS busid);
void	reset(enum LPS25H_BUS busid);
void	info();
void	calibrate(unsigned altitude, enum LPS25H_BUS busid);
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct lps25h_bus_option &bus)
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

	bus.dev = new LPS25H(interface, bus.devpath);

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
start(enum LPS25H_BUS busid)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == LPS25H_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != LPS25H_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		exit(1);
	}
}

/**
 * find a bus structure for a busid
 */
struct lps25h_bus_option &find_bus(enum LPS25H_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == LPS25H_BUS_ALL ||
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
test(enum LPS25H_BUS busid)
{
	struct lps25h_bus_option &bus = find_bus(busid);
	struct baro_report report;
	ssize_t sz;
	int ret;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'lps25h start' if the driver is not running)");
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
		warnx("temperature K: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}

	close(fd);
	errx(0, "PASS");
}


/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude, enum LPS25H_BUS busid)
{
	struct lps25h_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'lps25h start' if the driver is not running", path);
	}

	// TODO: Implement calibration

	close(fd);
}

/**
 * Reset the driver.
 */
void
reset(enum LPS25H_BUS busid)
{
	struct lps25h_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

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
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct lps25h_bus_option &bus = bus_options[i];

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
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'calibrate'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (internal I2C bus)");
	warnx("    -S    (external SPI bus)");
	warnx("    -s    (internal SPI bus)");
}

} // namespace

int
lps25h_main(int argc, char *argv[])
{
	enum LPS25H_BUS busid = LPS25H_BUS_ALL;
	int ch;

	while ((ch = getopt(argc, argv, "XIS:")) != EOF) {
		switch (ch) {
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)

		case 'I':
			busid = LPS25H_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = LPS25H_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = LPS25H_BUS_SPI;
			break;

		default:
			lps25h::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		lps25h::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		lps25h::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		lps25h::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		lps25h::info();
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(verb, "calibrate")) {
		if (argc < 2) {
			errx(1, "missing altitude");
		}

		long altitude = strtol(argv[optind + 1], nullptr, 10);

		lps25h::calibrate(altitude, busid);
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}

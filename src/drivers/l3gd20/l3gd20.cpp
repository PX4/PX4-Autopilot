/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file Driver for the ST L3GD20 MEMS gyro connected via SPI.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <drivers/device/spi.h>
#include <drivers/drv_gyro.h>


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM				0xD4

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */
/* keep lowpass low to avoid noise issues */
#define RATE_95HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_25HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_380HZ_LP_30HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_760HZ_LP_30HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */
#define RANGE_250DPS				(0<<4)
#define RANGE_500DPS				(1<<4)
#define RANGE_2000DPS				(3<<4)

#define ADDR_CTRL_REG5			0x24
#define ADDR_REFERENCE			0x25
#define ADDR_OUT_TEMP			0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_FIFO_CTRL_REG		0x2E
#define ADDR_FIFO_SRC_REG		0x2F
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH_XH		0x32
#define ADDR_INT1_TSH_XL		0x33
#define ADDR_INT1_TSH_YH		0x34
#define ADDR_INT1_TSH_YL		0x35
#define ADDR_INT1_TSH_ZH		0x36
#define ADDR_INT1_TSH_ZL		0x37
#define ADDR_INT1_DURATION		0x38


/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU				(1<<7)
#define REG4_BLE				(1<<6)
//#define REG4_SPI_3WIRE			(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define FIFO_CTRL_BYPASS_MODE			(0<<5)
#define FIFO_CTRL_FIFO_MODE			(1<<5)
#define FIFO_CTRL_STREAM_MODE			(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

extern "C" { __EXPORT int l3gd20_main(int argc, char *argv[]); }

class L3GD20 : public device::SPI
{
public:
	L3GD20(int bus, const char* path, spi_dev_e device);
	virtual ~L3GD20();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	struct hrt_call		_call;
	unsigned		_call_interval;

	unsigned		_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	struct gyro_report	*_reports;

	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;
	orb_advert_t		_gyro_topic;

	unsigned		_current_rate;
	unsigned		_current_range;

	perf_counter_t		_sample_perf;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Read a register from the L3GD20
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the L3GD20
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the L3GD20
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the L3GD20 measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_dps);

	/**
	 * Set the L3GD20 internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_samplerate(unsigned frequency);
};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)


L3GD20::L3GD20(int bus, const char* path, spi_dev_e device) :
	SPI("L3GD20", path, bus, device, SPIDEV_MODE3, 8000000),
	_call_interval(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(-1),
	_current_rate(0),
	_current_range(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "l3gd20_read"))
{
	// enable debug() calls
	_debug_enabled = true;

	// default scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;
}

L3GD20::~L3GD20()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;

	/* delete the perf counter */
	perf_free(_sample_perf);
}

int
L3GD20::init()
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_oldest_report = _next_report = 0;
	_reports = new struct gyro_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	/* advertise sensor topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_gyro_topic = orb_advertise(ORB_ID(sensor_gyro), &_reports[0]);

	/* set default configuration */
	write_reg(ADDR_CTRL_REG1, REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_reg(ADDR_CTRL_REG3, 0);		/* no interrupts - we don't use them */
	write_reg(ADDR_CTRL_REG4, REG4_BDU);
	write_reg(ADDR_CTRL_REG5, 0);

	write_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);	  /* disable wake-on-interrupt */
	write_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_STREAM_MODE); /* Enable FIFO, old data is overwritten */

	set_range(500);				/* default to 500dps */
	set_samplerate(0);			/* max sample rate */

	ret = OK;
out:
	return ret;
}

int
L3GD20::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	/* verify that the device is attached and functioning */
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM)
		return OK;

	return -EIO;
}

ssize_t
L3GD20::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct gyro_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_oldest_report != _next_report) {
				memcpy(buffer, _reports + _oldest_report, sizeof(*_reports));
				ret += sizeof(_reports[0]);
				INCREMENT(_oldest_report, _num_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_oldest_report = _next_report = 0;
	measure();

	/* measurement will have generated a report, copy it out */
	memcpy(buffer, _reports, sizeof(*_reports));
	ret = sizeof(*_reports);

	return ret;
}

int
L3GD20::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				/* With internal low pass filters enabled, 250 Hz is sufficient */
				return ioctl(filp, SENSORIOCSPOLLRATE, 250);

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* account for sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct gyro_report *buf = new struct gyro_report[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _reports;
			_num_reports = arg;
			_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_reports - 1;

	case SENSORIOCRESET:
		/* XXX implement */
		return -EINVAL;

	case GYROIOCSSAMPLERATE:
		return set_samplerate(arg);

	case GYROIOCGSAMPLERATE:
		return _current_rate;

	case GYROIOCSLOWPASS:
	case GYROIOCGLOWPASS:
		/* XXX not implemented due to wacky interaction with samplerate */
		return -EINVAL;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		return set_range(arg);

	case GYROIOCGRANGE:
		return _current_range;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
L3GD20::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
L3GD20::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
L3GD20::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int
L3GD20::set_range(unsigned max_dps)
{
	uint8_t bits = REG4_BDU;

	if (max_dps == 0)
		max_dps = 2000;

	if (max_dps <= 250) {
		_current_range = 250;
		bits |= RANGE_250DPS;

	} else if (max_dps <= 500) {
		_current_range = 500;
		bits |= RANGE_500DPS;

	} else if (max_dps <= 2000) {
		_current_range = 2000;
		bits |= RANGE_2000DPS;

	} else {
		return -EINVAL;
	}

	_gyro_range_rad_s = _current_range / 180.0f * M_PI_F;
	_gyro_range_scale = _gyro_range_rad_s / 32768.0f;
	write_reg(ADDR_CTRL_REG4, bits);

	return OK;
}

int
L3GD20::set_samplerate(unsigned frequency)
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

	if (frequency == 0)
		frequency = 760;

	if (frequency <= 95) {
		_current_rate = 95;
		bits |= RATE_95HZ_LP_25HZ;

	} else if (frequency <= 190) {
		_current_rate = 190;
		bits |= RATE_190HZ_LP_25HZ;

	} else if (frequency <= 380) {
		_current_rate = 380;
		bits |= RATE_380HZ_LP_30HZ;

	} else if (frequency <= 760) {
		_current_rate = 760;
		bits |= RATE_760HZ_LP_30HZ;

	} else {
		return -EINVAL;
	}

	write_reg(ADDR_CTRL_REG1, bits);

	return OK;
}

void
L3GD20::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_oldest_report = _next_report = 0;

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&L3GD20::measure_trampoline, this);
}

void
L3GD20::stop()
{
	hrt_cancel(&_call);
}

void
L3GD20::measure_trampoline(void *arg)
{
	L3GD20 *dev = (L3GD20 *)arg;

	/* make another measurement */
	dev->measure();
}

void
L3GD20::measure()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_report;
#pragma pack(pop)

	gyro_report		*report = &_reports[_next_report];

	/* start the performance counter */
	perf_begin(_sample_perf);

	/* fetch data from the sensor */
	raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */
	report->timestamp = hrt_absolute_time();
	
	/* swap x and y and negate y */
	report->x_raw = raw_report.y;
	report->y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
	report->z_raw = raw_report.z;

	report->x = ((report->x_raw * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	report->y = ((report->y_raw * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	report->z = ((report->z_raw * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;
	report->scaling = _gyro_range_scale;
	report->range_rad_s = _gyro_range_rad_s;

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_report, _num_reports);

	/* if we are running up against the oldest report, fix it */
	if (_next_report == _oldest_report)
		INCREMENT(_oldest_report, _num_reports);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	orb_publish(ORB_ID(sensor_gyro), _gyro_topic, report);

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
L3GD20::print_info()
{
	perf_print_counter(_sample_perf);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
}

/**
 * Local functions in support of the shell command.
 */
namespace l3gd20
{

L3GD20	*g_dev;

void	start();
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

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new L3GD20(1 /* XXX magic number */, GYRO_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_GYRO);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(GYRO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd_gyro = -1;
	struct gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	fd_gyro = open(GYRO_DEVICE_PATH, O_RDONLY);

	if (fd_gyro < 0)
		err(1, "%s open failed", GYRO_DEVICE_PATH);

	/* reset to manual polling */
	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
		err(1, "reset to manual polling");

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report))
		err(1, "immediate gyro read failed");

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	/* XXX add poll-rate tests here too */

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(GYRO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running\n");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}


} // namespace

int
l3gd20_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.

	 */
	if (!strcmp(argv[1], "start"))
		l3gd20::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		l3gd20::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		l3gd20::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		l3gd20::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}

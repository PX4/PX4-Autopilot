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
 * @file Driver for the Bosch BMA 180 MEMS accelerometer connected via SPI.
 */

#include <nuttx/config.h>

#include <device/spi.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
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

#include <arch/board/up_hrt.h>

#include <drivers/drv_accel.h>

extern "C" { __EXPORT int bma180_main(int argc, char *argv[]); }

class BMA180 : public device::SPI
{
public:
	BMA180(int bus, spi_dev_e device);
	~BMA180();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		open_first(struct file *filp);
	virtual int		close_last(struct file *filp);

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
	struct accel_report	*_reports;

	struct accel_scale	_scale;
	float			_range_scale;

	unsigned		_reads;

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
	 * Read a register from the BMA180
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the BMA180
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the BMA180
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the BMA180 measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_g);

	/**
	 * Set the BMA180 lowpass filter.
	 *
	 * @param frequency	Set the lowpass filter cutoff frequency to no less than
	 *			this frequency.
	 * @return		OK if the value can be supported.
	 */
	int			set_bandwidth(unsigned frequency);
};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)

#define ADDR_CHIP_ID			0x00
#define CHIP_ID				0x03

#define ADDR_ACC_X_LSB			0x02
#define ADDR_ACC_Y_LSB			0x04
#define ADDR_ACC_Z_LSB			0x06
#define ADDR_TEMPERATURE		0x08

#define ADDR_RESET			0x10
#define SOFT_RESET				0xB6

#define ADDR_BW_TCS			0x20
#define BW_TCS_BW_MASK				(0xf<<4)
#define BW_TCS_BW_10HZ				(0<<4)
#define BW_TCS_BW_20HZ				(1<<4)
#define BW_TCS_BW_40HZ				(2<<4)
#define BW_TCS_BW_75HZ				(3<<4)
#define BW_TCS_BW_150HZ				(4<<4)
#define BW_TCS_BW_300HZ				(5<<4)
#define BW_TCS_BW_600HZ				(6<<4)
#define BW_TCS_BW_1200HZ			(7<<4)

#define ADDR_HIGH_DUR			0x27
#define HIGH_DUR_DIS_I2C			(1<<0)

#define ADDR_TCO_Z			0x30
#define TCO_Z_MODE_MASK				0x3

#define ADDR_GAIN_Y			0x33
#define GAIN_Y_SHADOW_DIS			(1<<0)

#define ADDR_OFFSET_LSB1		0x35
#define OFFSET_LSB1_RANGE_MASK			(7<<1)
#define OFFSET_LSB1_RANGE_1G			(0<<1)
#define OFFSET_LSB1_RANGE_2G			(2<<1)
#define OFFSET_LSB1_RANGE_3G			(3<<1)
#define OFFSET_LSB1_RANGE_4G			(4<<1)
#define OFFSET_LSB1_RANGE_8G			(5<<1)
#define OFFSET_LSB1_RANGE_16G			(6<<1)

#define ADDR_OFFSET_T			0x37
#define OFFSET_T_READOUT_12BIT			(1<<0)

/*
 * Driver 'main' command.
 */
extern "C" { int bma180_main(int argc, char *argv[]); }


BMA180::BMA180(int bus, spi_dev_e device) :
	SPI("BMA180", ACCEL_DEVICE_PATH, bus, device, SPIDEV_MODE3, 8000000),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_reads(0)
{
	// enable debug() calls
	_debug_enabled = true;

	// default scale factors
	_scale.x_offset = 0;
	_scale.x_scale  = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale  = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale  = 1.0f;
}

BMA180::~BMA180()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
BMA180::init()
{
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup successful, finish chip init */
	if (ret == OK) {

		/* perform soft reset (p48) */
		write_reg(ADDR_RESET, SOFT_RESET);

		/* wait 10us (p49) */
		usleep(10);

		/* disable I2C interface */
		modify_reg(ADDR_HIGH_DUR, HIGH_DUR_DIS_I2C, 0);

		/* switch to low-noise mode */
		modify_reg(ADDR_TCO_Z, TCO_Z_MODE_MASK, 0);

		/* disable 12-bit mode */
		modify_reg(ADDR_OFFSET_T, OFFSET_T_READOUT_12BIT, 0);

		/* disable shadow-disable mode */
		modify_reg(ADDR_GAIN_Y, GAIN_Y_SHADOW_DIS, 0);
	}

	return ret;
}

int
BMA180::open_first(struct file *filp)
{
	/* reset to manual-poll mode */
	_call_interval = 0;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct accel_report[_num_reports];
	_oldest_report = _next_report = 0;

	/* set default range and lowpass */
	set_range(4);		/* 4G */
	set_bandwidth(600);	/* 600Hz */

	return OK;
}

int
BMA180::close_last(struct file *filp)
{
	/* stop measurement */
	stop();

	/* free report buffers */
	if (_reports != nullptr) {
		delete[] _reports;
		_num_reports = 0;
	}

	return OK;
}

int
BMA180::probe()
{
	if (read_reg(ADDR_CHIP_ID) == CHIP_ID)
		return OK;

	return -EIO;
}

ssize_t
BMA180::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct accel_report);
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

		_reads++;

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
BMA180::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case ACCELIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case ACC_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

				/* external signalling not supported */
			case ACC_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

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
					_call.period = _call_interval;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case ACCELIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct accel_report *buf = new struct accel_report[arg];

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

	case ACCELIOCSLOWPASS:
		return set_bandwidth(arg);

	case ACCELIORANGE:
		return set_range(arg);

	case ACCELIOCSSAMPLERATE:	/* sensor sample rate is not (really) adjustable */
	case ACCELIOCSREPORTFORMAT:	/* no alternate report formats */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
BMA180::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
BMA180::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
BMA180::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int
BMA180::set_range(unsigned max_g)
{
	uint8_t rangebits;
	float rangescale;

	if (max_g > 16) {
		return -ERANGE;

	} else if (max_g > 8) {		/* 16G */
		rangebits = OFFSET_LSB1_RANGE_16G;
		rangescale = 1.98;

	} else if (max_g > 4) {		/* 8G */
		rangebits = OFFSET_LSB1_RANGE_8G;
		rangescale = 0.99;

	} else if (max_g > 3) {		/* 4G */
		rangebits = OFFSET_LSB1_RANGE_4G;
		rangescale = 0.5;

	} else if (max_g > 2) {		/* 3G */
		rangebits = OFFSET_LSB1_RANGE_3G;
		rangescale = 0.38;

	} else if (max_g > 1) {		/* 2G */
		rangebits = OFFSET_LSB1_RANGE_2G;
		rangescale = 0.25;

	} else {			/* 1G */
		rangebits = OFFSET_LSB1_RANGE_1G;
		rangescale = 0.13;
	}

	/* adjust sensor configuration */
	modify_reg(ADDR_OFFSET_LSB1, OFFSET_LSB1_RANGE_MASK, rangebits);
	_range_scale = rangescale;

	return OK;
}

int
BMA180::set_bandwidth(unsigned frequency)
{
	uint8_t	bwbits;

	if (frequency > 1200) {
		return -ERANGE;

	} else if (frequency > 600) {
		bwbits = BW_TCS_BW_1200HZ;

	} else if (frequency > 300) {
		bwbits = BW_TCS_BW_600HZ;

	} else if (frequency > 150) {
		bwbits = BW_TCS_BW_300HZ;

	} else if (frequency > 75) {
		bwbits = BW_TCS_BW_150HZ;

	} else if (frequency > 40) {
		bwbits = BW_TCS_BW_75HZ;

	} else if (frequency > 20) {
		bwbits = BW_TCS_BW_40HZ;

	} else if (frequency > 10) {
		bwbits = BW_TCS_BW_20HZ;

	} else {
		bwbits = BW_TCS_BW_10HZ;
	}

	/* adjust sensor configuration */
	modify_reg(ADDR_BW_TCS, BW_TCS_BW_MASK, bwbits);

	return OK;
}

void
BMA180::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_oldest_report = _next_report = 0;

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&BMA180::measure_trampoline, this);
}

void
BMA180::stop()
{
	hrt_cancel(&_call);
}

void
BMA180::measure_trampoline(void *arg)
{
	BMA180 *dev = (BMA180 *)arg;

	/* make another measurement */
	dev->measure();
}

void
BMA180::measure()
{
	/*
	 * This evil is to deal with the stupid layout of the BMA180
	 * measurement registers vs. the SPI transaction model.
	 */
	union {
		uint8_t bytes[10];
		uint16_t words[5];
	} buf;

	/*
	 * Fetch the full set of measurements from the BMA180 in one pass;
	 * 7 bytes starting from the X LSB.
	 */
	buf.bytes[1] = ADDR_ACC_X_LSB;
	transfer(&buf.bytes[1], &buf.bytes[1], 8);

	/*
	 * Adjust and scale results to mg.
	 *
	 * Note that we ignore the "new data" bits.  At any time we read, each
	 * of the axis measurements are the "most recent", even if we've seen
	 * them before.  There is no good way to synchronise with the internal
	 * measurement flow without using the external interrupt.
	 */
	_reports[_next_report].timestamp = hrt_absolute_time();
	_reports[_next_report].x = (buf.words[1] >> 2) * _range_scale;
	_reports[_next_report].y = (buf.words[2] >> 2) * _range_scale;
	_reports[_next_report].z = (buf.words[3] >> 2) * _range_scale;

	/*
	 * @todo Apply additional scaling / calibration factors here.
	 */

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_report, _num_reports);

	/* if we are running up against the oldest report, fix it */
	if (_next_report == _oldest_report)
		INCREMENT(_oldest_report, _num_reports);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);
}

void
BMA180::print_info()
{
	printf("reads:          %u\n", _reads);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
}

/**
 * Local functions in support of the shell command.
 */
namespace
{

BMA180	*g_dev;

/*
 * XXX this should just be part of the generic sensors test...
 */

int
test()
{
	int fd = -1;
	struct accel_report report;
	ssize_t sz;
	const char *reason = "test OK";

	do {

		/* get the driver */
		fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			reason = "can't open driver";
			break;
		}

		/* do a simple demand read */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			reason = "immediate read failed";
			break;
		}

		printf("single read\n");
		fflush(stdout);
		printf("time:        %lld\n", report.timestamp);
		printf("x:           %f\n", report.x);
		printf("y:           %f\n", report.y);
		printf("z:           %f\n", report.z);

	} while (0);

	printf("BMA180: %s\n", reason);

	return OK;
}

int
info()
{
	if (g_dev == nullptr) {
		fprintf(stderr, "BMA180: driver not running\n");
		return -ENOENT;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return OK;
}


} // namespace

int
bma180_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 *
	 * XXX it would be nice to have a wrapper for this...
	 */
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr) {
			fprintf(stderr, "BMA180: already loaded\n");
			return -EBUSY;
		}

		/* create the driver */
		g_dev = new BMA180(CONFIG_BMA180_SPI_BUS, (spi_dev_e)CONFIG_BMA180_SPI_DEVICE);

		if (g_dev == nullptr) {
			fprintf(stderr, "BMA180: driver alloc failed\n");
			return -ENOMEM;
		}

		if (OK != g_dev->init()) {
			fprintf(stderr, "BMA180: driver init failed\n");
			usleep(100000);
			delete g_dev;
			g_dev = nullptr;
			return -EIO;
		}

		printf("BMA180: driver started\n");
		return OK;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		return test();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		return info();

	fprintf(stderr, "unrecognised command, try 'start', 'test' or 'info'\n");
	return -EINVAL;
}

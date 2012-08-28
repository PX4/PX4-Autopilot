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
#include <nuttx/clock.h>

#include <arch/board/up_hrt.h>

#include <drivers/drv_gyro.h>

extern "C" { __EXPORT int l3gd20_main(int argc, char *argv[]); }

class L3GD20 : public device::SPI
{
public:
	L3GD20(int bus, spi_dev_e device);
	~L3GD20();

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

	struct gyro_scale	_scale;
	float			_range_scale;

	unsigned		_reads;

	unsigned		_rate;
	unsigned		_range;

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
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_g);

	/**
	 * Set the L3GD20 lowpass filter.
	 *
	 * @param frequency	Set the lowpass filter cutoff frequency to no less than
	 *			this frequency.
	 * @return		OK if the value can be supported.
	 */
	int			set_bandwidth(unsigned frequency);
};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

#define ADDR_WHO_AM_I				0x0F
#define WHO_I_AM				0xD4
#define ADDR_CTRL_REG1				0x20
#define ADDR_CTRL_REG2				0x21
#define ADDR_CTRL_REG3				0x22
#define ADDR_CTRL_REG4				0x23
#define ADDR_CTRL_REG5				0x24
#define ADDR_REFERENCE				0x25
#define ADDR_OUT_TEMP				0x26
#define ADDR_STATUS_REG				0x27
#define ADDR_OUT_X_L				0x28
#define ADDR_OUT_X_H				0x29
#define ADDR_OUT_Y_L				0x2A
#define ADDR_OUT_Y_H				0x2B
#define ADDR_OUT_Z_L				0x2C
#define ADDR_OUT_Z_H				0x2D
#define ADDR_FIFO_CTRL_REG			0x2E
#define ADDR_FIFO_SRC_REG			0x2F
#define ADDR_INT1_CFG				0x30
#define ADDR_INT1_SRC				0x31
#define ADDR_INT1_TSH_XH			0x32
#define ADDR_INT1_TSH_XL			0x33
#define ADDR_INT1_TSH_YH			0x34
#define ADDR_INT1_TSH_YL			0x35
#define ADDR_INT1_TSH_ZH			0x36
#define ADDR_INT1_TSH_ZL			0x37
#define ADDR_INT1_DURATION			0x38

#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */

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

#define L3GD20_RANGE_250DPS			(0<<4)
#define L3GD20_RANGE_500DPS			(1<<4)
#define L3GD20_RANGE_2000DPS			(3<<4)

#define L3GD20_RATE_95HZ			((0<<6) | (0<<4))
#define L3GD20_RATE_190HZ			((1<<6) | (0<<4))
#define L3GD20_RATE_380HZ			((2<<6) | (1<<4))
#define L3GD20_RATE_760HZ			((3<<6) | (2<<4))

/*
 * Driver 'main' command.
 */
extern "C" { int l3gd20_main(int argc, char *argv[]); }


L3GD20::L3GD20(int bus, spi_dev_e device) :
	SPI("L3GD20", GYRO_DEVICE_PATH, bus, device, SPIDEV_MODE3, 8000000),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_reads(0),
	_rate(L3GD20_RATE_760HZ),
	_range(L3GD20_RANGE_2000DPS)
{
	// enable debug() calls
	_debug_enabled = true;

	// default scale factors XXX
	_scale.x_offset = 0;
	_scale.x_scale  = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale  = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale  = 1.0f;
}

L3GD20::~L3GD20()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
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

	/* set default configuration */
	write_reg(ADDR_CTRL_REG1, REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_reg(ADDR_CTRL_REG3, 0);		/* no interrupts - we don't use them */
	write_reg(ADDR_CTRL_REG4, 0x10);
	write_reg(ADDR_CTRL_REG5, 0);
		
	write_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);	  /* disable wake-on-interrupt */
	write_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_STREAM_MODE); /* Enable FIFO, old data is overwritten */

	if ((set_range(L3GD20_RANGE_500DPS) != 0) ||
	    (set_rate(L3GD20_RATE_760HZ_LP_100HZ) != 0))
		goto out;

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
				/* XXX 500Hz is just a wild guess */
				return ioctl(filp, SENSORIOCSPOLLRATE, 500);

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
		return _num_reports -1;

	case SENSORIOCRESET:
		/* XXX implement */
		return -EINVAL;

	case GYROIOCSSAMPLERATE:
	case GYROIOCGSAMPLERATE:
		/* XXX not implemented */
		return -EINVAL;

	case GYROIOCSLOWPASS:
	case GYROIOCGLOWPASS:
		/* XXX not implemented */
		return -EINVAL;

	case GYROIOCSSCALE:
	case GYROIOCGSCALE:
		/* XXX not implemented */
		return -EINVAL;

	case GYROIOCSRANGE:
	case GYROIOCGRANGE:
		/* XXX not implemented */
		// XXX change these two values on set:
		// _gyro_range_scale = xx
		// _gyro_range_m_s2 = xx
		return -EINVAL;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
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
L3GD20::set_range(unsigned max_g)
{
#if 0
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
#endif
	return OK;
}

int
L3GD20::set_bandwidth(unsigned frequency)
{
#if 0
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
#endif
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
	report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
	transfer(&report, &report, sizeof(report));

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
	/* XXX adjust for sensor alignment to board here */
	report->raw_x = raw_report.x;
	report->raw_y = raw_report.y;
	report->raw_z = raw_report.z;

	report->x = ((raw_report.x * _range_scale) - _scale.x_offset) * _scale.x_scale;
	report->y = ((raw_report.y * _range_scale) - _scale.y_offset) * _scale.y_scale;
	report->z = ((raw_report.z * _range_scale) - _scale.z_offset) * _scale.z_scale;
	report->scaling = _range_scale;
	report->range_rad_s = _range_rad_s;

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &_gyro_report);

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
L3GD20::print_info()
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

L3GD20	*g_dev;

/*
 * XXX this should just be part of the generic sensors test...
 */

int
test()
{
	int fd = -1;
	struct gyro_report report;
	ssize_t sz;
	const char *reason = "test OK";

	do {

		/* get the driver */
		fd = open(GYRO_DEVICE_PATH, O_RDONLY);

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

	printf("L3GD20: %s\n", reason);

	return OK;
}

int
info()
{
	if (g_dev == nullptr) {
		fprintf(stderr, "L3GD20: driver not running\n");
		return -ENOENT;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return OK;
}


} // namespace

int
l3gd20_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 *
	 * XXX it would be nice to have a wrapper for this...
	 */
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr) {
			fprintf(stderr, "L3GD20: already loaded\n");
			return -EBUSY;
		}

		/* create the driver */
		g_dev = new L3GD20(CONFIG_L3GD20_SPI_BUS, (spi_dev_e)CONFIG_L3GD20_SPI_DEVICE);

		if (g_dev == nullptr) {
			fprintf(stderr, "L3GD20: driver alloc failed\n");
			return -ENOMEM;
		}

		if (OK != g_dev->init()) {
			fprintf(stderr, "L3GD20: driver init failed\n");
			usleep(100000);
			delete g_dev;
			g_dev = nullptr;
			return -EIO;
		}

		printf("L3GD20: driver started\n");
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

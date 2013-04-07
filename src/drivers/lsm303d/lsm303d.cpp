/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file lsm303d.cpp
 * Driver for the ST LSM303D MEMS accelerometer / magnetometer connected via SPI.
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
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT			(1<<6)



/* register addresses: A: accel, M: mag, T: temp */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM				0x49

#define ADDR_OUT_L_T         	0x05
#define ADDR_OUT_H_T        	0x06
#define ADDR_STATUS_M           0x07
#define ADDR_OUT_X_L_M          0x08
#define ADDR_OUT_X_H_M          0x09
#define ADDR_OUT_Y_L_M          0x0A
#define ADDR_OUT_Y_H_M          0x0B
#define ADDR_OUT_Z_L_M          0x0C
#define ADDR_OUT_Z_H_M          0x0D

#define ADDR_OUT_TEMP_A			0x26
#define ADDR_STATUS_A			0x27
#define ADDR_OUT_X_L_A			0x28
#define ADDR_OUT_X_H_A			0x29
#define ADDR_OUT_Y_L_A			0x2A
#define ADDR_OUT_Y_H_A			0x2B
#define ADDR_OUT_Z_L_A			0x2C
#define ADDR_OUT_Z_H_A			0x2D

#define ADDR_CTRL_REG0			0x1F
#define ADDR_CTRL_REG1			0x20
#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define ADDR_CTRL_REG5			0x24
#define ADDR_CTRL_REG7			0x26

#define REG1_POWERDOWN_A		((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_3_125HZ_A		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_6_25HZ_A		((0<<7) | (0<<6) | (1<<5) | (0<<4))
#define REG1_RATE_12_5HZ_A		((0<<7) | (0<<6) | (1<<5) | (1<<4))
#define REG1_RATE_25HZ_A		((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define REG1_RATE_50HZ_A		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define REG1_RATE_100HZ_A		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define REG1_RATE_200HZ_A		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_400HZ_A		((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_800HZ_A		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_1600HZ_A		((1<<7) | (0<<6) | (1<<5) | (0<<4))

#define REG1_CONT_UPDATE_A		(0<<3)
#define REG1_Z_ENABLE_A			(1<<2)
#define REG1_Y_ENABLE_A			(1<<1)
#define REG1_X_ENABLE_A			(1<<0)

#define REG2_AA_FILTER_BW_773HZ_A		((0<<7) | (0<<6))
#define REG2_AA_FILTER_BW_194HZ_A		((0<<7) | (1<<6))
#define REG2_AA_FILTER_BW_362HZ_A		((1<<7) | (0<<6))
#define REG2_AA_FILTER_BW_50HZ_A		((1<<7) | (1<<6))

#define REG2_FULL_SCALE_2G_A	((0<<5) | (0<<4) | (0<<3))
#define REG2_FULL_SCALE_4G_A	((0<<5) | (0<<4) | (1<<3))
#define REG2_FULL_SCALE_6G_A	((0<<5) | (1<<4) | (0<<3))
#define REG2_FULL_SCALE_8G_A	((0<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_16G_A	((1<<5) | (0<<4) | (0<<3))

#define REG5_ENABLE_T			(1<<7)

#define REG5_RES_HIGH_M			((1<<6) | (1<<5))
#define REG5_RES_LOW_M			((0<<6) | (0<<5))

#define REG5_RATE_3_125HZ_M		((0<<4) | (0<<3) | (0<<2))
#define REG5_RATE_6_25HZ_M		((0<<4) | (0<<3) | (1<<2))
#define REG5_RATE_12_5HZ_M		((0<<4) | (1<<3) | (0<<2))
#define REG5_RATE_25HZ_M		((0<<4) | (1<<3) | (1<<2))
#define REG5_RATE_50HZ_M		((1<<4) | (0<<3) | (0<<2))
#define REG5_RATE_100HZ_M		((1<<4) | (0<<3) | (1<<2))
#define REG5_RATE_DO_NOT_USE_M	((1<<4) | (1<<3) | (0<<2))

#define REG6_FULL_SCALE_2GA_M	((0<<7) | (0<<6))
#define REG6_FULL_SCALE_4GA_M	((0<<7) | (1<<6))
#define REG6_FULL_SCALE_8GA_M	((1<<7) | (0<<6))
#define REG6_FULL_SCALE_12GA_M	((1<<7) | (1<<6))

#define REG7_CONT_MODE_M		((0<<1) | (0<<0))


#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13


extern "C" { __EXPORT int lsm303d_main(int argc, char *argv[]); }


class LSM303D_mag;

class LSM303D : public device::SPI
{
public:
	LSM303D(int bus, const char* path, spi_dev_e device);
	virtual ~LSM303D();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

	friend class LSM303D_mag;

	virtual ssize_t		mag_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		mag_ioctl(struct file *filp, int cmd, unsigned long arg);

private:

	LSM303D_mag		*_mag;

	struct hrt_call		_accel_call;
	struct hrt_call		_mag_call;

	unsigned		_call_accel_interval;
	unsigned		_call_mag_interval;

	unsigned		_num_accel_reports;
	volatile unsigned	_next_accel_report;
	volatile unsigned	_oldest_accel_report;
	struct accel_report	*_accel_reports;

	struct accel_scale	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;

	unsigned		_num_mag_reports;
	volatile unsigned	_next_mag_report;
	volatile unsigned	_oldest_mag_report;
	struct mag_report	*_mag_reports;

	struct mag_scale	_mag_scale;
	float			_mag_range_scale;
	float			_mag_range_ga;
	orb_advert_t		_mag_topic;

	unsigned		_current_accel_rate;
	unsigned		_current_accel_range;

	unsigned		_current_mag_rate;
	unsigned		_current_mag_range;

	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_mag_sample_perf;

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
	 * Static trampoline for the mag because it runs at a lower rate
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		mag_measure_trampoline(void *arg);

	/**
	 * Fetch accel measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Fetch mag measurements from the sensor and update the report ring.
	 */
	void			mag_measure();

	/**
	 * Read a register from the LSM303D
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the LSM303D
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the LSM303D
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the LSM303D measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_dps);

	/**
	 * Set the LSM303D internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_samplerate(unsigned frequency);
};

/**
 * Helper class implementing the mag driver node.
 */
class LSM303D_mag : public device::CDev
{
public:
	LSM303D_mag(LSM303D *parent);
	~LSM303D_mag();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

protected:
	friend class LSM303D;

	void			parent_poll_notify();
private:
	LSM303D			*_parent;

	void			measure();

	void			measure_trampoline(void *arg);
};


/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)


LSM303D::LSM303D(int bus, const char* path, spi_dev_e device) :
	SPI("LSM303D", path, bus, device, SPIDEV_MODE3, 8000000),
	_mag(new LSM303D_mag(this)),
	_call_accel_interval(0),
	_call_mag_interval(0),
	_num_accel_reports(0),
	_next_accel_report(0),
	_oldest_accel_report(0),
	_accel_reports(nullptr),
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(-1),
	_num_mag_reports(0),
	_next_mag_report(0),
	_oldest_mag_report(0),
	_mag_reports(nullptr),
	_mag_range_scale(0.0f),
	_mag_range_ga(0.0f),
	_current_accel_rate(0),
	_current_accel_range(0),
	_current_mag_rate(0),
	_current_mag_range(0),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "lsm303d_accel_read")),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "lsm303d_mag_read"))
{
	// enable debug() calls
	_debug_enabled = true;

	/* XXX fix this default values */
	_accel_range_scale = 1.0f;
	_mag_range_scale = 1.0f;

	// default scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_mag_scale.x_offset = 0;
	_mag_scale.x_scale = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale = 1.0f;
}

LSM303D::~LSM303D()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_accel_reports != nullptr)
		delete[] _accel_reports;
	if (_mag_reports != nullptr)
		delete[] _mag_reports;

	delete _mag;

	/* delete the perf counter */
	perf_free(_accel_sample_perf);
	perf_free(_mag_sample_perf);
}

int
LSM303D::init()
{
	int ret = ERROR;
	int mag_ret;
	int fd_mag;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_accel_reports = 2;
	_oldest_accel_report = _next_accel_report = 0;
	_accel_reports = new struct accel_report[_num_accel_reports];

	if (_accel_reports == nullptr)
		goto out;

	/* advertise accel topic */
	memset(&_accel_reports[0], 0, sizeof(_accel_reports[0]));
	_accel_topic = orb_advertise(ORB_ID(sensor_accel), &_accel_reports[0]);

	_num_mag_reports = 2;
	_oldest_mag_report = _next_mag_report = 0;
	_mag_reports = new struct mag_report[_num_mag_reports];

	if (_mag_reports == nullptr)
		goto out;

	/* advertise mag topic */
	memset(&_mag_reports[0], 0, sizeof(_mag_reports[0]));
	_mag_topic = orb_advertise(ORB_ID(sensor_mag), &_mag_reports[0]);

	/* XXX do this with ioctls */
	/* set default configuration */
	write_reg(ADDR_CTRL_REG1, REG1_RATE_400HZ_A | REG1_X_ENABLE_A | REG1_Y_ENABLE_A | REG1_Z_ENABLE_A);
	write_reg(ADDR_CTRL_REG7, REG7_CONT_MODE_M);
	write_reg(ADDR_CTRL_REG5, REG5_RATE_100HZ_M | REG5_RES_HIGH_M);

	/* XXX should we enable FIFO */

	set_range(500);				/* default to 500dps */
	set_samplerate(0);			/* max sample rate */

//	_current_accel_rate = 100;

	/* XXX test this when another mag is used */
	/* do CDev init for the mag device node, keep it optional */
	mag_ret = _mag->init();

	if (mag_ret != OK) {
		_mag_topic = -1;
	}

	ret = OK;
out:
	return ret;
}

int
LSM303D::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	/* verify that the device is attached and functioning */
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM)
		return OK;

	return -EIO;
}

ssize_t
LSM303D::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct accel_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_accel_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_oldest_accel_report != _next_accel_report) {
				memcpy(buffer, _accel_reports + _oldest_accel_report, sizeof(*_accel_reports));
				ret += sizeof(_accel_reports[0]);
				INCREMENT(_oldest_accel_report, _num_accel_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_oldest_accel_report = _next_accel_report = 0;
	measure();

	/* measurement will have generated a report, copy it out */
	memcpy(buffer, _accel_reports, sizeof(*_accel_reports));
	ret = sizeof(*_accel_reports);

	return ret;
}

ssize_t
LSM303D::mag_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_mag_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_oldest_mag_report != _next_mag_report) {
				memcpy(buffer, _mag_reports + _oldest_mag_report, sizeof(*_mag_reports));
				ret += sizeof(_mag_reports[0]);
				INCREMENT(_oldest_mag_report, _num_mag_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_oldest_mag_report = _next_mag_report = 0;
	measure();

	/* measurement will have generated a report, copy it out */
	memcpy(buffer, _mag_reports, sizeof(*_mag_reports));
	ret = sizeof(*_mag_reports);

	return ret;
}

int
LSM303D::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_accel_interval = 0;
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
					bool want_start = (_call_accel_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_accel_call.period = _call_accel_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_accel_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_accel_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* account for sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct accel_report *buf = new struct accel_report[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _accel_reports;
			_num_accel_reports = arg;
			_accel_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_accel_reports - 1;

	case SENSORIOCRESET:
		/* XXX implement */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
LSM303D::mag_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
		switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_mag_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				/* 50 Hz is max for mag */
				return mag_ioctl(filp, SENSORIOCSPOLLRATE, 50);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_mag_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_mag_call.period = _call_mag_interval = ticks;



					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_mag_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_mag_interval;
	case SENSORIOCSQUEUEDEPTH:
	case SENSORIOCGQUEUEDEPTH:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case MAGIOCSSAMPLERATE:
//	case MAGIOCGSAMPLERATE:
		/* XXX not implemented */
		return -EINVAL;

	case MAGIOCSLOWPASS:
//	case MAGIOCGLOWPASS:
		/* XXX not implemented */
//		_set_dlpf_filter((uint16_t)arg);
		return -EINVAL;

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCSRANGE:
//	case MAGIOCGRANGE:
		/* XXX not implemented */
		// XXX change these two values on set:
		// _mag_range_scale = xx
		// _mag_range_ga = xx
		return -EINVAL;

	case MAGIOCSELFTEST:
		/* XXX not implemented */
//		return self_test();
		return -EINVAL;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
LSM303D::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
LSM303D::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
LSM303D::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int
LSM303D::set_range(unsigned max_dps)
{
	/* XXX implement this */
//	uint8_t bits = REG4_BDU;
//
//	if (max_dps == 0)
//		max_dps = 2000;
//
//	if (max_dps <= 250) {
//		_current_range = 250;
//		bits |= RANGE_250DPS;
//
//	} else if (max_dps <= 500) {
//		_current_range = 500;
//		bits |= RANGE_500DPS;
//
//	} else if (max_dps <= 2000) {
//		_current_range = 2000;
//		bits |= RANGE_2000DPS;
//
//	} else {
//		return -EINVAL;
//	}
//
//	_gyro_range_rad_s = _current_range / 180.0f * M_PI_F;
//	_gyro_range_scale = _gyro_range_rad_s / 32768.0f;
//	write_reg(ADDR_CTRL_REG4, bits);

	return OK;
}

int
LSM303D::set_samplerate(unsigned frequency)
{
	/* XXX implement this */
//	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
//
//	if (frequency == 0)
//		frequency = 760;
//
//	if (frequency <= 95) {
//		_current_rate = 95;
//		bits |= RATE_95HZ_LP_25HZ;
//
//	} else if (frequency <= 190) {
//		_current_rate = 190;
//		bits |= RATE_190HZ_LP_25HZ;
//
//	} else if (frequency <= 380) {
//		_current_rate = 380;
//		bits |= RATE_380HZ_LP_30HZ;
//
//	} else if (frequency <= 760) {
//		_current_rate = 760;
//		bits |= RATE_760HZ_LP_30HZ;
//
//	} else {
//		return -EINVAL;
//	}
//
//	write_reg(ADDR_CTRL_REG1, bits);

	return OK;
}

void
LSM303D::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_oldest_accel_report = _next_accel_report = 0;
	_oldest_mag_report = _next_mag_report = 0;

	/* start polling at the specified rate */
	hrt_call_every(&_accel_call, 1000, _call_accel_interval, (hrt_callout)&LSM303D::measure_trampoline, this);
	hrt_call_every(&_mag_call, 1000, _call_mag_interval, (hrt_callout)&LSM303D::mag_measure_trampoline, this);
}

void
LSM303D::stop()
{
	hrt_cancel(&_accel_call);
	hrt_cancel(&_mag_call);
}

void
LSM303D::measure_trampoline(void *arg)
{
	LSM303D *dev = (LSM303D *)arg;

	/* make another measurement */
	dev->measure();
}

void
LSM303D::mag_measure_trampoline(void *arg)
{
	LSM303D *dev = (LSM303D *)arg;

	/* make another measurement */
	dev->mag_measure();
}

void
LSM303D::measure()
{
	/* status register and data as read back from the device */

#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_accel_report;
#pragma pack(pop)

	accel_report		*accel_report = &_accel_reports[_next_accel_report];

	/* start the performance counter */
	perf_begin(_accel_sample_perf);

	/* fetch data from the sensor */
	raw_accel_report.cmd = ADDR_STATUS_A | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_accel_report, (uint8_t *)&raw_accel_report, sizeof(raw_accel_report));

	/* XXX adapt the comment to specs */
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


	accel_report->timestamp = hrt_absolute_time();
	/* XXX adjust for sensor alignment to board here */
	accel_report->x_raw = raw_accel_report.x;
	accel_report->y_raw = raw_accel_report.y;
	accel_report->z_raw = raw_accel_report.z;

	accel_report->x = ((accel_report->x_raw * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	accel_report->y = ((accel_report->y_raw * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	accel_report->z = ((accel_report->z_raw * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;
//	report->scaling = _gyro_range_scale;
//	report->range_rad_s = _gyro_range_rad_s;

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_accel_report, _num_accel_reports);

	/* if we are running up against the oldest report, fix it */
	if (_next_accel_report == _oldest_accel_report)
		INCREMENT(_oldest_accel_report, _num_accel_reports);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	orb_publish(ORB_ID(sensor_accel), _accel_topic, accel_report);

	/* stop the perf counter */
	perf_end(_accel_sample_perf);
}

void
LSM303D::mag_measure()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_mag_report;
#pragma pack(pop)

	mag_report		*mag_report = &_mag_reports[_next_mag_report];

	/* start the performance counter */
	perf_begin(_mag_sample_perf);

	/* fetch data from the sensor */
	raw_mag_report.cmd = ADDR_STATUS_M | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_mag_report, (uint8_t *)&raw_mag_report, sizeof(raw_mag_report));

	/* XXX adapt the comment to specs */
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


	mag_report->timestamp = hrt_absolute_time();
	/* XXX adjust for sensor alignment to board here */
	mag_report->x_raw = raw_mag_report.x;
	mag_report->y_raw = raw_mag_report.y;
	mag_report->z_raw = raw_mag_report.z;
	mag_report->x = ((mag_report->x_raw * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mag_report->y = ((mag_report->y_raw * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mag_report->z = ((mag_report->z_raw * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;
//	report->scaling = _gyro_range_scale;
//	report->range_rad_s = _gyro_range_rad_s;

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_mag_report, _num_mag_reports);

	/* if we are running up against the oldest report, fix it */
	if (_next_mag_report == _oldest_mag_report)
		INCREMENT(_oldest_mag_report, _num_mag_reports);

	/* XXX please check this poll_notify, is it the right one? */
	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	orb_publish(ORB_ID(sensor_mag), _mag_topic, mag_report);

	/* stop the perf counter */
	perf_end(_mag_sample_perf);
}

void
LSM303D::print_info()
{
	perf_print_counter(_accel_sample_perf);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_accel_reports, _oldest_accel_report, _next_accel_report, _accel_reports);
	perf_print_counter(_mag_sample_perf);
		printf("report queue:   %u (%u/%u @ %p)\n",
		   _num_mag_reports, _oldest_mag_report, _next_mag_report, _mag_reports);
}

LSM303D_mag::LSM303D_mag(LSM303D *parent) :
	CDev("LSM303D_mag", MAG_DEVICE_PATH),
	_parent(parent)
{
}

LSM303D_mag::~LSM303D_mag()
{
}

void
LSM303D_mag::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
LSM303D_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->mag_read(filp, buffer, buflen);
}

int
LSM303D_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return _parent->mag_ioctl(filp, cmd, arg);
}

void
LSM303D_mag::measure()
{
	_parent->mag_measure();
}

void
LSM303D_mag::measure_trampoline(void *arg)
{
	_parent->mag_measure_trampoline(arg);
}

/**
 * Local functions in support of the shell command.
 */
namespace lsm303d
{

LSM303D	*g_dev;

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
	int fd, fd_mag;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new LSM303D(1 /* XXX magic number */, ACCEL_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_ACCEL_MAG);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	fd_mag = open(MAG_DEVICE_PATH, O_RDONLY);

	/* don't fail if open cannot be opened */
	if (0 <= fd_mag) {
		if (ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}
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
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd_accel = -1;
	struct accel_report a_report;
	ssize_t sz;

	/* get the driver */
	fd_accel = open(ACCEL_DEVICE_PATH, O_RDONLY);

	if (fd_accel < 0)
		err(1, "%s open failed", ACCEL_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd_accel, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report))
		err(1, "immediate read failed");

	/* XXX fix the test output */
//	warnx("accel x: \t% 9.5f\tm/s^2", (double)a_report.x);
//	warnx("accel y: \t% 9.5f\tm/s^2", (double)a_report.y);
//	warnx("accel z: \t% 9.5f\tm/s^2", (double)a_report.z);
	warnx("accel x: \t%d\traw", (int)a_report.x_raw);
	warnx("accel y: \t%d\traw", (int)a_report.y_raw);
	warnx("accel z: \t%d\traw", (int)a_report.z_raw);
//	warnx("accel range: %8.4f m/s^2", (double)a_report.range_m_s2);



	int fd_mag = -1;
	struct mag_report m_report;

	/* get the driver */
	fd_mag = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd_mag < 0)
		err(1, "%s open failed", MAG_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report))
		err(1, "immediate read failed");

	/* XXX fix the test output */
	warnx("mag x: \t%d\traw", (int)m_report.x_raw);
	warnx("mag y: \t%d\traw", (int)m_report.y_raw);
	warnx("mag z: \t%d\traw", (int)m_report.z_raw);

	/* XXX add poll-rate tests here too */

//	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

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
lsm303d_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.

	 */
	if (!strcmp(argv[1], "start"))
		lsm303d::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		lsm303d::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		lsm303d::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		lsm303d::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}

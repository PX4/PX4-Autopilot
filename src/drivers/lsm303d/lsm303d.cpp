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
#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/device/ringbuffer.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

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
#define ADDR_CTRL_REG6			0x25
#define ADDR_CTRL_REG7			0x26

#define REG1_RATE_BITS_A		((1<<7) | (1<<6) | (1<<5) | (1<<4))
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

#define REG1_BDU_UPDATE			(1<<3)
#define REG1_Z_ENABLE_A			(1<<2)
#define REG1_Y_ENABLE_A			(1<<1)
#define REG1_X_ENABLE_A			(1<<0)

#define REG2_ANTIALIAS_FILTER_BW_BITS_A	((1<<7) | (1<<6))
#define REG2_AA_FILTER_BW_773HZ_A		((0<<7) | (0<<6))
#define REG2_AA_FILTER_BW_194HZ_A		((0<<7) | (1<<6))
#define REG2_AA_FILTER_BW_362HZ_A		((1<<7) | (0<<6))
#define REG2_AA_FILTER_BW_50HZ_A		((1<<7) | (1<<6))

#define REG2_FULL_SCALE_BITS_A	((1<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_2G_A	((0<<5) | (0<<4) | (0<<3))
#define REG2_FULL_SCALE_4G_A	((0<<5) | (0<<4) | (1<<3))
#define REG2_FULL_SCALE_6G_A	((0<<5) | (1<<4) | (0<<3))
#define REG2_FULL_SCALE_8G_A	((0<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_16G_A	((1<<5) | (0<<4) | (0<<3))

#define REG5_ENABLE_T			(1<<7)

#define REG5_RES_HIGH_M			((1<<6) | (1<<5))
#define REG5_RES_LOW_M			((0<<6) | (0<<5))

#define REG5_RATE_BITS_M		((1<<4) | (1<<3) | (1<<2))
#define REG5_RATE_3_125HZ_M		((0<<4) | (0<<3) | (0<<2))
#define REG5_RATE_6_25HZ_M		((0<<4) | (0<<3) | (1<<2))
#define REG5_RATE_12_5HZ_M		((0<<4) | (1<<3) | (0<<2))
#define REG5_RATE_25HZ_M		((0<<4) | (1<<3) | (1<<2))
#define REG5_RATE_50HZ_M		((1<<4) | (0<<3) | (0<<2))
#define REG5_RATE_100HZ_M		((1<<4) | (0<<3) | (1<<2))
#define REG5_RATE_DO_NOT_USE_M	((1<<4) | (1<<3) | (0<<2))

#define REG6_FULL_SCALE_BITS_M	((1<<6) | (1<<5))
#define REG6_FULL_SCALE_2GA_M	((0<<6) | (0<<5))
#define REG6_FULL_SCALE_4GA_M	((0<<6) | (1<<5))
#define REG6_FULL_SCALE_8GA_M	((1<<6) | (0<<5))
#define REG6_FULL_SCALE_12GA_M	((1<<6) | (1<<5))

#define REG7_CONT_MODE_M		((0<<1) | (0<<0))


#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13

/* default values for this device */
#define LSM303D_ACCEL_DEFAULT_RANGE_G			8
#define LSM303D_ACCEL_DEFAULT_RATE			800
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ	50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define LSM303D_MAG_DEFAULT_RANGE_GA			2
#define LSM303D_MAG_DEFAULT_RATE			100

#define LSM303D_ONE_G					9.80665f

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

	friend class 		LSM303D_mag;

	virtual ssize_t		mag_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		mag_ioctl(struct file *filp, int cmd, unsigned long arg);

private:

	LSM303D_mag		*_mag;

	struct hrt_call		_accel_call;
	struct hrt_call		_mag_call;

	unsigned		_call_accel_interval;
	unsigned		_call_mag_interval;

	RingBuffer		*_accel_reports;
	RingBuffer		*_mag_reports;

	struct accel_scale	_accel_scale;
	unsigned		_accel_range_m_s2;
	float			_accel_range_scale;
	unsigned		_accel_samplerate;
	unsigned		_accel_onchip_filter_bandwith;

	struct mag_scale	_mag_scale;
	unsigned		_mag_range_ga;
	float			_mag_range_scale;
	unsigned		_mag_samplerate;

	orb_advert_t		_accel_topic;
	orb_advert_t		_mag_topic;

	unsigned		_accel_read;
	unsigned		_mag_read;

	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_mag_sample_perf;
	perf_counter_t		_reg7_resets;
	perf_counter_t		_reg1_resets;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

	// expceted values of reg1 and reg7 to catch in-flight
	// brownouts of the sensor
	uint8_t			_reg7_expected;
	uint8_t			_reg1_expected;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	void			reset();

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
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int			accel_self_test();

	/**
	 * Mag self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int			mag_self_test();

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
	 * Set the LSM303D accel measurement range.
	 *
	 * @param max_g	The measurement range of the accel is in g (9.81m/s^2)
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_range(unsigned max_g);

	/**
	 * Set the LSM303D mag measurement range.
	 *
	 * @param max_ga	The measurement range of the mag is in Ga
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			mag_set_range(unsigned max_g);

	/**
	 * Set the LSM303D on-chip anti-alias filter bandwith.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth);

	/**
	 * Set the driver lowpass filter bandwidth.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
	 * Set the LSM303D internal accel sampling frequency.
	 *
	 * @param frequency	The internal accel sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			accel_set_samplerate(unsigned frequency);

	/**
	 * Set the LSM303D internal mag sampling frequency.
	 *
	 * @param frequency	The internal mag sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			mag_set_samplerate(unsigned frequency);
};

/**
 * Helper class implementing the mag driver node.
 */
class LSM303D_mag : public device::CDev
{
public:
	LSM303D_mag(LSM303D *parent);
	~LSM303D_mag();

	virtual ssize_t			read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

protected:
	friend class LSM303D;

	void				parent_poll_notify();
private:
	LSM303D				*_parent;

	void				measure();

	void				measure_trampoline(void *arg);
};


LSM303D::LSM303D(int bus, const char* path, spi_dev_e device) :
	SPI("LSM303D", path, bus, device, SPIDEV_MODE3, 8000000),
	_mag(new LSM303D_mag(this)),
	_call_accel_interval(0),
	_call_mag_interval(0),
	_accel_reports(nullptr),
	_mag_reports(nullptr),
	_accel_range_m_s2(0.0f),
	_accel_range_scale(0.0f),
	_accel_samplerate(0),
	_accel_onchip_filter_bandwith(0),
	_mag_range_ga(0.0f),
	_mag_range_scale(0.0f),
	_mag_samplerate(0),
	_accel_topic(-1),
	_mag_topic(-1),
	_accel_read(0),
	_mag_read(0),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "lsm303d_accel_read")),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "lsm303d_mag_read")),
	_reg1_resets(perf_alloc(PC_COUNT, "lsm303d_reg1_resets")),
	_reg7_resets(perf_alloc(PC_COUNT, "lsm303d_reg7_resets")),
	_accel_filter_x(LSM303D_ACCEL_DEFAULT_RATE, LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(LSM303D_ACCEL_DEFAULT_RATE, LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(LSM303D_ACCEL_DEFAULT_RATE, LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_reg1_expected(0),
	_reg7_expected(0)
{
	// enable debug() calls
	_debug_enabled = true;

	// default scale factors
	_accel_scale.x_offset = 0.0f;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0.0f;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0.0f;
	_accel_scale.z_scale  = 1.0f;

	_mag_scale.x_offset = 0.0f;
	_mag_scale.x_scale = 1.0f;
	_mag_scale.y_offset = 0.0f;
	_mag_scale.y_scale = 1.0f;
	_mag_scale.z_offset = 0.0f;
	_mag_scale.z_scale = 1.0f;
}

LSM303D::~LSM303D()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_accel_reports != nullptr)
		delete _accel_reports;
	if (_mag_reports != nullptr)
		delete _mag_reports;

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

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		warnx("SPI init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_accel_reports = new RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == nullptr)
		goto out;

	/* advertise accel topic */
	struct accel_report zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_accel_topic = orb_advertise(ORB_ID(sensor_accel), &zero_report);

	_mag_reports = new RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr)
		goto out;

	reset();

	/* advertise mag topic */
	struct mag_report zero_mag_report;
	memset(&zero_mag_report, 0, sizeof(zero_mag_report));
	_mag_topic = orb_advertise(ORB_ID(sensor_mag), &zero_mag_report);

	/* do CDev init for the mag device node, keep it optional */
	mag_ret = _mag->init();

	if (mag_ret != OK) {
		_mag_topic = -1;
	}

	ret = OK;
out:
	return ret;
}

void
LSM303D::reset()
{
	/* enable accel*/
	_reg1_expected = REG1_X_ENABLE_A | REG1_Y_ENABLE_A | REG1_Z_ENABLE_A | REG1_BDU_UPDATE | REG1_RATE_800HZ_A;
	write_reg(ADDR_CTRL_REG1, _reg1_expected);

	/* enable mag */
	_reg7_expected = REG7_CONT_MODE_M;
	write_reg(ADDR_CTRL_REG7, _reg7_expected);
	write_reg(ADDR_CTRL_REG5, REG5_RES_HIGH_M);

	accel_set_range(LSM303D_ACCEL_DEFAULT_RANGE_G);
	accel_set_samplerate(LSM303D_ACCEL_DEFAULT_RATE);
	accel_set_driver_lowpass_filter((float)LSM303D_ACCEL_DEFAULT_RATE, (float)LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
	accel_set_onchip_lowpass_filter_bandwidth(LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ);

	mag_set_range(LSM303D_MAG_DEFAULT_RANGE_GA);
	mag_set_samplerate(LSM303D_MAG_DEFAULT_RATE);

	_accel_read = 0;
	_mag_read = 0;
}

int
LSM303D::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	/* verify that the device is attached and functioning */
	bool success = (read_reg(ADDR_WHO_AM_I) == WHO_I_AM);
	
	if (success)
		return OK;

	return -EIO;
}

ssize_t
LSM303D::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct accel_report);
	accel_report *arb = reinterpret_cast<accel_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_accel_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 */
		while (count--) {
			if (_accel_reports->get(arb)) {
				ret += sizeof(*arb);
				arb++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	measure();

	/* measurement will have generated a report, copy it out */
	if (_accel_reports->get(arb))
		ret = sizeof(*arb);

	return ret;
}

ssize_t
LSM303D::mag_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	mag_report *mrb = reinterpret_cast<mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_mag_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 */
		while (count--) {
			if (_mag_reports->get(mrb)) {
				ret += sizeof(*mrb);
				mrb++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_mag_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_mag_reports->get(mrb))
		ret = sizeof(*mrb);

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
				return ioctl(filp, SENSORIOCSPOLLRATE, 1600);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, LSM303D_ACCEL_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
				/* do we need to start internal polling? */
				bool want_start = (_call_accel_interval == 0);

				/* convert hz to hrt interval via microseconds */
				unsigned ticks = 1000000 / arg;

				/* check against maximum sane rate */
				if (ticks < 500)
					return -EINVAL;

				/* adjust filters */
				accel_set_driver_lowpass_filter((float)arg, _accel_filter_x.get_cutoff_freq());

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
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_accel_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _accel_reports->size();

	case SENSORIOCRESET:
		reset();
		return OK;

	case ACCELIOCSSAMPLERATE:
		return accel_set_samplerate(arg);

	case ACCELIOCGSAMPLERATE:
		return _accel_samplerate;

	case ACCELIOCSLOWPASS: {
		return accel_set_driver_lowpass_filter((float)_accel_samplerate, (float)arg);
	}

	case ACCELIOCGLOWPASS:
		return _accel_filter_x.get_cutoff_freq();

	case ACCELIOCSSCALE: {
		/* copy scale, but only if off by a few percent */
		struct accel_scale *s = (struct accel_scale *) arg;
		float sum = s->x_scale + s->y_scale + s->z_scale;
		if (sum > 2.0f && sum < 4.0f) {
			memcpy(&_accel_scale, s, sizeof(_accel_scale));
			return OK;
		} else {
			return -EINVAL;
		}
	}

	case ACCELIOCSRANGE:
		/* arg needs to be in G */
		return accel_set_range(arg);

	case ACCELIOCGRANGE:
		/* convert to m/s^2 and return rounded in G */
		return (unsigned long)((_accel_range_m_s2)/LSM303D_ONE_G + 0.5f);

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSELFTEST:
		return accel_self_test();

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
				/* 100 Hz is max for mag */
				return mag_ioctl(filp, SENSORIOCSPOLLRATE, 100);

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
	
	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_mag_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _mag_reports->size();

	case SENSORIOCRESET:
		reset();
		return OK;

	case MAGIOCSSAMPLERATE:
		return mag_set_samplerate(arg);

	case MAGIOCGSAMPLERATE:
		return _mag_samplerate;

	case MAGIOCSLOWPASS:
	case MAGIOCGLOWPASS:
		/* not supported, no internal filtering */
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
		return mag_set_range(arg);

	case MAGIOCGRANGE:
		return _mag_range_ga;

	case MAGIOCSELFTEST:
		return mag_self_test();

	case MAGIOCGEXTERNAL:
		/* no external mag board yet */
		return 0;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
LSM303D::accel_self_test()
{
	if (_accel_read == 0)
		return 1;

	/* inspect accel offsets */
	if (fabsf(_accel_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f)
		return 1;

	return 0;
}

int
LSM303D::mag_self_test()
{
	if (_mag_read == 0)
		return 1;

	/**
	 * inspect mag offsets
	 * don't check mag scale because it seems this is calibrated on chip
	 */
	if (fabsf(_mag_scale.x_offset) < 0.000001f)
		return 1;

	if (fabsf(_mag_scale.y_offset) < 0.000001f)
		return 1;

	if (fabsf(_mag_scale.z_offset) < 0.000001f)
		return 1;

	return 0;
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
LSM303D::accel_set_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG2_FULL_SCALE_BITS_A;
	float new_scale_g_digit = 0.0f;

	if (max_g == 0)
		max_g = 16;

	if (max_g <= 2) {
		_accel_range_m_s2 = 2.0f*LSM303D_ONE_G;
		setbits |= REG2_FULL_SCALE_2G_A;
		new_scale_g_digit = 0.061e-3f;

	} else if (max_g <= 4) {
		_accel_range_m_s2 = 4.0f*LSM303D_ONE_G;
		setbits |= REG2_FULL_SCALE_4G_A;
		new_scale_g_digit = 0.122e-3f;

	} else if (max_g <= 6) {
		_accel_range_m_s2 = 6.0f*LSM303D_ONE_G;
		setbits |= REG2_FULL_SCALE_6G_A;
		new_scale_g_digit = 0.183e-3f;

	} else if (max_g <= 8) {
		_accel_range_m_s2 = 8.0f*LSM303D_ONE_G;
		setbits |= REG2_FULL_SCALE_8G_A;
		new_scale_g_digit = 0.244e-3f;

	} else if (max_g <= 16) {
		_accel_range_m_s2 = 16.0f*LSM303D_ONE_G;
		setbits |= REG2_FULL_SCALE_16G_A;
		new_scale_g_digit = 0.732e-3f;

	} else {
		return -EINVAL;
	}

	_accel_range_scale = new_scale_g_digit * LSM303D_ONE_G;


	modify_reg(ADDR_CTRL_REG2, clearbits, setbits);

	return OK;
}

int
LSM303D::mag_set_range(unsigned max_ga)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG6_FULL_SCALE_BITS_M;
	float new_scale_ga_digit = 0.0f;

	if (max_ga == 0)
		max_ga = 12;

	if (max_ga <= 2) {
		_mag_range_ga = 2;
		setbits |= REG6_FULL_SCALE_2GA_M;
		new_scale_ga_digit = 0.080e-3f;

	} else if (max_ga <= 4) {
		_mag_range_ga = 4;
		setbits |= REG6_FULL_SCALE_4GA_M;
		new_scale_ga_digit = 0.160e-3f;

	} else if (max_ga <= 8) {
		_mag_range_ga = 8;
		setbits |= REG6_FULL_SCALE_8GA_M;
		new_scale_ga_digit = 0.320e-3f;

	} else if (max_ga <= 12) {
		_mag_range_ga = 12;
		setbits |= REG6_FULL_SCALE_12GA_M;
		new_scale_ga_digit = 0.479e-3f;

	} else {
		return -EINVAL;
	}

	_mag_range_scale = new_scale_ga_digit;

	modify_reg(ADDR_CTRL_REG6, clearbits, setbits);

	return OK;
}

int
LSM303D::accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG2_ANTIALIAS_FILTER_BW_BITS_A;

	if (bandwidth == 0)
		bandwidth = 773;

	if (bandwidth <= 50) {
		setbits |= REG2_AA_FILTER_BW_50HZ_A;
		_accel_onchip_filter_bandwith = 50;

	} else if (bandwidth <= 194) {
		setbits |= REG2_AA_FILTER_BW_194HZ_A;
		_accel_onchip_filter_bandwith = 194;

	} else if (bandwidth <= 362) {
		setbits |= REG2_AA_FILTER_BW_362HZ_A;
		_accel_onchip_filter_bandwith = 362;

	} else if (bandwidth <= 773) {
		setbits |= REG2_AA_FILTER_BW_773HZ_A;
		_accel_onchip_filter_bandwith = 773;

	} else {
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG2, clearbits, setbits);

	return OK;
}

int
LSM303D::accel_set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);

	return OK;
}

int
LSM303D::accel_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG1_RATE_BITS_A;

	if (frequency == 0)
		frequency = 1600;

	if (frequency <= 100) {
		setbits |= REG1_RATE_100HZ_A;
		_accel_samplerate = 100;

	} else if (frequency <= 200) {
		setbits |= REG1_RATE_200HZ_A;
		_accel_samplerate = 200;

	} else if (frequency <= 400) {
		setbits |= REG1_RATE_400HZ_A;
		_accel_samplerate = 400;

	} else if (frequency <= 800) {
		setbits |= REG1_RATE_800HZ_A;
		_accel_samplerate = 800;

	} else if (frequency <= 1600) {
		setbits |= REG1_RATE_1600HZ_A;
		_accel_samplerate = 1600;

	} else {
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG1, clearbits, setbits);
	_reg1_expected = (_reg1_expected & ~clearbits) | setbits;

	return OK;
}

int
LSM303D::mag_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG5_RATE_BITS_M;

	if (frequency == 0)
		frequency = 100;

	if (frequency <= 25) {
		setbits |= REG5_RATE_25HZ_M;
		_mag_samplerate = 25;

	} else if (frequency <= 50) {
		setbits |= REG5_RATE_50HZ_M;
		_mag_samplerate = 50;

	} else if (frequency <= 100) {
		setbits |= REG5_RATE_100HZ_M;
		_mag_samplerate = 100;

	} else {
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG5, clearbits, setbits);

	return OK;
}

void
LSM303D::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_accel_reports->flush();
	_mag_reports->flush();

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
	if (read_reg(ADDR_CTRL_REG1) != _reg1_expected) {
		perf_count(_reg1_resets);
		reset();
		return;
	}

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

	accel_report accel_report;

	/* start the performance counter */
	perf_begin(_accel_sample_perf);

	/* fetch data from the sensor */
	raw_accel_report.cmd = ADDR_STATUS_A | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_accel_report, (uint8_t *)&raw_accel_report, sizeof(raw_accel_report));

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


	accel_report.timestamp = hrt_absolute_time();
        accel_report.error_count = 0; // not reported

	accel_report.x_raw = raw_accel_report.x;
	accel_report.y_raw = raw_accel_report.y;
	accel_report.z_raw = raw_accel_report.z;

	float x_in_new = ((accel_report.x_raw * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((accel_report.y_raw * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((accel_report.z_raw * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	accel_report.x = _accel_filter_x.apply(x_in_new);
	accel_report.y = _accel_filter_y.apply(y_in_new);
	accel_report.z = _accel_filter_z.apply(z_in_new);

	accel_report.scaling = _accel_range_scale;
	accel_report.range_m_s2 = _accel_range_m_s2;

	_accel_reports->force(&accel_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);

	_accel_read++;

	/* stop the perf counter */
	perf_end(_accel_sample_perf);
}

void
LSM303D::mag_measure()
{
	if (read_reg(ADDR_CTRL_REG7) != _reg7_expected) {
		perf_count(_reg7_resets);
		reset();
		return;
	}

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

	mag_report mag_report;

	/* start the performance counter */
	perf_begin(_mag_sample_perf);

	/* fetch data from the sensor */
	raw_mag_report.cmd = ADDR_STATUS_M | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_mag_report, (uint8_t *)&raw_mag_report, sizeof(raw_mag_report));

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


	mag_report.timestamp = hrt_absolute_time();

	mag_report.x_raw = raw_mag_report.x;
	mag_report.y_raw = raw_mag_report.y;
	mag_report.z_raw = raw_mag_report.z;
	mag_report.x = ((mag_report.x_raw * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mag_report.y = ((mag_report.y_raw * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mag_report.z = ((mag_report.z_raw * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;
	mag_report.scaling = _mag_range_scale;
	mag_report.range_ga = (float)_mag_range_ga;

	_mag_reports->force(&mag_report);

	/* XXX please check this poll_notify, is it the right one? */
	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	orb_publish(ORB_ID(sensor_mag), _mag_topic, &mag_report);

	_mag_read++;

	/* stop the perf counter */
	perf_end(_mag_sample_perf);
}

void
LSM303D::print_info()
{
	printf("accel reads:          %u\n", _accel_read);
	printf("mag reads:            %u\n", _mag_read);
	perf_print_counter(_accel_sample_perf);
	_accel_reports->print_info("accel reports");
	_mag_reports->print_info("mag reports");
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
		errx(0, "already started");

	/* create the driver */
	g_dev = new LSM303D(1 /* XXX magic number */, ACCEL_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_ACCEL_MAG);

	if (g_dev == nullptr) {
		warnx("failed instantiating LSM303D obj");
		goto fail;
	}

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
	struct accel_report accel_report;
	ssize_t sz;
	int ret;

	/* get the driver */
	fd_accel = open(ACCEL_DEVICE_PATH, O_RDONLY);

	if (fd_accel < 0)
		err(1, "%s open failed", ACCEL_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd_accel, &accel_report, sizeof(accel_report));

	if (sz != sizeof(accel_report))
		err(1, "immediate read failed");


	warnx("accel x: \t% 9.5f\tm/s^2", (double)accel_report.x);
	warnx("accel y: \t% 9.5f\tm/s^2", (double)accel_report.y);
	warnx("accel z: \t% 9.5f\tm/s^2", (double)accel_report.z);
	warnx("accel x: \t%d\traw", (int)accel_report.x_raw);
	warnx("accel y: \t%d\traw", (int)accel_report.y_raw);
	warnx("accel z: \t%d\traw", (int)accel_report.z_raw);

	warnx("accel range: %8.4f m/s^2", (double)accel_report.range_m_s2);
	if (ERROR == (ret = ioctl(fd_accel, ACCELIOCGLOWPASS, 0)))
		warnx("accel antialias filter bandwidth: fail");
	else
		warnx("accel antialias filter bandwidth: %d Hz", ret);

	int fd_mag = -1;
	struct mag_report m_report;

	/* get the driver */
	fd_mag = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd_mag < 0)
		err(1, "%s open failed", MAG_DEVICE_PATH);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd_mag, MAGIOCGEXTERNAL, 0)) < 0)
		errx(1, "failed to get if mag is onboard or external");
	warnx("mag device active: %s", ret ? "external" : "onboard");

	/* do a simple demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report))
		err(1, "immediate read failed");

	warnx("mag x: \t% 9.5f\tga", (double)m_report.x);
	warnx("mag y: \t% 9.5f\tga", (double)m_report.y);
	warnx("mag z: \t% 9.5f\tga", (double)m_report.z);
	warnx("mag x: \t%d\traw", (int)m_report.x_raw);
	warnx("mag y: \t%d\traw", (int)m_report.y_raw);
	warnx("mag z: \t%d\traw", (int)m_report.z_raw);
	warnx("mag range: %8.4f ga", (double)m_report.range_ga);

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
	int fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "accel pollrate reset failed");

	fd = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		warnx("mag could not be opened, external mag might be used");
	} else {
		/* no need to reset the mag as well, the reset() is the same */
		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
			err(1, "mag pollrate reset failed");
	}

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

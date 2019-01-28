/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file l3gd20.cpp
 * Driver for the ST L3GD20 MEMS and L3GD20H mems gyros connected via SPI.
 *
 * Note: With the exception of the self-test feature, the ST L3G4200D is
 *       also supported by this driver.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>

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

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/device/spi.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

#define L3GD20_DEVICE_PATH "/dev/l3gd20"

/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM_H 				0xD7
#define WHO_I_AM				0xD4
#define WHO_I_AM_L3G4200D		0xD3	/* for L3G4200D */

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */

/* keep lowpass low to avoid noise issues */
#define RATE_95HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_190HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_380HZ_LP_20HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_380HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_760HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_760HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_760HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_760HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

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
#define ADDR_LOW_ODR			0x39


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

#define L3GD20_DEFAULT_RATE			760
#define L3G4200D_DEFAULT_RATE			800
#define L3GD20_MAX_OUTPUT_RATE			280
#define L3GD20_DEFAULT_RANGE_DPS		2000
#define L3GD20_DEFAULT_FILTER_FREQ		30
#define L3GD20_TEMP_OFFSET_CELSIUS		40

#define L3GD20_MAX_OFFSET			0.45f /**< max offset: 25 degrees/s */

#ifndef SENSOR_BOARD_ROTATION_DEFAULT
#define SENSOR_BOARD_ROTATION_DEFAULT		SENSOR_BOARD_ROTATION_270_DEG
#endif

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define L3GD20_TIMER_REDUCTION				600

extern "C" { __EXPORT int l3gd20_main(int argc, char *argv[]); }

class L3GD20 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	L3GD20(int bus, const char *path, uint32_t device, enum Rotation rotation);
	virtual ~L3GD20();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	// print register dump
	void			print_registers();

	// trigger an error
	void			test_error();

protected:
	virtual int		probe();

private:

	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_reports;

	struct gyro_calibration_s	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;
	orb_advert_t		_gyro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	unsigned		_current_rate;
	unsigned		_orientation;

	unsigned		_read;

	perf_counter_t		_sample_perf;
	perf_counter_t		_errors;
	perf_counter_t		_bad_registers;
	perf_counter_t		_duplicates;

	uint8_t			_register_wait;

	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	Integrator		_gyro_int;

	/* true if an L3G4200D is detected */
	bool	_is_l3g4200d;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define L3GD20_NUM_CHECKED_REGISTERS 8
	static const uint8_t	_checked_registers[L3GD20_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[L3GD20_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset the driver
	 */
	void			reset();

	/**
	 * disable I2C on the chip
	 */
	void			disable_i2c();

	void		Run() override;

	/**
	 * check key registers for correct values
	 */
	void			check_registers(void);

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
	 * Write a register in the L3GD20, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

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

	/**
	 * Set the lowpass filter of the driver
	 *
	 * @param samplerate	The current samplerate
	 * @param frequency	The cutoff frequency for the lowpass filter
	 */
	void			set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/* this class does not allow copying */
	L3GD20(const L3GD20 &);
	L3GD20 operator=(const L3GD20 &);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t L3GD20::_checked_registers[L3GD20_NUM_CHECKED_REGISTERS] = { ADDR_WHO_AM_I,
									   ADDR_CTRL_REG1,
									   ADDR_CTRL_REG2,
									   ADDR_CTRL_REG3,
									   ADDR_CTRL_REG4,
									   ADDR_CTRL_REG5,
									   ADDR_FIFO_CTRL_REG,
									   ADDR_LOW_ODR
									 };

L3GD20::L3GD20(int bus, const char *path, uint32_t device, enum Rotation rotation) :
	SPI("L3GD20", path, bus, device, SPIDEV_MODE3,
	    11 * 1000 * 1000 /* will be rounded to 10.4 MHz, within margins for L3GD20 */),
	ScheduledWorkItem(px4::device_bus_to_wq(this->get_device_id())),
	_call_interval(0),
	_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_current_rate(0),
	_orientation(SENSOR_BOARD_ROTATION_DEFAULT),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "l3gd20_read")),
	_errors(perf_alloc(PC_COUNT, "l3gd20_err")),
	_bad_registers(perf_alloc(PC_COUNT, "l3gd20_bad_reg")),
	_duplicates(perf_alloc(PC_COUNT, "l3gd20_dupe")),
	_register_wait(0),
	_gyro_filter_x(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_gyro_filter_y(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_gyro_filter_z(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_gyro_int(1000000 / L3GD20_MAX_OUTPUT_RATE, true),
	_is_l3g4200d(false),
	_rotation(rotation),
	_checked_next(0)
{
	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_L3GD20;

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
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_errors);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
L3GD20::init()
{
	int ret = PX4_ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	reset();

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_gyro_s grp;
	_reports->get(&grp);

	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
					  &_orb_class_instance, (external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

	if (_gyro_topic == nullptr) {
		DEVICE_DEBUG("failed to create sensor_gyro publication");
	}

	ret = OK;
out:
	return ret;
}

int
L3GD20::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	bool success = false;
	uint8_t v = 0;

	/* verify that the device is attached and functioning, accept
	 * L3GD20, L3GD20H and L3G4200D */
	if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM) {
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;

	} else if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_H) {
		_orientation = SENSOR_BOARD_ROTATION_180_DEG;
		success = true;

	} else if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_L3G4200D) {
		/* Detect the L3G4200D used on AeroCore */
		_is_l3g4200d = true;
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;
	}

	if (success) {
		_checked_values[0] = v;
		return OK;
	}

	return -EIO;
}

ssize_t
L3GD20::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_gyro_s);
	sensor_gyro_s *gbuf = reinterpret_cast<sensor_gyro_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_reports->get(gbuf)) {
				ret += sizeof(*gbuf);
				gbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(gbuf)) {
		ret = sizeof(*gbuf);
	}

	return ret;
}

int
L3GD20::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				if (_is_l3g4200d) {
					return ioctl(filp, SENSORIOCSPOLLRATE, L3G4200D_DEFAULT_RATE);
				}

				return ioctl(filp, SENSORIOCSPOLLRATE, L3GD20_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned interval = 1000000 / arg;

					/* check against maximum sane rate */
					if (interval < 1000) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = interval;

					/* adjust filters */
					float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / interval;
					set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		reset();
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

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
	cmd[1] = 0;

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
L3GD20::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < L3GD20_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}


void
L3GD20::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
L3GD20::set_range(unsigned max_dps)
{
	uint8_t bits = REG4_BDU;
	float new_range_scale_dps_digit;
	float new_range;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 250) {
		new_range = 250;
		bits |= RANGE_250DPS;
		new_range_scale_dps_digit = 8.75e-3f;

	} else if (max_dps <= 500) {
		new_range = 500;
		bits |= RANGE_500DPS;
		new_range_scale_dps_digit = 17.5e-3f;

	} else if (max_dps <= 2000) {
		new_range = 2000;
		bits |= RANGE_2000DPS;
		new_range_scale_dps_digit = 70e-3f;

	} else {
		return -EINVAL;
	}

	_gyro_range_rad_s = new_range / 180.0f * M_PI_F;
	_gyro_range_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;
	write_checked_reg(ADDR_CTRL_REG4, bits);

	return OK;
}

int
L3GD20::set_samplerate(unsigned frequency)
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

	if (frequency == 0) {
		frequency = _is_l3g4200d ? 800 : 760;
	}

	/*
	 * Use limits good for H or non-H models. Rates are slightly different
	 * for L3G4200D part but register settings are the same.
	 */
	if (frequency <= 100) {
		_current_rate = _is_l3g4200d ? 100 : 95;
		bits |= RATE_95HZ_LP_25HZ;

	} else if (frequency <= 200) {
		_current_rate = _is_l3g4200d ? 200 : 190;
		bits |= RATE_190HZ_LP_50HZ;

	} else if (frequency <= 400) {
		_current_rate = _is_l3g4200d ? 400 : 380;
		bits |= RATE_380HZ_LP_50HZ;

	} else if (frequency <= 800) {
		_current_rate = _is_l3g4200d ? 800 : 760;
		bits |= RATE_760HZ_LP_50HZ;

	} else {
		return -EINVAL;
	}

	write_checked_reg(ADDR_CTRL_REG1, bits);

	return OK;
}

void
L3GD20::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_gyro_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
L3GD20::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	ScheduleOnInterval(_call_interval - L3GD20_TIMER_REDUCTION, 10000);
}

void
L3GD20::stop()
{
	ScheduleClear();
}

void
L3GD20::disable_i2c(void)
{
	uint8_t retries = 10;

	while (retries--) {
		// add retries
		uint8_t a = read_reg(0x05);
		write_reg(0x05, (0x20 | a));

		if (read_reg(0x05) == (a | 0x20)) {
			// this sets the I2C_DIS bit on the
			// L3GD20H. The l3gd20 datasheet doesn't
			// mention this register, but it does seem to
			// accept it.
			write_checked_reg(ADDR_LOW_ODR, 0x08);
			return;
		}
	}

	DEVICE_DEBUG("FAILED TO DISABLE I2C");
}

void
L3GD20::reset()
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();

	/* set default configuration */
	write_checked_reg(ADDR_CTRL_REG1,
			  REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_checked_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_checked_reg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
	write_checked_reg(ADDR_CTRL_REG4, REG4_BDU);
	write_checked_reg(ADDR_CTRL_REG5, 0);
	write_checked_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	write_checked_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

	set_samplerate(0); // 760Hz or 800Hz
	set_range(L3GD20_DEFAULT_RANGE_DPS);
	set_driver_lowpass_filter(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ);

	_read = 0;
}

void
L3GD20::Run()
{
	/* make another measurement */
	measure();
}

void
L3GD20::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus. We skip zero as that is the WHO_AM_I, which
		  is not writeable
		 */
		if (_checked_next != 0) {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % L3GD20_NUM_CHECKED_REGISTERS;
}

void
L3GD20::measure()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		int8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_report;
#pragma pack(pop)

	sensor_gyro_s report;

	/* start the performance counter */
	perf_begin(_sample_perf);

	check_registers();

	/* fetch data from the sensor */
	memset(&raw_report, 0, sizeof(raw_report));
	raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

	if (!(raw_report.status & STATUS_ZYXDA)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		return;
	}

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
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_bad_registers);

	switch (_orientation) {

	case SENSOR_BOARD_ROTATION_000_DEG:
		/* keep axes in place */
		report.x_raw = raw_report.x;
		report.y_raw = raw_report.y;
		break;

	case SENSOR_BOARD_ROTATION_090_DEG:
		/* swap x and y */
		report.x_raw = raw_report.y;
		report.y_raw = raw_report.x;
		break;

	case SENSOR_BOARD_ROTATION_180_DEG:
		/* swap x and y and negate both */
		report.x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
		report.y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
		break;

	case SENSOR_BOARD_ROTATION_270_DEG:
		/* swap x and y and negate y */
		report.x_raw = raw_report.y;
		report.y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
		break;
	}

	report.z_raw = raw_report.z;

	float xraw_f = report.x_raw;
	float yraw_f = report.y_raw;
	float zraw_f = report.z_raw;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float xin = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float yin = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float zin = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	report.x = _gyro_filter_x.apply(xin);
	report.y = _gyro_filter_y.apply(yin);
	report.z = _gyro_filter_z.apply(zin);

	matrix::Vector3f gval(xin, yin, zin);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(report.timestamp, gval, gval_integrated, report.integral_dt);
	report.x_integral = gval_integrated(0);
	report.y_integral = gval_integrated(1);
	report.z_integral = gval_integrated(2);

	report.temperature = L3GD20_TEMP_OFFSET_CELSIUS - raw_report.temp;

	report.scaling = _gyro_range_scale;

	/* return device ID */
	report.device_id = _device_id.devid;

	_reports->force(&report);

	if (gyro_notify) {
		/* notify anyone waiting for data */
		poll_notify(POLLIN);

		/* publish for subscribers */
		if (!(_pub_blocked)) {
			/* publish it */
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &report);
		}
	}

	_read++;

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
L3GD20::print_info()
{
	printf("gyro reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_errors);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);
	_reports->print_info("report queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < L3GD20_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}
	}
}

void
L3GD20::print_registers()
{
	printf("L3GD20 registers\n");

	for (uint8_t reg = 0; reg <= 0x40; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if ((reg + 1) % 16 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}

void
L3GD20::test_error()
{
	// trigger a deliberate error
	write_reg(ADDR_CTRL_REG3, 0);
}

int
L3GD20::self_test()
{
	/* evaluate gyro offsets, complain if offset larger than 25 dps */
	if (fabsf(_gyro_scale.x_offset) > L3GD20_MAX_OFFSET) {
		return 1;
	}

	if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_offset) > L3GD20_MAX_OFFSET) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_offset) > L3GD20_MAX_OFFSET) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f) {
		return 1;
	}

	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace l3gd20
{

L3GD20	*g_dev;

void	usage();
void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	regdump();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;

	if (g_dev != nullptr) {
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_GYRO)
		g_dev = new L3GD20(PX4_SPI_BUS_EXT, L3GD20_DEVICE_PATH, PX4_SPIDEV_EXT_GYRO, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		g_dev = new L3GD20(PX4_SPI_BUS_SENSORS, L3GD20_DEVICE_PATH, PX4_SPIDEV_GYRO, rotation);
	}

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(L3GD20_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

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
	sensor_gyro_s g_report;
	ssize_t sz;

	/* get the driver */
	fd_gyro = open(L3GD20_DEVICE_PATH, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", L3GD20_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd_gyro);

	/* XXX add poll-rate tests here too */
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(L3GD20_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "accel pollrate reset failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running\n");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(void)
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

/**
 * trigger an error
 */
void
test_error(void)
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->test_error();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
l3gd20_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			l3gd20::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		l3gd20::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		l3gd20::start(external_bus, rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		l3gd20::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		l3gd20::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		l3gd20::info();
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		l3gd20::regdump();
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		l3gd20::test_error();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
	return -1;
}

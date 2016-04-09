/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mag.cpp
 *
 * Driver for the ak8963 magnetometer within the Invensense mpu9250
 *
 * @author Robert Dickenson
 *
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#include <systemlib/perf_counter.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mag.h"
#include "mpu9250.h"


/* in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */

#define MPU9250_MAG_RANGE_GA        1.5e-3f;

/* we are using the continuous fixed sampling rate of 100Hz */

#define MPU9250_AK8963_SAMPLE_RATE 100


/* mpu9250 master i2c bus specific register address and bit definitions */

#define MPUREG_I2C_MST_STATUS       0x36

#define BIT_I2C_READ_FLAG           0x80

#define MPUREG_I2C_MST_CTRL         0x24
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27

#define MPUREG_I2C_SLV4_ADDR        0x31
#define MPUREG_I2C_SLV4_REG         0x32
#define MPUREG_I2C_SLV4_DO          0x33
#define MPUREG_I2C_SLV4_CTRL        0x34
#define MPUREG_I2C_SLV4_DI          0x35

#define MPUREG_EXT_SENS_DATA_00     0x49

#define MPUREG_I2C_SLV0_D0          0x63
#define MPUREG_I2C_MST_DELAY_CTRL   0x67
#define MPUREG_USER_CTRL            0x6A

#define BIT_I2C_SLV0_NACK           0x01
#define BIT_I2C_FIFO_EN             0x40
#define BIT_I2C_MST_EN              0x20
#define BIT_I2C_IF_DIS              0x10
#define BIT_FIFO_RST                0x04
#define BIT_I2C_MST_RST             0x02
#define BIT_SIG_COND_RST            0x01

#define BIT_I2C_SLV0_EN             0x80
#define BIT_I2C_SLV0_BYTE_SW        0x40
#define BIT_I2C_SLV0_REG_DIS        0x20
#define BIT_I2C_SLV0_REG_GRP        0x10

#define BIT_I2C_MST_MULT_MST_EN     0x80
#define BIT_I2C_MST_WAIT_FOR_ES     0x40
#define BIT_I2C_MST_SLV_3_FIFO_EN   0x20
#define BIT_I2C_MST_P_NSR           0x10
#define BITS_I2C_MST_CLOCK_258HZ    0x08
#define BITS_I2C_MST_CLOCK_400HZ    0x0D

#define BIT_I2C_SLV0_DLY_EN         0x01
#define BIT_I2C_SLV1_DLY_EN         0x02
#define BIT_I2C_SLV2_DLY_EN         0x04
#define BIT_I2C_SLV3_DLY_EN         0x08


/* ak8963 register address and bit definitions */

#define AK8963_I2C_ADDR         0x0C
#define AK8963_DEVICE_ID        0x48

#define AK8963REG_WIA           0x00
#define AK8963REG_ST1           0x02
#define AK8963REG_HXL           0x03
#define AK8963REG_ASAX          0x10
#define AK8963REG_CNTL1         0x0A
#define AK8963REG_CNTL2         0x0B

#define AK8963_SINGLE_MEAS_MODE 0x01
#define AK8963_CONTINUOUS_MODE1 0x02
#define AK8963_CONTINUOUS_MODE2 0x06
#define AK8963_POWERDOWN_MODE   0x00
#define AK8963_SELFTEST_MODE    0x08
#define AK8963_FUZE_MODE        0x0F
#define AK8963_16BIT_ADC        0x10
#define AK8963_14BIT_ADC        0x00
#define AK8963_RESET            0x01


MPU9250_mag::MPU9250_mag(MPU9250 *parent, const char *path) :
	CDev("MPU9250_mag", path),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1),
	_mag_reading_data(false),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(),
	_mag_reads(perf_alloc(PC_COUNT, "mpu9250_mag_reads")),
	_mag_errors(perf_alloc(PC_COUNT, "mpu9250_mag_errors")),
	_mag_overruns(perf_alloc(PC_COUNT, "mpu9250_mag_overruns")),
	_mag_overflows(perf_alloc(PC_COUNT, "mpu9250_mag_overflows")),
	_mag_duplicates(perf_alloc(PC_COUNT, "mpu9250_mag_duplicates")),
	_mag_asa_x(1.0),
	_mag_asa_y(1.0),
	_mag_asa_z(1.0),
	_last_mag_data{}
{
	// default mag scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;

	_mag_range_scale = MPU9250_MAG_RANGE_GA;
}

MPU9250_mag::~MPU9250_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	perf_free(_mag_reads);
	perf_free(_mag_errors);
	perf_free(_mag_overruns);
	perf_free(_mag_overflows);
	perf_free(_mag_duplicates);
}

int
MPU9250_mag::init()
{
	int ret;

	ret = CDev::init();

	/* if setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("MPU9250 mag init failed");
		return ret;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr) {
		goto out;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ak8963_setup();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report mrp;
	_mag_reports->get(&mrp);

	_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					 &_mag_orb_class_instance, ORB_PRIO_LOW);
//			   &_mag_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_mag_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

bool MPU9250_mag::check_duplicate(uint8_t *mag_data)
{
	if (memcmp(mag_data, &_last_mag_data, sizeof(_last_mag_data)) == 0) {
		// it isn't new data - wait for next timer
		return true;

	} else {
		memcpy(&_last_mag_data, mag_data, sizeof(_last_mag_data));
		return false;
	}
}

void
MPU9250_mag::measure(struct ak8963_regs data)
{
	bool mag_notify = true;

	if (check_duplicate((uint8_t *)&data.x) && !(data.st1 & 0x02)) {
		perf_count(_mag_duplicates);
		return;
	}

	/* monitor for if data overrun flag is ever set */
	if (data.st1 & 0x02) {
		perf_count(_mag_overruns);
	}

	/* monitor for if magnetic sensor overflow flag is ever set noting that st2
	 * is usually not even refreshed, but will always be in the same place in the
	 * mpu's buffers regardless, hence the actual count would be bogus
	 */
	if (data.st2 & 0x08) {
		perf_count(_mag_overflows);
	}

	mag_report	mrb;
	mrb.timestamp = hrt_absolute_time();

	/*
	 * Align axes - note the accel & gryo are also re-aligned so this
	 *              doesn't look obvious with the datasheet
	 */
	mrb.x_raw =  data.x;
	mrb.y_raw = -data.y;
	mrb.z_raw = -data.z;

	float xraw_f =  data.x;
	float yraw_f = -data.y;
	float zraw_f = -data.z;

	/* apply user specified rotation */
	rotate_3f(_parent->_rotation, xraw_f, yraw_f, zraw_f);

	mrb.x = ((xraw_f * _mag_range_scale * _mag_asa_x) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mrb.y = ((yraw_f * _mag_range_scale * _mag_asa_y) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mrb.z = ((zraw_f * _mag_range_scale * _mag_asa_z) - _mag_scale.z_offset) * _mag_scale.z_scale;
	mrb.range_ga = (float)48.0;
	mrb.scaling = _mag_range_scale;
	mrb.temperature = _parent->_last_temperature;

	mrb.error_count = perf_event_count(_mag_errors);

	_mag_reports->force(&mrb);

	/* notify anyone waiting for data */
	if (mag_notify) {
		poll_notify(POLLIN);
	}

	if (mag_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _mag_topic, &mrb);
	}
}

ssize_t
MPU9250_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_parent->_call_interval == 0) {
		_mag_reports->flush();
		/* TODO: this won't work as getting valid magnetometer
		 *       data requires more than one measure cycle
		 */
		_parent->measure();
	}

	/* if no data, error (we could block here) */
	if (_mag_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_mag_reads);

	/* copy reports out of our buffer to the caller */
	mag_report *mrp = reinterpret_cast<mag_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_mag_reports->get(mrp)) {
			break;
		}

		transferred++;
		mrp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(mag_report));
}

int
MPU9250_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		/*
		 * TODO: we could implement a reset of the AK8963 registers
		 */
		//return reset();
		return _parent->ioctl(filp, cmd, arg);

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				/*
				 * TODO: investigate being able to stop
				 *       the continuous sampling
				 */
				//stop();
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 100);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, MPU9250_AK8963_SAMPLE_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					if (MPU9250_AK8963_SAMPLE_RATE != arg) {
						return -EINVAL;
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		return MPU9250_AK8963_SAMPLE_RATE;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

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

	case MAGIOCGSAMPLERATE:
		return MPU9250_AK8963_SAMPLE_RATE;

	case MAGIOCSSAMPLERATE:

		/*
		 * We don't currently support any means of changing
		 * the sampling rate of the mag
		 */
		if (MPU9250_AK8963_SAMPLE_RATE != arg) {
			return -EINVAL;
		}

		return OK;

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCSRANGE:
		return -EINVAL;

	case MAGIOCGRANGE:
		return 48; // fixed full scale measurement range of +/- 4800 uT == 48 Gauss

	case MAGIOCSELFTEST:
		return self_test();

#ifdef MAGIOCSHWLOWPASS

	case MAGIOCSHWLOWPASS:
		return -EINVAL;
#endif

#ifdef MAGIOCGHWLOWPASS

	case MAGIOCGHWLOWPASS:
		return -EINVAL;
#endif

	default:
		return (int)CDev::ioctl(filp, cmd, arg);
	}
}

int
MPU9250_mag::self_test(void)
{
	return 0;
}

void
MPU9250_mag::set_passthrough(uint8_t reg, uint8_t size, uint8_t *out)
{
	uint8_t addr;

	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // ensure slave r/w is disabled before changing the registers

	if (out) {
		_parent->write_reg(MPUREG_I2C_SLV0_D0, *out);
		addr = AK8963_I2C_ADDR;

	} else {
		addr = AK8963_I2C_ADDR | BIT_I2C_READ_FLAG;
	}

	_parent->write_reg(MPUREG_I2C_SLV0_ADDR, addr);
	_parent->write_reg(MPUREG_I2C_SLV0_REG,  reg);
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, size | BIT_I2C_SLV0_EN);
}

void
MPU9250_mag::read_block(uint8_t reg, uint8_t *val, uint8_t count)
{
	uint8_t addr = reg | 0x80;
	uint8_t tx[32] = { addr, };
	uint8_t rx[32];

	_parent->transfer(tx, rx, count + 1);
	memcpy(val, rx + 1, count);
}

void
MPU9250_mag::passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size)
{
	set_passthrough(reg, size);
	usleep(25 + 25 * size); // wait for the value to be read from slave
	read_block(MPUREG_EXT_SENS_DATA_00, buf, size);
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // disable new reads
}

bool
MPU9250_mag::ak8963_check_id(void)
{
	for (int i = 0; i < 5; i++) {
		uint8_t deviceid = 0;
		passthrough_read(AK8963REG_WIA, &deviceid, 0x01);

		if (AK8963_DEVICE_ID == deviceid) {
			return true;
		}
	}

	return false;
}

/*
 * 400kHz I2C bus speed = 2.5us per bit = 25us per byte
 */
void
MPU9250_mag::passthrough_write(uint8_t reg, uint8_t val)
{
	set_passthrough(reg, 1, &val);
	usleep(50); // wait for the value to be written to slave
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // disable new writes
}

void
MPU9250_mag::ak8963_reset(void)
{
	passthrough_write(AK8963REG_CNTL2, AK8963_RESET);
}

bool
MPU9250_mag::ak8963_read_adjustments(void)
{
	uint8_t response[3];
	float ak8963_ASA[3];

	passthrough_write(AK8963REG_CNTL1, AK8963_FUZE_MODE | AK8963_16BIT_ADC);
	usleep(50);
	passthrough_read(AK8963REG_ASAX, response, 3);
	passthrough_write(AK8963REG_CNTL1, AK8963_POWERDOWN_MODE);

	for (int i = 0; i < 3; i++) {
		if (0 != response[i] && 0xff != response[i]) {
			ak8963_ASA[i] = ((float)(response[i] - 128) / 256.0f) + 1.0f;

		} else {
			return false;
		}
	}

	_mag_asa_x = ak8963_ASA[0];
	_mag_asa_y = ak8963_ASA[1];
	_mag_asa_z = ak8963_ASA[2];

	return true;
}

bool
MPU9250_mag::ak8963_setup(void)
{
	int retries = 10;

	// enable the I2C master to slaves on the aux bus
	uint8_t user_ctrl = _parent->read_reg(MPUREG_USER_CTRL);
	_parent->write_checked_reg(MPUREG_USER_CTRL, user_ctrl | BIT_I2C_MST_EN);
	_parent->write_reg(MPUREG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR | BIT_I2C_MST_WAIT_FOR_ES | BITS_I2C_MST_CLOCK_400HZ);

	if (!ak8963_check_id()) {
		::printf("AK8963: bad id\n");
	}

	while (!ak8963_read_adjustments()) {
		if (!retries--) {
			::printf("AK8963: failed to read adjustment data\n");
			break;
		}
	}

	passthrough_write(AK8963REG_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);

	set_passthrough(AK8963REG_ST1, sizeof(struct ak8963_regs));

	return true;
}

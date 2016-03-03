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
 * Driver for the ak8963 magnetometer within the Invensense mpu9250.
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

////////////////////////////////////////////////////////////////////////////////

#define DIR_READ                    0x80
#define DIR_WRITE                   0x00

#define MPUREG_I2C_MST_CTRL         0x24
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27

#define MPUREG_EXT_SENS_DATA_00     0x49
#define MPUREG_I2C_SLV0_D0          0x63
#define MPUREG_I2C_MST_DELAY_CTRL   0x67
#define MPUREG_USER_CTRL            0x6A

#define BIT_I2C_MST_P_NSR           0x10
#define BIT_I2C_MST_EN              0x20
#define BITS_I2C_MST_CLOCK_400HZ    0x0D
//#define BITS_I2C_MST_DLY_1KHZ       0x09

#define BIT_I2C_SLV0_DLY_EN         0x01
#define BIT_I2C_SLV1_DLY_EN         0x02
#define BIT_I2C_SLV2_DLY_EN         0x04
#define BIT_I2C_SLV3_DLY_EN         0x08

////////////////////////////////////////////////////////////////////////////////

#define BIT_I2C_SLVO_EN   0x80
#define BIT_I2C_READ_FLAG 0x80

#define AK8963_I2C_ADDR   0x0C

#define AK8963_WIA 0x00
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_DEVICE_ID 0x48

#define AK8963_CNTL1 0x0A
#define AK8963_SINGLE_MEAS_MODE  0x01
#define AK8963_CONTINUOUS_MODE1  0x02
#define AK8963_CONTINUOUS_MODE2  0x06
#define AK8963_SELFTEST_MODE     0x08
#define AK8963_POWERDOWN_MODE    0x00
#define AK8963_FUZE_MODE         0x0F
#define AK8963_16BIT_ADC         0x10
#define AK8963_14BIT_ADC         0x00

#define AK8963_CNTL2 0x0B
#define AK8963_RESET             0x01

#define AK8963_ASAX              0x10
#define AK8963_HXL               0x03

////////////////////////////////////////////////////////////////////////////////

float ak8963_ASA[3] = { 0, 0, 0 }; // TODO, make member variable


MPU9250_mag::MPU9250_mag(MPU9250 *parent, const char *path) :
	CDev("MPU9250_mag", path),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1),
	_mag_reading_data(false),
	_mag_call_interval(0),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(0.0f),
	_mag_sample_rate(1000),
	_mag_reads(perf_alloc(PC_COUNT, "mpu9250_mag_read"))
{
	// default mag scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;

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
}

int
MPU9250_mag::init()
{
	int ret;

	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));
	if (_mag_reports == nullptr) {
		goto out;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	/* Initialize offsets and scales */
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;

	ak8963_setup();

//	measure(0);

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
	printf("MPU9250_mag::init() completed\n");
	return ret;
}

uint8_t cnt0 = 0;
uint8_t cnt1 = 0;
uint8_t cnt2 = 0;
uint8_t cnt3 = 0;
uint8_t cnt4 = 0;
/*
	struct ak8963_regs {
		uint8_t id;
		uint8_t info;
		uint8_t st1;
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t st2;
	};
 */

void
MPU9250_mag::measure(struct ak8963_regs data)
{
	bool mag_notify = true;

	if (data.st1 & 0x01) {
		if (false == _mag_reading_data) {
			cnt1++;
			_parent->write_reg(MPUREG_I2C_SLV0_CTRL, BIT_I2C_SLVO_EN | sizeof(struct ak8963_regs));
			_mag_reading_data = true;
			return;
		} else {
			cnt2++;
			_parent->write_reg(MPUREG_I2C_SLV0_CTRL, BIT_I2C_SLVO_EN | 1);
			_mag_reading_data = false;
		}
	} else {
		if (true == _mag_reading_data) {
			cnt4++;
			_parent->write_reg(MPUREG_I2C_SLV0_CTRL, BIT_I2C_SLVO_EN | 1);
			_mag_reading_data = false;
		} else {
			cnt3++;
		}
		return;
	}

	mag_report		mrb;
	mrb.timestamp = hrt_absolute_time();

	mrb.x_raw = data.x;
	mrb.y_raw = data.y;
	mrb.z_raw = data.z;

	float xraw_f = data.x;
	float yraw_f = data.y;
	float zraw_f = data.z;

	/* apply user specified rotation */
	rotate_3f(_parent->_rotation, xraw_f, yraw_f, zraw_f);
/*
struct sensor_mag_s {
		uint64_t timestamp;
		uint64_t error_count;
		float x;
		float y;
		float z;
		float range_ga;
		float scaling;
		float temperature;
		int16_t x_raw;
		int16_t y_raw;
		int16_t z_raw;
};
 */
	_mag_range_scale = 0.15e-3f;

	mrb.x = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mrb.y = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mrb.z = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;
	mrb.range_ga = (float)48.0;
	mrb.scaling = _mag_range_scale;
	mrb.temperature = _parent->_last_temperature;

		static uint64_t prev_mag_timestamp = 0;
		mrb.error_count = ((uint64_t)(mrb.timestamp - prev_mag_timestamp) << 40)
						+ ((uint64_t)cnt0 << 32)
						+ ((uint32_t)cnt1 << 24) + ((uint32_t)cnt2 << 16)
						+ ((uint32_t)cnt3 << 8) + cnt4;
		cnt0 = cnt1 = cnt2 = cnt3 = cnt4 = 0;
		prev_mag_timestamp = mrb.timestamp;

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
	printf("MPU9250_mag::read(..)\n");

	unsigned count = buflen / sizeof(mag_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_mag_call_interval == 0) {
		_mag_reports->flush();
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
//		return reset();
		return _parent->ioctl(filp, cmd, arg);

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
//				stop();
				_mag_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 100);

#define MPU9250_AK8963_DEFAULT_RATE 100
#define MPU9250_AK8963_TIMER_REDUCTION 0

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, MPU9250_AK8963_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
//					bool want_start = (_mag_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_mag_call_interval = ticks;

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_mag_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}
		return 1000000 / _mag_call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			printf("MPU9250_mag::ioctl(.. %l) SENSORIOCSQUEUEDEPTH\n", arg);
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
		printf("MPU9250_mag::ioctl() MAGIOCGSAMPLERATE\n");
		return _mag_sample_rate;

	case MAGIOCSSAMPLERATE:
		{
		/* convert hz to hrt interval via microseconds */
//		unsigned ticks = 1000000 / arg;

		/* check against maximum sane rate */
//		if (ticks < 1000) {
//			return -EINVAL;
//		}

		uint8_t div = 1000 / arg;
		if (div > 200) { div = 200; }
		if (div < 1) { div = 1; }

//		write_checked_reg(MPUREG_SMPLRT_DIV, div - 1);
		_mag_sample_rate = 1000 / div;

		printf("MPU9250_mag::ioctl MAGIOCSSAMPLERATE %u\n", _mag_sample_rate);
		}
		return OK;
/*
	case MAGIOCGLOWPASS:
		return _mag_filter_x.get_cutoff_freq();

	case MAGIOCSLOWPASS:
		// set software filtering
		_mag_filter_x.set_cutoff_frequency(1.0e6f / _mag_call_interval, arg);
		_mag_filter_y.set_cutoff_frequency(1.0e6f / _mag_call_interval, arg);
		_mag_filter_z.set_cutoff_frequency(1.0e6f / _mag_call_interval, arg);
		return OK;
 */
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
		printf("MPU9250_mag::ioctl() MAGIOCSELFTEST\n");
		return self_test();

#ifdef MAGIOCSHWLOWPASS
	case MAGIOCSHWLOWPASS:
		return -EINVAL;
#endif

#ifdef MAGIOCGHWLOWPASS
	case MAGIOCGHWLOWPASS:
		return -EINVAL;
#endif

//	case DEVIOCGDEVICEID:
//		return (int)CDev::ioctl(filp, cmd, arg);
//		break;

	default:
		return (int)CDev::ioctl(filp, cmd, arg);
	}
}

int
MPU9250_mag::self_test(void)
{
	return 1;
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
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, size | BIT_I2C_SLVO_EN);
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
		passthrough_read(AK8963_WIA, &deviceid, 0x01);
		if (deviceid == AK8963_DEVICE_ID) {
//			printf("ak8963_check_id: %02x\n", deviceid);
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
MPU9250_mag::ak8963_read(void)
{
/*	struct ak8963_regs data;

	memset(&data, 0, sizeof(struct ak8963_regs));
	passthrough_read(AK8963_WIA, (uint8_t*)&data, sizeof(struct ak8963_regs));
	if (data.id == 0x48) {
		printf("magxyz [%02x]:", data.st2);
		printf(" %d %d %d\n", data.x, data.y, data.z);
	} else {
		printf("invalid ak8963 read\n");
	}
 */
}

void
MPU9250_mag::ak8963_reset(void)
{
	passthrough_write(AK8963_CNTL2, AK8963_RESET);
}

bool
MPU9250_mag::ak8963_setup(void)
{
	// enable the I2C master to slaves on the aux bus
	uint8_t user_ctrl = _parent->read_reg(MPUREG_USER_CTRL);
	_parent->write_checked_reg(MPUREG_USER_CTRL, user_ctrl | BIT_I2C_MST_EN);
	_parent->write_reg(MPUREG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR | BITS_I2C_MST_CLOCK_400HZ);
//	_parent->write_reg(MPUREG_I2C_MST_DELAY_CTRL, BIT_I2C_SLV0_DLY_EN);

	if (!ak8963_check_id()) {
		printf("AK8963: bad id\n");
	}

	uint8_t response[3];
	passthrough_write(AK8963_CNTL1, AK8963_FUZE_MODE | AK8963_16BIT_ADC);
	passthrough_read(AK8963_ASAX, response, 3);
	for (int i = 0; i < 3; i++) {
		float data = response[i];
		ak8963_ASA[i] = ((data - 128) / 256 + 1);
		printf("AK8963_calibrate %d: %i, %f\n", i, response[i], (double)ak8963_ASA[i]);
	}

//	passthrough_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE1 | AK8963_16BIT_ADC);
	passthrough_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);
//	passthrough_write(AK8963_CNTL1, AK8963_SINGLE_MEAS_MODE | AK8963_16BIT_ADC);

	set_passthrough(AK8963_ST1, 1);
	return true;
}

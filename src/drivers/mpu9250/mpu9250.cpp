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
 * @file mpu9250.cpp
 *
 * Driver for the Invensense MPU9250 connected via I2C or SPI.
 *
 * @author Andrew Tridgell
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>

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
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

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
#include "gyro.h"
#include "mpu9250.h"

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers
 */
#define MPU9250_TIMER_REDUCTION				200


/*
  list of registers that will be checked in check_registers(). Note
  that MPUREG_PRODUCT_ID must be first in the list.
 */
const uint8_t MPU9250::_checked_registers[MPU9250_NUM_CHECKED_REGISTERS] = { MPUREG_WHOAMI,
									     MPUREG_PWR_MGMT_1,
									     MPUREG_PWR_MGMT_2,
									     MPUREG_USER_CTRL,
									     MPUREG_SMPLRT_DIV,
									     MPUREG_CONFIG,
									     MPUREG_GYRO_CONFIG,
									     MPUREG_ACCEL_CONFIG,
									     MPUREG_ACCEL_CONFIG2,
									     MPUREG_INT_ENABLE,
									     MPUREG_INT_PIN_CFG
									   };


MPU9250::MPU9250(device::Device *interface, device::Device *mag_interface, const char *path_accel,
		 const char *path_gyro, const char *path_mag,
		 enum Rotation rotation) :
	CDev("MPU9250", path_accel),
	_interface(interface),
	_gyro(new MPU9250_gyro(this, path_gyro)),
	_mag(new MPU9250_mag(this, mag_interface, path_mag)),
	_whoami(0),
#if defined(USE_I2C)
	_work {},
	_use_hrt(false),
#else
	_use_hrt(true),
#endif
	_call {},
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_dlpf_freq(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ),
	_sample_rate(1000),
	_accel_reads(perf_alloc(PC_COUNT, "mpu9250_acc_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "mpu9250_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpu9250_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "mpu9250_bad_trans")),
	_bad_registers(perf_alloc(PC_COUNT, "mpu9250_bad_reg")),
	_good_transfers(perf_alloc(PC_COUNT, "mpu9250_good_trans")),
	_reset_retries(perf_alloc(PC_COUNT, "mpu9250_reset")),
	_duplicates(perf_alloc(PC_COUNT, "mpu9250_dupe")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_register_wait(0),
	_reset_wait(0),
	_accel_filter_x(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / MPU9250_ACCEL_MAX_OUTPUT_RATE),
	_gyro_int(1000000 / MPU9250_GYRO_MAX_OUTPUT_RATE, true),
	_rotation(rotation),
	_checked_next(0),
	_last_temperature(0),
	_last_accel_data{},
	_got_duplicate(false)
{
	// disable debug() calls
	_debug_enabled = false;

	/* Set device parameters and make sure parameters of the bus device are adopted */
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_MPU9250;
	_device_id.devid_s.bus_type = (device::Device::DeviceBusType)_interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();

	/* Prime _gyro with parents devid. */
	/* Set device parameters and make sure parameters of the bus device are adopted */
	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_MPU9250;
	_gyro->_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_gyro->_device_id.devid_s.bus = _interface->get_device_bus();
	_gyro->_device_id.devid_s.address = _interface->get_device_address();

	/* Prime _mag with parents devid. */
	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_MPU9250;
	_mag->_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_mag->_device_id.devid_s.bus = _interface->get_device_bus();
	_mag->_device_id.devid_s.address = _interface->get_device_address();

	/* For an independent mag, ensure that it is connected to the i2c bus */
	_interface->set_device_type(_device_id.devid_s.devtype);

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
#if defined(USE_I2C)
	memset(&_work, 0, sizeof(_work));
#endif
}

MPU9250::~MPU9250()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	/* delete the magnetometer subdriver */
	delete _mag;

	/* free any existing reports */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

int
MPU9250::init()
{

#if defined(USE_I2C)
	unsigned dummy;
	use_i2c(_interface->ioctl(MPUIOCGIS_I2C, dummy));
#endif


	int ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("MPU9250 probe failed");
		return ret;
	}


	/* do init */

	ret = CDev::init();

	/* if init failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		return ret;
	}




	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == nullptr) {
		goto out;
	}

	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

	if (_gyro_reports == nullptr) {
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

#ifdef USE_I2C

	if (!_mag->is_passthrough() && _mag->_interface->init() != OK) {
		warnx("failed to setup ak8963 interface");
	}

#endif

	/* do CDev init for the mag device node, keep it optional */
	if (_whoami == MPU_WHOAMI_9250) {
		ret = _mag->init();
	}

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

int MPU9250::reset()
{
	write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	up_udelay(10000);

	write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_AUTO);
	up_udelay(1000);

	write_checked_reg(MPUREG_PWR_MGMT_2, 0);
	up_udelay(1000);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);
	usleep(1000);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	_set_dlpf_filter(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ);
	usleep(1000);

	// Gyro scale 2000 deg/s ()
	write_checked_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	usleep(1000);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	set_accel_range(16);

	usleep(1000);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	usleep(1000);

#ifdef USE_I2C
	bool bypass = !_mag->is_passthrough();
#else
	bool bypass = false;
#endif
	write_checked_reg(MPUREG_INT_PIN_CFG,
			  BIT_INT_ANYRD_2CLEAR | (bypass ? BIT_INT_BYPASS_EN :
					  0)); // INT: Clear on any read, also use i2c bypass is master mode isn't needed
	usleep(1000);

	write_checked_reg(MPUREG_ACCEL_CONFIG2, BITS_ACCEL_CONFIG2_41HZ);
	usleep(1000);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	return OK;
}

int
MPU9250::probe()
{
	/* look for device ID */
	_whoami = read_reg(MPUREG_WHOAMI);

	// verify product revision
	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
MPU9250::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	if (desired_sample_rate_hz == 0 ||
	    desired_sample_rate_hz == GYRO_SAMPLERATE_DEFAULT ||
	    desired_sample_rate_hz == ACCEL_SAMPLERATE_DEFAULT) {
		desired_sample_rate_hz = MPU9250_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }

	write_checked_reg(MPUREG_SMPLRT_DIV, div - 1);
	_sample_rate = 1000 / div;
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
MPU9250::_set_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter;

	/*
	   choose next highest filter frequency available
	 */
	if (frequency_hz == 0) {
		_dlpf_freq = 0;
		filter = BITS_DLPF_CFG_3600HZ;

	} else if (frequency_hz <= 5) {
		_dlpf_freq = 5;
		filter = BITS_DLPF_CFG_5HZ;

	} else if (frequency_hz <= 10) {
		_dlpf_freq = 10;
		filter = BITS_DLPF_CFG_10HZ;

	} else if (frequency_hz <= 20) {
		_dlpf_freq = 20;
		filter = BITS_DLPF_CFG_20HZ;

	} else if (frequency_hz <= 41) {
		_dlpf_freq = 41;
		filter = BITS_DLPF_CFG_41HZ;

	} else if (frequency_hz <= 92) {
		_dlpf_freq = 92;
		filter = BITS_DLPF_CFG_92HZ;

	} else if (frequency_hz <= 184) {
		_dlpf_freq = 184;
		filter = BITS_DLPF_CFG_184HZ;

	} else if (frequency_hz <= 250) {
		_dlpf_freq = 250;
		filter = BITS_DLPF_CFG_250HZ;

	} else {
		_dlpf_freq = 0;
		filter = BITS_DLPF_CFG_3600HZ;
	}

	write_checked_reg(MPUREG_CONFIG, filter);
}

ssize_t
MPU9250::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
MPU9250::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
MPU9250::accel_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}

int
MPU9250::gyro_self_test()
{
	if (self_test()) {
		return 1;
	}

	/*
	 * Maximum deviation of 20 degrees, according to
	 * http://www.invensense.com/mems/gyro/documents/PS-MPU-9250A-00v3.4.pdf
	 * Section 6.1, initial ZRO tolerance
	 */
	const float max_offset = 0.34f;
	/* 30% scale error is chosen to catch completely faulty units but
	 * to let some slight scale error pass. Requires a rate table or correlation
	 * with mag rotations + data fit to
	 * calibrate properly and is not done by default.
	 */
	const float max_scale = 0.3f;

	/* evaluate gyro offsets, complain if offset -> zero or larger than 20 dps. */
	if (fabsf(_gyro_scale.x_offset) > max_offset) {
		return 1;
	}

	/* evaluate gyro scale, complain if off by more than 30% */
	if (fabsf(_gyro_scale.x_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_scale - 1.0f) > max_scale) {
		return 1;
	}

	return 0;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
MPU9250::test_error()
{
	// deliberately trigger an error. This was noticed during
	// development as a handy way to test the reset logic
	uint8_t data[16];
	memset(data, 0, sizeof(data));
	_interface->read(MPU9250_SET_SPEED(MPUREG_INT_STATUS, MPU9250_LOW_BUS_SPEED), data, sizeof(data));
	::printf("error triggered\n");
	print_registers();
}

ssize_t
MPU9250::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}

int
MPU9250::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

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
				return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, MPU9250_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					_set_dlpf_filter(cutoff_freq_hz);
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_set_dlpf_filter(cutoff_freq_hz_gyro);
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = ticks;

					/*
					  set call interval faster than the sample time. We
					  then detect when we have duplicate samples and reject
					  them. This prevents aliasing due to a beat between the
					  stm32 clock and the mpu9250 clock
					 */
					_call.period = _call_interval - MPU9250_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_accel_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _accel_reports->size();

	case ACCELIOCGSAMPLERATE:
		return _sample_rate;

	case ACCELIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case ACCELIOCGLOWPASS:
		return _accel_filter_x.get_cutoff_freq();

	case ACCELIOCSLOWPASS:
		// set software filtering
		_accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		return OK;

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		return set_accel_range(arg);

	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2) / MPU9250_ONE_G + 0.5f);

	case ACCELIOCSELFTEST:
		return accel_self_test();

#ifdef ACCELIOCSHWLOWPASS

	case ACCELIOCSHWLOWPASS:
		_set_dlpf_filter(arg);
		return OK;
#endif

#ifdef ACCELIOCGHWLOWPASS

	case ACCELIOCGHWLOWPASS:
		return _dlpf_freq;
#endif

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

int
MPU9250::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_gyro_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _gyro_reports->size();

	case GYROIOCGSAMPLERATE:
		return _sample_rate;

	case GYROIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case GYROIOCGLOWPASS:
		return _gyro_filter_x.get_cutoff_freq();

	case GYROIOCSLOWPASS:
		// set software filtering
		_gyro_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_gyro_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_gyro_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		/* XXX not implemented */
		// XXX change these two values on set:
		// _gyro_range_scale = xx
		// _gyro_range_rad_s = xx
		return -EINVAL;

	case GYROIOCGRANGE:
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return gyro_self_test();

#ifdef GYROIOCSHWLOWPASS

	case GYROIOCSHWLOWPASS:
		_set_dlpf_filter(arg);
		return OK;
#endif

#ifdef GYROIOCGHWLOWPASS

	case GYROIOCGHWLOWPASS:
		return _dlpf_freq;
#endif

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

uint8_t
MPU9250::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t buf;
	_interface->read(MPU9250_SET_SPEED(reg, speed), &buf, 1);
	return buf;
}

uint16_t
MPU9250::read_reg16(unsigned reg)
{
	uint8_t buf[2];

	// general register transfer at low clock speed

	_interface->read(MPU9250_LOW_SPEED_OP(reg), &buf, arraySize(buf));
	return (uint16_t)(buf[0] << 8) | buf[1];
}

void
MPU9250::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed

	_interface->write(MPU9250_LOW_SPEED_OP(reg), &value, 1);
}

void
MPU9250::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MPU9250::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
			break;
		}
	}
}

int
MPU9250::set_accel_range(unsigned max_g_in)
{
	uint8_t afs_sel;
	float lsb_per_g;
	float max_accel_g;

	if (max_g_in > 8) { // 16g - AFS_SEL = 3
		afs_sel = 3;
		lsb_per_g = 2048;
		max_accel_g = 16;

	} else if (max_g_in > 4) { //  8g - AFS_SEL = 2
		afs_sel = 2;
		lsb_per_g = 4096;
		max_accel_g = 8;

	} else if (max_g_in > 2) { //  4g - AFS_SEL = 1
		afs_sel = 1;
		lsb_per_g = 8192;
		max_accel_g = 4;

	} else {                //  2g - AFS_SEL = 0
		afs_sel = 0;
		lsb_per_g = 16384;
		max_accel_g = 2;
	}

	write_checked_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);
	_accel_range_scale = (MPU9250_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * MPU9250_ONE_G;

	return OK;
}

void
MPU9250::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();
	_mag->_mag_reports->flush();

	if (_use_hrt) {
		/* start polling at the specified rate */
		hrt_call_every(&_call,
			       1000,
			       _call_interval - MPU9250_TIMER_REDUCTION,
			       (hrt_callout)&MPU9250::measure_trampoline, this);;

	} else {
#ifdef USE_I2C
		/* schedule a cycle to start things */
		work_queue(HPWORK, &_work, (worker_t)&MPU9250::cycle_trampoline, this, 1);
#endif
	}

}

void
MPU9250::stop()
{
	if (_use_hrt) {
		hrt_cancel(&_call);

	} else {
#ifdef USE_I2C
		work_cancel(HPWORK, &_work);
#endif
	}
}


#if defined(USE_I2C)
void
MPU9250::cycle_trampoline(void *arg)
{
	MPU9250 *dev = (MPU9250 *)arg;

	dev->cycle();
}

void
MPU9250::cycle()
{

//	int ret = measure();

	measure();

//	if (ret != OK) {
//		/* issue a reset command to the sensor */
//		reset();
//		start();
//		return;
//	}

	if (_call_interval != 0) {
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&MPU9250::cycle_trampoline,
			   this,
			   USEC2TICK(_call_interval - MPU9250_TIMER_REDUCTION));
	}
}
#endif


void
MPU9250::measure_trampoline(void *arg)
{
	MPU9250 *dev = reinterpret_cast<MPU9250 *>(arg);

	/* make another measurement */
	dev->measure();
}

void
MPU9250::check_registers(void)
{
	/*
	  we read the register at full speed, even though it isn't
	  listed as a high speed register. The low speed requirement
	  for some registers seems to be a propagation delay
	  requirement for changing sensor configuration, which should
	  not apply to reading a single register. It is also a better
	  test of SPI bus health to read at the same speed as we read
	  the data registers.
	*/
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next], MPU9250_HIGH_BUS_SPEED)) !=
	    _checked_values[_checked_next]) {
		_checked_bad[_checked_next] = v;

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
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
			write_reg(MPUREG_PWR_MGMT_2, MPU_CLK_SEL_AUTO);
			// after doing a reset we need to wait a long
			// time before we do any other register writes
			// or we will end up with the mpu9250 in a
			// bizarre state where it has all correct
			// register values but large offsets on the
			// accel axes
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % MPU9250_NUM_CHECKED_REGISTERS;
}

bool MPU9250::check_null_data(uint32_t *data, uint8_t size)
{
	while (size--) {
		if (*data++) {
			perf_count(_good_transfers);
			return false;
		}
	}

	// all zero data - probably a SPI bus error
	perf_count(_bad_transfers);
	perf_end(_sample_perf);
	// note that we don't call reset() here as a reset()
	// costs 20ms with interrupts disabled. That means if
	// the mpu6k does go bad it would cause a FMU failure,
	// regardless of whether another sensor is available,
	return true;
}

bool MPU9250::check_duplicate(uint8_t *accel_data)
{
	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
	if (!_got_duplicate && memcmp(accel_data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
		// it isn't new data - wait for next timer
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;

	} else {
		memcpy(&_last_accel_data, accel_data, sizeof(_last_accel_data));
		_got_duplicate = false;
	}

	return _got_duplicate;
}

void
MPU9250::measure()
{
	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct MPUReport mpu_report;

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the MPU9250 in one pass.
	 */
	if (OK != _interface->read(MPU9250_SET_SPEED(MPUREG_INT_STATUS, MPU9250_HIGH_BUS_SPEED),
				   (uint8_t *)&mpu_report,
				   sizeof(mpu_report))) {
		return;
	}

	check_registers();

	if (check_duplicate(&mpu_report.accel_x[0])) {
		return;
	}

#ifdef USE_I2C

	if (_mag->is_passthrough()) {
#endif
		_mag->_measure(mpu_report.mag);
#ifdef USE_I2C

	} else {
		_mag->measure();
	}

#endif

	/*
	 * Convert from big to little endian
	 */
	report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
	report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
	report.accel_z = int16_t_from_bytes(mpu_report.accel_z);
	report.temp    = int16_t_from_bytes(mpu_report.temp);
	report.gyro_x  = int16_t_from_bytes(mpu_report.gyro_x);
	report.gyro_y  = int16_t_from_bytes(mpu_report.gyro_y);
	report.gyro_z  = int16_t_from_bytes(mpu_report.gyro_z);

	if (check_null_data((uint32_t *)&report, sizeof(report) / 4)) {
		return;
	}

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using the sensor again
		// We still increment _good_transfers, but don't return any data yet
		_register_wait--;
		return;
	}

	/*
	 * Swap axes and negate y
	 */
	int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	int16_t gyro_xt = report.gyro_y;
	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

	/*
	 * Apply the swap
	 */
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;

	/*
	 * Report buffers.
	 */
	accel_report		arb;
	gyro_report		grb;

	/*
	 * Adjust and scale results to m/s^2.
	 */
	grb.timestamp = arb.timestamp = hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	grb.error_count = arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

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

	/* NOTE: Axes have been swapped to match the board a few lines above. */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
	math::Vector<3> aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	_last_temperature = (report.temp) / 361.0f + 35.0f;

	arb.temperature_raw = report.temp;
	arb.temperature = _last_temperature;

	/* return device ID */
	arb.device_id = _device_id.devid;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	xraw_f = report.gyro_x;
	yraw_f = report.gyro_y;
	zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	math::Vector<3> gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	math::Vector<3> gval_integrated;

	bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = _last_temperature;

	/* return device ID */
	grb.device_id = _gyro->_device_id.devid;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}

	if (accel_notify && !(_pub_blocked)) {
		/* log the time of this report */
		perf_begin(_controller_latency_perf);
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
MPU9250::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	_mag->_mag_reports->print_info("mag queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i], MPU9250_HIGH_BUS_SPEED);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	::printf("temperature: %.1f\n", (double)_last_temperature);
}

void
MPU9250::print_registers()
{
	printf("MPU9250 registers\n");

	for (uint8_t reg = 0; reg <= 126; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if (reg % 13 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}

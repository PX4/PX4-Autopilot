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
 * Driver for the Invensense ICM20948 connected via I2C or SPI.
 *
 *
 * based on the mpu9250 driver
 */

#include <px4_config.h>
#include <px4_time.h>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mag.h"
#include "accel.h"
#include "gyro.h"
#include "icm20948.h"

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers
 */
#define MPU9250_TIMER_REDUCTION				200

/* Set accel range used */
#define ACCEL_RANGE_G  16
/*
  list of registers that will be checked in check_registers(). Note
  that MPUREG_PRODUCT_ID must be first in the list.
 */
const uint16_t ICM20948::_icm20948_checked_registers[ICM20948_NUM_CHECKED_REGISTERS] = {
	ICMREG_20948_USER_CTRL,
	ICMREG_20948_PWR_MGMT_1,
	ICMREG_20948_PWR_MGMT_2,
	ICMREG_20948_INT_PIN_CFG,
	ICMREG_20948_INT_ENABLE,
	ICMREG_20948_INT_ENABLE_1,
	ICMREG_20948_INT_ENABLE_2,
	ICMREG_20948_INT_ENABLE_3,
	ICMREG_20948_GYRO_SMPLRT_DIV,
	ICMREG_20948_GYRO_CONFIG_1,
	ICMREG_20948_GYRO_CONFIG_2,
	ICMREG_20948_ACCEL_SMPLRT_DIV_1,
	ICMREG_20948_ACCEL_SMPLRT_DIV_2,
	ICMREG_20948_ACCEL_CONFIG,
	ICMREG_20948_ACCEL_CONFIG_2
};


ICM20948::ICM20948(device::Device *interface, device::Device *mag_interface, const char *path_accel,
		   const char *path_gyro, const char *path_mag,
		   enum Rotation rotation,
		   bool magnetometer_only) :
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_whoami(0),
	_accel(magnetometer_only ? nullptr : new ICM20948_accel(this, path_accel)),
	_gyro(magnetometer_only ? nullptr : new ICM20948_gyro(this, path_gyro)),
	_mag(new ICM20948_mag(this, mag_interface, path_mag)),
	_selected_bank(0xFF),	// invalid/improbable bank value, will be set on first read/write
	_magnetometer_only(magnetometer_only),
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_dlpf_freq(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ),
	_dlpf_freq_icm_gyro(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ),
	_dlpf_freq_icm_accel(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ),
	_sample_rate(1000),
	_accel_reads(perf_alloc(PC_COUNT, "mpu9250_acc_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "mpu9250_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpu9250_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "mpu9250_bad_trans")),
	_bad_registers(perf_alloc(PC_COUNT, "mpu9250_bad_reg")),
	_good_transfers(perf_alloc(PC_COUNT, "mpu9250_good_trans")),
	_reset_retries(perf_alloc(PC_COUNT, "mpu9250_reset")),
	_duplicates(perf_alloc(PC_COUNT, "mpu9250_dupe")),
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
	_checked_registers(nullptr),
	_checked_next(0),
	_num_checked_registers(0),
	_last_temperature(0),
	_last_accel_data{},
	_got_duplicate(false)
{
	if (_accel != nullptr) {
		/* Set device parameters and make sure parameters of the bus device are adopted */
		_accel->_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_MPU9250;
		_accel->_device_id.devid_s.bus_type = (device::Device::DeviceBusType)_interface->get_device_bus_type();
		_accel->_device_id.devid_s.bus = _interface->get_device_bus();
		_accel->_device_id.devid_s.address = _interface->get_device_address();
	}

	if (_gyro != nullptr) {
		/* Prime _gyro with common devid. */
		/* Set device parameters and make sure parameters of the bus device are adopted */
		_gyro->_device_id.devid = 0;
		_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_MPU9250;
		_gyro->_device_id.devid_s.bus_type = _interface->get_device_bus_type();
		_gyro->_device_id.devid_s.bus = _interface->get_device_bus();
		_gyro->_device_id.devid_s.address = _interface->get_device_address();
	}

	/* Prime _mag with common devid. */
	_mag->_device_id.devid = 0;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_MPU9250;
	_mag->_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_mag->_device_id.devid_s.bus = _interface->get_device_bus();
	_mag->_device_id.devid_s.address = _interface->get_device_address();

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
}

ICM20948::~ICM20948()
{
	/* make sure we are truly inactive */
	stop();
	_call_interval = 0;

	if (!_magnetometer_only) {
		orb_unadvertise(_accel_topic);
		orb_unadvertise(_gyro->_gyro_topic);
	}

	/* delete the accel subdriver */
	delete _accel;

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
ICM20948::init()
{
	irqstate_t state;

	/*
	 * If the MPU is using I2C we should reduce the sample rate to 200Hz and
	 * make the integration autoreset faster so that we integrate just one
	 * sample since the sampling rate is already low.
	*/
	const bool is_i2c = (_interface->get_device_bus_type() == device::Device::DeviceBusType_I2C);

	if (is_i2c && !_magnetometer_only) {
		_sample_rate = 200;
		_accel_int.set_autoreset_interval(1000000 / 1000);
		_gyro_int.set_autoreset_interval(1000000 / 1000);
	}

	int ret = probe();

	if (ret != OK) {
		PX4_DEBUG("probe failed");
		return ret;
	}

	state = px4_enter_critical_section();
	_reset_wait = hrt_absolute_time() + 100000;
	px4_leave_critical_section(state);

	if (reset_mpu() != OK) {
		PX4_ERR("Exiting! Device failed to take initialization");
		return ret;
	}

	if (!_magnetometer_only) {
		/* allocate basic report buffers */
		_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));
		ret = -ENOMEM;

		if (_accel_reports == nullptr) {
			return ret;
		}

		_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));

		if (_gyro_reports == nullptr) {
			return ret;
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

		// set software low pass filter for controllers
		param_t accel_cut_ph = param_find("IMU_ACCEL_CUTOFF");
		float accel_cut = MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ;

		if (accel_cut_ph != PARAM_INVALID && (param_get(accel_cut_ph, &accel_cut) == PX4_OK)) {
			PX4_INFO("accel cutoff set to %.2f Hz", double(accel_cut));

			_accel_filter_x.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, accel_cut);
			_accel_filter_y.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, accel_cut);
			_accel_filter_z.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, accel_cut);

		}

		param_t gyro_cut_ph = param_find("IMU_GYRO_CUTOFF");
		float gyro_cut = MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ;

		if (gyro_cut_ph != PARAM_INVALID && (param_get(gyro_cut_ph, &gyro_cut) == PX4_OK)) {
			PX4_INFO("gyro cutoff set to %.2f Hz", double(gyro_cut));

			_gyro_filter_x.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, gyro_cut);
			_gyro_filter_y.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, gyro_cut);
			_gyro_filter_z.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, gyro_cut);

		}

		/* do CDev init for the accel device node */
		ret = _accel->init();

		/* if probe/setup failed, bail now */
		if (ret != OK) {
			PX4_DEBUG("accel init failed");
			return ret;
		}

		/* do CDev init for the gyro device node */
		ret = _gyro->init();

		/* if probe/setup failed, bail now */
		if (ret != OK) {
			PX4_DEBUG("gyro init failed");
			return ret;
		}
	}

	/* Magnetometer setup */

#ifdef USE_I2C
	up_udelay(100);

	if (!_mag->is_passthrough() && _mag->_interface->init() != PX4_OK) {
		PX4_ERR("failed to setup ak8963 interface");
	}

#endif /* USE_I2C */

	/* do CDev init for the mag device node */
	ret = _mag->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		PX4_DEBUG("mag init failed");
		return ret;
	}

	ret = _mag->ak8963_reset();

	if (ret != OK) {
		PX4_DEBUG("mag reset failed");
		return ret;
	}

	measure();

	if (!_magnetometer_only) {
		/* advertise sensor topic, measure manually to initialize valid report */
		sensor_accel_s arp;
		_accel_reports->get(&arp);

		/* measurement will have generated a report, publish */
		_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
						   &_accel->_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

		if (_accel_topic == nullptr) {
			PX4_ERR("ADVERT FAIL");
			return ret;
		}

		/* advertise sensor topic, measure manually to initialize valid report */
		sensor_gyro_s grp;
		_gyro_reports->get(&grp);

		_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
				     &_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

		if (_gyro->_gyro_topic == nullptr) {
			PX4_ERR("ADVERT FAIL");
			return ret;
		}
	}

	return ret;
}

uint8_t ICM20948::get_whoami()
{
	return _whoami;
}

int ICM20948::reset()
{
	irqstate_t state;

	/* When the mpu9250 starts from 0V the internal power on circuit
	 * per the data sheet will require:
	 *
	 * Start-up time for register read/write From power-up Typ:11 max:100 ms
	 *
	 */

	px4_usleep(110000);

	// Hold off sampling until done (100 MS will be shortened)
	state = px4_enter_critical_section();
	_reset_wait = hrt_absolute_time() + 100000;
	px4_leave_critical_section(state);

	int ret;

	ret = reset_mpu();

	if (ret == OK && (_whoami == ICM_WHOAMI_20948)) {
		ret = _mag->ak8963_reset();
	}

	state = px4_enter_critical_section();
	_reset_wait = hrt_absolute_time() + 10;
	px4_leave_critical_section(state);

	return ret;
}

int ICM20948::reset_mpu()
{
	uint8_t retries;

	switch (_whoami) {
	case ICM_WHOAMI_20948:
		write_reg(ICMREG_20948_PWR_MGMT_1, BIT_H_RESET);
		usleep(1000);

		write_checked_reg(ICMREG_20948_PWR_MGMT_1, MPU_CLK_SEL_AUTO);
		usleep(200);
		write_checked_reg(ICMREG_20948_PWR_MGMT_2, 0);
		break;
	}

	// Enable I2C bus or Disable I2C bus (recommended on data sheet)
	const bool is_i2c = (_interface->get_device_bus_type() == device::Device::DeviceBusType_I2C);
	write_checked_reg(ICMREG_20948_USER_CTRL, is_i2c ? 0 : BIT_I2C_IF_DIS);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);

	_set_dlpf_filter(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ);

	// Gyro scale 2000 deg/s ()
	switch (_whoami) {
	case ICM_WHOAMI_20948:
		modify_checked_reg(ICMREG_20948_GYRO_CONFIG_1, ICM_BITS_GYRO_FS_SEL_MASK, ICM_BITS_GYRO_FS_SEL_2000DPS);
		break;
	}


	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	set_accel_range(ACCEL_RANGE_G);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPU_OR_ICM(MPUREG_INT_ENABLE, ICMREG_20948_INT_ENABLE_1),
			  BIT_RAW_RDY_EN);        // INT: Raw data ready

#ifdef USE_I2C
	bool bypass = !_mag->is_passthrough();
#else
	bool bypass = false;
#endif

	/* INT: Clear on any read.
	 * If this instance is for a device is on I2C bus the Mag will have an i2c interface
	 * that it will use to access the either: a) the internal mag device on the internal I2C bus
	 * or b) it could be used to access a downstream I2C devices connected to the chip on
	 * it's AUX_{ASD|SCL} pins. In either case we need to disconnect (bypass) the internal master
	 * controller that chip provides as a SPI to I2C bridge.
	 * so bypass is true if the mag has an i2c non null interfaces.
	 */

	write_checked_reg(MPU_OR_ICM(MPUREG_INT_PIN_CFG, ICMREG_20948_INT_PIN_CFG),
			  BIT_INT_ANYRD_2CLEAR | (bypass ? BIT_INT_BYPASS_EN : 0));

	write_checked_reg(MPU_OR_ICM(MPUREG_ACCEL_CONFIG2, ICMREG_20948_ACCEL_CONFIG_2),
			  MPU_OR_ICM(BITS_ACCEL_CONFIG2_41HZ, ICM_BITS_DEC3_CFG_32));

	retries = 3;
	bool all_ok = false;

	while (!all_ok && retries--) {

		// Assume all checked values are as expected
		all_ok = true;
		uint8_t reg;
		uint8_t bankcheck = 0;

		for (uint8_t i = 0; i < _num_checked_registers; i++) {
			if ((reg = read_reg(_checked_registers[i])) != _checked_values[i]) {
				if (_whoami == ICM_WHOAMI_20948) {
					_interface->read(MPU9250_LOW_SPEED_OP(ICMREG_20948_BANK_SEL), &bankcheck, 1);
				}

				write_reg(_checked_registers[i], _checked_values[i]);
				PX4_ERR("Reg %d is:%d s/b:%d Tries:%d - bank s/b %d, is %d", _checked_registers[i], reg, _checked_values[i], retries,
					REG_BANK(_checked_registers[i]), bankcheck);
				all_ok = false;
			}
		}
	}

	return all_ok ? OK : -EIO;
}

int
ICM20948::probe()
{
	int ret = PX4_ERROR;

	// Try first for mpu9250/6500
	_whoami = read_reg(MPUREG_WHOAMI);

	// must be an ICM
	// Make sure selected register bank is bank 0 (which contains WHOAMI)
	select_register_bank(REG_BANK(ICMREG_20948_WHOAMI));
	_whoami = read_reg(ICMREG_20948_WHOAMI);

	if (_whoami == ICM_WHOAMI_20948) {
		_num_checked_registers = ICM20948_NUM_CHECKED_REGISTERS;
		_checked_registers = _icm20948_checked_registers;
		memset(_checked_values, 0, ICM20948_NUM_CHECKED_REGISTERS);
		memset(_checked_bad, 0, ICM20948_NUM_CHECKED_REGISTERS);
		ret = PX4_OK;
	}

	_checked_values[0] = _whoami;
	_checked_bad[0] = _whoami;

	if (ret != PX4_OK) {
		PX4_DEBUG("unexpected whoami 0x%02x", _whoami);
	}

	return ret;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
  For ICM20948 accel and gyro samplerates are both set to the same value.
*/
void
ICM20948::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	uint8_t div = 1;

	if (desired_sample_rate_hz == 0) {
		desired_sample_rate_hz = MPU9250_GYRO_DEFAULT_RATE;
	}

	switch (_whoami) {
	case ICM_WHOAMI_20948:
		div = 1100 / desired_sample_rate_hz;
		break;
	}

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }


	switch (_whoami) {
	case ICM_WHOAMI_20948:
		write_checked_reg(ICMREG_20948_GYRO_SMPLRT_DIV, div - 1);
		// There's also an MSB for this allowing much higher dividers for the accelerometer.
		// For 1 < div < 200 the LSB is sufficient.
		write_checked_reg(ICMREG_20948_ACCEL_SMPLRT_DIV_2, div - 1);
		_sample_rate = 1100 / div;
		break;
	}
}

/*
 * Set poll rate
 */
int
ICM20948::_set_pollrate(unsigned long rate)
{
	if (rate == 0) {
		return -EINVAL;

	} else {
		/* do we need to start internal polling? */
		bool want_start = (_call_interval == 0);

		/* convert hz to hrt interval via microseconds */
		unsigned interval = 1000000 / rate;

		/* check against maximum sane rate */
		if (interval < 1000) {
			return -EINVAL;
		}

		// adjust filters
		float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
		float sample_rate = 1.0e6f / interval;
		_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
		_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
		_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


		float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
		_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
		_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
		_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

		/* update interval for next measurement */
		/* XXX this is a bit shady, but no other way to adjust... */
		_call_interval = interval;

		/* if we need to start the poll state machine, do it */
		if (want_start) {
			start();
		}

		return OK;
	}
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
ICM20948::_set_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter;

	switch (_whoami) {
	case ICM_WHOAMI_20948:

		/*
		   choose next highest filter frequency available for gyroscope
		 */
		if (frequency_hz == 0) {
			_dlpf_freq_icm_gyro = 0;
			filter = ICM_BITS_GYRO_DLPF_CFG_361HZ;

		} else if (frequency_hz <= 5) {
			_dlpf_freq_icm_gyro = 5;
			filter = ICM_BITS_GYRO_DLPF_CFG_5HZ;

		} else if (frequency_hz <= 11) {
			_dlpf_freq_icm_gyro = 11;
			filter = ICM_BITS_GYRO_DLPF_CFG_11HZ;

		} else if (frequency_hz <= 23) {
			_dlpf_freq_icm_gyro = 23;
			filter = ICM_BITS_GYRO_DLPF_CFG_23HZ;

		} else if (frequency_hz <= 51) {
			_dlpf_freq_icm_gyro = 51;
			filter = ICM_BITS_GYRO_DLPF_CFG_51HZ;

		} else if (frequency_hz <= 119) {
			_dlpf_freq_icm_gyro = 119;
			filter = ICM_BITS_GYRO_DLPF_CFG_119HZ;

		} else if (frequency_hz <= 151) {
			_dlpf_freq_icm_gyro = 151;
			filter = ICM_BITS_GYRO_DLPF_CFG_151HZ;

		} else if (frequency_hz <= 197) {
			_dlpf_freq_icm_gyro = 197;
			filter = ICM_BITS_GYRO_DLPF_CFG_197HZ;

		} else {
			_dlpf_freq_icm_gyro = 0;
			filter = ICM_BITS_GYRO_DLPF_CFG_361HZ;
		}

		write_checked_reg(ICMREG_20948_GYRO_CONFIG_1, filter);

		/*
		   choose next highest filter frequency available for accelerometer
		 */
		if (frequency_hz == 0) {
			_dlpf_freq_icm_accel = 0;
			filter = ICM_BITS_ACCEL_DLPF_CFG_473HZ;

		} else if (frequency_hz <= 5) {
			_dlpf_freq_icm_accel = 5;
			filter = ICM_BITS_ACCEL_DLPF_CFG_5HZ;

		} else if (frequency_hz <= 11) {
			_dlpf_freq_icm_accel = 11;
			filter = ICM_BITS_ACCEL_DLPF_CFG_11HZ;

		} else if (frequency_hz <= 23) {
			_dlpf_freq_icm_accel = 23;
			filter = ICM_BITS_ACCEL_DLPF_CFG_23HZ;

		} else if (frequency_hz <= 50) {
			_dlpf_freq_icm_accel = 50;
			filter = ICM_BITS_ACCEL_DLPF_CFG_50HZ;

		} else if (frequency_hz <= 111) {
			_dlpf_freq_icm_accel = 111;
			filter = ICM_BITS_ACCEL_DLPF_CFG_111HZ;

		} else if (frequency_hz <= 246) {
			_dlpf_freq_icm_accel = 246;
			filter = ICM_BITS_ACCEL_DLPF_CFG_246HZ;

		} else {
			_dlpf_freq_icm_accel = 0;
			filter = ICM_BITS_ACCEL_DLPF_CFG_473HZ;
		}

		write_checked_reg(ICMREG_20948_ACCEL_CONFIG, filter);
		break;
	}
}

int
ICM20948::select_register_bank(uint8_t bank)
{
	uint8_t ret;
	uint8_t buf;
	uint8_t retries = 3;

	if (_selected_bank != bank) {
		ret = _interface->write(MPU9250_LOW_SPEED_OP(ICMREG_20948_BANK_SEL), &bank, 1);

		if (ret != OK) {
			return ret;
		}
	}

	/*
	 * Making sure the right register bank is selected (even if it should be). Observed some
	 * unexpected changes to this, don't risk writing to the wrong register bank.
	 */
	_interface->read(MPU9250_LOW_SPEED_OP(ICMREG_20948_BANK_SEL), &buf, 1);

	while (bank != buf && retries > 0) {
		//PX4_WARN("user bank: expected %d got %d",bank,buf);
		ret = _interface->write(MPU9250_LOW_SPEED_OP(ICMREG_20948_BANK_SEL), &bank, 1);

		if (ret != OK) {
			return ret;
		}

		retries--;
		//PX4_WARN("BANK retries: %d", 4-retries);

		_interface->read(MPU9250_LOW_SPEED_OP(ICMREG_20948_BANK_SEL), &buf, 1);
	}


	_selected_bank = bank;

	if (bank != buf) {
		PX4_DEBUG("SELECT FAILED %d %d %d %d", retries, _selected_bank, bank, buf);
		return PX4_ERROR;

	} else {
		return PX4_OK;
	}
}

uint8_t
ICM20948::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t buf;

	if (_whoami == ICM_WHOAMI_20948) {
		select_register_bank(REG_BANK(reg));
		_interface->read(MPU9250_SET_SPEED(REG_ADDRESS(reg), speed), &buf, 1);

	} else {
		_interface->read(MPU9250_SET_SPEED(REG_ADDRESS(reg), speed), &buf, 1);
	}

	return buf;
}

uint8_t
ICM20948::read_reg_range(unsigned start_reg, uint32_t speed, uint8_t *buf, uint16_t count)
{
	uint8_t ret;

	if (buf == NULL) { return PX4_ERROR; }

	if (_whoami == ICM_WHOAMI_20948) {
		select_register_bank(REG_BANK(start_reg));
		ret = _interface->read(MPU9250_SET_SPEED(REG_ADDRESS(start_reg), speed), buf, count);

	} else {
		ret = _interface->read(MPU9250_SET_SPEED(REG_ADDRESS(start_reg), speed), buf, count);
	}

	return ret;
}

uint16_t
ICM20948::read_reg16(unsigned reg)
{
	uint8_t buf[2];

	// general register transfer at low clock speed

	if (_whoami == ICM_WHOAMI_20948) {
		select_register_bank(REG_BANK(reg));
		_interface->read(MPU9250_LOW_SPEED_OP(REG_ADDRESS(reg)), &buf, arraySize(buf));

	} else {
		_interface->read(MPU9250_LOW_SPEED_OP(reg), &buf, arraySize(buf));
	}

	return (uint16_t)(buf[0] << 8) | buf[1];
}

void
ICM20948::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed

	if (_whoami == ICM_WHOAMI_20948) {
		select_register_bank(REG_BANK(reg));
		_interface->write(MPU9250_LOW_SPEED_OP(REG_ADDRESS(reg)), &value, 1);

	} else {
		_interface->write(MPU9250_LOW_SPEED_OP(reg), &value, 1);
	}
}

void
ICM20948::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
ICM20948::modify_checked_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
ICM20948::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < _num_checked_registers; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
			break;
		}
	}
}

int
ICM20948::set_accel_range(unsigned max_g_in)
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

	switch (_whoami) {
	case ICM_WHOAMI_20948:
		modify_checked_reg(ICMREG_20948_ACCEL_CONFIG, ICM_BITS_ACCEL_FS_SEL_MASK, afs_sel << 1);
		break;
	}

	_accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * CONSTANTS_ONE_G;

	return OK;
}

void
ICM20948::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	if (!_magnetometer_only) {
		_accel_reports->flush();
		_gyro_reports->flush();
	}

	_mag->_mag_reports->flush();

	ScheduleOnInterval(_call_interval - MPU9250_TIMER_REDUCTION, 10000);
}

void
ICM20948::stop()
{
	ScheduleClear();
}

void
ICM20948::Run()
{
	/* make another measurement */
	measure();
}

void
ICM20948::check_registers(void)
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

			if (_whoami == ICM_WHOAMI_20948) {
				// reset_mpu();
			} else {
				write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
				write_reg(MPUREG_PWR_MGMT_2, MPU_CLK_SEL_AUTO);
			}

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

	_checked_next = (_checked_next + 1) % _num_checked_registers;
}

bool ICM20948::check_null_data(uint16_t *data, uint8_t size)
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
	// the mpu9250 does go bad it would cause a FMU failure,
	// regardless of whether another sensor is available,
	return true;
}

bool ICM20948::check_duplicate(uint8_t *accel_data)
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
ICM20948::measure()
{

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct MPUReport mpu_report;

	struct ICMReport icm_report;

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
	 * Fetch the full set of measurements from the MPU9250 in one pass
	 */

	if ((!_magnetometer_only || _mag->is_passthrough()) && _register_wait == 0) {

		select_register_bank(REG_BANK(ICMREG_20948_ACCEL_XOUT_H));

		if (OK != read_reg_range(ICMREG_20948_ACCEL_XOUT_H, MPU9250_HIGH_BUS_SPEED, (uint8_t *)&icm_report,
					 sizeof(icm_report))) {
			perf_end(_sample_perf);
			return;
		}

		check_registers();

		if (check_duplicate(MPU_OR_ICM(&mpu_report.accel_x[0], &icm_report.accel_x[0]))) {
			return;
		}
	}

	/*
	 * In case of a mag passthrough read, hand the magnetometer data over to _mag. Else,
	 * try to read a magnetometer report.
	 */

#   ifdef USE_I2C

	if (_mag->is_passthrough()) {
#   endif

		_mag->_measure(mpu_report.mag);

#   ifdef USE_I2C

	} else {
		_mag->measure();
	}

#   endif

	/*
	 * Continue evaluating gyro and accelerometer results
	 */
	if (!_magnetometer_only && _register_wait == 0) {

		/*
		 * Convert from big to little endian
		 */
		if (_whoami == ICM_WHOAMI_20948) {
			report.accel_x = int16_t_from_bytes(icm_report.accel_x);
			report.accel_y = int16_t_from_bytes(icm_report.accel_y);
			report.accel_z = int16_t_from_bytes(icm_report.accel_z);
			report.temp    = int16_t_from_bytes(icm_report.temp);
			report.gyro_x  = int16_t_from_bytes(icm_report.gyro_x);
			report.gyro_y  = int16_t_from_bytes(icm_report.gyro_y);
			report.gyro_z  = int16_t_from_bytes(icm_report.gyro_z);

		} else { // MPU9250/MPU6500
			report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
			report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
			report.accel_z = int16_t_from_bytes(mpu_report.accel_z);
			report.temp    = int16_t_from_bytes(mpu_report.temp);
			report.gyro_x  = int16_t_from_bytes(mpu_report.gyro_x);
			report.gyro_y  = int16_t_from_bytes(mpu_report.gyro_y);
			report.gyro_z  = int16_t_from_bytes(mpu_report.gyro_z);
		}

		if (check_null_data((uint16_t *)&report, sizeof(report) / 2)) {
			return;
		}
	}

	if (_register_wait != 0) {
		/*
		 * We are waiting for some good transfers before using the sensor again.
		 * We still increment _good_transfers, but don't return any data yet.
		 *
		*/
		_register_wait--;
		return;
	}

	/*
	 * Get sensor temperature
	 */
	_last_temperature = (report.temp) / 333.87f + 21.0f;


	/*
	 * Convert and publish accelerometer and gyrometer data.
	 */

	if (!_magnetometer_only) {

		/*
		 * Keeping the axes as they are for ICM20948 so orientation will match the actual chip orientation
		 */
		if (_whoami != ICM_WHOAMI_20948) {
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
		}

		/*
		 * Report buffers.
		 */
		sensor_accel_s		arb;
		sensor_gyro_s			grb;

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

		matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
		matrix::Vector3f aval_integrated;

		bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
		arb.x_integral = aval_integrated(0);
		arb.y_integral = aval_integrated(1);
		arb.z_integral = aval_integrated(2);

		arb.scaling = _accel_range_scale;

		arb.temperature = _last_temperature;

		/* return device ID */
		arb.device_id = _accel->_device_id.devid;

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

		matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
		matrix::Vector3f gval_integrated;

		bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
		grb.x_integral = gval_integrated(0);
		grb.y_integral = gval_integrated(1);
		grb.z_integral = gval_integrated(2);

		grb.scaling = _gyro_range_scale;

		grb.temperature = _last_temperature;

		/* return device ID */
		grb.device_id = _gyro->_device_id.devid;

		_accel_reports->force(&arb);
		_gyro_reports->force(&grb);

		/* notify anyone waiting for data */
		if (accel_notify) {
			_accel->poll_notify(POLLIN);
		}

		if (gyro_notify) {
			_gyro->parent_poll_notify();
		}

		if (accel_notify && !(_accel->_pub_blocked)) {
			/* publish it */
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
		}

		if (gyro_notify && !(_gyro->_pub_blocked)) {
			/* publish it */
			orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
		}
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
ICM20948::print_info()
{
	::printf("Device type:%d\n", _whoami);
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);
	::printf("temperature: %.1f\n", (double)_last_temperature);

	if (!_magnetometer_only) {
		_accel_reports->print_info("accel queue");
		_gyro_reports->print_info("gyro queue");
		_mag->_mag_reports->print_info("mag queue");
	}
}

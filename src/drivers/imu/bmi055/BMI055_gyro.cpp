/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "BMI055_gyro.hpp"

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */

const uint8_t BMI055_gyro::_checked_registers[BMI055_GYRO_NUM_CHECKED_REGISTERS] = {    BMI055_GYR_CHIP_ID,
											BMI055_GYR_LPM1,
											BMI055_GYR_BW,
											BMI055_GYR_RANGE,
											BMI055_GYR_INT_EN_0,
											BMI055_GYR_INT_EN_1,
											BMI055_GYR_INT_MAP_1
										   };

BMI055_gyro::BMI055_gyro(int bus, const char *path_gyro, uint32_t device, enum Rotation rotation) :
	BMI055("BMI055_GYRO", path_gyro, bus, device, SPIDEV_MODE3, BMI055_BUS_SPEED, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi055_gyro_read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "bmi055_gyro_measure_interval")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi055_gyro_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi055_gyro_bad_registers")),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(nullptr),
	_gyro_orb_class_instance(-1),
	_gyro_class_instance(-1),
	_gyro_sample_rate(BMI055_GYRO_DEFAULT_RATE),
	_gyro_filter_x(BMI055_GYRO_DEFAULT_RATE, BMI055_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(BMI055_GYRO_DEFAULT_RATE, BMI055_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(BMI055_GYRO_DEFAULT_RATE, BMI055_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_int(1000000 / BMI055_GYRO_MAX_PUBLISH_RATE, true),
	_last_temperature(0)
{
	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_BMI055;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
}

BMI055_gyro::~BMI055_gyro()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_gyro_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_measure_interval);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
}

int
BMI055_gyro::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));

	if (_gyro_reports == nullptr) {
		return -ENOMEM;
	}

	ret = reset();

	if (ret != OK) {
		return ret;
	}

	/* Initialize offsets and scales */
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	param_t gyro_cut_ph = param_find("IMU_GYRO_CUTOFF");
	float gyro_cut = BMI055_GYRO_DEFAULT_DRIVER_FILTER_FREQ;

	if (gyro_cut_ph != PARAM_INVALID && (param_get(gyro_cut_ph, &gyro_cut) == PX4_OK)) {

		_gyro_filter_x.set_cutoff_frequency(BMI055_GYRO_DEFAULT_RATE, gyro_cut);
		_gyro_filter_y.set_cutoff_frequency(BMI055_GYRO_DEFAULT_RATE, gyro_cut);
		_gyro_filter_z.set_cutoff_frequency(BMI055_GYRO_DEFAULT_RATE, gyro_cut);
	}

	_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_gyro_s grp;
	_gyro_reports->get(&grp);

	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
					  &_gyro_orb_class_instance, (external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	return ret;
}

int BMI055_gyro::reset()
{
	write_reg(BMI055_GYR_SOFTRESET, BMI055_SOFT_RESET);//Soft-reset
	usleep(5000);
	write_checked_reg(BMI055_GYR_BW,     0); // Write Gyro Bandwidth (will be overwritten in gyro_set_sample_rate())
	write_checked_reg(BMI055_GYR_RANGE,     0);// Write Gyro range
	write_checked_reg(BMI055_GYR_INT_EN_0,      BMI055_GYR_DRDY_INT_EN); //Enable DRDY interrupt
	write_checked_reg(BMI055_GYR_INT_MAP_1,     BMI055_GYR_DRDY_INT1); //Map DRDY interrupt on pin INT1

	set_gyro_range(BMI055_GYRO_DEFAULT_RANGE_DPS);// set Gyro range
	gyro_set_sample_rate(BMI055_GYRO_DEFAULT_RATE);// set Gyro ODR & Filter Bandwidth

	//Enable Gyroscope in normal mode
	write_reg(BMI055_GYR_LPM1, BMI055_GYRO_NORMAL);
	up_udelay(1000);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI055_GYRO_NUM_CHECKED_REGISTERS; i++) {
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
BMI055_gyro::probe()
{
	/* look for device ID */
	_whoami = read_reg(BMI055_GYR_CHIP_ID);

	// verify product revision
	switch (_whoami) {
	case BMI055_GYR_WHO_AM_I:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

int
BMI055_gyro::gyro_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI055_GYRO_BW_MASK;

	if (frequency <= 100) {
		setbits |= BMI055_GYRO_RATE_100; /* 32 Hz cutoff */
		_gyro_sample_rate = 100;

	} else if (frequency <= 250) {
		setbits |= BMI055_GYRO_RATE_400; /* 47 Hz cutoff */
		_gyro_sample_rate = 400;

	} else if (frequency <= 1000) {
		setbits |= BMI055_GYRO_RATE_1000; /* 116 Hz cutoff */
		_gyro_sample_rate = 1000;

	} else if (frequency > 1000) {
		setbits |= BMI055_GYRO_RATE_2000; /* 230 Hz cutoff */
		_gyro_sample_rate = 2000;

	} else {
		return -EINVAL;
	}

	modify_reg(BMI055_GYR_BW, clearbits, setbits);

	return OK;
}

int
BMI055_gyro::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
BMI055_gyro::test_error()
{
	write_reg(BMI055_GYR_SOFTRESET, BMI055_SOFT_RESET);
	::printf("error triggered\n");
	print_registers();
}

ssize_t
BMI055_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_gyro_s);

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

	/* copy reports out of our buffer to the caller */
	sensor_gyro_s *grp = reinterpret_cast<sensor_gyro_s *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(sensor_gyro_s));
}

int
BMI055_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, BMI055_GYRO_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					/* update interval for next measurement */
					_call_interval = ticks;

					/*
					  set call interval faster than the sample time. We
					  then detect when we have duplicate samples and reject
					  them. This prevents aliasing due to a beat between the
					  stm32 clock and the bmi055 clock
					 */
					_call.period = _call_interval - BMI055_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

void
BMI055_gyro::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
BMI055_gyro::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI055_GYRO_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI055_gyro::set_gyro_range(unsigned max_dps)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI055_GYRO_RANGE_125_DPS | BMI055_GYRO_RANGE_250_DPS;
	float lsb_per_dps;
	float max_gyro_dps;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 125) {
		max_gyro_dps = 125;
		lsb_per_dps = 262.4;
		setbits |= BMI055_GYRO_RANGE_125_DPS;

	} else if (max_dps <= 250) {
		max_gyro_dps = 250;
		lsb_per_dps = 131.2;
		setbits |= BMI055_GYRO_RANGE_250_DPS;

	} else if (max_dps <= 500) {
		max_gyro_dps = 500;
		lsb_per_dps = 65.6;
		setbits |= BMI055_GYRO_RANGE_500_DPS;

	} else if (max_dps <= 1000) {
		max_gyro_dps = 1000;
		lsb_per_dps = 32.8;
		setbits |= BMI055_GYRO_RANGE_1000_DPS;

	} else if (max_dps <= 2000) {
		max_gyro_dps = 2000;
		lsb_per_dps = 16.4;
		setbits |= BMI055_GYRO_RANGE_2000_DPS;

	} else {
		return -EINVAL;
	}

	_gyro_range_rad_s = (max_gyro_dps / 180.0f * M_PI_F);
	_gyro_range_scale = (M_PI_F / (180.0f * lsb_per_dps));

	modify_reg(BMI055_GYR_RANGE, clearbits, setbits);

	return OK;
}

void
BMI055_gyro::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_gyro_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call,
		       1000,
		       _call_interval - BMI055_TIMER_REDUCTION,
		       (hrt_callout)&BMI055_gyro::measure_trampoline, this);
	reset();
}

void
BMI055_gyro::stop()
{
	hrt_cancel(&_call);
}

void
BMI055_gyro::measure_trampoline(void *arg)
{
	BMI055_gyro *dev = reinterpret_cast<BMI055_gyro *>(arg);

	/* make another measurement */
	dev->measure();
}

void
BMI055_gyro::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) !=
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
			write_reg(BMI055_GYR_SOFTRESET, BMI055_SOFT_RESET);
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

	_checked_next = (_checked_next + 1) % BMI055_GYRO_NUM_CHECKED_REGISTERS;
}

void
BMI055_gyro::measure()
{
	perf_count(_measure_interval);

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct BMI_GyroReport bmi_gyroreport;

	struct Report {
		int16_t     temp;
		int16_t     gyro_x;
		int16_t     gyro_y;
		int16_t     gyro_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the BMI055 gyro in one pass.
	 */
	bmi_gyroreport.cmd = BMI055_GYR_X_L | DIR_READ;


	if (OK != transfer((uint8_t *)&bmi_gyroreport, ((uint8_t *)&bmi_gyroreport), sizeof(bmi_gyroreport))) {
		return;
	}

	check_registers();

	int8_t temp = read_reg(BMI055_ACC_TEMP);
	report.temp = temp;

	report.gyro_x = bmi_gyroreport.gyro_x;
	report.gyro_y = bmi_gyroreport.gyro_y;
	report.gyro_z = bmi_gyroreport.gyro_z;

	if (report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the bmi055 does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again, but don't return any data yet
		_register_wait--;
		return;
	}

	/*
	 * Report buffers.
	 */
	sensor_gyro_s grb;

	grb.timestamp = hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	grb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 *   at a nominally 'zero' input. Therefore the offset has to
	 *   be subtracted.
	 *
	 *   Example: A gyro outputs a value of 74 at zero angular rate
	 *        the offset is 74 from the origin and subtracting
	 *        74 from all measurements centers them around zero.
	 */

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	float xraw_f = report.gyro_x;
	float yraw_f = report.gyro_y;
	float zraw_f = report.gyro_z;

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

	bool gyro_notify = _gyro_int.put(grb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	grb.scaling = _gyro_range_scale;

	/*
	 * Temperature is reported as Eight-bit 2’s complement sensor temperature value
	 * with 0.5 °C/LSB sensitivity and an offset of 23.0 °C
	 */
	_last_temperature = (report.temp * 0.5f) + 23.0f;
	grb.temperature = _last_temperature;

	grb.device_id = _device_id.devid;

	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
	if (gyro_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI055_gyro::print_info()
{
	PX4_INFO("Gyro");

	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);

	_gyro_reports->print_info("gyro queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI055_GYRO_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

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
	printf("\n");
}

void
BMI055_gyro::print_registers()
{
	uint8_t index = 0;
	printf("BMI055 gyro registers\n");

	uint8_t reg = _checked_registers[index++];
	uint8_t v = read_reg(reg);
	printf("Gyro Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Power: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Bw: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Range: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Int-en-0: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Int-en-1: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Int-Map-1: %02x:%02x ", (unsigned)reg, (unsigned)v);

	printf("\n");
}

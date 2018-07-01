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

#include "RM3100.hpp"

RM3100::RM3100(int bus_number, int address, const char *path, enum Rotation rotation) :
	I2C("RM3100", path, bus_number, address, RM3100_DEFAULT_BUS_SPEED),
	_sample_perf(perf_alloc(PC_ELAPSED, "rm3100_read")),
	_comms_errors(perf_alloc(PC_COUNT, "rm3100_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "rm3100_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "rm3100_conf_err")),
	_rotation(rotation)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_RM3100;

	// enable debug() calls
	_debug_enabled = true;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

RM3100::~RM3100()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
RM3100::init()
{
	int ret = PX4_ERROR;

	ret = I2C::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr) {
		goto out;
	}

	/* reset the device configuration */
//	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}

int
RM3100::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int
RM3100::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}


/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void RM3100::check_conf(void)
{
	int ret;

	uint8_t ctrl_reg_in = 0;
	ret = read_reg(ADDR_CTRL3, ctrl_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (ctrl_reg_in != _ctl3_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CTRL3, _ctl3_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}

		_ctl_reg_mismatch = true;
	}


	_ctl_reg_mismatch = false;
}

ssize_t
RM3100::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(RM3100_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	return ret;
}

int
RM3100::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(RM3100_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(RM3100_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / TICK2USEC(_measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	//case SENSORIOCRESET:
	//return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000 / TICK2USEC(_measure_ticks);

	case MAGIOCSRANGE:
		return OK;

	case MAGIOCGRANGE:
		return 0;

	case MAGIOCEXSTRAP:
		return set_selftest(arg);

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		/* check calibration, but not actually return an error */
		(void)check_calibration();
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCSELFTEST:
		return check_calibration();

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return external();

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
RM3100::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&RM3100::cycle_trampoline, this, 1);
}

void
RM3100::stop()
{
	work_cancel(HPWORK, &_work);
}


//int
//RM3100::reset()
//{
/* software reset */
//	write_reg(ADDR_CTRL2);

/* configure control register 3 */
//	_ctl3_reg = CTRL3_SAMPLEAVG_16;
//	write_reg(ADDR_CTRL3, _ctl3_reg);

/* configure control register 4 */
//	_ctl4_reg = CTRL4_SRPD;
//	write_reg(ADDR_CTRL4, _ctl4_reg);

//	return OK;

//}


void
RM3100::cycle_trampoline(void *arg)
{
	RM3100 *dev = (RM3100 *)arg;

	dev->cycle();
}

/*int
RM3100::probe()
{
	uint8_t data[1] = {0};

	_retries = 10;

	//if (read(ADDR_WAI, &data[0], 1)) {
	//	DEVICE_DEBUG("read_reg fail");
	//	return -EIO;
	//}

	//_retries = 2;

	//if ((data[0] != WAI_EXPECTED_VALUE)) {
	//	DEVICE_DEBUG("ID byte mismatch (%02x) expected %02x", data[0], WAI_EXPECTED_VALUE);
	//	return -EIO;
	//}

	return OK;
}
*/
void
RM3100::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(RM3100_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&RM3100::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(RM3100_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&RM3100::cycle_trampoline,
		   this,
		   USEC2TICK(RM3100_CONVERSION_INTERVAL));
}

int
RM3100::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_CTRL1, CTRL1_MODE_SINGLE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
RM3100::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t     x[2];
		uint8_t     y[2];
		uint8_t     z[2];
	} report_buffer;
#pragma pack(pop)
	struct {
		int32_t     x, y, z;
	} report;

	int ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	struct mag_report new_report;
	const bool sensor_is_external = external();

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.is_external = sensor_is_external;
	new_report.error_count = perf_event_count(_comms_errors);
	new_report.range_ga = 1.6f; // constant for this sensor for x and y
	new_report.scaling = _range_scale;
	new_report.device_id = _device_id.devid;

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	ret = read(ADDR_DATA_OUT_X_LSB, (uint8_t *)&report_buffer, sizeof(report_buffer));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("I2C read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int32_t)report_buffer.x[1]) << 8) | (int32_t)report_buffer.x[0];
	report.y = (((int32_t)report_buffer.y[1]) << 8) | (int32_t)report_buffer.y[0];
	report.z = (((int32_t)report_buffer.z[1]) << 8) | (int32_t)report_buffer.z[0];


	/*
	 * Check if value makes sense according to the FSR and Resolution of
	 * this sensor, discarding outliers
	 */
	if (report.x > RM3100_MAX_VAL_XY || report.x < RM3100_MIN_VAL_XY ||
	    report.y > RM3100_MAX_VAL_XY || report.y < RM3100_MIN_VAL_XY ||
	    report.z > RM3100_MAX_VAL_Z  || report.z < RM3100_MIN_VAL_Z) {
		perf_count(_range_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/* temperature measurement is not available on RM3100 */
	new_report.temperature = 0;

	/*
	 * raw outputs
	 *
	 * Sensor doesn't follow right hand rule, swap x and y to make it obey
	 * it.
	 */
	new_report.x_raw = report.y;
	new_report.y_raw = report.x;
	new_report.z_raw = report.z;

	/* scale values for output */
	xraw_f = report.y;
	yraw_f = report.x;
	zraw_f = report.z;

	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);
	new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
							 &_orb_class_instance, sensor_is_external ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

			if (_mag_topic == nullptr) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	_reports->force(&new_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int RM3100::calibrate(struct file *filp, unsigned enable)
{
	struct mag_report report;
	ssize_t sz;
	int ret = 1;
	float total_x = 0.0f;
	float total_y = 0.0f;
	float total_z = 0.0f;

	// XXX do something smarter here
	int fd = (int)enable;

	struct mag_calibration_s mscale_previous;

	struct mag_calibration_s mscale_null;
	mscale_null.x_offset = 0.0f;
	mscale_null.x_scale = 1.0f;
	mscale_null.y_offset = 0.0f;
	mscale_null.y_scale = 1.0f;
	mscale_null.z_offset = 0.0f;
	mscale_null.z_scale = 1.0f;

	float sum_in_test[3] =   {0.0f, 0.0f, 0.0f};
	float sum_in_normal[3] = {0.0f, 0.0f, 0.0f};
	float *sum = &sum_in_normal[0];

	if (OK != ioctl(filp, MAGIOCGSCALE, (long unsigned int)&mscale_previous)) {
		PX4_WARN("FAILED: MAGIOCGSCALE 1");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
		PX4_WARN("FAILED: MAGIOCSSCALE 1");
		ret = 1;
		goto out;
	}

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(filp, SENSORIOCSPOLLRATE, 50)) {
		PX4_WARN("FAILED: SENSORIOCSPOLLRATE 50Hz");
		ret = 1;
		goto out;
	}

	// discard 10 samples to let the sensor settle
	/* read the sensor 50 times */

	for (uint8_t p = 0; p < 2; p++) {

		if (p == 1) {

			/* start the Self test */

			if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
				PX4_WARN("FAILED: MAGIOCEXSTRAP 1");
				ret = 1;
				goto out;
			}

			sum = &sum_in_test[0];
		}


		for (uint8_t i = 0; i < 30; i++) {


			struct pollfd fds;

			/* wait for data to be ready */
			fds.fd = fd;
			fds.events = POLLIN;
			ret = ::poll(&fds, 1, 2000);

			if (ret != 1) {
				PX4_WARN("ERROR: TIMEOUT 2");
				goto out;
			}

			/* now go get it */

			sz = ::read(fd, &report, sizeof(report));

			if (sz != sizeof(report)) {
				PX4_WARN("ERROR: READ 2");
				ret = -EIO;
				goto out;
			}

			if (i > 10) {
				sum[0] += report.x_raw;
				sum[1] += report.y_raw;
				sum[2] += report.z_raw;
			}
		}
	}

	total_x = fabsf(sum_in_test[0] - sum_in_normal[0]);
	total_y = fabsf(sum_in_test[1] - sum_in_normal[1]);
	total_z = fabsf(sum_in_test[2] - sum_in_normal[2]);

	ret = ((total_x + total_y + total_z) < (float)0.000001);

out:

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_previous)) {
		PX4_WARN("FAILED: MAGIOCSSCALE 2");
	}

	/* set back to normal mode */

	if (OK != ::ioctl(fd, MAGIOCEXSTRAP, 0)) {
		PX4_WARN("FAILED: MAGIOCEXSTRAP 0");
	}

	if (ret == OK) {
		if (check_scale()) {
			/* failed */
			PX4_WARN("FAILED: SCALE");
			ret = PX4_ERROR;
		}

	} else {
		PX4_ERR("FAILED: CALIBRATION SCALE %d, %d, %d", (int)total_x, (int)total_y, (int)total_z);
	}

	return ret;
}

int RM3100::check_scale()
{
	return OK;
}

int RM3100::check_offset()
{
	bool offset_valid;

	if ((-2.0f * FLT_EPSILON < _scale.x_offset && _scale.x_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.y_offset && _scale.y_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.z_offset && _scale.z_offset < 2.0f * FLT_EPSILON)) {
		/* offset is zero */
		offset_valid = false;

	} else {
		offset_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !offset_valid;
}

int RM3100::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {

		if (!scale_valid || !offset_valid) {
			PX4_WARN("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
				 (offset_valid) ? "" : "offset invalid");
		}

		_calibrated = (offset_valid && scale_valid);
	}

	/* return 0 if calibrated, 1 else */
	return (!_calibrated);
}

int
RM3100::set_selftest(unsigned enable)
{
	int ret;
	uint8_t str;
	/* arm the excitement strap */
	ret = read_reg(ADDR_STR, str);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	str &= ~STR_SELF_TEST_ON; // reset previous test

	if (enable > 0) {
		str |= STR_SELF_TEST_ON;

	}

	ret = write_reg(ADDR_STR, str);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t str_reg_ret = 0;
	read_reg(ADDR_STR, str_reg_ret);

	return !(str == str_reg_ret);
}

int
RM3100::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return write(reg, &buf, 1);
}

int
RM3100::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
RM3100::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t b[2];
		int16_t w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
RM3100::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	print_message(_last_report);
	_reports->print_info("report queue");
}

/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "QMC5883.hpp"

QMC5883::QMC5883(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus,
		 int i2c_address) :
	CDev("QMC5883", nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus, i2c_address),
	_interface(interface),
	_reports(nullptr),
	_scale{},
	_range_scale(1.0f / 12000.0f),
	_range_ga(2.0f),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_err")),
	_sensor_ok(false),
	_rotation(rotation),
	_range_bits(0),
	_conf_reg(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_QMC5883;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

QMC5883::~QMC5883()
{
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
QMC5883::init()
{
	int ret = PX4_ERROR;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_mag_s));

	if (_reports == nullptr) {
		goto out;
	}

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = OK;
	/* sensor is ok */
	_sensor_ok = true;

	_measure_interval = QMC5883_CONVERSION_INTERVAL;
	start();

out:
	return ret;
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void QMC5883::check_conf(void)
{
	int ret;

	uint8_t conf_reg_in = 0;
	ret = read_reg(QMC5883_ADDR_CONTROL_1, conf_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (conf_reg_in != _conf_reg) {
		perf_count(_conf_errors);
		ret = write_reg(QMC5883_ADDR_CONTROL_1, _conf_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

ssize_t
QMC5883::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_mag_s);
	sensor_mag_s *mag_buf = reinterpret_cast<sensor_mag_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(sensor_mag_s);
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

		/* wait for it to complete */
		px4_usleep(QMC5883_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(sensor_mag_s);
		}
	} while (0);

	return ret;
}

int
QMC5883::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_interval = QMC5883_CONVERSION_INTERVAL;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* convert hz to interval in microseconds */
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < QMC5883_CONVERSION_INTERVAL) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSRANGE:
		return OK;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
QMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleNow();
}

int
QMC5883::reset()
{
	/* read 0x00 once */
	uint8_t data_bits_in = 0;
	read_reg(QMC5883_ADDR_DATA_OUT_X_LSB, data_bits_in);

	/* software reset */
	write_reg(QMC5883_ADDR_CONTROL_2, QMC5883_SOFT_RESET);

	/* set reset period to 0x01 */
	write_reg(QMC5883_ADDR_SET_RESET, QMC5883_SET_DEFAULT);

	//use fixed range of 2G
	_range_scale = 1.0f / 12000.0f;   // 12000 LSB/Gauss at +/- 2G range
	_range_ga = 2.00f;
	_range_bits = 0x00;

	/* set control register */
	_conf_reg = QMC5883_MODE_REG_CONTINOUS_MODE |
		    QMC5883_OUTPUT_DATA_RATE_200 |
		    QMC5883_OVERSAMPLE_512 |
		    QMC5883_OUTPUT_RANGE_2G;
	write_reg(QMC5883_ADDR_CONTROL_1, _conf_reg);

	return OK;
}

void
QMC5883::RunImpl()
{
	if (_measure_interval == 0) {
		return;
	}

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
		if (_measure_interval > QMC5883_CONVERSION_INTERVAL) {
			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - QMC5883_CONVERSION_INTERVAL);

			return;
		}
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(QMC5883_CONVERSION_INTERVAL);
	}
}


int
QMC5883::collect()
{
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		y[2];
		uint8_t		z[2];
	} qmc_report{};

	struct {
		int16_t		x, y, z;
	} report{};

	int	ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	sensor_mag_s new_report;
	bool sensor_is_onboard = false;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);
	new_report.scaling = _range_scale;
	new_report.device_id = _device_id.devid;

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	ret = _interface->read(QMC5883_ADDR_DATA_OUT_X_LSB, (uint8_t *)&qmc_report, sizeof(qmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/* map data we just received LSB, MSB */
	report.x = (((int16_t)qmc_report.x[1]) << 8) + qmc_report.x[0];
	report.y = (((int16_t)qmc_report.y[1]) << 8) + qmc_report.y[0];
	report.z = (((int16_t)qmc_report.z[1]) << 8) + qmc_report.z[0];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > QMC5883_MAX_COUNT) ||
	    (abs(report.y) > QMC5883_MAX_COUNT) ||
	    (abs(report.z) > QMC5883_MAX_COUNT)) {
		perf_count(_comms_errors);
		goto out;
	}

	/* get temperature measurements from the device */
	new_report.temperature = 0;

	if (_temperature_counter++ == 100) {
		uint8_t raw_temperature[2];

		_temperature_counter = 0;

		ret = _interface->read(QMC5883_ADDR_TEMP_OUT_LSB,
				       raw_temperature, sizeof(raw_temperature));

		if (ret == OK) {
			int16_t temp16 = (((int16_t)raw_temperature[1]) << 8) +
					 raw_temperature[0];
			new_report.temperature = QMC5883_TEMP_OFFSET + temp16 * 1.0f / 100.0f;
		}

	} else {
		new_report.temperature = _last_report.temperature;
	}


	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	new_report.x_raw = -report.y;
	new_report.y_raw = report.x;
	/* z remains z */
	new_report.z_raw = report.z;

	/* scale values for output */

	// XXX revisit for SPI part, might require a bus type IOCTL
	unsigned dummy;
	sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);
	new_report.is_external = !sensor_is_onboard;

	if (sensor_is_onboard) {
		// convert onboard so it matches offboard for the
		// scaling below
		report.y = -report.y;
		report.x = -report.x;
	}

	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
	//TODO: sort out axes mapping
	xraw_f = -report.y;
	yraw_f = report.x;
	zraw_f = report.z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	/* flip axes and negate value for y */
	new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	/* z remains z */
	new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
							 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

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
	  periodically check the configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;


	if (check_counter == 0) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int
QMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
QMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void
QMC5883::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("interval:  %u us\n", _measure_interval);
	print_message(_last_report);
	_reports->print_info("report queue");
}

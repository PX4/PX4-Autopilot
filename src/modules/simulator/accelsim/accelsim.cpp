/****************************************************************************
 *
 * Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
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
 * @file accelsim.cpp
 * Driver for a simulated accelerometer / magnetometer.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <simulator/simulator.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include <VirtDevObj.hpp>

#define ACCELSIM_DEVICE_PATH_ACCEL	"/dev/sim_accel"
#define ACCELSIM_DEVICE_PATH_MAG	"/dev/sim_mag"

#define ADDR_WHO_AM_I			0x0F

#define ACCELSIM_ACCEL_DEFAULT_RATE			250
#define ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ACC_READ 				(1<<5)
#define MAG_READ 				(1<<6)

extern "C" { __EXPORT int accelsim_main(int argc, char *argv[]); }

using namespace DriverFramework;

class ACCELSIM_mag;

class ACCELSIM : public VirtDevObj
{
public:
	ACCELSIM(const char *path, enum Rotation rotation);
	virtual ~ACCELSIM();

	virtual int		init();

	virtual ssize_t		devRead(void *buffer, size_t buflen);
	virtual int		devIOCTL(unsigned long cmd, unsigned long arg);

	/**
	 * dump register values
	 */
	void			print_registers();

protected:
	friend class 		ACCELSIM_mag;

	ssize_t			mag_read(void *buffer, size_t buflen);
	int			mag_ioctl(unsigned long cmd, unsigned long arg);

	int			transfer(uint8_t *send, uint8_t *recv, unsigned len);
private:

	ACCELSIM_mag		*_mag;

	ringbuffer::RingBuffer	*_accel_reports;
	ringbuffer::RingBuffer	*_mag_reports;

	struct accel_calibration_s	_accel_scale;
	unsigned		_accel_range_m_s2;
	float			_accel_range_scale;
	unsigned		_accel_samplerate;
	unsigned		_accel_onchip_filter_bandwith;

	struct mag_calibration_s	_mag_scale;
	unsigned		_mag_range_ga;
	float			_mag_range_scale;
	unsigned		_mag_samplerate;

	orb_advert_t		_accel_topic{nullptr};
	int			_accel_orb_class_instance{-1};
	int			_accel_class_instance{-1};

	unsigned		_accel_read;
	unsigned		_mag_read;

	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_mag_sample_perf;
	perf_counter_t		_accel_reschedules;
	perf_counter_t		_bad_registers;
	perf_counter_t		_bad_values;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

	enum Rotation		_rotation;

	// values used to
	float			_last_accel[3];
	uint8_t			_constant_accel_count;

	// last temperature value
	float			_last_temperature;

	/**
	 * Override Start automatic measurement.
	 */
	virtual int		start();

	/**
	 * Override Stop automatic measurement.
	 */
	virtual int		stop();

	/**
	 * Fetch accel measurements from the sensor and update the report ring.
	 */
	virtual void		_measure();

	/**
	 * Fetch mag measurements from the sensor and update the report ring.
	 */
	void			mag_measure();

	/**
	 * Set the ACCELSIM accel measurement range.
	 *
	 * @param max_g	The measurement range of the accel is in g (9.81m/s^2)
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_range(unsigned max_g);

	/**
	 * Set the ACCELSIM mag measurement range.
	 *
	 * @param max_ga	The measurement range of the mag is in Ga
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			mag_set_range(unsigned max_g);

	/**
	 * Set the driver lowpass filter bandwidth.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_driver_lowpass_filter(float samplerate, float bandwidth);

	/* this class cannot be copied */
	ACCELSIM(const ACCELSIM &);
	ACCELSIM operator=(const ACCELSIM &);
};

/**
 * Helper class implementing the mag driver node.
 */
class ACCELSIM_mag : public VirtDevObj
{
public:
	ACCELSIM_mag(ACCELSIM *parent);
	~ACCELSIM_mag() = default;

	virtual ssize_t	devRead(void *buffer, size_t buflen);
	virtual int	devIOCTL(unsigned long cmd, unsigned long arg);

	virtual int 	start();
	virtual int 	stop();

protected:
	friend class ACCELSIM;

private:
	ACCELSIM	*_parent;

	orb_advert_t	_mag_topic{nullptr};
	int		_mag_orb_class_instance{-1};
	int		_mag_class_instance{-1};

	virtual void	_measure();

	/* this class does not allow copying due to ptr data members */
	ACCELSIM_mag(const ACCELSIM_mag &) = delete;
	ACCELSIM_mag operator=(const ACCELSIM_mag &) = delete;
};


ACCELSIM::ACCELSIM(const char *path, enum Rotation rotation) :
	VirtDevObj("ACCELSIM", path, ACCEL_BASE_DEVICE_PATH, 1e6 / 400),
	_mag(new ACCELSIM_mag(this)),
	_accel_reports(nullptr),
	_mag_reports(nullptr),
	_accel_scale{},
	_accel_range_m_s2(0.0f),
	_accel_range_scale(0.0f),
	_accel_samplerate(0),
	_accel_onchip_filter_bandwith(0),
	_mag_scale{},
	_mag_range_ga(0.0f),
	_mag_range_scale(0.0f),
	_mag_samplerate(0),
	_accel_read(0),
	_mag_read(0),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "sim_accel_read")),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "sim_mag_read")),
	_accel_reschedules(perf_alloc(PC_COUNT, "sim_accel_resched")),
	_bad_registers(perf_alloc(PC_COUNT, "sim_bad_registers")),
	_bad_values(perf_alloc(PC_COUNT, "sim_bad_values")),
	_accel_filter_x(ACCELSIM_ACCEL_DEFAULT_RATE, ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(ACCELSIM_ACCEL_DEFAULT_RATE, ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(ACCELSIM_ACCEL_DEFAULT_RATE, ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_rotation(rotation),
	_constant_accel_count(0),
	_last_temperature(0)
{
	m_id.dev_id_s.bus = 1;
	m_id.dev_id_s.devtype = DRV_ACC_DEVTYPE_ACCELSIM;

	/* Prime _mag with parents devid. */
	_mag->m_id.dev_id = m_id.dev_id;
	_mag->m_id.dev_id_s.devtype = DRV_MAG_DEVTYPE_ACCELSIM;

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

ACCELSIM::~ACCELSIM()
{
	/* make sure we are truly inactive */
	_mag->stop();
	stop();

	/* free any existing reports */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	delete _mag;

	/* delete the perf counter */
	perf_free(_accel_sample_perf);
	perf_free(_mag_sample_perf);
	perf_free(_bad_registers);
	perf_free(_bad_values);
	perf_free(_accel_reschedules);
}

int
ACCELSIM::init()
{
	/* do SIM init first */
	int ret = VirtDevObj::init();

	if (ret != PX4_OK) {
		PX4_WARN("SIM init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));

	if (_accel_reports == nullptr) {
		PX4_WARN("_accel_reports creation failed");
		return -ENOMEM;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_mag_s));

	if (_mag_reports == nullptr) {
		PX4_WARN("_mag_reports creation failed");
		return -ENOMEM;
	}

	/* do init for the mag device node */
	ret = _mag->init();

	if (ret != PX4_OK) {
		PX4_WARN("MAG init failed");
		return ret;
	}

	return PX4_OK;
}

int
ACCELSIM::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	uint8_t cmd = send[0];

	if (cmd & DIR_READ) {
		// Get data from the simulator
		Simulator *sim = Simulator::getInstance();

		if (sim == nullptr) {
			return ENODEV;
		}

		// FIXME - not sure what interrupt status should be
		recv[1] = 0;

		// skip cmd and status bytes
		if (cmd & ACC_READ) {
			sim->getRawAccelReport(&recv[2], len - 2);

		} else if (cmd & MAG_READ) {
			sim->getMagReport(&recv[2], len - 2);
		}
	}

	return PX4_OK;
}

ssize_t
ACCELSIM::devRead(void *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_accel_s);
	sensor_accel_s *arb = reinterpret_cast<sensor_accel_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (m_sample_interval_usecs > 0) {
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
	_measure();

	/* measurement will have generated a report, copy it out */
	if (_accel_reports->get(arb)) {
		ret = sizeof(*arb);
	}

	return ret;
}

ssize_t
ACCELSIM::mag_read(void *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	mag_report *mrb = reinterpret_cast<mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_mag->m_sample_interval_usecs > 0) {

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
	_mag->_measure();

	/* measurement will have generated a report, copy it out */
	if (_mag_reports->get(mrb)) {
		ret = sizeof(*mrb);
	}

	return ret;
}

int
ACCELSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	unsigned long ul_arg = (unsigned long)arg;

	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (ul_arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				return devIOCTL(SENSORIOCSPOLLRATE, ACCELSIM_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* convert hz to hrt interval via microseconds */
					unsigned interval = 1000000 / ul_arg;

					/* check against maximum sane rate */
					if (interval < 500) {
						return -EINVAL;
					}

					/* adjust filters */
					accel_set_driver_lowpass_filter((float)ul_arg, _accel_filter_x.get_cutoff_freq());

					bool want_start = (m_sample_interval_usecs == 0);

					/* update interval for next measurement */
					setSampleInterval(interval);

					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		// Nothing to do for simulator
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

	default:
		/* give it to the superclass */
		return VirtDevObj::devIOCTL(cmd, arg);
	}
}

int
ACCELSIM::mag_ioctl(unsigned long cmd, unsigned long arg)
{
	unsigned long ul_arg = (unsigned long)arg;

	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				/* 100 Hz is max for mag */
				return mag_ioctl(SENSORIOCSPOLLRATE, 100);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* convert hz to hrt interval via microseconds */
					unsigned interval = 1000000 / ul_arg;

					/* check against maximum sane rate (1ms) */
					if (interval < 10000) {
						return -EINVAL;
					}

					bool want_start = (_mag->m_sample_interval_usecs == 0);

					/* update interval for next measurement */
					_mag->setSampleInterval(interval);

					if (want_start) {
						_mag->start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		// Nothing to do for simulator
		return OK;

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_calibration_s *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_calibration_s *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCSRANGE:
		return mag_set_range(arg);

	case MAGIOCGEXTERNAL:
		/* Even if this sensor is on the "external" SPI bus
		 * it is still fixed to the autopilot assembly,
		 * so always return 0.
		 */
		return 0;

	default:
		/* give it to the superclass */
		return VirtDevObj::devIOCTL(cmd, arg);
	}
}

int
ACCELSIM::accel_set_range(unsigned max_g)
{
	float new_scale_g_digit = 0.732e-3f;

	_accel_range_scale = new_scale_g_digit * CONSTANTS_ONE_G;

	return OK;
}

int
ACCELSIM::mag_set_range(unsigned max_ga)
{
	float new_scale_ga_digit = 0.479e-3f;

	_mag_range_scale = new_scale_ga_digit;

	return OK;
}

int
ACCELSIM::accel_set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);

	return OK;
}

int
ACCELSIM::start()
{
	//PX4_INFO("ACCELSIM::start");
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_accel_reports->flush();
	_mag_reports->flush();

	int ret2 = VirtDevObj::start();

	if (ret2 != 0) {
		PX4_ERR("ACCELSIM::start base class start failed");
	}

	return (ret2 != 0) ? -1 : 0;
}

int
ACCELSIM::stop()
{
	//PX4_INFO("ACCELSIM::stop");
	return VirtDevObj::stop();
}

void
ACCELSIM::_measure()
{
#if 0
	static int x = 0;

	// Verify the samples are being taken at the expected rate
	if (x == 99) {
		x = 0;
		PX4_INFO("ACCELSIM::measure %" PRIu64, hrt_absolute_time());

	} else {
		x++;
	}

#endif

	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		float		temperature;
		float		x;
		float		y;
		float		z;
	} raw_accel_report;
#pragma pack(pop)

	sensor_accel_s accel_report = {};

	/* start the performance counter */
	perf_begin(_accel_sample_perf);

	/* fetch data from the sensor */
	memset(&raw_accel_report, 0, sizeof(raw_accel_report));
	raw_accel_report.cmd = DIR_READ | ACC_READ;

	if (OK != transfer((uint8_t *)&raw_accel_report, (uint8_t *)&raw_accel_report, sizeof(raw_accel_report))) {
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

	accel_report.timestamp = hrt_absolute_time();
	accel_report.device_id = 1310728;

	// use the temperature from the last mag reading
	accel_report.temperature = _last_temperature;

	// report the error count as the sum of the number of bad
	// register reads and bad values. This allows the higher level
	// code to decide if it should use this sensor based on
	// whether it has had failures
	accel_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

	if (math::isZero(_accel_range_scale)) {
		_accel_range_scale = FLT_EPSILON;
	}

	accel_report.x_raw = math::constrainFloatToInt16(raw_accel_report.x / _accel_range_scale);
	accel_report.y_raw = math::constrainFloatToInt16(raw_accel_report.y / _accel_range_scale);
	accel_report.z_raw = math::constrainFloatToInt16(raw_accel_report.z / _accel_range_scale);

	accel_report.x = raw_accel_report.x;
	accel_report.y = raw_accel_report.y;
	accel_report.z = raw_accel_report.z;

	//accel_report.integral_dt = 0;
	//accel_report.x_integral = 0.0f;
	//accel_report.y_integral = 0.0f;
	//accel_report.z_integral = 0.0f;

	accel_report.temperature = 25.0f;

	accel_report.scaling = _accel_range_scale;

	_accel_reports->force(&accel_report);

	orb_publish_auto(ORB_ID(sensor_accel), &_accel_topic, &accel_report, &_accel_orb_class_instance, ORB_PRIO_DEFAULT);

	_accel_read++;

	/* stop the perf counter */
	perf_end(_accel_sample_perf);
}

void
ACCELSIM::mag_measure()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		float		temperature;
		float		x;
		float		y;
		float		z;
	} raw_mag_report;
#pragma pack(pop)

	/* start the performance counter */
	perf_begin(_mag_sample_perf);

	/* fetch data from the sensor */
	memset(&raw_mag_report, 0, sizeof(raw_mag_report));
	raw_mag_report.cmd = DIR_READ | MAG_READ;

	if (OK != transfer((uint8_t *)&raw_mag_report, (uint8_t *)&raw_mag_report, sizeof(raw_mag_report))) {
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

	mag_report mag_report = {};
	mag_report.timestamp = hrt_absolute_time();
	mag_report.device_id = 196616;
	mag_report.is_external = false;

	if (math::isZero(_mag_range_scale)) {
		_mag_range_scale = FLT_EPSILON;
	}

	float xraw_f = math::constrainFloatToInt16(raw_mag_report.x / _mag_range_scale);
	float yraw_f = math::constrainFloatToInt16(raw_mag_report.y / _mag_range_scale);
	float zraw_f = math::constrainFloatToInt16(raw_mag_report.z / _mag_range_scale);

	mag_report.x_raw = xraw_f;
	mag_report.y_raw = yraw_f;
	mag_report.z_raw = zraw_f;

	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	/* remember the temperature. The datasheet isn't clear, but it
	 * seems to be a signed offset from 25 degrees C in units of 0.125C
	 */
	_last_temperature = raw_mag_report.temperature;
	mag_report.temperature = _last_temperature;
	mag_report.x = raw_mag_report.x;
	mag_report.y = raw_mag_report.y;
	mag_report.z = raw_mag_report.z;

	_mag_reports->force(&mag_report);

	orb_publish_auto(ORB_ID(sensor_mag), &_mag->_mag_topic, &mag_report, &_mag->_mag_orb_class_instance, ORB_PRIO_LOW);

	_mag_read++;

	/* stop the perf counter */
	perf_end(_mag_sample_perf);
}

ACCELSIM_mag::ACCELSIM_mag(ACCELSIM *parent) :
	VirtDevObj("ACCELSIM_mag", ACCELSIM_DEVICE_PATH_MAG, MAG_BASE_DEVICE_PATH, 10000),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1)
{
	m_id.dev_id_s.bus = 1;
	m_id.dev_id_s.devtype = DRV_ACC_DEVTYPE_ACCELSIM;
}

ssize_t
ACCELSIM_mag::devRead(void *buffer, size_t buflen)
{
	return _parent->mag_read(buffer, buflen);
}

int
ACCELSIM_mag::devIOCTL(unsigned long cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case DEVIOCGDEVICEID:
		ret = (int)VirtDevObj::devIOCTL(cmd, arg);
		//PX4_WARN("DEVICE ID: %d", ret);
		return ret;
		break;

	default:
		return _parent->mag_ioctl(cmd, arg);
	}
}

int ACCELSIM_mag::start()
{
	//PX4_INFO("ACCELSIM_mag::start");
	return VirtDevObj::start();
}

int ACCELSIM_mag::stop()
{
	//PX4_INFO("ACCELSIM_mag::stop");
	return VirtDevObj::stop();
}

void ACCELSIM_mag::_measure()
{
	//PX4_INFO("ACCELSIM_mag::_measure");
	_parent->mag_measure();
}

/**
 * Local functions in support of the shell command.
 */
namespace accelsim
{

ACCELSIM	*g_dev;

int	start(enum Rotation rotation);
int	info();
void	usage();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
int
start(enum Rotation rotation)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	DevHandle h;
	DevHandle h_mag;

	/* create the driver */
	g_dev = new ACCELSIM(ACCELSIM_DEVICE_PATH_ACCEL, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating ACCELSIM obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed init of ACCELSIM obj");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	DevMgr::getHandle(ACCELSIM_DEVICE_PATH_ACCEL, h);

	if (!h.isValid()) {
		PX4_WARN("open %s failed", ACCELSIM_DEVICE_PATH_ACCEL);
		goto fail;
	}

	if (h.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("ioctl SENSORIOCSPOLLRATE %s failed", ACCELSIM_DEVICE_PATH_ACCEL);
		DevMgr::releaseHandle(h);
		goto fail;
	}

	DevMgr::getHandle(ACCELSIM_DEVICE_PATH_MAG, h_mag);

	/* don't fail if mag dev cannot be opened */
	if (h_mag.isValid()) {
		if (h_mag.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			PX4_ERR("ioctl SENSORIOCSPOLLRATE %s failed", ACCELSIM_DEVICE_PATH_MAG);
		}

	} else {
		PX4_ERR("ioctl SENSORIOCSPOLLRATE %s failed", ACCELSIM_DEVICE_PATH_MAG);
	}

	DevMgr::releaseHandle(h);
	DevMgr::releaseHandle(h_mag);

	return 0;
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return 1;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	unsigned dummy = 0;
	PX4_WARN("device_id: %u", (unsigned int)g_dev->devIOCTL(DEVIOCGDEVICEID, dummy));

	return 0;
}

void
usage()
{
	PX4_WARN("Usage: accelsim 'start', 'info'");
	PX4_WARN("options:");
	PX4_WARN("    -R rotation");
}

} // namespace

int
accelsim_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			accelsim::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		accelsim::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ret = accelsim::start(rotation);
	}

	/*
	 * Print driver information.
	 */
	else if (!strcmp(verb, "info")) {
		ret = accelsim::info();
	}

	else {
		accelsim::usage();
		return 1;
	}

	return ret;
}

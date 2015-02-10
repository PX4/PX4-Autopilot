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
 * @file bmp085.cpp
 * @author Greg Hulands <ghulands@me.com>
 *
 * Driver for the BMP085 barometer connected via I2C.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <float.h>

#include "board_config.h"

/*
 * BMP085 internal constants and data structures.
 */

#define BMP085_BUS			PX4_I2C_BUS_EXPANSION
#define BMP085_ADDRESS		0x77

/* Max measurement time is 30ms when using kUltraHighResolution */
#define BMP085_CONVERSION_INTERVAL	(30000)	/* microseconds */

/* BMP085 Registers addresses */

#define BMP085_REGISTER_CALIBRATION_DATA 0XAA
#define BMP085_REGISTER_TAKE_TEMPERATURE 0xF4
#define BMP085_REGISTER_READ_TEMPERATURE 0xF6
#define BMP085_REGISTER_TAKE_PRESSURE 0XF4
#define BMP085_REGISTER_READ_PRESSURE 0xF6

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* Eclipse doesn't resolve OK properly */
#ifndef OK
#define OK 0
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

typedef enum {
	kUltraLowPower = 0,
	kStandardResolution,
	kHighResolution,
	kUltraHighResolution
}BMP085OversamplingMode;

uint32_t BMP085PressureWaitTimes[] = { 5000, 8000, 14000, 26000 }; // wait an extra 0.5ms

typedef struct {
	int16_t		ac1;
	int16_t		ac2;
	int16_t		ac3;
	uint16_t	ac4;
	uint16_t	ac5;
	uint16_t	ac6;
	int16_t		b1;
	int16_t		b2;
	int16_t		mb;
	int16_t		mc;
	int16_t		md;
} BMP085CalibrationData;

class BMP085 : public device::I2C
{
public:
	BMP085(int bus = BMP085_BUS, int address = BMP085_ADDRESS, const char *path = BARO_DEVICE_PATH);
	virtual ~BMP085();

	virtual int     init();

	virtual ssize_t   read(struct file *filp, char *buffer, size_t buflen);
	virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void        print_info();

protected:
	virtual int     probe();

private:
	BMP085OversamplingMode	_oversampling_mode;
	BMP085CalibrationData	_calibration;
	int16_t				_raw_temperature;
	int32_t				_raw_pressure;
	int32_t				_sampled_temperature;
	int32_t				_sampled_pressure;
	uint64_t			_temperature_last_read;
	unsigned			_msl_pressure;	/* in kPa */
	uint8_t				_sample_count;
	work_s				_work;
	unsigned			_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	baro_report			*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;

	orb_advert_t		_barometer_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Read in the calibration data from the device
	 * @return OK if calibration was read.
	 */
	int					read_calibration_data();

	/**
	 * Read the temperature from the device
	 * @return OK if the temperature was read
	 */
	int					read_temperature();

	/**
	 * Read the pressure from the device
	 * @return OK if the pressure was read
	 */
	int					read_pressure();

	/**
	 * Calculate the calibrated temperature and pressure.
	 */
	void				calculate();

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp085_main(int argc, char *argv[]);

BMP085::BMP085(int bus, int address, const char *path) :
	I2C("BMP085", path, bus, address, 100000),
	_oversampling_mode(kUltraHighResolution),
	_raw_temperature(0),
	_raw_pressure(0),
	_sampled_temperature(0),
	_sampled_pressure(0),
	_temperature_last_read(0),
	_msl_pressure(101325),
	_sample_count(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_barometer_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp085_read")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp085_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "bmp085x_buffer_overflows"))
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

BMP085::~BMP085()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
	{
		delete[] _reports;
	}
}

int
BMP085::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct baro_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the range finder topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_barometer_topic = orb_advertise(ORB_ID(sensor_baro), &_reports[0]);

	if (_barometer_topic < 0)
		debug("failed to create sensor_range_finder object. Did you start uOrb?");

	ret = read_calibration_data();
	if (ret == OK)
	{
		ret = read_temperature();
		if (ret == OK)
		{
			_temperature_last_read = hrt_absolute_time();
		}
	}
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;
out:
	return ret;
}

/* turn off type limits for probe() */
#pragma GCC diagnostic ignored "-Wtype-limits"
int
BMP085::probe()
{
	int ret = read_calibration_data();
	if (ret == OK)
	{
		if (!(_calibration.ac1 > 0 && _calibration.ac1 < 0xFFFF) &&
			 (_calibration.ac2 > 0 && _calibration.ac2 < 0xFFFF) &&
			 (_calibration.ac3 > 0 && _calibration.ac3 < 0xFFFF) &&
			 (_calibration.ac4 > 0 && _calibration.ac4 < 0xFFFF) &&
			 (_calibration.ac5 > 0 && _calibration.ac5 < 0xFFFF) &&
			 (_calibration.ac6 > 0 && _calibration.ac6 < 0xFFFF) &&
			 (_calibration.b1 > 0 && _calibration.b1 < 0xFFFF) &&
			 (_calibration.b2 > 0 && _calibration.b2 < 0xFFFF) &&
			 (_calibration.mb > 0 && _calibration.mb < 0xFFFF) &&
			 (_calibration.mc > 0 && _calibration.mc < 0xFFFF) &&
			 (_calibration.md > 0 && _calibration.md < 0xFFFF))
		{
			ret = -ENOLINK;
		}
	}

	return ret;
}

int
BMP085::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(BMP085_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(BMP085_CONVERSION_INTERVAL))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		} break;

	case SENSORIOCGPOLLRATE: {
		if (_measure_ticks == 0)
					return SENSOR_POLLRATE_MANUAL;

				return (1000 / _measure_ticks);
	}
	break;

	case SENSORIOCSQUEUEDEPTH: {
			/* add one to account for the sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct baro_report *buf = new struct baro_report[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _reports;
			_num_reports = arg;
			_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_reports - 1;

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case BAROIOCSMSLPRESSURE:
	{
		log("%d", arg);
		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000))
			log("Value out of bounds. %d must be between 80000 and 120000", arg);
			return -EINVAL;

		_msl_pressure = arg;
		return OK;
	}
	break;

	case BAROIOCGMSLPRESSURE:
	{
		return _msl_pressure;
	}
	break;
	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
BMP085::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_oldest_report != _next_report) {
				memcpy(buffer, _reports + _oldest_report, sizeof(*_reports));
				ret += sizeof(_reports[0]);
				INCREMENT(_oldest_report, _num_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_oldest_report = _next_report = 0;

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(BMP085_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		memcpy(buffer, _reports, sizeof(*_reports));
		ret = sizeof(*_reports);

	} while (0);

	return ret;
}

int
BMP085::read_calibration_data()
{
	uint8_t cmd = BMP085_REGISTER_CALIBRATION_DATA;
	uint8_t buf[22];

	int ret = transfer(&cmd, sizeof(cmd), buf, sizeof(buf));
	if (OK != ret)
	{
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
		return ret;
	}

	_calibration.ac1 = ((int16_t)buf[0] << 8) | buf[1];
	_calibration.ac2 = ((int16_t)buf[2] << 8) | buf[3];
	_calibration.ac3 = ((int16_t)buf[4] << 8) | buf[5];
	_calibration.ac4 = ((uint16_t)buf[6] << 8) | buf[7];
	_calibration.ac5 = ((uint16_t)buf[8] << 8) | buf[9];
	_calibration.ac6 = ((uint16_t)buf[10] << 8) | buf[11];
	_calibration.b1 = ((int16_t)buf[12] << 8) | buf[13];
	_calibration.b2 = ((int16_t)buf[14] << 8) | buf[15];
	_calibration.mb = ((int16_t)buf[16] << 8) | buf[17];
	_calibration.mc = ((int16_t)buf[18] << 8) | buf[19];
	_calibration.md = ((int16_t)buf[20] << 8) | buf[21];

	return ret;
}

int BMP085::read_temperature()
{
	uint8_t cmd[] = {BMP085_REGISTER_TAKE_TEMPERATURE, 0X2E};
	int ret = transfer(&cmd[0], sizeof(cmd), nullptr, 0);
	usleep(4500); /* 4.5 ms */

	uint8_t buf[2];
	uint8_t read_cmd = BMP085_REGISTER_READ_TEMPERATURE;
	ret = transfer(&read_cmd, sizeof(read_cmd), buf, sizeof(buf));
	_raw_temperature = ((int16_t)buf[0] << 8) | buf[1];

	return ret;
}

int BMP085::read_pressure()
{
	uint8_t cmd[2];
	cmd[0] = BMP085_REGISTER_TAKE_PRESSURE;
	cmd[1] = 0x34 + ((uint8_t)_oversampling_mode << 6);
	int ret = transfer(&cmd[0], sizeof(cmd), nullptr, 0);

	usleep(BMP085PressureWaitTimes[_oversampling_mode]);

	uint8_t buf[3];
	uint8_t read_cmd = BMP085_REGISTER_READ_PRESSURE;
	ret = transfer(&read_cmd, sizeof(read_cmd), buf, 3);

	_raw_pressure = (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2])) >> (8 - _oversampling_mode);

	return ret;
}

void
BMP085::calculate()
{
	/* This algorithm taken from ardupilot */
	int32_t x1, x2, x3, b3, b5, b6, tmp, p;
	uint32_t b4, b7;

	/* calc temperature */
	x1 = (((int32_t)_raw_temperature) - _calibration.ac6) * _calibration.ac5 >> 15;
	x2 = ((int32_t)_calibration.mc << 11) / (x1 + _calibration.md);
	b5 = x1 + x2;
	_sampled_temperature = (b5 + 8) >> 4;

	/* calc pressure */
	b6 = b5 - 4000;
	x1 = (_calibration.b2 * (b6	* (b6 >> 12))) >> 11;
	x2 = _calibration.ac2 * (b6 >> 11);
	x3 = x1 + x2;

	tmp = _calibration.ac3;
	tmp = ((tmp * 4) + x3) << _oversampling_mode;
	b3 = (tmp + 2) / 4;
	x1 = _calibration.ac3 * b6 >> 13;
	x2 = (_calibration.b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (_calibration.ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t)_raw_pressure - b3) * (50000 >> _oversampling_mode);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	_sampled_pressure = p + ((x1 + x2 + 3791) >> 4);
}

int
BMP085::measure()
{
	int ret = OK;
	uint64_t now = hrt_absolute_time();

	/* read temperature every second */
	if ((now - _temperature_last_read) > 1 * 1000 * 1000)
	{
		ret = read_temperature();
		if (ret != OK)
		{
			return ret;
		}
		_temperature_last_read = hrt_absolute_time();
	}

	ret = read_pressure();

	return ret;
}

int
BMP085::collect()
{
	int	ret = -EIO;

	perf_begin(_sample_perf);

	int32_t temperature = _sampled_temperature, pressure = _sampled_pressure;

	calculate();

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();
	_reports[_next_report].temperature = temperature / 10.0f;
	_reports[_next_report].pressure = pressure / 100.0f; /* Pa to millibar */

	_sampled_temperature = 0;
	_sampled_pressure = 0;

	/* calculate the altitude */
	/*
	 * PERFORMANCE HINT:
	 *
	 * The single precision calculation is 50 microseconds faster than the double
	 * precision variant. It is however not obvious if double precision is required.
	 * Pending more inspection and tests, we'll leave the double precision variant active.
	 *
	 * Measurements:
	 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
	 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
	 */
#if 0/* USE_FLOAT */
	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0f + 273.15f;	/* temperature at base height in Kelvin */
	const float a  = -6.5f / 1000f;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	float p1 = _msl_pressure / 1000.0f;

	/* measured pressure in kPa */
	float p = P / 1000.0f;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	_reports[_next_report].altitude = (((powf((p / p1), (-(a * R) / g))) * T1) - T1) / a;
#else
	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = _msl_pressure / 1000.0;

	/* measured pressure in Pa */
	double p = pressure * 1.0;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	_reports[_next_report].altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
#endif

	/* publish it */
	orb_publish(ORB_ID(sensor_baro), _barometer_topic, &_reports[_next_report]);

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_report, _num_reports);

	/* if we are running up against the oldest report, toss it */
	if (_next_report == _oldest_report) {
		perf_count(_buffer_overflows);
		INCREMENT(_oldest_report, _num_reports);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;
	goto out;

out:
	perf_end(_sample_perf);
	return ret;
}

void
BMP085::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BMP085::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_ABSPRESSURE};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
BMP085::stop()
{
	work_cancel(HPWORK, &_work);
}

void
BMP085::cycle_trampoline(void *arg)
{
	BMP085 *dev = (BMP085 *)arg;

	dev->cycle();
}

void
BMP085::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(BMP085_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&BMP085::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(BMP085_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");


	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&BMP085::cycle_trampoline,
		   this,
		   USEC2TICK(BMP085_CONVERSION_INTERVAL));
}

void
BMP085::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
}

namespace bmp085
{

BMP085	*g_dev;

void	start(const char *path);
void	stop();
void	reset();
void	test();
void	info();
void	calibrate(unsigned altitude);

/**
 * Start the driver.
 */
void
start(const char *path)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new BMP085(BMP085_BUS, BMP085_ADDRESS, path);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

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
	struct baro_report report;
	ssize_t sz;
	int ret;
	int fd = open(g_dev->device_name(), O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmp085 start' if the driver is not running)", g_dev->device_name());

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.pressure);
	warnx("altitude:    %11.4f", (double)report.altitude);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("pressure:    %10.4f", (double)report.pressure);
		warnx("altitude:    %11.4f", (double)report.altitude);
		warnx("temperature: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}
	close(fd);

	errx(0, "PASS");
}

/**
 * Stop the driver
 */
void
stop()
{
	if (g_dev != nullptr)
	{
		delete g_dev;
		g_dev = nullptr;
	}
	else
	{
		errx(1, "driver not running");
	}
	exit(0);
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(g_dev->device_name(), O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude)
{
	struct baro_report report;
	float	pressure;
	float	p1;

	int fd = open(g_dev->device_name(), O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmp085 start' if the driver is not running)", g_dev->device_name());

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX))
		errx(1, "failed to set poll rate");

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "sensor read failed");

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK)
		err(1, "BAROIOCSMSLPRESSURE");
	close(fd);

	exit(0);
}

} // namespace

int
bmp085_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
	{
		/* see if there is already a barometer around */
		int fd = open(BARO_DEVICE_PATH, O_RDONLY);
		int8_t i = -1;
		char *dev = NULL;

		if (fd >= 0)
		{
			// /dev/baro exists
			while (fd >= 0)
			{
				i++;
				close(fd);
				asprintf(&dev, "%s%d", BARO_DEVICE_PATH, i);
				fd = open(dev, O_RDONLY);
				free(dev);
			}
			asprintf(&dev, "%s%d", BARO_DEVICE_PATH, i);
		}
		else
		{
			asprintf(&dev, "%s", BARO_DEVICE_PATH);
		}

		bmp085::start(dev);
		//free(dev); // we don't free this as CDev doesn't copy the device name
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop"))
	{
		bmp085::stop();
	}


	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
	{
		bmp085::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
	{
		bmp085::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
	{
		bmp085::info();
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(argv[1], "calibrate")) {
		if (argc < 2)
			errx(1, "missing altitude");

		long altitude = strtol(argv[2], nullptr, 10);

		bmp085::calibrate(altitude);
	}

	errx(1, "unrecognised command, try 'start', 'stop', 'calibrate', 'test', 'reset' or 'info'");
}

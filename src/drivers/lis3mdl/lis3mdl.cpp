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
 * @file lis3mdl.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 *
 * Based on the hmc5883 driver.
 */

#include <px4_config.h>
#include <px4_defines.h>

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

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

#include "lis3mdl.h"

/*
 * LIS3MDL internal constants and data structures.
 */

/* Max measurement rate is 80Hz */
#define LIS3MDL_CONVERSION_INTERVAL	(1000000 / 80)	/* 12,500 microseconds */


#define ADDR_CTRL_REG1		0x20
#define ADDR_CTRL_REG2		0x21
#define ADDR_CTRL_REG3		0x22
#define ADDR_CTRL_REG4		0x23
#define ADDR_CTRL_REG5		0x24

#define ADDR_STATUS_REG		0x27
#define ADDR_OUT_X_L		0x28
#define ADDR_OUT_X_H		0x29
#define ADDR_OUT_Y_L		0x2a
#define ADDR_OUT_Y_H		0x2b
#define ADDR_OUT_Z_L		0x2c
#define ADDR_OUT_Z_H		0x2d
#define ADDR_OUT_T_L		0x2e
#define ADDR_OUT_T_H		0x2f

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

enum LIS3MDL_BUS {
	LIS3MDL_BUS_ALL = 0,
	LIS3MDL_BUS_I2C_INTERNAL,
	LIS3MDL_BUS_I2C_EXTERNAL,
	LIS3MDL_BUS_SPI
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class LIS3MDL : public device::CDev
{
public:
	LIS3MDL(device::Device *interface, const char *path, enum Rotation rotation);
	virtual ~LIS3MDL();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

private:
	work_s			_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer	*_reports;
	struct mag_calibration_s	_scale;
	float 			_range_scale;
	float 			_range_ga;
	bool			_collect_phase;
	int			_class_instance;
	int			_orb_class_instance;

	orb_advert_t		_mag_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	/* status reporting */
	bool			_sensor_ok;		/**< sensor was found and reports ok */
	bool			_calibrated;		/**< the calibration is valid */

	enum Rotation		_rotation;

	struct mag_report	_last_report;           /**< used for info() */

	uint8_t			_range_bits;
	uint8_t			_cntl_reg1;
	uint8_t			_cntl_reg4;
	uint8_t			_cntl_reg5;
	uint8_t			_temperature_counter;
	uint8_t			_temperature_error_count;
	uint8_t			_check_state_cnt;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(struct file *filp, unsigned enable);

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int			set_excitement(unsigned enable);

	/**
	 * Set the sensor range.
	 *
	 * Sets the internal range to handle at least the argument in Gauss.
	 */
	int 			set_range(unsigned range);

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf(void);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	int 			check_calibration();

	/**
	* Check the current scale calibration
	*
	* @return 0 if scale calibration is ok, 1 else
	*/
	int 			check_scale();

	/**
	* Check the current offset calibration
	*
	* @return 0 if offset calibration is ok, 1 else
	*/
	int 			check_offset();

	/* this class has pointer data members, do not allow copying it */
	LIS3MDL(const LIS3MDL &);
	LIS3MDL operator=(const LIS3MDL &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int lis3mdl_main(int argc, char *argv[]);


LIS3MDL::LIS3MDL(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("LIS3MDL", path),
	_interface(interface),
	_work{},
	_measure_ticks(0),
	_reports(nullptr),
	_scale{},
	_range_scale(0), /* default range scale from counts to gauss */
	_range_ga(4.0f),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "lis3mdl_read")),
	_comms_errors(perf_alloc(PC_COUNT, "lis3mdl_comms_errors")),
	_range_errors(perf_alloc(PC_COUNT, "lis3mdl_range_errors")),
	_conf_errors(perf_alloc(PC_COUNT, "lis3mdl_conf_errors")),
	_sensor_ok(false),
	_calibrated(false),
	_rotation(rotation),
	_last_report{0},
	_range_bits(0),
	_cntl_reg1(0),
	_cntl_reg4(0),
	_cntl_reg5(0),
	_temperature_counter(0),
	_temperature_error_count(0),
	_check_state_cnt(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LIS3MDL;

	// enable debug() calls
	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	_cntl_reg1 = 0xFC;
	_cntl_reg4 = 0x0C; // Z-axis ultra high performance mode
	_cntl_reg5 = 0x40; // block data update for magnetic data

}

LIS3MDL::~LIS3MDL()
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
LIS3MDL::init()
{
	int ret = PX4_ERROR;

	ret = CDev::init();

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
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}

int LIS3MDL::set_range(unsigned range)
{
	if (range < 6) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 6842.0f;
		_range_ga = 4.0f;

	} else if (range <= 10) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 3421.0f;
		_range_ga = 8.0f;

	} else if (range <= 14) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 2281.0f;
		_range_ga = 12.0f;

	} else {
		_range_bits = 0x03;
		_range_scale = 1.0f / 1711.0f;
		_range_ga = 16.0f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CTRL_REG2, (_range_bits << 5));

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CTRL_REG2, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return !(range_bits_in == (_range_bits << 5));
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void LIS3MDL::check_conf(void)
{
	int ret;
	uint8_t reg_in = 0;

	switch (_check_state_cnt++) {
	case 0:
		ret = read_reg(ADDR_CTRL_REG1, reg_in);

		if (OK != ret) {
			perf_count(_comms_errors);
			return;
		}

		if (reg_in != _cntl_reg1) {
			perf_count(_conf_errors);
			ret = write_reg(ADDR_CTRL_REG1, _cntl_reg1);

			if (OK != ret) {
				perf_count(_comms_errors);
			}
		}

		break;

	case 50:
		ret = read_reg(ADDR_CTRL_REG4, reg_in);

		if (OK != ret) {
			perf_count(_comms_errors);
			return;
		}

		if (reg_in != _cntl_reg4) {
			perf_count(_conf_errors);
			ret = write_reg(ADDR_CTRL_REG4, _cntl_reg4);

			if (OK != ret) {
				perf_count(_comms_errors);
			}
		}

		break;

	case 100:
		ret = read_reg(ADDR_CTRL_REG5, reg_in);

		if (OK != ret) {
			perf_count(_comms_errors);
			return;
		}

		if (reg_in != _cntl_reg5) {
			perf_count(_conf_errors);
			ret = write_reg(ADDR_CTRL_REG5, _cntl_reg5);

			if (OK != ret) {
				perf_count(_comms_errors);
			}
		}

		break;

	case 150:
		ret = read_reg(ADDR_CTRL_REG2, reg_in);

		if (OK != ret) {
			perf_count(_comms_errors);
			return;
		}

		if (reg_in != (_range_bits << 5)) {
			perf_count(_range_errors);
			ret = write_reg(ADDR_CTRL_REG2, (_range_bits << 5));

			if (OK != ret) {
				perf_count(_comms_errors);
			}
		}

		break;

	default:
		break;
	}
}

ssize_t
LIS3MDL::read(struct file *filp, char *buffer, size_t buflen)
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
		usleep(LIS3MDL_CONVERSION_INTERVAL);

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
LIS3MDL::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

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
					_measure_ticks = USEC2TICK(LIS3MDL_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(LIS3MDL_CONVERSION_INTERVAL)) {
// RobD: quick fix for Phil's testing						return -EINVAL;
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

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000 / TICK2USEC(_measure_ticks);

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCGRANGE:
		return _range_ga;

	case MAGIOCSLOWPASS:
	case MAGIOCGLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

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

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

	case MAGIOCSELFTEST:
		return check_calibration();

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	case DEVIOCGDEVICEID:
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
LIS3MDL::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LIS3MDL::cycle_trampoline, this, 1);
}

void
LIS3MDL::stop()
{
	if (_measure_ticks > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_ticks = 0;
		work_cancel(HPWORK, &_work);
	}
}

int
LIS3MDL::reset()
{
	/* set range */
	return set_range(_range_ga);
}

void
LIS3MDL::cycle_trampoline(void *arg)
{
	LIS3MDL *dev = (LIS3MDL *)arg;

	dev->cycle();
}

void
LIS3MDL::cycle()
{
	if (_measure_ticks == 0) {
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
		if (_measure_ticks > USEC2TICK(LIS3MDL_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&LIS3MDL::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(LIS3MDL_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_ticks > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&LIS3MDL::cycle_trampoline,
			   this,
			   USEC2TICK(LIS3MDL_CONVERSION_INTERVAL));
	}
}

int
LIS3MDL::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_CTRL_REG3, MODE_REG_SINGLE_MODE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
LIS3MDL::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		y[2];
		uint8_t		z[2];
		uint8_t		t[2];
	}	lis_report;

	struct {
		int16_t		x;
		int16_t		y;
		int16_t		z;
		int16_t		t;
	} report;
#pragma pack(pop)

	int	ret;
//	uint8_t check_counter;

	perf_begin(_sample_perf);
	struct mag_report new_report;
	bool sensor_is_onboard = false;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	ret = _interface->read(ADDR_OUT_X_L, (uint8_t *)&lis_report, sizeof(lis_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/* convert the data we just received */
	report.x = (((int16_t)lis_report.x[1]) << 8) + lis_report.x[0];
	report.y = (((int16_t)lis_report.y[1]) << 8) + lis_report.y[0];
	report.z = (((int16_t)lis_report.z[1]) << 8) + lis_report.z[0];
	report.t = (((int16_t)lis_report.t[1]) << 8) + lis_report.t[0];

	/* get measurements from the device */
	new_report.temperature = report.t;
	new_report.temperature = 25 + (report.t / (16 * 8.0f));

	// XXX revisit for SPI part, might require a bus type IOCTL

	unsigned dummy;
	sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);

	/*
	 * RAW outputs
	 *
	 */
	new_report.x_raw = report.x;
	new_report.y_raw = report.y;
	new_report.z_raw = report.z;

	/* the LIS3MDL mag on Pixhawk Pro by Drotek has x pointing towards,
	 * y pointing to the right, and z down, therefore no switch needed,
	 * it is better to have no artificial rotation inside the
	 * driver and then use the startup script with -R command with the
	 * real rotation between the sensor and body frame */
	xraw_f = report.x;
	yraw_f = report.y;
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

	check_conf();

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int LIS3MDL::calibrate(struct file *filp, unsigned enable)
{
	struct mag_report report;
	ssize_t sz;
	int ret = 1;

	// XXX do something smarter here
	int fd = (int)enable;

	float sum_excited[3] = {0.0f, 0.0f, 0.0f};
	float sum_non_excited[3] = {0.0f, 0.0f, 0.0f};

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(filp, SENSORIOCSPOLLRATE, 50)) {
		warn("FAILED: SENSORIOCSPOLLRATE 50Hz");
		ret = 1;
		goto out;
	}

	/* Set to 4 Gauss */
	if (OK != ioctl(filp, MAGIOCSRANGE, 12)) {
		warnx("FAILED: MAGIOCSRANGE 12 Ga");
		ret = 1;
		goto out;
	}

	usleep(20000);

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 10x */
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		sum_non_excited[0] += report.x;
		sum_non_excited[1] += report.y;
		sum_non_excited[2] += report.z;
	}

	sum_non_excited[0] /= 10.0f;
	sum_non_excited[1] /= 10.0f;
	sum_non_excited[2] /= 10.0f;


	if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
		warnx("FAILED: MAGIOCEXSTRAP 1");
		ret = 1;
		goto out;
	}

	usleep(60000);

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 10x */
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		sum_excited[0] += report.x;
		sum_excited[1] += report.y;
		sum_excited[2] += report.z;
	}

	sum_excited[0] /= 10.0f;
	sum_excited[1] /= 10.0f;
	sum_excited[2] /= 10.0f;

	if (1.0f < fabsf(sum_excited[0] - sum_non_excited[0]) && fabsf(sum_excited[0] - sum_non_excited[0]) < 3.0f &&
	    1.0f < fabsf(sum_excited[1] - sum_non_excited[1]) && fabsf(sum_excited[1] - sum_non_excited[1]) < 3.0f &&
	    0.1f < fabsf(sum_excited[2] - sum_non_excited[2]) && fabsf(sum_excited[2] - sum_non_excited[2]) < 1.0f) {
		ret = OK;

	} else {
		ret = -EIO;
		goto out;
	}

	ret = OK;

out:

	/* set back to normal mode */
	/* Set to 4 Gauss */
	if (OK != ::ioctl(fd, MAGIOCSRANGE, 4)) {
		warnx("FAILED: MAGIOCSRANGE 4 Ga");
	}

	if (OK != ::ioctl(fd, MAGIOCEXSTRAP, 0)) {
		warnx("FAILED: MAGIOCEXSTRAP 0");
	}

	usleep(20000);

	return ret;
}

int LIS3MDL::check_scale()
{
	bool scale_valid;

	if ((-FLT_EPSILON + 1.0f < _scale.x_scale && _scale.x_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.y_scale && _scale.y_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.z_scale && _scale.z_scale < FLT_EPSILON + 1.0f)) {
		/* scale is one */
		scale_valid = false;

	} else {
		scale_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !scale_valid;
}

int LIS3MDL::check_offset()
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

int LIS3MDL::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		warnx("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
		      (offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);
	}

	/* return 0 if calibrated, 1 else */
	return !_calibrated;
}

int LIS3MDL::set_excitement(unsigned enable)
{
	int ret;
	/* arm the excitement strap */
	ret = read_reg(ADDR_CTRL_REG1, _cntl_reg1);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	_cntl_reg1 &= ~0x01; // reset previous excitement mode

	if (((int)enable) < 0) {
		warnx("WARN: set_excitement negative not supported\n");

	} else if (enable > 0) {
		_cntl_reg1 |= 0x01;
	}

	::printf("set_excitement enable=%d cntl1=0x%x\n", (int)enable, (unsigned)_cntl_reg1);

	ret = write_reg(ADDR_CTRL_REG1, _cntl_reg1);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t conf_reg_ret = 0;
	read_reg(ADDR_CTRL_REG1, conf_reg_ret);

	//print_info();

	return !(_cntl_reg1 == conf_reg_ret);
}

int
LIS3MDL::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
LIS3MDL::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
LIS3MDL::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
LIS3MDL::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.2f %.2f %.2f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f range_ga %.2f\n",
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)(1.0f / _range_scale), (double)_range_ga);
	printf("temperature %.2f\n", (double)_last_report.temperature);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace lis3mdl
{

/*
  list of supported bus configurations
 */
struct lis3mdl_bus_option {
	enum LIS3MDL_BUS busid;
	const char *devpath;
	LIS3MDL_constructor interface_constructor;
	uint8_t busnum;
	LIS3MDL	*dev;
} bus_options[] = {
	{ LIS3MDL_BUS_I2C_EXTERNAL, "/dev/lis3mdl_ext", &LIS3MDL_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_ONBOARD
	{ LIS3MDL_BUS_I2C_INTERNAL, "/dev/lis3mdl_int", &LIS3MDL_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_LIS
	{ LIS3MDL_BUS_SPI, "/dev/lis3mdl_spi", &LIS3MDL_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void	start(enum LIS3MDL_BUS busid, enum Rotation rotation);
int		stop();
bool	start_bus(struct lis3mdl_bus_option &bus, enum Rotation rotation);
struct lis3mdl_bus_option &find_bus(enum LIS3MDL_BUS busid);
void	test(enum LIS3MDL_BUS busid);
void	reset(enum LIS3MDL_BUS busid);
int	info(enum LIS3MDL_BUS busid);
int	calibrate(enum LIS3MDL_BUS busid);
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct lis3mdl_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new LIS3MDL(interface, bus.devpath, rotation);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
	}

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 50)) {
		warn("FAILED: SENSORIOCSPOLLRATE 50Hz");
	}

	printf("set poll rate to 50Hz\n");

	/* Set to 4 Gauss */
	if (OK != ioctl(fd, MAGIOCSRANGE, 4)) {
		warnx("FAILED: MAGIOCSRANGE 4 Ga");
	}

	printf("set range to 4 Ga\n");

	close(fd);

	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum LIS3MDL_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == LIS3MDL_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != LIS3MDL_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
	}

	if (!started) {
		exit(1);
	}
}

/**
 * Stop the driver.
 */
int
stop()
{
	bool stopped = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			bus_options[i].dev->stop();
			delete bus_options[i].dev;
			bus_options[i].dev = nullptr;
			stopped = true;
		}
	}

	return !stopped;
}

/**
 * find a bus structure for a busid
 */
struct lis3mdl_bus_option &find_bus(enum LIS3MDL_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == LIS3MDL_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum LIS3MDL_BUS busid)
{
	struct lis3mdl_bus_option &bus = find_bus(busid);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'lis3mdl start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	warnx("time:        %lld", report.timestamp);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external");
	}

	warnx("device active: %s", ret ? "external" : "onboard");

	/* set the queue depth to 5 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}


/**
 * Self test check.
 *
 * Unlike HMC5883, self test feature cannot be used to calculate
 * scale.
 *
 * SELF TEST OPERATION
 * To check the LIS3MDL for proper operation, a self test feature is incorporated :
 * sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias).
 * A few measurements are taken and stored with and without the additional magnetic
 * field. According to ST datasheet, those values must stay between thresholds in order
 * to pass the self test.
 */
int calibrate(enum LIS3MDL_BUS busid)
{
	int ret;
	struct lis3mdl_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'lis3mdl start' if the driver is not running", path);
	}

	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		warnx("failed to enable sensor calibration mode");
	}

	close(fd);

	return ret;
}

/**
 * Reset the driver.
 */
void
reset(enum LIS3MDL_BUS busid)
{
	struct lis3mdl_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
int
info(enum LIS3MDL_BUS busid)
{
	struct lis3mdl_bus_option &bus = find_bus(busid);

	warnx("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'info', 'calibrate'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -C calibrate on start");
	warnx("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
lis3mdl_main(int argc, char *argv[])
{
	int ch;
	enum LIS3MDL_BUS busid = LIS3MDL_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;

	while ((ch = getopt(argc, argv, "XISR:CT")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)

		case 'I':
			busid = LIS3MDL_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = LIS3MDL_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = LIS3MDL_BUS_SPI;
			break;

		case 'C':
			calibrate = true;
			break;

		default:
			lis3mdl::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		lis3mdl::start(busid, rotation);

		if (calibrate && lis3mdl::calibrate(busid) != 0) {
			errx(1, "calibration failed");
		}

		exit(0);
	}

	/*
	 * Stop the driver.
	 */
	if (!strcmp(verb, "stop")) {
		return lis3mdl::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		lis3mdl::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		lis3mdl::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		lis3mdl::info(busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (lis3mdl::calibrate(busid) == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'calibrate' 'or 'info'");
}

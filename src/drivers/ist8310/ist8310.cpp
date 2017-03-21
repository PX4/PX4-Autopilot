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
 * @file ist8310.cpp
 *
 * Driver for the IST8310 magnetometer connected via I2C.
 *
 * @author David Sidrane
 * @author Maelok Dong
 */

#include <px4_config.h>

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

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

/*
 * IST8310 internal constants and data structures.
 */

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150
 * The datasheet gives 200Hz maximum measurement rate, but it's not true according to tech support from iSentek*/
#define IST8310_CONVERSION_INTERVAL (1000000 / 150) /* microseconds */

#define IST8310_BUS_I2C_ADDR     	0xE
#define IST8310_DEFAULT_BUS_SPEED 	400000

/* compensation matrix scalar */
#define IST8310_COMPENSATION_MATRIX_M	3/20.0f

/* Cross-axis calibration sensitivity and bit shift setting */
#define OTPsensitivity 			(330)
#define CROSSAXISINV_BITSHIFT	(16)

/* Use float calculation */
#define IST_CROSS_AXIS_CALI_FLOAT

/* Hardware definitions */

#define ADDR_WAI                0		/* WAI means 'Who Am I'*/
# define WAI_EXPECTED_VALUE     0x10

#define ADDR_STAT1              0x02
# define STAT1_DRDY_SHFITS      0x0
# define STAT1_DRDY             (1 << STAT1_DRDY_SHFITS)
# define STAT1_DRO_SHFITS       0x1
# define STAT1_DRO              (1 << STAT1_DRO_SHFITS)

#define ADDR_DATA_OUT_X_LSB     0x03
#define ADDR_DATA_OUT_X_MSB     0x04
#define ADDR_DATA_OUT_Y_LSB     0x05
#define ADDR_DATA_OUT_Y_MSB     0x06
#define ADDR_DATA_OUT_Z_LSB     0x07
#define ADDR_DATA_OUT_Z_MSB     0x08

#define ADDR_STAT2              0x09
# define STAT2_INT_SHFITS       3
# define STAT2_INT              (1 << STAT2_INT_SHFITS)

#define ADDR_CTRL1              0x0a
# define CTRL1_MODE_SHFITS      0
# define CTRL1_MODE_STDBY       (0 << CTRL1_MODE_SHFITS)
# define CTRL1_MODE_SINGLE      (1 << CTRL1_MODE_SHFITS)

#define ADDR_CTRL2              0x0b
# define CTRL2_SRST_SHFITS      0   /* Begin POR (auto cleared) */
# define CTRL2_SRST             (1 << CTRL2_SRST_SHFITS)
# define CTRL2_DRP_SHIFTS       2
# define CTRL2_DRP              (1 << CTRL2_DRP_SHIFTS)
# define CTRL2_DREN_SHIFTS      3
# define CTRL2_DREN             (1 << CTRL2_DREN_SHIFTS)

#define ADDR_CTRL3				0x41
# define CTRL3_SAMPLEAVG_16		0x24	/* Sample Averaging 16 */
# define CTRL3_SAMPLEAVG_8		0x1b	/* Sample Averaging 8 */
# define CTRL3_SAMPLEAVG_4		0x12	/* Sample Averaging 4 */
# define CTRL3_SAMPLEAVG_2		0x09	/* Sample Averaging 2 */

#define ADDR_CTRL4				0x42
# define CTRL4_SRPD				0xC0	/* Set Reset Pulse Duration */

#define ADDR_STR                0x0c
# define STR_SELF_TEST_SHFITS   6
# define STR_SELF_TEST_ON       (1 << STR_SELF_TEST_SHFITS)
# define STR_SELF_TEST_OFF      (0 << STR_SELF_TEST_SHFITS)

#define ADDR_Y11_Low			0x9c
#define ADDR_Y11_High			0x9d
#define ADDR_Y12_Low			0x9e
#define ADDR_Y12_High			0x9f
#define ADDR_Y13_Low			0xa0
#define ADDR_Y13_High			0xa1
#define ADDR_Y21_Low			0xa2
#define ADDR_Y21_High			0xa3
#define ADDR_Y22_Low			0xa4
#define ADDR_Y22_High			0xa5
#define ADDR_Y23_Low			0xa6
#define ADDR_Y23_High			0xa7
#define ADDR_Y31_Low			0xa8
#define ADDR_Y31_High			0xa9
#define ADDR_Y32_Low			0xaa
#define ADDR_Y32_High			0xab
#define ADDR_Y33_Low			0xac
#define ADDR_Y33_High			0xad

#define ADDR_TEMPL              0x1c
#define ADDR_TEMPH              0x1d

enum IST8310_BUS {
	IST8310_BUS_ALL           = 0,
	IST8310_BUS_I2C_EXTERNAL = 1,
	IST8310_BUS_I2C_INTERNAL = 2,
};

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class IST8310 : public device::I2C
{
public:
	IST8310(int bus_number, int address, const char *path, enum Rotation rotation);
	virtual ~IST8310();

	virtual int     init();

	virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	virtual int         ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void            print_info();

protected:
	virtual int probe();

private:
	work_s          _work;
	unsigned        _measure_ticks;

	ringbuffer::RingBuffer  *_reports;
	struct mag_calibration_s    _scale;
	float           _range_scale;
	bool        _collect_phase;
	int         _class_instance;
	int         _orb_class_instance;

	orb_advert_t        _mag_topic;

	perf_counter_t      _sample_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _range_errors;
	perf_counter_t      _conf_errors;

	/* status reporting */
	bool            _sensor_ok;         /**< sensor was found and reports ok */
	bool            _calibrated;        /**< the calibration is valid */
	bool			_ctl_reg_mismatch;	/**< control register value mismatch after checking */

	enum Rotation       _rotation;

	struct mag_report   _last_report;           /**< used for info() */

	uint8_t 		_ctl3_reg;
	uint8_t			_ctl4_reg;


#ifdef IST_CROSS_AXIS_CALI_FLOAT
	float crossaxis_inv[9];
#else
	int64_t crossaxis_inv[9];
#endif

	int32_t crossaxis_det[1];

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void		start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void        stop();

	/**
	 * Reset the device
	 */
	int         reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *   will however reflect the uncalibrated sensor state until
	 *   the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int         calibrate(struct file *filp, unsigned enable);

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void            check_conf(void);

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
	void            cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg       Instance pointer for the driver that is polling.
	 */
	static void     cycle_trampoline(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg       The register to write.
	 * @param val       The value to write.
	 * @return      OK on write success.
	 */
	int         write_reg(uint8_t reg, uint8_t val);

	/**
	 * Write to a register block.
	 *
	 * @param address   The register address to write to.
	 * @param data      The buffer to write from.
	 * @param count     The number of bytes to write.
	 * @return      OK on write success.
	 */
	int     write(unsigned address, void *data, unsigned count);

	/**
	 * Read a register.
	 *
	 * @param reg       The register to read.
	 * @param val       The value read.
	 * @return      OK on read success.
	 */
	int         read_reg(uint8_t reg, uint8_t &val);

	/**
	 * read register block.
	 *
	 * @param address   The register address to read from.
	 * @param data      The buffer to read into.
	 * @param count     The number of bytes to read.
	 * @return      OK on write success.
	 */
	int read(unsigned address, void *data, unsigned count);

	/**
	 * Issue a measurement command.
	 *
	 * @return      OK if the measurement command was successful.
	 */
	int         measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int         collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in        A signed 16-bit big-endian value.
	 * @return      The floating-point representation of the value.
	 */
	float       meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	int         check_calibration();

	/**
	* Check the current scale calibration
	*
	* @return 0 if scale calibration is ok, 1 else
	*/
	int         check_scale();

	/**
	* Check the current offset calibration
	*
	* @return 0 if offset calibration is ok, 1 else
	*/
	int         check_offset();

	/**
	* Place the device in self test mode
	*
	* @return 0 if mode is set, 1 else
	*/
	int         set_selftest(unsigned enable);

	/**
	 * Initiate cross-axis matrix
	 *
	 * @param bitshift		The bit shift configuration for different chip, used when using integer calculation
	 * @param enable		Enable cross-axis compensation
	 * @return				Initialization succeed
	 */
	bool		crossaxis_matrix_init(int bitshift, int enable);

	/**
	 * Transfer the raw sensor data using cross-axis matrix
	 *
	 * @param xyz		The pointer of the sensor buffer
	 */
	void		crossaxis_transformation(int16_t *xyz);

	/* this class has pointer data members, do not allow copying it */
	IST8310(const IST8310 &);
	IST8310 operator=(const IST8310 &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ist8310_main(int argc, char *argv[]);


IST8310::IST8310(int bus_number, int address, const char *path, enum Rotation rotation) :
	I2C("IST8310", path, bus_number, address, IST8310_DEFAULT_BUS_SPEED),
	_work{},
	_measure_ticks(0),
	_reports(nullptr),
	_scale{},
	_range_scale(0.003), /* default range scale from counts to gauss */
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "ist8310_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ist8310_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "ist8310_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "ist8310_conf_err")),
	_sensor_ok(false),
	_calibrated(false),
	_ctl_reg_mismatch(false),
	_rotation(rotation),
	_last_report{0},
	_ctl3_reg(0),
	_ctl4_reg(0)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_IST8310;

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
}

IST8310::~IST8310()
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
IST8310::init()
{
	int ret = ERROR;

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
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}

int
IST8310::write(unsigned address, void *data, unsigned count)
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
IST8310::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}


/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void IST8310::check_conf(void)
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

	ret = read_reg(ADDR_CTRL4, ctrl_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (ctrl_reg_in != _ctl4_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CTRL4, _ctl4_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}

		_ctl_reg_mismatch = true;
	}

	_ctl_reg_mismatch = false;
}

ssize_t
IST8310::read(struct file *filp, char *buffer, size_t buflen)
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
		usleep(IST8310_CONVERSION_INTERVAL);

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
IST8310::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(IST8310_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(IST8310_CONVERSION_INTERVAL)) {
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
		return OK;

	case MAGIOCGRANGE:
		return 0;

	case MAGIOCEXSTRAP:
		return set_selftest(arg);


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

	case MAGIOCSELFTEST:
		return check_calibration();

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return 1;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
IST8310::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&IST8310::cycle_trampoline, this, 1);
}

void
IST8310::stop()
{
	work_cancel(HPWORK, &_work);
}

int
IST8310::reset()
{
	/* software reset */
	write_reg(ADDR_CTRL2, CTRL2_SRST);

	/* configure control register 3 */
	_ctl3_reg = CTRL3_SAMPLEAVG_16;
	write_reg(ADDR_CTRL3, _ctl3_reg);

	/* configure control register 4 */
	_ctl4_reg = CTRL4_SRPD;
	write_reg(ADDR_CTRL4, _ctl4_reg);

	/* initiate cross-axis matrix */
//    uint8_t wbuffer1[2] = {0};
//    uint8_t wbuffer2[2] = {0};
//    uint8_t try_times = 0;
//    while (++try_times <= 10) {
//		if (read(ADDR_Y11_Low, wbuffer1, 2) == OK) {
//			if (read(ADDR_Y11_Low, wbuffer2, 2) == OK) {
//				if ((wbuffer1[0] == wbuffer2[0]) && (wbuffer1[1] == wbuffer2[1])) {
//					int crossaxis_enable = 0;
//					uint8_t cross_mask = 0xFF;
//					if ((wbuffer1[0] == cross_mask) && (wbuffer1[1] == cross_mask))	crossaxis_enable = 0;
//					else												            crossaxis_enable = 1;
//
//					if (crossaxis_matrix_init(CROSSAXISINV_BITSHIFT, crossaxis_enable)) {
//						return OK;
//					}
//				}
//			}
//		}
//    }

	return OK;
}

void
IST8310::cycle_trampoline(void *arg)
{
	IST8310 *dev = (IST8310 *)arg;

	dev->cycle();
}

int
IST8310::probe()
{
	uint8_t data[1] = {0};

	_retries = 10;

	if (read(ADDR_WAI, &data[0], 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if ((data[0] != WAI_EXPECTED_VALUE)) {
		DEVICE_DEBUG("ID byte mismatch (%02x) expected %02x", data[0], WAI_EXPECTED_VALUE);
		return -EIO;
	}

	return OK;
}

void
IST8310::cycle()
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
		if (_measure_ticks > USEC2TICK(IST8310_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&IST8310::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(IST8310_CONVERSION_INTERVAL));

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
		   (worker_t)&IST8310::cycle_trampoline,
		   this,
		   USEC2TICK(IST8310_CONVERSION_INTERVAL));
}

int
IST8310::measure()
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
IST8310::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t     x[2];
		uint8_t     y[2];
		uint8_t     z[2];
	} report_buffer;
#pragma pack(pop)
	struct {
		int16_t     x, y, z;
	} report;

	int ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	struct mag_report new_report;
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
	ret = read(ADDR_DATA_OUT_X_LSB, (uint8_t *)&report_buffer, sizeof(report_buffer));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)report_buffer.x[1]) << 8) | (int16_t)report_buffer.x[0];
	report.y = (((int16_t)report_buffer.y[1]) << 8) | (int16_t)report_buffer.y[0];
	report.z = (((int16_t)report_buffer.z[1]) << 8) | (int16_t)report_buffer.z[0];

	/* perform cross-axis compensation */
//	crossaxis_transformation((int16_t *)&report);

	/* temperature measurement is not available on IST8310 */
	new_report.temperature = 0;

	/*
	 * raw outputs
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

int IST8310::calibrate(struct file *filp, unsigned enable)
{
	struct mag_report report;
	ssize_t sz;
	int ret = 1;
	float total_x;
	float total_y;
	float total_z;

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


		for (uint8_t i = 0; i < 60; i++) {


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

	total_x = fabs(sum_in_test[0]) - fabs(sum_in_normal[0]);
	total_y = fabs(sum_in_test[1]) - fabs(sum_in_normal[1]);
	total_z = fabs(sum_in_test[2]) - fabs(sum_in_normal[2]);

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
			ret = ERROR;
		}

	}

	return ret;
}

int IST8310::check_scale()
{
	return OK;
}

int IST8310::check_offset()
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

int IST8310::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		PX4_WARN("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
			 (offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);
	}

	/* return 0 if calibrated, 1 else */
	return (!_calibrated);
}

int
IST8310::set_selftest(unsigned enable)
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

bool
IST8310::crossaxis_matrix_init(int bitshift, int enable)
{
	int i = 0;
	uint8_t crossxbuf[6];
	uint8_t crossybuf[6];
	uint8_t crosszbuf[6];
	uint8_t crossxbuf2[6];
	uint8_t crossybuf2[6];
	uint8_t crosszbuf2[6];
	int16_t OTPcrossaxis[9] = {0};
#ifdef IST_CROSS_AXIS_CALI_FLOAT
	float inv[9] = {0};
#else
	int64_t inv[9] = {0};
#endif

	if (enable == 0) {
#ifdef IST_CROSS_AXIS_CALI_FLOAT
		*crossaxis_inv = 1;
#else
		*crossaxis_inv = (1 << bitshift);
#endif
		*(crossaxis_inv + 1) = 0;
		*(crossaxis_inv + 2) = 0;
		*(crossaxis_inv + 3) = 0;
#ifdef IST_CROSS_AXIS_CALI_FLOAT
		*(crossaxis_inv + 4) = 1;
#else
		*(crossaxis_inv + 4) = (1 << bitshift);
#endif
		*(crossaxis_inv + 5) = 0;
		*(crossaxis_inv + 6) = 0;
		*(crossaxis_inv + 7) = 0;
#ifdef IST_CROSS_AXIS_CALI_FLOAT
		*(crossaxis_inv + 8) = 1;
#else
		*(crossaxis_inv + 8) = (1 << bitshift);
#endif
		*crossaxis_det = 1;

		return true;

	} else {
		bool pass_all = false;
		uint8_t retry_times = 0;

		do {
			pass_all = true;

			if (read(ADDR_Y11_Low, crossxbuf, 6) && read(ADDR_Y21_Low, crossybuf, 6) && read(ADDR_Y31_Low, crosszbuf, 6)) {
				if (read(ADDR_Y11_Low, crossxbuf2, 6) && read(ADDR_Y21_Low, crossybuf2, 6) && read(ADDR_Y31_Low, crosszbuf2, 6)) {
					for (i = 0; i < 6; ++i) {
						if (crossxbuf[i] != crossxbuf2[i]) {
							pass_all = false;
						}
					}
				}
			}

			if (pass_all) { break; }
		} while (++retry_times < 5);

		if (retry_times >= 5 && !pass_all) { return false; }

		OTPcrossaxis[0] = ((int16_t) crossxbuf[1]) << 8 | ((int16_t) crossxbuf[0]);
		OTPcrossaxis[3] = ((int16_t) crossxbuf[3]) << 8 | ((int16_t) crossxbuf[2]);
		OTPcrossaxis[6] = ((int16_t) crossxbuf[5]) << 8 | ((int16_t) crossxbuf[4]);
		OTPcrossaxis[1] = ((int16_t) crossybuf[1]) << 8 | ((int16_t) crossybuf[0]);
		OTPcrossaxis[4] = ((int16_t) crossybuf[3]) << 8 | ((int16_t) crossybuf[2]);
		OTPcrossaxis[7] = ((int16_t) crossybuf[5]) << 8 | ((int16_t) crossybuf[4]);
		OTPcrossaxis[2] = ((int16_t) crosszbuf[1]) << 8 | ((int16_t) crosszbuf[0]);
		OTPcrossaxis[5] = ((int16_t) crosszbuf[3]) << 8 | ((int16_t) crosszbuf[2]);
		OTPcrossaxis[8] = ((int16_t) crosszbuf[5]) << 8 | ((int16_t) crosszbuf[4]);
		*crossaxis_det = ((int32_t)OTPcrossaxis[0]) * OTPcrossaxis[4] * OTPcrossaxis[8] +
				 ((int32_t)OTPcrossaxis[1]) * OTPcrossaxis[5] * OTPcrossaxis[6] +
				 ((int32_t)OTPcrossaxis[2]) * OTPcrossaxis[3] * OTPcrossaxis[7] -
				 ((int32_t)OTPcrossaxis[0]) * OTPcrossaxis[5] * OTPcrossaxis[7] -
				 ((int32_t)OTPcrossaxis[2]) * OTPcrossaxis[4] * OTPcrossaxis[6] -
				 ((int32_t)OTPcrossaxis[1]) * OTPcrossaxis[3] * OTPcrossaxis[8];

		if (*crossaxis_det == 0) {
			return false;
		}

#ifdef IST_CROSS_AXIS_CALI_FLOAT
		inv[0] = (float)OTPcrossaxis[4] * OTPcrossaxis[8] - (float)OTPcrossaxis[5] * OTPcrossaxis[7];
		inv[1] = (float)OTPcrossaxis[2] * OTPcrossaxis[7] - (float)OTPcrossaxis[1] * OTPcrossaxis[8];
		inv[2] = (float)OTPcrossaxis[1] * OTPcrossaxis[5] - (float)OTPcrossaxis[2] * OTPcrossaxis[4];
		inv[3] = (float)OTPcrossaxis[5] * OTPcrossaxis[6] - (float)OTPcrossaxis[3] * OTPcrossaxis[8];
		inv[4] = (float)OTPcrossaxis[0] * OTPcrossaxis[8] - (float)OTPcrossaxis[2] * OTPcrossaxis[6];
		inv[5] = (float)OTPcrossaxis[2] * OTPcrossaxis[3] - (float)OTPcrossaxis[0] * OTPcrossaxis[5];
		inv[6] = (float)OTPcrossaxis[3] * OTPcrossaxis[7] - (float)OTPcrossaxis[4] * OTPcrossaxis[6];
		inv[7] = (float)OTPcrossaxis[1] * OTPcrossaxis[6] - (float)OTPcrossaxis[0] * OTPcrossaxis[7];
		inv[8] = (float)OTPcrossaxis[0] * OTPcrossaxis[4] - (float)OTPcrossaxis[1] * OTPcrossaxis[3];

		for (i = 0; i < 9; i++) {
			crossaxis_inv[i] = inv[i] * OTPsensitivity / (*crossaxis_det);
		}

#else
		inv[0] = (int64_t)OTPcrossaxis[4] * OTPcrossaxis[8] - (int64_t)OTPcrossaxis[5] * OTPcrossaxis[7];
		inv[1] = (int64_t)OTPcrossaxis[2] * OTPcrossaxis[7] - (int64_t)OTPcrossaxis[1] * OTPcrossaxis[8];
		inv[2] = (int64_t)OTPcrossaxis[1] * OTPcrossaxis[5] - (int64_t)OTPcrossaxis[2] * OTPcrossaxis[4];
		inv[3] = (int64_t)OTPcrossaxis[5] * OTPcrossaxis[6] - (int64_t)OTPcrossaxis[3] * OTPcrossaxis[8];
		inv[4] = (int64_t)OTPcrossaxis[0] * OTPcrossaxis[8] - (int64_t)OTPcrossaxis[2] * OTPcrossaxis[6];
		inv[5] = (int64_t)OTPcrossaxis[2] * OTPcrossaxis[3] - (int64_t)OTPcrossaxis[0] * OTPcrossaxis[5];
		inv[6] = (int64_t)OTPcrossaxis[3] * OTPcrossaxis[7] - (int64_t)OTPcrossaxis[4] * OTPcrossaxis[6];
		inv[7] = (int64_t)OTPcrossaxis[1] * OTPcrossaxis[6] - (int64_t)OTPcrossaxis[0] * OTPcrossaxis[7];
		inv[8] = (int64_t)OTPcrossaxis[0] * OTPcrossaxis[4] - (int64_t)OTPcrossaxis[1] * OTPcrossaxis[3];

		for (i = 0; i < 9; i++) {
			crossaxis_inv[i] = (inv[i] << bitshift) * OTPsensitivity;
		}

#endif
	}

	return true;
}

void
IST8310::crossaxis_transformation(int16_t *xyz)
{
#ifdef IST_CROSS_AXIS_CALI_FLOAT
	float outputtmp[3];
#else
	int64_t outputtmp[3];
#endif

	outputtmp[0] = xyz[0] * crossaxis_inv[0] +
		       xyz[1] * crossaxis_inv[1] +
		       xyz[2] * crossaxis_inv[2];

	outputtmp[1] = xyz[0] * crossaxis_inv[3] +
		       xyz[1] * crossaxis_inv[4] +
		       xyz[2] * crossaxis_inv[5];

	outputtmp[2] = xyz[0] * crossaxis_inv[6] +
		       xyz[1] * crossaxis_inv[7] +
		       xyz[2] * crossaxis_inv[8];
#ifdef IST_CROSS_AXIS_CALI_FLOAT
	xyz[0] = (short)(outputtmp[0]);
	xyz[1] = (short)(outputtmp[1]);
	xyz[2] = (short)(outputtmp[2]);
#else
	int i = 0;

	for (i = 0; i < 3; i++) {
		outputtmp[i] = outputtmp[i] / (*crossaxis_det);
	}

	xyz[0] = (short)(outputtmp[0] >> CROSSAXISINV_BITSHIFT);
	xyz[1] = (short)(outputtmp[1] >> CROSSAXISINV_BITSHIFT);
	xyz[2] = (short)(outputtmp[2] >> CROSSAXISINV_BITSHIFT);
#endif
}

int
IST8310::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return write(reg, &buf, 1);
}

int
IST8310::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
IST8310::meas_to_float(uint8_t in[2])
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
IST8310::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.2f %.2f %.2f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f\n",
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)(1.0f / _range_scale));
	printf("temperature %.2f\n", (double)_last_report.temperature);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace ist8310
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

/*
  list of supported bus configurations
 */
struct ist8310_bus_option {
	enum IST8310_BUS busid;
	const char *devpath;
	uint8_t busnum;
	IST8310 *dev;
} bus_options[] = {
	{ IST8310_BUS_I2C_EXTERNAL, "/dev/ist8310_ext", PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_ONBOARD
	{ IST8310_BUS_I2C_INTERNAL, "/dev/ist8310_int", PX4_I2C_BUS_ONBOARD, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void    start(enum IST8310_BUS busid, int address, enum Rotation rotation);
bool    start_bus(struct ist8310_bus_option &bus, int address, enum Rotation rotation);
struct ist8310_bus_option &find_bus(enum IST8310_BUS busid);
void    test(enum IST8310_BUS busid);
void    reset(enum IST8310_BUS busid);
int info(enum IST8310_BUS busid);
int calibrate(enum IST8310_BUS busid);
void    usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct ist8310_bus_option &bus, int address, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	IST8310 *interface = new IST8310(bus.busnum, address,  bus.devpath, rotation);

	if (interface->init() != OK) {
		delete interface;
		PX4_INFO("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = interface;

	int fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
	}

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
start(enum IST8310_BUS busid, int address, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == IST8310_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != IST8310_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], address, rotation);
	}

	if (!started) {
		exit(1);
	}
}

/**
 * find a bus structure for a busid
 */
struct ist8310_bus_option &find_bus(enum IST8310_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == IST8310_BUS_ALL ||
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
test(enum IST8310_BUS busid)
{
	struct ist8310_bus_option &bus = find_bus(busid);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'ist8310 start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	PX4_INFO("single read");
	PX4_INFO("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	PX4_INFO("time:        %lld", report.timestamp);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external");
	}

	PX4_INFO("device active: %s", ret ? "external" : "onboard");

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

		PX4_INFO("periodic read %u", i);
		PX4_INFO("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		PX4_INFO("time:        %lld", report.timestamp);
	}

	PX4_INFO("PASS");
	exit(0);
}


/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766 | for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713 | for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the IST8310L for proper operation, a self test feature in incorporated
 * in which the sensor will change the polarity on all 3 axis. The values with and
 * with and without selftest on shoult be compared and if the absolete value are equal
 * the IC is functional.
 */
int calibrate(enum IST8310_BUS busid)
{
	int ret;
	struct ist8310_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'ist8310 start' if the driver is not running", path);
	}

	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		PX4_WARN("failed to enable sensor calibration mode");
	}

	close(fd);

	return ret;
}

/**
 * Reset the driver.
 */
void
reset(enum IST8310_BUS busid)
{
	struct ist8310_bus_option &bus = find_bus(busid);
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

	// Relay on at_)exit to close handel
	exit(0);
}



/**
 * Print a little info about the driver.
 */
int
info(enum IST8310_BUS busid)
{
	struct ist8310_bus_option &bus = find_bus(busid);

	PX4_INFO("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'test', 'reset', 'calibrate'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
	PX4_INFO("    -C calibrate on start");
	PX4_INFO("    -a 12C Address (0x%02x)", IST8310_BUS_I2C_ADDR);
	PX4_INFO("    -b 12C bus (%d|%d)", IST8310_BUS_I2C_EXTERNAL, IST8310_BUS_I2C_INTERNAL);
}

} // namespace

int
ist8310_main(int argc, char *argv[])
{
	int ch;

	IST8310_BUS i2c_busid = IST8310_BUS_ALL;
	int i2c_addr = IST8310_BUS_I2C_ADDR; /* 7bit */

	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;

	while ((ch = getopt(argc, argv, "R:Ca:b:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		case 'a':
			i2c_addr = (int)strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2c_busid = (IST8310_BUS)strtol(optarg, NULL, 0);
			break;

		case 'C':
			calibrate = true;
			break;

		default:
			ist8310::usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		ist8310::usage();
		exit(1);
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ist8310::start(i2c_busid, i2c_addr, rotation);

		if (calibrate && ist8310::calibrate(i2c_busid) != 0) {
			errx(1, "calibration failed");
		}

		exit(0);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		ist8310::test(i2c_busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		ist8310::reset(i2c_busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		ist8310::info(i2c_busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (ist8310::calibrate(i2c_busid) == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate', 'tempoff', 'tempon' or 'info'");
}

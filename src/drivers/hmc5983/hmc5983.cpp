/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file hmc5983.cpp
 * Driver for the Honeywell HMC5983 MEMS magnetometer connected via SPI.
 *
 * Note: This driver is intentionally limited to SPI. Please refer to the
 *       HMC5883 driver (which is API-compatible) for an I2C interfaced version.
 */

#include <nuttx/config.h>

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

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_mag.h>
#include <drivers/device/ringbuffer.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "../hmc5883/hmc5x83.h"

#define HMC5983_DEVICE_PATH "/dev/hmc5983"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

#ifndef SENSOR_BOARD_ROTATION_DEFAULT
#define SENSOR_BOARD_ROTATION_DEFAULT		SENSOR_BOARD_ROTATION_270_DEG
#endif

#define HMC5983_DEFAULT_RATE 150
#define HMC5983_DEFAULT_FILTER_FREQ 30

extern "C" { __EXPORT int hmc5983_main(int argc, char *argv[]); }

class HMC5983 : public device::SPI
{
public:
	HMC5983(int bus, const char* path, spi_dev_e device, enum Rotation rotation);
	virtual ~HMC5983();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	struct hrt_call		_call;
	unsigned		_call_interval;
	
	RingBuffer		*_reports;

	struct mag_scale	_mag_scale;
	float			_mag_range_scale;
	float			_mag_range_rad_s;
	orb_advert_t		_mag_topic;
	orb_id_t		_orb_id;
	int			_class_instance;

	unsigned		_current_rate;
	unsigned		_orientation;

	unsigned		_read;

	perf_counter_t		_sample_perf;
	perf_counter_t		_reschedules;
	perf_counter_t		_errors;

	math::LowPassFilter2p	_mag_filter_x;
	math::LowPassFilter2p	_mag_filter_y;
	math::LowPassFilter2p	_mag_filter_z;

	/* true if an L3G4200D is detected */
	bool	_is_l3g4200d;

	enum Rotation		_rotation;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset the driver
	 */
	void			reset();

	/**
	 * disable I2C on the chip
	 */
	void			disable_i2c();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Read a register from the HMC5983
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the HMC5983
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the HMC5983
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the HMC5983 measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_dps);

	/**
	 * Set the HMC5983 internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_samplerate(unsigned frequency);

	/**
	 * Set the lowpass filter of the driver
	 *
	 * @param samplerate	The current samplerate
	 * @param frequency	The cutoff frequency for the lowpass filter
	 */
	void			set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();

	/* this class does not allow copying */
	HMC5983(const HMC5983&);
	HMC5983 operator=(const HMC5983&);
};

HMC5983::HMC5983(int bus, const char* path, spi_dev_e device, enum Rotation rotation) :
	SPI("HMC5983", path, bus, device, SPIDEV_MODE3, 11*1000*1000 /* will be rounded to 10.4 MHz, within margins for HMC5983 */),
	_call{},
	_call_interval(0),
	_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(0.0f),
	_mag_range_rad_s(0.0f),
	_mag_topic(-1),
	_orb_id(nullptr),
	_class_instance(-1),
	_current_rate(0),
	_orientation(SENSOR_BOARD_ROTATION_DEFAULT),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "hmc5983_read")),
	_reschedules(perf_alloc(PC_COUNT, "hmc5983_reschedules")),
	_errors(perf_alloc(PC_COUNT, "hmc5983_errors")),
	_mag_filter_x(HMC5983_DEFAULT_RATE, HMC5983_DEFAULT_FILTER_FREQ),
	_mag_filter_y(HMC5983_DEFAULT_RATE, HMC5983_DEFAULT_FILTER_FREQ),
	_mag_filter_z(HMC5983_DEFAULT_RATE, HMC5983_DEFAULT_FILTER_FREQ),
	_is_l3g4200d(false),
        _rotation(rotation)                                            
{
	// enable debug() calls
	_debug_enabled = true;

	// default scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;
}

HMC5983::~HMC5983()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;

	if (_class_instance != -1)
		unregister_class_devname(MAG_DEVICE_PATH, _class_instance);

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_reschedules);
	perf_free(_errors);
}

int
HMC5983::init()
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr)
		goto out;

	_class_instance = register_class_devname(MAG_DEVICE_PATH);

	switch (_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_orb_id = ORB_ID(sensor_mag0);
			break;

		case CLASS_DEVICE_SECONDARY:
			_orb_id = ORB_ID(sensor_mag1);
			break;

		case CLASS_DEVICE_TERTIARY:
			_orb_id = ORB_ID(sensor_mag2);
			break;
	}

	reset();

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report grp;
	_reports->get(&grp);

	_mag_topic = orb_advertise(_orb_id, &grp);

	if (_mag_topic < 0) {
		debug("failed to create sensor_mag publication");
	}

	ret = OK;
out:
	return ret;
}

int
HMC5983::probe()
{
	uint8_t data[3] = {0, 0, 0};

	data[0] = read_reg(ADDR_ID_A);
	data[1] = read_reg(ADDR_ID_B);
	data[2] = read_reg(ADDR_ID_C);

	if ((data[0] != ID_A_WHO_AM_I) ||
	    (data[1] != ID_B_WHO_AM_I) ||
	    (data[2] != ID_C_WHO_AM_I)) {
		debug("ID byte mismatch (%02x,%02x,%02x)", data[0], data[1], data[2]);
		return -EIO;
	}

	_orientation = SENSOR_BOARD_ROTATION_DEFAULT;

	return OK;
}

ssize_t
HMC5983::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *gbuf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_reports->get(gbuf)) {
				ret += sizeof(*gbuf);
				gbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(gbuf)) {
		ret = sizeof(*gbuf);
	}

	return ret;
}

int
HMC5983::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

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
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, HMC5983_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					/* adjust filters */
					float cutoff_freq_hz = _mag_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f/ticks;
					set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);
		
		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		reset();
		return OK;

	case MAGIOCSSAMPLERATE:
		return set_samplerate(arg);

	case MAGIOCGSAMPLERATE:
		return _current_rate;

	case MAGIOCSLOWPASS: {
		float cutoff_freq_hz = arg;
		float sample_rate = 1.0e6f / _call_interval;
		set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

		return OK;
	}

	// case MAGIOCGLOWPASS:
	// 	return _mag_filter_x.get_cutoff_freq();

	// case MAGIOCSSCALE:
	// 	/* copy scale in */
	// 	memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
	// 	return OK;

	// case MAGIOCGSCALE:
	// 	/* copy scale out */
	// 	memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
	// 	return OK;

	// case GYROIOCSRANGE:
	// 	/* arg should be in dps */
	// 	return set_range(arg);

	// case GYROIOCGRANGE:
	// 	/* convert to dps and round */
	// 	return (unsigned long)(_mag_range_rad_s * 180.0f / M_PI_F + 0.5f);

	// case GYROIOCSELFTEST:
	// 	return self_test();

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
HMC5983::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
HMC5983::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
HMC5983::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int
HMC5983::set_range(unsigned max_dps)
{
	// XXX implement

	return OK;
}

int
HMC5983::set_samplerate(unsigned frequency)
{
	// XXX implement

	return OK;
}

void
HMC5983::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_mag_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_mag_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_mag_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
HMC5983::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&HMC5983::measure_trampoline, this);
}

void
HMC5983::stop()
{
	hrt_cancel(&_call);
}

void
HMC5983::disable_i2c(void)
{
	// XXX check if necessary
}

void
HMC5983::reset()
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();

	// XXX implement

	set_samplerate(0); // 760Hz or 800Hz
	// set_range(HMC5983_DEFAULT_RANGE_DPS);
	set_driver_lowpass_filter(HMC5983_DEFAULT_RATE, HMC5983_DEFAULT_FILTER_FREQ);

	_read = 0;
}

void
HMC5983::measure_trampoline(void *arg)
{
	HMC5983 *dev = (HMC5983 *)arg;

	/* make another measurement */
	dev->measure();
}

#ifdef GPIO_EXTI_GYRO_DRDY
# define HMC5983_USE_DRDY 1
#else
# define HMC5983_USE_DRDY 0
#endif

void
HMC5983::measure()
{
// #if HMC5983_USE_DRDY
// 	// if the mag doesn't have any data ready then re-schedule
// 	// for 100 microseconds later. This ensures we don't double
// 	// read a value and then miss the next value
// 	if (_bus == PX4_SPI_BUS_SENSORS && stm32_gpioread(GPIO_EXTI_GYRO_DRDY) == 0) {
// 		perf_count(_reschedules);
// 		hrt_call_delay(&_call, 100);
// 		return;
// 	}
// #endif

// 	/* status register and data as read back from the device */
// #pragma pack(push, 1)
// 	struct {
// 		uint8_t		cmd;
// 		uint8_t		temp;
// 		uint8_t		status;
// 		int16_t		x;
// 		int16_t		y;
// 		int16_t		z;
// 	} raw_report;
// #pragma pack(pop)

// 	mag_report report;

// 	/* start the performance counter */
// 	perf_begin(_sample_perf);

// 	/* fetch data from the sensor */
// 	memset(&raw_report, 0, sizeof(raw_report));
// 	raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
// 	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

// #if HMC5983_USE_DRDY
//         if ((raw_report.status & 0xF) != 0xF) {
//             /*
//               we waited for DRDY, but did not see DRDY on all axes
//               when we captured. That means a transfer error of some sort
//              */
//             perf_count(_errors);            
//             return;
//         }
// #endif
	
// 	 * 1) Scale raw value to SI units using scaling from datasheet.
// 	 * 2) Subtract static offset (in SI units)
// 	 * 3) Scale the statically calibrated values with a linear
// 	 *    dynamically obtained factor
// 	 *
// 	 * Note: the static sensor offset is the number the sensor outputs
// 	 * 	 at a nominally 'zero' input. Therefore the offset has to
// 	 * 	 be subtracted.
// 	 *
// 	 *	 Example: A mag outputs a value of 74 at zero angular rate
// 	 *	 	  the offset is 74 from the origin and subtracting
// 	 *		  74 from all measurements centers them around zero.
	 
// 	report.timestamp = hrt_absolute_time();
//         report.error_count = 0; // not recorded
	
// 	switch (_orientation) {

// 		case SENSOR_BOARD_ROTATION_000_DEG:
// 			/* keep axes in place */
// 			report.x_raw = raw_report.x;
// 			report.y_raw = raw_report.y;
// 			break;

// 		case SENSOR_BOARD_ROTATION_090_DEG:
// 			/* swap x and y */
// 			report.x_raw = raw_report.y;
// 			report.y_raw = raw_report.x;
// 			break;

// 		case SENSOR_BOARD_ROTATION_180_DEG:
// 			/* swap x and y and negate both */
// 			report.x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
// 			report.y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
// 			break;

// 		case SENSOR_BOARD_ROTATION_270_DEG:
// 			/* swap x and y and negate y */
// 			report.x_raw = raw_report.y;
// 			report.y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
// 			break;
// 	}

// 	report.z_raw = raw_report.z;

// 	report.x = ((report.x_raw * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
// 	report.y = ((report.y_raw * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
// 	report.z = ((report.z_raw * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;

// 	report.x = _mag_filter_x.apply(report.x);
// 	report.y = _mag_filter_y.apply(report.y);
// 	report.z = _mag_filter_z.apply(report.z);

// 	// apply user specified rotation
// 	rotate_3f(_rotation, report.x, report.y, report.z);

// 	report.scaling = _mag_range_scale;
// 	report.range_rad_s = _mag_range_rad_s;

// 	_reports->force(&report);

// 	/* notify anyone waiting for data */
// 	poll_notify(POLLIN);

// 	/* publish for subscribers */
// 	if (!(_pub_blocked)) {
// 		/* publish it */
// 		orb_publish(_orb_id, _mag_topic, &report);
// 	}

// 	_read++;

// 	/* stop the perf counter */
// 	perf_end(_sample_perf);
}

void
HMC5983::print_info()
{
	printf("mag reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_reschedules);
	perf_print_counter(_errors);
	_reports->print_info("report queue");
}

int
HMC5983::self_test()
{
	// XXX implement

	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace hmc5983
{

HMC5983	*g_dev;

void	usage();
void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;

	if (g_dev != nullptr)
		errx(0, "already started");

	/* create the driver */
	g_dev = new HMC5983(PX4_SPI_BUS_SENSORS, HMC5983_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_HMC, rotation);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(HMC5983_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

        close(fd);

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
	int fd_mag = -1;
	struct mag_report g_report;
	ssize_t sz;

	/* get the driver */
	fd_mag = open(HMC5983_DEVICE_PATH, O_RDONLY);

	if (fd_mag < 0)
		err(1, "%s open failed", HMC5983_DEVICE_PATH);

	/* reset to manual polling */
	if (ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
		err(1, "reset to manual polling");

	/* do a simple demand read */
	sz = read(fd_mag, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report))
		err(1, "immediate mag read failed");

	warnx("mag x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("mag y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("mag z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("mag x: \t%d\traw", (int)g_report.x_raw);
	warnx("mag y: \t%d\traw", (int)g_report.y_raw);
	warnx("mag z: \t%d\traw", (int)g_report.z_raw);
	// warnx("mag range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      // (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

        close(fd_mag);

	/* XXX add poll-rate tests here too */

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(HMC5983_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "accel pollrate reset failed");

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
		errx(1, "driver not running\n");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
hmc5983_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
		default:
			hmc5983::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start"))
		hmc5983::start(external_bus, rotation);

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test"))
		hmc5983::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset"))
		hmc5983::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info"))
		hmc5983::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}

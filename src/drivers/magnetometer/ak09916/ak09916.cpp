/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file mag.cpp
 *
 * Driver for the ak09916 magnetometer within the Invensense mpu9250
 *
 * @author Robert Dickenson
 *
 */

#include <px4_config.h>
#include <px4_log.h>
#include <px4_time.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>


#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include "ak09916.hpp"
#include <px4_getopt.h>

/** driver 'main' command */
extern "C" { __EXPORT int ak09916_main(int argc, char *argv[]); }

#define AK09916_CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

namespace ak09916
{

AK09916 *g_dev_ext;
AK09916 *g_dev_int;

void start(bool, enum Rotation);
void test(bool);
void reset(bool);
void info(bool);
void usage();


/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void start(bool external_bus, enum Rotation rotation)
{
	int fd;
	AK09916 **g_dev_ptr = (external_bus ? &g_dev_ext : &g_dev_int);
	const char *path = (external_bus ? AK09916_DEVICE_PATH_MAG_EXT : AK09916_DEVICE_PATH_MAG);

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		PX4_ERR("already started");
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_I2C_BUS_EXPANSION)
		*g_dev_ptr = new AK09916(PX4_I2C_BUS_EXPANSION, path, rotation);
#else
		PX4_ERR("External I2C not available");
		exit(0);
#endif

	} else {
		PX4_ERR("Internal I2C not available");
		exit(0);
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}


	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		PX4_ERR("Failed to setup poll rate");
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	PX4_ERR("driver start failed");
	exit(1);

}


void test(bool external_bus)
{
	int fd = -1;
	const char *path = (external_bus ? AK09916_DEVICE_PATH_MAG_EXT : AK09916_DEVICE_PATH_MAG);
	struct mag_report m_report;
	ssize_t sz;


	/* get the driver */
	fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'AK09916 start' if the driver is not running)", path);
		exit(1);
	}

	/* do a simple demand read */
	sz = read(fd, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		PX4_ERR("immediate mag read failed");
		exit(1);
	}

	PX4_WARN("single read");
	PX4_WARN("time:     %lld", m_report.timestamp);
	PX4_WARN("mag  x:  \t%8.4f\t", (double)m_report.x);
	PX4_WARN("mag  y:  \t%8.4f\t", (double)m_report.y);
	PX4_WARN("mag  z:  \t%8.4f\t", (double)m_report.z);
	PX4_WARN("mag  x:  \t%d\traw 0x%0x", (short)m_report.x_raw, (unsigned short)m_report.x_raw);
	PX4_WARN("mag  y:  \t%d\traw 0x%0x", (short)m_report.y_raw, (unsigned short)m_report.y_raw);
	PX4_WARN("mag  z:  \t%d\traw 0x%0x", (short)m_report.z_raw, (unsigned short)m_report.z_raw);

	PX4_ERR("PASS");
	exit(0);

}


void
reset(bool external_bus)
{
	const char *path = external_bus ? AK09916_DEVICE_PATH_MAG_EXT : AK09916_DEVICE_PATH_MAG;
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		exit(1);
	}

	exit(0);

}

void
info(bool external_bus)
{
	AK09916 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);

}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'stop',\n'reset'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external bus)");

}

} // namespace AK09916

// If interface is non-null, then it will used for interacting with the device.
// Otherwise, it will passthrough the parent AK09916
AK09916::AK09916(int bus, const char *path, enum Rotation rotation) :
	I2C("AK09916", path, bus, AK09916_I2C_ADDR, 400000),
	_measure_ticks(0),
	_rotation(rotation),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1),
	_mag_reading_data(false),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(),
	_mag_reads(perf_alloc(PC_COUNT, "ak09916_mag_reads")),
	_mag_errors(perf_alloc(PC_COUNT, "ak09916_mag_errors")),
	_mag_overruns(perf_alloc(PC_COUNT, "ak09916_mag_overruns")),
	_mag_overflows(perf_alloc(PC_COUNT, "ak09916_mag_overflows")),
	_mag_duplicates(perf_alloc(PC_COUNT, "ak09916_mag_duplicates")),
	_mag_asa_x(1.0),
	_mag_asa_y(1.0),
	_mag_asa_z(1.0),
	_last_mag_data{}
{
	// default mag scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;

	_mag_range_scale = AK09916_MAG_RANGE_GA;


	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

AK09916::~AK09916()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	orb_unadvertise(_mag_topic);

	perf_free(_mag_reads);
	perf_free(_mag_errors);
	perf_free(_mag_overruns);
	perf_free(_mag_overflows);
	perf_free(_mag_duplicates);
}

int
AK09916::init()
{
	int ret = I2C::init();

	/* if cdev init failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("AK09916 mag init failed");

		return ret;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr) {
		return -ENOMEM;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	reset();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report mrp;
	_mag_reports->get(&mrp);

	_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					 &_mag_orb_class_instance, ORB_PRIO_VERY_HIGH);
//    &_mag_orb_class_instance, ORB_PRIO_LOW);

	if (_mag_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
		return -ENOMEM;
	}

	return OK;
}

bool AK09916::check_duplicate(uint8_t *mag_data)
{
	if (memcmp(mag_data, &_last_mag_data, sizeof(_last_mag_data)) == 0) {
		// it isn't new data - wait for next timer
		return true;

	} else {
		memcpy(&_last_mag_data, mag_data, sizeof(_last_mag_data));
		return false;
	}
}

void
AK09916::measure()
{
	uint8_t ret, cmd = AK09916REG_ST1;
	struct ak09916_regs raw_data;

	ret = transfer(&cmd, 1, (uint8_t *)(&raw_data), sizeof(struct ak09916_regs));

	if (ret == OK) {
		raw_data.st2 = raw_data.st2;

		_measure(raw_data);
	}
}

void
AK09916::_measure(struct ak09916_regs data)
{
	bool mag_notify = true;

	if (check_duplicate((uint8_t *)&data.x) && !(data.st1 & 0x02)) {
		perf_count(_mag_duplicates);
		return;
	}

	/* monitor for if data overrun flag is ever set */
	if (data.st1 & 0x02) {
		perf_count(_mag_overruns);
	}

	/* monitor for if magnetic sensor overflow flag is ever set noting that st2
	 * is usually not even refreshed, but will always be in the same place in the
	 * mpu's buffers regardless, hence the actual count would be bogus
	 */
	if (data.st2 & 0x08) {
		perf_count(_mag_overflows);
	}

	mag_report	mrb;
	mrb.timestamp = hrt_absolute_time();
	mrb.is_external = true;

	float xraw_f, yraw_f, zraw_f;
	mrb.x_raw = data.x;
	mrb.y_raw = data.y;
	mrb.z_raw = data.z;

	xraw_f = data.x;
	yraw_f = data.y;
	zraw_f = data.z;
	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);


	mrb.x = ((xraw_f * _mag_range_scale * _mag_asa_x) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mrb.y = ((yraw_f * _mag_range_scale * _mag_asa_y) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mrb.z = ((zraw_f * _mag_range_scale * _mag_asa_z) - _mag_scale.z_offset) * _mag_scale.z_scale;
	_last_report.x = mrb.x;
	_last_report.y = mrb.y;
	_last_report.z = mrb.z;
	mrb.scaling = _mag_range_scale;
	mrb.device_id = _device_id.devid;

	mrb.error_count = perf_event_count(_mag_errors);

	_mag_reports->force(&mrb);

	/* notify anyone waiting for data */
	if (mag_notify) {
		poll_notify(POLLIN);
	}

	if (mag_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _mag_topic, &mrb);
	}
}

ssize_t
AK09916::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_mag_reports->flush();

		/* trigger a measurement */
		measure();

		if (_mag_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	/* return the number of bytes transferred */
	return ret;

}


void
AK09916::print_info()
{
	perf_print_counter(_mag_reads);
	perf_print_counter(_mag_errors);
	perf_print_counter(_mag_overruns);
	_mag_reports->print_info("mag queue");

	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("\n");

}


int
AK09916::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	/*
	 * Repeated in ICM20948_accel::ioctl
	 * Both accel and mag CDev could be unused in case of magnetometer only mode or MPU6500
	 */

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(AK09916_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(AK09916_CONVERSION_INTERVAL)) {
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

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCCALIBRATE:
		return OK;

	case MAGIOCEXSTRAP:
		return OK;

	default:
		return (int)I2C::ioctl(filp, cmd, arg);
	}
}

uint8_t
AK09916::read_reg(uint8_t reg)
{
	const uint8_t cmd = reg;
	uint8_t ret;

	transfer(&cmd, 1, &ret, 1);

	return ret;
}

bool
AK09916::check_id(uint8_t &deviceid)
{
	deviceid = read_reg(AK09916REG_WIA);

	return (AK09916_DEVICE_ID_A == deviceid);
}

void
AK09916::write_reg(uint8_t reg, uint8_t value)
{
	const uint8_t cmd[2] = { reg, value};
	transfer(cmd, 2, nullptr, 0);
}

int
AK09916::reset(void)
{
	// First initialize it to use the bus
	int rv = setup();

	if (rv == OK) {
		// Now reset the mag
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		// Then re-initialize the bus/mag
		rv = setup();
	}

	return rv;
}

int
AK09916::setup(void)
{
	int retries = 10;

	do {
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		uint8_t id = 0;

		if (check_id(id)) {
			break;
		}

		retries--;
	} while (retries > 0);

	write_reg(AK09916REG_CNTL2, AK09916_CNTL2_CONTINOUS_MODE_100HZ);

	return OK;
}


void
AK09916::cycle_trampoline(void *arg)
{
	AK09916 *dev = (AK09916 *)arg;

	dev->cycle();
}


void
AK09916::start()
{
	/* reset the report ring and state machine */
	_mag_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&AK09916::cycle_trampoline, this, 1);
}

void
AK09916::stop()
{
	if (_measure_ticks > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_ticks = 0;
		work_cancel(HPWORK, &_work);
	}
}

void
AK09916::cycle()
{
	if (_measure_ticks == 0) {
		return;
	}

	/* measurement phase */
	measure();

	if (_measure_ticks > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&AK09916::cycle_trampoline,
			   this,
			   USEC2TICK(AK09916_CONVERSION_INTERVAL));
	}
}

int
ak09916_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			ak09916::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		ak09916::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ak09916::start(external_bus, rotation);
	}


	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		ak09916::test(external_bus);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		ak09916::reset(external_bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		ak09916::info(external_bus);
	}

	ak09916::usage();
	return -1;
}
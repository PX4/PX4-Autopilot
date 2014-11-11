/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file ms4525_baro.cpp
 * @author Lorenz Meier
 * @author Sarthak Kaingade
 * @author Simon Wilks
 * @author Thomas Gubler
 *
 * Driver for the MEAS Spec series connected via I2C.
 *
 * Supported sensors:
 *
 *    - MS4525DO (http://www.meas-spec.com/downloads/MS4525DO.pdf)
 *    - untested: MS5525DSO (http://www.meas-spec.com/downloads/MS5525DSO.pdf)
 *
 * Interface application notes:
 *
 *    - Interfacing to MEAS Digital Pressure Modules (http://www.meas-spec.com/downloads/Interfacing_to_MEAS_Digital_Pressure_Modules.pdf)
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

#include <board_config.h>

#include <systemlib/airspeed.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>

#include <drivers/airspeed/airspeed.h>

#include "ms4525_baro.h"

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms4525_baro_main(int argc, char *argv[]);

/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MS4525Baro::MS4525Baro(int bus, int address, const char *path) : MEASAirspeed(bus, address, path)
{
}

int
MS4525Baro::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	if (_reports) {
		delete _reports;
	}
	_reports = new RingBuffer(2, sizeof(differential_pressure_s));
	if (_reports == nullptr)
		goto out;

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_DEVICE_PATH);

	switch (_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_orb_id = ORB_ID(sensor_baro0);
			break;
		
		case CLASS_DEVICE_SECONDARY:
			_orb_id = ORB_ID(sensor_baro1);
			break;

		case CLASS_DEVICE_TERTIARY:
			_orb_id = ORB_ID(sensor_baro2);
			break;

	}

	/* publication init */
	if (_class_instance == CLASS_DEVICE_PRIMARY) {

		/* advertise sensor topic, measure manually to initialize valid report */
		struct baro_report arp;
		measure();
		_reports->get(&arp);

		/* measurement will have generated a report, publish */
		_pub = orb_advertise(_orb_id, &arp);

		if (_pub < 0)
			warnx("ADVERT FAIL: uORB started?");
	}

	ret = OK;

out:
	return ret;
}

MS4525Baro::~MS4525Baro()
{
	if (_class_instance != -1)
		unregister_class_devname(BARO_DEVICE_PATH, _class_instance);

	// Let the other destructors see a sane de-init value
	_class_instance = -1;
}

int
MS4525Baro::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[4] = {0, 0, 0, 0};


	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 4);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint8_t status = (val[0] & 0xC0) >> 6;

	switch (status) {
	case 0:
		break;

	case 1:
		/* fallthrough */
	case 2:
		/* fallthrough */
	case 3:
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	int16_t dp_raw = 0, dT_raw = 0;
	dp_raw = (val[0] << 8) + val[1];
	/* mask the used bits */
	dp_raw = 0x3FFF & dp_raw;
	dT_raw = (val[2] << 8) + val[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;
	float temperature = ((200.0f * dT_raw) / 2047) - 50;

	// Calculate absolute pressure. As its centered around 8000
	// and can go positive or negative
	const float P_min = 0.0f;
	const float P_max = 15.0f;
	const float PSI_to_Pa = 6894.757f;
	/*
	  this equation is an inversion of the equation in the
	  pressure transfer function figure on page 4 of the datasheet
	 */
	float press_PSI = -((dp_raw - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
	float press_pa_raw = press_PSI * PSI_to_Pa;

        // correct for 5V rail voltage if possible
        voltage_correction(press_pa_raw, temperature);

	// the raw value still should be compensated for the known offset
	press_pa_raw -= _diff_pres_offset;

	float press_pa = fabsf(press_pa_raw);
	
	struct baro_report report;

	/* track maximum differential pressure measured (so we can work out top speed). */
	if (press_pa > _max_differential_pressure_pa) {
		_max_differential_pressure_pa = press_pa;
	}

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = temperature;
	report.pressure = press_pa;
	//filtered = _filter.apply(press_pa);

	// /* the dynamics of the filter can make it overshoot into the negative range */
	// if (report.differential_pressure_filtered_pa < 0.0f) {
	// 	report.differential_pressure_filtered_pa = _filter.reset(press_pa);
	// }

	if ((_pub > 0) && !(_pub_blocked)) {
		/* publish it */
		orb_publish(_orb_id, _pub, &report);
	}

	if (!_reports->force(&report))
		perf_count(_buffer_overflows);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

/**
 * Local functions in support of the shell command.
 */
namespace ms4525_baro
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MS4525Baro	*g_dev = nullptr;

void	start(int i2c_bus);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is up and running
 * or failed to detect the sensor.
 */
void
start(int i2c_bus)
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver, try the MS4525DO first */
	g_dev = new MS4525Baro(i2c_bus, I2C_ADDRESS_MS4525DO, PATH_MS4525_BARO);

	/* check if the MS4525DO was instantiated */
	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->MS4525Baro::init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(PATH_MS4525_BARO, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "no MS4525 baro sensor connected");
}

/**
 * Stop the driver
 */
void
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
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

	int fd = open(PATH_MS4525_BARO, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'ms4525_baro start' if the driver is not running", PATH_MS4525_BARO);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("absolute pressure: %d pa", (int)report.pressure);

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
		warnx("diff pressure: %d pa", (int)report.pressure);
		warnx("temperature: %d C (0x%02x)", (int)report.temperature, (unsigned) report.temperature);
	}

	/* reset the sensor polling to its default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(PATH_MS4525_BARO, O_RDONLY);

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
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace


static void
ms4525_baro_usage()
{
	warnx("usage: ms4525_baro command [options]");
	warnx("options:");
	warnx("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	warnx("command:");
	warnx("\tstart|stop|reset|test|info");
}

int
ms4525_baro_main(int argc, char *argv[])
{
	int i2c_bus = PX4_I2C_BUS_DEFAULT;

	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bus") == 0) {
			if (argc > i + 1) {
				i2c_bus = atoi(argv[i + 1]);
			}
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		ms4525_baro::start(i2c_bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		ms4525_baro::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		ms4525_baro::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		ms4525_baro::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		ms4525_baro::info();
	}

	ms4525_baro_usage();
	exit(0);
}

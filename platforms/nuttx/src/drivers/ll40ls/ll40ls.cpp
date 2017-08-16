/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file ll40ls.cpp
 * @author Allyson Kreft
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author James Goppert <james.goppert@gmail.com>
 *
 * Interface for the PulsedLight Lidar-Lite range finders.
 */

#include "LidarLiteI2C.h"
#include "LidarLitePWM.h"
#include <board_config.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <cstdlib>
#include <string.h>
#include <stdio.h>

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define LL40LS_DEVICE_PATH_PWM  "/dev/ll40ls_pwm"

enum LL40LS_BUS {
	LL40LS_BUS_I2C_ALL = 0,
	LL40LS_BUS_I2C_INTERNAL,
	LL40LS_BUS_I2C_EXTERNAL,
	LL40LS_BUS_PWM
};

static struct ll40ls_bus_option {
	enum LL40LS_BUS busid;
	const char *devname;
	uint8_t busnum;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ LL40LS_BUS_I2C_EXTERNAL, "/dev/ll40ls_ext", PX4_I2C_BUS_EXPANSION },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ LL40LS_BUS_I2C_EXTERNAL, "/dev/ll40ls_ext1", PX4_I2C_BUS_EXPANSION1 },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ LL40LS_BUS_I2C_INTERNAL, "/dev/ll40ls_int", PX4_I2C_BUS_ONBOARD },
#endif
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ll40ls_main(int argc, char *argv[]);


/**
 * Local functions in support of the shell command.
 */
namespace ll40ls
{

LidarLite *instance = nullptr;

void	start(enum LL40LS_BUS busid);
void	stop();
void	test();
void	reset();
void	info();
void	regdump();
void	usage();

/**
 * Start the driver.
 */
void start(enum LL40LS_BUS busid)
{
	int fd, ret;

	if (instance) {
		warnx("driver already started");
	}

	if (busid == LL40LS_BUS_PWM) {
		instance = new LidarLitePWM(LL40LS_DEVICE_PATH_PWM);

		if (!instance) {
			warnx("Failed to instantiate LidarLitePWM");
			return;
		}

		if (instance->init() != PX4_OK) {
			warnx("failed to initialize LidarLitePWM");
			goto fail;
		}

	} else {
		for (uint8_t i = 0; i < (sizeof(bus_options) / sizeof(bus_options[0])); i++) {
			if (busid != LL40LS_BUS_I2C_ALL && busid != bus_options[i].busid) {
				continue;
			}

			instance = new LidarLiteI2C(bus_options[i].busnum, bus_options[i].devname);

			if (!instance) {
				warnx("Failed to instantiate LidarLiteI2C");
				return;
			}

			if (instance->init() == PX4_OK) {
				break;
			}

			warnx("failed to initialize LidarLiteI2C on busnum=%u", bus_options[i].busnum);
			delete instance;
			instance = nullptr;
		}
	}

	if (!instance) {
		warnx("No LidarLite found");
		return;
	}

	fd = open(instance->get_dev_name(), O_RDONLY);

	if (fd == -1) {
		warnx("Error opening fd");
		goto fail;
	}

	ret = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
	close(fd);

	if (ret < 0) {
		warnx("pollrate fail");
		goto fail;
	}

	return;

fail:
	delete instance;
	instance = nullptr;
}

/**
 * Stop the driver
 */
void stop()
{
	delete instance;
	instance = nullptr;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	if (!instance) {
		warnx("No ll40ls driver running");
		errx(1, "FAIL");
	}

	int fd = open(instance->get_dev_name(), O_RDONLY);

	if (fd < 0) {
		warnx("Error opening fd");
		errx(1, "FAIL");
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		warnx("immediate read failed");
		goto error;
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("time:        %lld", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (PX4_OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		warnx("failed to set 2Hz poll rate");
		goto error;
	}

	/* read the sensor 5 times and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			warnx("timed out waiting for sensor data");
			goto error;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warnx("periodic read failed");
			goto error;
		}

		warnx("periodic read %u", i);
		warnx("valid %u", (float)report.current_distance > report.min_distance
		      && (float)report.current_distance < report.max_distance ? 1 : 0);
		warnx("measurement: %0.3f m", (double)report.current_distance);
		warnx("time:        %lld", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (PX4_OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		warnx("failed to set default poll rate");
		goto error;
	}

	close(fd);
	errx(0, "PASS");

error:
	close(fd);
	errx(1, "FAIL");
}

/**
 * Reset the driver.
 */
void
reset()
{
	if (!instance) {
		warnx("No ll40ls driver running");
		return;
	}

	int fd = open(instance->get_dev_name(), O_RDONLY);

	if (fd < 0) {
		warnx("Error opening fd");
		return;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		warnx("driver reset failed");
		goto error;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		warnx("driver poll restart failed");
		goto error;
	}

error:
	close(fd);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (!instance) {
		warnx("No ll40ls driver running");
		return;
	}

	printf("state @ %p\n", instance);
	instance->print_info();
}

/**
 * Dump registers
 */
void
regdump()
{
	if (!instance) {
		warnx("No ll40ls driver running");
		return;
	}

	printf("regdump @ %p\n", instance);
	instance->print_registers();
}

void
usage()
{
	warnx("missing command: try 'start', 'stop', 'info', 'test', 'reset', 'info' or 'regdump' [i2c|pwm]");
	warnx("options for I2C:");
	warnx("    -X only external bus");
#ifdef PX4_I2C_BUS_ONBOARD
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
ll40ls_main(int argc, char *argv[])
{
	int ch;
	enum LL40LS_BUS busid = LL40LS_BUS_I2C_ALL;

	while ((ch = getopt(argc, argv, "XI")) != EOF) {
		switch (ch) {
#ifdef PX4_I2C_BUS_ONBOARD

		case 'I':
			busid = LL40LS_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = LL40LS_BUS_I2C_EXTERNAL;
			break;

		default:
			ll40ls::usage();
			return 0;
		}
	}

	/* determine protocol first because it's needed next */
	if (argc > optind + 1) {
		const char *protocol = argv[optind + 1];

		if (!strcmp(protocol, "pwm")) {
			busid = LL40LS_BUS_PWM;;

		} else if (!strcmp(protocol, "i2c")) {
			// Do nothing

		} else {
			warnx("unknown protocol, choose pwm or i2c");
			ll40ls::usage();
			return 0;
		}
	}

	/* now determine action */
	if (argc > optind) {
		const char *verb = argv[optind];

		if (!strcmp(verb, "start")) {
			ll40ls::start(busid);

		} else if (!strcmp(verb, "stop")) {
			ll40ls::stop();

		} else if (!strcmp(verb, "test")) {
			ll40ls::test();

		} else if (!strcmp(verb, "reset")) {
			ll40ls::reset();

		} else if (!strcmp(verb, "regdump")) {
			ll40ls::regdump();

		} else if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
			ll40ls::info();

		} else {
			ll40ls::usage();
		}

		return 0;
	}

	warnx("unrecognized command, try 'start', 'test', 'reset', 'info' or 'regdump'");
	ll40ls::usage();
	return 0;
}

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

#define LL40LS_DEVICE_PATH_INT  "/dev/ll40ls_int"
#define LL40LS_DEVICE_PATH_EXT  "/dev/ll40ls_ext"
#define LL40LS_DEVICE_PATH_PWM  "/dev/ll40ls_pwm"

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ll40ls_main(int argc, char *argv[]);


/**
 * Local functions in support of the shell command.
 */
namespace ll40ls
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

LidarLiteI2C	*g_dev_int;
LidarLiteI2C	*g_dev_ext;
LidarLitePWM	*g_dev_pwm;

LidarLite *get_dev(const bool use_i2c, const int bus);
void	start(const bool use_i2c, const int bus);
void	stop(const bool use_i2c, const int bus);
void	test(const bool use_i2c, const int bus);
void	reset(const bool use_i2c, const int bus);
void	info(const bool use_i2c, const int bus);
void    regdump(const bool use_i2c, const int bus);
void	usage();

/**
 * Get the correct device pointer
 */
LidarLite *get_dev(const bool use_i2c, const int bus)
{
	LidarLite *g_dev = nullptr;

	if (use_i2c) {
		g_dev = static_cast<LidarLite *>(bus == PX4_I2C_BUS_EXPANSION ? g_dev_ext : g_dev_int);

		if (g_dev == nullptr) {
			errx(1, "i2c driver not running");
		}

	} else {
		g_dev = static_cast<LidarLite *>(g_dev_pwm);

		if (g_dev == nullptr) {
			errx(1, "pwm driver not running");
		}
	}

	return g_dev;
};

/**
 * Start the driver.
 */
void start(const bool use_i2c, const int bus)
{
	if (g_dev_int != nullptr || g_dev_ext != nullptr || g_dev_pwm != nullptr) {
		errx(1, "driver already started");
	}

	if (use_i2c) {
		/* create the driver, attempt expansion bus first */
		if (bus == -1 || bus == PX4_I2C_BUS_EXPANSION) {
			if (g_dev_ext != nullptr) {
				errx(0, "already started external");
			}

			g_dev_ext = new LidarLiteI2C(PX4_I2C_BUS_EXPANSION, LL40LS_DEVICE_PATH_EXT);

			if (g_dev_ext != nullptr && OK != g_dev_ext->init()) {
				delete g_dev_ext;
				g_dev_ext = nullptr;

				if (bus == PX4_I2C_BUS_EXPANSION) {
					goto fail;
				}
			}
		}

#ifdef PX4_I2C_BUS_ONBOARD

		/* if this failed, attempt onboard sensor */
		if (bus == -1 || bus == PX4_I2C_BUS_ONBOARD) {
			if (g_dev_int != nullptr) {
				errx(0, "already started internal");
			}

			g_dev_int = new LidarLiteI2C(PX4_I2C_BUS_ONBOARD, LL40LS_DEVICE_PATH_INT);

			if (g_dev_int != nullptr && OK != g_dev_int->init()) {
				/* tear down the failing onboard instance */
				delete g_dev_int;
				g_dev_int = nullptr;

				if (bus == PX4_I2C_BUS_ONBOARD) {
					goto fail;
				}
			}

			if (g_dev_int == nullptr && bus == PX4_I2C_BUS_ONBOARD) {
				goto fail;
			}
		}

#endif

		/* set the poll rate to default, starts automatic data collection */
		if (g_dev_int != nullptr) {
			int fd = open(LL40LS_DEVICE_PATH_INT, O_RDONLY);

			if (fd == -1) {
				goto fail;
			}

			int ret = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
			close(fd);

			if (ret < 0) {
				goto fail;
			}
		}

		if (g_dev_ext != nullptr) {
			int fd = open(LL40LS_DEVICE_PATH_EXT, O_RDONLY);

			if (fd == -1) {
				goto fail;
			}

			int ret = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
			close(fd);

			if (ret < 0) {
				goto fail;
			}
		}

	} else {
		g_dev_pwm = new LidarLitePWM(LL40LS_DEVICE_PATH_PWM);

		if (g_dev_pwm != nullptr && OK != g_dev_pwm->init()) {
			delete g_dev_pwm;
			g_dev_pwm = nullptr;
			warnx("failed to init PWM");
		}

		if (g_dev_pwm != nullptr) {
			int fd = open(LL40LS_DEVICE_PATH_PWM, O_RDONLY);

			if (fd == -1) {
				warnx("fd nothing");
				goto fail;
			}

			int ret = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
			close(fd);

			if (ret < 0) {
				warnx("pollrate fail");
				goto fail;
			}
		}
	}

	exit(0);

fail:

#ifdef PX4_I2C_BUS_ONBOARD

	if (g_dev_int != nullptr && (bus == -1 || bus == PX4_I2C_BUS_ONBOARD)) {
		delete g_dev_int;
		g_dev_int = nullptr;
	}

#endif

	if (g_dev_ext != nullptr && (bus == -1 || bus == PX4_I2C_BUS_EXPANSION)) {
		delete g_dev_ext;
		g_dev_ext = nullptr;
	}

	if (g_dev_pwm != nullptr) {
		delete g_dev_pwm;
		g_dev_pwm = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop(const bool use_i2c, const int bus)
{
	if (use_i2c) {
		if (bus == PX4_I2C_BUS_EXPANSION) {
			if (g_dev_ext != nullptr) {
				delete g_dev_ext;
				g_dev_ext = nullptr;
			}

		} else {
			if (g_dev_int != nullptr) {
				delete g_dev_int;
				g_dev_int = nullptr;
			}
		}

	} else {
		if (g_dev_pwm != nullptr)  {
			delete g_dev_pwm;
			g_dev_pwm = nullptr;
		}
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(const bool use_i2c, const int bus)
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	const char *path;

	if (use_i2c) {
		path = ((bus == PX4_I2C_BUS_EXPANSION) ? LL40LS_DEVICE_PATH_EXT : LL40LS_DEVICE_PATH_INT);

	} else {
		path = LL40LS_DEVICE_PATH_PWM;
	}

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed, is the driver running?", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("time:        %lld", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5 times and report each value */
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
		warnx("valid %u", (float)report.current_distance > report.min_distance
		      && (float)report.current_distance < report.max_distance ? 1 : 0);
		warnx("measurement: %0.3f m", (double)report.current_distance);
		warnx("time:        %lld", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(const bool use_i2c, const int bus)
{

	const char *path;

	if (use_i2c) {
		path = ((bus == PX4_I2C_BUS_EXPANSION) ? LL40LS_DEVICE_PATH_EXT : LL40LS_DEVICE_PATH_INT);

	} else {
		path = LL40LS_DEVICE_PATH_PWM;
	}

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
void
info(const bool use_i2c, const int bus)
{
	LidarLite *g_dev = get_dev(use_i2c, bus);
	printf("state @ %p\n", g_dev);
	g_dev->print_info();
	exit(0);
}

/**
 * Dump registers
 */
void
regdump(const bool use_i2c, const int bus)
{
	LidarLite *g_dev = get_dev(use_i2c, bus);
	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();
	exit(0);
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
	int bus = -1;

	while ((ch = getopt(argc, argv, "XI")) != EOF) {
		switch (ch) {
#ifdef PX4_I2C_BUS_ONBOARD

		case 'I':
			bus = PX4_I2C_BUS_ONBOARD;
			break;
#endif

		case 'X':
			bus = PX4_I2C_BUS_EXPANSION;
			break;

		default:
			ll40ls::usage();
			exit(0);
		}
	}

	/* default to I2C if no protocol is given */
	bool use_i2c = true;

	/* determine protocol first because it's needed next */
	if (argc > optind + 1) {
		const char *protocol = argv[optind + 1];

		if (!strcmp(protocol, "pwm")) {
			use_i2c = false;

		} else if (!strcmp(protocol, "i2c")) {
			use_i2c = true;

		} else {
			warnx("unknown protocol, choose pwm or i2c");
			ll40ls::usage();
			exit(0);
		}
	}

	/* now determine action */
	if (argc > optind) {
		const char *verb = argv[optind];

		/* Start/load the driver. */
		if (!strcmp(verb, "start")) {
			ll40ls::start(use_i2c, bus);
		}

		/* Stop the driver */
		if (!strcmp(verb, "stop")) {
			ll40ls::stop(use_i2c, bus);
		}

		/* Test the driver/device. */
		else if (!strcmp(verb, "test")) {
			ll40ls::test(use_i2c, bus);
		}

		/* Reset the driver. */
		else if (!strcmp(verb, "reset")) {
			ll40ls::reset(use_i2c, bus);
		}

		/* dump registers */
		else if (!strcmp(verb, "regdump")) {
			ll40ls::regdump(use_i2c, bus);
		}

		/* Print driver information. */
		else if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
			ll40ls::info(use_i2c, bus);
		}

		else {
			ll40ls::usage();
			exit(0);
		}
	}

	warnx("unrecognized command, try 'start', 'test', 'reset', 'info' or 'regdump'");
	ll40ls::usage();
	exit(0);
}

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
#include <px4_getopt.h>

#define LL40LS_DEVICE_PATH_PWM  "/dev/ll40ls_pwm"

enum LL40LS_BUS {
	LL40LS_BUS_I2C_ALL = 0,
	LL40LS_BUS_I2C_INTERNAL,
	LL40LS_BUS_I2C_EXTERNAL,
	LL40LS_BUS_PWM
};

static constexpr struct ll40ls_bus_option {
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
#ifdef PX4_I2C_BUS_EXPANSION2
	{ LL40LS_BUS_I2C_EXTERNAL, "/dev/ll40ls_ext2", PX4_I2C_BUS_EXPANSION2 },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ LL40LS_BUS_I2C_INTERNAL, "/dev/ll40ls_int", PX4_I2C_BUS_ONBOARD },
#endif
};

/**
 * @brief Driver 'main' command.
 */
extern "C" __EXPORT int ll40ls_main(int argc, char *argv[]);


/**
 * @brief Local functions in support of the shell command.
 */
namespace ll40ls
{

LidarLite *instance = nullptr;

void    start(enum LL40LS_BUS busid, uint8_t rotation);
void    stop();
void    test();
void    reset();
void    info();
void    regdump();
void    usage();

/**
 * @brief Starts the driver.
 */
void start(enum LL40LS_BUS busid, uint8_t rotation)
{
	int fd, ret;

	if (instance) {
		PX4_INFO("driver already started");
		return;
	}

	if (busid == LL40LS_BUS_PWM) {
		instance = new LidarLitePWM(LL40LS_DEVICE_PATH_PWM, rotation);

		if (!instance) {
			PX4_ERR("Failed to instantiate LidarLitePWM");
			return;
		}

		if (instance->init() != PX4_OK) {
			PX4_ERR("failed to initialize LidarLitePWM");
			stop();
			return;
		}

	} else {
		for (uint8_t i = 0; i < (sizeof(bus_options) / sizeof(bus_options[0])); i++) {
			if (busid != LL40LS_BUS_I2C_ALL && busid != bus_options[i].busid) {
				continue;
			}

			instance = new LidarLiteI2C(bus_options[i].busnum, bus_options[i].devname, rotation);

			if (!instance) {
				PX4_ERR("Failed to instantiate LidarLiteI2C");
				return;
			}

			if (instance->init() == PX4_OK) {
				break;
			}

			PX4_ERR("failed to initialize LidarLiteI2C on busnum=%u", bus_options[i].busnum);
			stop();
		}
	}

	if (!instance) {
		PX4_WARN("No LidarLite found");
		return;
	}

	fd = px4_open(instance->get_dev_name(), O_RDONLY);

	if (fd == -1) {
		PX4_ERR("Error opening fd");
		stop();
		return;
	}

	ret = px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
	px4_close(fd);

	if (ret < 0) {
		PX4_ERR("pollrate fail");
		stop();
		return;
	}
}

/**
 * @brief Stops the driver
 */
void stop()
{
	delete instance;
	instance = nullptr;
}

/**
 * @brief Performs some basic functional tests on the driver;
 *        make sure we can collect data from the sensor in polled
 *        and automatic modes.
 */
void
test()
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	if (!instance) {
		PX4_ERR("No ll40ls driver running");
		return;
	}

	int fd = px4_open(instance->get_dev_name(), O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Error opening fd");
		return;
	}

	/* Do a simple demand read. */
	sz = px4_read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return;
	}

	print_message(report);

	/* Start the sensor polling at 2Hz. */
	if (PX4_OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return;
	}

	/* Read the sensor 5 times and report each value. */
	for (unsigned i = 0; i < 5; i++) {
		px4_pollfd_struct_t fds;

		/* Wait for data to be ready. */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_WARN("timed out waiting for sensor data");
			return;
		}

		/* Now go get it. */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_WARN("periodic read failed");
			return;
		}

		print_message(report);
	}

	/* Reset the sensor polling to default rate. */
	if (PX4_OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_WARN("failed to set default poll rate");
	}

	px4_close(fd);
}

/**
 * @brief Resets the driver.
 */
void
reset()
{
	if (!instance) {
		PX4_WARN("No ll40ls driver running");
		return;
	}

	int fd = px4_open(instance->get_dev_name(), O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Error opening fd");
		return;
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		px4_close(fd);
		return;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		px4_close(fd);
		return;
	}
}

/**
 * @brief Prints status info about the driver.
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
 * @brief Dumps the register information.
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

/**
 * @brief Displays driver usage at the console.
 */
void
usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'info', 'test', 'reset', 'info' or 'regdump' [i2c|pwm]");
	PX4_INFO("options for I2C:");
	PX4_INFO("    -X only external bus");
#ifdef PX4_I2C_BUS_ONBOARD
	PX4_INFO("    -I only internal bus");
#endif
	PX4_INFO("E.g. ll40ls start i2c -R 0");
}

} // namespace ll40ls

int
ll40ls_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	enum LL40LS_BUS busid = LL40LS_BUS_I2C_ALL;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "IXR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
#ifdef PX4_I2C_BUS_ONBOARD

		case 'I':
			busid = LL40LS_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = LL40LS_BUS_I2C_EXTERNAL;
			break;

		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting Lidar orientation to %d", (int)rotation);
			break;

		default:
			ll40ls::usage();
			return 0;
		}
	}

	/* Determine protocol first because it's needed next. */
	if (argc > myoptind + 1) {
		const char *protocol = argv[myoptind + 1];

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

	/* Now determine action. */
	if (argc > myoptind) {
		const char *verb = argv[myoptind];

		if (!strcmp(verb, "start")) {
			ll40ls::start(busid, rotation);

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

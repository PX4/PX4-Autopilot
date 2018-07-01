/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "RM3100.hpp"

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int rm3100_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace rm3100
{

/*
  list of supported bus configurations
 */
struct rm3100_bus_option {
	enum RM3100_BUS busid;
	const char *devpath;
	uint8_t busnum;
	RM3100 *dev;
} bus_options[] = {
	{ RM3100_BUS_I2C_EXTERNAL, "/dev/rm3100_ext", PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_EXPANSION1
	{ RM3100_BUS_I2C_EXTERNAL1, "/dev/ist8311_ext1", PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ RM3100_BUS_I2C_EXTERNAL2, "/dev/ist8312_ext2", PX4_I2C_BUS_EXPANSION2, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ RM3100_BUS_I2C_INTERNAL, "/dev/rm3100_int", PX4_I2C_BUS_ONBOARD, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void    start(enum RM3100_BUS busid, int address, enum Rotation rotation);
bool    start_bus(struct rm3100_bus_option &bus, int address, enum Rotation rotation);
struct rm3100_bus_option &find_bus(enum RM3100_BUS busid);
void    test(enum RM3100_BUS busid);
void    reset(enum RM3100_BUS busid);
int info(enum RM3100_BUS busid);
int calibrate(enum RM3100_BUS busid);
void    usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct rm3100_bus_option &bus, int address, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	RM3100 *interface = new RM3100(bus.busnum, address,  bus.devpath, rotation);

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
start(enum RM3100_BUS busid, int address, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == RM3100_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != RM3100_BUS_ALL && bus_options[i].busid != busid) {
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
struct rm3100_bus_option &find_bus(enum RM3100_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == RM3100_BUS_ALL ||
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
test(enum RM3100_BUS busid)
{
	struct rm3100_bus_option &bus = find_bus(busid);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'rm3100 start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external");
	}

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

		print_message(report);
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
 * To check the RM3100L for proper operation, a self test feature in incorporated
 * in which the sensor will change the polarity on all 3 axis. The values with and
 * with and without selftest on shoult be compared and if the absolete value are equal
 * the IC is functional.
 */
int calibrate(enum RM3100_BUS busid)
{
	int ret;
	struct rm3100_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'rm3100 start' if the driver is not running", path);
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
reset(enum RM3100_BUS busid)
{
	struct rm3100_bus_option &bus = find_bus(busid);
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
info(enum RM3100_BUS busid)
{
	struct rm3100_bus_option &bus = find_bus(busid);

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
	PX4_INFO("    -a 12C Address (0x%02x)", RM3100_BUS_I2C_ADDR);
	PX4_INFO("    -b 12C bus (%d-%d)", RM3100_BUS_I2C_EXTERNAL, RM3100_BUS_I2C_INTERNAL);
}

} // namespace

int
rm3100_main(int argc, char *argv[])
{
	RM3100_BUS i2c_busid = RM3100_BUS_ALL;
	int i2c_addr = RM3100_BUS_I2C_ADDR; /* 7bit */

	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:Ca:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'a':
			i2c_addr = (int)strtol(myoptarg, NULL, 0);
			break;

		case 'b':
			i2c_busid = (RM3100_BUS)strtol(myoptarg, NULL, 0);
			break;

		case 'C':
			calibrate = true;
			break;

		default:
			rm3100::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		rm3100::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		rm3100::start(i2c_busid, i2c_addr, rotation);

		if (i2c_busid == RM3100_BUS_ALL) {
			PX4_ERR("calibration only feasible against one bus");
			return 1;

		} else if (calibrate && (rm3100::calibrate(i2c_busid) != 0)) {
			PX4_ERR("calibration failed");
			return 1;
		}

		return 0;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		rm3100::test(i2c_busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		rm3100::reset(i2c_busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		rm3100::info(i2c_busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (rm3100::calibrate(i2c_busid) == 0) {
			PX4_INFO("calibration successful");
			return 0;

		} else {
			PX4_ERR("calibration failed");
			return 1;
		}
	}

	rm3100::usage();
	return 1;
}

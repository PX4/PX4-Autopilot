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
 * @file lis3mdl_main.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 */

#include "lis3mdl_main.h"
#include <px4_getopt.h>

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int lis3mdl_main(int argc, char *argv[]);

int
lis3mdl::calibrate(LIS3MDL_BUS bus_id)
{
	int ret;
	struct lis3mdl_bus_option &bus = find_bus(bus_id);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("%s open failed (try 'lis3mdl start' if the driver is not running", path);
		return 1;
	}

	if ((ret = ioctl(fd, MAGIOCCALIBRATE, fd)) != OK) {
		PX4_WARN("failed to enable sensor calibration mode");
	}

	close(fd);

	return ret;
}

int
lis3mdl::info(LIS3MDL_BUS bus_id)
{
	struct lis3mdl_bus_option &bus = find_bus(bus_id);

	PX4_WARN("running on bus: %u (%s)\n", (unsigned)bus.bus_id, bus.devpath);
	bus.dev->print_info();
	return 1;
}

bool
lis3mdl::init(LIS3MDL_BUS bus_id)
{
	struct lis3mdl_bus_option &bus = find_bus(bus_id);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
		return false;

	} else {
		PX4_INFO("Poll rate set to max (80hz)");
	}

	close(fd);

	return true;
}

bool
lis3mdl::start_bus(struct lis3mdl_bus_option &bus, Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
		return false;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.bus_id);
		return false;
	}

	bus.dev = new LIS3MDL(interface, bus.devpath, rotation);

	if (bus.dev != nullptr &&
	    bus.dev->init() != OK) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	return true;
}

int
lis3mdl::start(LIS3MDL_BUS bus_id, Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_id == LIS3MDL_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (bus_id != LIS3MDL_BUS_ALL && bus_options[i].bus_id != bus_id) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
		//init(bus_id);
	}

	return started;
}

int
lis3mdl::stop()
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

bool
lis3mdl::test(LIS3MDL_BUS bus_id)
{
	struct lis3mdl_bus_option &bus = find_bus(bus_id);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("%s open failed (try 'lis3mdl start')", path);
		return 1;
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_WARN("immediate read failed");
		return 1;
	}

	print_message(report);

	/* check if mag is onboard or external */
	if (ioctl(fd, MAGIOCGEXTERNAL, 0) < 0) {
		PX4_WARN("failed to get if mag is onboard or external");
		return 1;
	}

	/* set the queue depth to 5 */
	if (ioctl(fd, SENSORIOCSQUEUEDEPTH, 10) != OK) {
		PX4_WARN("failed to set queue depth");
		return 1;
	}

	/* start the sensor polling at 2Hz */
	if (ioctl(fd, SENSORIOCSPOLLRATE, 2) != OK) {
		PX4_WARN("failed to set 2Hz poll rate");
		return 1;
	}

	struct pollfd fds;

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_WARN("timed out waiting for sensor data");
			return 1;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_WARN("periodic read failed");
			return 1;
		}

		print_message(report);
	}

	PX4_INFO("PASS");
	return 1;
}

bool
lis3mdl::reset(LIS3MDL_BUS bus_id)
{
	struct lis3mdl_bus_option &bus = find_bus(bus_id);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("open failed ");
		return 1;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_WARN("driver reset failed");
		return 1;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_WARN("driver poll restart failed");
		return 1;
	}

	return 0;
}

void
lis3mdl::usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'reset', 'info', 'calibrate'");
	PX4_WARN("options:");
	PX4_WARN("    -R rotation");
	PX4_WARN("    -C calibrate on start");
	PX4_WARN("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)
	PX4_WARN("    -I only internal bus");
#endif
}

int
lis3mdl_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	bool calibrate = false;
	enum LIS3MDL_BUS bus_id = LIS3MDL_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XISR:CT", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)

		case 'I':
			bus_id = LIS3MDL_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			bus_id = LIS3MDL_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			bus_id = LIS3MDL_BUS_SPI;
			break;

		case 'C':
			calibrate = true;
			break;

		default:
			lis3mdl::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		lis3mdl::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	// Start/load the driver
	if (!strcmp(verb, "start")) {

		if (lis3mdl::start(bus_id, rotation)) {
			if (calibrate) {
				if (OK != lis3mdl::calibrate(bus_id)) {
					PX4_WARN("calibration failed");
					return 1;
				}
			}

			lis3mdl::init(bus_id);

			return 0;

		} else {
			return 1;
		}
	}

	// Stop the driver
	if (!strcmp(verb, "stop")) {
		return lis3mdl::stop();
	}

	// Test the driver/device
	if (!strcmp(verb, "test")) {
		return lis3mdl::test(bus_id);
	}

	// Reset the driver
	if (!strcmp(verb, "reset")) {
		return lis3mdl::reset(bus_id);
	}

	// Print driver information
	if (!strcmp(verb, "info") ||
	    !strcmp(verb, "status")) {
		return lis3mdl::info(bus_id);
	}

	// Autocalibrate the scaling
	if (!strcmp(verb, "calibrate")) {
		if (lis3mdl::calibrate(bus_id) == 0) {
			PX4_INFO("calibration successful");
			return 0;

		} else {
			PX4_INFO("calibration failed");
			return 1;
		}

	}

	PX4_INFO("unrecognized command, try 'start', 'test', 'reset', 'calibrate' 'or 'info'");
	return 1;
}

struct
lis3mdl::lis3mdl_bus_option &lis3mdl::find_bus(LIS3MDL_BUS bus_id)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((bus_id == LIS3MDL_BUS_ALL ||
		     bus_id == bus_options[i].bus_id) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)bus_id);
}

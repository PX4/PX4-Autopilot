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

#include "LPS22HB.hpp"

#include <px4_getopt.h>

#include <cstring>

extern "C" __EXPORT int lps22hb_main(int argc, char *argv[]);

enum LPS22HB_BUS {
	LPS22HB_BUS_ALL = 0,
	LPS22HB_BUS_I2C_INTERNAL,
	LPS22HB_BUS_I2C_EXTERNAL,
	LPS22HB_BUS_SPI
};

/**
 * Local functions in support of the shell command.
 */
namespace lps22hb
{

struct lps22hb_bus_option {
	enum LPS22HB_BUS busid;
	const char *devpath;
	LPS22HB_constructor interface_constructor;
	uint8_t busnum;
	LPS22HB	*dev;
} bus_options[] = {
	{ LPS22HB_BUS_I2C_EXTERNAL, "/dev/lps22hb_ext", &LPS22HB_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_EXPANSION1
	{ LPS22HB_BUS_I2C_EXTERNAL, "/dev/lps22hb_ext1", &LPS22HB_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ LPS22HB_BUS_I2C_EXTERNAL, "/dev/lps22hb_ext2", &LPS22HB_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ LPS22HB_BUS_I2C_INTERNAL, "/dev/lps22hb_int", &LPS22HB_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_LPS22HB
	{ LPS22HB_BUS_SPI, "/dev/lps22hb_spi", &LPS22HB_SPI_interface, PX4_SPI_BUS_SENSOR4, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

int		start(enum LPS22HB_BUS busid);
bool	start_bus(struct lps22hb_bus_option &bus);
struct lps22hb_bus_option &find_bus(enum LPS22HB_BUS busid);
int		reset(enum LPS22HB_BUS busid);
int		info();
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct lps22hb_bus_option &bus)
{
	PX4_INFO("starting %s", bus.devpath);

	if (bus.dev != nullptr) {
		PX4_WARN("bus option already started");
		return false;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new LPS22HB(interface, bus.devpath);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		PX4_WARN("init failed");
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	int fd = px4_open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		PX4_ERR("can't open baro device");
		return false;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		px4_close(fd);
		PX4_ERR("failed setting default poll rate");
		return false;
	}

	px4_close(fd);

	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
int
start(enum LPS22HB_BUS busid)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == LPS22HB_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != LPS22HB_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * find a bus structure for a busid
 */
struct lps22hb_bus_option &find_bus(enum LPS22HB_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == LPS22HB_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * Reset the driver.
 */
int
reset(enum LPS22HB_BUS busid)
{
	struct lps22hb_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct lps22hb_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			warnx("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	return 0;
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'reset'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external I2C bus)");
	PX4_INFO("    -I    (internal I2C bus)");
	PX4_INFO("    -S    (external SPI bus)");
}

} // namespace

int
lps22hb_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	enum LPS22HB_BUS busid = LPS22HB_BUS_ALL;

	while ((ch = px4_getopt(argc, argv, "IXS", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
#if (PX4_I2C_BUS_ONBOARD)

		case 'I':
			busid = LPS22HB_BUS_I2C_INTERNAL;
			break;
#endif /* PX4_I2C_BUS_ONBOARD */

		case 'X':
			busid = LPS22HB_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = LPS22HB_BUS_SPI;
			break;

		default:
			lps22hb::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		lps22hb::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		return lps22hb::start(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		return lps22hb::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		return lps22hb::info();
	}

	PX4_WARN("unrecognised command, try 'start', 'reset' or 'info'");
	return 0;
}

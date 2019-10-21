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

#include <px4_getopt.h>

#include "bmp388.h"

enum BMP388_BUS {
	BMP388_BUS_ALL = 0,
	BMP388_BUS_I2C_INTERNAL,
	BMP388_BUS_I2C_INTERNAL1,
	BMP388_BUS_I2C_EXTERNAL,
	BMP388_BUS_SPI_INTERNAL,
	BMP388_BUS_SPI_EXTERNAL
};

/**
 * Local functions in support of the shell command.
 */
namespace bmp388
{

/*
 * list of supported bus configurations
 */
struct bmp388_bus_option {
	enum BMP388_BUS busid;
	const char *devpath;
	BMP388_constructor interface_constructor;
	uint8_t busnum;
	uint32_t device;
	bool external;
	BMP388 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ BMP388_BUS_SPI_EXTERNAL, "/dev/bmp388_spi_ext", &bmp388_spi_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BARO, true, NULL },
#endif
#if defined(PX4_SPIDEV_BARO)
#  if defined(PX4_SPIDEV_BARO_BUS)
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPIDEV_BARO_BUS, PX4_SPIDEV_BARO, false, NULL },
#  else
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BARO, false, NULL },
#  endif
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_BMP388)
	{ BMP388_BUS_I2C_INTERNAL, "/dev/bmp388_i2c_int", &bmp388_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_BMP388, false, NULL },
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV1_BMP388)
	{ BMP388_BUS_I2C_INTERNAL1, "/dev/bmp388_i2c_int1", &bmp388_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV1_BMP388, false, NULL },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_OBDEV_BMP388)
	{ BMP388_BUS_I2C_EXTERNAL, "/dev/bmp388_i2c_ext", &bmp388_i2c_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_BMP388, true, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

/**
 * Start the driver.
 */
bool
start_bus(struct bmp388_bus_option &bus)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		exit(1);
	}

	IBMP388 *interface = bus.interface_constructor(bus.busnum, bus.device, bus.external);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new BMP388(interface, bus.devpath);

	if (bus.dev == nullptr) {
		return false;
	}

	if (OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum BMP388_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == BMP388_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != BMP388_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		PX4_WARN("bus option number is %d", i);
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct bmp388_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_WARN("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external I2C bus TODO)");
	PX4_WARN("    -I    (internal I2C bus TODO)");
	PX4_WARN("    -S    (external SPI bus)");
	PX4_WARN("    -s    (internal SPI bus)");
}

} // namespace

extern "C" __EXPORT int bmp388_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	enum BMP388_BUS busid = BMP388_BUS_ALL;

	while ((ch = px4_getopt(argc, argv, "XIJSs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = BMP388_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = BMP388_BUS_I2C_INTERNAL;
			break;

		case 'J':
			busid = BMP388_BUS_I2C_INTERNAL1;
			break;

		case 'S':
			busid = BMP388_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = BMP388_BUS_SPI_INTERNAL;
			break;

		default:
			bmp388::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmp388::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmp388::start(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmp388::info();
	}

	PX4_ERR("unrecognized command, try 'start' or 'info'");
	return -1;
}

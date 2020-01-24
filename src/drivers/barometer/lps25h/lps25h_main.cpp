/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include "LPS25H.hpp"

enum class LPS25H_BUS {
	ALL = 0,
	I2C_INTERNAL,
	I2C_EXTERNAL,
	SPI_INTERNAL,
	SPI_EXTERNAL
};

namespace lps25h
{

struct lps25h_bus_option {
	LPS25H_BUS busid;
	LPS25H_constructor interface_constructor;
	uint8_t busnum;
	LPS25H	*dev;
} bus_options[] = {
	{ LPS25H_BUS::I2C_EXTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_EXPANSION, nullptr },
#if defined(PX4_I2C_BUS_EXPANSION1)
	{ LPS25H_BUS::I2C_EXTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_EXPANSION1, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION2)
	{ LPS25H_BUS::I2C_EXTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_EXPANSION2, nullptr },
#endif
#if defined(PX4_I2C_BUS_ONBOARD)
	{ LPS25H_BUS::I2C_INTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_ONBOARD, nullptr },
#endif
#if defined(PX4_SPI_BUS_SENSORS) && defined(PX4_SPIDEV_LPS22H)
	{ LPS25H_BUS::SPI_EXTERNAL, &LPS25H_SPI_interface, PX4_SPI_BUS_SENSORS, nullptr },
#endif
};

// find a bus structure for a busid
static struct lps25h_bus_option *find_bus(LPS25H_BUS busid)
{
	for (lps25h_bus_option &bus_option : bus_options) {
		if ((busid == LPS25H_BUS::ALL ||
		     busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(lps25h_bus_option &bus)
{
	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		delete interface;
		return false;
	}

	LPS25H *dev = new LPS25H(interface);

	if (dev == nullptr) {
		PX4_ERR("alloc failed");
		return false;
	}

	if (dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete dev;
		delete interface;
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(LPS25H_BUS busid)
{
	for (lps25h_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != LPS25H_BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option)) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static int stop(LPS25H_BUS busid)
{
	lps25h_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status(LPS25H_BUS busid)
{
	lps25h_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		bus->dev->print_info();
		return PX4_OK;
	}

	PX4_WARN("driver not running");
	return PX4_ERROR;
}

static int usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'status'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (i2c external bus)");
	PX4_INFO("    -I    (i2c internal bus)");
	PX4_INFO("    -s    (spi internal bus)");
	PX4_INFO("    -S    (spi external bus)");

	return 0;
}

} // namespace

extern "C" int lps25h_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	LPS25H_BUS busid = LPS25H_BUS::ALL;

	while ((ch = px4_getopt(argc, argv, "XISs:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = LPS25H_BUS::I2C_EXTERNAL;
			break;

		case 'I':
			busid = LPS25H_BUS::I2C_INTERNAL;
			break;

		case 'S':
			busid = LPS25H_BUS::SPI_EXTERNAL;
			break;

		case 's':
			busid = LPS25H_BUS::SPI_INTERNAL;
			break;

		default:
			return lps25h::usage();
		}
	}

	if (myoptind >= argc) {
		return lps25h::usage();
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		lps25h::start(busid);

	} else if (!strcmp(verb, "stop")) {
		return lps25h::stop(busid);

	} else if (!strcmp(verb, "status")) {
		lps25h::status(busid);
	}

	return lps25h::usage();
}

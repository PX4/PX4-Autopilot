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

#include "DPS310.hpp"

#include <px4_platform_common/getopt.h>

namespace dps310
{

enum class DPS310_BUS {
	ALL = 0,
	I2C_INTERNAL,
	I2C_EXTERNAL,
	SPI_INTERNAL,
	SPI_EXTERNAL
};

extern device::Device *DPS310_SPI_interface(uint8_t bus, uint32_t device);
extern device::Device *DPS310_I2C_interface(uint8_t bus, uint32_t device);
typedef device::Device *(*DPS310_constructor)(uint8_t, uint32_t);

struct dps310_bus_option {
	DPS310_BUS busid;
	DPS310_constructor interface_constructor;
	uint8_t busnum;
	uint32_t address;
	DPS310	*dev;
} bus_options[] = {
#if defined(PX4_SPI_BUS_2) && defined(PX4_SPIDEV_BARO)
	{ DPS310_BUS::SPI_INTERNAL, &DPS310_SPI_interface, PX4_SPI_BUS_2, PX4_SPIDEV_BARO, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_OBDEV_DPS310)
	{ DPS310_BUS::I2C_EXTERNAL, &DPS310_I2C_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_DPS310, nullptr },
#endif
};

// find a bus structure for a busid
static dps310_bus_option *find_bus(DPS310_BUS busid)
{
	for (dps310_bus_option &bus_option : bus_options) {
		if ((busid == DPS310_BUS::ALL ||
		     busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(dps310_bus_option &bus)
{
	device::Device *interface = bus.interface_constructor(bus.busnum, bus.address);

	if ((interface == nullptr) || (interface->init() != PX4_OK)) {
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		delete interface;
		return false;
	}

	DPS310 *dev = new DPS310(interface);

	if (dev == nullptr || dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete dev;
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(DPS310_BUS busid)
{
	for (dps310_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != DPS310_BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option)) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static int stop(DPS310_BUS busid)
{
	dps310_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status(DPS310_BUS busid)
{
	dps310_bus_option *bus = find_bus(busid);

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

} // namespace dsp310

extern "C" int dps310_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	using namespace dps310;

	DPS310_BUS busid = DPS310_BUS::ALL;

	while ((ch = px4_getopt(argc, argv, "XISs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = DPS310_BUS::I2C_EXTERNAL;
			break;

		case 'I':
			busid = DPS310_BUS::I2C_INTERNAL;
			break;

		case 'S':
			busid = DPS310_BUS::SPI_EXTERNAL;
			break;

		case 's':
			busid = DPS310_BUS::SPI_INTERNAL;
			break;

		default:
			return usage();
		}
	}

	if (myoptind >= argc) {
		return usage();
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return start(busid);

	} else if (!strcmp(verb, "stop")) {
		return stop(busid);

	} else if (!strcmp(verb, "status")) {
		return status(busid);
	}

	return usage();
}

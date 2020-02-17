/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "MPL3115A2.hpp"

#include <px4_platform_common/getopt.h>

enum class MPL3115A2_BUS {
	ALL = 0,
	I2C_INTERNAL,
	I2C_EXTERNAL,
};

namespace mpl3115a2
{

struct mpl3115a2_bus_option {
	MPL3115A2_BUS busid;
	uint8_t busnum;
	MPL3115A2 *dev;
} bus_options[] = {
#if defined(PX4_I2C_BUS_ONBOARD)
	{ MPL3115A2_BUS::I2C_INTERNAL, PX4_I2C_BUS_ONBOARD, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION)
	{ MPL3115A2_BUS::I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION1)
	{ MPL3115A2_BUS::I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION1, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION2)
	{ MPL3115A2_BUS::I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION2, nullptr },
#endif
};

// find a bus structure for a busid
static mpl3115a2_bus_option *find_bus(MPL3115A2_BUS busid)
{
	for (mpl3115a2_bus_option &bus_option : bus_options) {
		if ((busid == MPL3115A2_BUS::ALL ||
		     busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(mpl3115a2_bus_option &bus)
{
	MPL3115A2 *dev = new MPL3115A2(bus.busnum);

	if (dev == nullptr || (dev->init() != PX4_OK)) {
		PX4_ERR("driver start failed");
		delete dev;
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(MPL3115A2_BUS busid)
{
	for (mpl3115a2_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != MPL3115A2_BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option)) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static int stop(MPL3115A2_BUS busid)
{
	mpl3115a2_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status(MPL3115A2_BUS busid)
{
	mpl3115a2_bus_option *bus = find_bus(busid);

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

	return 0;
}

} // namespace

extern "C" int mpl3115a2_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	MPL3115A2_BUS busid = MPL3115A2_BUS::ALL;

	while ((ch = px4_getopt(argc, argv, "XI", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MPL3115A2_BUS::I2C_EXTERNAL;
			break;

		case 'I':
			busid = MPL3115A2_BUS::I2C_INTERNAL;
			break;

		default:
			return mpl3115a2::usage();
		}
	}

	if (myoptind >= argc) {
		return mpl3115a2::usage();
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return mpl3115a2::start(busid);

	} else if (!strcmp(verb, "stop")) {
		return mpl3115a2::stop(busid);

	} else if (!strcmp(verb, "status")) {
		return mpl3115a2::status(busid);
	}

	return mpl3115a2::usage();
}

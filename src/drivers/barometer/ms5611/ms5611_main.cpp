/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "MS5611.hpp"
#include "ms5611.h"

enum class MS5611_BUS {
	ALL = 0,
	I2C_INTERNAL,
	I2C_EXTERNAL,
	SPI_INTERNAL,
	SPI_EXTERNAL
};

namespace ms5611
{

// list of supported bus configurations
struct ms5611_bus_option {
	MS5611_BUS busid;
	MS5611_constructor interface_constructor;
	uint8_t busnum;
	MS5611 *dev;
} bus_options[] = {
#if defined(PX4_I2C_BUS_ONBOARD)
	{ MS5611_BUS::I2C_INTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_ONBOARD, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION)
	{ MS5611_BUS::I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION1)
	{ MS5611_BUS::I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION1, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION2)
	{ MS5611_BUS::I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION2, nullptr },
#endif
#if defined(PX4_SPI_BUS_BARO) && defined(PX4_SPIDEV_BARO)
	{ MS5611_BUS::SPI_INTERNAL, &MS5611_spi_interface, PX4_SPI_BUS_BARO, nullptr },
#endif
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_BARO)
	{ MS5611_BUS::SPI_EXTERNAL, &MS5611_spi_interface, PX4_SPI_BUS_EXT, nullptr },
#endif
};

// find a bus structure for a busid
static struct ms5611_bus_option *find_bus(MS5611_BUS busid)
{
	for (ms5611_bus_option &bus_option : bus_options) {
		if ((busid == MS5611_BUS::ALL ||
		     busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(ms5611_bus_option &bus, enum MS56XX_DEVICE_TYPES device_type)
{
	prom_u prom_buf;
	device::Device *interface = bus.interface_constructor(prom_buf, bus.busnum);

	if ((interface == nullptr) || (interface->init() != PX4_OK)) {
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		delete interface;
		return false;
	}

	MS5611 *dev = new MS5611(interface, prom_buf, device_type);

	if (dev == nullptr) {
		PX4_ERR("alloc failed");
		return false;
	}

	if (dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete dev;
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(MS5611_BUS busid, enum MS56XX_DEVICE_TYPES device_type)
{
	for (ms5611_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != MS5611_BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option, device_type)) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static int stop(MS5611_BUS busid)
{
	ms5611_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status(MS5611_BUS busid)
{
	ms5611_bus_option *bus = find_bus(busid);

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
	PX4_INFO("    -T    5611|5607 (default 5611)");
	PX4_INFO("    -T    0 (autodetect version)");

	return 0;
}

} // namespace

extern "C" int ms5611_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	MS5611_BUS busid = MS5611_BUS::ALL;
	enum MS56XX_DEVICE_TYPES device_type = MS5611_DEVICE;

	while ((ch = px4_getopt(argc, argv, "XISsT:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MS5611_BUS::I2C_EXTERNAL;
			break;

		case 'I':
			busid = MS5611_BUS::I2C_INTERNAL;
			break;

		case 'S':
			busid = MS5611_BUS::SPI_EXTERNAL;
			break;

		case 's':
			busid = MS5611_BUS::SPI_INTERNAL;
			break;

		case 'T': {
				int val = atoi(myoptarg);

				if (val == 5611) {
					device_type = MS5611_DEVICE;

				} else if (val == 5607) {
					device_type = MS5607_DEVICE;

				} else if (val == 0) {
					device_type = MS56XX_DEVICE;
				}
			}
			break;

		default:
			return ms5611::usage();
		}
	}

	if (myoptind >= argc) {
		return ms5611::usage();
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return ms5611::start(busid, device_type);

	} else if (!strcmp(verb, "stop")) {
		return ms5611::stop(busid);

	} else if (!strcmp(verb, "status")) {
		return ms5611::status(busid);
	}

	return ms5611::usage();
}

/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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

#include "BMP280.hpp"

enum class BMP280_BUS {
	ALL = 0,
	I2C_INTERNAL,
	I2C_EXTERNAL,
	SPI_INTERNAL,
	SPI_EXTERNAL
};

namespace bmp280
{

// list of supported bus configurations
struct bmp280_bus_option {
	BMP280_BUS busid;
	BMP280_constructor interface_constructor;
	uint8_t busnum;
	uint32_t address;
	BMP280	*dev;
} bus_options[] = {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_BARO)
	{ BMP280_BUS::SPI_EXTERNAL, &bmp280_spi_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BARO, nullptr },
#endif
#if defined(PX4_SPIDEV_BARO_BUS) && defined(PX4_SPIDEV_BARO)
	{ BMP280_BUS::SPI_INTERNAL, &bmp280_spi_interface, PX4_SPIDEV_BARO_BUS, PX4_SPIDEV_BARO, nullptr },
#elif defined(PX4_SPI_BUS_SENSORS) && defined(PX4_SPIDEV_BARO)
	{ BMP280_BUS::SPI_INTERNAL, &bmp280_spi_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BARO, nullptr },
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_BMP280)
	{ BMP280_BUS::I2C_INTERNAL, &bmp280_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_BMP280, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_OBDEV_BMP280)
	{ BMP280_BUS::I2C_EXTERNAL, &bmp280_i2c_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_BMP280, nullptr },
#endif
};

// find a bus structure for a busid
static struct bmp280_bus_option *find_bus(BMP280_BUS busid)
{
	for (bmp280_bus_option &bus_option : bus_options) {
		if ((busid == BMP280_BUS::ALL ||
		     busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(bmp280_bus_option &bus)
{
	bmp280::IBMP280 *interface = bus.interface_constructor(bus.busnum, bus.address);

	if ((interface == nullptr) || (interface->init() != PX4_OK)) {
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		delete interface;
		return false;
	}

	BMP280 *dev = new BMP280(interface);

	if (dev == nullptr) {
		PX4_ERR("driver allocate failed");
		delete interface;
		return false;
	}

	if (dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete dev;	// BMP280 deletes the interface
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(BMP280_BUS busid)
{
	for (bmp280_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != BMP280_BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option)) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static int stop(BMP280_BUS busid)
{
	bmp280_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status(BMP280_BUS busid)
{
	bmp280_bus_option *bus = find_bus(busid);

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

extern "C" int bmp280_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	BMP280_BUS busid = BMP280_BUS::ALL;

	while ((ch = px4_getopt(argc, argv, "XISs:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = BMP280_BUS::I2C_EXTERNAL;
			break;

		case 'I':
			busid = BMP280_BUS::I2C_INTERNAL;
			break;

		case 'S':
			busid = BMP280_BUS::SPI_EXTERNAL;
			break;

		case 's':
			busid = BMP280_BUS::SPI_INTERNAL;
			break;

		default:
			return bmp280::usage();
		}
	}

	if (myoptind >= argc) {
		return bmp280::usage();
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return bmp280::start(busid);

	} else if (!strcmp(verb, "stop")) {
		return bmp280::stop(busid);

	} else if (!strcmp(verb, "status")) {
		return bmp280::status(busid);
	}

	return bmp280::usage();
}

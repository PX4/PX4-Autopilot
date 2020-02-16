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

#include "FXAS21002C.hpp"

extern "C" { __EXPORT int fxas21002c_main(int argc, char *argv[]); }

enum class FXAS21002C_BUS {
	ALL = 0,
	I2C_INTERNAL,
	I2C_EXTERNAL,
	SPI_INTERNAL,
	SPI_EXTERNAL
};

/**
 * Local functions in support of the shell command.
 */
namespace fxas21002c
{

// list of supported bus configurations
struct fxas21002c_bus_option {
	FXAS21002C_BUS busid;
	FXAS21002C_constructor interface_constructor;
	uint8_t busnum;
	uint32_t address;
    bool external;
	FXAS21002C	*dev;
} bus_options[] = {
#if defined(FXAS21002C_USE_I2C)

#  if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_FXAS21002C_ADDR)
	{ FXAS21002C_BUS::I2C_INTERNAL, &FXAS21002C_I2C_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_FXAS21002C_ADDR, false, nullptr },
#  endif

#endif

#if defined(PX4_SPI_BUS_SENSORS) && defined(PX4_SPIDEV_GYRO)
	{ FXAS21002C_BUS::SPI_INTERNAL, &FXAS21002C_SPI_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_GYRO, false, nullptr },
#endif

#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_GYRO)
	{ FXAS21002C_BUS::SPI_EXTERNAL, &FXAS21002C_SPI_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_GYRO, true, nullptr },
#endif
};

// find a bus structure for a busid
static fxas21002c_bus_option *find_bus(FXAS21002C_BUS busid) {
	for (fxas21002c_bus_option &bus_option : bus_options) {
		if ((busid == FXAS21002C_BUS::ALL ||
		     busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

int	start(FXAS21002C_BUS busid, enum Rotation rotation);
int	info(FXAS21002C_BUS busid);
int	regdump(FXAS21002C_BUS busid);
int	usage();
int	test_error(FXAS21002C_BUS busid);

static bool start_bus(fxas21002c_bus_option &bus, enum Rotation rotation) {
	device::Device *interface = bus.interface_constructor(bus.busnum, bus.address);

	if ((interface == nullptr) || (interface->init() != PX4_OK)) {
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		delete interface;
		return false;
	}

	FXAS21002C	*dev = new FXAS21002C(interface, rotation);

	if (dev == nullptr) {
		PX4_ERR("failed instantiating FXAS21002C obj");
		goto fail;
	}

	if (OK != dev->init()) {
		goto fail;
	}

    bus.dev = dev;
    return PX4_OK;
    
fail:

	if (dev != nullptr) {
		delete dev;
		dev = nullptr;
	}

    PX4_ERR("driver start failed");
    return PX4_ERROR;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
int
start(FXAS21002C_BUS busid, enum Rotation rotation){
	for (fxas21002c_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != FXAS21002C_BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option, rotation)) {
			return 0;
		}
	}

	return -1;
}

/**
 * Print a little info about the driver.
 */
int
info(FXAS21002C_BUS busid)
{
	fxas21002c_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
        printf("state @ %p\n", bus->dev);
		bus->dev->print_info();
		return 0;
	}

	PX4_WARN("driver not running");
	return -1;
}


/**
 * dump registers from device
 */
int
regdump(FXAS21002C_BUS busid)
{
	fxas21002c_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
        printf("regdump @ %p\n", bus->dev);
		bus->dev->print_registers();
		return 0;
	}

	PX4_WARN("driver not running");
	return -1;
}


/**
 * trigger an error
 */
int
test_error(FXAS21002C_BUS busid)
{
	fxas21002c_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
        printf("test_error @ %p\n", bus->dev);
		bus->dev->test_error();
		return 0;
	}

	PX4_WARN("driver not running");
	return -1;
}

int
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'testerror' or 'regdump'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (i2c external bus)");
	PX4_INFO("    -I    (i2c internal bus)");
	PX4_INFO("    -s    (spi internal bus)");
	PX4_INFO("    -S    (spi external bus)");
	PX4_INFO("    -R rotation");

        return 0;
}

} // namespace

int
fxas21002c_main(int argc, char *argv[])
{
	FXAS21002C_BUS busid = FXAS21002C_BUS::ALL;
	enum Rotation rotation = ROTATION_NONE;

	int ch = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

    while ((ch = px4_getopt(argc, argv, "XISsR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = FXAS21002C_BUS::I2C_EXTERNAL;
			break;

		case 'I':
			busid = FXAS21002C_BUS::I2C_INTERNAL;
			break;

		case 'S':
			busid = FXAS21002C_BUS::SPI_EXTERNAL;
			break;

		case 's':
			busid = FXAS21002C_BUS::SPI_INTERNAL;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			fxas21002c::usage();
			return 0;
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
        return fxas21002c::start(busid, rotation);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
        return fxas21002c::info(busid);
	}

	/*
	 * dump device registers
	 */
	if (!strcmp(verb, "regdump")) {
        return fxas21002c::regdump(busid);
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
        return fxas21002c::test_error(busid);
	}

	PX4_WARN("unrecognized command, try 'start', 'info', 'testerror' or 'regdump'");
    return PX4_ERROR;
}

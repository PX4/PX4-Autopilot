/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * Driver for the Invensense icm20948 connected via I2C or SPI.
 *
 * based on the mpu9250 driver
 */

#include <px4_platform_common/config.h>
#include <px4_platform_common/getopt.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/conversions.h>
#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <lib/conversion/rotation.h>

#include "icm20948.h"

#define ICM_DEVICE_PATH_EXT  "/dev/icm20948_ext"

/** driver 'main' command */
extern "C" { __EXPORT int icm20948_main(int argc, char *argv[]); }

enum ICM20948_BUS {
	ICM20948_BUS_ALL = 0,
	ICM20948_BUS_I2C_INTERNAL,
	ICM20948_BUS_I2C_EXTERNAL,
	ICM20948_BUS_SPI_INTERNAL,
//	ICM20948_BUS_SPI_INTERNAL2,
	ICM20948_BUS_SPI_EXTERNAL
};

/**
 * Local functions in support of the shell command.
 */
namespace icm20948
{

/*
  list of supported bus configurations
 */

struct icm20948_bus_option {
	enum ICM20948_BUS busid;
	const char *path;
	ICM20948_constructor interface_constructor;
	bool magpassthrough;
	uint8_t busnum;
	uint32_t address;
	ICM20948 *dev;
} bus_options[] = {

#if defined(PX4_SPIDEV_ICM_20948) && defined(PX4_SPI_BUS_1)
	{ ICM20948_BUS_SPI_INTERNAL, ICM_DEVICE_PATH_EXT, &ICM20948_SPI_interface, true, PX4_SPI_BUS_1, PX4_SPIDEV_ICM_20948, nullptr },
#endif

#if defined (USE_I2C)

#  if defined(PX4_I2C_BUS_EXPANSION)
	{ ICM20948_BUS_I2C_EXTERNAL, ICM_DEVICE_PATH_EXT, &ICM20948_I2C_interface, false, PX4_I2C_BUS_EXPANSION, PX4_I2C_EXT_ICM20948_1, nullptr },
#  endif

#endif

};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))


void	start(enum ICM20948_BUS busid, enum Rotation rotation, bool external_bus, bool magnetometer_only);
bool	start_bus(struct icm20948_bus_option &bus, enum Rotation rotation, bool external_bus, bool magnetometer_only);
struct icm20948_bus_option &find_bus(enum ICM20948_BUS busid);
void	stop(enum ICM20948_BUS busid);
void	info(enum ICM20948_BUS busid);
void	usage();

/**
 * find a bus structure for a busid
 */
struct icm20948_bus_option &find_bus(enum ICM20948_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == ICM20948_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != nullptr) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct icm20948_bus_option &bus, enum Rotation rotation, bool external, bool magnetometer_only)
{
	PX4_INFO("Bus probed: %d", bus.busid);

	if (bus.dev != nullptr) {
		warnx("%s SPI not available", external ? "External" : "Internal");
		return false;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum, bus.address, external);

	if (interface == nullptr) {
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	device::Device *mag_interface = nullptr;

#ifdef USE_I2C
	/* For i2c interfaces, connect to the magnetomer directly */
	bool is_i2c = bus.busid == ICM20948_BUS_I2C_INTERNAL || bus.busid == ICM20948_BUS_I2C_EXTERNAL;

	if (is_i2c) {
		mag_interface = AK8963_I2C_interface(bus.busnum, external);
	}

#endif

	bus.dev = new ICM20948(interface, mag_interface, bus.path, rotation, magnetometer_only);

	if (bus.dev == nullptr) {
		delete interface;

		if (mag_interface != nullptr) {
			delete mag_interface;
		}

		return false;
	}

	if (OK != bus.dev->init()) {
		goto fail;
	}

	return true;

fail:

	if (bus.dev != nullptr) {
		delete (bus.dev);
		bus.dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(enum ICM20948_BUS busid, enum Rotation rotation, bool external, bool magnetometer_only)
{

	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			// this device is already started
			continue;
		}

		if (busid != ICM20948_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation, external, magnetometer_only);

		if (started) { break; }
	}

	exit(started ? 0 : 1);

}

void
stop(enum ICM20948_BUS busid)
{
	struct icm20948_bus_option &bus = find_bus(busid);


	if (bus.dev != nullptr) {
		delete bus.dev;
		bus.dev = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(enum ICM20948_BUS busid)
{
	struct icm20948_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", bus.dev);
	bus.dev->print_info();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'stop',\n 'regdump', 'testerror'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (i2c external bus)");
	PX4_INFO("    -I    (i2c internal bus)");
	PX4_INFO("    -s    (spi internal bus)");
	PX4_INFO("    -S    (spi external bus)");
	PX4_INFO("    -t    (spi internal bus, 2nd instance)");
	PX4_INFO("    -R rotation");
	PX4_INFO("    -M only enable magnetometer, accel/gyro disabled - not av. on MPU6500");
}

} // namespace icm20948

int
icm20948_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	enum ICM20948_BUS busid = ICM20948_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool magnetometer_only = false;

	while ((ch = px4_getopt(argc, argv, "XISstMR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = ICM20948_BUS_I2C_EXTERNAL;
			break;

//		case 'I':
//			busid = ICM20948_BUS_I2C_INTERNAL;
//			break;

//		case 'S':
//			busid = ICM20948_BUS_SPI_EXTERNAL;
//			break;
//
//		case 's':
//			busid = ICM20948_BUS_SPI_INTERNAL;
//			break;
//
//		case 't':
//			busid = ICM20948_BUS_SPI_INTERNAL2;
//			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'M':
			magnetometer_only = true;
			break;

		default:
			icm20948::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		icm20948::usage();
		return -1;
	}

	bool external = busid == ICM20948_BUS_I2C_EXTERNAL || busid == ICM20948_BUS_SPI_EXTERNAL;
	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		icm20948::start(busid, rotation, external, magnetometer_only);
	}

	if (!strcmp(verb, "stop")) {
		icm20948::stop(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		icm20948::info(busid);
	}

	icm20948::usage();
	return 0;
}

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

#include "MPU6000.hpp"

/**
 * Local functions in support of the shell command.
 */
namespace mpu6000
{

/*
  list of supported bus configurations
 */

struct mpu6000_bus_option {
	enum MPU6000_BUS busid;
	MPU_DEVICE_TYPE device_type;
	const char *devpath;
	MPU6000_constructor interface_constructor;
	uint8_t busnum;
	bool external;
	MPU6000	*dev;
} bus_options[] = {
#if defined (USE_I2C)
#	if defined(PX4_I2C_BUS_ONBOARD)
	{ MPU6000_BUS_I2C_INTERNAL, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH,  &MPU6000_I2C_interface, PX4_I2C_BUS_ONBOARD, false, NULL },
#	endif
#	if defined(PX4_I2C_BUS_EXPANSION)
	{ MPU6000_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH_EXT, &MPU6000_I2C_interface, PX4_I2C_BUS_EXPANSION,  true, NULL },
#	endif
#	if defined(PX4_I2C_BUS_EXPANSION1)
	{ MPU6000_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH_EXT1, &MPU6000_I2C_interface, PX4_I2C_BUS_EXPANSION1,  true, NULL },
#	endif
#	if defined(PX4_I2C_BUS_EXPANSION2)
	{ MPU6000_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH_EXT2, &MPU6000_I2C_interface, PX4_I2C_BUS_EXPANSION2,  true, NULL },
#	endif
#endif
#ifdef PX4_SPIDEV_MPU
	{ MPU6000_BUS_SPI_INTERNAL1, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH, &MPU6000_SPI_interface, PX4_SPI_BUS_SENSORS,  false, NULL },
#endif
#if defined(PX4_SPI_BUS_EXT)
	{ MPU6000_BUS_SPI_EXTERNAL1, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH_EXT, &MPU6000_SPI_interface, PX4_SPI_BUS_EXT,  true, NULL },
#endif
#if defined(PX4_SPIDEV_ICM_20602) && defined(PX4_SPI_BUS_SENSORS)
	{ MPU6000_BUS_SPI_INTERNAL1, MPU_DEVICE_TYPE_ICM20602, ICM20602_DEVICE_PATH, &MPU6000_SPI_interface, PX4_SPI_BUS_SENSORS,  false, NULL },
#endif
#if defined(PX4_SPIDEV_ICM_20602) && defined(PX4_SPI_BUS_SENSORS1)
	{ MPU6000_BUS_SPI_INTERNAL1, MPU_DEVICE_TYPE_ICM20602, ICM20602_DEVICE_PATH, &MPU6000_SPI_interface, PX4_SPI_BUS_SENSORS1,  false, NULL },
#endif
#if defined(PX4_SPIDEV_ICM_20602) && defined(PX4_SPI_BUS_1)
	{ MPU6000_BUS_SPI_INTERNAL1, MPU_DEVICE_TYPE_ICM20602, ICM20602_DEVICE_PATH, &MPU6000_SPI_interface, PX4_SPI_BUS_1,  false, NULL },
#endif
#ifdef PX4_SPIDEV_ICM_20608
	{ MPU6000_BUS_SPI_INTERNAL1, MPU_DEVICE_TYPE_ICM20608, ICM20608_DEVICE_PATH, &MPU6000_SPI_interface, PX4_SPI_BUS_SENSORS,  false, NULL },
#endif
#ifdef PX4_SPIDEV_ICM_20689
	{ MPU6000_BUS_SPI_INTERNAL2, MPU_DEVICE_TYPE_ICM20689, ICM20689_DEVICE_PATH, &MPU6000_SPI_interface, PX4_SPI_BUS_SENSORS,  false, NULL },
#endif
#if defined(PX4_SPI_BUS_EXTERNAL)
	{ MPU6000_BUS_SPI_EXTERNAL1, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH_EXT, &MPU6000_SPI_interface, PX4_SPI_BUS_EXTERNAL, true,  NULL },
	{ MPU6000_BUS_SPI_EXTERNAL2, MPU_DEVICE_TYPE_MPU6000, MPU_DEVICE_PATH_EXT1, &MPU6000_SPI_interface, PX4_SPI_BUS_EXTERNAL, true,  NULL },
#endif
};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))


void	start(enum MPU6000_BUS busid, enum Rotation rotation, int device_type);
bool 	start_bus(struct mpu6000_bus_option &bus, enum Rotation rotation, int device_type);
void	stop(enum MPU6000_BUS busid);
static struct mpu6000_bus_option &find_bus(enum MPU6000_BUS busid);
void	reset(enum MPU6000_BUS busid);
void	info(enum MPU6000_BUS busid);
void	regdump(enum MPU6000_BUS busid);
void	testerror(enum MPU6000_BUS busid);
#ifndef CONSTRAINED_FLASH
void	factorytest(enum MPU6000_BUS busid);
#endif
void	usage();

/**
 * find a bus structure for a busid
 */
struct mpu6000_bus_option &find_bus(enum MPU6000_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MPU6000_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct mpu6000_bus_option &bus, enum Rotation rotation, int device_type)
{
	if (bus.dev != nullptr) {
		warnx("%s SPI not available", bus.external ? "External" : "Internal");
		return false;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum, device_type, bus.external);

	if (interface == nullptr) {
		warnx("failed creating interface for bus #%u (SPI%u)", (unsigned)bus.busid, (unsigned)bus.busnum);
		return false;
	}

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus #%u (SPI%u)", (unsigned)bus.busid, (unsigned)bus.busnum);
		return false;
	}

	bus.dev = new MPU6000(interface, bus.devpath, rotation, device_type);

	if (bus.dev == nullptr) {
		delete interface;
		return false;
	}

	if (OK != bus.dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */

	bus.dev->start();

	return true;

fail:

	if (bus.dev != nullptr) {
		delete bus.dev;
		bus.dev = nullptr;
	}

	return false;
}

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(enum MPU6000_BUS busid, enum Rotation rotation, int device_type)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MPU6000_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MPU6000_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (bus_options[i].device_type != device_type) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation, device_type);
	}

	exit(started ? 0 : 1);
}

void
stop(enum MPU6000_BUS busid)
{
	struct mpu6000_bus_option &bus = find_bus(busid);

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
 * Reset the driver.
 */
void
reset(enum MPU6000_BUS busid)
{
	struct mpu6000_bus_option &bus = find_bus(busid);

	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	bus.dev->reset();

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(enum MPU6000_BUS busid)
{
	struct mpu6000_bus_option &bus = find_bus(busid);

	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	bus.dev->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(enum MPU6000_BUS busid)
{
	struct mpu6000_bus_option &bus = find_bus(busid);

	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", bus.dev);
	bus.dev->print_registers();

	exit(0);
}

/**
 * deliberately produce an error to test recovery
 */
void
testerror(enum MPU6000_BUS busid)
{
	struct mpu6000_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	bus.dev->test_error();

	exit(0);
}

#ifndef CONSTRAINED_FLASH
/**
 * Dump the register information
 */
void
factorytest(enum MPU6000_BUS busid)
{
	struct mpu6000_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	bus.dev->factory_self_test();

	exit(0);
}
#endif

void
usage()
{
	warnx("missing command: try 'start', 'info', 'stop',\n'reset', 'regdump', 'testerror'"
#ifndef CONSTRAINED_FLASH
	      ", 'factorytest'"
#endif
	     );
	warnx("options:");
	warnx("    -X external I2C bus");
	warnx("    -I internal I2C bus");
	warnx("    -S external SPI bus");
	warnx("    -s internal SPI bus");
	warnx("    -Z external1 SPI bus");
	warnx("    -z internal2 SPI bus");
	warnx("    -T 6000|20608|20602 (default 6000)");
	warnx("    -R rotation");
}

} // namespace

/** driver 'main' command */
extern "C" { __EXPORT int mpu6000_main(int argc, char *argv[]); }

int
mpu6000_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	enum MPU6000_BUS busid = MPU6000_BUS_ALL;
	int device_type = MPU_DEVICE_TYPE_MPU6000;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "T:XISsZzR:a:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MPU6000_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MPU6000_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = MPU6000_BUS_SPI_EXTERNAL1;
			break;

		case 's':
			busid = MPU6000_BUS_SPI_INTERNAL1;
			break;

		case 'Z':
			busid = MPU6000_BUS_SPI_EXTERNAL2;
			break;

		case 'z':
			busid = MPU6000_BUS_SPI_INTERNAL2;
			break;

		case 'T':
			device_type = atoi(myoptarg);
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			mpu6000::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		mpu6000::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		mpu6000::start(busid, rotation, device_type);
	}

	if (!strcmp(verb, "stop")) {
		mpu6000::stop(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mpu6000::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		mpu6000::info(busid);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		mpu6000::regdump(busid);
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(verb, "factorytest")) {
		mpu6000::factorytest(busid);
	}

#endif

	if (!strcmp(verb, "testerror")) {
		mpu6000::testerror(busid);
	}

	mpu6000::usage();
	return -1;
}

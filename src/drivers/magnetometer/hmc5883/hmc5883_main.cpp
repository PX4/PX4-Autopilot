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

/**
 * @file hmc5883.cpp
 *
 * Driver for the HMC5883 / HMC5983 magnetometer connected via I2C or SPI.
 */

#include <px4_platform_common/getopt.h>

#include "HMC5883.hpp"
#include "hmc5883.h"

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hmc5883_main(int argc, char *argv[]);

enum HMC5883_BUS {
	HMC5883_BUS_ALL = 0,
	HMC5883_BUS_I2C_INTERNAL,
	HMC5883_BUS_I2C_EXTERNAL,
	HMC5883_BUS_SPI
};

/**
 * Local functions in support of the shell command.
 */
namespace hmc5883
{

/*
  list of supported bus configurations
 */
struct hmc5883_bus_option {
	enum HMC5883_BUS busid;
	const char *devpath;
	HMC5883_constructor interface_constructor;
	uint8_t busnum;
	HMC5883	*dev;
} bus_options[] = {
	{ HMC5883_BUS_I2C_EXTERNAL, "/dev/hmc5883_ext", &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_EXPANSION1
	{ HMC5883_BUS_I2C_EXTERNAL, "/dev/hmc5883_ext1", &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ HMC5883_BUS_I2C_EXTERNAL, "/dev/hmc5883_ext2", &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ HMC5883_BUS_I2C_INTERNAL, "/dev/hmc5883_int", &HMC5883_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_HMC
	{ HMC5883_BUS_SPI, "/dev/hmc5883_spi", &HMC5883_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

/**
 * start driver for a specific bus option
 */
static bool
start_bus(struct hmc5883_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u (type: %u)", (unsigned)bus.busnum, (unsigned)bus.busid);
		return false;
	}

	bus.dev = new HMC5883(interface, bus.devpath, rotation);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = px4_open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		px4_close(fd);
		PX4_ERR("Failed to setup poll rate");
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
static int
start(enum HMC5883_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == HMC5883_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != HMC5883_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
	}

	if (!started) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int
stop()
{
	bool stopped = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			bus_options[i].dev->stop();
			delete bus_options[i].dev;
			bus_options[i].dev = nullptr;
			stopped = true;
		}
	}

	return !stopped;
}

/**
 * find a bus structure for a busid
 */
static struct hmc5883_bus_option *
find_bus(enum HMC5883_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == HMC5883_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return &bus_options[i];
		}
	}

	PX4_ERR("bus %u not started", (unsigned)busid);
	return nullptr;
}

/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766	| for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713	| for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the HMC5883L for proper operation, a self test feature in incorporated
 * in which the sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias)
 * or 10 (negetive bias), e.g. 0x11 or 0x12.
 * Then, by placing the mode register into single-measurement mode (0x01),
 * two data acquisition cycles will be made on each magnetic vector.
 * The first acquisition will be a set pulse followed shortly by measurement
 * data of the external field. The second acquisition will have the offset strap
 * excited (about 10 mA) in the positive bias mode for X, Y, and Z axes to create
 * about a ±1.1 gauss self test field plus the external field. The first acquisition
 * values will be subtracted from the second acquisition, and the net measurement
 * will be placed into the data output registers.
 * Since self test adds ~1.1 Gauss additional field to the existing field strength,
 * using a reduced gain setting prevents sensor from being saturated and data registers
 * overflowed. For example, if the configuration register B is set to 0x60 (Gain=3),
 * values around +766 LSB (1.16 Ga * 660 LSB/Ga) will be placed in the X and Y data
 * output registers and around +713 (1.08 Ga * 660 LSB/Ga) will be placed in Z data
 * output register. To leave the self test mode, change MS1 and MS0 bit of the
 * configuration register A back to 00 (Normal Measurement Mode), e.g. 0x10.
 * Using the self test method described above, the user can scale sensor
 */
static int calibrate(enum HMC5883_BUS busid)
{
	struct hmc5883_bus_option *bus = find_bus(busid);

	if (bus == nullptr) {
		return 0;
	}

	const char *path = bus->devpath;

	int fd = px4_open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'hmc5883 start' if the driver is not running", path);
		return PX4_ERROR;
	}

	int ret = 0;

	if (OK != (ret = px4_ioctl(fd, MAGIOCCALIBRATE, fd))) {
		PX4_ERR("failed to enable sensor calibration mode");
	}

	px4_close(fd);

	return ret;
}

/**
 * enable/disable temperature compensation
 */
static int
temp_enable(enum HMC5883_BUS busid, bool enable)
{
	struct hmc5883_bus_option *bus = find_bus(busid);

	if (bus == nullptr) {
		return 0;
	}

	const char *path = bus->devpath;

	int fd = px4_open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed ");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, MAGIOCSTEMPCOMP, (unsigned)enable) < 0) {
		PX4_ERR("set temperature compensation failed");
		return PX4_ERROR;
	}

	px4_close(fd);
	return 0;
}

/**
 * Print a little info about the driver.
 */
static int
info(enum HMC5883_BUS busid)
{
	struct hmc5883_bus_option *bus = find_bus(busid);

	if (bus == nullptr) {
		return 0;
	}

	if (bus != nullptr) {
		PX4_INFO("running on bus: %u (%s)\n", (unsigned)bus->busid, bus->devpath);
		bus->dev->print_info();
	}

	return PX4_OK;
}

static int
usage()
{
	warnx("missing command: try 'start', 'info', 'calibrate'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -C calibrate on start");
	warnx("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)
	warnx("    -I only internal bus");
#endif
	return PX4_OK;
}

} // namespace

int
hmc5883_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	enum HMC5883_BUS busid = HMC5883_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;
	bool temp_compensation = false;

	if (argc < 2) {
		return hmc5883::usage();
	}

	while ((ch = px4_getopt(argc, argv, "XISR:CT", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)

		case 'I':
			busid = HMC5883_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = HMC5883_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = HMC5883_BUS_SPI;
			break;

		case 'C':
			calibrate = true;
			break;

		case 'T':
			temp_compensation = true;
			break;

		default:
			return hmc5883::usage();
		}
	}

	if (myoptind >= argc) {
		return hmc5883::usage();
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		hmc5883::start(busid, rotation);

		if (calibrate && hmc5883::calibrate(busid) != 0) {
			PX4_ERR("calibration failed");
			return -1;
		}

		if (temp_compensation) {
			// we consider failing to setup temperature
			// compensation as non-fatal
			hmc5883::temp_enable(busid, true);
		}

		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		return hmc5883::stop();
	}

	/*
	 * enable/disable temperature compensation
	 */
	if (!strcmp(verb, "tempoff")) {
		return hmc5883::temp_enable(busid, false);
	}

	if (!strcmp(verb, "tempon")) {
		return hmc5883::temp_enable(busid, true);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		return hmc5883::info(busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (hmc5883::calibrate(busid) == 0) {
			PX4_INFO("calibration successful");
			return 0;

		} else {
			PX4_ERR("calibration failed");
			return -1;
		}
	}

	return hmc5883::usage();
}

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
 * Driver for the Invensense mpu9250 connected via I2C or SPI.
 *
 * @authors Andrew Tridgell
 *          Robert Dickenson
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <px4_getopt.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mpu9250.h"

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu9250_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu9250_gyro"
#define MPU_DEVICE_PATH_MAG		"/dev/mpu9250_mag"

#define MPU_DEVICE_PATH_ACCEL_1		"/dev/mpu9250_accel1"
#define MPU_DEVICE_PATH_GYRO_1		"/dev/mpu9250_gyro1"
#define MPU_DEVICE_PATH_MAG_1		"/dev/mpu9250_mag1"

#define MPU_DEVICE_PATH_ACCEL_EXT	"/dev/mpu9250_accel_ext"
#define MPU_DEVICE_PATH_GYRO_EXT	"/dev/mpu9250_gyro_ext"
#define MPU_DEVICE_PATH_MAG_EXT 	"/dev/mpu9250_mag_ext"

#define MPU_DEVICE_PATH_ACCEL_EXT1	"/dev/mpu9250_accel_ext1"
#define MPU_DEVICE_PATH_GYRO_EXT1	"/dev/mpu9250_gyro_ext1"
#define MPU_DEVICE_PATH_MAG_EXT1 	"/dev/mpu9250_mag_ext1"

#define MPU_DEVICE_PATH_ACCEL_EXT2	"/dev/mpu9250_accel_ext2"
#define MPU_DEVICE_PATH_GYRO_EXT2	"/dev/mpu9250_gyro_ext2"
#define MPU_DEVICE_PATH_MAG_EXT2	"/dev/mpu9250_mag_ext2"

/** driver 'main' command */
extern "C" { __EXPORT int mpu9250_main(int argc, char *argv[]); }

enum MPU9250_BUS {
	MPU9250_BUS_ALL = 0,
	MPU9250_BUS_I2C_INTERNAL,
	MPU9250_BUS_I2C_EXTERNAL,
	MPU9250_BUS_SPI_INTERNAL,
	MPU9250_BUS_SPI_INTERNAL2,
	MPU9250_BUS_SPI_EXTERNAL
};

/**
 * Local functions in support of the shell command.
 */

static int task_handle_started = -1;

namespace mpu9250
{

static enum MPU9250_BUS bus_id = MPU9250_BUS_ALL;
static enum Rotation mpu_rotation = ROTATION_NONE;
static bool is_external = false;

/*
  list of supported bus configurations
 */

struct mpu9250_bus_option {
	enum MPU9250_BUS busid;
	const char *accelpath;
	const char *gyropath;
	const char *magpath;
	MPU9250_constructor interface_constructor;
	bool magpassthrough;
	uint8_t busnum;
	uint32_t address;
	MPU9250	*dev;
	int task_id;
} bus_options[] = {
#if defined (USE_I2C)
<<<<<<< HEAD
#  if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG,  &MPU9250_I2C_interface, false, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION)
#  if defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#endif
#  if defined(PX4_I2C_BUS_EXPANSION1) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT1, MPU_DEVICE_PATH_GYRO_EXT1, MPU_DEVICE_PATH_MAG_EXT1, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION1, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION2) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT2, MPU_DEVICE_PATH_GYRO_EXT2, MPU_DEVICE_PATH_MAG_EXT2, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION2, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#endif
#ifdef PX4_SPIDEV_MPU
	{ MPU9250_BUS_SPI_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU, nullptr },
#endif
#ifdef PX4_SPIDEV_MPU2
	{ MPU9250_BUS_SPI_INTERNAL2, MPU_DEVICE_PATH_ACCEL_1, MPU_DEVICE_PATH_GYRO_1, MPU_DEVICE_PATH_MAG_1, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU2, nullptr },
#endif
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_MPU)
	{ MPU9250_BUS_SPI_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_SPI_interface, true, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU, nullptr },
=======
#  if defined(PX4_I2C_BUS_ONBOARD)
	{ MPU9250_BUS_I2C_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG,  &MPU9250_I2C_interface, false, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_MPU9250, NULL, -1 },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_MPU9250, NULL, -1 },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION1)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT1, MPU_DEVICE_PATH_GYRO_EXT1, MPU_DEVICE_PATH_MAG_EXT1, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION1, PX4_I2C_OBDEV_MPU9250, NULL, -1 },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION2)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT2, MPU_DEVICE_PATH_GYRO_EXT2, MPU_DEVICE_PATH_MAG_EXT2, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION2, PX4_I2C_OBDEV_MPU9250, NULL, -1 },
#  endif
#endif
#ifdef PX4_SPIDEV_MPU
	{ MPU9250_BUS_SPI_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU, NULL, -1 },
#endif
#ifdef PX4_SPIDEV_MPU2
	{ MPU9250_BUS_SPI_INTERNAL2, MPU_DEVICE_PATH_ACCEL_1, MPU_DEVICE_PATH_GYRO_1, MPU_DEVICE_PATH_MAG_1, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU2, NULL, -1 },
#endif
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_MPU)
	{ MPU9250_BUS_SPI_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_SPI_interface, true, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU, NULL, -1 },
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
#endif
};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))


<<<<<<< HEAD
void	start(enum MPU9250_BUS busid, enum Rotation rotation, bool external_bus, bool magnetometer_only);
bool	start_bus(struct mpu9250_bus_option &bus, enum Rotation rotation, bool external_bus, bool magnetometer_only);
=======
int	start_task(int argc, char *argv[]);
bool	start_bus(struct mpu9250_bus_option &bus, enum Rotation rotation, bool external_bus);
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
struct mpu9250_bus_option &find_bus(enum MPU9250_BUS busid);
void	stop(enum MPU9250_BUS busid);
void	reset(enum MPU9250_BUS busid);
void	info(enum MPU9250_BUS busid);
void	usage();

/**
 * find a bus structure for a busid
 */
struct mpu9250_bus_option &find_bus(enum MPU9250_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MPU9250_BUS_ALL ||
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
start_bus(struct mpu9250_bus_option &bus, enum Rotation rotation, bool external, bool magnetometer_only)
{
	int fd = -1;

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
	bool is_i2c = bus.busid == MPU9250_BUS_I2C_INTERNAL || bus.busid == MPU9250_BUS_I2C_EXTERNAL;

	if (is_i2c) {
		mag_interface = AK8963_I2C_interface(bus.busnum, external);
	}

#endif

<<<<<<< HEAD
	bus.dev = new MPU9250(interface, mag_interface, bus.accelpath, bus.gyropath, bus.magpath, rotation,
			      magnetometer_only);
=======
	MPU9250 mpu(interface, mag_interface, bus.accelpath, bus.gyropath, bus.magpath, rotation);

	bus.dev = &mpu;
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9

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

	/*
	 * Set the poll rate to default, starts automatic data collection.
	 * Doing this through the mag device for the time being - it's always there, even in magnetometer only mode.
	 * Using accel device for MPU6500.
	 */
	if (bus.dev->get_whoami() == MPU_WHOAMI_6500) {
		fd = open(bus.accelpath, O_RDONLY);

	} else {
		fd = open(bus.magpath, O_RDONLY);
	}

	if (fd < 0) {
		PX4_INFO("ioctl failed");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}


	close(fd);

	mpu.start();

	return true;

fail:

	if (fd >= 0) {
		close(fd);
	}

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
<<<<<<< HEAD
void
start(enum MPU9250_BUS busid, enum Rotation rotation, bool external, bool magnetometer_only)
=======
int
start_task(int argc, char *argv[])
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
<<<<<<< HEAD
		if (bus_options[i].dev != nullptr) {
=======
		if (bus_id == MPU9250_BUS_ALL && bus_options[i].dev != NULL) {
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
			// this device is already started
			continue;
		}

		if (bus_id != MPU9250_BUS_ALL && bus_options[i].busid != bus_id) {
			// not the one that is asked for
			continue;
		}

<<<<<<< HEAD
		started |= start_bus(bus_options[i], rotation, external, magnetometer_only);

		if (started) { break; }
=======
		started |= start_bus(bus_options[i], mpu_rotation, is_external);

		if (started) {
			bus_options[i].task_id = task_handle_started;
		}
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
	}

	return (started ? 0 : 1);

}

void
stop(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


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
reset(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);
	int fd = open(bus.accelpath, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


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
	PX4_INFO("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (i2c external bus)");
	PX4_INFO("    -I    (i2c internal bus)");
	PX4_INFO("    -s    (spi internal bus)");
	PX4_INFO("    -S    (spi external bus)");
	PX4_INFO("    -t    (spi internal bus, 2nd instance)");
	PX4_INFO("    -R rotation");
	PX4_INFO("    -M only enable magnetometer, accel/gyro disabled - not av. on MPU6500");
}

} // namespace

int
mpu9250_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

<<<<<<< HEAD
	enum MPU9250_BUS busid = MPU9250_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool magnetometer_only = false;
=======
	mpu9250::bus_id = MPU9250_BUS_ALL;
	mpu9250::mpu_rotation = ROTATION_NONE;
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9

	while ((ch = px4_getopt(argc, argv, "XISstMR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			mpu9250::bus_id = MPU9250_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			mpu9250::bus_id = MPU9250_BUS_I2C_INTERNAL;
			break;

		case 'S':
			mpu9250::bus_id = MPU9250_BUS_SPI_EXTERNAL;
			break;

		case 's':
			mpu9250::bus_id = MPU9250_BUS_SPI_INTERNAL;
			break;

		case 't':
			mpu9250::bus_id = MPU9250_BUS_SPI_INTERNAL2;
			break;

		case 'R':
			mpu9250::mpu_rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'M':
			magnetometer_only = true;
			break;

		default:
			mpu9250::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		mpu9250::usage();
		return -1;
	}

	mpu9250::is_external = (mpu9250::bus_id == MPU9250_BUS_I2C_EXTERNAL) || (mpu9250::bus_id == MPU9250_BUS_SPI_EXTERNAL);
	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
<<<<<<< HEAD
		mpu9250::start(busid, rotation, external, magnetometer_only);
=======

		/* start the task */
		task_handle_started = px4_task_spawn_cmd("mpu9250",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX,
				      6000,
				      (px4_main_t)&mpu9250::start_task,
				      nullptr);

		if (task_handle_started < 0) {
			PX4_WARN("task start failed");
			return -errno;
		}
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
	}

	if (!strcmp(verb, "stop")) {
		mpu9250::stop(mpu9250::bus_id);
	}

	/*
<<<<<<< HEAD
=======
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		mpu9250::test(mpu9250::bus_id);
	}

	/*
>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mpu9250::reset(mpu9250::bus_id);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		mpu9250::info(mpu9250::bus_id);
	}

<<<<<<< HEAD
=======
	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		mpu9250::regdump(mpu9250::bus_id);
	}

	if (!strcmp(verb, "testerror")) {
		mpu9250::testerror(mpu9250::bus_id);
	}

>>>>>>> 4ef9763e64145fb8b0a1c91ab887a6e9e6fefcc9
	mpu9250::usage();
	return 0;
}

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
#include <px4_platform_common/i2c_spi_buses.h>

/**
 * Local functions in support of the shell command.
 */
namespace mpu6000
{

static const int max_num_instances = 3;
static I2CSPIInstance *instances[MPU_DEVICE_TYPE_COUNT][max_num_instances] {};

void	start(BusInstanceIterator &iterator, int instance_type_idx, enum Rotation rotation, int device_type);
void	stop(BusInstanceIterator &iterator);
void	reset(BusInstanceIterator &iterator);
void	call_instances(BusInstanceIterator &iterator, void (MPU6000::*method)());
void	usage();

void
start(BusInstanceIterator &iterator, int instance_type_idx, enum Rotation rotation, int device_type)
{
	if (iterator.configuredBusOption() == I2CSPIBusOption::All) {
		PX4_ERR("need to specify a bus type");
		usage();
		exit(1);
	}

	bool started = false;

	while (iterator.next()) {
		if (iterator.instance()) {
			continue; // already running
		}

		const int free_index = iterator.nextFreeInstance();

		if (free_index < 0) {
			PX4_ERR("Not enough instances");
			return;
		}

		device::Device *interface = nullptr;

		if (iterator.busType() == BOARD_I2C_BUS) {
			interface = MPU6000_I2C_interface(iterator.bus(), iterator.devid(), device_type, iterator.external());

		} else if (iterator.busType() == BOARD_SPI_BUS) {
			interface = MPU6000_SPI_interface(iterator.bus(), iterator.devid(), device_type, iterator.external());
		}

		if (interface == nullptr) {
			PX4_ERR("failed creating interface for bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
			continue;
		}

		if (interface->init() != OK) {
			delete interface;
			PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
			continue;
		}

		MPU6000 *dev = new MPU6000(interface, rotation, device_type, iterator.configuredBusOption(), iterator.bus());

		if (dev == nullptr) {
			delete interface;
			continue;
		}

		if (OK != dev->init()) {
			delete dev;
			continue;
		}

		instances[instance_type_idx][free_index] = dev;
		dev->start();
		started = true;
	}

	exit(started ? 0 : 1);
}

void
stop(BusInstanceIterator &iterator)
{
	while (iterator.next()) {
		if (iterator.instance()) {
			delete iterator.instance();
			iterator.resetInstance();
		}
	}

	exit(0);
}

void
call_instances(BusInstanceIterator &iterator, void (MPU6000::*method)())
{
	while (iterator.next()) {
		if (iterator.instance()) {
			MPU6000 *instance = (MPU6000 *)iterator.instance();
			(instance->*method)();
		}
	}

	exit(0);
}

/**
 * Reset the driver.
 */
void
reset(BusInstanceIterator &iterator)
{
	while (iterator.next()) {
		if (iterator.instance()) {
			MPU6000 *instance = (MPU6000 *)iterator.instance();
			instance->reset();
		}
	}

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(BusInstanceIterator &iterator)
{
	while (iterator.next()) {
		if (iterator.instance()) {
			MPU6000 *instance = (MPU6000 *)iterator.instance();
			instance->print_registers();
		}
	}

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'stop',\n'reset', 'regdump', 'testerror'"
#ifndef CONSTRAINED_FLASH
		 ", 'factorytest'"
#endif
		);
	PX4_INFO("options:");
	PX4_INFO("    -X external I2C bus");
	PX4_INFO("    -I internal I2C bus");
	PX4_INFO("    -S external SPI bus");
	PX4_INFO("    -s internal SPI bus");
	PX4_INFO("    -c chip-select index (for ext SPI, default=1)");
	PX4_INFO("    -b specific bus (default=all)");
	PX4_INFO("    -T 6000|20608|20602 (default 6000)");
	PX4_INFO("    -R rotation");
}

} // namespace

/** driver 'main' command */
extern "C" { __EXPORT int mpu6000_main(int argc, char *argv[]); }

#include <px4_platform_common/spi.h>

int
mpu6000_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	BusCLIArguments cli;
	int device_type = MPU_DEVICE_TYPE_MPU6000;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "T:XISsR:b:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			cli.bus_option = I2CSPIBusOption::I2CExternal;
			break;

		case 'I':
			cli.bus_option = I2CSPIBusOption::I2CInternal;
			break;

		case 'S':
			cli.bus_option = I2CSPIBusOption::SPIExternal;
			break;

		case 's':
			cli.bus_option = I2CSPIBusOption::SPIInternal;
			break;

		case 'T':
			device_type = atoi(myoptarg);
			break;

		case 'c':
			cli.chipselect_index = atoi(myoptarg);
			break;

		case 'b':
			cli.requested_bus = atoi(myoptarg);
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

	int instance_type_idx = 0;
	uint16_t dev_type_driver = 0;

	switch (device_type) {
	case MPU_DEVICE_TYPE_MPU6000:
		instance_type_idx = 0;
		dev_type_driver = DRV_GYR_DEVTYPE_MPU6000;
		break;

	case MPU_DEVICE_TYPE_ICM20602:
		instance_type_idx = 1;
		dev_type_driver = DRV_GYR_DEVTYPE_ICM20602;
		break;

	case MPU_DEVICE_TYPE_ICM20608:
		instance_type_idx = 2;
		dev_type_driver = DRV_GYR_DEVTYPE_ICM20608;
		break;

	case MPU_DEVICE_TYPE_ICM20689:
		instance_type_idx = 3;
		dev_type_driver = DRV_GYR_DEVTYPE_ICM20689;
		break;
	}

	BusInstanceIterator iterator(mpu6000::instances[instance_type_idx], mpu6000::max_num_instances, cli, dev_type_driver);

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		mpu6000::start(iterator, instance_type_idx, rotation, device_type);
	}

	if (!strcmp(verb, "stop")) {
		mpu6000::stop(iterator);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mpu6000::reset(iterator);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		mpu6000::call_instances(iterator, &MPU6000::print_info);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		mpu6000::call_instances(iterator, &MPU6000::print_registers);
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(verb, "factorytest")) {
		mpu6000::call_instances(iterator, &MPU6000::factory_self_test);
	}

#endif

	if (!strcmp(verb, "testerror")) {
		mpu6000::call_instances(iterator, &MPU6000::test_error);
	}

	mpu6000::usage();
	return -1;
}

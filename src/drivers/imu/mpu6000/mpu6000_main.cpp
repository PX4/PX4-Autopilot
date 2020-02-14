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

void
MPU6000::print_usage()
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

void
MPU6000::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom2) {
	case 0: reset();
		break;

	case 1: print_registers();
		break;
#ifndef CONSTRAINED_FLASH

	case 2: factory_self_test();
		break;
#endif

	case 3: test_error();
		break;
	}
}

I2CSPIDriverBase *MPU6000::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	device::Device *interface = nullptr;
	int device_type = cli.custom1;

	if (iterator.busType() == BOARD_I2C_BUS) {
		interface = MPU6000_I2C_interface(iterator.bus(), iterator.devid(), device_type, iterator.external());

	} else if (iterator.busType() == BOARD_SPI_BUS) {
		interface = MPU6000_SPI_interface(iterator.bus(), iterator.devid(), device_type, iterator.external());
	}

	if (interface == nullptr) {
		PX4_ERR("failed creating interface for bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	MPU6000 *dev = new MPU6000(interface, cli.rotation, device_type, iterator.configuredBusOption(), iterator.bus());

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	dev->start();
	return dev;
}

/** driver 'main' command */
extern "C" { __EXPORT int mpu6000_main(int argc, char *argv[]); }

int
mpu6000_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	BusCLIArguments cli;
	cli.custom1 = MPU_DEVICE_TYPE_MPU6000;
	using ThisDriver = MPU6000;

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
			cli.custom1 = atoi(myoptarg);
			break;

		case 'c':
			cli.chipselect_index = atoi(myoptarg);
			break;

		case 'b':
			cli.requested_bus = atoi(myoptarg);
			break;

		case 'R':
			cli.rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			ThisDriver::print_usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		ThisDriver::print_usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	uint16_t dev_type_driver = 0;

	switch (cli.custom1) {
	case MPU_DEVICE_TYPE_MPU6000:
		dev_type_driver = DRV_GYR_DEVTYPE_MPU6000;
		break;

	case MPU_DEVICE_TYPE_ICM20602:
		dev_type_driver = DRV_GYR_DEVTYPE_ICM20602;
		break;

	case MPU_DEVICE_TYPE_ICM20608:
		dev_type_driver = DRV_GYR_DEVTYPE_ICM20608;
		break;

	case MPU_DEVICE_TYPE_ICM20689:
		dev_type_driver = DRV_GYR_DEVTYPE_ICM20689;
		break;
	}

	BusInstanceIterator iterator(ThisDriver::instances(), ThisDriver::max_num_instances, cli, dev_type_driver);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom2 = 0;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "regdump")) {
		cli.custom2 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(verb, "factorytest")) {
		cli.custom2 = 2;
		return ThisDriver::module_custom_method(cli, iterator);
	}

#endif

	if (!strcmp(verb, "testerror")) {
		cli.custom2 = 3;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

#include "mmc5616.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>


I2CSPIDriverBase *MMC5616::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	device::Device *interface = MMC5616_I2C_interface(config);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i", config.bus);
		return nullptr;
	}

	MMC5616 *dev = new MMC5616(interface, config);

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	return dev;
}

void MMC5616::print_usage()
{
	PRINT_MODULE_USAGE_NAME("mmc5616", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(I2C_ADDRESS_DEFAULT);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int mmc5616_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = MMC5616;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = I2C_SPEED;
	cli.i2c_address = I2C_ADDRESS_DEFAULT;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_MMC5616);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

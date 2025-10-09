/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team.
 *   Modified for Blue Robotics BAR100 Sensor.
 *
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

#include "bar100.hpp"
#include "bar100_registers.h"

#define MODULE_NAME "bar100"

void Bar100::print_usage()
{
    PRINT_MODULE_USAGE_NAME("bar100", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("baro");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAM_INT('b', 3, 1, 10, "I2C bus number", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('X', "External I2C bus", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int bar100_main(int argc, char *argv[])
{
    using ThisDriver = Bar100;
    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = 400000;
    cli.i2c_address = BAR100_I2C_ADDR;

    uint16_t dev_type_driver = DRV_BARO_DEVTYPE_BAR100;

    const char *verb = cli.parseDefaultArguments(argc, argv);

    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, dev_type_driver);

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

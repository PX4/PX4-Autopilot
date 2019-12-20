
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/power_monitor.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "ina226.h"

#define MAX_I2C_BATTERY_COUNT 2

extern "C" __EXPORT int ina226_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace ina226
{

INA226	*g_dev[MAX_I2C_BATTERY_COUNT] = {nullptr};

int 	start(int i2c_bus, int address);
int 	start_bus(int i2c_bus, int address, int battery_index, bool force);
int 	stop(int i2c_bus, int address);
int 	info(int i2c_bus, int address);

/**
 * If a device is already running on the specified bus and address, return the index of that device.
 * If not, return the index of the first empty slot (nullptr) in the array.
 * @param i2c_bus The bus, as an index in i2c_bus_options
 * @param address I2C address of the power monitor
 * @return If there is already a device running on the given bus with the given address: Return the index of that device
 *         If there is not already a device running: Return the index of the first nullptr in the array.
 *         If there are no empty slots in the array, return -1.
 */
int get_index(int i2c_bus, int address)
{
	int first_nullptr = -1;

	for (size_t i = 0; i < MAX_I2C_BATTERY_COUNT; i++) {
		//PX4_INFO("Checking number %lu", i);
		if (g_dev[i] == nullptr) {
			if (first_nullptr < 0) {
				first_nullptr = i;
			}

		} else if (g_dev[i]->get_device_bus() == i2c_bus_options[i2c_bus] && g_dev[i]->get_device_address() == address) {
			return i;
		}
	}

	return first_nullptr;
}

/**
 *
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	for (unsigned i2c_bus = 0; i2c_bus < NUM_I2C_BUS_OPTIONS; i2c_bus++) {
		int index = get_index(i2c_bus, INA226_BASEADDR);

		if (index < 0) {
			PX4_ERR("There are already %d instances of INA226 running. No more can be instantiated.",
				MAX_I2C_BATTERY_COUNT);
			return PX4_ERROR;

		} else if (g_dev[index] == nullptr && start_bus(i2c_bus, INA226_BASEADDR, 1, false) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(int i2c_bus, int address, int battery_index, bool force)
{
	int idx = get_index(i2c_bus, address);

	if (idx < 0) {
		PX4_ERR("There are already %d instances of INA226 running. No more can be instantiated.",
			MAX_I2C_BATTERY_COUNT);
		return PX4_ERROR;
	}

	if (g_dev[idx] != nullptr) {
		PX4_ERR("Already started on bus %d, address 0x%02X", i2c_bus, address);
		return PX4_ERROR;
	}

	/* create the driver */
	// TODO: Possibly change this to use statically-allocated memory and placement-new?
	g_dev[idx] = new INA226(battery_index, i2c_bus, address);

	if (g_dev[idx] == nullptr) {
		PX4_ERR("Unable to allocate memory for INA226");
		goto fail;
	}

	if (force) {
		if (g_dev[idx]->force_init() != OK) {
			PX4_INFO("Failed to initialize INA226 on bus %d, but will try again periodically.", i2c_bus);
		}

	} else if (OK != g_dev[idx]->init()) {
		PX4_ERR("Failed to initialize on bus %d, address 0x%02X", i2c_bus, address);
		goto fail;

	} else {
		PX4_INFO("Started INA226 on bus %d", i2c_bus);
	}

	return PX4_OK;

fail:

	if (g_dev[idx] != nullptr) {
		delete g_dev[idx];
		g_dev[idx] = nullptr;

	}

	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop(int bus, int address)
{
	int idx = get_index(bus, address);

	if (idx < 0 || g_dev[idx] == nullptr) {
		PX4_ERR("Driver not running on bus %d, address 0x%02X", bus, address);
		return PX4_ERROR;

	} else {
		delete g_dev[idx];
		g_dev[idx] = nullptr;
		return PX4_OK;
	}
}

/**
 * Print a little info about the driver.
 */
int
info(int bus, int address)
{
	bool any_running = false;

	for (int i = 0; i < MAX_I2C_BATTERY_COUNT; i++) {
		if (g_dev[i] != nullptr) {
			any_running = true;
			PX4_INFO("Bus %d, address 0x%02X:", (int) g_dev[i]->index, g_dev[i]->get_device_address());
			g_dev[i]->print_info();
		}
	}

	if (!any_running) {
		PX4_INFO("No devices are running.");
	}

	return PX4_OK;
}

} /* namespace */


static void
ina2262_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the INA226 power monitor.

Multiple instances of this driver can run simultaneously, if each instance has a separate bus OR I2C address.

For example, one instance can run on Bus 2, address 0x41, and one can run on Bus 2, address 0x43.

If the INA226 module is not powered, then by default, initialization of the driver will fail. To change this, use
the -f flag. If this flag is set, then if initialization fails, the driver will keep trying to initialize again
every 0.5 seconds. With this flag set, you can plug in a battery after the driver starts, and it will work. Without
this flag set, the battery must be plugged in before starting the driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ina226", "driver");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start a new instance of the driver");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "If set, try to start the driver on each availabe I2C bus until a module is found", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "If initialization fails, keep retrying periodically. Ignored if the -a flag is set. See full driver documentation for more info", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, NUM_I2C_BUS_OPTIONS - 1, "I2C bus (default: use board-specific bus)", true);
	// The module documentation system INSISTS on decimal literals here. So we can't use a #define constant, and
	// we can't use hexadecimal.
	PRINT_MODULE_USAGE_PARAM_INT('d', 65, 0, UINT8_MAX, "I2C Address (Start with '0x' for hexadecimal)", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 1, 1, 2, "Which battery calibration values should be used (1 or 2)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop one instance of the driver");
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, NUM_I2C_BUS_OPTIONS - 1, "I2C bus (default: use board-specific bus)", true);
	PRINT_MODULE_USAGE_PARAM_INT('d', 65, 0, UINT8_MAX, "I2C Address (Start with '0x' for hexadecimal)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Status of every instance of the driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Status of every instance of the driver");
}

int
ina226_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;
	bool force = false;

	int i2c_bus = INA226_BUS_DEFAULT;
	int address = INA226_BASEADDR;
	int battery = 1;

	while ((ch = px4_getopt(argc, argv, "afb:d:t:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

			case 'b':
				i2c_bus = atoi(myoptarg);
				break;

			case 'a':
				start_all = true;
				break;

			case 'f':
				force = true;
				break;

			case 'd':
				address = strtol(myoptarg, nullptr, 0);
				break;

			case 't':
				battery = atoi(myoptarg);
				break;

			default:
				PX4_WARN("Unknown option!");
				goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	if (address > 255 || address < 0) {
		PX4_ERR("Address must be between 0 and 255");
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return ina226::start();

		} else {
			return ina226::start_bus(i2c_bus, address, battery, force);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return ina226::stop(i2c_bus, address);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return ina226::info(i2c_bus, address);
	}

	out_error:
	ina2262_usage();
	return PX4_ERROR;
}

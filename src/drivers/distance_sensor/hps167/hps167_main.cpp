#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

#include "HPS167.hpp"

/**
 * Local functions in support of the shell command.
 */
namespace hps167
{

HPS167 *g_dev;

int reset(const char *port);
int start(const char *port, const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int usage();

/**
 * Reset the driver.
 */
int
reset(const char *port)
{
	if (stop() == PX4_OK) {
		return start(port);
	}

	return PX4_ERROR;
}

/**
 * Start the driver.
 */
int
start(const char *port, const uint8_t rotation)
{
	if (port == nullptr) {
		PX4_ERR("invalid port");
		return PX4_ERROR;
	}

	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new HPS167(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

int
usage()
{
	PX4_INFO("usage: hps167 command [options]");
	PX4_INFO("command:");
	PX4_INFO("\t|reset|start|status|stop|");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace hps167

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int hps167_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R': {
				int rot = -1;

				if (px4_get_parameter_value(myoptarg, rot) != 0) {
					PX4_ERR("rotation parsing failed");
					return -1;
				}

				rotation = (uint8_t)rot;
				break;
			}

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return hps167::usage();
		}
	}

	if (myoptind >= argc) {
		return hps167::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return hps167::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return hps167::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return hps167::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return hps167::stop();
	}

	return hps167::usage();
}

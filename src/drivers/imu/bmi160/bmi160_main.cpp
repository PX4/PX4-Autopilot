
#include "bmi160.hpp"

#include <board_config.h>
#include <px4_getopt.h>

/** driver 'main' command */
extern "C" { __EXPORT int bmi160_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace bmi160
{

BMI160	*g_dev_int; // on internal bus
BMI160	*g_dev_ext; // on external bus

void	start(bool, enum Rotation);
void	stop(bool);
void	test(bool);
void	reset(bool);
void	info(bool);
void	regdump(bool);
void	testerror(bool);
void	usage();


/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
	const char *path_accel = external_bus ? BMI160_DEVICE_PATH_ACCEL_EXT : BMI160_DEVICE_PATH_ACCEL;
	const char *path_gyro  = external_bus ? BMI160_DEVICE_PATH_GYRO_EXT : BMI160_DEVICE_PATH_GYRO;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_BMI)
		*g_dev_ptr = new BMI160(PX4_SPI_BUS_EXT, path_accel, path_gyro, PX4_SPIDEV_EXT_BMI, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
#if defined(PX4_SPIDEV_BMI)
		*g_dev_ptr = new BMI160(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, PX4_SPIDEV_BMI, rotation);
#else
		errx(0, "No Internal SPI CS");
#endif
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path_accel, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

void
stop(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
	const char *path_accel = external_bus ? BMI160_DEVICE_PATH_ACCEL_EXT : BMI160_DEVICE_PATH_ACCEL;
	const char *path_gyro  = external_bus ? BMI160_DEVICE_PATH_GYRO_EXT : BMI160_DEVICE_PATH_GYRO;
	sensor_accel_s a_report{};
	sensor_gyro_s g_report{};
	ssize_t sz;

	/* get the driver */
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmi160 start')",
		    path_accel);

	/* get the driver */
	int fd_gyro = open(path_gyro, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", path_gyro);
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	print_message(a_report);

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	/* reset to default polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd);
	close(fd_gyro);

	/* XXX add poll-rate tests here too */

	reset(external_bus);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
	const char *path_accel = external_bus ? BMI160_DEVICE_PATH_ACCEL_EXT : BMI160_DEVICE_PATH_ACCEL;
	int fd = open(path_accel, O_RDONLY);

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
info(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_registers();

	exit(0);
}


/**
 * deliberately produce an error to test recovery
 */
void
testerror(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	(*g_dev_ptr)->test_error();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
bmi160_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			bmi160::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmi160::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmi160::start(external_bus, rotation);
	}

	if (!strcmp(verb, "stop")) {
		bmi160::stop(external_bus);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		bmi160::test(external_bus);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		bmi160::reset(external_bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmi160::info(external_bus);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		bmi160::regdump(external_bus);
	}

	if (!strcmp(verb, "testerror")) {
		bmi160::testerror(external_bus);
	}

	bmi160::usage();
	return -1;
}

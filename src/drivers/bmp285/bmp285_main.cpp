
#include <drivers/bmp285/bmp285.hpp>


/** driver 'main' command */
extern "C" { __EXPORT int bmp285_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace bmp285
{

BMP285	*g_dev_int; // on internal bus
BMP285	*g_dev_ext; // on external bus


void    start(bool);
void    test(bool);
void    reset(bool);
void    info(bool);
void    calibrate(bool, unsigned);
void    usage();



/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus)
{
	int fd;
	BMP285 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
	const char *path = external_bus ? BMP285_DEVICE_PATH_PRESSURE_EXT : BMP285_DEVICE_PATH_PRESSURE;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}


	/* create the driver */
	if (external_bus) {
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_EXT_OBDEV_BMP285)
		*g_dev_ptr = new BMP285(PX4_I2C_BUS_EXPANSION, BMP285_SLAVE_ADDRESS, path, external_bus);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		*g_dev_ptr = new BMP285(PX4_I2C_BUS_BMP285, BMP285_SLAVE_ADDRESS, path, external_bus);
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);


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
		delete(*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
	const char *path = external_bus ? BMP285_DEVICE_PATH_PRESSURE_EXT : BMP285_DEVICE_PATH_PRESSURE;
	struct baro_report report;
	ssize_t sz;
	int ret;


	/* get the driver */
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'bmp285 start' if the driver is not running)");
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.pressure);
	warnx("altitude:    %11.4f", (double)report.altitude);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("pressure:    %10.4f", (double)report.pressure);
		warnx("altitude:    %11.4f", (double)report.altitude);
		warnx("temperature K: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}

	close(fd);
	errx(0, "PASS");

}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
	const char *path = external_bus ? BMP285_DEVICE_PATH_PRESSURE_EXT : BMP285_DEVICE_PATH_PRESSURE;
	int fd =  open(path, O_RDONLY);

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
	BMP285 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);

}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(bool external_bus, unsigned altitude)
{
	const char *path = external_bus ? BMP285_DEVICE_PATH_PRESSURE_EXT : BMP285_DEVICE_PATH_PRESSURE;
	struct baro_report report;
	float   pressure;
	float   p1;

	/* get the driver */
	int fd = open(path, O_RDONLY);


	if (fd < 0) {
		err(1, "open failed (try 'bmp285 start' if the driver is not running)");
	}

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX)) {
		errx(1, "failed to set poll rate");
	}

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "sensor read failed");
		}

		pressure += report.pressure;
	}

	pressure /= 20;     /* average */
	pressure /= 10;     /* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15; /* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;   /* temperature gradient in degrees per metre */
	const float g  = 9.80665f;  /* gravity constant in m/s/s */
	const float R  = 287.05f;   /* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	float value = (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));
	warnx("power value is %10.4f", (double)value);
	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;
	warnx("p1 is %10.4fkPa at %um", (double)p1);

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK) {
		err(1, "BAROIOCSMSLPRESSURE");
	}

	close(fd);
	exit(0);
}


void
usage()
{
	warnx("missing command: try 'start',  'test',  'info',\n'reset',  'calibrate'");
	warnx("options:");
	warnx("    -X    (external bus)");
}

} // namespace

int
bmp285_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "X:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		default:
			bmp285::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmp285::start(external_bus);
	}


	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		bmp285::test(external_bus);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		bmp285::reset(external_bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmp285::info(external_bus);
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(verb, "calibrate")) {
		if (argc < 2) {
			errx(1, "missing altitude");
		}

		long altitude = strtol(argv[optind + 1], nullptr, 10);

		bmp285::calibrate(external_bus, altitude);
	}

	bmp285::usage();
	exit(1);
}

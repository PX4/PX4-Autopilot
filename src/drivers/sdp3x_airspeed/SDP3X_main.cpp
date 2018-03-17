/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "SDP3X.hpp"

// Driver 'main' command.
extern "C" __EXPORT int sdp3x_airspeed_main(int argc, char *argv[]);

// Local functions in support of the shell command.
namespace sdp3x_airspeed
{
SDP3X *g_dev = nullptr;

void start(int i2c_bus);
void stop();
void test();
void reset();
void info();

// Start the driver.
// This function call only returns once the driver is up and running
// or failed to detect the sensor.
void
start(int i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	g_dev = new SDP3X(i2c_bus, I2C_ADDRESS_1_SDP3X, PATH_SDP3X);

	/* check if the MS4525DO was instantiated */
	if (g_dev == nullptr) {
		goto fail;
	}

	/* try the next SDP3XDSO if init fails */
	if (OK != g_dev->Airspeed::init()) {
		delete g_dev;

		PX4_WARN("trying SDP3X 2");

		g_dev = new SDP3X(i2c_bus, I2C_ADDRESS_2_SDP3X, PATH_SDP3X);

		/* check if the SDP3XDSO was instantiated */
		if (g_dev == nullptr) {
			PX4_WARN("SDP3X was not instantiated");
			goto fail;
		}

		/* both versions failed if the init for the SDP3XDSO fails, give up */
		if (OK != g_dev->Airspeed::init()) {
			PX4_WARN("SDP3X init fail");
			goto fail;
		}
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(PATH_SDP3X, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_WARN("no SDP3X airspeed sensor connected");
	exit(1);
}

// stop the driver
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

// perform some basic functional tests on the driver;
// make sure we can collect data from the sensor in polled
// and automatic modes.
void test()
{
	int fd = open(PATH_SDP3X, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("%s open failed (try 'sdp3x_airspeed start' if the driver is not running", PATH_SDP3X);
		exit(1);
	}

	// do a simple demand read
	differential_pressure_s report;
	ssize_t sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_WARN("immediate read failed");
		exit(1);
	}

	PX4_WARN("single read");
	PX4_WARN("diff pressure: %d pa", (int)report.differential_pressure_filtered_pa);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_WARN("failed to set 2Hz poll rate");
		exit(1);
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		PX4_WARN("periodic read %u", i);
		PX4_WARN("diff pressure: %d pa", (int)report.differential_pressure_filtered_pa);
		PX4_WARN("temperature: %d C (0x%02x)", (int)report.temperature, (unsigned) report.temperature);
	}

	/* reset the sensor polling to its default rate */
	if (PX4_OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_WARN("failed to set default rate");
		exit(1);
	}

	exit(0);
}

// reset the driver
void reset()
{
	int fd = open(PATH_SDP3X, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

// print a little info about the driver
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace sdp3x_airspeed


static void
sdp3x_airspeed_usage()
{
	PX4_WARN("usage: sdp3x_airspeed command [options]");
	PX4_WARN("options:");
	PX4_WARN("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	PX4_WARN("command:");
	PX4_WARN("\tstart|stop|reset|test|info");
}

int
sdp3x_airspeed_main(int argc, char *argv[])
{
	int i2c_bus = PX4_I2C_BUS_DEFAULT;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bus") == 0) {
			if (argc > i + 1) {
				i2c_bus = atoi(argv[i + 1]);
			}
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		sdp3x_airspeed::start(i2c_bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		sdp3x_airspeed::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		sdp3x_airspeed::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		sdp3x_airspeed::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		sdp3x_airspeed::info();
	}

	sdp3x_airspeed_usage();
	exit(0);
}

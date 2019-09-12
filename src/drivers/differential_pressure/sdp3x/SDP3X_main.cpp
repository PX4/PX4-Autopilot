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

int start();
int start_bus(uint8_t i2c_bus);
int stop();
int reset();

/**
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
	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function call only returns once the driver is up and running
 * or failed to detect the sensor.
 */
int
start_bus(uint8_t i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	g_dev = new SDP3X(i2c_bus, I2C_ADDRESS_1_SDP3X, PATH_SDP3X);

	/* check if the SDP3XDSO was instantiated */
	if (g_dev == nullptr) {
		goto fail;
	}

	/* try the next SDP3XDSO if init fails */
	if (g_dev->init() != PX4_OK) {
		delete g_dev;

		g_dev = new SDP3X(i2c_bus, I2C_ADDRESS_2_SDP3X, PATH_SDP3X);

		/* check if the SDP3XDSO was instantiated */
		if (g_dev == nullptr) {
			PX4_ERR("SDP3X was not instantiated (RAM)");
			goto fail;
		}

		/* both versions failed if the init for the SDP3XDSO fails, give up */
		if (g_dev->init() != PX4_OK) {
			goto fail;
		}
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(PATH_SDP3X, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

// stop the driver
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

// reset the driver
int reset()
{
	int fd = px4_open(PATH_SDP3X, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed ");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

} // namespace sdp3x_airspeed


static void
sdp3x_airspeed_usage()
{
	PX4_INFO("usage: sdp3x_airspeed command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|reset");
}

int
sdp3x_airspeed_main(int argc, char *argv[])
{
	uint8_t i2c_bus = PX4_I2C_BUS_DEFAULT;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			sdp3x_airspeed_usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		sdp3x_airspeed_usage();
		return -1;
	}


	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return sdp3x_airspeed::start();

		} else {
			return sdp3x_airspeed::start_bus(i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return sdp3x_airspeed::stop();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return sdp3x_airspeed::reset();
	}

	sdp3x_airspeed_usage();
	return 0;
}

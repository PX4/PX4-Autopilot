/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file fxos8701cq.cpp
 * Driver for the NXP FXOS8701CQ 6-axis sensor with integrated linear accelerometer and
 * magnetometer connected via SPI.
 */

#include "FXOS8701CQ.hpp"

/**
 * Local functions in support of the shell command.
 */
namespace fxos8701cq
{

FXOS8701CQ	*g_dev;

void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	stop();
void	regdump();
void	usage();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_INFO("already started");
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_ACCEL_MAG)
		g_dev = new FXOS8701CQ(PX4_SPI_BUS_EXT, FXOS8701C_DEVICE_PATH_ACCEL, PX4_SPIDEV_EXT_ACCEL_MAG, rotation);
#else
		PX4_ERR("External SPI not available");
		exit(0);
#endif

	} else {
		g_dev = new FXOS8701CQ(PX4_SPI_BUS_SENSORS, FXOS8701C_DEVICE_PATH_ACCEL, PX4_SPIDEV_ACCEL_MAG, rotation);
	}

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating FXOS8701C obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(FXOS8701C_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	int fd_mag;
	fd_mag = open(FXOS8701C_DEVICE_PATH_MAG, O_RDONLY);

	/* don't fail if open cannot be opened */
	if (0 <= fd_mag) {
		if (ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}
	}

	close(fd_mag);
#endif

	close(fd);

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int rv = 1;
	int fd_accel = -1;
	sensor_accel_s accel_report;
	ssize_t sz;
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	int fd_mag = -1;
	int ret;
	struct mag_report m_report;
#endif

	/* get the driver */
	fd_accel = open(FXOS8701C_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd_accel < 0) {
		PX4_ERR("%s open failed", FXOS8701C_DEVICE_PATH_ACCEL);
		goto exit_none;
	}

	/* do a simple demand read */
	sz = read(fd_accel, &accel_report, sizeof(sensor_accel_s));

	if (sz != sizeof(sensor_accel_s)) {
		PX4_ERR("immediate read failed");
		goto exit_with_accel;
	}


	PX4_INFO("accel x: \t% 9.5f\tm/s^2", (double)accel_report.x);
	PX4_INFO("accel y: \t% 9.5f\tm/s^2", (double)accel_report.y);
	PX4_INFO("accel z: \t% 9.5f\tm/s^2", (double)accel_report.z);
	PX4_INFO("accel x: \t%d\traw", (int)accel_report.x_raw);
	PX4_INFO("accel y: \t%d\traw", (int)accel_report.y_raw);
	PX4_INFO("accel z: \t%d\traw", (int)accel_report.z_raw);

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	/* get the driver */
	fd_mag = open(FXOS8701C_DEVICE_PATH_MAG, O_RDONLY);

	if (fd_mag < 0) {
		PX4_ERR("%s open failed", FXOS8701C_DEVICE_PATH_MAG);
		goto exit_with_accel;
	}

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd_mag, MAGIOCGEXTERNAL, 0)) < 0) {
		PX4_ERR("failed to get if mag is onboard or external");
		goto exit_with_mag_accel;
	}

	PX4_INFO("mag device active: %s", ret ? "external" : "onboard");

	/* do a simple demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		PX4_ERR("immediate read failed");
		goto exit_with_mag_accel;
	}

	PX4_INFO("mag x: \t% 9.5f\tga", (double)m_report.x);
	PX4_INFO("mag y: \t% 9.5f\tga", (double)m_report.y);
	PX4_INFO("mag z: \t% 9.5f\tga", (double)m_report.z);
	PX4_INFO("mag x: \t%d\traw", (int)m_report.x_raw);
	PX4_INFO("mag y: \t%d\traw", (int)m_report.y_raw);
	PX4_INFO("mag z: \t%d\traw", (int)m_report.z_raw);
#endif

	/* reset to default polling */
	if (ioctl(fd_accel, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("reset to default polling");

	} else {
		rv = 0;
	}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
exit_with_mag_accel:
	close(fd_mag);
#endif

exit_with_accel:

	close(fd_accel);

	reset();

	if (rv == 0) {
		PX4_INFO("PASS");
	}

exit_none:
	exit(rv);
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(FXOS8701C_DEVICE_PATH_ACCEL, O_RDONLY);
	int rv = 1;

	if (fd < 0) {
		PX4_ERR("Open failed\n");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("accel pollrate reset failed");
		exit(1);
	}

	close(fd);

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	fd = open(FXOS8701C_DEVICE_PATH_MAG, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("mag could not be opened, external mag might be used");

	} else {
		/* no need to reset the mag as well, the reset() is the same */
		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			PX4_ERR("mag pollrate reset failed");

		} else {
			rv = 0;
		}
	}

	close(fd);
#endif

	exit(rv);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

void
stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	delete g_dev;
	g_dev = nullptr;

	exit(0);
}

/**
 * dump registers from device
 */
void
regdump()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

/**
 * trigger an error
 */
void
test_error()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	g_dev->test_error();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'stop', 'test', 'reset', 'testerror' or 'regdump'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R rotation");
}

} // namespace

int
fxos8701cq_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "XR:a:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			fxos8701cq::usage();
			exit(0);
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		fxos8701cq::start(external_bus, rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		fxos8701cq::test();
	}

	if (!strcmp(verb, "stop")) {
		fxos8701cq::stop();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		fxos8701cq::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		fxos8701cq::info();
	}

	/*
	 * dump device registers
	 */
	if (!strcmp(verb, "regdump")) {
		fxos8701cq::regdump();
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		fxos8701cq::test_error();
	}

	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}

/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "LSM9DS1_MAG.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/spi.h>

namespace lsm9ds1_mag
{

LSM9DS1_MAG *g_dev{nullptr};

static int start(enum Rotation rotation)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	// create the driver
	SPIBusIterator bus_iterator(SPIBusIterator::FilterType::InternalBus, DRV_MAG_DEVTYPE_ST_LSM9DS1_M);

	if (bus_iterator.next()) {
		g_dev = new LSM9DS1_MAG(bus_iterator.bus().bus, bus_iterator.devid(), rotation);
	}

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return -1;
	}

	if (!g_dev->Init()) {
		PX4_ERR("driver init failed");
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

static int stop()
{
	if (g_dev == nullptr) {
		PX4_WARN("driver not running");
	}

	g_dev->Stop();
	delete g_dev;

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_WARN("driver not running");
		return -1;
	}

	g_dev->PrintInfo();

	return 0;
}

static void usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'status'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace lsm9ds1_mag

extern "C" __EXPORT int lsm9ds1_mag_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;

	/* start options */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			lsm9ds1_mag::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		lsm9ds1_mag::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return lsm9ds1_mag::start(rotation);

	} else if (!strcmp(verb, "stop")) {
		return lsm9ds1_mag::stop();

	} else if (!strcmp(verb, "status")) {
		return lsm9ds1_mag::status();
	}

	lsm9ds1_mag::usage();

	return 0;
}

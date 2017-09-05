/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file baro.cpp
 * Driver for the simulated barometric pressure sensor
 */

#include <inttypes.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_time.h>
#include <px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>

#include <simulator/simulator.h>

#include <systemlib/err.h>

#include <VirtDevObj.hpp>

#define DEV_PATH "/dev/barosim"
#define MEASURE_INTERVAL_US (10000)

using namespace DriverFramework;

class BAROSIM : public VirtDevObj
{
public:
	BAROSIM(const char *path);
	BAROSIM() = delete;

	virtual ~BAROSIM();

	BAROSIM operator=(const BAROSIM &) = delete;

	virtual int init() override final;
	virtual int start() override final;
	virtual int devIOCTL(unsigned long cmd, unsigned long arg) override final;

private:
	virtual void _measure() override final;

private:
	/* altitude conversion calibration */
	/* TODO: this is currently not in use */
	unsigned _msl_pressure; /* in Pa */

	orb_advert_t _topic;
	int _orb_class_instance;
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int barosim_main(int argc, char *argv[]);

BAROSIM::BAROSIM(const char *path) :
	VirtDevObj("BAROSIM", path, BARO_BASE_DEVICE_PATH, MEASURE_INTERVAL_US),
	_msl_pressure(101325),
	_topic(nullptr),
	_orb_class_instance(-1)
{

}

BAROSIM::~BAROSIM()
{
	/* make sure we are truly inactive */
	stop();
}

int BAROSIM::init()
{
	int ret;

	ret = VirtDevObj::init();

	if (ret != 0) {
		PX4_WARN("Base class init failed (%d)", ret);
		ret = 1;
		goto out;
	}

	ret = start();

	if (ret != OK) {
		PX4_ERR("start failed (%d)", ret);
		return ret;
	}

out:
	return ret;
}

int BAROSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	switch (cmd) {

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return VirtDevObj::devIOCTL(cmd, arg);
}

int BAROSIM::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	return DevObj::start();
}

void BAROSIM::_measure()
{
	simulator::RawBaroData raw_report;
	baro_report report;
	Simulator *sim = Simulator::getInstance();

	if (sim == nullptr) {
		PX4_WARN("failed accessing simulator");
		return;
	}

	if (!sim->getBaroSample(&raw_report)) {
		return;
	}

	report.timestamp = hrt_absolute_time();
	report.pressure = raw_report.pressure;
	report.altitude = raw_report.altitude;
	report.temperature = raw_report.temperature;
	report.error_count = 0;

	/* fake device ID */
	report.device_id = 478459;

	if (_topic) {
		orb_publish(ORB_ID(sensor_baro), _topic, &report);

	} else {
		_topic = orb_advertise_multi(ORB_ID(sensor_baro), &report,
					     &_orb_class_instance, ORB_PRIO_HIGH);

		if (_topic == nullptr) {
			PX4_ERR("ADVERT FAIL");
			return;
		}
	}
}

namespace barosim
{

static BAROSIM *g_dev = nullptr;

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
static int start()
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	g_dev = new BAROSIM(DEV_PATH);

	if (g_dev == nullptr) {
		PX4_ERR("failed to allocate BAROSIM");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed to init BAROSIM");
		goto fail;
	}

	return 0;
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_WARN("driver start failed");
	return 1;
}

static void usage()
{
	PX4_WARN("missing command: try 'start'");
}

}; // namespace barosim

int
barosim_main(int argc, char *argv[])
{
	int ret;

	if (argc < 2) {
		barosim::usage();
		return 1;
	}

	const char *verb = argv[1];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ret = barosim::start();

	} else {
		barosim::usage();
		return 1;
	}

	return ret;
}

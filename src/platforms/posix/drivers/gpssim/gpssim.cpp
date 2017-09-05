/****************************************************************************
 *
 *   Copyright (c) 2015-2017 Roman Bapst. All rights reserved.
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
 * @file gpssim.cpp
 * Simulated GPS driver
 */

#include <sys/types.h>
#include <px4_defines.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <drivers/drv_gps.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <simulator/simulator.h>

#include <string>

#include <DevMgr.hpp>
#include <VirtDevObj.hpp>

using namespace DriverFramework;

#define DEV_PATH "/dev/gpssim"
#define MEASURE_INTERVAL_US (100000)

class GPSSIM : public VirtDevObj
{
public:
	GPSSIM(const std::string &path);
	GPSSIM() = delete;

	virtual ~GPSSIM();

	GPSSIM operator=(const GPSSIM &) = delete;

	virtual int init() override final;
	virtual int start() override final;
	virtual int devIOCTL(unsigned long cmd, unsigned long arg) override final;

private:
	virtual void _measure() override final;

private:
	orb_advert_t _topic;
	int _orb_class_instance;
};

extern "C" __EXPORT int gpssim_main(int argc, char *argv[]);

GPSSIM::GPSSIM(const std::string &path) :
	VirtDevObj("GPSSIM", path.c_str(), "/dev/gps", MEASURE_INTERVAL_US),
	_topic(nullptr),
	_orb_class_instance(-1)
{
}

GPSSIM::~GPSSIM()
{
	stop();

	if (_topic) {
		orb_unadvertise(_topic);
	}
}

int GPSSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {
	default:
		/* give it to parent if no one wants it */
		ret = VirtDevObj::devIOCTL(cmd, arg);
		break;
	}

	return ret;
}

int GPSSIM::init()
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

int GPSSIM::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	return DevObj::start();
}

void GPSSIM::_measure()
{
	simulator::RawGPSData raw_report;
	Simulator *sim = Simulator::getInstance();

	if (sim == nullptr) {
		PX4_WARN("failed accessing simulator");
		return;
	}

	if (!sim->getGPSSample(&raw_report)) {
		return;
	}

	vehicle_gps_position_s report = {};

	report.timestamp = hrt_absolute_time();
	report.timestamp_time_relative = 0;
	report.lat = raw_report.lat;
	report.lon = raw_report.lon;
	report.alt = raw_report.alt;
	report.eph = (float)raw_report.eph * 1e-2f;
	report.epv = (float)raw_report.epv * 1e-2f;
	report.vel_m_s = (float)(raw_report.vel) / 100.0f;
	report.vel_n_m_s = (float)(raw_report.vn) / 100.0f;
	report.vel_e_m_s = (float)(raw_report.ve) / 100.0f;
	report.vel_d_m_s = (float)(raw_report.vd) / 100.0f;
	report.cog_rad = (float)(raw_report.cog) * 3.1415f / (100.0f * 180.0f);
	report.fix_type = raw_report.fix_type;
	report.satellites_used = raw_report.satellites_visible;

	if (_topic) {
		orb_publish(ORB_ID(vehicle_gps_position), _topic, &report);

	} else {
		_topic = orb_advertise_multi(ORB_ID(vehicle_gps_position), &report,
					     &_orb_class_instance, ORB_PRIO_HIGH);

		if (_topic == nullptr) {
			PX4_WARN("ADVERT FAIL");
			return;
		}
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace gpssim
{

static GPSSIM *g_dev = nullptr;

static int start();
static int stop();
static void usage();

int start()
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	g_dev = new GPSSIM(DEV_PATH);

	if (g_dev == nullptr) {
		PX4_ERR("failed to allocate GPSSIM");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed to init GPSSIM");
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

int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		/* warn, but not an error */
		PX4_WARN("already stopped.");
	}

	return 0;
}

void usage()
{
	PX4_INFO("missing command: try 'start', 'stop'");
}

} // namespace

int gpssim_main(int argc, char *argv[])
{
	int ret;
	int myoptind = 1;

	if (argc <= 1) {
		gpssim::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ret = gpssim::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = gpssim::stop();
	}

	else  {
		gpssim::usage();
		ret = 1;
	}

	return ret;
}

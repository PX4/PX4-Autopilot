/****************************************************************************
 *
 *   Copyright (c) 2013, 2014, 2017 PX4 Development Team. All rights reserved.
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
 * @file meas_airspeed_sim.cpp
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Sarthak Kaingade
 * @author Simon Wilks
 * @author Thomas Gubler
 * @author Roman Bapst
 *
 * Driver for a simulated airspeed sensor.
 *
 */


#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <simulator/simulator.h>

#include <systemlib/airspeed.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>

#include <VirtDevObj.hpp>

#include <string>

#define DEV_PATH "/dev/measairspeedsim"
#define MEASURE_INTERVAL_US (10000)
#define FILTER_FREQ 1.2f

class MEASAirspeedSim : public DriverFramework::VirtDevObj
{
public:
	MEASAirspeedSim(const std::string &path);
	MEASAirspeedSim() = delete;

	virtual ~MEASAirspeedSim();

	MEASAirspeedSim operator=(const MEASAirspeedSim &) = delete;

	virtual int init() override final;
	virtual int start() override final;
	virtual int devIOCTL(unsigned long cmd, unsigned long arg) override final;

private:
	virtual void _measure() override final;

private:
	math::LowPassFilter2p _filter;

	orb_advert_t _topic;
	int _orb_class_instance;
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int measairspeedsim_main(int argc, char *argv[]);

MEASAirspeedSim::MEASAirspeedSim(const std::string &path) :
	VirtDevObj("MEASAIRSPEEDSIM", path.c_str(), AIRSPEED_BASE_DEVICE_PATH, MEASURE_INTERVAL_US),
	_filter(1e6 / MEASURE_INTERVAL_US, FILTER_FREQ),
	_topic(nullptr),
	_orb_class_instance(-1)
{
}

MEASAirspeedSim::~MEASAirspeedSim()
{
	/* make sure we are truly inactive */
	stop();

	if (_topic) {
		orb_unadvertise(_topic);
	}
}

int MEASAirspeedSim::devIOCTL(unsigned long cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCCALTEST: {
			return OK;
		} break;

	case DEVIOCGDEVICEID: {
			return (int)VirtDevObj::devIOCTL(cmd, arg);
		} break;
	}

	return OK;
}

int MEASAirspeedSim::init()
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

int MEASAirspeedSim::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	return DevObj::start();
}

void MEASAirspeedSim::_measure()
{
	Simulator *sim = Simulator::getInstance();

	simulator::RawAirspeedData raw_report;

	if (sim == nullptr) {
		PX4_WARN("failed accessing simulator");
		return;
	}

	if (!sim->getAirspeedSample(&raw_report)) {
		return;
	}

	float temperature = raw_report.temperature;
	float diff_press_pa_raw = raw_report.diff_pressure * 100.0f; // convert from millibar to bar

	differential_pressure_s report = {};

	report.timestamp = hrt_absolute_time();
	report.error_count = 0;
	report.temperature = temperature;
	report.differential_pressure_filtered_pa = _filter.apply(diff_press_pa_raw);
	report.differential_pressure_raw_pa = diff_press_pa_raw;

	if (_topic) {
		orb_publish(ORB_ID(differential_pressure), _topic, &report);

	} else {
		_topic = orb_advertise_multi(ORB_ID(differential_pressure), &report,
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
namespace meas_airspeed_sim
{

static MEASAirspeedSim *g_dev = nullptr;

static int start();
static int stop();
static void usage();

int start()
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	g_dev = new MEASAirspeedSim(DEV_PATH);

	if (g_dev == nullptr) {
		PX4_ERR("failed to allocate MEASAirspeedSim");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed to init MEASAirspeedSim");
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

int measairspeedsim_main(int argc, char *argv[])
{
	int ret;
	int myoptind = 1;

	if (argc <= 1) {
		meas_airspeed_sim::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ret = meas_airspeed_sim::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = meas_airspeed_sim::stop();
	}

	else  {
		meas_airspeed_sim::usage();
		ret = 1;
	}

	return ret;
}

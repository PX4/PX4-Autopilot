/****************************************************************************
 *
 * Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file accelsim.cpp
 * Driver for a simulated accelerometer.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <simulator/simulator.h>

#include <systemlib/err.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <VirtDevObj.hpp>

#define DEV_PATH "/dev/accelsim"
#define MEASURE_INTERVAL_US (4000)
#define FILTER_FREQ 30

extern "C" { __EXPORT int accelsim_main(int argc, char *argv[]); }

using namespace DriverFramework;

/*
 * TODO: The existing driver is not using the LowPassFilter2p
 */

class ACCELSIM : public VirtDevObj
{
public:
	ACCELSIM(const char *path);
	ACCELSIM() = delete;

	virtual ~ACCELSIM();

	ACCELSIM operator=(const ACCELSIM &) = delete;

	virtual int init() override final;
	virtual int start() override final;
	virtual int stop() override final;
	virtual int devIOCTL(unsigned long cmd, unsigned long arg) override final;

private:
	virtual void _measure() override final;

private:
	orb_advert_t _topic;
	int _orb_class_instance;

	Integrator _integrator;
	math::LowPassFilter2p _filter_x;
	math::LowPassFilter2p _filter_y;
	math::LowPassFilter2p _filter_z;
};


ACCELSIM::ACCELSIM(const char *path) :
	VirtDevObj("ACCELSIM", path, ACCEL_BASE_DEVICE_PATH, MEASURE_INTERVAL_US),
	_topic(nullptr),
	_orb_class_instance(-1),
	_integrator(MEASURE_INTERVAL_US, true),
	_filter_x(1e6 / MEASURE_INTERVAL_US, FILTER_FREQ),
	_filter_y(1e6 / MEASURE_INTERVAL_US, FILTER_FREQ),
	_filter_z(1e6 / MEASURE_INTERVAL_US, FILTER_FREQ)
{
	m_id.dev_id_s.bus = 1;
	m_id.dev_id_s.devtype = DRV_ACC_DEVTYPE_ACCELSIM;
}

ACCELSIM::~ACCELSIM()
{
	/* make sure we are truly inactive */
	stop();
}

int ACCELSIM::init()
{
	int ret;

	ret = VirtDevObj::init();

	if (ret != OK) {
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

int ACCELSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCCALTEST:
		return OK;

	case DEVIOCGDEVICEID: {
			return (int)VirtDevObj::devIOCTL(cmd, arg);
		} break;
	}

	return OK;
}

int ACCELSIM::start()
{
	/* make sure we are stopped first */
	stop();

	int ret = VirtDevObj::start();

	if (ret != 0) {
		PX4_ERR("ACCELSIM::start base class start failed");
	}

	return (ret != 0) ? -1 : 0;
}

int ACCELSIM::stop()
{
	return VirtDevObj::stop();
}

void ACCELSIM::_measure()
{
	simulator::RawAccelData raw_report;
	accel_report report;
	Simulator *sim = Simulator::getInstance();

	if (sim == nullptr) {
		PX4_WARN("failed accessing simulator");
		return;
	}

	if (!sim->getAccelReport(&raw_report)) {
		return;
	}

	report.timestamp = hrt_absolute_time();

	math::Vector<3> aval(raw_report.x, raw_report.y, raw_report.z);
	math::Vector<3> aval_integrated;

	bool accel_notify = _integrator.put(report.timestamp, aval, aval_integrated, report.integral_dt);

	if (!accel_notify) {
		return;
	}

	report.error_count = 0;

	report.x = raw_report.x;
	report.y = raw_report.y;
	report.z = raw_report.z;

	report.x_integral = aval_integrated(0);
	report.y_integral = aval_integrated(1);
	report.z_integral = aval_integrated(2);

	report.temperature = raw_report.temperature;
	report.range_m_s2 = 0;
	report.scaling = 0;

	report.x_raw = 0;
	report.y_raw = 0;
	report.z_raw = 0;
	report.temperature_raw = 0;

	report.device_id = 6789478;

	if (_topic) {
		orb_publish(ORB_ID(sensor_accel), _topic, &report);

	} else {
		_topic = orb_advertise_multi(ORB_ID(sensor_accel), &report,
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
namespace accelsim
{

static ACCELSIM *g_dev;

static void usage();
static int start();

int start()
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	g_dev = new ACCELSIM(DEV_PATH);

	if (g_dev == nullptr) {
		PX4_ERR("failed to allocate ACCELSIM");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed to init ACCELSIM");
		goto fail;
	}

	return 0;
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return 1;
}

void usage()
{
	PX4_WARN("Usage: accelsim 'start'");
}

} // namespace

int
accelsim_main(int argc, char *argv[])
{
	int ret;
	int myoptind = 1;

	if (argc <= 1) {
		accelsim::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ret = accelsim::start();
	}

	else {
		accelsim::usage();
		return 1;
	}

	return ret;
}

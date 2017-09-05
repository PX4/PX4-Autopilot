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
 * @file gyrosim.cpp
 *
 * Driver for the simulated gyro
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Mark Charlebois
 */

#include <inttypes.h>

#include <px4_config.h>
#include <px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string>

#include <simulator/simulator.h>

#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/device.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_gyro.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <VirtDevObj.hpp>

using namespace DriverFramework;

#define DEV_PATH "/dev/gyrosim"
#define MEASURE_INTERVAL_US (2500)
#define FILTER_FREQ 30

class GYROSIM : public VirtDevObj
{
public:
	GYROSIM(const std::string &path);
	GYROSIM() = delete;

	virtual ~GYROSIM();

	GYROSIM operator=(const GYROSIM &) = delete;

	virtual int init() override final;
	virtual int start() override final;
	virtual int devIOCTL(unsigned long cmd, unsigned long arg) override final;

private:
	virtual void _measure() override final;

private:
	Integrator _integrator;
	math::LowPassFilter2p _filter_x;
	math::LowPassFilter2p _filter_y;
	math::LowPassFilter2p _filter_z;

	orb_advert_t _topic;
	int _orb_class_instance;
};

/** driver 'main' command */
extern "C" { __EXPORT int gyrosim_main(int argc, char *argv[]); }

GYROSIM::GYROSIM(const std::string &path) :
	VirtDevObj("GYROSIM", path.c_str(), GYRO_BASE_DEVICE_PATH, MEASURE_INTERVAL_US),
	_integrator(MEASURE_INTERVAL_US, true),
	_filter_x(1e6 / MEASURE_INTERVAL_US, FILTER_FREQ),
	_filter_y(1e6 / MEASURE_INTERVAL_US, FILTER_FREQ),
	_filter_z(1e6 / MEASURE_INTERVAL_US, FILTER_FREQ),
	_topic(nullptr),
	_orb_class_instance(-1)
{

	m_id.dev_id_s.bus = 1;
	m_id.dev_id_s.devtype = DRV_GYR_DEVTYPE_GYROSIM;
}

GYROSIM::~GYROSIM()
{
	/* make sure we are truly inactive */
	stop();
}

int GYROSIM::devIOCTL(unsigned long cmd, unsigned long arg)
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

int GYROSIM::init()
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

int GYROSIM::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	return DevObj::start();
}

void GYROSIM::_measure()
{
	simulator::RawGyroData raw_report;
	Simulator *sim = Simulator::getInstance();

	if (sim == nullptr) {
		PX4_WARN("failed accessing simulator");
		return;
	}

	if (!sim->getGyroReport(&raw_report)) {
		return;
	}

	uint64_t timestamp = hrt_absolute_time();
	uint64_t integral_dt;

	float x_filt = _filter_x.apply(raw_report.gyro_x);
	float y_filt = _filter_y.apply(raw_report.gyro_y);
	float z_filt = _filter_z.apply(raw_report.gyro_z);

	math::Vector<3> val(raw_report.gyro_x, raw_report.gyro_y, raw_report.gyro_z);
	math::Vector<3> val_integrated;

	bool notify = _integrator.put(timestamp, val, val_integrated, integral_dt);

	if (!notify) {
		return;
	}

	gyro_report report = {
		.timestamp = timestamp,
		.integral_dt = integral_dt,
		.error_count = 0,
		.x = x_filt,
		.y = y_filt,
		.z = z_filt,
		.x_integral = val_integrated(0),
		.y_integral = val_integrated(1),
		.z_integral = val_integrated(2),
		.temperature = raw_report.temperature,
		// TODO: get these right
		.range_rad_s = -1.0f,
		.scaling = -1.0f,
		.device_id = 3467548,
		.x_raw = 0,
		.y_raw = 0,
		.z_raw = 0,
		.temperature_raw = 0,
	};

	if (_topic) {
		orb_publish(ORB_ID(sensor_gyro), _topic, &report);

	} else {
		_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &report,
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
namespace gyrosim
{

static GYROSIM *g_dev = nullptr;

static int start();
static int stop();
static void usage();

int start()
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	g_dev = new GYROSIM(DEV_PATH);

	if (g_dev == nullptr) {
		PX4_ERR("failed to allocate GYROSIM");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed to init GYROSIM");
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

int gyrosim_main(int argc, char *argv[])
{
	int ret;
	int myoptind = 1;

	if (argc <= 1) {
		gyrosim::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ret = gyrosim::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = gyrosim::stop();
	}

	else  {
		gyrosim::usage();
		ret = 1;
	}

	return ret;
}

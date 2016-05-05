/****************************************************************************
 *
 * Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_trone_wrapper.cpp
 * Driver to access the TROne of the DriverFramework.
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
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
#include <string>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_range_finder.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

#include <trone/TROne.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_trone_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfTROneWrapper : public TROne
{
public:
	DfTROneWrapper();
	~DfTROneWrapper();


	/**
	 * Start automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		start();

	/**
	 * Stop automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		stop();

private:
	int _publish(struct range_sensor_data &data);

	orb_advert_t		_range_topic;

	int			_orb_class_instance;

	// perf_counter_t		_range_sample_perf;

};

DfTROneWrapper::DfTROneWrapper(/*enum Rotation rotation*/) :
	TROne(TRONE_DEVICE_PATH),
	_range_topic(nullptr),
	_orb_class_instance(-1)
{
}

DfTROneWrapper::~DfTROneWrapper()
{
}

int DfTROneWrapper::start()
{
	struct distance_sensor_s d;
	_range_topic = orb_advertise_multi(ORB_ID(distance_sensor), &d,
					   &_orb_class_instance, ORB_PRIO_DEFAULT);

	int ret;

	/* Init device and start sensor. */
	ret = init();

	if (ret != 0) {
		PX4_ERR("TROne init fail: %d", ret);
		return ret;
	}

	ret = TROne::start();

	if (ret != 0) {
		PX4_ERR("TROne start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfTROneWrapper::stop()
{
	/* Stop sensor. */
	int ret = TROne::stop();

	if (ret != 0) {
		PX4_ERR("TROne stop fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfTROneWrapper::_publish(struct range_sensor_data &data)
{
	if (!_range_topic) {
		return 1;
	}

	struct distance_sensor_s d;

	memset(&d, 0, sizeof(d));

	d.timestamp = hrt_absolute_time();

	d.min_distance = float(TRONE_MIN_DISTANCE); /* m */

	d.max_distance = float(TRONE_MAX_DISTANCE); /* m */

	d.current_distance = float(data.dist);

	d.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

	d.id = 0; // TODO set proper ID

	d.orientation = 0; // TODO no idea what to put here

	d.covariance = 0.0f;

	orb_publish(ORB_ID(distance_sensor), _range_topic, &d);

	/* Notify anyone waiting for data. */
	DevMgr::updateNotify(*this);

	return 0;
};


namespace df_trone_wrapper
{

DfTROneWrapper *g_dev = nullptr;

int start();
int stop();
int info();
int probe();
void usage();

int start()
{
	PX4_ERR("start");
	g_dev = new DfTROneWrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfTROneWrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfTROneWrapper start failed");
		return ret;
	}

	// Open the range sensor
	DevHandle h;
	DevMgr::getHandle(TRONE_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    TRONE_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	return 0;
}

/**
 * Who am i
 */
int
probe()
{
	int ret;

	if (g_dev == nullptr) {
		ret = start();

		if (ret) {
			PX4_ERR("Failed to start");
			return ret;
		}
	}

	ret = g_dev->probe();

	if (ret) {
		PX4_ERR("Failed to probe");
		return ret;
	}

	PX4_DEBUG("state @ %p", g_dev);

	return 0;
}


void
usage()
{
	PX4_WARN("Usage: df_trone_wrapper 'start', 'info', 'stop'");
}

} // namespace df_trone_wrapper


int
df_trone_wrapper_main(int argc, char *argv[])
{
	int ch;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		default:
			df_trone_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_trone_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_trone_wrapper::start(/*rotation*/);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_trone_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_trone_wrapper::info();
	}

	else if (!strcmp(verb, "probe")) {
		ret = df_trone_wrapper::probe();
	}

	else {
		df_trone_wrapper::usage();
		return 1;
	}

	return ret;
}

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
 * @file df_bebop_rangefinder_wrapper.cpp
 * Driver to access the Bebop rangefinder of the DriverFramework.
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
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

#include <bebop_rangefinder/BebopRangeFinder.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_bebop_rangefinder_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfBebopRangeFinderWrapper : public BebopRangeFinder
{
public:
	DfBebopRangeFinderWrapper();
	~DfBebopRangeFinderWrapper() = default;


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
	int _publish(struct bebop_range &data);

	orb_advert_t		_range_topic;

	int			_orb_class_instance;

	// perf_counter_t		_range_sample_perf;

};

DfBebopRangeFinderWrapper::DfBebopRangeFinderWrapper(/*enum Rotation rotation*/) :
	BebopRangeFinder(BEBOP_RANGEFINDER_DEVICE_PATH),
	_range_topic(nullptr),
	_orb_class_instance(-1)
{
}

int DfBebopRangeFinderWrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("BebopRangeFinder init fail: %d", ret);
		return ret;
	}

	ret = BebopRangeFinder::start();

	if (ret != 0) {
		PX4_ERR("BebopRangeFinder start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfBebopRangeFinderWrapper::stop()
{
	/* Stop sensor. */
	int ret = BebopRangeFinder::stop();

	if (ret != 0) {
		PX4_ERR("BebopRangeFinder stop fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfBebopRangeFinderWrapper::_publish(struct bebop_range &data)
{
	struct distance_sensor_s distance_data;

	memset(&distance_data, 0, sizeof(distance_sensor_s));

	distance_data.timestamp = hrt_absolute_time();
	distance_data.min_distance = float(BEBOP_RANGEFINDER_MIN_DISTANCE_M); /* m */
	distance_data.max_distance = float(BEBOP_RANGEFINDER_MAX_DISTANCE_M); /* m */

	distance_data.current_distance = float(data.height_m);

	distance_data.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;

	distance_data.id = 0; // TODO set proper ID

	distance_data.orientation = 25; // MAV_SENSOR_ROTATION_PITCH_270

	distance_data.covariance = 1.0f; // TODO set correct value

	if (_range_topic == nullptr) {
		_range_topic = orb_advertise_multi(ORB_ID(distance_sensor), &distance_data,
						   &_orb_class_instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(distance_sensor), _range_topic, &distance_data);
	}

	/* Notify anyone waiting for data. */
	DevMgr::updateNotify(*this);

	return 0;
};


namespace df_bebop_rangefinder_wrapper
{

DfBebopRangeFinderWrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
	g_dev = new DfBebopRangeFinderWrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfBebopRangeFinderWrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfBebopRangeFinderWrapper start failed");
		return ret;
	}

	// Open the range sensor
	DevHandle h;
	DevMgr::getHandle(BEBOP_RANGEFINDER_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    BEBOP_RANGEFINDER_DEVICE_PATH, h.getError());
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

void
usage()
{
	PX4_WARN("Usage: df_bebop_rangefinder_wrapper 'start', 'info', 'stop'");
}

} // namespace df_bebop_rangefinder_wrapper


int
df_bebop_rangefinder_wrapper_main(int argc, char *argv[])
{
	int ch;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		// Add rotation if necessary
		default:
			df_bebop_rangefinder_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_bebop_rangefinder_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_bebop_rangefinder_wrapper::start(/*rotation*/);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_bebop_rangefinder_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_bebop_rangefinder_wrapper::info();
	}

	else {
		df_bebop_rangefinder_wrapper::usage();
		return 1;
	}

	return ret;
}

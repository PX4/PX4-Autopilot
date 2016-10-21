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
 * @file df_isl29501_wrapper.cpp
 * Driver to access the ISL29501 of the DriverFramework.
 *
 * @author Zach Lovett <zach.lovett@3drobotics.com>
 * @author Siddharth B Purohit <sid@3drobotics.com>
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

#include <isl29501/isl29501.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_isl29501_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfISL29501Wrapper : public ISL29501
{
public:
	DfISL29501Wrapper();
	~DfISL29501Wrapper();


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

DfISL29501Wrapper::DfISL29501Wrapper(/*enum Rotation rotation*/) :
	ISL29501(ISL_DEVICE_PATH),
	_range_topic(nullptr),
	_orb_class_instance(-1)
{
}

DfISL29501Wrapper::~DfISL29501Wrapper()
{
}

int DfISL29501Wrapper::start()
{
	int ret;
	ret = ISL29501::init();

	if (ret != 0) {
		PX4_ERR("ISL init fail: %d", ret);
		return ret;
	}

	ret = ISL29501::start();

	if (ret != 0) {
		PX4_ERR("ISL start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfISL29501Wrapper::stop()
{
	/* Stop sensor. */
	int ret = ISL29501::stop();

	if (ret != 0) {
		PX4_ERR("ISL stop fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfISL29501Wrapper::_publish(struct range_sensor_data &data)
{
	struct distance_sensor_s d;

	memset(&d, 0, sizeof(d));

	d.timestamp = hrt_absolute_time();

	d.min_distance = float(ISL_MIN_DISTANCE); /* m */

	d.max_distance = float(ISL_MAX_DISTANCE); /* m */

	d.current_distance = float(data.dist);

	d.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

	d.id = 0; // TODO set proper ID

	d.covariance = 0.0f;

	if (_range_topic == nullptr) {
		_range_topic = orb_advertise_multi(ORB_ID(distance_sensor), &d,
						   &_orb_class_instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(distance_sensor), _range_topic, &d);
	}

	/* Notify anyone waiting for data. */
	DevMgr::updateNotify(*this);

	return 0;
};


namespace df_isl29501_wrapper
{

DfISL29501Wrapper *g_dev = nullptr;

int start();
int stop();
int info();
int probe();
int calibration();
void usage();

int start()
{
	PX4_ERR("start");
	g_dev = new DfISL29501Wrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfISL29501Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfISL29501Wrapper start failed");
		return ret;
	}

	// Open the range sensor
	DevHandle h;
	DevMgr::getHandle(ISL_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    ISL_DEVICE_PATH, h.getError());
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

/**
 * Calibration
 * runs calibration routine for ISL29501
 * TODO: implement calibration user interface and parameter system to store calib
 * Note: Currently only serves debugging purpose, user is required to manually
 * set offset inside code.
 */
int
calibration()
{
	int ret;

	if (g_dev == nullptr) {
		ret = start();

		if (ret) {
			PX4_ERR("Failed to start");
			return ret;
		}
	}

	ret = g_dev->calibration();

	if (ret) {
		PX4_ERR("Failed to calibrate");
		return ret;
	}

	PX4_DEBUG("state @ %p", g_dev);

	return 0;
}


void
usage()
{
	PX4_WARN("Usage: df_isl_wrapper 'start', 'info', 'stop', 'calib', 'probe'");
}

} // namespace df_isl_wrapper


int
df_isl29501_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_isl29501_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_isl29501_wrapper::start(/*rotation*/);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_isl29501_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_isl29501_wrapper::info();
	}

	else if (!strcmp(verb, "probe")) {
		ret = df_isl29501_wrapper::probe();
	}

	else if (!strcmp(verb, "calib")) {
		df_isl29501_wrapper::calibration();
		return 1;
	}

	else {
		df_isl29501_wrapper::usage();
		return 1;
	}

	return ret;
}

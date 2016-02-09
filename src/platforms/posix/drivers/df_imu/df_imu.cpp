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
 * @file df_imu.cpp
 * Lightweight driver to access IMU driver of DriverFramework.
 *
 * @author Julian Oes <julian@oes.ch>
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

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_accel.h>

#include <board_config.h>
//#include <mathlib/math/filter/LowPassFilter2p.hpp>
//#include <lib/conversion/rotation.h>
//
#include <mpu9250/MPU9250.hpp>
#include <DevMgr.hpp>
#include <VirtDevObj.hpp>


#define ACCELSIM_DEVICE_PATH_ACCEL	"/dev/df_accel"


extern "C" { __EXPORT int df_imu_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfImu : public MPU9250
{
public:
	DfImu(/*enum Rotation rotation*/);
	~DfImu();

	//enum Rotation		_rotation;

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
	virtual int _publish_callback(struct imu_sensor_data &data);

	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;

	perf_counter_t		_accel_sample_perf;

};

DfImu::DfImu(/*enum Rotation rotation*/) :
	MPU9250(IMU_DEVICE_PATH),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "df_accel_read"))
	/*_rotation(rotation)*/
{
	m_id.dev_id_s.bus = 1;
	m_id.dev_id_s.devtype = DRV_ACC_DEVTYPE_MPU9250;
}

DfImu::~DfImu()
{
}

int DfImu::start()
{
	struct accel_report arp = {};

	arp.timestamp = 1234;

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_accel_topic == nullptr) {
		PX4_WARN("ADVERT ERR");
		return -1;
	}

	/* init device and start sensor */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("MPU9250 init fail: %d", ret);
		return ret;
	}

	ret = MPU9250::start();

	if (ret != 0) {
		PX4_ERR("MPU9250 start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfImu::stop()
{
	/* stop sensor */
	return 0;
}

int DfImu::_publish_callback(struct imu_sensor_data &data)
{
	accel_report accel_report = {};

	accel_report.timestamp = data.last_read_time_usec;

	// TODO: remove these (or get the values)
	accel_report.x_raw = NAN;
	accel_report.y_raw = NAN;
	accel_report.z_raw = NAN;

	accel_report.x = data.accel_m_s2_x;
	accel_report.y = data.accel_m_s2_y;
	accel_report.z = data.accel_m_s2_z;

	// TODO: get these right
	accel_report.scaling = -1.0f;
	accel_report.range_m_s2 = -1.0f;

	if (!(m_pub_blocked)) {
		/* publish it */

		if (_accel_topic != nullptr) {
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		}
	}

	/* notify anyone waiting for data */
	DevMgr::updateNotify(*this);

	return 0;
};


namespace df_imu
{

DfImu *g_dev = nullptr;

int start(/* enum Rotation rotation */);
int stop();
int info();
void usage();

int start(/*enum Rotation rotation*/)
{
	g_dev = new DfImu(/*rotation*/);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfImu object");
		return -1;
	}

	int ret = g_dev->start();
	if (ret != 0) {
		PX4_ERR("DfImu start failed");
		return ret;
	}

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(IMU_DEVICE_PATH, h);
	if (!h.isValid())
	{
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			IMU_DEVICE_PATH, h.getError());
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
	PX4_WARN("Usage: df_imu 'start', 'info', 'stop'");
	PX4_WARN("options:");
	//PX4_WARN("    -R rotation");
}

} // namespace

int
df_imu_main(int argc, char *argv[])
{
	int ch;
	// enum Rotation rotation = ROTATION_NONE;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		//case 'R':
		//	rotation = (enum Rotation)atoi(myoptarg);
		//	break;

		default:
			df_imu::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_imu::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_imu::start(/*rotation*/);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_imu::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_imu::info();
	}

	else {
		df_imu::usage();
		return 1;
	}

	return ret;
}

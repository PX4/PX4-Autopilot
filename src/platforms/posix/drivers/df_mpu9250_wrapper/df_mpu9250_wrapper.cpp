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
 * @file df_mpu9250_wrapper.cpp
 * Lightweight driver to access the MPU9250 of the DriverFramework.
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
#include <drivers/drv_gyro.h>
#include <drivers/device/integrator.h>

#include <uORB/topics/parameter_update.h>

#include <mpu9250/MPU9250.hpp>
#include <DevMgr.hpp>


// publish frequency of 250 Hz
#define MPU9250_PUBLISH_INTERVAL_US 4000


extern "C" { __EXPORT int df_mpu9250_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfMpu9250Wrapper : public MPU9250
{
public:
	DfMpu9250Wrapper(/*enum Rotation rotation*/);
	~DfMpu9250Wrapper();


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
	int _publish(struct imu_sensor_data &data);

	void _update_accel_calibration();
	void _update_gyro_calibration();

	//enum Rotation		_rotation;

	orb_advert_t		    _accel_topic;
	orb_advert_t		    _gyro_topic;

	int			    _param_update_sub;

	struct accel_calibration_s {
		float x_offset;
		float x_scale;
		float y_offset;
		float y_scale;
		float z_offset;
		float z_scale;
	} _accel_calibration;

	struct gyro_calibration_s {
		float x_offset;
		float x_scale;
		float y_offset;
		float y_scale;
		float z_offset;
		float z_scale;
	} _gyro_calibration;

	int			    _accel_orb_class_instance;
	int			    _gyro_orb_class_instance;

	Integrator		    _accel_int;
	Integrator		    _gyro_int;

	perf_counter_t		    _accel_sample_perf;
	perf_counter_t		    _gyro_sample_perf;

};

DfMpu9250Wrapper::DfMpu9250Wrapper(/*enum Rotation rotation*/) :
	MPU9250(IMU_DEVICE_PATH),
	_accel_topic(nullptr),
	_gyro_topic(nullptr),
	_param_update_sub(-1),
	_accel_calibration{},
	_gyro_calibration{},
	_accel_orb_class_instance(-1),
	_gyro_orb_class_instance(-1),
	_accel_int(MPU9250_PUBLISH_INTERVAL_US, false),
	_gyro_int(MPU9250_PUBLISH_INTERVAL_US, true),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "df_accel_read")),
	_gyro_sample_perf(perf_alloc(PC_ELAPSED, "df_gyro_read"))
	/*_rotation(rotation)*/
{
	// Set sane default calibration values
	_accel_calibration.x_scale = 1.0f;
	_accel_calibration.y_scale = 1.0f;
	_accel_calibration.z_scale = 1.0f;
	_accel_calibration.x_offset = 0.0f;
	_accel_calibration.y_offset = 0.0f;
	_accel_calibration.z_offset = 0.0f;

	_gyro_calibration.x_scale = 1.0f;
	_gyro_calibration.y_scale = 1.0f;
	_gyro_calibration.z_scale = 1.0f;
	_gyro_calibration.x_offset = 0.0f;
	_gyro_calibration.y_offset = 0.0f;
	_gyro_calibration.z_offset = 0.0f;
}

DfMpu9250Wrapper::~DfMpu9250Wrapper()
{
	perf_free(_accel_sample_perf);
	perf_free(_gyro_sample_perf);
}

int DfMpu9250Wrapper::start()
{
	// TODO: don't publish garbage here
	accel_report accel_report = {};
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &accel_report,
					   &_accel_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_accel_topic == nullptr) {
		PX4_ERR("sensor_accel advert fail");
		return -1;
	}

	// TODO: don't publish garbage here
	gyro_report gyro_report = {};
	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &gyro_report,
					  &_gyro_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_gyro_topic == nullptr) {
		PX4_ERR("sensor_gyro advert fail");
		return -1;
	}

	/* Subscribe to param update topic. */
	if (_param_update_sub < 0) {
		_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	}

	/* Init device and start sensor. */
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

	PX4_INFO("MPU9250 device id is: %d", m_id.dev_id);

	/* Force getting the calibration values. */
	_update_accel_calibration();
	_update_gyro_calibration();

	return 0;
}

int DfMpu9250Wrapper::stop()
{
	/* Stop sensor. */
	int ret = MPU9250::stop();

	if (ret != 0) {
		PX4_ERR("MPU9250 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

void DfMpu9250Wrapper::_update_gyro_calibration()
{
	// TODO: replace magic number
	for (unsigned i = 0; i < 3; ++i) {

		// TODO: remove printfs and add error counter

		char str[30];
		(void)sprintf(str, "CAL_GYRO%u_ID", i);
		int32_t device_id;
		int res = param_get(param_find(str), &device_id);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
			continue;
		}

		if ((uint32_t)device_id != m_id.dev_id) {
			continue;
		}

		(void)sprintf(str, "CAL_GYRO%u_XSCALE", i);
		res = param_get(param_find(str), &_gyro_calibration.x_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_GYRO%u_YSCALE", i);
		res = param_get(param_find(str), &_gyro_calibration.y_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_GYRO%u_ZSCALE", i);
		res = param_get(param_find(str), &_gyro_calibration.z_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_GYRO%u_XOFF", i);
		res = param_get(param_find(str), &_gyro_calibration.x_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_GYRO%u_YOFF", i);
		res = param_get(param_find(str), &_gyro_calibration.y_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_GYRO%u_ZOFF", i);
		res = param_get(param_find(str), &_gyro_calibration.z_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		// We got calibration values, let's exit.
		return;
	}

	_gyro_calibration.x_scale = 1.0f;
	_gyro_calibration.y_scale = 1.0f;
	_gyro_calibration.z_scale = 1.0f;
	_gyro_calibration.x_offset = 0.0f;
	_gyro_calibration.y_offset = 0.0f;
	_gyro_calibration.z_offset = 0.0f;
}

void DfMpu9250Wrapper::_update_accel_calibration()
{
	// TODO: replace magic number
	for (unsigned i = 0; i < 3; ++i) {

		// TODO: remove printfs and add error counter

		char str[30];
		(void)sprintf(str, "CAL_ACC%u_ID", i);
		int32_t device_id;
		int res = param_get(param_find(str), &device_id);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
			continue;
		}

		if ((uint32_t)device_id != m_id.dev_id) {
			continue;
		}

		(void)sprintf(str, "CAL_ACC%u_XSCALE", i);
		res = param_get(param_find(str), &_accel_calibration.x_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_YSCALE", i);
		res = param_get(param_find(str), &_accel_calibration.y_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_ZSCALE", i);
		res = param_get(param_find(str), &_accel_calibration.z_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_XOFF", i);
		res = param_get(param_find(str), &_accel_calibration.x_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_YOFF", i);
		res = param_get(param_find(str), &_accel_calibration.y_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_ZOFF", i);
		res = param_get(param_find(str), &_accel_calibration.z_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		// We got calibration values, let's exit.
		return;
	}

	// Set sane default calibration values
	_accel_calibration.x_scale = 1.0f;
	_accel_calibration.y_scale = 1.0f;
	_accel_calibration.z_scale = 1.0f;
	_accel_calibration.x_offset = 0.0f;
	_accel_calibration.y_offset = 0.0f;
	_accel_calibration.z_offset = 0.0f;
}

int DfMpu9250Wrapper::_publish(struct imu_sensor_data &data)
{
	bool should_notify = false;

	/* Check if calibration values are still up-to-date. */
	bool updated;
	orb_check(_param_update_sub, &updated);

	if (updated) {
		parameter_update_s parameter_update;
		orb_copy(ORB_ID(parameter_update), _param_update_sub, &parameter_update);

		_update_accel_calibration();
		_update_gyro_calibration();
	}

	/* Publish accel first. */
	perf_begin(_accel_sample_perf);

	accel_report accel_report = {};
	accel_report.timestamp = hrt_absolute_time();

	// TODO: remove these (or get the values)
	accel_report.x_raw = NAN;
	accel_report.y_raw = NAN;
	accel_report.z_raw = NAN;
	accel_report.x = (data.accel_m_s2_x - _accel_calibration.x_offset) * _accel_calibration.x_scale;
	accel_report.y = (data.accel_m_s2_y - _accel_calibration.y_offset) * _accel_calibration.y_scale;
	accel_report.z = (data.accel_m_s2_z - _accel_calibration.z_offset) * _accel_calibration.z_scale;

	math::Vector<3> accel_val(accel_report.x,
				  accel_report.y,
				  accel_report.z);
	math::Vector<3> accel_val_integrated;

	const bool should_publish_accel = _accel_int.put(accel_report.timestamp,
					  accel_val,
					  accel_val_integrated,
					  accel_report.integral_dt);

	accel_report.x_integral = accel_val_integrated(0);
	accel_report.y_integral = accel_val_integrated(1);
	accel_report.z_integral = accel_val_integrated(2);


	// TODO: get these right
	accel_report.scaling = -1.0f;
	accel_report.range_m_s2 = -1.0f;

	accel_report.device_id = m_id.dev_id;

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked) && should_publish_accel) {

		if (_accel_topic != nullptr) {
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		}

		should_notify = true;
	}

	perf_end(_accel_sample_perf);


	/* Then publish gyro. */
	perf_begin(_gyro_sample_perf);

	gyro_report gyro_report = {};
	gyro_report.timestamp = hrt_absolute_time();

	// TODO: remove these (or get the values)
	gyro_report.x_raw = NAN;
	gyro_report.y_raw = NAN;
	gyro_report.z_raw = NAN;
	gyro_report.x = (data.gyro_rad_s_x - _gyro_calibration.x_offset) * _gyro_calibration.x_scale;
	gyro_report.y = (data.gyro_rad_s_y - _gyro_calibration.y_offset) * _gyro_calibration.y_scale;
	gyro_report.z = (data.gyro_rad_s_z - _gyro_calibration.z_offset) * _gyro_calibration.z_scale;

	math::Vector<3> gyro_val(gyro_report.x,
				 gyro_report.y,
				 gyro_report.z);
	math::Vector<3> gyro_val_integrated(gyro_report.x,
					    gyro_report.y,
					    gyro_report.z);

	const bool should_publish_gyro = _gyro_int.put(gyro_report.timestamp,
					 gyro_val,
					 gyro_val_integrated,
					 gyro_report.integral_dt);

	gyro_report.x_integral = gyro_val_integrated(0);
	gyro_report.y_integral = gyro_val_integrated(1);
	gyro_report.z_integral = gyro_val_integrated(2);

	// TODO: get these right
	gyro_report.scaling = -1.0f;
	gyro_report.range_rad_s = -1.0f;

	gyro_report.device_id = m_id.dev_id;

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked) && should_publish_gyro) {

		if (_gyro_topic != nullptr) {
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &gyro_report);
		}

		should_notify = true;
	}

	perf_end(_gyro_sample_perf);

	if (should_notify) {
		/* Notify anyone waiting for data. */
		DevMgr::updateNotify(*this);
	}

	// TODO: check the return codes of this function
	return 0;
};


namespace df_mpu9250_wrapper
{

DfMpu9250Wrapper *g_dev = nullptr;

int start(/* enum Rotation rotation */);
int stop();
int info();
void usage();

int start(/*enum Rotation rotation*/)
{
	g_dev = new DfMpu9250Wrapper(/*rotation*/);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfMpu9250Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfMpu9250Wrapper start failed");
		return ret;
	}

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(IMU_DEVICE_PATH, h);

	if (!h.isValid()) {
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
	PX4_WARN("Usage: df_mpu9250_wrapper 'start', 'info', 'stop'");
	PX4_WARN("options:");
	//PX4_WARN("    -R rotation");
}

} // namespace df_mpu9250_wrapper


int
df_mpu9250_wrapper_main(int argc, char *argv[])
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
			df_mpu9250_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_mpu9250_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_mpu9250_wrapper::start(/*rotation*/);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_mpu9250_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_mpu9250_wrapper::info();
	}

	else {
		df_mpu9250_wrapper::usage();
		return 1;
	}

	return ret;
}

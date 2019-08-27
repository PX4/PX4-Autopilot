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
 * @file df_hmc5883_wrapper.cpp
 * Lightweight driver to access the HMC5883 of the DriverFramework.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_platform_common/config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_platform_common/getopt.h>
#include <errno.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/parameter_update.h>

#include <board_config.h>

#include <lib/conversion/rotation.h>

#include <hmc5883/HMC5883.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_hmc5883_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfHmc5883Wrapper : public HMC5883
{
public:
	DfHmc5883Wrapper(enum Rotation rotation, const char *path);
	~DfHmc5883Wrapper();


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
	int _publish(struct mag_sensor_data &data);

	void _update_mag_calibration();

	orb_advert_t		_mag_topic;

	int			_param_update_sub;

	struct mag_calibration_s {
		float x_offset;
		float x_scale;
		float y_offset;
		float y_scale;
		float z_offset;
		float z_scale;
	} _mag_calibration;

	matrix::Dcmf      _rotation_matrix;

	int			_mag_orb_class_instance;

	perf_counter_t		_mag_sample_perf;

};

DfHmc5883Wrapper::DfHmc5883Wrapper(enum Rotation rotation, const char *path) :
	HMC5883(path),
	_mag_topic(nullptr),
	_param_update_sub(-1),
	_mag_calibration{},
	_mag_orb_class_instance(-1),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "df_mag_read"))
{
	// Set sane default calibration values
	_mag_calibration.x_scale = 1.0f;
	_mag_calibration.y_scale = 1.0f;
	_mag_calibration.z_scale = 1.0f;
	_mag_calibration.x_offset = 0.0f;
	_mag_calibration.y_offset = 0.0f;
	_mag_calibration.z_offset = 0.0f;

	// Get sensor rotation matrix
	_rotation_matrix = get_rot_matrix(rotation);
}

DfHmc5883Wrapper::~DfHmc5883Wrapper()
{
	perf_free(_mag_sample_perf);
}

int DfHmc5883Wrapper::start()
{
	/* Subscribe to param update topic. */
	if (_param_update_sub < 0) {
		_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	}

	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("HMC5883 init fail: %d", ret);
		return ret;
	}

	ret = HMC5883::start();

	if (ret != 0) {
		PX4_ERR("HMC5883 start fail: %d", ret);
		return ret;
	}

	/* Force getting the calibration values. */
	_update_mag_calibration();

	return 0;
}

int DfHmc5883Wrapper::stop()
{
	/* Stop sensor. */
	int ret = HMC5883::stop();

	if (ret != 0) {
		PX4_ERR("HMC5883 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

void DfHmc5883Wrapper::_update_mag_calibration()
{
	// TODO: replace magic number
	for (unsigned i = 0; i < 3; ++i) {

		// TODO: remove printfs and add error counter

		char str[30];
		(void)sprintf(str, "CAL_MAG%u_ID", i);
		int32_t device_id;
		int res = param_get(param_find(str), &device_id);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
			continue;
		}

		if ((uint32_t)device_id != m_id.dev_id) {
			continue;
		}

		(void)sprintf(str, "CAL_MAG%u_XSCALE", i);
		res = param_get(param_find(str), &_mag_calibration.x_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_YSCALE", i);
		res = param_get(param_find(str), &_mag_calibration.y_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_ZSCALE", i);
		res = param_get(param_find(str), &_mag_calibration.z_scale);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_XOFF", i);
		res = param_get(param_find(str), &_mag_calibration.x_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_YOFF", i);
		res = param_get(param_find(str), &_mag_calibration.y_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_ZOFF", i);
		res = param_get(param_find(str), &_mag_calibration.z_offset);

		if (res != OK) {
			PX4_ERR("Could not access param %s", str);
		}
	}
}


int DfHmc5883Wrapper::_publish(struct mag_sensor_data &data)
{
	/* Check if calibration values are still up-to-date. */
	bool updated;
	orb_check(_param_update_sub, &updated);

	if (updated) {
		parameter_update_s parameter_update;
		orb_copy(ORB_ID(parameter_update), _param_update_sub, &parameter_update);

		_update_mag_calibration();
	}

	/* Publish mag first. */
	perf_begin(_mag_sample_perf);

	mag_report mag_report = {};
	mag_report.timestamp = hrt_absolute_time();
	mag_report.is_external = true;

	/* The standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y. */
	const float tmp = data.field_x_ga;
	data.field_x_ga = -data.field_y_ga;
	data.field_y_ga = tmp;

	// TODO: remove these (or get the values)
	mag_report.x_raw = 0;
	mag_report.y_raw = 0;
	mag_report.z_raw = 0;

	matrix::Vector3f mag_val(data.field_x_ga,
				 data.field_y_ga,
				 data.field_z_ga);

	// apply sensor rotation on the accel measurement
	mag_val = _rotation_matrix * mag_val;

	// Apply calibration after rotation.
	mag_report.x = (mag_val(0) - _mag_calibration.x_offset) * _mag_calibration.x_scale;
	mag_report.y = (mag_val(1) - _mag_calibration.y_offset) * _mag_calibration.y_scale;
	mag_report.z = (mag_val(2) - _mag_calibration.z_offset) * _mag_calibration.z_scale;

	// TODO: get these right
	//mag_report.scaling = -1.0f;

	mag_report.device_id = m_id.dev_id;

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked)) {

		if (_mag_topic == nullptr) {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mag_report,
							 &_mag_orb_class_instance, ORB_PRIO_HIGH);

		} else {
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &mag_report);
		}

	}

	perf_end(_mag_sample_perf);

	return 0;
};


namespace df_hmc5883_wrapper
{

DfHmc5883Wrapper *g_dev = nullptr;

int start(enum Rotation rotation, const char *path);
int stop();
int info();
void usage();

int start(enum Rotation rotation, const char *path)
{
	g_dev = new DfHmc5883Wrapper(rotation, path);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfHmc5883Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfHmc5883Wrapper start failed");
		return ret;
	}

	// Open the MAG sensor
	DevHandle h;
	DevMgr::getHandle(path, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    path, h.getError());
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
	PX4_INFO("Usage: df_hmc5883_wrapper 'start', 'info', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace df_hmc5883_wrapper


int
df_hmc5883_wrapper_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;
	const char *device_path = MAG_DEVICE_PATH;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:D:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'D':
			device_path = myoptarg;
			break;

		default:
			df_hmc5883_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_hmc5883_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_hmc5883_wrapper::start(rotation, device_path);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_hmc5883_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_hmc5883_wrapper::info();
	}

	else {
		df_hmc5883_wrapper::usage();
		return 1;
	}

	return ret;
}

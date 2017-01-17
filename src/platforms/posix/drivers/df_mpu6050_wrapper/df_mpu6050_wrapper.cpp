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
 * @file df_mpu6050_wrapper.cpp
 * Lightweight driver to access the MPU6050 of the DriverFramework.
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

#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/integrator.h>

#include <lib/conversion/rotation.h>

#include <uORB/topics/parameter_update.h>

#include <mpu6050/MPU6050.hpp>
#include <DevMgr.hpp>

// We don't want to auto publish, therefore set this to 0.
#define MPU6050_NEVER_AUTOPUBLISH_US 0


extern "C" { __EXPORT int df_mpu6050_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfMPU6050Wrapper : public MPU6050
{
public:
	DfMPU6050Wrapper(enum Rotation rotation);
	~DfMPU6050Wrapper();


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

	/**
	 * Print some debug info.
	 */
	void		info();

private:
	int _publish(struct imu_sensor_data &data);

	void _update_accel_calibration();
	void _update_gyro_calibration();

	orb_advert_t		    _accel_topic;
	orb_advert_t		    _gyro_topic;

	orb_advert_t		    _mavlink_log_pub;

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

	math::Matrix<3, 3>	    _rotation_matrix;

	int			    _accel_orb_class_instance;
	int			    _gyro_orb_class_instance;

	Integrator		    _accel_int;
	Integrator		    _gyro_int;

	unsigned		    _publish_count;

	perf_counter_t		    _read_counter;
	perf_counter_t		    _error_counter;
	perf_counter_t		    _fifo_overflow_counter;
	perf_counter_t		    _fifo_corruption_counter;
	perf_counter_t		    _gyro_range_hit_counter;
	perf_counter_t		    _accel_range_hit_counter;
	perf_counter_t		    _publish_perf;

	hrt_abstime		    _last_accel_range_hit_time;
	uint64_t		    _last_accel_range_hit_count;

};

DfMPU6050Wrapper::DfMPU6050Wrapper(enum Rotation rotation) :
	MPU6050(IMU_DEVICE_PATH),
	_accel_topic(nullptr),
	_gyro_topic(nullptr),
	_mavlink_log_pub(nullptr),
	_param_update_sub(-1),
	_accel_calibration{},
	_gyro_calibration{},
	_accel_orb_class_instance(-1),
	_gyro_orb_class_instance(-1),
	_accel_int(MPU6050_NEVER_AUTOPUBLISH_US, false),
	_gyro_int(MPU6050_NEVER_AUTOPUBLISH_US, true),
	_publish_count(0),
	_read_counter(perf_alloc(PC_COUNT, "mpu6050_reads")),
	_error_counter(perf_alloc(PC_COUNT, "mpu6050_errors")),
	_fifo_overflow_counter(perf_alloc(PC_COUNT, "mpu6050_fifo_overflows")),
	_fifo_corruption_counter(perf_alloc(PC_COUNT, "mpu6050_fifo_corruptions")),
	_gyro_range_hit_counter(perf_alloc(PC_COUNT, "mpu6050_gyro_range_hits")),
	_accel_range_hit_counter(perf_alloc(PC_COUNT, "mpu6050_accel_range_hits")),
	_publish_perf(perf_alloc(PC_ELAPSED, "mpu6050_publish")),
	_last_accel_range_hit_time(0),
	_last_accel_range_hit_count(0)
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

	// Get sensor rotation matrix
	get_rot_matrix(rotation, &_rotation_matrix);
}

DfMPU6050Wrapper::~DfMPU6050Wrapper()
{
	perf_free(_read_counter);
	perf_free(_error_counter);
	perf_free(_fifo_overflow_counter);
	perf_free(_fifo_corruption_counter);
	perf_free(_gyro_range_hit_counter);
	perf_free(_accel_range_hit_counter);

	perf_free(_publish_perf);
}

int DfMPU6050Wrapper::start()
{
	/* Subscribe to param update topic. */
	if (_param_update_sub < 0) {
		_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	}

	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("MPU6050 init fail: %d", ret);
		return ret;
	}

	ret = MPU6050::start();

	if (ret != 0) {
		PX4_ERR("MPU6050 start fail: %d", ret);
		return ret;
	}

	PX4_DEBUG("MPU6050 device id is: %d", m_id.dev_id);

	/* Force getting the calibration values. */
	_update_accel_calibration();
	_update_gyro_calibration();

	return 0;
}

int DfMPU6050Wrapper::stop()
{
	/* Stop sensor. */
	int ret = MPU6050::stop();

	if (ret != 0) {
		PX4_ERR("MPU6050 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

void DfMPU6050Wrapper::info()
{
	perf_print_counter(_read_counter);
	perf_print_counter(_error_counter);
	perf_print_counter(_fifo_overflow_counter);
	perf_print_counter(_fifo_corruption_counter);
	perf_print_counter(_gyro_range_hit_counter);
	perf_print_counter(_accel_range_hit_counter);

	perf_print_counter(_publish_perf);
}

void DfMPU6050Wrapper::_update_gyro_calibration()
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

void DfMPU6050Wrapper::_update_accel_calibration()
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

int DfMPU6050Wrapper::_publish(struct imu_sensor_data &data)
{
	/* Check if calibration values are still up-to-date. */
	bool updated;
	orb_check(_param_update_sub, &updated);

	if (updated) {
		parameter_update_s parameter_update;
		orb_copy(ORB_ID(parameter_update), _param_update_sub, &parameter_update);

		_update_accel_calibration();
		_update_gyro_calibration();
	}

	uint64_t now = hrt_absolute_time();

	math::Vector<3> vec_integrated_unused;
	uint64_t integral_dt_unused;

	math::Vector<3> accel_val(data.accel_m_s2_x, data.accel_m_s2_y, data.accel_m_s2_z);

	// apply sensor rotation on the accel measurement
	accel_val = _rotation_matrix * accel_val;

	accel_val(0) = (accel_val(0) - _accel_calibration.x_offset) * _accel_calibration.x_scale;
	accel_val(1) = (accel_val(1) - _accel_calibration.y_offset) * _accel_calibration.y_scale;
	accel_val(2) = (accel_val(2) - _accel_calibration.z_offset) * _accel_calibration.z_scale;

	_accel_int.put(now,
		       accel_val,
		       vec_integrated_unused,
		       integral_dt_unused);

	math::Vector<3> gyro_val(data.gyro_rad_s_x, data.gyro_rad_s_y, data.gyro_rad_s_z);

	// apply sensor rotation on the gyro measurement
	gyro_val = _rotation_matrix * gyro_val;

	gyro_val(0) = (gyro_val(0) - _gyro_calibration.x_offset) * _gyro_calibration.x_scale;
	gyro_val(1) = (gyro_val(1) - _gyro_calibration.y_offset) * _gyro_calibration.y_scale;
	gyro_val(2) = (gyro_val(2) - _gyro_calibration.z_offset) * _gyro_calibration.z_scale;

	_gyro_int.put(now,
		      gyro_val,
		      vec_integrated_unused,
		      integral_dt_unused);

	// If we are not receiving the last sample from the FIFO buffer yet, let's stop here
	// and wait for more packets.
	if (!data.is_last_fifo_sample) {
		return 0;
	}

	// The driver empties the FIFO buffer at 1kHz, however we only need to publish at 250Hz.
	// Therefore, only publish every forth time.
	++_publish_count;

	if (_publish_count < 4) {
		return 0;
	}

	_publish_count = 0;

	// Update all the counters.
	perf_set_count(_read_counter, data.read_counter);
	perf_set_count(_error_counter, data.error_counter);
	perf_set_count(_fifo_overflow_counter, data.fifo_overflow_counter);
	perf_set_count(_fifo_corruption_counter, data.fifo_overflow_counter);
	perf_set_count(_gyro_range_hit_counter, data.gyro_range_hit_counter);
	perf_set_count(_accel_range_hit_counter, data.accel_range_hit_counter);

	perf_begin(_publish_perf);

	accel_report accel_report = {};
	gyro_report gyro_report = {};

	accel_report.timestamp = gyro_report.timestamp = hrt_absolute_time();

	// TODO: get these right
	gyro_report.scaling = -1.0f;
	gyro_report.range_rad_s = -1.0f;
	gyro_report.device_id = m_id.dev_id;

	accel_report.scaling = -1.0f;
	accel_report.range_m_s2 = -1.0f;
	accel_report.device_id = m_id.dev_id;

	// TODO: remove these (or get the values)
	gyro_report.x_raw = 0;
	gyro_report.y_raw = 0;
	gyro_report.z_raw = 0;

	accel_report.x_raw = 0;
	accel_report.y_raw = 0;
	accel_report.z_raw = 0;

	math::Vector<3> gyro_val_filt;
	math::Vector<3> accel_val_filt;

	// Read and reset.
	math::Vector<3> gyro_val_integ = _gyro_int.get_and_filtered(true, gyro_report.integral_dt, gyro_val_filt);
	math::Vector<3> accel_val_integ = _accel_int.get_and_filtered(true, accel_report.integral_dt, accel_val_filt);

	// Use the filtered (by integration) values to get smoother / less noisy data.
	gyro_report.x = gyro_val_filt(0);
	gyro_report.y = gyro_val_filt(1);
	gyro_report.z = gyro_val_filt(2);

	accel_report.x = accel_val_filt(0);
	accel_report.y = accel_val_filt(1);
	accel_report.z = accel_val_filt(2);

	gyro_report.x_integral = gyro_val_integ(0);
	gyro_report.y_integral = gyro_val_integ(1);
	gyro_report.z_integral = gyro_val_integ(2);

	accel_report.x_integral = accel_val_integ(0);
	accel_report.y_integral = accel_val_integ(1);
	accel_report.z_integral = accel_val_integ(2);

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked)) {

		if (_gyro_topic == nullptr) {
			_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &gyro_report,
							  &_gyro_orb_class_instance, ORB_PRIO_DEFAULT);

		} else {
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &gyro_report);
		}

		if (_accel_topic == nullptr) {
			_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &accel_report,
							   &_accel_orb_class_instance, ORB_PRIO_DEFAULT);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		}

		/* Notify anyone waiting for data. */
		DevMgr::updateNotify(*this);

		// Report if there are high vibrations, every 10 times it happens.
		const bool threshold_reached = (data.accel_range_hit_counter - _last_accel_range_hit_count > 10);

		// Report every 5s.
		const bool due_to_report = (hrt_elapsed_time(&_last_accel_range_hit_time) > 5000000);

		if (threshold_reached && due_to_report) {
			mavlink_log_critical(&_mavlink_log_pub,
					     "High accelerations, range exceeded %llu times",
					     data.accel_range_hit_counter);

			_last_accel_range_hit_time = hrt_absolute_time();
			_last_accel_range_hit_count = data.accel_range_hit_counter;
		}
	}

	perf_end(_publish_perf);

	// TODO: check the return codes of this function
	return 0;
};


namespace df_mpu6050_wrapper
{

DfMPU6050Wrapper *g_dev = nullptr;

int start(enum Rotation rotation);
int stop();
int info();
void usage();

int start(enum Rotation rotation)
{
	g_dev = new DfMPU6050Wrapper(rotation);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfMPU6050Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfMPU6050Wrapper start failed");
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

	PX4_INFO("state @ %p", g_dev);
	g_dev->info();

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: df_mpu6050_wrapper 'start', 'info', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace df_mpu6050_wrapper


int
df_mpu6050_wrapper_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			df_mpu6050_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_mpu6050_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ret = df_mpu6050_wrapper::start(rotation);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_mpu6050_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_mpu6050_wrapper::info();
	}

	else {
		df_mpu6050_wrapper::usage();
		return 1;
	}

	return ret;
}

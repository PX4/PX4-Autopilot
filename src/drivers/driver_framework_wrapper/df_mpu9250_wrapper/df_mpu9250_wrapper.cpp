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

#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/device/integrator.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <lib/conversion/rotation.h>

#include <uORB/topics/parameter_update.h>

#include <mpu9250/MPU9250.hpp>
#include <DevMgr.hpp>

#define MPU9250_ACCEL_DEFAULT_RATE 1000
#define MPU9250_GYRO_DEFAULT_RATE 1000

#define MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define MPU9250_PUB_RATE 280


extern "C" { __EXPORT int df_mpu9250_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfMpu9250Wrapper : public MPU9250
{
public:
	DfMpu9250Wrapper(bool mag_enabled, enum Rotation rotation);
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

	/**
	 * Print some debug info.
	 */
	void		info();

private:
	int _publish(struct imu_sensor_data &data);

	void _update_accel_calibration();
	void _update_gyro_calibration();
	void _update_mag_calibration();

	orb_advert_t		    _accel_topic;
	orb_advert_t		    _gyro_topic;
	orb_advert_t		    _mag_topic;

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

	struct mag_calibration_s {
		float x_offset;
		float x_scale;
		float y_offset;
		float y_scale;
		float z_offset;
		float z_scale;
	} _mag_calibration;

	int			    _accel_orb_class_instance;
	int			    _gyro_orb_class_instance;
	int			    _mag_orb_class_instance;

	Integrator		    _accel_int;
	Integrator		    _gyro_int;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	perf_counter_t		    _read_counter;
	perf_counter_t		    _error_counter;
	perf_counter_t		    _fifo_overflow_counter;
	perf_counter_t		    _fifo_corruption_counter;
	perf_counter_t		    _gyro_range_hit_counter;
	perf_counter_t		    _accel_range_hit_counter;
	perf_counter_t		    _mag_fifo_overflow_counter;
	perf_counter_t		    _publish_perf;

	hrt_abstime		    _last_accel_range_hit_time;
	uint64_t		    _last_accel_range_hit_count;

	bool _mag_enabled;

	enum Rotation _rotation;
};

DfMpu9250Wrapper::DfMpu9250Wrapper(bool mag_enabled, enum Rotation rotation) :
	MPU9250(IMU_DEVICE_PATH, mag_enabled),
	_accel_topic(nullptr),
	_gyro_topic(nullptr),
	_mag_topic(nullptr),
	_mavlink_log_pub(nullptr),
	_param_update_sub(-1),
	_accel_calibration{},
	_gyro_calibration{},
	_mag_calibration{},
	_accel_orb_class_instance(-1),
	_gyro_orb_class_instance(-1),
	_mag_orb_class_instance(-1),
	_accel_int(1000000 / MPU9250_PUB_RATE, false),
	_gyro_int(1000000 / MPU9250_PUB_RATE, true),
	_accel_filter_x(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_read_counter(perf_alloc(PC_COUNT, "mpu9250_reads")),
	_error_counter(perf_alloc(PC_COUNT, "mpu9250_errors")),
	_fifo_overflow_counter(perf_alloc(PC_COUNT, "mpu9250_fifo_overflows")),
	_fifo_corruption_counter(perf_alloc(PC_COUNT, "mpu9250_fifo_corruptions")),
	_gyro_range_hit_counter(perf_alloc(PC_COUNT, "mpu9250_gyro_range_hits")),
	_accel_range_hit_counter(perf_alloc(PC_COUNT, "mpu9250_accel_range_hits")),
	_mag_fifo_overflow_counter(perf_alloc(PC_COUNT, "mpu9250_mag_fifo_overflows")),
	_publish_perf(perf_alloc(PC_ELAPSED, "mpu9250_publish")),
	_last_accel_range_hit_time(0),
	_last_accel_range_hit_count(0),
	_mag_enabled(mag_enabled),
	_rotation(rotation)
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

	if (_mag_enabled) {
		_mag_calibration.x_scale = 1.0f;
		_mag_calibration.y_scale = 1.0f;
		_mag_calibration.z_scale = 1.0f;
		_mag_calibration.x_offset = 0.0f;
		_mag_calibration.y_offset = 0.0f;
		_mag_calibration.z_offset = 0.0f;
	}

	// set software low pass filter for controllers
	param_t param_handle = param_find("IMU_ACCEL_CUTOFF");
	float param_val = MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ;

	if (param_handle != PARAM_INVALID && (param_get(param_handle, &param_val) == PX4_OK)) {
		_accel_filter_x.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, param_val);
		_accel_filter_y.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, param_val);
		_accel_filter_z.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, param_val);

	} else {
		PX4_ERR("IMU_ACCEL_CUTOFF param invalid");
	}

	param_handle = param_find("IMU_GYRO_CUTOFF");
	param_val = MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ;

	if (param_handle != PARAM_INVALID && (param_get(param_handle, &param_val) == PX4_OK)) {
		_gyro_filter_x.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, param_val);
		_gyro_filter_y.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, param_val);
		_gyro_filter_z.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, param_val);

	} else {
		PX4_ERR("IMU_GYRO_CUTOFF param invalid");
	}
}

DfMpu9250Wrapper::~DfMpu9250Wrapper()
{
	perf_free(_read_counter);
	perf_free(_error_counter);
	perf_free(_fifo_overflow_counter);
	perf_free(_fifo_corruption_counter);
	perf_free(_gyro_range_hit_counter);
	perf_free(_accel_range_hit_counter);

	if (_mag_enabled) {
		perf_free(_mag_fifo_overflow_counter);
	}

	perf_free(_publish_perf);
}

int DfMpu9250Wrapper::start()
{
	// TODO: don't publish garbage here
	sensor_accel_s accel_report = {};
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &accel_report,
					   &_accel_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_accel_topic == nullptr) {
		PX4_ERR("sensor_accel advert fail");
		return -1;
	}

	// TODO: don't publish garbage here
	sensor_gyro_s gyro_report = {};
	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &gyro_report,
					  &_gyro_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_gyro_topic == nullptr) {
		PX4_ERR("sensor_gyro advert fail");
		return -1;
	}

	if (_mag_enabled) {
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

	PX4_DEBUG("MPU9250 device id is: %d", m_id.dev_id);

	/* Force getting the calibration values. */
	_update_accel_calibration();
	_update_gyro_calibration();
	_update_mag_calibration();

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

void DfMpu9250Wrapper::info()
{
	perf_print_counter(_read_counter);
	perf_print_counter(_error_counter);
	perf_print_counter(_fifo_overflow_counter);
	perf_print_counter(_fifo_corruption_counter);
	perf_print_counter(_gyro_range_hit_counter);
	perf_print_counter(_accel_range_hit_counter);

	if (_mag_enabled) {
		perf_print_counter(_mag_fifo_overflow_counter);
	}

	perf_print_counter(_publish_perf);
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

void DfMpu9250Wrapper::_update_mag_calibration()
{
	if (!_mag_enabled) {
		return;
	}

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

		// We got calibration values, let's exit.
		return;
	}

	// Set sane default calibration values
	_mag_calibration.x_scale = 1.0f;
	_mag_calibration.y_scale = 1.0f;
	_mag_calibration.z_scale = 1.0f;
	_mag_calibration.x_offset = 0.0f;
	_mag_calibration.y_offset = 0.0f;
	_mag_calibration.z_offset = 0.0f;
}

int DfMpu9250Wrapper::_publish(struct imu_sensor_data &data)
{
	/* Check if calibration values are still up-to-date. */
	bool updated;
	orb_check(_param_update_sub, &updated);

	if (updated) {
		parameter_update_s parameter_update;
		orb_copy(ORB_ID(parameter_update), _param_update_sub, &parameter_update);

		_update_accel_calibration();
		_update_gyro_calibration();
		_update_mag_calibration();
	}

	sensor_accel_s accel_report = {};
	sensor_gyro_s gyro_report = {};
	mag_report mag_report = {};

	accel_report.timestamp = gyro_report.timestamp = hrt_absolute_time();

	accel_report.error_count = gyro_report.error_count = mag_report.error_count = data.error_counter;

	// ACCEL

	float xraw_f = data.accel_m_s2_x;
	float yraw_f = data.accel_m_s2_y;
	float zraw_f = data.accel_m_s2_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	// MPU9250 driver from DriverFramework does not provide any raw values
	// TEMP We misuse the raw values on the Snapdragon to publish unfiltered data for VISLAM
	accel_report.x_raw = (int16_t)(xraw_f * 1000); // (int16) [m / s^2 * 1000];
	accel_report.y_raw = (int16_t)(yraw_f * 1000); // (int16) [m / s^2 * 1000];
	accel_report.z_raw = (int16_t)(zraw_f * 1000); // (int16) [m / s^2 * 1000];

	// adjust values according to the calibration
	float x_in_new = (xraw_f - _accel_calibration.x_offset) * _accel_calibration.x_scale;
	float y_in_new = (yraw_f - _accel_calibration.y_offset) * _accel_calibration.y_scale;
	float z_in_new = (zraw_f - _accel_calibration.z_offset) * _accel_calibration.z_scale;

	accel_report.x = _accel_filter_x.apply(x_in_new);
	accel_report.y = _accel_filter_y.apply(y_in_new);
	accel_report.z = _accel_filter_z.apply(z_in_new);

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	_accel_int.put(accel_report.timestamp, aval, aval_integrated, accel_report.integral_dt);
	accel_report.x_integral = aval_integrated(0);
	accel_report.y_integral = aval_integrated(1);
	accel_report.z_integral = aval_integrated(2);

	// GYRO

	xraw_f = data.gyro_rad_s_x;
	yraw_f = data.gyro_rad_s_y;
	zraw_f = data.gyro_rad_s_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	// MPU9250 driver from DriverFramework does not provide any raw values
	// TEMP We misuse the raw values on the Snapdragon to publish unfiltered data for VISLAM
	gyro_report.x_raw = (int16_t)(xraw_f * 1000); // (int16) [rad / s * 1000];
	gyro_report.y_raw = (int16_t)(yraw_f * 1000); // (int16) [rad / s * 1000];
	gyro_report.z_raw = (int16_t)(zraw_f * 1000); // (int16) [rad / s * 1000];

	// adjust values according to the calibration
	float x_gyro_in_new = (xraw_f - _gyro_calibration.x_offset) * _gyro_calibration.x_scale;
	float y_gyro_in_new = (yraw_f - _gyro_calibration.y_offset) * _gyro_calibration.y_scale;
	float z_gyro_in_new = (zraw_f - _gyro_calibration.z_offset) * _gyro_calibration.z_scale;

	gyro_report.x = _gyro_filter_x.apply(x_gyro_in_new);
	gyro_report.y = _gyro_filter_y.apply(y_gyro_in_new);
	gyro_report.z = _gyro_filter_z.apply(z_gyro_in_new);

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool sensor_notify = _gyro_int.put(gyro_report.timestamp, gval, gval_integrated, gyro_report.integral_dt);
	gyro_report.x_integral = gval_integrated(0);
	gyro_report.y_integral = gval_integrated(1);
	gyro_report.z_integral = gval_integrated(2);


	// if gyro integrator did not return a sample we can return here
	// Note: the accel integrator receives the same timestamp as the gyro integrator
	// so we do not need to handle it seperately
	if (!sensor_notify) {
		return 0;
	}

	// Update all the counters.
	perf_set_count(_read_counter, data.read_counter);
	perf_set_count(_error_counter, data.error_counter);
	perf_set_count(_fifo_overflow_counter, data.fifo_overflow_counter);
	perf_set_count(_fifo_corruption_counter, data.fifo_overflow_counter);
	perf_set_count(_gyro_range_hit_counter, data.gyro_range_hit_counter);
	perf_set_count(_accel_range_hit_counter, data.accel_range_hit_counter);

	if (_mag_enabled) {
		perf_set_count(_mag_fifo_overflow_counter, data.mag_fifo_overflow_counter);
	}

	perf_begin(_publish_perf);

	// TODO: get these right
	gyro_report.scaling = -1.0f;
	gyro_report.device_id = m_id.dev_id;

	accel_report.scaling = -1.0f;
	accel_report.device_id = m_id.dev_id;

	if (_mag_enabled) {
		mag_report.timestamp = accel_report.timestamp;
		mag_report.is_external = false;

		mag_report.scaling = -1.0f;
		mag_report.device_id = m_id.dev_id;

		xraw_f = data.mag_ga_x;
		yraw_f = data.mag_ga_y;
		zraw_f = data.mag_ga_z;

		rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

		// MPU9250 driver from DriverFramework does not provide any raw values
		// TEMP We misuse the raw values on the Snapdragon to publish unfiltered data for VISLAM
		mag_report.x_raw = xraw_f * 1000; // (int16) [Gs * 1000]
		mag_report.y_raw = yraw_f * 1000; // (int16) [Gs * 1000]
		mag_report.z_raw = zraw_f * 1000; // (int16) [Gs * 1000]

		mag_report.x = (xraw_f - _mag_calibration.x_offset) * _mag_calibration.x_scale;
		mag_report.y = (yraw_f - _mag_calibration.y_offset) * _mag_calibration.y_scale;
		mag_report.z = (zraw_f - _mag_calibration.z_offset) * _mag_calibration.z_scale;
	}

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked) && sensor_notify) {

		if (_gyro_topic != nullptr) {
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &gyro_report);
		}

		if (_accel_topic != nullptr) {
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		}

		if (_mag_enabled) {

			if (_mag_topic == nullptr) {
				_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mag_report,
								 &_mag_orb_class_instance, ORB_PRIO_LOW);

			} else {
				orb_publish(ORB_ID(sensor_mag), _mag_topic, &mag_report);
			}
		}

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


namespace df_mpu9250_wrapper
{

DfMpu9250Wrapper *g_dev = nullptr;

int start(bool mag_enabled, enum Rotation rotation);
int stop();
int info();
void usage();

int start(bool mag_enabled, enum Rotation rotation)
{
	g_dev = new DfMpu9250Wrapper(mag_enabled, rotation);

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

	PX4_INFO("state @ %p", g_dev);
	g_dev->info();

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: df_mpu9250_wrapper 'start', 'start_without_mag', 'info', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace df_mpu9250_wrapper


int
df_mpu9250_wrapper_main(int argc, char *argv[])
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
			df_mpu9250_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_mpu9250_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start_without_mag")) {
		ret = df_mpu9250_wrapper::start(false, rotation);
	}

	else if (!strcmp(verb, "start")) {
		ret = df_mpu9250_wrapper::start(true, rotation);
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

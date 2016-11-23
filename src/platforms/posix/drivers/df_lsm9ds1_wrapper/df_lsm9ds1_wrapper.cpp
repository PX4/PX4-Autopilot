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
 * @file df_lsm9ds1_wrapper.cpp
 * Lightweight driver to access the MPU9250 of the DriverFramework.
 *
 * @author Miguel Arroyo <miguel@arroyo.me>
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
#include <drivers/drv_mag.h>
#include <drivers/device/integrator.h>

#include <lib/conversion/rotation.h>

#include <uORB/topics/parameter_update.h>

#include <lsm9ds1/LSM9DS1.hpp>
#include <DevMgr.hpp>

// We don't want to auto publish, therefore set this to 0.
#define LSM9DS1_NEVER_AUTOPUBLISH_US 0


extern "C" { __EXPORT int df_lsm9ds1_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfLsm9ds1Wrapper : public LSM9DS1
{
public:
	DfLsm9ds1Wrapper(bool mag_enabled, enum Rotation rotation);
	~DfLsm9ds1Wrapper();


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
	orb_advert_t        	    _mag_topic;
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

	math::Matrix<3, 3>	    _rotation_matrix;
	int			    _accel_orb_class_instance;
	int			    _gyro_orb_class_instance;
	int         _mag_orb_class_instance;

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

	bool _mag_enabled;
};

DfLsm9ds1Wrapper::DfLsm9ds1Wrapper(bool mag_enabled, enum Rotation rotation) :
	LSM9DS1(IMU_DEVICE_ACC_GYRO, IMU_DEVICE_MAG),
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
	_accel_int(LSM9DS1_NEVER_AUTOPUBLISH_US, false),
	_gyro_int(LSM9DS1_NEVER_AUTOPUBLISH_US, true),
	_publish_count(0),
	_read_counter(perf_alloc(PC_COUNT, "lsm9ds1_reads")),
	_error_counter(perf_alloc(PC_COUNT, "lsm9ds1_errors")),
	_fifo_overflow_counter(perf_alloc(PC_COUNT, "lsm9ds1_fifo_overflows")),
	_fifo_corruption_counter(perf_alloc(PC_COUNT, "lsm9ds1_fifo_corruptions")),
	_gyro_range_hit_counter(perf_alloc(PC_COUNT, "lsm9ds1_gyro_range_hits")),
	_accel_range_hit_counter(perf_alloc(PC_COUNT, "lsm9ds1_accel_range_hits")),
	_publish_perf(perf_alloc(PC_ELAPSED, "lsm9ds1_publish")),
	_last_accel_range_hit_time(0),
	_last_accel_range_hit_count(0),
	_mag_enabled(mag_enabled)
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

	// Get sensor rotation matrix
	get_rot_matrix(rotation, &_rotation_matrix);
}

DfLsm9ds1Wrapper::~DfLsm9ds1Wrapper()
{
	perf_free(_read_counter);
	perf_free(_error_counter);
	perf_free(_fifo_overflow_counter);
	perf_free(_fifo_corruption_counter);
	perf_free(_gyro_range_hit_counter);
	perf_free(_accel_range_hit_counter);

	perf_free(_publish_perf);
}

int DfLsm9ds1Wrapper::start()
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

	if (_mag_enabled) {
		mag_report mag_report = {};
		_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mag_report,
						 &_mag_orb_class_instance, ORB_PRIO_DEFAULT);

		if (_mag_topic == nullptr) {
			PX4_ERR("sensor_mag advert fail");
			return -1;
		}
	}

	/* Subscribe to param update topic. */
	if (_param_update_sub < 0) {
		_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	}

	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("LSM9DS1 init fail: %d", ret);
		return ret;
	}

	ret = LSM9DS1::start();

	if (ret != 0) {
		PX4_ERR("LSM9DS1 start fail: %d", ret);
		return ret;
	}

	PX4_DEBUG("LSM9DS1 device id is: %d", m_id.dev_id);

	/* Force getting the calibration values. */
	_update_accel_calibration();
	_update_gyro_calibration();
	_update_mag_calibration();

	return 0;
}

int DfLsm9ds1Wrapper::stop()
{
	/* Stop sensor. */
	int ret = LSM9DS1::stop();

	if (ret != 0) {
		PX4_ERR("LSM9DS1 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

void DfLsm9ds1Wrapper::info()
{
	perf_print_counter(_read_counter);
	perf_print_counter(_error_counter);
	perf_print_counter(_fifo_overflow_counter);
	perf_print_counter(_fifo_corruption_counter);
	perf_print_counter(_gyro_range_hit_counter);
	perf_print_counter(_accel_range_hit_counter);

	perf_print_counter(_publish_perf);
}

void DfLsm9ds1Wrapper::_update_gyro_calibration()
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

void DfLsm9ds1Wrapper::_update_accel_calibration()
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

void DfLsm9ds1Wrapper::_update_mag_calibration()
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

int DfLsm9ds1Wrapper::_publish(struct imu_sensor_data &data)
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

	math::Vector<3> vec_integrated_unused;
	uint64_t integral_dt_unused;
	math::Vector<3> accel_val(data.accel_m_s2_x,
				  data.accel_m_s2_y,
				  data.accel_m_s2_z);
	// apply sensor rotation on the accel measurement
	accel_val = _rotation_matrix * accel_val;
	// Apply calibration after rotation
	accel_val(0) = (accel_val(0) - _accel_calibration.x_offset) * _accel_calibration.x_scale;
	accel_val(1) = (accel_val(1) - _accel_calibration.y_offset) * _accel_calibration.y_scale;
	accel_val(2) = (accel_val(2) - _accel_calibration.z_offset) * _accel_calibration.z_scale;
	_accel_int.put_with_interval(data.fifo_sample_interval_us,
				     accel_val,
				     vec_integrated_unused,
				     integral_dt_unused);
	math::Vector<3> gyro_val(data.gyro_rad_s_x,
				 data.gyro_rad_s_y,
				 data.gyro_rad_s_z);
	// apply sensor rotation on the gyro measurement
	gyro_val = _rotation_matrix * gyro_val;
	// Apply calibration after rotation
	gyro_val(0) = (gyro_val(0) - _gyro_calibration.x_offset) * _gyro_calibration.x_scale;
	gyro_val(1) = (gyro_val(1) - _gyro_calibration.y_offset) * _gyro_calibration.y_scale;
	gyro_val(2) = (gyro_val(2) - _gyro_calibration.z_offset) * _gyro_calibration.z_scale;
	_gyro_int.put_with_interval(data.fifo_sample_interval_us,
				    gyro_val,
				    vec_integrated_unused,
				    integral_dt_unused);

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
	mag_report mag_report = {};

	accel_report.timestamp = gyro_report.timestamp = hrt_absolute_time();
	mag_report.timestamp = accel_report.timestamp;

	// TODO: get these right
	gyro_report.scaling = -1.0f;
	gyro_report.range_rad_s = -1.0f;
	gyro_report.device_id = m_id.dev_id;

	accel_report.scaling = -1.0f;
	accel_report.range_m_s2 = -1.0f;
	accel_report.device_id = m_id.dev_id;

	if (_mag_enabled) {
		mag_report.scaling = -1.0f;
		mag_report.range_ga = -1.0f;
		mag_report.device_id = m_id.dev_id;
	}

	// TODO: remove these (or get the values)
	gyro_report.x_raw = 0;
	gyro_report.y_raw = 0;
	gyro_report.z_raw = 0;

	accel_report.x_raw = 0;
	accel_report.y_raw = 0;
	accel_report.z_raw = 0;

	if (_mag_enabled) {
		mag_report.x_raw = 0;
		mag_report.y_raw = 0;
		mag_report.z_raw = 0;
	}

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

	if (_mag_enabled) {
		math::Vector<3> mag_val(data.mag_ga_x,
					data.mag_ga_y,
					data.mag_ga_z);
		mag_val = _rotation_matrix * mag_val;
		mag_val(0) = (mag_val(0) - _mag_calibration.x_offset) * _mag_calibration.x_scale;
		mag_val(1) = (mag_val(1) - _mag_calibration.y_offset) * _mag_calibration.y_scale;
		mag_val(2) = (mag_val(2) - _mag_calibration.z_offset) * _mag_calibration.z_scale;

		mag_report.x = mag_val(0);
		mag_report.y = mag_val(1);
		mag_report.z = mag_val(2);
	}

	gyro_report.x_integral = gyro_val_integ(0);
	gyro_report.y_integral = gyro_val_integ(1);
	gyro_report.z_integral = gyro_val_integ(2);

	accel_report.x_integral = accel_val_integ(0);
	accel_report.y_integral = accel_val_integ(1);
	accel_report.z_integral = accel_val_integ(2);

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked)) {


		if (_gyro_topic != nullptr) {
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &gyro_report);
		}

		if (_accel_topic != nullptr) {
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		}

		if (_mag_topic != nullptr) {
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &mag_report);
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


namespace df_lsm9ds1_wrapper
{

DfLsm9ds1Wrapper *g_dev = nullptr;

int start(bool mag_enabled, enum Rotation rotation);
int stop();
int info();
void usage();

int start(bool mag_enabled, enum Rotation rotation)
{
	g_dev = new DfLsm9ds1Wrapper(mag_enabled, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfLsm9ds1Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfLsm9ds1Wrapper start failed");
		return ret;
	}

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(IMU_DEVICE_ACC_GYRO, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    IMU_DEVICE_ACC_GYRO, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	DevMgr::getHandle(IMU_DEVICE_MAG, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    IMU_DEVICE_MAG, h.getError());
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
	PX4_INFO("Usage: df_lsm9ds1_wrapper 'start', 'info', 'stop', 'start_without_mag'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace df_lsm9ds1_wrapper


int
df_lsm9ds1_wrapper_main(int argc, char *argv[])
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
			df_lsm9ds1_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_lsm9ds1_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start_without_mag")) {
		ret = df_lsm9ds1_wrapper::start(false, rotation);
	}

	else if (!strcmp(verb, "start")) {
		ret = df_lsm9ds1_wrapper::start(true, rotation);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_lsm9ds1_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_lsm9ds1_wrapper::info();
	}

	else {
		df_lsm9ds1_wrapper::usage();
		return 1;
	}

	return ret;
}

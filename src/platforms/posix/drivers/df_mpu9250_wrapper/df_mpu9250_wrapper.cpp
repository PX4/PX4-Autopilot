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

#include "df_mpu9250_wrapper.hpp"

#include <lib/conversion/rotation.h>

#define MPU9250_ACCEL_SAMPLE_RATE_INTERVAL_US (1000000 / MPU9250_ACCEL_SAMPLE_RATE_HZ)
#define MPU9250_GYRO_SAMPLE_RATE_INTERVAL_US (1000000 / MPU9250_GYRO_SAMPLE_RATE_HZ)

#define MPU9250_ACCEL_PUBLISH_RATE (250)
#define MPU9250_GYRO_PUBLISH_RATE (250)

#define MPU9250_ACCEL_FILTER_FREQ 67
#define MPU9250_GYRO_FILTER_FREQ 67

extern "C" { int df_mpu9250_wrapper_main(int argc, char *argv[]); }

static DfMpu9250Wrapper *g_dev = nullptr;

DfMpu9250Wrapper::DfMpu9250Wrapper(bool mag_enabled, enum Rotation rotation) :
	MPU9250(IMU_DEVICE_PATH, mag_enabled),
	_accel_topic(nullptr),
	_gyro_topic(nullptr),
	_mag_topic(nullptr),
	_mavlink_log_pub(nullptr),
	_param_update_sub(-1),
	_accel_orb_class_instance(-1),
	_gyro_orb_class_instance(-1),
	_mag_orb_class_instance(-1),
	_accel_int(1000000 / MPU9250_ACCEL_PUBLISH_RATE, false),
	_gyro_int(1000000 / MPU9250_GYRO_PUBLISH_RATE, true),
	_accel_filter_x(MPU9250_ACCEL_SAMPLE_RATE_HZ, MPU9250_ACCEL_FILTER_FREQ),
	_accel_filter_y(MPU9250_ACCEL_SAMPLE_RATE_HZ, MPU9250_ACCEL_FILTER_FREQ),
	_accel_filter_z(MPU9250_ACCEL_SAMPLE_RATE_HZ, MPU9250_ACCEL_FILTER_FREQ),
	_gyro_filter_x(MPU9250_GYRO_SAMPLE_RATE_HZ, MPU9250_GYRO_FILTER_FREQ),
	_gyro_filter_y(MPU9250_GYRO_SAMPLE_RATE_HZ, MPU9250_GYRO_FILTER_FREQ),
	_gyro_filter_z(MPU9250_GYRO_SAMPLE_RATE_HZ, MPU9250_GYRO_FILTER_FREQ),
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

	_mag_calibration.x_scale = 1.0f;
	_mag_calibration.y_scale = 1.0f;
	_mag_calibration.z_scale = 1.0f;
	_mag_calibration.x_offset = 0.0f;
	_mag_calibration.y_offset = 0.0f;
	_mag_calibration.z_offset = 0.0f;

	_accel_published = perf_alloc(PC_COUNT, "mpu9250_accel_published");
	_accel_callbacks = perf_alloc(PC_COUNT, "mpu9250_accel_callbacks");
	_accel_interval = perf_alloc(PC_INTERVAL, "mpu9250_accel_interval");

	_gyro_published = perf_alloc(PC_COUNT, "mpu9250_gyro_published");
	_gyro_callbacks = perf_alloc(PC_COUNT, "mpu9250_gyro_callbacks");
	_gyro_interval = perf_alloc(PC_INTERVAL, "mpu9250_gyro_interval");

	_mag_published = perf_alloc(PC_COUNT, "mpu9250_mag_published");
	_mag_callbacks = perf_alloc(PC_COUNT, "mpu9250_mag_callbacks");
	_mag_interval = perf_alloc(PC_INTERVAL, "mpu9250_mag_interval");

	_accel_range_hits = perf_alloc(PC_COUNT, "mpu9250_accel_range_hits");
	_gyro_range_hits = perf_alloc(PC_COUNT, "mpu9250_gyro_range_hits");

	_accel_duplicates = perf_alloc(PC_COUNT, "mpu9250_accel_duplicates");
	_gyro_duplicates = perf_alloc(PC_COUNT, "mpu9250_gyro_duplicates");
	_mag_duplicates = perf_alloc(PC_COUNT, "mpu9250_mag_duplicates");

	_mag_overflows = perf_alloc(PC_COUNT, "mpu9250_mag_overflows");
	_mag_overruns = perf_alloc(PC_COUNT, "mpu9250_mag_overruns");

	_fifo_overflows = perf_alloc(PC_COUNT, "mpu9250_fifo_overflows");
	_fifo_reads = perf_alloc(PC_COUNT, "mpu9250_fifo_reads");
	_fifo_avg_packets = perf_alloc(PC_COUNT, "mpu9250_fifo_avg_packets");
	_fifo_corruptions = perf_alloc(PC_COUNT, "mpu9250_fifo_corruptions");

	_errors = perf_alloc(PC_COUNT, "mpu9250_errors");
}

DfMpu9250Wrapper::~DfMpu9250Wrapper()
{
	perf_free(_accel_callbacks);
	perf_free(_accel_published);

	perf_free(_gyro_published);
	perf_free(_gyro_callbacks);

	perf_free(_mag_published);
	perf_free(_mag_callbacks);

	perf_free(_accel_range_hits);
	perf_free(_gyro_range_hits);

	perf_free(_accel_duplicates);
	perf_free(_gyro_duplicates);
	perf_free(_mag_duplicates);

	perf_free(_mag_overflows);
	perf_free(_mag_overruns);
	perf_free(_fifo_overflows);
	perf_free(_fifo_reads);
	perf_free(_fifo_corruptions);
	perf_free(_errors);
}

int DfMpu9250Wrapper::start()
{
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
	perf_print_counter(_accel_published);
	perf_print_counter(_accel_callbacks);
	perf_print_counter(_accel_interval);
	perf_print_counter(_accel_range_hits);
	perf_print_counter(_accel_duplicates);

	perf_print_counter(_gyro_published);
	perf_print_counter(_gyro_callbacks);
	perf_print_counter(_gyro_interval);
	perf_print_counter(_gyro_range_hits);
	perf_print_counter(_gyro_duplicates);

	perf_print_counter(_mag_published);
	perf_print_counter(_mag_callbacks);
	perf_print_counter(_mag_interval);
	perf_print_counter(_mag_duplicates);
	perf_print_counter(_mag_overflows);
	perf_print_counter(_mag_overruns);

	perf_print_counter(_fifo_overflows);
	perf_print_counter(_fifo_reads);
	perf_print_counter(_fifo_avg_packets);
	perf_print_counter(_fifo_corruptions);

	perf_print_counter(_errors);
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

int DfMpu9250Wrapper::_publish(const struct accel_data &data)
{
	uint64_t timestamp = hrt_absolute_time();
	bool updated;
	bool accel_notify;

	perf_count(_accel_interval);
	perf_count(_accel_callbacks);

	/* copy perf info from MPU9250, this will go away when DF is merged back */
	perf_set_count(_accel_range_hits, _counters.accel_range_hits);
	perf_set_count(_gyro_range_hits, _counters.gyro_range_hits);

	perf_set_count(_accel_duplicates, _counters.accel_duplicates);
	perf_set_count(_gyro_duplicates, _counters.gyro_duplicates);
	perf_set_count(_mag_duplicates, _counters.mag_duplicates);

	perf_set_count(_mag_overflows, _counters.mag_overflows);
	perf_set_count(_mag_overruns, _counters.mag_overruns);

	perf_set_count(_fifo_overflows, _counters.fifo_overflows);
	perf_set_count(_fifo_reads, _counters.fifo_reads);
	perf_set_count(_fifo_avg_packets, _counters.fifo_avg_packets);
	perf_set_count(_fifo_corruptions, _counters.fifo_corruptions);
	perf_set_count(_errors, _counters.errors);

	/* Check if calibration values are still up-to-date. */
	orb_check(_param_update_sub, &updated);
	if (updated) {
		parameter_update_s parameter_update;
		orb_copy(ORB_ID(parameter_update), _param_update_sub, &parameter_update);

		_update_accel_calibration();
		_update_gyro_calibration();
		_update_mag_calibration();
	}

	float xraw_f = data.accel_m_s2_x;
	float yraw_f = data.accel_m_s2_y;
	float zraw_f = data.accel_m_s2_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	// adjust values according to the calibration
	float x_in_new = (xraw_f - _accel_calibration.x_offset) * _accel_calibration.x_scale;
	float y_in_new = (yraw_f - _accel_calibration.y_offset) * _accel_calibration.y_scale;
	float z_in_new = (zraw_f - _accel_calibration.z_offset) * _accel_calibration.z_scale;

	float x_filt = _accel_filter_x.apply(x_in_new);
	float y_filt = _accel_filter_y.apply(y_in_new);
	float z_filt = _accel_filter_z.apply(z_in_new);

	math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
	math::Vector<3> aval_integrated;

	uint64_t integral_dt;
	accel_notify = _accel_int.put_with_interval(MPU9250_ACCEL_SAMPLE_RATE_INTERVAL_US
			, aval
			, aval_integrated
			, integral_dt);

	if (accel_notify) {
		accel_report accel_report = {
				.timestamp = timestamp,
				.integral_dt = integral_dt,
				.error_count = 0,
				.x = x_filt,
				.y = y_filt,
				.z = z_filt,
				.x_integral = aval_integrated(0),
				.y_integral = aval_integrated(1),
				.z_integral = aval_integrated(2),
				.temperature = -1.0f,
				// TODO: get these right
				.range_m_s2 = -1.0f,
				.scaling = -1.0f,
				.device_id = m_id.dev_id,
				.x_raw = 0,
				.y_raw = 0,
				.z_raw = 0,
				.temperature_raw = 0,
		};

		if (_accel_topic != nullptr) {
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		} else {
			_accel_topic = orb_advertise_multi_queue(ORB_ID(sensor_accel), &accel_report,
							   &_accel_orb_class_instance, ORB_PRIO_DEFAULT, 2);

			if (_accel_topic == nullptr) {
				PX4_ERR("sensor_accel advert fail");
				return -1;
			}
		}

		/* Notify anyone waiting for data. */
		DevMgr::updateNotify(*this);

		perf_count(_accel_published);
	}
	// Report if there are high vibrations, every 10 times it happens.
	const bool threshold_reached = (_counters.accel_range_hits - _last_accel_range_hit_count > 10);

	// Report every 5s.
	const bool due_to_report = (hrt_elapsed_time(&_last_accel_range_hit_time) > 5000000);

	if (threshold_reached && due_to_report) {
		mavlink_log_critical(&_mavlink_log_pub,
				     "High accelerations, range exceeded %llu times",
				     _counters.accel_range_hits);

		_last_accel_range_hit_time = hrt_absolute_time();
		_last_accel_range_hit_count = _counters.accel_range_hits;
	}

	return 0;
}

int DfMpu9250Wrapper::_publish(const struct gyro_data &data)
{
	uint64_t timestamp = hrt_absolute_time();
	bool gyro_notify;

	perf_count(_gyro_interval);
	perf_count(_gyro_callbacks);

	float xraw_f = data.gyro_rad_s_x;
	float yraw_f = data.gyro_rad_s_y;
	float zraw_f = data.gyro_rad_s_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	// adjust values according to the calibration
	float x_gyro_in_new = (xraw_f - _gyro_calibration.x_offset) * _gyro_calibration.x_scale;
	float y_gyro_in_new = (yraw_f - _gyro_calibration.y_offset) * _gyro_calibration.y_scale;
	float z_gyro_in_new = (zraw_f - _gyro_calibration.z_offset) * _gyro_calibration.z_scale;

	float x_filt = _gyro_filter_x.apply(x_gyro_in_new);
	float y_filt = _gyro_filter_y.apply(y_gyro_in_new);
	float z_filt = _gyro_filter_z.apply(z_gyro_in_new);

	math::Vector<3> gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	math::Vector<3> gval_integrated;

	uint64_t integral_dt;
	gyro_notify = _gyro_int.put_with_interval(MPU9250_GYRO_SAMPLE_RATE_INTERVAL_US
			, gval
			, gval_integrated
			, integral_dt);

	if (gyro_notify) {
		gyro_report gyro_report = {
				.timestamp = timestamp,
				.integral_dt = integral_dt,
				.error_count = 0,
				.x = x_filt,
				.y = y_filt,
				.z = z_filt,
				.x_integral = gval_integrated(0),
				.y_integral = gval_integrated(1),
				.z_integral = gval_integrated(2),
				.temperature = -1.0f,
				// TODO: get these right
				.range_rad_s = -1.0f,
				.scaling = -1.0f,
				.device_id = m_id.dev_id,
				.x_raw = 0,
				.y_raw = 0,
				.z_raw = 0,
				.temperature_raw = 0,
		};

		if (_gyro_topic != nullptr) {
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &gyro_report);
		} else {
			_gyro_topic = orb_advertise_multi_queue(ORB_ID(sensor_gyro), &gyro_report,
							  &_gyro_orb_class_instance, ORB_PRIO_DEFAULT, 2);

			if (_gyro_topic == nullptr) {
				PX4_ERR("sensor_gyro advert fail");
				return -1;
			}
		}

		/* Notify anyone waiting for data. */
		DevMgr::updateNotify(*this);

		perf_count(_gyro_published);
	}

	// Report if there are high vibrations, every 10 times it happens.
	const bool threshold_reached = (_counters.gyro_range_hits - _last_gyro_range_hit_count > 10);

	// Report every 5s.
	const bool due_to_report = (hrt_elapsed_time(&_last_gyro_range_hit_time) > 5000000);

	if (threshold_reached && due_to_report) {
		mavlink_log_critical(&_mavlink_log_pub,
				     "High rotations, range exceeded %llu times",
					 _counters.gyro_range_hits);

		_last_gyro_range_hit_time = hrt_absolute_time();
		_last_gyro_range_hit_count = _counters.gyro_range_hits;
	}

	return 0;
}

int DfMpu9250Wrapper::_publish(const struct mag_data &data)
{
	uint64_t timestamp = hrt_absolute_time();

	perf_count(_mag_interval);

	float xraw_f = data.mag_ga_x;
	float yraw_f = data.mag_ga_y;
	float zraw_f = data.mag_ga_z;

	perf_count(_mag_callbacks);

	xraw_f = (xraw_f - _mag_calibration.x_offset) * _mag_calibration.x_scale;
	yraw_f = (yraw_f - _mag_calibration.y_offset) * _mag_calibration.y_scale;
	zraw_f = (zraw_f - _mag_calibration.z_offset) * _mag_calibration.z_scale;

	mag_report mag_report = {
			.timestamp = timestamp,
			.error_count = 0,
			.x = xraw_f,
			.y = yraw_f,
			.z = zraw_f,
			// TODO: get these right
			.range_ga = -1.0f,
			.scaling = -1.0f,
			.temperature = -1.0f,
			.device_id = m_id.dev_id,
			.x_raw = 0,
			.y_raw = 0,
			.z_raw = 0,
			.is_external = false,
	};

	if (_mag_topic != nullptr) {
		orb_publish(ORB_ID(sensor_mag), _mag_topic, &mag_report);
	} else {
		_mag_topic = orb_advertise_multi_queue(ORB_ID(sensor_mag), &mag_report,
						 &_mag_orb_class_instance, ORB_PRIO_DEFAULT, 2);
		if (_gyro_topic == nullptr) {
			PX4_ERR("sensor_mag advert fail");
			return -1;
		}
	}

	/* Notify anyone waiting for data. */
	DevMgr::updateNotify(*this);

	perf_count(_mag_published);

	return 0;
}

static int start(bool mag_enabled, enum Rotation rotation);
static int stop();
static int info();
static void usage();

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
	PX4_INFO("    -p <cpu_affinity>");
}

int
df_mpu9250_wrapper_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;
	int cpu_pinned = -1;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "Rp:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'p':
			cpu_pinned = atoi(myoptarg);
			break;

		default:
			usage();
			return 0;
		}
	}

	if (argc <= 1) {
		usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start_without_mag")) {
		ret = start(false, rotation);
	}

	else if (!strcmp(verb, "start")) {
		ret = start(true, rotation);
		if (cpu_pinned != -1 && ret == 0) {
			int ret2 = g_dev->pinThread(cpu_pinned);
			if (ret2 != 0)
				PX4_WARN("df_mpu9250_wrapper CPU pin failed: %d", ret2);
		}
	}

	else if (!strcmp(verb, "stop")) {
		ret = stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = info();
	}

	else {
		usage();
		return 1;
	}

	return ret;
}


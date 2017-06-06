/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file voted_sensors_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "voted_sensors_update.h"

#include <systemlib/mavlink_log.h>

#include <conversion/rotation.h>

#define MAG_ROT_VAL_INTERNAL		-1
#define CAL_ERROR_APPLY_CAL_MSG "FAILED APPLYING %s CAL #%u"


using namespace sensors;
using namespace DriverFramework;


const double VotedSensorsUpdate::_msl_pressure = 101.325f;

VotedSensorsUpdate::VotedSensorsUpdate(const Parameters &parameters, bool hil_enabled)
	: _parameters(parameters), _hil_enabled(hil_enabled)
{
	memset(&_last_sensor_data, 0, sizeof(_last_sensor_data));
	memset(&_last_accel_timestamp, 0, sizeof(_last_accel_timestamp));
	memset(&_last_mag_timestamp, 0, sizeof(_last_mag_timestamp));
	memset(&_last_baro_timestamp, 0, sizeof(_last_baro_timestamp));
	memset(&_accel_diff, 0, sizeof(_accel_diff));
	memset(&_gyro_diff, 0, sizeof(_gyro_diff));

	// initialise the corrections
	memset(&_corrections, 0, sizeof(_corrections));

	for (unsigned i = 0; i < 3; i++) {
		_corrections.gyro_scale_0[i] = 1.0f;
		_corrections.accel_scale_0[i] = 1.0f;
		_corrections.gyro_scale_1[i] = 1.0f;
		_corrections.accel_scale_1[i] = 1.0f;
		_corrections.gyro_scale_2[i] = 1.0f;
		_corrections.accel_scale_2[i] = 1.0f;
	}

	_corrections.baro_scale_0 = 1.0f;
	_corrections.baro_scale_1 = 1.0f;
	_corrections.baro_scale_2 = 1.0f;

	_baro.voter.set_timeout(300000);
	_mag.voter.set_timeout(300000);
	_mag.voter.set_equal_value_threshold(1000);

	if (_hil_enabled) { // HIL has less accurate timing so increase the timeouts a bit
		_gyro.voter.set_timeout(500000);
		_accel.voter.set_timeout(500000);
	}
}

int VotedSensorsUpdate::init(sensor_combined_s &raw)
{
	raw.accelerometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
	raw.magnetometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
	raw.baro_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
	raw.timestamp = 0;

	initialize_sensors();

	_corrections_changed = true; //make sure to initially publish the corrections topic
	_selection_changed = true;

	return 0;
}

void VotedSensorsUpdate::initialize_sensors()
{
	init_sensor_class(ORB_ID(sensor_gyro), _gyro, GYRO_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_mag), _mag, MAG_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_accel), _accel, ACCEL_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_baro), _baro, BARO_COUNT_MAX);
}

void VotedSensorsUpdate::deinit()
{
	for (unsigned i = 0; i < _gyro.subscription_count; i++) {
		orb_unsubscribe(_gyro.subscription[i]);
	}

	for (unsigned i = 0; i < _accel.subscription_count; i++) {
		orb_unsubscribe(_accel.subscription[i]);
	}

	for (unsigned i = 0; i < _mag.subscription_count; i++) {
		orb_unsubscribe(_mag.subscription[i]);
	}

	for (unsigned i = 0; i < _baro.subscription_count; i++) {
		orb_unsubscribe(_baro.subscription[i]);
	}
}

void VotedSensorsUpdate::parameters_update()
{
	get_rot_matrix((enum Rotation)_parameters.board_rotation, &_board_rotation);
	/* fine tune board offset */
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _parameters.board_offset[0],
					 M_DEG_TO_RAD_F * _parameters.board_offset[1],
					 M_DEG_TO_RAD_F * _parameters.board_offset[2]);

	_board_rotation = board_rotation_offset * _board_rotation;

	/* Load & apply the sensor calibrations.
	 * IMPORTANT: we assume all sensor drivers are running and published sensor data at this point
	 */

	/* temperature compensation */
	_temperature_compensation.parameters_update();

	/* gyro */
	for (unsigned topic_instance = 0; topic_instance < GYRO_COUNT_MAX; ++topic_instance) {

		if (topic_instance < _gyro.subscription_count) {
			// valid subscription, so get the driver id by getting the published sensor data
			struct gyro_report report;

			if (orb_copy(ORB_ID(sensor_gyro), _gyro.subscription[topic_instance], &report) == 0) {
				int temp = _temperature_compensation.set_sensor_id_gyro(report.device_id, topic_instance);

				if (temp < 0) {
					PX4_ERR("gyro temp compensation init: failed to find device ID %u for instance %i",
						report.device_id, topic_instance);
					_corrections.gyro_mapping[topic_instance] =  0;

				} else {
					_corrections.gyro_mapping[topic_instance] =  temp;

				}
			}
		}
	}


	/* accel */
	for (unsigned topic_instance = 0; topic_instance < ACCEL_COUNT_MAX; ++topic_instance) {

		if (topic_instance < _accel.subscription_count) {
			// valid subscription, so get the driver id by getting the published sensor data
			struct accel_report report;

			if (orb_copy(ORB_ID(sensor_accel), _accel.subscription[topic_instance], &report) == 0) {
				int temp = _temperature_compensation.set_sensor_id_accel(report.device_id, topic_instance);

				if (temp < 0) {
					PX4_ERR("accel temp compensation init: failed to find device ID %u for instance %i",
						report.device_id, topic_instance);
					_corrections.accel_mapping[topic_instance] =  0;

				} else {
					_corrections.accel_mapping[topic_instance] =  temp;

				}
			}
		}
	}

	/* baro */
	for (unsigned topic_instance = 0; topic_instance < BARO_COUNT_MAX; ++topic_instance) {

		if (topic_instance < _baro.subscription_count) {
			// valid subscription, so get the driver id by getting the published sensor data
			struct baro_report report;

			if (orb_copy(ORB_ID(sensor_baro), _baro.subscription[topic_instance], &report) == 0) {
				int temp = _temperature_compensation.set_sensor_id_baro(report.device_id, topic_instance);

				if (temp < 0) {
					PX4_ERR("baro temp compensation init: failed to find device ID %u for instance %i",
						report.device_id, topic_instance);
					_corrections.baro_mapping[topic_instance] =  0;

				} else {
					_corrections.baro_mapping[topic_instance] =  temp;

				}
			}
		}
	}


	/* set offset parameters to new values */
	bool failed;
	char str[30];
	unsigned mag_count = 0;
	unsigned gyro_count = 0;
	unsigned accel_count = 0;
	unsigned gyro_cal_found_count = 0;
	unsigned accel_cal_found_count = 0;

	/* run through all gyro sensors */
	for (unsigned driver_index = 0; driver_index < GYRO_COUNT_MAX; driver_index++) {

		(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, driver_index);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			continue;
		}

		uint32_t driver_device_id = h.ioctl(DEVIOCGDEVICEID, 0);
		bool config_ok = false;

		/* run through all stored calibrations that are applied at the driver level*/
		for (unsigned i = 0; i < GYRO_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_GYRO%u_ID", i);
			int device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			if (failed) {
				DevMgr::releaseHandle(h);
				continue;
			}

			if (driver_index == 0 && device_id > 0) {
				gyro_cal_found_count++;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == driver_device_id) {
				struct gyro_calibration_s gscale = {};
				(void)sprintf(str, "CAL_GYRO%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_offset));
				(void)sprintf(str, "CAL_GYRO%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_offset));
				(void)sprintf(str, "CAL_GYRO%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.z_offset));
				(void)sprintf(str, "CAL_GYRO%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_scale));
				(void)sprintf(str, "CAL_GYRO%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_scale));
				(void)sprintf(str, "CAL_GYRO%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.z_scale));

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro", i);

				} else {
					/* apply new scaling and offsets */
					config_ok = apply_gyro_calibration(h, &gscale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			gyro_count++;
		}
	}

	// There are less gyros than calibrations
	// reset the board calibration and fail the initial load
	if (gyro_count < gyro_cal_found_count) {

		// run through all stored calibrations and reset them
		for (unsigned i = 0; i < GYRO_COUNT_MAX; i++) {

			int device_id = 0;
			(void)sprintf(str, "CAL_GYRO%u_ID", i);
			(void)param_set(param_find(str), &device_id);
		}
	}

	/* run through all accel sensors */
	for (unsigned driver_index = 0; driver_index < ACCEL_COUNT_MAX; driver_index++) {

		(void)sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, driver_index);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			continue;
		}

		uint32_t driver_device_id = h.ioctl(DEVIOCGDEVICEID, 0);
		bool config_ok = false;

		/* run through all stored calibrations */
		for (unsigned i = 0; i < ACCEL_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_ACC%u_ID", i);
			int device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			if (failed) {
				DevMgr::releaseHandle(h);
				continue;
			}

			if (driver_index == 0 && device_id > 0) {
				accel_cal_found_count++;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == driver_device_id) {
				struct accel_calibration_s ascale = {};
				(void)sprintf(str, "CAL_ACC%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_offset));
				(void)sprintf(str, "CAL_ACC%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_offset));
				(void)sprintf(str, "CAL_ACC%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.z_offset));
				(void)sprintf(str, "CAL_ACC%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_scale));
				(void)sprintf(str, "CAL_ACC%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_scale));
				(void)sprintf(str, "CAL_ACC%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.z_scale));

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel", i);

				} else {
					/* apply new scaling and offsets */
					config_ok = apply_accel_calibration(h, &ascale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			accel_count++;
		}
	}

	// There are less accels than calibrations
	// reset the board calibration and fail the initial load
	if (accel_count < accel_cal_found_count) {

		// run through all stored calibrations and reset them
		for (unsigned i = 0; i < ACCEL_COUNT_MAX; i++) {

			int device_id = 0;
			(void)sprintf(str, "CAL_ACC%u_ID", i);
			(void)param_set(param_find(str), &device_id);
		}
	}

	/* run through all mag sensors */
	for (unsigned driver_index = 0; driver_index < MAG_COUNT_MAX; driver_index++) {

		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, driver_index);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			/* the driver is not running, abort */
			continue;
		}

		_mag_device_id[driver_index] = h.ioctl(DEVIOCGDEVICEID, 0);
		bool config_ok = false;

		/* run through all stored calibrations */
		for (unsigned i = 0; i < MAG_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_MAG%u_ID", i);
			int device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			if (failed) {
				DevMgr::releaseHandle(h);
				continue;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == _mag_device_id[driver_index]) {
				struct mag_calibration_s mscale = {};
				(void)sprintf(str, "CAL_MAG%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_offset));
				(void)sprintf(str, "CAL_MAG%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_offset));
				(void)sprintf(str, "CAL_MAG%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_offset));
				(void)sprintf(str, "CAL_MAG%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_scale));
				(void)sprintf(str, "CAL_MAG%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_scale));
				(void)sprintf(str, "CAL_MAG%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_scale));

				(void)sprintf(str, "CAL_MAG%u_ROT", i);

				if (h.ioctl(MAGIOCGEXTERNAL, 0) <= 0) {
					/* mag is internal - reset param to -1 to indicate internal mag */
					int32_t minus_one;
					param_get(param_find(str), &minus_one);

					if (minus_one != MAG_ROT_VAL_INTERNAL) {
						minus_one = MAG_ROT_VAL_INTERNAL;
						param_set_no_notification(param_find(str), &minus_one);
					}

				} else {
					/* mag is external */
					int32_t mag_rot;
					param_get(param_find(str), &mag_rot);

					/* check if this mag is still set as internal, otherwise leave untouched */
					if (mag_rot < 0) {
						/* it was marked as internal, change to external with no rotation */
						mag_rot = 0;
						param_set_no_notification(param_find(str), &mag_rot);
					}
				}

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag", i);

				} else {

					/* apply new scaling and offsets */
					config_ok = apply_mag_calibration(h, &mscale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			mag_count++;
		}
	}

}

void VotedSensorsUpdate::accel_poll(struct sensor_combined_s &raw)
{
	float *offsets[] = {_corrections.accel_offset_0, _corrections.accel_offset_1, _corrections.accel_offset_2 };
	float *scales[] = {_corrections.accel_scale_0, _corrections.accel_scale_1, _corrections.accel_scale_2 };

	for (unsigned uorb_index = 0; uorb_index < _accel.subscription_count; uorb_index++) {
		bool accel_updated;
		orb_check(_accel.subscription[uorb_index], &accel_updated);

		if (accel_updated) {
			struct accel_report accel_report;

			orb_copy(ORB_ID(sensor_accel), _accel.subscription[uorb_index], &accel_report);

			if (accel_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_accel.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_accel.subscription[uorb_index], &priority);
				_accel.priority[uorb_index] = (uint8_t)priority;
			}

			_accel_device_id[uorb_index] = accel_report.device_id;

			math::Vector<3> accel_data;

			if (accel_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * becasue it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta velocities to an equivalent acceleration before application of corrections
				float dt_inv = 1.e6f / accel_report.integral_dt;
				accel_data = math::Vector<3>(accel_report.x_integral * dt_inv, accel_report.y_integral * dt_inv,
							     accel_report.z_integral * dt_inv);

				_last_sensor_data[uorb_index].accelerometer_integral_dt = accel_report.integral_dt / 1.e6f;

			} else {
				// using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				accel_data = math::Vector<3>(accel_report.x, accel_report.y, accel_report.z);

				// handle the cse where this is our first output
				if (_last_accel_timestamp[uorb_index] == 0) {
					_last_accel_timestamp[uorb_index] = accel_report.timestamp - 1000;
				}

				// approximate the  delta time using the difference in accel data time stamps
				_last_sensor_data[uorb_index].accelerometer_integral_dt =
					(accel_report.timestamp - _last_accel_timestamp[uorb_index]) / 1.e6f;
			}

			// handle temperature compensation
			if (!_hil_enabled) {
				if (_temperature_compensation.apply_corrections_accel(uorb_index, accel_data, accel_report.temperature,
						offsets[uorb_index], scales[uorb_index]) == 2) {
					_corrections_changed = true;
				}
			}

			// rotate corrected measurements from sensor to body frame
			accel_data = _board_rotation * accel_data;

			_last_sensor_data[uorb_index].accelerometer_m_s2[0] = accel_data(0);
			_last_sensor_data[uorb_index].accelerometer_m_s2[1] = accel_data(1);
			_last_sensor_data[uorb_index].accelerometer_m_s2[2] = accel_data(2);

			_last_accel_timestamp[uorb_index] = accel_report.timestamp;
			_accel.voter.put(uorb_index, accel_report.timestamp, _last_sensor_data[uorb_index].accelerometer_m_s2,
					 accel_report.error_count, _accel.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index;
	_accel.voter.get_best(hrt_absolute_time(), &best_index);

	// write the best sensor data to the output variables
	if (best_index >= 0) {
		raw.accelerometer_integral_dt = _last_sensor_data[best_index].accelerometer_integral_dt;

		if (best_index != _accel.last_best_vote) {
			_accel.last_best_vote = (uint8_t)best_index;
			_corrections.selected_accel_instance = (uint8_t)best_index;
			_corrections_changed = true;
		}

		if (_selection.accel_device_id != _accel_device_id[best_index]) {
			_selection_changed = true;
			_selection.accel_device_id = _accel_device_id[best_index];
		}

		for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
			raw.accelerometer_m_s2[axis_index] = _last_sensor_data[best_index].accelerometer_m_s2[axis_index];
		}
	}
}

void VotedSensorsUpdate::gyro_poll(struct sensor_combined_s &raw)
{
	float *offsets[] = {_corrections.gyro_offset_0, _corrections.gyro_offset_1, _corrections.gyro_offset_2 };
	float *scales[] = {_corrections.gyro_scale_0, _corrections.gyro_scale_1, _corrections.gyro_scale_2 };

	for (unsigned uorb_index = 0; uorb_index < _gyro.subscription_count; uorb_index++) {
		bool gyro_updated;
		orb_check(_gyro.subscription[uorb_index], &gyro_updated);

		if (gyro_updated) {
			struct gyro_report gyro_report;

			orb_copy(ORB_ID(sensor_gyro), _gyro.subscription[uorb_index], &gyro_report);

			if (gyro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_gyro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_gyro.subscription[uorb_index], &priority);
				_gyro.priority[uorb_index] = (uint8_t)priority;
			}

			_gyro_device_id[uorb_index] = gyro_report.device_id;

			math::Vector<3> gyro_rate;

			if (gyro_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * becasue it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta angles to an equivalent angular rate before application of corrections
				float dt_inv = 1.e6f / gyro_report.integral_dt;
				gyro_rate = math::Vector<3>(gyro_report.x_integral * dt_inv, gyro_report.y_integral * dt_inv,
							    gyro_report.z_integral * dt_inv);

				_last_sensor_data[uorb_index].gyro_integral_dt = gyro_report.integral_dt / 1.e6f;

			} else {
				//using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				gyro_rate = math::Vector<3>(gyro_report.x, gyro_report.y, gyro_report.z);

				// handle the case where this is our first output
				if (_last_sensor_data[uorb_index].timestamp == 0) {
					_last_sensor_data[uorb_index].timestamp = gyro_report.timestamp - 1000;
				}

				// approximate the  delta time using the difference in gyro data time stamps
				_last_sensor_data[uorb_index].gyro_integral_dt =
					(gyro_report.timestamp - _last_sensor_data[uorb_index].timestamp) / 1.e6f;
			}

			// handle temperature compensation
			if (!_hil_enabled) {
				if (_temperature_compensation.apply_corrections_gyro(uorb_index, gyro_rate, gyro_report.temperature,
						offsets[uorb_index], scales[uorb_index]) == 2) {
					_corrections_changed = true;
				}
			}

			// rotate corrected measurements from sensor to body frame
			gyro_rate = _board_rotation * gyro_rate;

			_last_sensor_data[uorb_index].gyro_rad[0] = gyro_rate(0);
			_last_sensor_data[uorb_index].gyro_rad[1] = gyro_rate(1);
			_last_sensor_data[uorb_index].gyro_rad[2] = gyro_rate(2);

			_last_sensor_data[uorb_index].timestamp = gyro_report.timestamp;
			_gyro.voter.put(uorb_index, gyro_report.timestamp, _last_sensor_data[uorb_index].gyro_rad,
					gyro_report.error_count, _gyro.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index;
	_gyro.voter.get_best(hrt_absolute_time(), &best_index);

	// write data for the best sensor to output variables
	if (best_index >= 0) {
		raw.gyro_integral_dt = _last_sensor_data[best_index].gyro_integral_dt;
		raw.timestamp = _last_sensor_data[best_index].timestamp;

		if (_gyro.last_best_vote != best_index) {
			_gyro.last_best_vote = (uint8_t)best_index;
			_corrections.selected_gyro_instance = (uint8_t)best_index;
			_corrections_changed = true;
		}

		if (_selection.gyro_device_id != _gyro_device_id[best_index]) {
			_selection_changed = true;
			_selection.gyro_device_id = _gyro_device_id[best_index];
		}

		for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
			raw.gyro_rad[axis_index] = _last_sensor_data[best_index].gyro_rad[axis_index];
		}
	}
}

void VotedSensorsUpdate::mag_poll(struct sensor_combined_s &raw)
{
	for (unsigned uorb_index = 0; uorb_index < _mag.subscription_count; uorb_index++) {
		bool mag_updated;
		orb_check(_mag.subscription[uorb_index], &mag_updated);

		if (mag_updated) {
			struct mag_report mag_report;

			orb_copy(ORB_ID(sensor_mag), _mag.subscription[uorb_index], &mag_report);

			if (mag_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_mag.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_mag.subscription[uorb_index], &priority);
				_mag.priority[uorb_index] = (uint8_t)priority;

				// Match rotation parameters to uORB topics first time we get data
				char str[30];
				int32_t mag_rot;

				for (int driver_index = 0; driver_index < MAG_COUNT_MAX; driver_index++) {
					if (mag_report.device_id == _mag_device_id[driver_index]) {

						(void)sprintf(str, "CAL_MAG%u_ROT", driver_index);
						param_get(param_find(str), &mag_rot);

						if (mag_rot < 0) {
							// Set internal magnetometers to use the board rotation
							_mag_rotation[uorb_index] = _board_rotation;

						} else {
							// Set external magnetometers to use the parameter value
							get_rot_matrix((enum Rotation)mag_rot, &_mag_rotation[uorb_index]);
						}
					}
				}
			}

			math::Vector<3> vect(mag_report.x, mag_report.y, mag_report.z);
			vect = _mag_rotation[uorb_index] * vect;

			_last_sensor_data[uorb_index].magnetometer_ga[0] = vect(0);
			_last_sensor_data[uorb_index].magnetometer_ga[1] = vect(1);
			_last_sensor_data[uorb_index].magnetometer_ga[2] = vect(2);

			_last_mag_timestamp[uorb_index] = mag_report.timestamp;
			_mag.voter.put(uorb_index, mag_report.timestamp, vect.data,
				       mag_report.error_count, _mag.priority[uorb_index]);
		}
	}

	int best_index;
	_mag.voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		raw.magnetometer_ga[0] = _last_sensor_data[best_index].magnetometer_ga[0];
		raw.magnetometer_ga[1] = _last_sensor_data[best_index].magnetometer_ga[1];
		raw.magnetometer_ga[2] = _last_sensor_data[best_index].magnetometer_ga[2];
		_mag.last_best_vote = (uint8_t)best_index;
	}

	if (_selection.mag_device_id != _mag_device_id[best_index]) {
		_selection_changed = true;
		_selection.mag_device_id = _mag_device_id[best_index];
	}
}

void VotedSensorsUpdate::baro_poll(struct sensor_combined_s &raw)
{
	bool got_update = false;
	float *offsets[] = {&_corrections.baro_offset_0, &_corrections.baro_offset_1, &_corrections.baro_offset_2 };
	float *scales[] = {&_corrections.baro_scale_0, &_corrections.baro_scale_1, &_corrections.baro_scale_2 };

	for (unsigned uorb_index = 0; uorb_index < _baro.subscription_count; uorb_index++) {
		bool baro_updated;
		orb_check(_baro.subscription[uorb_index], &baro_updated);

		if (baro_updated) {
			struct baro_report baro_report;

			orb_copy(ORB_ID(sensor_baro), _baro.subscription[uorb_index], &baro_report);

			if (baro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// Convert from millibar to Pa
			float corrected_pressure = 100.0f * baro_report.pressure;

			// handle temperature compensation
			if (!_hil_enabled) {
				if (_temperature_compensation.apply_corrections_baro(uorb_index, corrected_pressure, baro_report.temperature,
						offsets[uorb_index], scales[uorb_index]) == 2) {
					_corrections_changed = true;
				}
			}

			// First publication with data
			if (_baro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_baro.subscription[uorb_index], &priority);
				_baro.priority[uorb_index] = (uint8_t)priority;
			}

			_baro_device_id[uorb_index] = baro_report.device_id;

			got_update = true;
			math::Vector<3> vect(baro_report.altitude, 0.f, 0.f);

			_last_sensor_data[uorb_index].baro_alt_meter = baro_report.altitude;
			_last_sensor_data[uorb_index].baro_temp_celcius = baro_report.temperature;
			_last_baro_pressure[uorb_index] = corrected_pressure;

			_last_baro_timestamp[uorb_index] = baro_report.timestamp;
			_baro.voter.put(uorb_index, baro_report.timestamp, vect.data,
					baro_report.error_count, _baro.priority[uorb_index]);
		}
	}

	if (got_update) {
		int best_index;
		_baro.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0) {
			raw.baro_temp_celcius = _last_sensor_data[best_index].baro_temp_celcius;
			_last_best_baro_pressure = _last_baro_pressure[best_index];

			if (_baro.last_best_vote != best_index) {
				_baro.last_best_vote = (uint8_t)best_index;
				_corrections.selected_baro_instance = (uint8_t)best_index;
				_corrections_changed = true;
			}

			if (_selection.baro_device_id != _baro_device_id[best_index]) {
				_selection_changed = true;
				_selection.baro_device_id = _baro_device_id[best_index];
			}

			/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

			/*
			 * PERFORMANCE HINT:
			 *
			 * The single precision calculation is 50 microseconds faster than the double
			 * precision variant. It is however not obvious if double precision is required.
			 * Pending more inspection and tests, we'll leave the double precision variant active.
			 *
			 * Measurements:
			 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
			 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
			 */

			/* tropospheric properties (0-11km) for standard atmosphere */
			const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
			const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
			const double g  = 9.80665;	/* gravity constant in m/s/s */
			const double R  = 287.05;	/* ideal gas constant in J/kg/K */

			/* current pressure at MSL in kPa */
			const double p1 = _msl_pressure;

			/* measured pressure in kPa */
			const double p = 0.001f * _last_best_baro_pressure;

			/*
			 * Solve:
			 *
			 *     /        -(aR / g)     \
			 *    | (p / p1)          . T1 | - T1
			 *     \                      /
			 * h = -------------------------------  + h1
			 *                   a
			 */
			raw.baro_alt_meter = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

		}
	}
}

bool VotedSensorsUpdate::check_failover(SensorData &sensor, const char *sensor_name)
{
	if (sensor.last_failover_count != sensor.voter.failover_count()) {

		uint32_t flags = sensor.voter.failover_state();

		if (flags == DataValidator::ERROR_FLAG_NO_ERROR) {
			int failover_index = sensor.voter.failover_index();

			if (failover_index != -1) {
				//we switched due to a non-critical reason. No need to panic.
				PX4_INFO("%s sensor switch from #%i", sensor_name, failover_index);
			}

		} else {
			mavlink_log_emergency(&_mavlink_log_pub, "%s #%i fail: %s%s%s%s%s!",
					      sensor_name,
					      sensor.voter.failover_index(),
					      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
					      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
					      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TOUT" : ""),
					      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ECNT" : ""),
					      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " EDNST" : ""));
		}

		sensor.last_failover_count = sensor.voter.failover_count();
		return true;
	}

	return false;
}

bool VotedSensorsUpdate::check_vibration()
{
	bool ret = false;
	hrt_abstime cur_time = hrt_absolute_time();

	if (!_vibration_warning && (_gyro.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold ||
				    _accel.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold ||
				    _mag.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold)) {

		if (_vibration_warning_timestamp == 0) {
			_vibration_warning_timestamp = cur_time;

		} else if (hrt_elapsed_time(&_vibration_warning_timestamp) > 10000 * 1000) {
			_vibration_warning = true;
			mavlink_log_critical(&_mavlink_log_pub, "HIGH VIBRATION! g: %d a: %d m: %d",
					     (int)(100 * _gyro.voter.get_vibration_factor(cur_time)),
					     (int)(100 * _accel.voter.get_vibration_factor(cur_time)),
					     (int)(100 * _mag.voter.get_vibration_factor(cur_time)));
			ret = true;
		}

	} else {
		_vibration_warning_timestamp = 0;
	}

	return ret;
}

void VotedSensorsUpdate::init_sensor_class(const struct orb_metadata *meta, SensorData &sensor_data,
		uint8_t sensor_count_max)
{
	unsigned group_count = orb_group_count(meta);

	if (group_count > sensor_count_max) {
		PX4_WARN("Detected %u %s sensors, but will only use %u", group_count, meta->o_name, sensor_count_max);
		group_count = sensor_count_max;
	}

	for (unsigned i = 0; i < group_count; i++) {
		if (sensor_data.subscription[i] < 0) {
			sensor_data.subscription[i] = orb_subscribe_multi(meta, i);

			if (i > 0) {
				/* the first always exists, but for each further sensor, add a new validator */
				if (!sensor_data.voter.add_new_validator()) {
					PX4_ERR("failed to add validator for sensor %s %i", meta->o_name, i);
				}
			}
		}
	}

	sensor_data.subscription_count = group_count;
}

void VotedSensorsUpdate::print_status()
{
	PX4_INFO("gyro status:");
	_gyro.voter.print();
	PX4_INFO("accel status:");
	_accel.voter.print();
	PX4_INFO("mag status:");
	_mag.voter.print();
	PX4_INFO("baro status:");
	_baro.voter.print();

	_temperature_compensation.print_status();
}

bool
VotedSensorsUpdate::apply_gyro_calibration(DevHandle &h, const struct gyro_calibration_s *gcal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(GYROIOCSSCALE, (long unsigned int)gcal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
VotedSensorsUpdate::apply_accel_calibration(DevHandle &h, const struct accel_calibration_s *acal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(ACCELIOCSSCALE, (long unsigned int)acal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
VotedSensorsUpdate::apply_mag_calibration(DevHandle &h, const struct mag_calibration_s *mcal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(MAGIOCSSCALE, (long unsigned int)mcal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

void VotedSensorsUpdate::sensors_poll(sensor_combined_s &raw)
{
	accel_poll(raw);
	gyro_poll(raw);
	mag_poll(raw);
	baro_poll(raw);

	// publish sensor corrections if necessary
	if (!_hil_enabled && _corrections_changed) {
		_corrections.timestamp = hrt_absolute_time();

		if (_sensor_correction_pub == nullptr) {
			_sensor_correction_pub = orb_advertise(ORB_ID(sensor_correction), &_corrections);

		} else {
			orb_publish(ORB_ID(sensor_correction), _sensor_correction_pub, &_corrections);
		}

		_corrections_changed = false;
	}

	// publish sensor selection if changed
	if (_selection_changed) {
		_selection.timestamp = hrt_absolute_time();

		if (_sensor_selection_pub == nullptr) {
			_sensor_selection_pub = orb_advertise(ORB_ID(sensor_selection), &_selection);

		} else {
			orb_publish(ORB_ID(sensor_selection), _sensor_selection_pub, &_selection);
		}

		_selection_changed = false;
	}
}

void VotedSensorsUpdate::check_failover()
{
	check_failover(_accel, "Accel");
	check_failover(_gyro, "Gyro");
	check_failover(_mag, "Mag");
	check_failover(_baro, "Baro");
}

void VotedSensorsUpdate::set_relative_timestamps(sensor_combined_s &raw)
{
	if (_last_accel_timestamp[_accel.last_best_vote]) {
		raw.accelerometer_timestamp_relative = (int32_t)((int64_t)_last_accel_timestamp[_accel.last_best_vote] -
						       (int64_t)raw.timestamp);
	}

	if (_last_mag_timestamp[_mag.last_best_vote]) {
		raw.magnetometer_timestamp_relative = (int32_t)((int64_t)_last_mag_timestamp[_mag.last_best_vote] -
						      (int64_t)raw.timestamp);
	}

	if (_last_baro_timestamp[_baro.last_best_vote]) {
		raw.baro_timestamp_relative = (int32_t)((int64_t)_last_baro_timestamp[_baro.last_best_vote] - (int64_t)raw.timestamp);
	}
}

void
VotedSensorsUpdate::calc_accel_inconsistency(sensor_preflight_s &preflt)
{
	float accel_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (unsigned sensor_index = 0; sensor_index < _accel.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if ((_accel.priority[sensor_index] > 0) && (sensor_index != _accel.last_best_vote)) {

			float accel_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison agains the primary

			// calculate accel_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_accel_diff[axis_index][check_index] = 0.95f * _accel_diff[axis_index][check_index] + 0.05f *
								       (_last_sensor_data[_accel.last_best_vote].accelerometer_m_s2[axis_index] -
									_last_sensor_data[sensor_index].accelerometer_m_s2[axis_index]);
				accel_diff_sum_sq += _accel_diff[axis_index][check_index] * _accel_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (accel_diff_sum_sq > accel_diff_sum_max_sq) {
				accel_diff_sum_max_sq = accel_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// skip check if less than 2 sensors
	if (check_index < 1) {
		preflt.accel_inconsistency_m_s_s = 0.0f;

	} else {
		// get the vector length of the largest difference and write to the combined sensor struct
		preflt.accel_inconsistency_m_s_s = sqrtf(accel_diff_sum_max_sq);
	}
}

void VotedSensorsUpdate::calc_gyro_inconsistency(sensor_preflight_s &preflt)
{
	float gyro_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (unsigned sensor_index = 0; sensor_index < _gyro.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if ((_gyro.priority[sensor_index] > 0) && (sensor_index != _gyro.last_best_vote)) {

			float gyro_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison against the primary

			// calculate gyro_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_gyro_diff[axis_index][check_index] = 0.95f * _gyro_diff[axis_index][check_index] + 0.05f *
								      (_last_sensor_data[_gyro.last_best_vote].gyro_rad[axis_index] -
								       _last_sensor_data[sensor_index].gyro_rad[axis_index]);
				gyro_diff_sum_sq += _gyro_diff[axis_index][check_index] * _gyro_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (gyro_diff_sum_sq > gyro_diff_sum_max_sq) {
				gyro_diff_sum_max_sq = gyro_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// skip check if less than 2 sensors
	if (check_index < 1) {
		preflt.gyro_inconsistency_rad_s = 0.0f;

	} else {
		// get the vector length of the largest difference and write to the combined sensor struct
		preflt.gyro_inconsistency_rad_s = sqrtf(gyro_diff_sum_max_sq);
	}
}



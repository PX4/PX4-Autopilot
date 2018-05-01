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

#include <systemlib/airspeed.h>
#include <systemlib/mavlink_log.h>

#include <conversion/rotation.h>
#include <ecl/geo/geo.h>

#define MAG_ROT_VAL_INTERNAL		-1
#define CAL_ERROR_APPLY_CAL_MSG "FAILED APPLYING %s CAL #%u"

using namespace sensors;
using namespace DriverFramework;
using namespace time_literals;

VotedSensorsUpdate::VotedSensorsUpdate(const Parameters &parameters, bool hil_enabled)
	: _parameters(parameters), _hil_enabled(hil_enabled)
{
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

	_dpres.voter.set_timeout(300000);
	_dpres.voter.set_equal_value_threshold(100);

	if (_hil_enabled) { // HIL has less accurate timing so increase the timeouts a bit
		_gyro.voter.set_timeout(500000);
		_accel.voter.set_timeout(500000);
	}

	initialize_sensors();

	publish_sensor_corrections();
	publish_sensor_selection();
}

void VotedSensorsUpdate::initialize_sensors()
{
	init_sensor_class(ORB_ID(sensor_gyro), _gyro, GYRO_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_mag), _mag, MAG_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_accel), _accel, ACCEL_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_baro), _baro, BARO_COUNT_MAX);
	init_sensor_class(ORB_ID(differential_pressure), _dpres, DPRES_COUNT_MAX);
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

	for (unsigned i = 0; i < _dpres.subscription_count; i++) {
		orb_unsubscribe(_dpres.subscription[i]);
	}

	orb_unadvertise(_air_data_pub);
	orb_unadvertise(_airspeed_pub);
	orb_unadvertise(_magnetometer_pub);
	orb_unadvertise(_sensor_combined_pub);

	orb_unadvertise(_sensor_correction_pub);
	orb_unadvertise(_sensor_preflight_pub);
	orb_unadvertise(_sensor_selection_pub);
}

void VotedSensorsUpdate::parameters_update()
{
	/* fine tune board offset */
	matrix::Dcmf board_rotation_offset = matrix::Eulerf(
			M_DEG_TO_RAD_F * _parameters.board_offset[0],
			M_DEG_TO_RAD_F * _parameters.board_offset[1],
			M_DEG_TO_RAD_F * _parameters.board_offset[2]);

	_board_rotation = board_rotation_offset * get_rot_matrix((enum Rotation)_parameters.board_rotation);

	/* Load & apply the sensor calibrations.
	 * IMPORTANT: we assume all sensor drivers are running and published sensor data at this point
	 */

	/* temperature compensation */
	_temperature_compensation.parameters_update();

	parameters_update_accel();
	parameters_update_baro();
	parameters_update_gyro();
	parameters_update_mag();
	parameters_update_dpres();
}

void VotedSensorsUpdate::parameters_update_accel()
{
	/* accel */
	for (unsigned topic_instance = 0; topic_instance < ACCEL_COUNT_MAX; ++topic_instance) {

		if (topic_instance < _accel.subscription_count) {
			// valid subscription, so get the driver id by getting the published sensor data
			struct accel_report report;

			if (orb_copy(ORB_ID(sensor_accel), _accel.subscription[topic_instance], &report) == 0) {
				int temp = _temperature_compensation.set_sensor_id_accel(report.device_id, topic_instance);

				if (temp < 0) {
					PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i",
						"accel", report.device_id, topic_instance);
					_corrections.accel_mapping[topic_instance] = 0;

				} else {
					_corrections.accel_mapping[topic_instance] = temp;

				}
			}
		}
	}

	unsigned accel_count = 0;
	unsigned accel_cal_found_count = 0;
	char str[30];
	bool failed = false;

	/* run through all accel sensors */
	for (unsigned driver_index = 0; driver_index < ACCEL_COUNT_MAX; driver_index++) {

		sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, driver_index);

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

			sprintf(str, "CAL_ACC%u_ID", i);
			int32_t device_id = 0;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			sprintf(str, "CAL_ACC%u_EN", i);
			int32_t device_enabled = 1;
			failed = failed || (OK != param_get(param_find(str), &device_enabled));

			_accel.enabled[i] = (device_enabled == 1);

			if (failed) {
				continue;
			}

			if (driver_index == 0 && device_id > 0) {
				accel_cal_found_count++;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == driver_device_id) {
				struct accel_calibration_s ascale = {};
				sprintf(str, "CAL_ACC%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_offset));
				sprintf(str, "CAL_ACC%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_offset));
				sprintf(str, "CAL_ACC%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.z_offset));
				sprintf(str, "CAL_ACC%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_scale));
				sprintf(str, "CAL_ACC%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_scale));
				sprintf(str, "CAL_ACC%u_ZSCALE", i);
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
			int32_t device_id = 0;
			sprintf(str, "CAL_ACC%u_ID", i);
			param_set(param_find(str), &device_id);
		}
	}
}

void VotedSensorsUpdate::parameters_update_baro()
{
	/* baro */
	for (unsigned topic_instance = 0; topic_instance < BARO_COUNT_MAX; ++topic_instance) {

		if (topic_instance < _baro.subscription_count) {
			// valid subscription, so get the driver id by getting the published sensor data
			struct baro_report report;

			if (orb_copy(ORB_ID(sensor_baro), _baro.subscription[topic_instance], &report) == 0) {
				int temp = _temperature_compensation.set_sensor_id_baro(report.device_id, topic_instance);

				if (temp < 0) {
					PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i", "baro", report.device_id,
						topic_instance);
					_corrections.baro_mapping[topic_instance] = 0;

				} else {
					_corrections.baro_mapping[topic_instance] = temp;
				}
			}
		}
	}
}

void VotedSensorsUpdate::parameters_update_dpres()
{
	/* differential pressure */
	// TODO: expand to cover all differential pressure sensors

	char str[30];
	sprintf(str, "%s%u", AIRSPEED_BASE_DEVICE_PATH, 0);

	DevHandle h;
	DevMgr::getHandle(str, h);

	if (!h.isValid()) {
		return;
	}

	airspeed_scale ascale = {
		_parameters.diff_pres_offset_pa,
		1.0f,
	};

	uint32_t device_id = h.ioctl(DEVIOCGDEVICEID, 0);

	/* apply new scaling and offsets */
	bool config_ok = apply_dpres_calibration(h, &ascale, device_id);

	if (!config_ok) {
		PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "differential pressure ", 0);
	}
}

void VotedSensorsUpdate::parameters_update_gyro()
{
	/* gyro */
	for (unsigned topic_instance = 0; topic_instance < GYRO_COUNT_MAX; ++topic_instance) {

		if (topic_instance < _gyro.subscription_count) {
			// valid subscription, so get the driver id by getting the published sensor data
			sensor_gyro_s report;

			if (orb_copy(ORB_ID(sensor_gyro), _gyro.subscription[topic_instance], &report) == 0) {
				int temp = _temperature_compensation.set_sensor_id_gyro(report.device_id, topic_instance);

				if (temp < 0) {
					PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i",
						"gyro", report.device_id, topic_instance);
					_corrections.gyro_mapping[topic_instance] = 0;

				} else {
					_corrections.gyro_mapping[topic_instance] = temp;
				}
			}
		}
	}

	/* set offset parameters to new values */
	bool failed = false;
	char str[30];
	unsigned gyro_count = 0;
	unsigned gyro_cal_found_count = 0;

	/* run through all gyro sensors */
	for (unsigned driver_index = 0; driver_index < GYRO_COUNT_MAX; driver_index++) {

		sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, driver_index);

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

			sprintf(str, "CAL_GYRO%u_ID", i);
			int32_t device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			sprintf(str, "CAL_GYRO%u_EN", i);
			int32_t device_enabled = 1;
			failed = failed || (OK != param_get(param_find(str), &device_enabled));

			_gyro.enabled[i] = (device_enabled == 1);

			if (failed) {
				continue;
			}

			if (driver_index == 0 && device_id > 0) {
				gyro_cal_found_count++;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == driver_device_id) {
				struct gyro_calibration_s gscale = {};
				sprintf(str, "CAL_GYRO%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_offset));
				sprintf(str, "CAL_GYRO%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_offset));
				sprintf(str, "CAL_GYRO%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.z_offset));
				sprintf(str, "CAL_GYRO%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_scale));
				sprintf(str, "CAL_GYRO%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_scale));
				sprintf(str, "CAL_GYRO%u_ZSCALE", i);
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

			int32_t device_id = 0;
			sprintf(str, "CAL_GYRO%u_ID", i);
			param_set(param_find(str), &device_id);
		}
	}
}

void VotedSensorsUpdate::parameters_update_mag()
{
	// initialze all mag rotations with the board rotation in case there is no calibration data available
	for (int topic_instance = 0; topic_instance < MAG_COUNT_MAX; ++topic_instance) {
		_mag_rotation[topic_instance] = _board_rotation;
	}

	bool failed = false;
	char str[30];

	/* run through all mag sensors
	 * Because we store the device id in _mag_device_id, we need to get the id via uorb topic since
	 * the DevHandle method does not work on POSIX.
	 */
	for (unsigned topic_instance = 0; topic_instance < MAG_COUNT_MAX
	     && topic_instance < _mag.subscription_count; ++topic_instance) {

		sensor_mag_s report;

		if (orb_copy(ORB_ID(sensor_mag), _mag.subscription[topic_instance], &report) != 0) {
			continue;
		}

		int topic_device_id = report.device_id;
		bool is_external = report.is_external;
		_mag.device_id[topic_instance] = topic_device_id;

		// find the driver handle that matches the topic_device_id
		DevHandle h;

		for (unsigned driver_index = 0; driver_index < MAG_COUNT_MAX; ++driver_index) {

			sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, driver_index);

			DevMgr::getHandle(str, h);

			if (!h.isValid()) {
				/* the driver is not running, continue with the next */
				continue;
			}

			int driver_device_id = h.ioctl(DEVIOCGDEVICEID, 0);

			if (driver_device_id == topic_device_id) {
				break; // we found the matching driver

			} else {
				DevMgr::releaseHandle(h);
			}
		}

		bool config_ok = false;

		/* run through all stored calibrations */
		for (unsigned i = 0; i < MAG_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			sprintf(str, "CAL_MAG%u_ID", i);
			int32_t device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			sprintf(str, "CAL_MAG%u_EN", i);
			int32_t device_enabled = 1;
			failed = failed || (OK != param_get(param_find(str), &device_enabled));

			_mag.enabled[i] = (device_enabled == 1);

			if (failed) {
				continue;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == _mag.device_id[topic_instance]) {
				struct mag_calibration_s mscale = {};
				sprintf(str, "CAL_MAG%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_offset));
				sprintf(str, "CAL_MAG%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_offset));
				sprintf(str, "CAL_MAG%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_offset));
				sprintf(str, "CAL_MAG%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_scale));
				sprintf(str, "CAL_MAG%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_scale));
				sprintf(str, "CAL_MAG%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_scale));

				sprintf(str, "CAL_MAG%u_ROT", i);

				int32_t mag_rot = 0;
				param_get(param_find(str), &mag_rot);

				if (is_external) {

					/* check if this mag is still set as internal, otherwise leave untouched */
					if (mag_rot < 0) {
						/* it was marked as internal, change to external with no rotation */
						mag_rot = 0;
						param_set_no_notification(param_find(str), &mag_rot);
					}

				} else {
					/* mag is internal - reset param to -1 to indicate internal mag */
					if (mag_rot != MAG_ROT_VAL_INTERNAL) {
						mag_rot = MAG_ROT_VAL_INTERNAL;
						param_set_no_notification(param_find(str), &mag_rot);
					}
				}

				/* now get the mag rotation */
				if (mag_rot >= 0) {
					// Set external magnetometers to use the parameter value
					_mag_rotation[topic_instance] = get_rot_matrix((enum Rotation)mag_rot);

				} else {
					// Set internal magnetometers to use the board rotation
					_mag_rotation[topic_instance] = _board_rotation;
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
	}
}

bool VotedSensorsUpdate::accel_poll()
{
	float *offsets[] = {_corrections.accel_offset_0, _corrections.accel_offset_1, _corrections.accel_offset_2 };
	float *scales[] = {_corrections.accel_scale_0, _corrections.accel_scale_1, _corrections.accel_scale_2 };

	for (unsigned uorb_index = 0; uorb_index < _accel.subscription_count; uorb_index++) {
		bool accel_updated = false;
		orb_check(_accel.subscription[uorb_index], &accel_updated);

		if (accel_updated && _accel.enabled[uorb_index]) {
			sensor_accel_s accel_report;

			int ret = orb_copy(ORB_ID(sensor_accel), _accel.subscription[uorb_index], &accel_report);

			if (ret != PX4_OK || accel_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_accel.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_accel.subscription[uorb_index], &priority);
				_accel.priority[uorb_index] = (uint8_t)priority;
			}

			_accel.device_id[uorb_index] = accel_report.device_id;

			matrix::Vector3f accel_data;

			if (accel_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * because it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta velocities to an equivalent acceleration before application of corrections
				float dt_inv = 1.e6f / accel_report.integral_dt;
				accel_data = matrix::Vector3f(accel_report.x_integral * dt_inv,
							      accel_report.y_integral * dt_inv,
							      accel_report.z_integral * dt_inv);

				_last_accel_dt[uorb_index] = accel_report.integral_dt;

			} else {
				// using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				accel_data = matrix::Vector3f(accel_report.x, accel_report.y, accel_report.z);

				// handle the case where this is our first output
				if (_accel.timestamp_last[uorb_index] == 0) {
					_accel.timestamp_last[uorb_index] = accel_report.timestamp - 1000;
				}

				// approximate the  delta time using the difference in accel data time stamps
				_last_accel_dt[uorb_index] = (accel_report.timestamp - _accel.timestamp_last[uorb_index]);
			}

			// handle temperature compensation
			if (!_hil_enabled) {
				if (_temperature_compensation.apply_corrections_accel(uorb_index, accel_data, accel_report.temperature,
						offsets[uorb_index], scales[uorb_index]) == 2) {

					publish_sensor_corrections();
				}
			}

			_accel.timestamp_last[uorb_index] = accel_report.timestamp;

			// rotate corrected measurements from sensor to body frame
			accel_data = _board_rotation * accel_data;

			_accel.voter.put(uorb_index, accel_report.timestamp, accel_data.data(), accel_report.error_count,
					 _accel.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index = -1;
	float *best_accel = _accel.voter.get_best(hrt_absolute_time(), &best_index);

	// write the best sensor data to the output variables
	if ((best_index >= 0) && (best_accel != nullptr)) {
		_imu.accelerometer_integral_dt = _last_accel_dt[best_index];
		_imu.accelerometer_m_s2[0] = best_accel[0];
		_imu.accelerometer_m_s2[1] = best_accel[1];
		_imu.accelerometer_m_s2[2] = best_accel[2];

		if (best_index != _accel.last_best_vote) {
			_accel.last_best_vote = (uint8_t)best_index;
			_corrections.selected_accel_instance = (uint8_t)best_index;
			publish_sensor_corrections();
		}

		if (_selection.accel_device_id != _accel.device_id[best_index]) {
			_selection.accel_device_id = _accel.device_id[best_index];
			publish_sensor_selection();
		}

		return true;
	}

	return false;
}

bool VotedSensorsUpdate::gyro_poll()
{
	float *offsets[] = {_corrections.gyro_offset_0, _corrections.gyro_offset_1, _corrections.gyro_offset_2 };
	float *scales[] = {_corrections.gyro_scale_0, _corrections.gyro_scale_1, _corrections.gyro_scale_2 };

	for (unsigned uorb_index = 0; uorb_index < _gyro.subscription_count; uorb_index++) {
		bool gyro_updated = false;
		orb_check(_gyro.subscription[uorb_index], &gyro_updated);

		if (gyro_updated && _gyro.enabled[uorb_index]) {
			struct gyro_report gyro_report;

			int ret = orb_copy(ORB_ID(sensor_gyro), _gyro.subscription[uorb_index], &gyro_report);

			if (ret != PX4_OK || gyro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_gyro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_gyro.subscription[uorb_index], &priority);
				_gyro.priority[uorb_index] = (uint8_t)priority;
			}

			_gyro.device_id[uorb_index] = gyro_report.device_id;

			matrix::Vector3f gyro_rate;

			if (gyro_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * becasue it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta angles to an equivalent angular rate before application of corrections
				float dt_inv = 1.e6f / gyro_report.integral_dt;
				gyro_rate = matrix::Vector3f(
						    gyro_report.x_integral * dt_inv,
						    gyro_report.y_integral * dt_inv,
						    gyro_report.z_integral * dt_inv);

				_last_gyro_dt[uorb_index] = gyro_report.integral_dt;

			} else {
				//using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				gyro_rate = matrix::Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);

				// handle the case where this is our first output
				if (_gyro.timestamp_last[uorb_index] == 0) {
					_gyro.timestamp_last[uorb_index] = gyro_report.timestamp - 1000;
				}

				// approximate the delta time using the difference in gyro data time stamps
				_last_gyro_dt[uorb_index] = (gyro_report.timestamp - _gyro.timestamp_last[uorb_index]);
			}

			// handle temperature compensation
			if (!_hil_enabled) {
				if (_temperature_compensation.apply_corrections_gyro(uorb_index, gyro_rate, gyro_report.temperature,
						offsets[uorb_index], scales[uorb_index]) == 2) {
					publish_sensor_corrections();
				}
			}

			_gyro.timestamp_last[uorb_index] = gyro_report.timestamp;

			// rotate corrected measurements from sensor to body frame
			gyro_rate = _board_rotation * gyro_rate;

			_gyro.voter.put(uorb_index, gyro_report.timestamp, gyro_rate.data(), gyro_report.error_count,
					_gyro.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index = -1;
	float *best_gyro = _gyro.voter.get_best(hrt_absolute_time(), &best_index);

	// write data for the best sensor to output variables
	if ((best_index >= 0) && (best_gyro != nullptr)) {
		_imu.timestamp = _gyro.timestamp_last[best_index];
		_imu.gyro_integral_dt = _last_gyro_dt[best_index];
		_imu.gyro_rad[0] = best_gyro[0];
		_imu.gyro_rad[1] = best_gyro[1];
		_imu.gyro_rad[2] = best_gyro[2];

		if (_gyro.last_best_vote != best_index) {
			_gyro.last_best_vote = (uint8_t)best_index;
			_corrections.selected_gyro_instance = (uint8_t)best_index;

			publish_sensor_corrections();
		}

		if (_selection.gyro_device_id != _gyro.device_id[best_index]) {
			_selection.gyro_device_id = _gyro.device_id[best_index];

			publish_sensor_selection();
		}

		return true;
	}

	return false;
}

void VotedSensorsUpdate::mag_poll()
{
	bool updated = false;

	for (unsigned uorb_index = 0; uorb_index < _mag.subscription_count; uorb_index++) {
		bool mag_updated;
		orb_check(_mag.subscription[uorb_index], &mag_updated);

		if (mag_updated && _mag.enabled[uorb_index]) {
			struct mag_report mag_report;

			int ret = orb_copy(ORB_ID(sensor_mag), _mag.subscription[uorb_index], &mag_report);

			if (ret != PX4_OK || mag_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_mag.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_mag.subscription[uorb_index], &priority);
				_mag.priority[uorb_index] = (uint8_t)priority;
			}

			_mag.timestamp_last[uorb_index] = mag_report.timestamp;

			matrix::Vector3f vect{_mag_rotation[uorb_index] *matrix::Vector3f{mag_report.x, mag_report.y, mag_report.z}};

			_mag.voter.put(uorb_index, mag_report.timestamp, vect.data(), mag_report.error_count, _mag.priority[uorb_index]);

			updated = true;
		}
	}

	if (updated) {
		int best_index = -1;
		float *best_mag = _mag.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0 && (best_mag != nullptr)) {

			vehicle_magnetometer_s magnetometer;

			magnetometer.timestamp = _mag.timestamp_last[best_index];
			magnetometer.magnetometer_ga[0] = best_mag[0];
			magnetometer.magnetometer_ga[1] = best_mag[1];
			magnetometer.magnetometer_ga[2] = best_mag[2];

			_mag.last_best_vote = (uint8_t)best_index;

			if (_selection.mag_device_id != _mag.device_id[best_index]) {
				_selection.mag_device_id = _mag.device_id[best_index];

				publish_sensor_selection();
			}

			int instance;
			orb_publish_auto(ORB_ID(vehicle_magnetometer), &_magnetometer_pub, &magnetometer, &instance, ORB_PRIO_HIGH);
		}
	}
}

bool VotedSensorsUpdate::baro_poll()
{
	bool updated = false;
	float *offsets[] = {&_corrections.baro_offset_0, &_corrections.baro_offset_1, &_corrections.baro_offset_2 };
	float *scales[] = {&_corrections.baro_scale_0, &_corrections.baro_scale_1, &_corrections.baro_scale_2 };

	for (unsigned uorb_index = 0; uorb_index < _baro.subscription_count; uorb_index++) {
		bool baro_updated;
		orb_check(_baro.subscription[uorb_index], &baro_updated);

		if (baro_updated) {
			struct baro_report baro_report;

			int ret = orb_copy(ORB_ID(sensor_baro), _baro.subscription[uorb_index], &baro_report);

			if (ret != PX4_OK || baro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// Convert from millibar to Pa
			float corrected_pressure = 100.0f * baro_report.pressure;

			// handle temperature compensation
			if (!_hil_enabled) {
				if (_temperature_compensation.apply_corrections_baro(uorb_index, corrected_pressure, baro_report.temperature,
						offsets[uorb_index], scales[uorb_index]) == 2) {

					publish_sensor_corrections();
				}
			}

			// First publication with data
			if (_baro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_baro.subscription[uorb_index], &priority);
				_baro.priority[uorb_index] = (uint8_t)priority;
			}

			_baro.device_id[uorb_index] = baro_report.device_id;

			matrix::Vector3f vect{corrected_pressure, 0.0f, baro_report.temperature};

			_baro.timestamp_last[uorb_index] = baro_report.timestamp;

			_baro.voter.put(uorb_index, baro_report.timestamp, vect.data(), baro_report.error_count, _baro.priority[uorb_index]);

			updated = true;
		}
	}

	if (updated) {
		int best_index = -1;
		float *best_baro = _baro.voter.get_best(hrt_absolute_time(), &best_index);

		if ((best_index >= 0) && (best_baro != nullptr)) {
			_air_data.timestamp = _baro.timestamp_last[best_index];
			_air_data.baro_temp_celcius = best_baro[2];
			_air_data.baro_pressure_pa = best_baro[0];

			if (_baro.last_best_vote != best_index) {

				_baro.last_best_vote = (uint8_t)best_index;
				_corrections.selected_baro_instance = (uint8_t)best_index;

				publish_sensor_corrections();
			}

			if (_selection.baro_device_id != _baro.device_id[best_index]) {
				_selection.baro_device_id = _baro.device_id[best_index];

				publish_sensor_selection();
			}

			// calculate altitude using the hypsometric equation
			static constexpr float T1 = 15.0f - CONSTANTS_ABSOLUTE_NULL_CELSIUS;	/* temperature at base height in Kelvin */
			static constexpr float a  = -6.5f / 1000.0f;	/* temperature gradient in degrees per metre */

			/* current pressure at MSL in kPa (QNH in hPa)*/
			const float p1 = _parameters.baro_qnh * 0.1f;

			/* measured pressure in kPa */
			const float p = _air_data.baro_pressure_pa * 0.001f;

			/*
			 * Solve:
			 *
			 *     /        -(aR / g)     \
			 *    | (p / p1)          . T1 | - T1
			 *     \                      /
			 * h = -------------------------------  + h1
			 *                   a
			 */
			_air_data.baro_alt_meter = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;

			bool airspeed_valid = (hrt_elapsed_time(&_airspeed.timestamp) < 1_s);
			bool airspeed_temp_valid = (PX4_ISFINITE(_airspeed.air_temperature_celsius)
						    && (_airspeed.air_temperature_celsius > CONSTANTS_ABSOLUTE_NULL_CELSIUS));

			if (airspeed_valid && airspeed_temp_valid) {
				// use temperature from airspeed
				_air_data.temperature = _airspeed.air_temperature_celsius;

			} else {
				_air_data.temperature = NAN;
			}

			_air_data.air_density = get_air_density(_air_data.baro_pressure_pa, _air_data.temperature);

			const bool alt_valid = PX4_ISFINITE(_air_data.baro_alt_meter);
			const bool pressure_valid = PX4_ISFINITE(_air_data.baro_pressure_pa);
			const bool rho_valid = PX4_ISFINITE(_air_data.air_density);

			if (alt_valid && pressure_valid && rho_valid) {
				int instance;
				orb_publish_auto(ORB_ID(vehicle_air_data), &_air_data_pub, &_air_data, &instance, ORB_PRIO_DEFAULT);
			}

			return true;
		}
	}

	return false;
}

bool VotedSensorsUpdate::diff_pres_poll()
{
	bool updated = false;

	for (unsigned uorb_index = 0; uorb_index < _dpres.subscription_count; uorb_index++) {
		bool dpres_updated = false;
		orb_check(_dpres.subscription[uorb_index], &dpres_updated);

		if (dpres_updated) {
			differential_pressure_s diff_press;

			int ret = orb_copy(ORB_ID(differential_pressure), _dpres.subscription[uorb_index], &diff_press);

			if (ret != PX4_OK || diff_press.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_dpres.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_dpres.subscription[uorb_index], &priority);
				_dpres.priority[uorb_index] = (uint8_t)priority;
			}

			_dpres.device_id[uorb_index] = diff_press.device_id;

			matrix::Vector3f vect{diff_press.differential_pressure_filtered_pa, diff_press.differential_pressure_raw_pa, diff_press.temperature};

			_dpres.timestamp_last[uorb_index] = diff_press.timestamp;

			_dpres.voter.put(uorb_index, diff_press.timestamp, vect.data(), diff_press.error_count, _dpres.priority[uorb_index]);

			updated = true;
		}
	}

	if (updated) {
		int best_index = -1;
		float *best_dpres = _dpres.voter.get_best(hrt_absolute_time(), &best_index);

		if ((best_index >= 0) && (best_dpres != nullptr)) {

			const float dpres = best_dpres[0];
			const float temperature = best_dpres[2];

			const float static_pressure = _air_data.baro_pressure_pa;

			if (PX4_ISFINITE(temperature) && (temperature > CONSTANTS_ABSOLUTE_NULL_CELSIUS)) {
				_airspeed.air_temperature_celsius = temperature;

			} else {
				_airspeed.air_temperature_celsius = NAN;
			}

			switch ((_dpres.device_id[best_index] >> 16) & 0xFF) {
			case DRV_DIFF_PRESS_DEVTYPE_SDP31:
			case DRV_DIFF_PRESS_DEVTYPE_SDP32:
			case DRV_DIFF_PRESS_DEVTYPE_SDP33:
				_airspeed.indicated_airspeed_m_s = calc_indicated_airspeed_corrected(
						(enum AIRSPEED_COMPENSATION_MODEL) _parameters.air_cmodel, AIRSPEED_SENSOR_MODEL_SDP3X,
						_parameters.air_tube_length, _parameters.air_tube_diameter_mm,
						dpres, static_pressure, _airspeed.air_temperature_celsius);
				break;

			default:
				_airspeed.indicated_airspeed_m_s = calc_indicated_airspeed(dpres,
								   get_air_density(static_pressure, _airspeed.air_temperature_celsius));
				break;
			}

			_airspeed.true_airspeed_m_s = calc_true_airspeed_from_indicated(_airspeed.indicated_airspeed_m_s, static_pressure,
						      _airspeed.air_temperature_celsius);
			_airspeed.timestamp = _dpres.timestamp_last[best_index];

			DataValidator *best_validator = _dpres.voter.get_last_best();

			if (best_validator != nullptr) {
				_airspeed.confidence = best_validator->confidence(hrt_absolute_time());
			}

			if (_airspeed.indicated_airspeed_m_s < 0.0f) {
				_airspeed.indicated_airspeed_m_s = 0.0f;
			}

			if (_airspeed.true_airspeed_m_s < 0.0f) {
				_airspeed.true_airspeed_m_s = 0.0f;
			}

			// only publish finite airspeed
			if (PX4_ISFINITE(_airspeed.indicated_airspeed_m_s) && PX4_ISFINITE(_airspeed.true_airspeed_m_s)) {
				int instance;
				orb_publish_auto(ORB_ID(airspeed), &_airspeed_pub, &_airspeed, &instance, ORB_PRIO_DEFAULT);
			}

			return true;
		}
	}

	return false;
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
			int failover_index = sensor.voter.failover_index();

			if (failover_index != -1) {
				mavlink_log_emergency(&_mavlink_log_pub, "%s #%i fail: %s%s%s%s%s!",
						      sensor_name,
						      failover_index,
						      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
						      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
						      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
						      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
						      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));

				// reduce priority of failed sensor to the minimum
				sensor.priority[failover_index] = 1;
			}
		}

		sensor.last_failover_count = sensor.voter.failover_count();
		return true;
	}

	return false;
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

	PX4_INFO("diff press status:");
	_dpres.voter.print();

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
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX)

	if (!h.isValid()) {
		return false;
	}

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(MAGIOCSSCALE, (long unsigned int)mcal);

#else
	/* On QURT & POSIX, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
VotedSensorsUpdate::apply_dpres_calibration(DevHandle &h, const struct airspeed_scale *acal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX)

	if (!h.isValid()) {
		return false;
	}

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(AIRSPEEDIOCSSCALE, (long unsigned int)acal);

#else
	/* On QURT & POSIX, the params are read directly in the respective wrappers. */
	return true;
#endif
}

void VotedSensorsUpdate::update_imu()
{
	mag_poll();

	bool accel_updated = accel_poll();
	bool gyro_updated = gyro_poll();

	_imu.accelerometer_timestamp_relative = (int32_t)((int64_t)_accel.timestamp_last[_accel.last_best_vote] -
						(int64_t)_imu.timestamp);

	if (accel_updated || gyro_updated) {
		int instance;
		orb_publish_auto(ORB_ID(sensor_combined), &_sensor_combined_pub, &_imu, &instance, ORB_PRIO_HIGH);
	}
}

void VotedSensorsUpdate::update_air_data()
{
	baro_poll();
	diff_pres_poll();
}

void VotedSensorsUpdate::check_failover()
{
	check_failover(_accel, "Accel");
	check_failover(_gyro, "Gyro");
	check_failover(_mag, "Mag");
	check_failover(_baro, "Baro");
	check_failover(_dpres, "Diff Pressure");
}

void
VotedSensorsUpdate::publish_sensor_preflight()
{
	sensor_preflight_s preflt = {};

	preflt.accel_inconsistency_m_s_s = _accel.voter.calc_inconsistency();
	preflt.gyro_inconsistency_rad_s = _gyro.voter.calc_inconsistency();
	preflt.mag_inconsistency_ga = _mag.voter.calc_inconsistency();

	preflt.timestamp = hrt_absolute_time();

	int instance;
	orb_publish_auto(ORB_ID(sensor_preflight), &_sensor_preflight_pub, &preflt, &instance, ORB_PRIO_LOW);
}

void
VotedSensorsUpdate::publish_sensor_selection()
{
	_selection.timestamp = hrt_absolute_time();

	int instance;
	orb_publish_auto(ORB_ID(sensor_selection), &_sensor_selection_pub, &_selection, &instance, ORB_PRIO_DEFAULT);
}

void
VotedSensorsUpdate::publish_sensor_corrections()
{
	// publish sensor corrections if necessary
	if (!_hil_enabled) {
		_corrections.timestamp = hrt_absolute_time();

		int instance;
		orb_publish_auto(ORB_ID(sensor_correction), &_sensor_correction_pub, &_corrections, &instance, ORB_PRIO_DEFAULT);
	}
}

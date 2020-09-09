/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem("ekf2_selector", px4::wq_configurations::nav_and_controllers)
{
}

EKF2Selector::~EKF2Selector()
{
	Stop();
}

bool EKF2Selector::Start()
{
	ScheduleNow();
	return true;
}

void EKF2Selector::Stop()
{
	for (int i = 0; i < MAX_INSTANCES; i++) {
		_instance[i].estimator_attitude_sub.unregisterCallback();
	}

	ScheduleClear();
}

bool EKF2Selector::SelectInstance(uint8_t ekf_instance, bool force_reselect)
{
	if (ekf_instance != _selected_instance || force_reselect) {
		if (_selected_instance != UINT8_MAX) {
			// switch callback registration
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();

			PX4_WARN("primary EKF changed %d -> %d", _selected_instance, ekf_instance);
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback();

		_selected_instance = ekf_instance;
		_instance_changed_count++;
		_last_instance_change = hrt_absolute_time();
		_instance[_selected_instance].time_last_selected = _last_instance_change;

		for (auto &inst : _instance) {
			inst.relative_test_ratio = 0;
		}

		// handle resets on change

		// vehicle_attitude: quat_reset_counter
		vehicle_attitude_s attitude_new;

		if (_instance[_selected_instance].estimator_attitude_sub.copy(&attitude_new)) {
			++_quat_reset_counter;
			_delta_q_reset = Quatf{attitude_new.q} - Quatf{_attitude_last.q};

			// save new estimator_attitude
			_attitude_last = attitude_new;

			attitude_new.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude_new.delta_q_reset);

			// publish new vehicle_attitude immediately
			_vehicle_attitude_pub.publish(attitude_new);
		}

		// vehicle_local_position: xy_reset_counter, z_reset_counter, vxy_reset_counter, vz_reset_counter
		vehicle_local_position_s local_position_new;

		if (_instance[_selected_instance].estimator_local_position_sub.copy(&local_position_new)) {

			// update sensor_selection immediately
			{
				sensor_selection_s sensor_selection{};
				sensor_selection.accel_device_id = _instance[_selected_instance].estimator_status.accel_device_id;
				sensor_selection.gyro_device_id = _instance[_selected_instance].estimator_status.gyro_device_id;
				sensor_selection.timestamp = hrt_absolute_time();
				_sensor_selection_pub.publish(sensor_selection);
			}

			++_xy_reset_counter;
			++_z_reset_counter;
			++_vxy_reset_counter;
			++_vz_reset_counter;
			++_heading_reset_counter;

			_delta_xy = Vector2f{local_position_new.x, local_position_new.y} - Vector2f{_local_position_last.x, _local_position_last.y};
			_delta_z = local_position_new.z - _local_position_last.z;
			_delta_vxy = Vector2f{local_position_new.vx, local_position_new.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy};
			_delta_vz = local_position_new.vz - _local_position_last.vz;
			_delta_heading = matrix::wrap_2pi(local_position_new.heading - _local_position_last.heading);

			// save new estimator_local_position
			_local_position_last = local_position_new;

			local_position_new.xy_reset_counter = _xy_reset_counter;
			local_position_new.z_reset_counter = _z_reset_counter;
			local_position_new.vxy_reset_counter = _vxy_reset_counter;
			local_position_new.vz_reset_counter = _vz_reset_counter;
			_delta_xy.copyTo(local_position_new.delta_xy);
			_delta_vxy.copyTo(local_position_new.delta_vxy);
			local_position_new.delta_z = _delta_z;
			local_position_new.delta_vz = _delta_vz;
			local_position_new.delta_heading = _delta_heading;

			// publish new vehicle_local_position immediately
			_vehicle_local_position_pub.publish(local_position_new);
		}

		// vehicle_global_position: lat_lon_reset_counter, alt_reset_counter
		vehicle_global_position_s global_position_new;

		if (_instance[_selected_instance].estimator_global_position_sub.copy(&global_position_new)) {
			++_alt_reset_counter;
			++_lat_lon_reset_counter;

			_delta_lat = global_position_new.lat - _global_position_last.lat;
			_delta_lon = global_position_new.lon - _global_position_last.lon;
			_delta_alt = global_position_new.delta_alt - _global_position_last.delta_alt;

			// save new estimator_global_position
			_global_position_last = global_position_new;

			global_position_new.alt_reset_counter = _alt_reset_counter;
			global_position_new.delta_alt = _delta_alt;

			// publish new vehicle_global_position immediately
			_vehicle_global_position_pub.publish(_global_position_last);
		}

		return true;
	}

	return false;
}

void EKF2Selector::updateErrorScores()
{
	bool primary_updated = false;

	// calculate individual error scores
	for (uint8_t i = 0; i < MAX_INSTANCES; i++) {
		if (_instance[i].estimator_status_sub.update(&_instance[i].estimator_status)) {
			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
			}

			if (i == _selected_instance) {
				primary_updated = true;
			}

			const estimator_status_s &status = _instance[i].estimator_status;

			const bool tilt_align = status.control_mode_flags & (1 << estimator_status_s::CS_TILT_ALIGN);
			const bool yaw_align = status.control_mode_flags & (1 << estimator_status_s::CS_YAW_ALIGN);

			_instance[i].combined_test_ratio = 0.0f;

			if (tilt_align && yaw_align) {
				_instance[i].combined_test_ratio = fmaxf(_instance[i].combined_test_ratio,
								   0.5f * (status.vel_test_ratio + status.pos_test_ratio));
				_instance[i].combined_test_ratio = fmaxf(_instance[i].combined_test_ratio, status.hgt_test_ratio);
			}

			_instance[i].healthy = tilt_align && yaw_align && (status.filter_fault_flags == 0);

		} else if (hrt_elapsed_time(&_instance[i].estimator_status.timestamp) > 20_ms) {
			_instance[i].healthy = false;
		}
	}

	// update relative test ratios if primary has updated
	if (primary_updated) {
		for (uint8_t i = 0; i < _available_instances; i++) {
			if (i != _selected_instance) {

				const float error_delta = _instance[i].combined_test_ratio - _instance[_selected_instance].combined_test_ratio;

				// reduce error only if its better than the primary instance by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes
				const float threshold = _gyro_fault_detected ? fmaxf(_param_ekf2_sel_err_red.get(), 0.05f) : 0.0f;
				if (error_delta > 0 || error_delta < -threshold) {
					_instance[i].relative_test_ratio += error_delta;
					_instance[i].relative_test_ratio = math::constrain(_instance[i].relative_test_ratio, -_rel_err_score_lim,
									   _rel_err_score_lim);
				}
			}
		}
	}
}

void EKF2Selector::Run()
{
	// re-schedule as backup timeout
	ScheduleDelayed(10_ms);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	_sensors_status_imu.update();
	const sensors_status_imu_s &sensors_status_imu = _sensors_status_imu.get();

	_gyro_fault_detected = false;
	uint32_t faulty_gyro_id = 0;
	if (_selected_instance != UINT8_MAX) {
		static constexpr uint8_t IMU_STATUS_SIZE = (sizeof(sensors_status_imu.gyro_inconsistency_rad_s) / sizeof(
					sensors_status_imu.gyro_inconsistency_rad_s[0]));
		static constexpr uint8_t IMU_INDEX_LIMIT = math::min(IMU_STATUS_SIZE, MAX_INSTANCES);
		const float angle_rate_threshold = math::radians(_param_ekf2_sel_gyr_rate.get());
		const float angle_threshold = math::radians(_param_ekf2_sel_gyr_angle.get());
		const float time_step_s = fminf(1E-6f * (float)hrt_elapsed_time(&_last_update_us), 1.0f);
		if (sensors_status_imu.gyro_device_id_primary == _instance[_selected_instance].estimator_status.gyro_device_id) {
			// accumulate excess gyro error
			uint8_t n_gyros = 0;
			uint8_t n_exceedances = 0;
			for (unsigned i = 0; i < IMU_INDEX_LIMIT; i++) {
				if (sensors_status_imu.gyro_device_ids[i] != 0) {
					n_gyros++;
					if (sensors_status_imu.gyro_device_ids[i] == sensors_status_imu.gyro_device_id_primary) {
						_accumulated_gyro_error[i] = 0.0f;
					} else {
						_accumulated_gyro_error[i] += (sensors_status_imu.gyro_inconsistency_rad_s[i] - angle_rate_threshold) * time_step_s;
						_accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.0f);
					}
				} else {
					// no sensor
					_accumulated_gyro_error[i] = 0.0f;
				}
				if (_accumulated_gyro_error[i] > angle_threshold) {
					n_exceedances++;
				}
			}
			if (n_exceedances > 0) {
				if (n_gyros >= 3) {
					// If there are 3 or more gyros, see if we can work out which one is faulty
					_gyro_fault_detected = true;
					// if all sensors other than the primary have an elevated difference, then the primary is faulty
					// because all differences are measured relative to the primary
					if (n_exceedances == n_gyros - 1) {
						faulty_gyro_id = sensors_status_imu.gyro_device_id_primary;
					}
				} else if (n_gyros == 2) {
					_gyro_fault_detected = true;
				}
			}
		}
	}


	// update combined test ratio for all estimators
	updateErrorScores();

	bool lower_error_available = false;
	float alternative_error = 0.f; // looking for instances that have error lower than the current primary
	uint8_t best_ekf_instance = _selected_instance;

	// loop through all available instances to find if an alternative is available
	for (int i = 0; i < _available_instances; i++) {
		// if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_gyro_fault_detected && _instance[i].estimator_status.gyro_device_id == faulty_gyro_id) {
			_instance[i].healthy = false;
		}

		// Use an alternative instance if  -
		// (healthy and has updated recently)
		// AND
		// (has relative error less than selected instance and has not been the selected instance for at least 10 seconds
		// OR
		// selected instance has stopped updating
		if (_instance[i].healthy && (i != _selected_instance)) {
			const float relative_error = _instance[i].relative_test_ratio;

			if (relative_error < alternative_error) {

				alternative_error = relative_error;
				best_ekf_instance = i;

				// relative error less than selected instance and has not been the selected instance for at least 10 seconds
				if ((relative_error <= -_rel_err_thresh) && hrt_elapsed_time(&_instance[i].time_last_selected) > 10_s) {
					lower_error_available = true;
				}
			}
		}
	}

	if ((_selected_instance == UINT8_MAX)
	    || (!_instance[_selected_instance].healthy && (best_ekf_instance == _selected_instance))) {

		// force initial selection
		uint8_t best_instance = _selected_instance;
		float best_test_ratio = FLT_MAX;

		for (int i = 0; i < _available_instances; i++) {
			if (_instance[i].healthy) {
				const float test_ratio = _instance[i].combined_test_ratio;

				if ((test_ratio > 0) && (test_ratio < best_test_ratio)) {
					best_instance = i;
					best_test_ratio = test_ratio;
				}
			}
		}

		SelectInstance(best_instance);

	} else if (best_ekf_instance != _selected_instance) {
		//  if this instance has a significantly lower relative error to the active primary, we consider it as a
		// better instance and would like to switch to it even if the current primary is healthy
		//  switch immediately if the current selected is no longer healthy
		if ((lower_error_available && hrt_elapsed_time(&_last_instance_change) > 10_s)
		    || !_instance[_selected_instance].healthy) {

			SelectInstance(best_ekf_instance);
		}
	}

	if (_selected_instance != UINT8_MAX) {

		// vehicle_attitude
		vehicle_attitude_s attitude;

		if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {

			if (attitude.quat_reset_counter > _attitude_last.quat_reset_counter) {
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};
			}

			// save latest estimator_attitude
			_attitude_last = attitude;

			// update with total estimator resets
			attitude.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude.delta_q_reset);

			_vehicle_attitude_pub.publish(attitude);
		}

		// vehicle_local_position
		vehicle_local_position_s local_position;

		if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) {

			// XY reset
			if (local_position.xy_reset_counter > _local_position_last.xy_reset_counter) {
				++_xy_reset_counter;
				_delta_xy = Vector2f{local_position.delta_xy};
			}

			// Z reset
			if (local_position.z_reset_counter > _local_position_last.z_reset_counter) {
				++_z_reset_counter;
				_delta_z = local_position.delta_z;
			}

			// VXY reset
			if (local_position.vxy_reset_counter > _local_position_last.vxy_reset_counter) {
				++_vxy_reset_counter;
				_delta_vxy = Vector2f{local_position.delta_vxy};
			}

			// VZ reset
			if (local_position.vz_reset_counter > _local_position_last.vz_reset_counter) {
				++_vz_reset_counter;
				_delta_z = local_position.delta_vz;
			}

			// heading reset
			if (local_position.heading_reset_counter > _local_position_last.heading_reset_counter) {
				++_heading_reset_counter;
				_delta_heading = local_position.delta_heading;
			}

			// save new estimator_local_position
			_local_position_last = local_position;

			// update with total estimator resets
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			local_position.heading_reset_counter = _heading_reset_counter;

			_delta_xy.copyTo(local_position.delta_xy);
			_delta_vxy.copyTo(local_position.delta_vxy);
			local_position.delta_z = _delta_z;
			local_position.delta_vz = _delta_vz;
			local_position.delta_heading = _delta_heading;

			_vehicle_local_position_pub.publish(local_position);
		}

		// vehicle_global_position
		vehicle_global_position_s global_position;

		if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) {

			// Z reset
			if (global_position.alt_reset_counter > _global_position_last.alt_reset_counter) {
				++_alt_reset_counter;
				_delta_alt = global_position.delta_alt;
			}

			// save new estimator_global_position
			_global_position_last = global_position;

			// update with total estimator resets
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt;

			_vehicle_global_position_pub.publish(global_position);
		}

		estimator_selector_status_s selector_status{};
		selector_status.primary_instance = _selected_instance;
		selector_status.instances_available = _available_instances;
		selector_status.instance_changed_count = _instance_changed_count;
		selector_status.last_instance_change = _last_instance_change;

		for (int i = 0; i < _available_instances; i++) {
			selector_status.combined_test_ratio[i] = _instance[i].combined_test_ratio;
			selector_status.relative_test_ratio[i] = _instance[i].relative_test_ratio;
			selector_status.healthy[i] = _instance[i].healthy;
		}

		selector_status.timestamp = hrt_absolute_time();
		_estimator_selector_status_pub.publish(selector_status);
	}

	_last_update_us = hrt_absolute_time();

}

void EKF2Selector::PrintStatus()
{
	PX4_INFO("available instances: %d", _available_instances);

	if (_selected_instance == UINT8_MAX) {
		PX4_WARN("selected instance: None");
	}

	for (int i = 0; i < MAX_INSTANCES; i++) {
		const EstimatorInstance &inst = _instance[i];

		if (inst.estimator_status.timestamp > 0) {
			PX4_INFO("%d: ACC: %d, GYRO: %d, MAG: %d, %s, test ratio: %.5f (%.5f) %s",
				 inst.instance, inst.estimator_status.accel_device_id, inst.estimator_status.gyro_device_id,
				 inst.estimator_status.mag_device_id,
				 inst.healthy ? "healthy" : "unhealthy",
				 (double)inst.combined_test_ratio, (double)inst.relative_test_ratio,
				 (_selected_instance == i) ? "*" : "");
		}
	}
}

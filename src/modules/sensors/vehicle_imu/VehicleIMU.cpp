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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

VehicleIMU::VehicleIMU(uint8_t accel_index, uint8_t gyro_index) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_sensor_accel_integrated_sub(this, ORB_ID(sensor_accel_integrated), accel_index),
	_sensor_gyro_integrated_sub(this, ORB_ID(sensor_gyro_integrated), gyro_index)
{
}

VehicleIMU::~VehicleIMU()
{
	Stop();
	delete _name;
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	return _sensor_accel_integrated_sub.registerCallback() && _sensor_gyro_integrated_sub.registerCallback();
}

void VehicleIMU::Stop()
{
	Deinit();

	// clear all registered callbacks
	_sensor_accel_integrated_sub.unregisterCallback();
	_sensor_gyro_integrated_sub.unregisterCallback();
}

void VehicleIMU::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {
		sensor_correction_s corrections{};

		if (_sensor_correction_sub.copy(&corrections)) {

			// accel
			if (_accel_device_id > 0) {
				if ((_corrections_selected_accel_instance < 0) || force) {
					_corrections_selected_accel_instance = -1;

					// find sensor_corrections index
					for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
						if (corrections.accel_device_ids[i] == _accel_device_id) {
							_corrections_selected_accel_instance = i;
						}
					}
				}

				switch (_corrections_selected_accel_instance) {
				case 0:
					_accel_offset = Vector3f{corrections.accel_offset_0};
					_accel_scale = Vector3f{corrections.accel_scale_0};
					break;
				case 1:
					_accel_offset = Vector3f{corrections.accel_offset_1};
					_accel_scale = Vector3f{corrections.accel_scale_1};
					break;
				case 2:
					_accel_offset = Vector3f{corrections.accel_offset_2};
					_accel_scale = Vector3f{corrections.accel_scale_2};
					break;
				default:
					_accel_offset = Vector3f{0.f, 0.f, 0.f};
					_accel_scale = Vector3f{1.f, 1.f, 1.f};
				}
			}

			// gyro
			if (_gyro_device_id > 0) {
				if ((_corrections_selected_gyro_instance < 0) || force) {
					_corrections_selected_gyro_instance = -1;

					// find sensor_corrections index
					for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
						if (corrections.gyro_device_ids[i] == _gyro_device_id) {
							_corrections_selected_gyro_instance = i;
						}
					}
				}

				switch (_corrections_selected_gyro_instance) {
				case 0:
					_gyro_offset = Vector3f{corrections.gyro_offset_0};
					_gyro_scale = Vector3f{corrections.gyro_scale_0};
					break;
				case 1:
					_gyro_offset = Vector3f{corrections.gyro_offset_1};
					_gyro_scale = Vector3f{corrections.gyro_scale_1};
					break;
				case 2:
					_gyro_offset = Vector3f{corrections.gyro_offset_2};
					_gyro_scale = Vector3f{corrections.gyro_scale_2};
					break;
				default:
					_gyro_offset = Vector3f{0.f, 0.f, 0.f};
					_gyro_scale = Vector3f{1.f, 1.f, 1.f};
				}
			}
		}
	}
}

void VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		// get transformation matrix from sensor/board to body frame
		const Dcmf board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

		// fine tune the rotation
		const Dcmf board_rotation_offset(Eulerf(
				radians(_param_sens_board_x_off.get()),
				radians(_param_sens_board_y_off.get()),
				radians(_param_sens_board_z_off.get())));

		_board_rotation = board_rotation_offset * board_rotation;
	}
}

void VehicleIMU::Run()
{
	if (_sensor_accel_integrated_sub.updated() && _sensor_gyro_integrated_sub.updated()) {
		sensor_accel_integrated_s accel;
		_sensor_accel_integrated_sub.copy(&accel);
		_accel_device_id = accel.device_id;

		sensor_gyro_integrated_s gyro;
		_sensor_gyro_integrated_sub.copy(&gyro);
		_gyro_device_id = gyro.device_id;

		SensorCorrectionsUpdate();
		ParametersUpdate();

		// delta angle: apply offsets, scale, and board rotation
		Vector3f delta_angle{gyro.delta_angle};
		const float gyro_dt = 1.e-6f * gyro.dt;
		// apply offsets
		delta_angle = delta_angle - (_gyro_offset * gyro_dt);
		// apply scale
		delta_angle = delta_angle.emult(_gyro_scale);
		// apply board rotation
		delta_angle = _board_rotation * delta_angle;

		// delta velocity: apply offsets, scale, and board rotation
		Vector3f delta_velocity{accel.delta_velocity};
		const float accel_dt = 1.e-6f * accel.dt;
		// apply offsets
		delta_velocity = delta_velocity - (_accel_offset * accel_dt);
		// apply scale
		delta_velocity = delta_velocity.emult(_accel_scale);
		// apply board rotation
		delta_velocity = _board_rotation * delta_velocity;


		// publich vehicle_imu
		vehicle_imu_s imu;

		imu.timestamp_sample = accel.timestamp_sample;
		imu.accel_device_id = accel.device_id;
		imu.gyro_device_id = gyro.device_id;

		delta_angle.copyTo(imu.delta_angle);
		delta_velocity.copyTo(imu.delta_velocity);

		imu.dt = accel.dt;
		imu.integrated_samples = accel.samples;
		imu.clip_count = accel.clip_count;
		imu.timestamp = hrt_absolute_time();

		_vehicle_imu_pub.publish(imu);
	}
}

void VehicleIMU::PrintStatus()
{
	PX4_INFO("selected IMU: accel: %d gyro: %d", _accel_device_id, _gyro_device_id);
}

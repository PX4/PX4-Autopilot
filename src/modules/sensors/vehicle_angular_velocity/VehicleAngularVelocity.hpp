/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/conversion/rotation.h>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_log.h>
#include <px4_module_params.h>
#include <px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_selection.h>

#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_control.h>
#include <uORB/topics/vehicle_angular_velocity.h>

class VehicleAngularVelocity : public ModuleParams, public px4::WorkItem
{
public:

	VehicleAngularVelocity();
	virtual ~VehicleAngularVelocity();

	void	Run() override;

	bool	Start();
	void	Stop();

	void	PrintStatus();

private:

	void	ParametersUpdate(bool force = false);
	void	SensorBiasUpdate(bool force = false);
	bool	SensorCorrectionsUpdate(bool force = false);

	static constexpr int MAX_SENSOR_COUNT = 3;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_BOARD_ROT>) _param_sens_board_rot,

		(ParamFloat<px4::params::SENS_BOARD_X_OFF>) _param_sens_board_x_off,
		(ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _param_sens_board_y_off,
		(ParamFloat<px4::params::SENS_BOARD_Z_OFF>) _param_sens_board_z_off
	)

	uORB::Publication<vehicle_angular_velocity_s>	_vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription			_params_sub{ORB_ID(parameter_update)};			/**< parameter updates subscription */
	uORB::Subscription			_sensor_bias_sub{ORB_ID(sensor_bias)};			/**< sensor in-run bias correction subscription */
	uORB::Subscription			_sensor_correction_sub{ORB_ID(sensor_correction)};	/**< sensor thermal correction subscription */

	uORB::SubscriptionCallbackWorkItem	_sensor_selection_sub{this, ORB_ID(sensor_selection)};	/**< selected primary sensor subscription */

	uORB::SubscriptionCallbackWorkItem	_sensor_sub[MAX_SENSOR_COUNT] {				/**< sensor data subscription */
		{this, ORB_ID(sensor_gyro), 0},
		{this, ORB_ID(sensor_gyro), 1},
		{this, ORB_ID(sensor_gyro), 2}
	};

	uORB::SubscriptionCallbackWorkItem	_sensor_control_sub[MAX_SENSOR_COUNT] {			/**< sensor control data subscription */
		{this, ORB_ID(sensor_gyro_control), 0},
		{this, ORB_ID(sensor_gyro_control), 1},
		{this, ORB_ID(sensor_gyro_control), 2}
	};

	matrix::Dcmf				_board_rotation;				/**< rotation matrix for the orientation that the board is mounted */

	matrix::Vector3f			_offset;
	matrix::Vector3f			_scale;
	matrix::Vector3f			_bias;

	perf_counter_t				_cycle_perf;
	perf_counter_t				_interval_perf;
	perf_counter_t				_sensor_latency_perf;

	uint32_t				_selected_sensor_device_id{0};
	uint8_t					_selected_sensor{0};
	uint8_t					_selected_sensor_control{0};
	bool					_sensor_control_available{false};

};

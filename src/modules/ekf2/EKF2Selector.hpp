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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>

class EKF2Selector : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2Selector();
	~EKF2Selector() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	bool SelectInstance(uint8_t instance, bool force_reselect = false);

	// Subscriptions (per estimator instance)
	struct EstimatorInstance {
		EstimatorInstance(EKF2Selector *selector, uint8_t i) :
			estimator_attitude_sub{selector, ORB_ID(estimator_attitude), i},
			estimator_status_sub{ORB_ID(estimator_status), i},
			estimator_global_position_sub{ORB_ID(estimator_global_position), i},
			estimator_local_position_sub{ORB_ID(estimator_local_position), i},
			estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias), i},
			instance(i)
		{}

		uORB::SubscriptionCallbackWorkItem estimator_attitude_sub;
		uORB::Subscription estimator_status_sub;
		uORB::Subscription estimator_global_position_sub;
		uORB::Subscription estimator_local_position_sub;
		uORB::Subscription estimator_sensor_bias_sub;

		estimator_status_s estimator_status{};

		hrt_abstime time_last_selected{0};

		float combined_test_ratio{0.f};
		float relative_test_ratio{0.f};

		bool healthy{false};

		const uint8_t instance;
	};

	static constexpr uint8_t MAX_INSTANCES{4};

	static constexpr float _rel_err_score_lim{1.0f}; // +- limit applied to the relative error score
	static constexpr float _rel_err_thresh{0.5f};    // the relative score difference needs to be greater than this to switch from an otherwise healthy instance

	EstimatorInstance _instance[MAX_INSTANCES] {
		{this, 0},
		{this, 1},
		{this, 2},
		{this, 3},
	};

	float _accumulated_gyro_error[MAX_INSTANCES]{};
	hrt_abstime _last_update_us{0};
	bool _gyro_fault_detected{false};

	uint8_t _available_instances{0};
	uint8_t _selected_instance{UINT8_MAX};

	uint32_t _instance_changed_count{0};
	hrt_abstime _last_instance_change{0};

	uORB::SubscriptionData<sensors_status_imu_s> _sensors_status_imu{ORB_ID(sensors_status_imu)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// vehicle_attitude: reset counters
	vehicle_attitude_s _attitude_last{};
	matrix::Quatf _delta_q_reset;
	uint8_t _quat_reset_counter{0};

	// vehicle_local_position: reset counters
	vehicle_local_position_s _local_position_last{};

	matrix::Vector2f _delta_xy;
	uint8_t _xy_reset_counter{0};

	float _delta_z{0.f};
	uint8_t _z_reset_counter{0};

	matrix::Vector2f _delta_vxy;
	uint8_t _vxy_reset_counter{0};

	float _delta_vz{0.f};
	uint8_t _vz_reset_counter{0};

	float _delta_heading{0};
	uint8_t _heading_reset_counter{0};

	// vehicle_global_position: reset counters
	vehicle_global_position_s _global_position_last{};
	float _delta_alt{0.f};
	uint8_t _alt_reset_counter{0};

	double _delta_lat{0};
	double _delta_lon{0};
	uint8_t _lat_lon_reset_counter{0};

	// Update the error scores for all available instances
	void updateErrorScores();

	// Publications
	uORB::Publication<estimator_selector_status_s> _estimator_selector_status_pub{ORB_ID(estimator_selector_status)};
	uORB::Publication<sensor_selection_s>          _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_attitude_s>          _vehicle_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>   _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>    _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EKF2_SEL_ERR_RED>) _param_ekf2_sel_err_red,
		(ParamFloat<px4::params::EKF2_SEL_GYR_RAT>) _param_ekf2_sel_gyr_rate,
		(ParamFloat<px4::params::EKF2_SEL_GYR_ANG>) _param_ekf2_sel_gyr_angle
	)
};

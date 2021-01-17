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

#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2pArray.hpp>
#include <lib/mathlib/math/filter/NotchFilterArray.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fft.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>

using namespace time_literals;

namespace sensors
{

class VehicleAngularVelocity : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleAngularVelocity();
	~VehicleAngularVelocity() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void CheckFilters();
	void ResetFilters(const matrix::Vector3f &angular_velocity, const matrix::Vector3f &angular_acceleration);

	float GetSampleRateForGyro(uint32_t device_id);

	void ParametersUpdate(bool force = false);
	void SensorBiasUpdate(bool force = false);
	bool SensorSelectionUpdate(bool force = false);

	void Publish(const hrt_abstime &timestamp_sample, const matrix::Vector3f &angular_velocity,
		     const matrix::Vector3f &angular_acceleration);

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Publication<vehicle_angular_acceleration_s> _vehicle_angular_acceleration_pub{ORB_ID(vehicle_angular_acceleration)};
	uORB::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _sensor_gyro_fft_sub{ORB_ID(sensor_gyro_fft)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};
	uORB::SubscriptionCallbackWorkItem _sensor_sub{this, ORB_ID(sensor_gyro)};
	uORB::SubscriptionCallbackWorkItem _sensor_fifo_sub{this, ORB_ID(sensor_gyro_fifo)};

	matrix::Vector3f _fifo_data_filtered_prev{};

	calibration::Gyroscope _calibration{};

	matrix::Vector3f _bias{};

	matrix::Vector3f _angular_velocity_last{};
	hrt_abstime _timestamp_sample_last{0};

	hrt_abstime _publish_interval_min_us{0};
	hrt_abstime _last_publish{0};
	static constexpr const float kInitialRateHz{1000.f}; /**< sensor update rate used for initialization */
	float _update_rate_hz{kInitialRateHz}; /**< current rate-controller loop update rate in [Hz] */

	uint8_t _required_sample_updates{0}; /**< number or sensor publications required for configured rate */

	static constexpr int MAX_NUM_FFT_PEAKS = sizeof(sensor_gyro_fft_s::peak_frequencies_x) / sizeof(
				sensor_gyro_fft_s::peak_frequencies_x[0]);

	// angular velocity filters
	math::LowPassFilter2pArray _lp_filter_velocity[3] {{kInitialRateHz, 30.f}, {kInitialRateHz, 30.f}, {kInitialRateHz, 30.f}};
	math::NotchFilterArray<float> _notch_filter_velocity[3] {};
	math::NotchFilterArray<float> _dynamic_notch_filter[MAX_NUM_FFT_PEAKS][3] {};

	// angular acceleration filter
	math::LowPassFilter2p _lp_filter_acceleration[3] {{kInitialRateHz, 30.f}, {kInitialRateHz, 30.f}, {kInitialRateHz, 30.f}};

	float _filter_sample_rate{kInitialRateHz};

	float _sensor_sample_rate[MAX_SENSOR_COUNT] {NAN, NAN, NAN, NAN};

	uint32_t _selected_sensor_device_id{0};
	uint8_t _selected_sensor_sub_index{0};

	hrt_abstime _timestamp_interval_last{0};

	bool _sample_rate_determined{false};
	bool _reset_filters{false};

	bool _fifo_available{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff,
		(ParamFloat<px4::params::IMU_GYRO_NF_FREQ>) _param_imu_gyro_nf_freq,
		(ParamFloat<px4::params::IMU_GYRO_NF_BW>) _param_imu_gyro_nf_bw,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_rate_max,

		(ParamFloat<px4::params::IMU_DGYRO_CUTOFF>) _param_imu_dgyro_cutoff
	)
};

} // namespace sensors

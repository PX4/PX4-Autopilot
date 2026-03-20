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

#include <containers/Bitset.hpp>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fft.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_acceleration.h>

using namespace time_literals;

namespace sensors
{

class VehicleAcceleration : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleAcceleration();
	~VehicleAcceleration() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	inline float FilterAcceleration(int axis, float data[], int N = 1);

	void DisableDynamicNotchEscRpm();
	void DisableDynamicNotchFFT();
	void ParametersUpdate(bool force = false);

	void ResetFilters(const hrt_abstime &time_now_us);
	void SensorBiasUpdate(bool force = false);
	bool SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force = false);
	void UpdateDynamicNotchEscRpm(const hrt_abstime &time_now_us, bool force = false);
	void UpdateDynamicNotchFFT(const hrt_abstime &time_now_us, bool force = false);
	bool UpdateSampleRate();

	matrix::Vector3f GetResetAcceleration() const;

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Publication<vehicle_acceleration_s> _vehicle_acceleration_pub{ORB_ID(vehicle_acceleration)};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
#if !defined(CONSTRAINED_FLASH)
	uORB::Subscription _esc_status_sub {ORB_ID(esc_status)};
	uORB::Subscription _sensor_gyro_fft_sub{ORB_ID(sensor_gyro_fft)};
#endif // !CONSTRAINED_FLASH

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};
	uORB::SubscriptionCallbackWorkItem _sensor_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionCallbackWorkItem _sensor_accel_fifo_sub{this, ORB_ID(sensor_accel_fifo)};

	calibration::Accelerometer _calibration{};

	matrix::Vector3f _bias{};

	matrix::Vector3f _acceleration{};

	hrt_abstime _publish_interval_min_us{0};
	hrt_abstime _last_publish{0};

	float _filter_sample_rate_hz{NAN};

	// acceleration filters
	math::LowPassFilter2p<float> _lp_filter[3] {};
	math::NotchFilter<float> _notch_filter0[3] {};
	math::NotchFilter<float> _notch_filter1[3] {};

#if !defined(CONSTRAINED_FLASH)

	enum DynamicNotch {
		EscRpm = 1,
		FFT    = 2,
	};

	static constexpr hrt_abstime DYNAMIC_NOTCH_FITLER_TIMEOUT = 3_s;

	// ESC RPM
	static constexpr int MAX_NUM_ESCS = sizeof(esc_status_s::esc) / sizeof(esc_status_s::esc[0]);

	using NotchFilterHarmonic = math::NotchFilter<float>[3][MAX_NUM_ESCS];
	NotchFilterHarmonic *_dynamic_notch_filter_esc_rpm{nullptr};

	int _esc_rpm_harmonics{0};
	px4::Bitset<MAX_NUM_ESCS> _esc_available{};
	hrt_abstime _last_esc_rpm_notch_update[MAX_NUM_ESCS] {};

	perf_counter_t _dynamic_notch_filter_esc_rpm_disable_perf{nullptr};
	perf_counter_t _dynamic_notch_filter_esc_rpm_init_perf{nullptr};
	perf_counter_t _dynamic_notch_filter_esc_rpm_update_perf{nullptr};

	// FFT
	static constexpr int MAX_NUM_FFT_PEAKS = sizeof(sensor_gyro_fft_s::peak_frequencies_x)
			/ sizeof(sensor_gyro_fft_s::peak_frequencies_x[0]);

	math::NotchFilter<float> _dynamic_notch_filter_fft[3][MAX_NUM_FFT_PEAKS] {};

	perf_counter_t _dynamic_notch_filter_fft_disable_perf{nullptr};
	perf_counter_t _dynamic_notch_filter_fft_update_perf{nullptr};

	bool _dynamic_notch_fft_available{false};
#endif // !CONSTRAINED_FLASH

	uint32_t _selected_sensor_device_id{0};
	uint32_t _selected_gyro_device_id{0};

	bool _reset_filters{true};
	bool _fifo_available{false};
	bool _update_sample_rate{true};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": accel filter")};
	perf_counter_t _filter_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": accel filter reset")};
	perf_counter_t _selection_changed_perf{perf_alloc(PC_COUNT, MODULE_NAME": accel selection changed")};

	DEFINE_PARAMETERS(
#if !defined(CONSTRAINED_FLASH)
		(ParamInt<px4::params::IMU_ACC_DNF_EN>) _param_imu_acc_dnf_en,
		(ParamInt<px4::params::IMU_ACC_DNF_HMC>) _param_imu_acc_dnf_hmc,
		(ParamFloat<px4::params::IMU_ACC_DNF_BW>) _param_imu_acc_dnf_bw,
		(ParamFloat<px4::params::IMU_ACC_DNF_MIN>) _param_imu_acc_dnf_min,
#endif // !CONSTRAINED_FLASH
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff,
		(ParamFloat<px4::params::IMU_ACC_NF0_FRQ>) _param_imu_acc_nf0_frq,
		(ParamFloat<px4::params::IMU_ACC_NF0_BW>) _param_imu_acc_nf0_bw,
		(ParamFloat<px4::params::IMU_ACC_NF1_FRQ>) _param_imu_acc_nf1_frq,
		(ParamFloat<px4::params::IMU_ACC_NF1_BW>) _param_imu_acc_nf1_bw,
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate
	)
};

} // namespace sensors

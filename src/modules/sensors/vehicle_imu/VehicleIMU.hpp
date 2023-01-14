/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include "IMU.hpp"
#include "VehicleAcceleration.hpp"
#include "VehicleAngularVelocity.hpp"

#include <include/containers/Bitset.hpp>

#include <lib/mathlib/math/Limits.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// publications
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/sensor_selection.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_imu_fifo.h>
#include <uORB/topics/vehicle_control_mode.h>

using namespace time_literals;

namespace sensors
{

class VehicleIMU : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleIMU();
	~VehicleIMU() override;

	void PrintStatus();
	bool Start();
	void Stop();

private:
	void Run() override;

	bool ParametersUpdate(bool force = false);

	int8_t findAccelInstance(uint32_t device_id);

	bool PublishImu(sensors::IMU &imu, const matrix::Vector3f &delta_angle, const uint16_t delta_angle_dt,
			const matrix::Vector3f &delta_velocity, const uint16_t delta_velocity_dt);
	void PublishSensorsStatusIMU();

	void SensorBiasUpdate();
	bool SensorSelectionUpdate(bool force = false);
	void UpdateSensorImuFifo(uint8_t sensor_instance);
	void UpdateSensorAccel(uint8_t sensor_instance);
	void UpdateSensorGyro(uint8_t sensor_instance);

	void SensorCalibrationUpdate();
	void SensorCalibrationSaveAccel();
	void SensorCalibrationSaveGyro();

	void updateGyroCalibration();

	// return the square of two floating point numbers
	static constexpr float sq(float var) { return var * var; }

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::SubscriptionMultiArray<estimator_sensor_bias_s> _estimator_sensor_bias_subs{ORB_ID::estimator_sensor_bias};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	uORB::SubscriptionCallbackWorkItem _sensor_imu_fifo_subs[MAX_SENSOR_COUNT] {
		{this, ORB_ID::sensor_imu_fifo, 0},
		{this, ORB_ID::sensor_imu_fifo, 1},
		{this, ORB_ID::sensor_imu_fifo, 2},
		{this, ORB_ID::sensor_imu_fifo, 3}
	};

	uORB::SubscriptionCallbackWorkItem _sensor_accel_subs[MAX_SENSOR_COUNT] {
		{this, ORB_ID::sensor_accel, 0},
		{this, ORB_ID::sensor_accel, 1},
		{this, ORB_ID::sensor_accel, 2},
		{this, ORB_ID::sensor_accel, 3}
	};

	uORB::SubscriptionCallbackWorkItem _sensor_gyro_subs[MAX_SENSOR_COUNT] {
		{this, ORB_ID::sensor_gyro, 0},
		{this, ORB_ID::sensor_gyro, 1},
		{this, ORB_ID::sensor_gyro, 2},
		{this, ORB_ID::sensor_gyro, 3}
	};

	IMU _imus[MAX_SENSOR_COUNT] {};



	// struct SensorData {
	// 	DataValidatorGroup voter{1};
	// 	unsigned int last_failover_count{0};
	// 	int32_t priority[MAX_SENSOR_COUNT] {};
	// 	int32_t priority_configured[MAX_SENSOR_COUNT] {};
	// 	uint8_t last_best_vote{0}; /**< index of the latest best vote */
	// 	uint8_t subscription_count{0};
	// 	bool advertised[MAX_SENSOR_COUNT] {false, false, false};
	// };

	// SensorData _accel{};
	// SensorData _gyro{};

	hrt_abstime _last_error_message{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	matrix::Vector3f _accel_diff[MAX_SENSOR_COUNT] {};		/**< filtered accel differences between IMU units (m/s/s) */
	matrix::Vector3f _gyro_diff[MAX_SENSOR_COUNT] {};			/**< filtered gyro differences between IMU uinits (rad/s) */



	// TODO:
	uORB::Publication<sensor_combined_s>    _sensor_combined_pub{ORB_ID(sensor_combined)}; // legacy
	uORB::Publication<sensor_selection_s>   _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<sensors_status_imu_s> _sensors_status_imu_pub{ORB_ID(sensors_status_imu)};

	VehicleAcceleration _vehicle_acceleration{};
	VehicleAngularVelocity _vehicle_angular_velocity{};

	uint32_t _selected_accel_device_id{0};
	uint32_t _selected_gyro_device_id{0};
	int8_t _selected_imu_index{-1};


	// calibration
	hrt_abstime _last_calibration_update{0};

	static constexpr hrt_abstime INFLIGHT_CALIBRATION_QUIET_PERIOD_US{30_s};

	hrt_abstime _in_flight_calibration_check_timestamp_last{0};
	bool _accel_cal_available{false};
	bool _gyro_cal_available{false};


	bool _armed{false};


	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": IMU update")};
	perf_counter_t _selection_changed_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU selection changed")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SENS_IMU_AUTOCAL>) _param_sens_imu_autocal,
		(ParamBool<px4::params::SENS_IMU_MODE>) _param_sens_imu_mode,
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax
	)

};

} // namespace sensors

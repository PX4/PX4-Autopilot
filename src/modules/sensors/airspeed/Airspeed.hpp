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

#include "data_validator/DataValidatorGroup.hpp"

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>

using namespace time_literals;

namespace sensors
{
class Airspeed : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	Airspeed();
	~Airspeed() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);

	void Publish(uint8_t instance, bool multi = false);

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::PublicationMulti<airspeed_s> _airspeed_multi_pub[MAX_SENSOR_COUNT] {
		{ORB_ID(airspeed)},
		{ORB_ID(airspeed)},
		{ORB_ID(airspeed)},
		{ORB_ID(airspeed)},
	};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};

	uORB::SubscriptionCallbackWorkItem _sensor_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(differential_pressure), 0},
		{this, ORB_ID(differential_pressure), 1},
		{this, ORB_ID(differential_pressure), 2},
		{this, ORB_ID(differential_pressure), 3},
	};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	hrt_abstime _last_error_message{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	DataValidatorGroup _voter{1};
	unsigned _last_failover_count{0};

	uint32_t _device_id[MAX_SENSOR_COUNT] {};
	uint64_t _timestamp_sample_sum[MAX_SENSOR_COUNT] {0};
	hrt_abstime _last_publication_timestamp[MAX_SENSOR_COUNT] {};
	float _differential_pressure_sum[MAX_SENSOR_COUNT] {};
	float _temperature_sum[MAX_SENSOR_COUNT] {};
	int _sum_count[MAX_SENSOR_COUNT] {};

	differential_pressure_s _last_data[MAX_SENSOR_COUNT] {};
	bool _advertised[MAX_SENSOR_COUNT] {};

	uint8_t _priority[MAX_SENSOR_COUNT] {100, 100, 100, 100};

	int8_t _selected_sensor_sub_index{-1};

	float _baro_pressure_pa{101325.f}; // Sea level standard atmospheric pressure
	float _baro_air_temperature{15.f};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_DPRES_MODE>) _param_sens_dpres_mode,
		(ParamFloat<px4::params::SENS_DPRES_OFF>) _param_sens_dpres_off,
		(ParamFloat<px4::params::SENS_DPRES_RATE>) _param_sens_dpres_rate,
		(ParamInt<px4::params::CAL_AIR_CMODEL>) _param_cal_air_cmodel,
		(ParamFloat<px4::params::CAL_AIR_TUBELEN>) _param_cal_air_tubelen,
		(ParamFloat<px4::params::CAL_AIR_TUBED_MM>) _param_cal_air_tubed_mm
	)
};
}; // namespace sensors

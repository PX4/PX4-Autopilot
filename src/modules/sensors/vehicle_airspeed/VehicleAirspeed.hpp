/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "data_validator/DataValidator.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_calibration/DifferentialPressure.hpp>
#include <lib/parameters/param.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>

using namespace time_literals;

namespace sensors
{

/**
 * Converts the differential pressure of a single sensor into an airspeed measurement.
 *
 * One instance is created per advertised differential_pressure instance.
 * The airspeed selector is responsible for validating and choosing between them.
 */
class VehicleAirspeed : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit VehicleAirspeed(uint8_t instance);
	~VehicleAirspeed() override;

	bool Start();
	void Stop();

	void PrintStatus();

	uint32_t device_id() const { return _calibration.device_id(); }

private:
	void Run() override;

	void ParametersUpdate(bool force = false);
	void ResetAccumulation();

	static constexpr hrt_abstime kPublishInterval{50_ms};

	const uint8_t _instance;

	uORB::SubscriptionCallbackWorkItem _sensor_sub;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};

	uORB::PublicationMulti<airspeed_s> _airspeed_pub{ORB_ID(airspeed)};

	calibration::DifferentialPressure _calibration;

	DataValidator _airspeed_validator;

	// accumulated between publications so that the published airspeed is an average
	uint64_t _timestamp_sample_sum{0};
	float _differential_pressure_sum{0.f};
	float _baro_pressure_sum{0.f};
	float _temperature_sum{0.f};
	int _sample_count{0};

	hrt_abstime _last_publish{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CAL_AIR_CMODEL>) _param_cal_air_cmodel,
		(ParamFloat<px4::params::CAL_AIR_TUBELEN>) _param_cal_air_tubelen,
		(ParamFloat<px4::params::CAL_AIR_TUBED_MM>) _param_cal_air_tubed_mm
	)
};

} // namespace sensors

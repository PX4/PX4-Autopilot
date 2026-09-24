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

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_slot_binder/SensorSlotBinder.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/pps_capture.h>

#include "PpsTimeSync.hpp"

using namespace time_literals;

namespace sensors
{
class VehicleGPSPosition : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleGPSPosition();
	~VehicleGPSPosition() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);

	int8_t receiverSlot(uint8_t instance, uint32_t device_id);

	static constexpr uint8_t GPS_MAX_RECEIVERS = 2;

	// uORB instance == SENS_GPS<i> slot
	SlotPublications<sensor_gps_s, GPS_MAX_RECEIVERS, ORB_ID::vehicle_gps_position> _vehicle_gps_position_pubs{};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gps_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gps), 0},
		{this, ORB_ID(sensor_gps), 1},
	};

	uORB::Subscription _pps_capture_sub{ORB_ID(pps_capture)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	PpsTimeSync _pps_time_sync;

	SensorSlotBinder _slot_binder{};
	int8_t _instance_slot[GPS_MAX_RECEIVERS] {-1, -1};
	bool _no_slot_warned[GPS_MAX_RECEIVERS] {};

	struct GpsParamSlot {
		matrix::Vector3f offset{};
		hrt_abstime delay_us{110_ms};
	} _gps_param_slots[GPS_MAX_RECEIVERS] {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_GPS0_OFFX>) _param_sens_gps0_offx,
		(ParamFloat<px4::params::SENS_GPS0_OFFY>) _param_sens_gps0_offy,
		(ParamFloat<px4::params::SENS_GPS0_OFFZ>) _param_sens_gps0_offz,
		(ParamFloat<px4::params::SENS_GPS1_OFFX>) _param_sens_gps1_offx,
		(ParamFloat<px4::params::SENS_GPS1_OFFY>) _param_sens_gps1_offy,
		(ParamFloat<px4::params::SENS_GPS1_OFFZ>) _param_sens_gps1_offz,
		(ParamInt<px4::params::SENS_GPS0_DELAY>) _param_sens_gps0_delay,
		(ParamInt<px4::params::SENS_GPS1_DELAY>) _param_sens_gps1_delay
	)
};
}; // namespace sensors

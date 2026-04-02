/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

/**
 * @file BaroThrustEstimator.hpp
 *
 * Online estimator for barometer thrust compensation parameters.
 * Module wrapper around BaroThrustCfRls — handles uORB I/O,
 * parameter persistence, and lifecycle.
 */

#pragma once

#include "baro_thrust_cf_rls.hpp"

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/baro_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

using namespace time_literals;

class BaroThrustEstimator : public ModuleBase, public ModuleParams,
	public px4::WorkItem
{
public:
	static Descriptor desc;

	BaroThrustEstimator();
	~BaroThrustEstimator() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	static constexpr float MAX_VZ = 2.f;
	static constexpr float MAX_VXY = 5.f;
	static constexpr float PCOEF_MAX = 30.f;
	static constexpr float MIN_K_UPDATE_THRESHOLD = 0.1f;

	void Run() override;
	void updateParams() override;
	void saveParameters();
	void publishStatus(hrt_abstime now, float residual, bool estimation_active);

	BaroThrustCfRls _estimator{};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_air_data_sub{this, ORB_ID(vehicle_air_data)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// Publication
	uORB::Publication<baro_thrust_estimate_s> _baro_thrust_estimate_pub{ORB_ID(baro_thrust_estimate)};

	// State
	bool _armed{false};
	bool _landed{true};
	hrt_abstime _estimation_start_time{0};
	hrt_abstime _last_update_time{0};
	hrt_abstime _last_publish_time{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_BARO_PCOEF>) _param_sens_baro_pcoef,
		(ParamFloat<px4::params::SENS_BAR_CF_BW>) _param_sens_bar_cf_bw
	)
};

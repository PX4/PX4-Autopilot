/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file fake_ev.hpp
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_odometry.h>

using namespace time_literals;

class FakeEV : public ModuleBase<FakeEV>, public ModuleParams,
	public px4::WorkItem
{
public:
	FakeEV();
	~FakeEV() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	static constexpr vehicle_odometry_s vehicle_odometry_empty {
		.timestamp = 0,
		.timestamp_sample = 0,
		.position = {NAN, NAN, NAN},
		.q = {NAN, NAN, NAN, NAN},
		.velocity = {NAN, NAN, NAN},
		.angular_velocity = {NAN, NAN, NAN},
		.position_variance = {NAN, NAN, NAN},
		.orientation_variance = {NAN, NAN, NAN},
		.velocity_variance = {NAN, NAN, NAN},
		.pose_frame = vehicle_odometry_s::POSE_FRAME_UNKNOWN,
		.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN,
		.reset_counter = 0,
		.quality = 0
	};

	void Run() override;
	void updateParams() override;

	vehicle_odometry_s gpsToOdom(const sensor_gps_s &gps);

	uORB::Publication<vehicle_odometry_s> _vehicle_visual_odometry_pub{ORB_ID(vehicle_fake_visual_odometry)};

	uORB::SubscriptionCallbackWorkItem _vehicle_gps_position_sub{this, ORB_ID(vehicle_gps_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	hrt_abstime _timestamp_last{0};

	MapProjection _pos_ref{}; // Contains WGS-84 position latitude and longitude
	float _alt_ref{NAN};

	hrt_abstime _last_run{0};
	float _h_drift{0.f};
	float _h_drift_rate{0.05f};

	bool _is_stale{false};
	matrix::Vector2f _hpos_prev;

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FEV_EN>) _param_fev_en,
		(ParamBool<px4::params::FEV_STALE>) _param_fev_stale,
		(ParamFloat<px4::params::FEV_H_DRIFT_RATE>) _param_fev_h_drift_rate
	)
};

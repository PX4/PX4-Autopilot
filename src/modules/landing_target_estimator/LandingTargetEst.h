/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file LandingTargetEst.h
 * Landing target position estimator. Filter and publish the position of a landing target on the ground as observed by an onboard sensor.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <lib/hysteresis/hysteresis.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_acceleration.h>

#include <parameters/param.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>

#include "LTEstPosition.h"
#include "LTEstOrientation.h"


namespace landing_target_estimator
{

class LandingTargetEst : public ModuleBase<LandingTargetEst>, ModuleParams, px4::ScheduledWorkItem
{
public:
	LandingTargetEst();
	virtual ~LandingTargetEst();

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

	static int task_spawn(int argc, char *argv[]);

private:
	void Run() override;
	void updateParams() override;

	void reset_filters();
	bool get_input(matrix::Vector3f &acc_ned, matrix::Quaternionf &q_att);

	perf_counter_t _cycle_perf_pos{perf_alloc(PC_ELAPSED, MODULE_NAME": ltest cycle pos")};
	perf_counter_t _cycle_perf_yaw{perf_alloc(PC_ELAPSED, MODULE_NAME": ltest cycle yaw")};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": ltest cycle ")};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	bool _start_filters{false};
	uint64_t _land_time{0};


	bool _ltest_orientation_valid{false};
	LTEstOrientation *_ltest_orientation {nullptr};
	hrt_abstime _last_update_yaw{0};

	LTEstPosition *_ltest_position {nullptr};
	bool _ltest_position_valid{false};
	hrt_abstime _last_update_pos{0};

	struct localPose {
		bool pos_valid = false;
		matrix::Vector3f xyz;

		bool dist_valid = false;
		float dist_bottom = 0, f;

		bool yaw_valid = false;
		float yaw = 0.f;
	};

	bool get_local_pose(localPose &local_pose);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LTEST_YAW_EN>) _param_ltest_yaw_en,
		(ParamInt<px4::params::LTEST_POS_EN>) _param_ltest_pos_en
	)
};

} // namespace land_detector

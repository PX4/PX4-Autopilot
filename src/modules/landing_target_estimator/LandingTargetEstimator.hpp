/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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

#include "KalmanFilter.hpp"

#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/irlock_report.h>
#include <uORB/topics/landing_target_innovations.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/uwb_distance.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class LandingTargetEstimator : public ModuleBase<LandingTargetEstimator>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	LandingTargetEstimator();
	~LandingTargetEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	enum class TargetMode {
		Moving = 0,
		Stationary
	};

	static constexpr uint32_t TARGET_UPDATE_TIMEOUT_US{2_s};

	uORB::Publication<landing_target_pose_s> _landing_target_pose_pub{ORB_ID(landing_target_pose)};
	uORB::Publication<landing_target_innovations_s> _landing_target_innovations_pub{ORB_ID(landing_target_innovations)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _irlock_report_sub{ORB_ID(irlock_report)};
	uORB::Subscription _uwb_distance_sub{ORB_ID(uwb_distance)};

	bool _estimator_initialized{false};

	KalmanFilter _kalman_filter_x{};
	KalmanFilter _kalman_filter_y{};
	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LTEST_MODE>) _param_ltest_mode,
		(ParamFloat<px4::params::LTEST_ACC_UNC>) _param_ltest_acc_unc,
		(ParamFloat<px4::params::LTEST_MEAS_UNC>) _param_ltest_meas_unc,
		(ParamFloat<px4::params::LTEST_POS_UNC_IN>) _param_ltest_pos_unc_in,
		(ParamFloat<px4::params::LTEST_VEL_UNC_IN>) _param_ltest_vel_unc_in,
		(ParamFloat<px4::params::LTEST_SCALE_X>) _param_ltest_scale_x,
		(ParamFloat<px4::params::LTEST_SCALE_Y>) _param_ltest_scale_y,
		(ParamInt<px4::params::LTEST_SENS_ROT>) _param_ltest_sens_rot,
		(ParamFloat<px4::params::LTEST_SENS_POS_X>) _param_ltest_sens_pos_x,
		(ParamFloat<px4::params::LTEST_SENS_POS_Y>) _param_ltest_sens_pos_y,
		(ParamFloat<px4::params::LTEST_SENS_POS_Z>) _param_ltest_sens_pos_z
	)
};

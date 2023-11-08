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
 * @file VisionTargetEst.h
 * @brief Handles the position and orientation estimators.
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
#include <uORB/topics/vehicle_angular_velocity.h>
#if !defined(CONSTRAINED_FLASH)
#include <uORB/topics/prec_land_status.h>
#endif

#include <parameters/param.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>

#include "Position/VTEPosition.h"
#include "Orientation/VTEOrientation.h"


namespace vision_target_estimator
{

class VisionTargetEst : public ModuleBase<VisionTargetEst>, ModuleParams, px4::ScheduledWorkItem
{
public:
	VisionTargetEst();
	virtual ~VisionTargetEst();

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

	void update_task_topics();
	bool should_task_start();
	bool is_current_task_done();
	bool start_position_estimator();
	void stop_position_estimator();
	bool start_orientation_estimator();
	void stop_orientation_estimator();
	bool get_input(matrix::Vector3f &acc_ned, matrix::Vector3f &gps_pos_offset, matrix::Vector3f &gps_vel_offset,
		       bool gps_vel_offset_updated = false);

	perf_counter_t _cycle_perf_pos{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE cycle pos")};
	perf_counter_t _cycle_perf_yaw{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE cycle yaw")};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE cycle ")};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
#if !defined(CONSTRAINED_FLASH)
	uORB::Subscription _prec_land_status_sub {ORB_ID(prec_land_status)};
#endif

	uORB::Publication<vehicle_acceleration_s> _vte_acc_input_pub{ORB_ID(vte_acc_input)};

	enum VisionTargetEstTask : uint16_t {
		// Bit locations for VTE tasks
		VTE_FOR_PREC_LAND  = (1 << 0),    ///< set to true if target GPS position data is ready to be fused
	};

	int _vte_current_task{0};
	int _vte_task_mask{0};

	bool _position_estimator_running{false};
	bool _orientation_estimator_running{false};
	bool _is_in_prec_land{false}; // Start target estimator during precision landing
	uint64_t _vte_position_stop_time{0};
	uint64_t _vte_orientation_stop_time{0};
	bool _vte_task_running{false};

	bool _vte_orientation_enabled{false};
	VTEOrientation *_vte_orientation {nullptr};
	hrt_abstime _last_update_yaw{0};

	VTEPosition *_vte_position {nullptr};
	bool _vte_position_enabled{false};
	hrt_abstime _last_update_pos{0};

	struct localPose {
		bool pos_valid = false;
		matrix::Vector3f xyz;

		bool vel_valid = false;
		matrix::Vector3f vel_xyz;

		bool dist_valid = false;
		float dist_bottom = 0, f;

		bool yaw_valid = false;
		float yaw = 0.f;

		hrt_abstime timestamp{0};
	};

	matrix::Vector3f _gps_pos_offset;
	bool _gps_pos_is_offset;

	bool get_gps_velocity_offset(matrix::Vector3f &vel_offset);
	bool get_local_pose(localPose &local_pose);

	/* Down sample acceleration data */
	matrix::Vector3f _vehicle_acc_ned_sum;
	int _loops_count{0};
	hrt_abstime _last_acc_reset{0};
	void reset_acc_downsample();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VTE_YAW_EN>) _param_vte_yaw_en,
		(ParamInt<px4::params::VTE_POS_EN>) _param_vte_pos_en,
		(ParamInt<px4::params::VTE_TASK_MASK>) _param_vte_task_mask
	)
};

} // namespace land_detector

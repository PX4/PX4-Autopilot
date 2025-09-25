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
#include <stdio.h>
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
#include <uORB/topics/vision_target_est_input.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/position_setpoint_triplet.h>
#if !defined(CONSTRAINED_FLASH)
#include <uORB/topics/prec_land_status.h>
#endif

#include <parameters/param.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <matrix/Quaternion.hpp>

#include "Position/VTEPosition.h"
#include "Orientation/VTEOrientation.h"
#include "common.h"

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
	struct LocalPose {
		bool pos_valid = false;
		matrix::Vector3f xyz{};

		bool vel_valid = false;
		matrix::Vector3f vel_xyz{};

		bool dist_valid = false;
		float dist_bottom = 0.f;

		bool yaw_valid = false;
		float yaw = 0.f;

		hrt_abstime timestamp{0};
	};

	void Run() override;
	void updateParams() override;
	void handleExit();
	void stopAllEstimators();
	void startEstIfNeeded();
	bool allEstStoppedDueToTimeout();
	void updateEstimators();
	void updatePosEst(const LocalPose &local_pose, const bool local_pose_updated,
			  const matrix::Vector3f &vehicle_acc_ned,
			  const matrix::Vector3f &gps_pos_offset_ned,
			  const matrix::Vector3f &vel_offset_ned,
			  const bool vel_offset_updated);
	void updateYawEst(const LocalPose &local_pose, const bool local_pose_updated);
	void publishVteInput(const matrix::Vector3f &vehicle_acc_ned_sampled, const matrix::Quaternionf &q_att);

	inline bool noActiveTask() const {return _current_task.value == 0;};
	inline bool noEstRunning() const
	{
		return (!_vte_orientation_enabled || !_orientation_estimator_running) && (!_vte_position_enabled
				|| !_position_estimator_running);
	};

	void updateTaskTopics();
	bool isNewTaskAvailable();
	bool isCurrentTaskComplete();
	bool startPosEst();
	void stopPosEst();
	bool startYawEst();
	void stopYawEst();
	bool pollEstimatorInput(matrix::Vector3f &acc_ned, matrix::Quaternionf &q_att, matrix::Vector3f &gps_pos_offset,
				matrix::Vector3f &vel_offset_ned,
				bool vel_offset_updated,
				bool &acc_valid);

	perf_counter_t _cycle_perf_pos{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE cycle pos")};
	perf_counter_t _cycle_perf_yaw{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE cycle yaw")};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE cycle ")};

	Vector3fStamped _vehicle_acc_body{};
	matrix::Quaternionf _last_att{1.f, 0.f, 0.f, 0.f};
	hrt_abstime _acc_sample_warn_last{0};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
#if !defined(CONSTRAINED_FLASH)
	uORB::Subscription _prec_land_status_sub {ORB_ID(prec_land_status)};
#endif

	uORB::Publication<vision_target_est_input_s> _vision_target_est_input_pub{ORB_ID(vision_target_est_input)};

	param_t _param_ekf2_gps_pos_x{PARAM_INVALID};
	param_t _param_ekf2_gps_pos_y{PARAM_INVALID};
	param_t _param_ekf2_gps_pos_z{PARAM_INVALID};

	union VisionTargetEstTaskMaskU {
		struct {
			uint8_t for_prec_land : 1; ///< bit0: precision landing task active
			uint8_t debug         : 1; ///< bit1: debug task active
			uint8_t reserved      : 6; ///< bits2..7: reserved for future use
		} flags;

		uint8_t value{0};
	};

	static_assert(sizeof(VisionTargetEstTaskMaskU) == 1, "Unexpected task mask size");

	VisionTargetEstTaskMaskU _current_task{};
	VisionTargetEstTaskMaskU _vte_task_mask{};

	bool _position_estimator_running{false};
	bool _orientation_estimator_running{false};
	bool _is_in_prec_land{false}; // Start target estimator during precision landing
	uint64_t _vte_position_stop_time{0};
	uint64_t _vte_orientation_stop_time{0};

	uint32_t _vte_timeout_us{3_s};
	uint32_t _target_valid_timeout_us{2_s};
	uint32_t _meas_recent_timeout_us{1_s};
	uint32_t _meas_updated_timeout_us{100_ms};
	float _pos_update_rate_hz{50.f};
	float _yaw_update_rate_hz{50.f};
	hrt_abstime _pos_update_period_us{20_ms};
	hrt_abstime _yaw_update_period_us{20_ms};
	SensorFusionMaskU _vte_aid_mask{};
	uint16_t adjustAidMask(const int input_mask);
	void printAidMask();

	bool _vte_orientation_enabled{false};
	VTEOrientation _vte_orientation{};
	hrt_abstime _last_update_yaw{0};

	VTEPosition _vte_position{};
	bool _vte_position_enabled{false};
	hrt_abstime _last_update_pos{0};

	matrix::Vector3f _gps_pos_offset_xyz{};
	bool _gps_pos_is_offset{false};

	bool computeGpsVelocityOffset(matrix::Vector3f &vel_offset_body);
	bool pollLocalPose(LocalPose &local_pose);
	const position_setpoint_s *findLandSetpoint();

	bool updateWhenIntervalElapsed(hrt_abstime &last_time, const hrt_abstime interval) const;

	/* Down sample acceleration data */
	matrix::Vector3f _vehicle_acc_ned_sum{};
	uint32_t _acc_sample_count{0};
	hrt_abstime _last_acc_reset{0};
	void resetAccDownsample();
	position_setpoint_triplet_s _pos_sp_triplet_buffer{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VTE_YAW_EN>) _param_vte_yaw_en,
		(ParamInt<px4::params::VTE_POS_EN>) _param_vte_pos_en,
		(ParamInt<px4::params::VTE_TASK_MASK>) _param_vte_task_mask,
		(ParamFloat<px4::params::VTE_BTOUT>) _param_vte_btout,
		(ParamFloat<px4::params::VTE_TGT_TOUT>) _param_vte_tgt_tout,
		(ParamFloat<px4::params::VTE_M_REC_TOUT>) _param_vte_mrec_tout,
		(ParamFloat<px4::params::VTE_M_UPD_TOUT>) _param_vte_mupd_tout,
		(ParamFloat<px4::params::VTE_POS_RATE>) _param_vte_pos_rate,
		(ParamFloat<px4::params::VTE_YAW_RATE>) _param_vte_yaw_rate,
		(ParamInt<px4::params::VTE_AID_MASK>) _param_vte_aid_mask
	)
};

} // namespace vision_target_estimator

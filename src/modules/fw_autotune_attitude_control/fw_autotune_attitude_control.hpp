/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file fw_autotune_attitude_control.hpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/pid_design/pid_design.hpp>
#include <lib/system_identification/system_identification.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <mathlib/mathlib.h>
#include <lib/systemlib/mavlink_log.h>

using namespace time_literals;

class FwAutotuneAttitudeControl : public ModuleBase<FwAutotuneAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FwAutotuneAttitudeControl(bool is_vtol);
	~FwAutotuneAttitudeControl() override;

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
	enum Axes : int32_t {
		roll = (1 << 0),
		pitch = (1 << 1),
		yaw = (1 << 2)
	};

	void Run() override;

	void reset();

	void checkFilters();

	void updateStateMachine(hrt_abstime now);
	void copyGains(int index);
	bool areGainsGood() const;
	void saveGainsToParams();
	void backupAndSaveGainsToParams();
	void revertParamGains();
	bool isAuxEnableSwitchEnabled();

	const matrix::Vector3f getIdentificationSignal();

	uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub;
	uORB::SubscriptionCallbackWorkItem _parameter_update_sub{this, ORB_ID(parameter_update)};

	uORB::Subscription _actuator_controls_status_sub;
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::PublicationData<autotune_attitude_control_status_s> _autotune_attitude_control_status_pub{ORB_ID(autotune_attitude_control_status)};

	SystemIdentification _sys_id;

	enum class state {
		idle = autotune_attitude_control_status_s::STATE_IDLE,
		init = autotune_attitude_control_status_s::STATE_INIT,
		roll = autotune_attitude_control_status_s::STATE_ROLL,
		roll_pause = autotune_attitude_control_status_s::STATE_ROLL_PAUSE,
		pitch = autotune_attitude_control_status_s::STATE_PITCH,
		pitch_pause = autotune_attitude_control_status_s::STATE_PITCH_PAUSE,
		yaw = autotune_attitude_control_status_s::STATE_YAW,
		yaw_pause = autotune_attitude_control_status_s::STATE_YAW_PAUSE,
		apply = autotune_attitude_control_status_s::STATE_APPLY,
		test = autotune_attitude_control_status_s::STATE_TEST,
		verification = autotune_attitude_control_status_s::STATE_VERIFICATION,
		complete = autotune_attitude_control_status_s::STATE_COMPLETE,
		fail = autotune_attitude_control_status_s::STATE_FAIL,
		wait_for_disarm = autotune_attitude_control_status_s::STATE_WAIT_FOR_DISARM
	} _state{state::idle};

	hrt_abstime _state_start_time{0};
	uint8_t _steps_counter{0};
	uint8_t _max_steps{5};
	int8_t _signal_sign{0};

	bool _armed{false};
	uint8_t _nav_state{0};
	uint8_t _start_flight_mode{0};
	bool _aux_switch_en{false};

	orb_advert_t _mavlink_log_pub{nullptr};

	matrix::Vector3f _kiff{};
	matrix::Vector3f _rate_k{};
	matrix::Vector3f _rate_i{};
	matrix::Vector3f _rate_ff{};

	float _attitude_p{0.f};
	matrix::Vector3f _att_p{};

	matrix::Vector3f _control_power{};

	bool _gains_backup_available{false}; // true if a backup of the parameters has been done

	/**
	 * Scale factor applied to the input data to have the same input/output range
	 * When input and output scales are a lot different, some elements of the covariance
	 * matrix will collapse much faster than other ones, creating an ill-conditionned matrix
	 */
	float _input_scale{1.f};

	hrt_abstime _last_run{0};
	hrt_abstime _last_publish{0};

	float _interval_sum{0.f};
	float _interval_count{0.f};
	float _sample_interval_avg{0.01f};
	float _filter_sample_rate{1.f};
	bool _are_filters_initialized{false};

	AlphaFilter<float> _signal_filter; ///< used to create a wash-out filter

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FW_AT_AXES>) _param_fw_at_axes,
		(ParamBool<px4::params::FW_AT_START>) _param_fw_at_start,
		(ParamInt<px4::params::FW_AT_MAN_AUX>) _param_fw_at_man_aux,
		(ParamFloat<px4::params::FW_AT_SYSID_AMP>) _param_fw_at_sysid_amp,
		(ParamInt<px4::params::FW_AT_APPLY>) _param_fw_at_apply,

		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff,

		(ParamFloat<px4::params::FW_RR_P>) _param_fw_rr_p,
		(ParamFloat<px4::params::FW_RR_I>) _param_fw_rr_i,
		(ParamFloat<px4::params::FW_RR_FF>) _param_fw_rr_ff,
		(ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc,
		(ParamFloat<px4::params::FW_PR_P>) _param_fw_pr_p,
		(ParamFloat<px4::params::FW_PR_I>) _param_fw_pr_i,
		(ParamFloat<px4::params::FW_PR_FF>) _param_fw_pr_ff,
		(ParamFloat<px4::params::FW_P_TC>) _param_fw_p_tc,
		(ParamFloat<px4::params::FW_YR_P>) _param_fw_yr_p,
		(ParamFloat<px4::params::FW_YR_I>) _param_fw_yr_i,
		(ParamFloat<px4::params::FW_YR_FF>) _param_fw_yr_ff
	)

	static constexpr float _publishing_dt_s = 100e-3f;
	static constexpr hrt_abstime _publishing_dt_hrt = 100_ms;
};

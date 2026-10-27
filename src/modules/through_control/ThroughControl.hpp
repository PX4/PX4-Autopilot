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

/**
 * @file ThroughControl.hpp
 *
 * THROUGH flight mode: track a colored ring with the gimbal camera and dive
 * through it using proportional navigation.
 *
 * Runs at a fixed 100 Hz (ScheduledWorkItem) rather than on vehicle_attitude
 * callbacks: the mode logic has to keep answering arming check requests and
 * tracking the phase timers even when estimator topics stall.
 *
 * Phases: ALIGN (hover, yaw to the LOS azimuth, gimbal tracking) -> RAMP
 * (blend hover into PN guidance, ramp the base thrust) -> GUIDE (PN) ->
 * TERMINAL (frozen attitude and thrust) -> EXIT (switch to THRU_EXIT_MODE).
 */

#pragma once

#include "GimbalTracker.hpp"
#include "LosEstimator.hpp"
#include "ModeRegistration.hpp"
#include "PnGuidance.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/through_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vision_target.h>

using namespace time_literals;

class ThroughControl : public ModuleBase<ThroughControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ThroughControl();
	~ThroughControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	static constexpr hrt_abstime RUN_INTERVAL = 10_ms; // 100 Hz
	static constexpr hrt_abstime EXIT_RETRY_INTERVAL = 1_s;
	static constexpr hrt_abstime OPENING_TIMEOUT = 500_ms; ///< GUIDE: vc < 0 for this long -> EXIT (spec 8.2)

	using Phase = uint8_t;   ///< through_status_s::PHASE_*
	using Reason = uint8_t;  ///< through_status_s::REASON_*

	void Run() override;

	void updateParams() override;
	void updateSubscriptions(hrt_abstime now);

	/** Condition one (spec 9.2), evaluated while the mode is not active */
	through::ModeRegistration::ArmingFailure checkConditionOne(hrt_abstime now) const;

	bool targetFresh(hrt_abstime now) const;
	float altitude() const;

	void enterPhase(Phase phase, hrt_abstime now);
	void enterExit(Reason reason, hrt_abstime now);
	void runAlign(hrt_abstime now, float dt);
	void runRamp(hrt_abstime now, float dt);
	void runGuide(hrt_abstime now, float dt);
	void runTerminal(hrt_abstime now);
	void runExit(hrt_abstime now);

	/**
	 * Exit conditions shared by ALIGN, RAMP and GUIDE (unlock, target lost, minimum altitude).
	 * In GUIDE a lost target close to the ring leads to TERMINAL instead.
	 * @return true if the phase changed
	 */
	bool checkCommonExits(hrt_abstime now);

	/** Vehicle velocity (NED), zero for invalid components. */
	matrix::Vector3f velocity() const;

	/** RAMP/GUIDE command: (1 - alpha) * a_hover + alpha * a_guide, base thrust ramped with alpha. */
	void publishGuidanceSetpoint(hrt_abstime now, float alpha);

	/** Feed new images to the LOS estimator (ALIGN onwards). */
	void updateLos();

	/** Yaw towards the LOS azimuth and point the gimbal along the LOS (spec 7 step 6/7). */
	void trackTarget(hrt_abstime now, float dt);

	/** Measured camera pitch relative to the body x axis [rad] (+ = down), NAN if no fresh feedback. */
	float measuredGimbalPitch(hrt_abstime now) const;

	void publishHoverSetpoint(hrt_abstime now, float dt);
	void publishAttitudeSetpoint(const through::AttitudeCommand &cmd, hrt_abstime now);
	void sendExitModeCommand(hrt_abstime now);
	void checkExitCommandAck();
	void publishStatus(hrt_abstime now);

	static const char *phaseString(Phase phase);
	static const char *reasonString(Reason reason);

	through::ModeRegistration _mode_registration{"THROUGH"};
	through::PnGuidance _guidance{};
	through::LosEstimator _los{};
	through::GimbalTracker _gimbal{};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vision_target_sub{ORB_ID(vision_target)};
	uORB::Subscription _gimbal_device_attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};

	// Publications
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<through_status_s> _through_status_pub{ORB_ID(through_status)};

	orb_advert_t _mavlink_log_pub{nullptr};

	// Latest inputs
	vehicle_local_position_s _local_pos{};
	vehicle_attitude_s _attitude{};
	vision_target_s _vision_target{};
	hover_thrust_estimate_s _hover_thrust_estimate{};
	uint8_t _nav_state{vehicle_status_s::NAVIGATION_STATE_MAX};
	bool _armed{false};
	bool _landed{true};

	bool _vision_target_new{false};                ///< received but not yet given to the LOS estimator
	hrt_abstime _vision_target_last_time{0};       ///< reception time of the latest vision_target
	hrt_abstime _vision_target_last_valid_time{0}; ///< reception time of the latest valid detection
	float _last_valid_target_size{0.f};

	// Mode state
	Phase _phase{through_status_s::PHASE_IDLE};
	Reason _exit_reason{through_status_s::REASON_NONE};
	hrt_abstime _mode_start_time{0};
	hrt_abstime _phase_start_time{0};
	hrt_abstime _last_run_time{0};
	hrt_abstime _last_status_publish_time{0};

	float _hover_thrust{0.5f};  ///< T_h latched on mode entry (VERIFICATION.md D4)
	float _yaw_sp{0.f};
	float _yaw_target{0.f};     ///< LOS azimuth the yaw setpoint is rate limited towards
	hrt_abstime _aligned_since{0};
	hrt_abstime _saturated_since{0};  ///< GUIDE: tilt or thrust saturated since (0 = not saturated)
	hrt_abstime _opening_since{0};    ///< GUIDE: vc < 0 since (0 = closing)
	float _ramp_alpha{0.f};
	through::AttitudeCommand _last_cmd{};
	through::GuidanceCommand _last_guidance{};
	bool _guidance_active{false};     ///< _last_guidance is from this cycle (RAMP/GUIDE)

	matrix::Quatf _q_gimbal{};  ///< latest camera attitude feedback (camera FRD -> NED)
	hrt_abstime _gimbal_status_time{0};

	hrt_abstime _exit_command_time{0};
	bool _exit_command_acked{false};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::THRU_THR_BASE>) _param_thru_thr_base,
		(ParamFloat<px4::params::THRU_THR_MIN>) _param_thru_thr_min,
		(ParamFloat<px4::params::THRU_THR_MAX>) _param_thru_thr_max,
		(ParamFloat<px4::params::THRU_PN_N>) _param_thru_pn_n,
		(ParamFloat<px4::params::THRU_A_MAX>) _param_thru_a_max,
		(ParamFloat<px4::params::THRU_V_MAX>) _param_thru_v_max,
		(ParamFloat<px4::params::THRU_K_VLIM>) _param_thru_k_vlim,
		(ParamFloat<px4::params::THRU_VC_MIN>) _param_thru_vc_min,
		(ParamFloat<px4::params::THRU_TILT_MAX>) _param_thru_tilt_max,
		(ParamFloat<px4::params::THRU_LOS_LPF>) _param_thru_los_lpf,
		(ParamFloat<px4::params::THRU_K_VDAMP>) _param_thru_k_vdamp,
		(ParamFloat<px4::params::THRU_YAW_RMAX>) _param_thru_yaw_rmax,
		(ParamFloat<px4::params::THRU_ALN_ERR>) _param_thru_aln_err,
		(ParamFloat<px4::params::THRU_ALN_T_OK>) _param_thru_aln_t_ok,
		(ParamFloat<px4::params::THRU_ALN_T_MAX>) _param_thru_aln_t_max,
		(ParamFloat<px4::params::THRU_RAMP_T>) _param_thru_ramp_t,
		(ParamFloat<px4::params::THRU_TERM_SZ>) _param_thru_term_sz,
		(ParamFloat<px4::params::THRU_TERM_NEAR>) _param_thru_term_near,
		(ParamFloat<px4::params::THRU_TERM_T>) _param_thru_term_t,
		(ParamFloat<px4::params::THRU_LOST_TO>) _param_thru_lost_to,
		(ParamFloat<px4::params::THRU_MIN_ALT>) _param_thru_min_alt,
		(ParamFloat<px4::params::THRU_SAT_T>) _param_thru_sat_t,
		(ParamFloat<px4::params::THRU_T_MAX>) _param_thru_t_max,
		(ParamFloat<px4::params::THRU_GMB_K>) _param_thru_gmb_k,
		(ParamInt<px4::params::THRU_EXIT_MODE>) _param_thru_exit_mode,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>) _param_mav_comp_id
	)
};

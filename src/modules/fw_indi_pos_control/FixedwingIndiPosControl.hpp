/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include <lib/rate_control/rate_control.hpp>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

using namespace time_literals;

class FixedwingIndiPosControl final : public ModuleBase<FixedwingIndiPosControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FixedwingIndiPosControl(bool vtol = false);
	~FixedwingIndiPosControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void		airspeed_poll();
	void		vehicle_acceleration_poll();
	void 		vehicle_local_position_poll();
	void		vehicle_attitude_poll();
	void		vehicle_status_poll();


	matrix::Quatf get_flat_attitude(matrix::Vector3f vel, matrix::Vector3f f, float rho_corrected);

	matrix::Quatf computeIndi(matrix::Vector3f pos_ref, matrix::Vector3f vel_ref, matrix::Vector3f acc_ref);

	matrix::Vector3f controlAttitude(matrix::Quatf ref_attitude);

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};     	// linear acceleration
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	uORB::SubscriptionMultiArray<control_allocator_status_s, 2> _control_allocator_status_subs{ORB_ID::control_allocator_status};


	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<offboard_control_mode_s>	_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

	manual_control_setpoint_s		_manual_control_setpoint{};
	vehicle_control_mode_s			_vcontrol_mode{};
	vehicle_rates_setpoint_s		_rates_sp{};
	vehicle_status_s			_vehicle_status{};

	perf_counter_t _loop_perf;

	hrt_abstime _last_run{0};
	hrt_abstime _airspeed_last_valid{0};			///< last time airspeed was received. Used to detect timeouts.

	// controller frequency
	const float _sample_frequency = 200.f;
	// Low-Pass filters stage 1
	const float _cutoff_frequency_1 = 20.f;
	// smoothing filter to reject HF noise in control output
	const float _cutoff_frequency_smoothing =
		20.f; // we want to attenuate noise at 30Hz with -10dB -> need cutoff frequency 5 times lower (6Hz)
	math::LowPassFilter2p<float> _lp_filter_accel[3] {{_sample_frequency, _cutoff_frequency_1}, {_sample_frequency, _cutoff_frequency_1}, {_sample_frequency, _cutoff_frequency_1}};	// linear acceleration
	math::LowPassFilter2p<float> _lp_filter_force[3] {{_sample_frequency, _cutoff_frequency_1}, {_sample_frequency, _cutoff_frequency_1}, {_sample_frequency, _cutoff_frequency_1}};	// force command
	math::LowPassFilter2p<float> _lp_filter_omega[3] {{_sample_frequency, _cutoff_frequency_1}, {_sample_frequency, _cutoff_frequency_1}, {_sample_frequency, _cutoff_frequency_1}};	// body rates
	math::LowPassFilter2p<float> _lp_filter_ctrl0[3] {{_sample_frequency, _cutoff_frequency_smoothing}, {_sample_frequency, _cutoff_frequency_smoothing}, {_sample_frequency, _cutoff_frequency_smoothing}};	// force command stage 1
	math::LowPassFilter2p<float> _lp_filter_rud {_sample_frequency, 10};	// rudder command

	//Path parameters
	matrix::Vector3f segment_start;


	matrix::Vector3f vehicle_position_;
	matrix::Vector3f vehicle_velocity_;
	matrix::Vector3f _acc;
	matrix::Vector3f wind_estimate_{0.0, 0.0, 0.0};
	matrix::Quatf vehicle_attitude_;

	float _cal_airspeed{0.0f};
	float _rho{1.225f};
	float _true_airspeed{1.0};


	matrix::Vector3f pos_ref_;
	matrix::Vector3f vel_ref_;
	matrix::Vector3f acc_ref_;

	matrix::Vector3f _f_command {};
	matrix::Vector3f _m_command {};
	matrix::Vector3f acc_filtered_{};

	matrix::Matrix3f _K_x;
	matrix::Matrix3f _K_v;
	matrix::Matrix3f _K_a;
	matrix::Matrix3f _K_q;

	// Vehicle parameters (From Easyglider)
	float _aoa_offset{0.07};
	float _C_L0{0.3};
	float _C_D0{0.0288};
	float _C_L1{2.354};
	float _C_D1{0.3783};
	float _C_D2{1.984};
	float _area{0.4};
	float _stall_speed{1.0};
	float _mass{1.5};

	bool _landed{true};
	bool _switch_saturation{true};
	bool _airspeed_valid{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_INDI_MASS>) _param_fw_mass,
		(ParamFloat<px4::params::FW_INDI_KP_X>) _param_kp_x,
		(ParamFloat<px4::params::FW_INDI_KP_Y>) _param_kp_y,
		(ParamFloat<px4::params::FW_INDI_KP_Z>) _param_kp_z,
		(ParamFloat<px4::params::FW_INDI_KV_X>) _param_kv_x,
		(ParamFloat<px4::params::FW_INDI_KV_Y>) _param_kv_y,
		(ParamFloat<px4::params::FW_INDI_KV_Z>) _param_kv_z,
		(ParamFloat<px4::params::FW_INDI_K_ROLL>) _param_rot_k_roll,
		(ParamFloat<px4::params::FW_INDI_K_PITCH>) _param_rot_k_pitch
	)
};

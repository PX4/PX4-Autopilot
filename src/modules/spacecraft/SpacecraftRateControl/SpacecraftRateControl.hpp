/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class SpacecraftRateControl : public ModuleParams
{
public:
	SpacecraftRateControl(ModuleParams *parent);
	~SpacecraftRateControl() = default;

	/**
	 * @brief Update rate controller.
	 */
	void updateRateControl();

protected:
	void updateParams() override;

private:
	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};

	vehicle_control_mode_s		_vehicle_control_mode{};
	vehicle_status_s		_vehicle_status{};
	vehicle_angular_velocity_s 	angular_velocity{};

	bool _landed{true};
	bool _maybe_landed{true};

	hrt_abstime _last_run{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};
	float _manual_torque_max{1.0};
	float _manual_force_max{1.0};

	float _battery_status_scale{0.0f};
	matrix::Vector3f _thrust_setpoint{};
	matrix::Vector3f _torque_setpoint{};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SC_ROLLRATE_P>) _param_sc_rollrate_p,
		(ParamFloat<px4::params::SC_ROLLRATE_I>) _param_sc_rollrate_i,
		(ParamFloat<px4::params::SC_RR_INT_LIM>) _param_sc_rr_int_lim,
		(ParamFloat<px4::params::SC_ROLLRATE_D>) _param_sc_rollrate_d,
		(ParamFloat<px4::params::SC_ROLLRATE_FF>) _param_sc_rollrate_ff,
		(ParamFloat<px4::params::SC_ROLLRATE_K>) _param_sc_rollrate_k,

		(ParamFloat<px4::params::SC_PITCHRATE_P>) _param_sc_pitchrate_p,
		(ParamFloat<px4::params::SC_PITCHRATE_I>) _param_sc_pitchrate_i,
		(ParamFloat<px4::params::SC_PR_INT_LIM>) _param_sc_pr_int_lim,
		(ParamFloat<px4::params::SC_PITCHRATE_D>) _param_sc_pitchrate_d,
		(ParamFloat<px4::params::SC_PITCHRATE_FF>) _param_sc_pitchrate_ff,
		(ParamFloat<px4::params::SC_PITCHRATE_K>) _param_sc_pitchrate_k,

		(ParamFloat<px4::params::SC_YAWRATE_P>) _param_sc_yawrate_p,
		(ParamFloat<px4::params::SC_YAWRATE_I>) _param_sc_yawrate_i,
		(ParamFloat<px4::params::SC_YR_INT_LIM>) _param_sc_yr_int_lim,
		(ParamFloat<px4::params::SC_YAWRATE_D>) _param_sc_yawrate_d,
		(ParamFloat<px4::params::SC_YAWRATE_FF>) _param_sc_yawrate_ff,
		(ParamFloat<px4::params::SC_YAWRATE_K>) _param_sc_yawrate_k,

		(ParamFloat<px4::params::SC_ACRO_R_MAX>) _param_sc_acro_r_max,
		(ParamFloat<px4::params::SC_ACRO_P_MAX>) _param_sc_acro_p_max,
		(ParamFloat<px4::params::SC_ACRO_Y_MAX>) _param_sc_acro_y_max,
		(ParamFloat<px4::params::SC_ACRO_EXPO>) _param_sc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::SC_ACRO_EXPO_Y>) _param_sc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::SC_ACRO_SUPEXPO>) _param_sc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::SC_ACRO_SUPEXPOY>) _param_sc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */

		(ParamFloat<px4::params::SC_MAN_F_MAX>) _param_sc_manual_f_max,
		(ParamFloat<px4::params::SC_MAN_T_MAX>) _param_sc_manual_t_max,

		(ParamBool<px4::params::SC_BAT_SCALE_EN>) _param_sc_bat_scale_en
	)
};

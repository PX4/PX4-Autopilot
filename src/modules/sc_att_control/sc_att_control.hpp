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

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

#include <AttitudeControl.hpp>

using namespace time_literals;

class SpacecraftAttitudeControl : public ModuleBase<SpacecraftAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	SpacecraftAttitudeControl();
	~SpacecraftAttitudeControl() override;

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
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	float throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt, bool reset_yaw_sp);

	ScAttitudeControl _attitude_control; /**< class for attitude control calculations */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Attitude setpoint and current attitude sub
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};

	// Manual control setpoint sub
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	// Vehicle control mode sub and status
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Vehicle local position sub
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// Publish rate setpoint for rate controller and att_control status
	uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};    /**< rate setpoint publication */
	uORB::Publication<vehicle_attitude_setpoint_s>  _vehicle_attitude_setpoint_pub;

	manual_control_setpoint_s       _manual_control_setpoint {};    /**< manual control setpoint */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< vehicle control mode */

	perf_counter_t  _loop_perf;             /**< loop duration performance counter */

	matrix::Vector3f _thrust_setpoint_body; /**< body frame 3D thrust vector */

	float _man_yaw_sp{0.f};                 /**< current yaw setpoint in manual mode */
	float _man_tilt_max;                    /**< maximum tilt allowed for manual flight [rad] */

	AlphaFilter<float> _man_roll_input_filter;
	AlphaFilter<float> _man_pitch_input_filter;

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};

	bool _reset_yaw_sp{true};
	bool _heading_good_for_control{true}; ///< initialized true to have heading lock when local position never published
	bool _vehicle_type_spacecraft{true};

	uint8_t _quat_reset_counter{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SC_MAN_TILT_TAU>)  _param_sc_man_tilt_tau,

		(ParamFloat<px4::params::SC_ROLL_P>)        _param_sc_roll_p,
		(ParamFloat<px4::params::SC_PITCH_P>)       _param_sc_pitch_p,
		(ParamFloat<px4::params::SC_YAW_P>)         _param_sc_yaw_p,
		(ParamFloat<px4::params::SC_YAW_WEIGHT>)    _param_sc_yaw_weight,

		(ParamFloat<px4::params::SC_ROLLRATE_MAX>)  _param_sc_rollrate_max,
		(ParamFloat<px4::params::SC_PITCHRATE_MAX>) _param_sc_pitchrate_max,
		(ParamFloat<px4::params::SC_YAWRATE_MAX>)   _param_sc_yawrate_max,
		(ParamFloat<px4::params::SC_MAN_TILT_MAX>)  _param_sc_man_tilt_max,

		/* Stabilized mode params */
		(ParamFloat<px4::params::SC_MAN_Y_SCALE>)    _param_sc_man_y_scale /**< scaling factor from stick to yaw rate */
	)
};


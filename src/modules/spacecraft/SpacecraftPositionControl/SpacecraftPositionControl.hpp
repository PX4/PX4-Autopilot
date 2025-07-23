/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * Multicopter position controller.
 */

#pragma once

#include "PositionControl/PositionControl.hpp"

#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint6dof.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class SpacecraftPositionControl : public ModuleParams
{
public:
	SpacecraftPositionControl(ModuleParams *parent);
	~SpacecraftPositionControl() = default;

	void updatePositionControl();

protected:
	/**
	 * Update our local parameter cache.
	 */
	void updateParams();

private:

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */

	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};	/**< vehicle local position */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; 	/**< notification of manual control updates */

	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint6dof)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	hrt_abstime _time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _time_position_control_enabled{0};
	hrt_abstime _manual_setpoint_last_called{0};

	trajectory_setpoint6dof_s 	_setpoint{ScPositionControl::empty_trajectory_setpoint};
	vehicle_control_mode_s 		_vehicle_control_mode{};
	manual_control_setpoint_s	_manual_control_setpoint{};			    /**< r/c channel data */

	DEFINE_PARAMETERS(
		// Position Control
		(ParamFloat<px4::params::SPC_POS_P>)        _param_mpc_pos_p,
		(ParamFloat<px4::params::SPC_POS_I>)        _param_mpc_pos_i,
		(ParamFloat<px4::params::SPC_POS_I_LIM>)    _param_mpc_pos_i_lim,
		(ParamFloat<px4::params::SPC_VEL_P>) 		_param_mpc_vel_p_acc,
		(ParamFloat<px4::params::SPC_VEL_I>) 		_param_mpc_vel_i_acc,
		(ParamFloat<px4::params::SPC_VEL_I_LIM>)    _param_mpc_vel_i_lim,
		(ParamFloat<px4::params::SPC_VEL_D>) 		_param_mpc_vel_d_acc,
		(ParamFloat<px4::params::SPC_VEL_ALL>)   	_param_mpc_vel_all,
		(ParamFloat<px4::params::SPC_VEL_MAX>)   	_param_mpc_vel_max,
		(ParamFloat<px4::params::SPC_VEL_CRUISE>)   _param_mpc_vel_cruise,
		(ParamFloat<px4::params::SPC_VEL_MANUAL>)   _param_mpc_vel_manual,
		(ParamFloat<px4::params::SPC_VEHICLE_RESP>) _param_sys_vehicle_resp,
		(ParamFloat<px4::params::SPC_ACC>)      	_param_mpc_acc,
		(ParamFloat<px4::params::SPC_ACC_MAX>)      _param_mpc_acc_max,
		(ParamFloat<px4::params::SPC_MAN_Y_MAX>)    _param_mpc_man_y_max,
		(ParamFloat<px4::params::SPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,
		(ParamFloat<px4::params::SPC_JERK_AUTO>)    _param_mpc_jerk_auto,
		(ParamFloat<px4::params::SPC_JERK_MAX>)     _param_mpc_jerk_max,
		(ParamFloat<px4::params::SPC_THR_MAX>)      _param_mpc_thr_max
	);

	matrix::Vector3f target_pos_sp;
	float yaw_rate;
	bool stabilized_pos_sp_initialized{false};

	ScPositionControl _control;  /**< class for core PID position control */

	hrt_abstime _last_warn{0}; /**< timer when the last warn message was sent out */

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// Manual setpoints on yaw and reset
	bool _reset_yaw_sp{true};
	float _manual_yaw_sp{0.f};
	float _throttle_control{0.f};
	float _yaw_control{0.f};

	/**
	 * Check for validity of positon/velocity states.
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, const vehicle_attitude_s &att);

	/**
	 * Check for manual setpoints.
	 */
	void poll_manual_setpoint(const float dt, const vehicle_local_position_s
				  &vehicle_local_position, const vehicle_attitude_s &_vehicle_att);

	/**
	 * @brief publishes target setpoint.
	 *
	 */
	void publishLocalPositionSetpoint(vehicle_attitude_setpoint_s &_att_sp);

	/**
	 * Generate setpoint to bridge no executable setpoint being available.
	 * Used to handle transitions where no proper setpoint was generated yet and when the received setpoint is invalid.
	 * This should only happen briefly when transitioning and never during mode operation or by design.
	 */
	trajectory_setpoint6dof_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states,
			bool warn);
};

/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the hippocampus control module and is designed for the
 * BlueROV2.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Tim Hansen <t.hansen@jacobs-university.de>
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */
#pragma once
#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/pid/pid.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/trajectory_setpoint.h>

#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>

// huh??
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/uORB.h>

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using uORB::SubscriptionData;

using namespace time_literals;

class USVOmniControl: public ModuleBase<USVOmniControl>, public ModuleParams, public px4::WorkItem
{
public:
	USVOmniControl();
	~USVOmniControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	// for logging maybe?
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	// the most important
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< notification of manual control updates */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

	vehicle_attitude_s _vehicle_attitude{};

	// Inputs, setpoints
	manual_control_setpoint_s	_manual_control_sp{};	/**< r/c channel data */
	position_setpoint_triplet_s	_pos_sp_triplet{};	/**< triplet of mission items */
	vehicle_attitude_setpoint_s	_att_sp{};		/**< attitude setpoint > */
	// State
	vehicle_control_mode_s		_control_mode{};	/**< control mode */
	vehicle_global_position_s	_global_pos{};		/**< global vehicle position */
	vehicle_local_position_s	_local_pos{};		/**< local vehicle position */
	vehicle_attitude_s		_att{};

	matrix::Vector3f _thrust_setpoint{};
	// matrix::Vector3f _torque_setpoint{};

	perf_counter_t	_loop_perf;

	DEFINE_PARAMETERS(
		// These are used as a hacky way to not oversaturate c
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_scale_thrust_ca,
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_scale_torque_ca,
		// Position Control
		(ParamFloat<px4::params::USV_XY_P>)         _param_usv_xy_p,
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_usv_xy_vel_p_acc,
		(ParamFloat<px4::params::USV_XY_VEL_I_ACC>) _param_usv_xy_vel_i_acc,
		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_usv_xy_vel_d_acc,
		(ParamFloat<px4::params::USV_XY_VEL_MAX>)   _param_usv_xy_vel_max,

		// TODO: refactor, delete

		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_pose_gain_y,
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_pose_gain_z,
		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_pose_gain_d_x,
		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_pose_gain_d_y,
		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_pose_gain_d_z,

		(ParamInt<px4::params::USV_INPUT_MODE>) _param_input_mode,
		(ParamInt<px4::params::USV_STAB_MODE>) _param_stabilization,
		(ParamInt<px4::params::USV_SKIP_CTRL>) _param_skip_ctrl
	)

	void Run() override;

	void parameters_update(bool force = false);

	/**
	 * Control Attitude
	 */
	void publishAttitudeSetpoint(const float thrust_x, const float thrust_y, const float thrust_z,
				       const float roll_des, const float pitch_des, const float yaw_des);
	void pose_controller_6dof(const Vector3f &pos_des,
				  const float roll_des, const float pitch_des, const float yaw_des,
				  vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos);
	void stabilization_controller_6dof(const Vector3f &pos_des,
					   const float roll_des, const float pitch_des, const float yaw_des,
					   vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos);

	/**
	 * Setpoint handlers
	 */
	void handleManualInputs();
	void handleVelocityInputs();
	void handlePositionInputs();

	/**
	 * Control
	 */
	bool controlPosition(const matrix::Vector2d &global_pos, const matrix::Vector3f &ground_speed,
					 const position_setpoint_triplet_s &_pos_sp_triplet);
	void controlVelocity(const matrix::Vector3f &current_velocity);
	void controlAttitude(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp);

	// Output setpoints
	void publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);
};

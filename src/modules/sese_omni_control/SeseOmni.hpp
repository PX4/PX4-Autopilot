/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/differential_drive_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <lib/pid/pid.h>

using namespace time_literals;

class SeseOmni : public ModuleBase<SeseOmni>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SeseOmni();
	~SeseOmni() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	void updateParams() override;

private:
	void Run() override;
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

	uORB::Publication<differential_drive_setpoint_s> _differential_drive_setpoint_pub{ORB_ID(differential_drive_setpoint)};

	// Add Publications for control allocator
	uORB::Publication<actuator_controls_status_s> _actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)}; /**< vehicle torque setpoint publication */
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)}; /**< vehicle thrust setpoint publication */

	bool _manual_driving = false;
	bool _mission_driving = false;
	bool _acro_driving = false;
	bool _position_control = false;
	vehicle_local_position_s _local_pos{};
	hrt_abstime _time_stamp_last{0}; /**< time stamp when task was last updated */

	// PID attitude controller
	PID_t _att_pid{};
	// PIDs position controller
	PID_t _x_pos_pid{};
	PID_t _y_pos_pid{};

	// PIDs velocity controller
	PID_t _x_velocity_pid{};
	PID_t _y_velocity_pid{};

	float _max_speed{0.1f};
	float _max_angular_velocity{0.1f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ATT_P_GAIN>) att_p_gain,
		(ParamFloat<px4::params::ATT_I_GAIN>) att_i_gain,
		(ParamFloat<px4::params::ATT_D_GAIN>) att_d_gain,
		(ParamFloat<px4::params::X_POS_P_GAIN>) x_pos_p_gain,
		(ParamFloat<px4::params::X_POS_I_GAIN>) x_pos_i_gain,
		(ParamFloat<px4::params::X_POS_D_GAIN>) x_pos_d_gain,
		(ParamFloat<px4::params::Y_POS_P_GAIN>) y_pos_p_gain,
		(ParamFloat<px4::params::Y_POS_I_GAIN>) y_pos_i_gain,
		(ParamFloat<px4::params::Y_POS_D_GAIN>) y_pos_d_gain,
		(ParamFloat<px4::params::X_VEL_P_GAIN>) x_velocity_p_gain,
		(ParamFloat<px4::params::X_VEL_I_GAIN>) x_velocity_i_gain,
		(ParamFloat<px4::params::X_VEL_D_GAIN>) x_velocity_d_gain,
		(ParamFloat<px4::params::Y_VEL_P_GAIN>) y_velocity_p_gain,
		(ParamFloat<px4::params::Y_VEL_I_GAIN>) y_velocity_i_gain,
		(ParamFloat<px4::params::Y_VEL_D_GAIN>) y_velocity_d_gain,
		(ParamFloat<px4::params::HEADING_SP>) heading_sp,
		(ParamFloat<px4::params::X_POS_SP>) x_pos_sp,
		(ParamFloat<px4::params::Y_POS_SP>) y_pos_sp,
		(ParamFloat<px4::params::THRUST_SCALING>) thrust_scaling,
		(ParamFloat<px4::params::TORQUE_SCALING>) torque_scaling

	);
};

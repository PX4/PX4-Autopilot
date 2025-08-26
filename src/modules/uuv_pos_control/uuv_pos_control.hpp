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

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
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
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint6dof.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/uORB.h>

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using uORB::SubscriptionData;

using namespace time_literals;

class UUVPOSControl: public ModuleBase<UUVPOSControl>, public ModuleParams, public px4::WorkItem
{
public:
	UUVPOSControl();
	~UUVPOSControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint6dof)};
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< notification of manual control updates */

	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

	vehicle_attitude_s _vehicle_attitude{};
	trajectory_setpoint6dof_s _trajectory_setpoint{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_control_mode_s _vcontrol_mode{};
	vehicle_attitude_setpoint_s _attitude_setpoint{};

	perf_counter_t	_loop_perf;
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::UUV_GAIN_X_P>) _param_pose_gain_x,
		(ParamFloat<px4::params::UUV_GAIN_Y_P>) _param_pose_gain_y,
		(ParamFloat<px4::params::UUV_GAIN_Z_P>) _param_pose_gain_z,
		(ParamFloat<px4::params::UUV_GAIN_X_D>) _param_pose_gain_d_x,
		(ParamFloat<px4::params::UUV_GAIN_Y_D>) _param_pose_gain_d_y,
		(ParamFloat<px4::params::UUV_GAIN_Z_D>) _param_pose_gain_d_z,
		(ParamInt<px4::params::UUV_STAB_MODE>) _param_stab_mode,
		(ParamFloat<px4::params::UUV_POS_STICK_DB>) _param_pos_stick_db,
		(ParamFloat<px4::params::UUV_PGM_VEL>) _param_pgm_vel,
		(ParamFloat<px4::params::UUV_SGM_ROLL>) _param_sgm_roll,
		(ParamFloat<px4::params::UUV_SGM_PITCH>) _param_sgm_pitch,
		(ParamFloat<px4::params::UUV_SGM_YAW>) _param_sgm_yaw,
		(ParamFloat<px4::params::UUV_SP_MAX_AGE>) _param_setpoint_max_age,
		(ParamInt<px4::params::UUV_POS_MODE>) _param_pos_mode
	)

	void Run() override;
	/**
	 * Update our local parameter cache.
	 */
	void parameters_update(bool force = false);

	/**
	 * Control Attitude
	 */
	void publish_attitude_setpoint(const float thrust_x, const float thrust_y, const float thrust_z,
				       const float roll_des, const float pitch_des, const float yaw_des);
	void pose_controller_6dof(const Vector3f &pos_des, vehicle_attitude_s &vehicle_attitude,
				  vehicle_local_position_s &vlocal_pos, bool altitude_mode);
	void stabilization_controller_6dof(const Vector3f &pos_des,
					   const float roll_des, const float pitch_des, const float yaw_des,
					   vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos);
	void generate_trajectory_setpoint(vehicle_local_position_s &vlocal_pos, vehicle_attitude_s &vehicle_attitude, float dt);
	void reset_trajectory_setpoint(vehicle_local_position_s &vlocal_pos);
	void check_setpoint_validity(vehicle_local_position_s &vlocal_pos);
};

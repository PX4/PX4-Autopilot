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
 * This module is a modification of the rover attitide control module and is designed for the
 * TUHH hippocampus.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 * @author Philipp Hastedt <philipp.hastedt@tuhh.de>
 * @author Tim Hansen <t.hansen@tuhh.de>
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
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/uORB.h>

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Vector2f;
using matrix::Dcmf;
using matrix::AxisAnglef;
using matrix::AxisAngle;

using uORB::SubscriptionData;

using namespace time_literals;

class UUVAttitudeControl: public ModuleBase<UUVAttitudeControl>, public ModuleParams, public px4::WorkItem
{
public:
	UUVAttitudeControl();
	~UUVAttitudeControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void publishTorqueSetpoint(const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);

	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};	/**< vehicle attitude setpoint */
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)}; /**< vehicle bodyrates setpoint subscriber */
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};	/**< vehicle angular velocity subscription */
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< notification of manual control updates */
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	vehicle_thrust_setpoint_s _vehicle_thrust_setpoint{};
	vehicle_torque_setpoint_s _vehicle_torque_setpoint{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_attitude_setpoint_s _attitude_setpoint{};
	vehicle_rates_setpoint_s _rates_setpoint{};
	vehicle_control_mode_s _vcontrol_mode{};

	perf_counter_t	_loop_perf;
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::UUV_ROLL_P>) _param_roll_p,
		(ParamFloat<px4::params::UUV_ROLL_D>) _param_roll_d,
		(ParamFloat<px4::params::UUV_PITCH_P>) _param_pitch_p,
		(ParamFloat<px4::params::UUV_PITCH_D>) _param_pitch_d,
		(ParamFloat<px4::params::UUV_YAW_P>) _param_yaw_p,
		(ParamFloat<px4::params::UUV_YAW_D>) _param_yaw_d,
		// gains for the different modes
		(ParamFloat<px4::params::UUV_SGM_ROLL>) _param_sgm_roll,
		(ParamFloat<px4::params::UUV_SGM_PITCH>) _param_sgm_pitch,
		(ParamFloat<px4::params::UUV_SGM_YAW>) _param_sgm_yaw,
		(ParamFloat<px4::params::UUV_SGM_THRTL>) _param_sgm_thrtl,
		(ParamFloat<px4::params::UUV_RGM_ROLL>) _param_rgm_roll,
		(ParamFloat<px4::params::UUV_RGM_PITCH>) _param_rgm_pitch,
		(ParamFloat<px4::params::UUV_RGM_YAW>) _param_rgm_yaw,
		(ParamFloat<px4::params::UUV_RGM_THRTL>) _param_rgm_thrtl,
		(ParamFloat<px4::params::UUV_MGM_ROLL>) _param_mgm_roll,
		(ParamFloat<px4::params::UUV_MGM_PITCH>) _param_mgm_pitch,
		(ParamFloat<px4::params::UUV_MGM_YAW>) _param_mgm_yaw,
		(ParamFloat<px4::params::UUV_MGM_THRTL>) _param_mgm_thrtl,
		(ParamFloat<px4::params::UUV_TORQUE_SAT>) _param_torque_sat,
		(ParamFloat<px4::params::UUV_THRUST_SAT>) _param_thrust_sat,
		(ParamFloat<px4::params::UUV_SP_MAX_AGE>) _param_setpoint_max_age

	)

	void Run() override;
	/**
	 * Update our local parameter cache.
	 */
	void parameters_update(bool force = false);

	/**
	 * Control Attitude
	 */
	void control_attitude_geo(const vehicle_attitude_s &attitude, const vehicle_attitude_setpoint_s &attitude_setpoint,
				  const vehicle_angular_velocity_s &angular_velocity, const vehicle_rates_setpoint_s &rates_setpoint,
				  bool attitude_control_enabled);
	void constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u,
					 float thrust_x, float thrust_y, float thrust_z);
	void generate_attitude_setpoint(float dt);
	void generate_rates_setpoint(float dt);
	void reset_attitude_setpoint(vehicle_attitude_s &v_att);
	void check_setpoint_validity(vehicle_attitude_s &v_att);
};

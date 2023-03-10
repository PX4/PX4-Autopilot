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
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};	/**< vehicle attitude setpoint */
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};


	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< notification of manual control updates */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<vehicle_acceleration_s>		_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};


	// Inputs, setpoints
	// manual_control_setpoint_s	_manual_control_sp{};	/**< r/c channel data */ not used globally,
	position_setpoint_triplet_s	_pos_sp_triplet{};	/**< triplet of mission items */
	vehicle_attitude_setpoint_s	_att_sp{};		/**< attitude setpoint > */
	trajectory_setpoint_s		_trajectory_sp{};	/**< trajectory setpoint > */
	// State
	vehicle_control_mode_s		_control_mode{};	/**< control mode */
	vehicle_global_position_s	_global_pos{};		/**< global vehicle position */
	vehicle_local_position_s	_local_pos{};		/**< local vehicle position */
	vehicle_attitude_s		_att{};

	matrix::Vector3f _thrust_setpoint{};
	matrix::Vector3f _torque_setpoint{};

	perf_counter_t	_loop_perf;

	hrt_abstime _control_position_last_called{0}; 	/**<last call of control_position  */
	hrt_abstime _manual_setpoint_last_called{0};

	MapProjection _global_local_proj_ref{};
	float         _global_local_alt0{NAN};

	/* Pid controller for the speed. Here we assume we can control airspeed but the control variable is actually on
	 the throttle. For now just assuming a proportional scaler between controlled airspeed and throttle output.*/
	PID_t _speed_ctrl{};

	enum UGV_POSCTRL_MODE {
		UGV_POSCTRL_MODE_AUTO,
		UGV_POSCTRL_MODE_OTHER
	} _control_mode_current{UGV_POSCTRL_MODE_OTHER};

	enum POS_CTRLSTATES {
		GOTO_WAYPOINT,
		STOPPING
	} _pos_ctrl_state {STOPPING};			/// Position control state machine

	/* previous waypoint */
	matrix::Vector2d _prev_wp{0, 0};

	enum class VelocityFrame {
		NED,
		BODY,
	} _velocity_frame{VelocityFrame::NED};

	float _manual_yaw_sp{0.0};
	bool _reset_yaw_sp{true};

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

		// TODO: refactor, delete, I thinkd I only need pose z, aka yaw
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_pose_gain_x,
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_pose_gain_y,
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_pose_gain_z,
		(ParamFloat<px4::params::USV_XY_VEL_P_ACC>) _param_yaw_p,

		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_pose_gain_d_x,
		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_pose_gain_d_y,
		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_pose_gain_d_z,

		(ParamFloat<px4::params::USV_XY_VEL_D_ACC>) _param_yaw_d,

		(ParamInt<px4::params::USV_INPUT_MODE>) _param_input_mode,
		(ParamInt<px4::params::USV_STAB_MODE>) _param_stabilization,
		(ParamInt<px4::params::USV_SKIP_CTRL>) _param_skip_ctrl,

		(ParamFloat<px4::params::GND_SPEED_TRIM>) _param_gndspeed_trim,
		(ParamFloat<px4::params::GND_SPEED_MAX>) _param_gndspeed_max,

		(ParamInt<px4::params::GND_SP_CTRL_MODE>) _param_speed_control_mode,
		(ParamFloat<px4::params::GND_SPEED_P>) _param_speed_p,
		(ParamFloat<px4::params::GND_SPEED_I>) _param_speed_i,
		(ParamFloat<px4::params::GND_SPEED_D>) _param_speed_d,
		(ParamFloat<px4::params::GND_SPEED_IMAX>) _param_speed_imax,
		(ParamFloat<px4::params::GND_SPEED_THR_SC>) _param_throttle_speed_scaler,

		(ParamFloat<px4::params::GND_THR_MIN>) _param_throttle_min,
		(ParamFloat<px4::params::GND_THR_MAX>) _param_throttle_max,
		(ParamFloat<px4::params::GND_THR_CRUISE>) _param_throttle_cruise,

		(ParamFloat<px4::params::GND_WHEEL_BASE>) _param_wheel_base,
		(ParamFloat<px4::params::GND_MAX_ANG>) _param_max_turn_angle,
		(ParamFloat<px4::params::GND_MAN_Y_MAX>) _param_gnd_man_y_max,
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad	/**< loiter radius for Rover */
	)

	void Run() override;

	void parameters_update(bool force = false);

	/**
	 * Things from UUV Position Controller
	 */
	void publishAttitudeSetpoint(const Vector3f &thrust_body_sp,
				     const float roll_des, const float pitch_des, const float yaw_des);
	/// @brief position controller(global + yaw)
	void poseController6dof(const Vector3f &pos_des,
				const float roll_des, const float pitch_des, const float yaw_des,
				vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos);
	/// @brief stabilization controller(keep pos and hold depth + angle)
	void stabilizationController6dof(const Vector3f &pos_des,
					 const float roll_des, const float pitch_des, const float yaw_des,
					 vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos);

	/**
	 * Setpoint handlers
	 */
	void handleManualInputs(const manual_control_setpoint_s &manual_control_setpoint);
	// TODO: change to Trajectory?
	void handleVelocityInputs();
	void handlePositionInputs(const matrix::Vector2d &current_position,
				  const matrix::Vector3f &ground_speed,
				  const position_setpoint_triplet_s &pos_sp_triplet);

	/**
	 * Control
	 */
	bool controlPosition(const matrix::Vector2d &global_pos, const matrix::Vector3f &ground_speed,
			     const position_setpoint_triplet_s &_pos_sp_triplet);
	void controlVelocity(const matrix::Vector3f &current_velocity);

	/// @brief Publishes to torque and thrust setpoints
	void controlAttitude(const vehicle_attitude_s &att,
			     const vehicle_attitude_setpoint_s &att_sp,
			     const vehicle_angular_velocity_s &angular_velocity,
			     const vehicle_rates_setpoint_s &rates_setpoint);

	// Output setpoints
	void publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);
};

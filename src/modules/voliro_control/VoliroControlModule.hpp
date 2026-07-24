#pragma once

#include "VoliroControl.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint6dof.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

class VoliroControlModule : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;
	VoliroControlModule();
	~VoliroControlModule() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	bool init();
	int print_status() override;

private:
	void Run() override;
	bool updateConfiguration();
	bool updateSetpoint();
	bool controlEnabled() const;
	bool setpointValid(hrt_abstime now) const;
	bool stateFromOdometry(const vehicle_odometry_s &odometry, VoliroControl::State &state) const;
	void publishWrench(const VoliroControl::Output &output, uint64_t timestamp_sample);

	uORB::SubscriptionCallbackWorkItem _odometry_sub{this, ORB_ID(vehicle_odometry)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint6dof)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1'000'000};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	VoliroControl _controller;
	trajectory_setpoint6dof_s _setpoint{};
	offboard_control_mode_s _offboard_control_mode{};
	vehicle_control_mode_s _vehicle_control_mode{};
	matrix::Vector3f _setpoint_angular_acceleration;
	matrix::Vector3f _previous_setpoint_angular_velocity;
	uint64_t _previous_setpoint_timestamp{0};
	hrt_abstime _last_valid_setpoint{0};
	VoliroControl::Output _last_output{};
	bool _active{false};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::VCTRL_EN>) _param_enable,
		(ParamFloat<px4::params::VCTRL_MASS>) _param_mass,
		(ParamFloat<px4::params::VCTRL_IXX>) _param_ixx,
		(ParamFloat<px4::params::VCTRL_IYY>) _param_iyy,
		(ParamFloat<px4::params::VCTRL_IZZ>) _param_izz,
		(ParamFloat<px4::params::VCTRL_POS_X>) _param_position_x,
		(ParamFloat<px4::params::VCTRL_POS_Y>) _param_position_y,
		(ParamFloat<px4::params::VCTRL_POS_Z>) _param_position_z,
		(ParamFloat<px4::params::VCTRL_VEL_X>) _param_velocity_x,
		(ParamFloat<px4::params::VCTRL_VEL_Y>) _param_velocity_y,
		(ParamFloat<px4::params::VCTRL_VEL_Z>) _param_velocity_z,
		(ParamFloat<px4::params::VCTRL_ATT_R>) _param_attitude_roll,
		(ParamFloat<px4::params::VCTRL_ATT_P>) _param_attitude_pitch,
		(ParamFloat<px4::params::VCTRL_ATT_Y>) _param_attitude_yaw,
		(ParamFloat<px4::params::VCTRL_RATE_R>) _param_rate_roll,
		(ParamFloat<px4::params::VCTRL_RATE_P>) _param_rate_pitch,
		(ParamFloat<px4::params::VCTRL_RATE_Y>) _param_rate_yaw,
		(ParamFloat<px4::params::VCTRL_SP_TMO>) _param_setpoint_timeout,
		(ParamFloat<px4::params::VOLA_TMAX>) _param_max_thrust,
		(ParamFloat<px4::params::VOLA_ARM>) _param_arm_radius,
		(ParamFloat<px4::params::VOLA_KAPPA>) _param_kappa
	)
};

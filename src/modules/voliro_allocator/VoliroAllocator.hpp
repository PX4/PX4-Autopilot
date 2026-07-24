#pragma once

#include "VoliroAllocation.hpp"
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/voliro_allocator_status.h>

class VoliroAllocator : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;
	VoliroAllocator();
	~VoliroAllocator() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	bool init();
	int print_status() override;

private:
	static constexpr uint16_t TILT_FEEDBACK_ID = 4242;
	static constexpr char TILT_FEEDBACK_NAME[] = "VOLA_TILT";
	void Run() override;
	bool updateConfiguration();
	void updateTiltFeedback();
	void conventionalFallback(const matrix::Vector3f &thrust_sp, const matrix::Vector3f &torque_sp,
				  VoliroAllocation::RotorVector &motor_normalized) const;
	VoliroAllocation::RotorVector limitTiltCommand(const VoliroAllocation::RotorVector &target, float dt,
		uint8_t &rate_limited_mask);
	void synchronizeTiltCommand();
	void publishOutputs(const VoliroAllocation::RotorVector &motor_normalized,
			    const VoliroAllocation::RotorVector &tilt_command, uint64_t timestamp_sample);
	void publishStatus(const VoliroAllocation::WrenchVector &requested,
			   const VoliroAllocation::WrenchVector &achieved,
			   const VoliroAllocation::RotorVector &thrust,
			   const VoliroAllocation::RotorVector &tilt_command,
			   uint8_t thrust_saturation_mask, uint8_t tilt_rate_limited_mask,
			   uint8_t fallback_reason, bool optimization_used, bool solver_success,
			   uint16_t iterations, float solver_time_us, uint64_t timestamp_sample);

	uORB::SubscriptionCallbackWorkItem _torque_sp_sub{this, ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _thrust_sp_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1'000'000};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<control_allocator_status_s> _control_allocator_status_pub{ORB_ID(control_allocator_status)};
	uORB::Publication<voliro_allocator_status_s> _voliro_status_pub{ORB_ID(voliro_allocator_status)};

	VoliroAllocation _allocator;
	VoliroAllocation::RotorVector _measured_tilt;
	VoliroAllocation::RotorVector _measured_tilt_velocity;
	VoliroAllocation::RotorVector _tilt_command;
	hrt_abstime _last_feedback{0};
	hrt_abstime _last_run{0};
	uint64_t _last_setpoint_sample{0};
	uint8_t _last_fallback_reason{voliro_allocator_status_s::FALLBACK_FEEDBACK_STALE};
	bool _armed{false};
	bool _landed{true};
	bool _publish_controls{true};
	bool _tilt_command_initialized{false};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::VOLA_EN>) _param_enable,
		(ParamFloat<px4::params::VOLA_TMAX>) _param_max_thrust,
		(ParamFloat<px4::params::VOLA_ARM>) _param_arm_radius,
		(ParamFloat<px4::params::VOLA_KAPPA>) _param_kappa,
		(ParamFloat<px4::params::VOLA_TILT_MIN>) _param_tilt_min,
		(ParamFloat<px4::params::VOLA_TILT_MAX>) _param_tilt_max,
		(ParamFloat<px4::params::VOLA_TILT_RATE>) _param_tilt_rate,
		(ParamFloat<px4::params::VOLA_TILT_TAU>) _param_tilt_tau,
		(ParamFloat<px4::params::VOLA_FB_TMO>) _param_feedback_timeout,
		(ParamInt<px4::params::VOLA_MAX_ITER>) _param_max_iterations,
		(ParamFloat<px4::params::VOLA_SOL_TOL>) _param_solver_tolerance,
		(ParamFloat<px4::params::VOLA_REG>) _param_regularization
	)
};

/****************************************************************************
 * CdusAllocator.hpp
 *
 * Simple quadcopter control allocation module
 *
 * - Hard-coded quad geometry
 * - Pseudo-inverse allocator
 * - Subscribes to torque + thrust setpoints
 * - Publishes actuator_motors
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

class CdusAllocatorDuct :
	public ModuleBase<CdusAllocatorDuct>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:

	static constexpr int NUM_MOTORS = 4;

	CdusAllocatorDuct();
	~CdusAllocatorDuct() override = default;

	// ModuleBase interface
	static int task_spawn(int argc, char *argv[]);
	static CdusAllocatorDuct *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void Run() override;

private:

	void init_effectiveness_matrix();
	void normalize_allocation_matrix();

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _torque_sp_sub{
		this, ORB_ID(vehicle_torque_setpoint)
	};

	uORB::Subscription _thrust_sp_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _hover_thrust_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	// Publication
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	// Effectiveness matrix and pseudoinverse
	matrix::Matrix<float, 4, NUM_MOTORS> _B{};
	matrix::Matrix<float, NUM_MOTORS, 4> _B_pinv{};
	matrix::Vector<float, NUM_MOTORS> _control_allocation_scale;

	bool _armed{false};
	bool _rate_control_enabled{false};

	float _last_u[NUM_MOTORS] {0.f, 0.f, 0.f, 0.f};
	hrt_abstime _last_run{0};

	bool _manual_torque_test{false};

	manual_control_setpoint_s _manual_control_input;

};

extern "C" __EXPORT int cdus_allocator_duct_main(int argc, char *argv[]);

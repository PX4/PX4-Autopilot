#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <matrix/matrix/math.hpp>

using matrix::Vector3f;

/**
 * Very simple quadcopter rate controller.
 *
 * - PID on body rates (roll, pitch, yaw)
 * - Takes rate + body thrust setpoints via vehicle_rates_setpoint
 * - Outputs vehicle_torque_setpoint + vehicle_thrust_setpoint
 */
class CdusRateControl final :
	public ModuleBase<CdusRateControl>,
	public ModuleParams,
	public px4::WorkItem
{
public:
	CdusRateControl();
	~CdusRateControl() override;

	/** Initialize subscriptions and schedule */
	bool init();

	/** WorkItem entry point (runs on gyro updates) */
	void Run() override;

	/** ModuleBase interface */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

private:
	void reset_integrator();

	// uORB subscriptions
	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};

	// uORB publications
	uORB::Publication<vehicle_torque_setpoint_s> _torque_sp_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_sp_pub{ORB_ID(vehicle_thrust_setpoint)};

	// latest control mode
	vehicle_control_mode_s _control_mode{};

	// internal state
	Vector3f _rate_sp{0.f, 0.f, 0.f};       // desired body rates [rad/s]
	Vector3f _thrust_sp{0.f, 0.f, 0.f};     // desired thrust in body frame
	Vector3f _integral{0.f, 0.f, 0.f};      // integrator term

	uint64_t _last_run{0};                  // last timestamp_sample [µs]

	// simple fixed gains (tune as needed)
	const Vector3f _k_p{0.1f, 0.1f, 0.05f};   // P gains roll, pitch, yaw
	const Vector3f _k_i{0.05f, 0.05f, 0.02f}; // I gains roll, pitch, yaw
	const Vector3f _k_d{0.002f, 0.002f, 0.001f}; // D gains roll, pitch, yaw

	// integrator limit (absolute value for each axis)
	const float _integrator_limit = 0.5f;
};

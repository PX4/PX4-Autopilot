#pragma once

#include "yaw_damper.h"
#include "roll_damper.h"
#include "pitch_damper.h"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/landing_gear_wheel.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/sensor_accel.h>


using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

using namespace time_literals;

struct AccelCommands {
	float Az_cmd{0.0f}; // Normal acceleration command
	float Ay_cmd{0.0f}; // Lateral acceleration command
	bool valid{false};  // Flag to indicate if data is valid
    };



class InterceptorControl final : public ModuleBase<InterceptorControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	InterceptorControl();
	~InterceptorControl() override;

	static int main(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	void set_accel_commands(float Ay, float Az);

	static InterceptorControl *instance() { return static_cast<InterceptorControl *>(ModuleBase<InterceptorControl>::_object.load()); }


	roll_damper _roll_damper;
	pitch_damper _pitch_damper;
	yaw_damper _yaw_damper;



private:

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};  // Interceptor attitude
	uORB::Subscription _acc_sub{ORB_ID(vehicle_local_position)};  // Contains acceleration (Ax, Ay, Az)
	uORB::SubscriptionData<airspeed_validated_s> _airspd_sub{ORB_ID(airspeed_validated)};  // Speed (TAS)
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)}; // Declare it here
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};				/**< vehicle status subscription */
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)}; // Correctly declare the subscription

	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	AccelCommands _accel_commands;



	bool get_accel_command(matrix::Vector3f &vec);
	void parameters_update();
	bool init();
	void Run() override;
};

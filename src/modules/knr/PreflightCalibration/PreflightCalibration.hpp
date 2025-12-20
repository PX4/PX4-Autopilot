/****************************************************************************
 *

 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_outputs.h>

using namespace time_literals;

class PreflightCalibration : public ModuleBase<PreflightCalibration>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PreflightCalibration();
	~PreflightCalibration() override;
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	static constexpr hrt_abstime INTERVAL_US = 20000_us;
	static constexpr int MAX_ACTUATORS = 4;

	void Run() override;

	void parameters_updated();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID::vehicle_status};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID::vehicle_control_mode};
	uORB::SubscriptionCallbackWorkItem _actuator_outputs{this, ORB_ID(actuator_outputs)};

	// Should also subscribe to uavcan.actuator_status if available from servos

	hrt_abstime _last_calibration_update{0};

	bool _armed{false};
	bool _system_calibrating{false};

	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _calibration_updated_perf{perf_alloc(PC_COUNT, MODULE_NAME": calibration updated")};
};

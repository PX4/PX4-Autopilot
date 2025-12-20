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
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/actuator_outputs.h>

using namespace time_literals;

static constexpr hrt_abstime CAL_INTERVAL = 50_ms;
static constexpr hrt_abstime CAL_TIMEOUT = 2000_ms;

// TODO - why when set to 4 controls only 1 servo and set to 8 then controls both?
static constexpr int NUM_ACTUATORS = 8;

static constexpr uint8_t ACTION_DO_CONTROL = 1;
static constexpr uint8_t FUNCTION_SERVO1 = 201;

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
	hrt_abstime _last_calibration_update{0};

	bool _armed{false};
	bool _calibration_done{true};

	struct ServoCalibrationState {
		bool reached_min{false};
		bool reached_max{false};
		float value{0.0f};
		bool forward{true};
	};

	ServoCalibrationState _sv_states[NUM_ACTUATORS];

	void Run() override;
	void parameters_updated();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID::vehicle_status};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID::vehicle_control_mode};
	uORB::SubscriptionCallbackWorkItem _actuator_outputs{
		this,
		ORB_ID(actuator_outputs),
		1	// Explicit instance setting for servos
	};

	uORB::Publication<actuator_test_s> _actuator_test_pub{ORB_ID(actuator_test)};

	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _calibration_updated_perf{perf_alloc(PC_COUNT, MODULE_NAME": calibration updated")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SV_CAL_MAX_VAL>) _sv_cal_max,
		(ParamFloat<px4::params::SV_CAL_MIN_VAL>) _sv_cal_min,
		(ParamFloat<px4::params::SV_CAL_STEP_VAL>) _sv_cal_step
	);
};

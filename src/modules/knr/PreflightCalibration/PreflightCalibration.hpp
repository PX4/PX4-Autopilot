/****************************************************************************
 *
 * This module is triggered by external mavlink command 'DO_PREFLIGHT' to perform preflight checks and calibration.
 * This ensures that the vehicle is properly calibrated before starting an autonomous mission.
 *
 * Remember to make with propoer hardware target => make cubepilot_cubeorangeplus
 *
 * SHOULD WE NAME ALL OF THE CUSTOM MODULES WITH A PREFIX LIKE KNR_ ???
 *
 * Helpful CLI commands to test the module:
 * - To start the module: 			preflight_calibration start
 * - To see the status: 			preflight_calibration status
 *
 * - To see actuator test values: 		listener actuator_test
 * - To see current servo values: 		listener actuator_outputs
 * - To see preflight calibration status: 	listener preflight_calibration_status
 * - To see preflight calibration control: 	listener preflight_calibration_control
 *
 * - To trigger preflight calibration via uORB (if not started via CLI):
 * uorb pub preflight_calibration_control \ '{timestamp:0, action:1}'
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
#include <uORB/topics/preflight_calibration_status.h>
#include <uORB/topics/preflight_calibration_control.h>

using namespace time_literals;

static constexpr hrt_abstime CAL_INTERVAL = 50_ms;
static constexpr hrt_abstime CAL_TIMEOUT = 2000_ms;

// TODO - why when set to 4 controls only 1 servo and set to 8 then controls both?
static constexpr int NUM_ACTUATORS = 16;

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
	bool _request_stop{false};
	bool _calibration_active{false};

	struct ServoCalibrationState {
		bool reached_min{false};
		bool reached_max{false};
		float value{0.0f};
		bool forward{true};
		uint8_t status{static_cast<uint8_t>(ServoStatus::NOT_INITIALIZED)};
	};

	enum class ServoStatus {
		NOT_INITIALIZED = 0,
		CALIBRATING,
		CALIBRATED,
		NOT_HEALTHY 		// TODO - implement checks
	};

	ServoCalibrationState _sv_states[NUM_ACTUATORS];

	void Run() override;

	void _parameters_updated();
	bool _is_calibration_successful();
	bool _do_calibration_ended();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID::vehicle_status};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID::vehicle_control_mode};
	uORB::Subscription _preflight_calibration_control_sub{ORB_ID(preflight_calibration_control)};
	uORB::SubscriptionCallbackWorkItem _actuator_outputs{
		this,
		ORB_ID(actuator_outputs),
		1	// Instance for servos, 0 used for motors
	};

	uORB::Publication<actuator_test_s> _actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<preflight_calibration_status_s> _preflight_calibration_status_pub{ORB_ID(preflight_calibration_status)};

	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _calibration_updated_perf{perf_alloc(PC_COUNT, MODULE_NAME": calibration updated")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SV_CAL_MAX_VAL>) _sv_cal_max,
		(ParamFloat<px4::params::SV_CAL_MIN_VAL>) _sv_cal_min,
		(ParamFloat<px4::params::SV_CAL_STEP_VAL>) _sv_cal_step
	);
};

/****************************************************************************
 *
 * This module is triggered by external mavlink command 'DO_PREFLIGHT' to perform preflight checks and calibration.
 * This ensures that the vehicle is properly calibrated before starting an autonomous mission.
 *
 * Remember to make with propoer hardware target => cubepilot_cubeorange
 *
 * SHOULD WE NAME ALL OOF THE CUSTOM MODULES WITH A PREFIX LIKE KNR_ ???
 *
 * Helpful CLI commands to test the module:
 * - To start the module: preflight_calibration start
 * - To see the status: preflight_calibration status
 * - To see actuator test values: listener actuator_test
 * - To see current servo values: listener actuator_outputs
 ****************************************************************************/

#include "PreflightCalibration.hpp"

using namespace time_literals;
using matrix::Vector3f;

PreflightCalibration::PreflightCalibration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

PreflightCalibration::~PreflightCalibration()
{
	perf_free(_loop_interval_perf);
	perf_free(_calibration_updated_perf);
}

bool PreflightCalibration::init()
{
	if (!_actuator_outputs.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	for(int i=0;i<NUM_ACTUATORS;i++) {
		_sv_states[i].value = _sv_cal_min.get();
		_sv_states[i].forward = true;
	}

	return true;
}

void
PreflightCalibration::parameters_updated()
{
	ModuleParams::updateParams();
}

void PreflightCalibration::Run()
{
	// Add business logic from docs
	perf_count(_loop_interval_perf);

	if(_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		parameters_updated();
	}

	const hrt_abstime now = hrt_absolute_time();
	if (now - _last_calibration_update < CAL_INTERVAL) {
		return;
	}

	_last_calibration_update = now;

	const float max = _sv_cal_max.get();
	const float min = _sv_cal_min.get();
	const float step = _sv_cal_step.get();
	bool all_done{true};

	for(int i=0; i<NUM_ACTUATORS; i++) {
		auto &s = _sv_states[i];

		s.value += s.forward ? step : -step;

		if(!s.forward && s.value <= min) {
			s.reached_min = true;
		} else if(s.forward && s.value >= max) {
			s.reached_max = true;
			s.forward = false;
		}

		if (!(s.reached_min && s.reached_max)) {
		all_done = false;
		}

		if(s.reached_min && s.reached_max) {
			continue; // Skip this servo if done calibrating
		}

		actuator_test_s actuator_test{};
		actuator_test.timestamp = now;
		actuator_test.timeout_ms = CAL_TIMEOUT;
		actuator_test.action = ACTION_DO_CONTROL;
		actuator_test.function = FUNCTION_SERVO1 + i;
		actuator_test.value = s.value;

		_actuator_test_pub.publish(actuator_test);

		PX4_INFO("Calibrating actuator %d to value %.3f", i+1, static_cast<double>(actuator_test.value));

		actuator_outputs_s outputs;
		if (_actuator_outputs.update(&outputs)) {

		PX4_INFO("Current servos position: [%.3f, %.3f, %.3f, %.3f]",
			(double)outputs.output[0],
			(double)outputs.output[1],
			(double)outputs.output[2],
			(double)outputs.output[3]
		);
	}
	}

	if(all_done && !_calibration_done) {
		PX4_INFO("Preflight calibration completed for all actuators.");
		_calibration_done = true;

		 // Stop the scheduled work item
		ScheduleClear();

		exit_and_cleanup();
		return;
	}

	perf_count(_calibration_updated_perf);
}

int PreflightCalibration::task_spawn(int argc, char *argv[])
{
	PreflightCalibration *instance = new PreflightCalibration();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PreflightCalibration::print_status()
{
	// TODO - print status

	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_calibration_updated_perf);
	return 0;
}

int PreflightCalibration::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PreflightCalibration::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	[KNR_MODULE] Preflight calibration as additional step for autonomous mission start.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("preflight_calibration", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int preflight_calibration_main(int argc, char *argv[])
{
	return PreflightCalibration::main(argc, argv);
}

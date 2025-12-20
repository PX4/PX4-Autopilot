/****************************************************************************
 *
 * This module is triggered by external mavlink command 'DO_PREFLIGHT' to perform preflight checks and calibration.
 * This ensures that the vehicle is properly calibrated before starting an autonomous mission.
 *
 * Remember to make with propoer hardware target => cubepilot_cubeorange
 *
 * SHOULD WE NAME ALL OOF THE CUSTOM MODULES WITH A PREFIX LIKE KNR_ ???
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

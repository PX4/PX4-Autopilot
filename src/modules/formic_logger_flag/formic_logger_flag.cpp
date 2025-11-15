#include "formic_logger_flag.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

using namespace time_literals;

FormicLoggerFlag::FormicLoggerFlag() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_update(true);
}

bool FormicLoggerFlag::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void FormicLoggerFlag::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void FormicLoggerFlag::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();
	run_once();
}

int FormicLoggerFlag::task_spawn(int argc, char *argv[])
{
	FormicLoggerFlag *instance = new FormicLoggerFlag();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int FormicLoggerFlag::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicLoggerFlag::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal skeleton module ready for custom logic.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_logger_flag", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	return PX4_OK;
}

FormicLoggerFlag *FormicLoggerFlag::instantiate(int argc, char *argv[])
{
	return new FormicLoggerFlag();
}

void FormicLoggerFlag::run_once()
{
	const int32_t selected_channel = _param_flf_rc_ch.get();

	if (selected_channel != _active_rc_channel) {
		_active_rc_channel = selected_channel;
		PX4_INFO("Formic logger RC channel set to %ld", (long)_active_rc_channel);
	}
}

extern "C" __EXPORT int formic_logger_flag_main(int argc, char *argv[])
{
	return FormicLoggerFlag::main(argc, argv);
}


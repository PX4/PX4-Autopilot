
#include "vertiq_io.hpp"

//outputmoduleinterface(const char *name, const px4::wq_config_t &config)

VertiqIo::VertiqIo() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::test1)
{

}

VertiqIo::~VertiqIo()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool VertiqIo::init()
{
	//Schedule to run every 1 second
	//calls Run() every second
	ScheduleOnInterval(1_s);

	return true;
}

//This is the same as a while(1) loop. Gets called at a set interval, or
//is triggered by some uORB publication
void VertiqIo::Run()
{
	//If we should leave, then clean up our mess and get out
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	//Start the loop timer
	//Increment our loop counter
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	PX4_INFO("Application is running!");

	//stop our timer
	perf_end(_loop_perf);
}

int VertiqIo::task_spawn(int argc, char *argv[])
{
	VertiqIo *instance = new VertiqIo();

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

int VertiqIo::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int VertiqIo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("test", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int VertiqIo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

bool VertiqIo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated){
	return true;
}

extern "C" __EXPORT int vertiq_io_main(int argc, char *argv[])
{
	PX4_INFO("Hello Vertiq!");
	return VertiqIo::main(argc, argv);
}

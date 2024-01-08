
#include "vertiq_io.hpp"

#include <px4_platform_common/log.h>

px4::atomic_bool VertiqIo::_request_telemetry_init{false};
char VertiqIo::_telemetry_device[] {};

VertiqIo::VertiqIo() :
	OutputModuleInterface(MODULE_NAME "-actuators-esc", px4::wq_configurations::hp_default),
	prop_test(0),
	brushless_drive_test(0),
	_serial_interface(num_clients)
{
	test[0] = &prop_test;
	test[1] = &brushless_drive_test;
}

VertiqIo::~VertiqIo()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

//called by our task_spawn function
bool VertiqIo::init()
{
	//Schedule to run every 2.5 ms
	//calls Run() every second
	ScheduleOnInterval(2500_us);

	return true;
}

//This is the same as a while(1) loop. Gets called at a set interval, or
//is triggered by some uORB publication
void VertiqIo::Run()
{
	//Start the loop timer
	//Increment our loop counter
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//If we should leave, then clean up our mess and get out
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// telemetry device update request?
	if (_request_telemetry_init.load()) {
		PX4_INFO("Asked for serial init");
		_serial_interface.init_serial(_telemetry_device);
		// init_serial(_telemetry_device);
		_request_telemetry_init.store(false);
	}

	prop_test.ctrl_velocity_.set(*_serial_interface.get_iquart_interface(), 400);
	brushless_drive_test.obs_velocity_.get(*_serial_interface.get_iquart_interface());

	//Update our serial rx
	_serial_interface.process_serial_rx(test);

	//Update our serial tx
	_serial_interface.process_serial_tx();

	//Read the velo we got
	if (brushless_drive_test.obs_velocity_.IsFresh()) {
		PX4_INFO("Got velocity response %f", (double)brushless_drive_test.obs_velocity_.get_reply());

	}

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

	PRINT_MODULE_USAGE_NAME("vertiq_io", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int VertiqIo::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "telemetry")) {
		if (argc > 1) {
			// telemetry can be requested before the module is started
			strncpy(_telemetry_device, argv[1], sizeof(_telemetry_device) - 1);
			_telemetry_device[sizeof(_telemetry_device) - 1] = '\0';
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	return print_usage("unknown command");
}

bool VertiqIo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			     unsigned num_control_groups_updated)
{
	return true;
}


extern "C" __EXPORT int vertiq_io_main(int argc, char *argv[])
{
	return VertiqIo::main(argc, argv);
}

#include "eulernav_driver.h"
#include <px4_platform_common/getopt.h>

EulerNavDriver::EulerNavDriver(const char* device_name) :
	_serial_port{device_name, 115200, ByteSize::EightBits, Parity::None, StopBits::One, FlowControl::Disabled}
{
	_serial_port.open();

	if (_serial_port.isOpen())
	{
		PX4_INFO("Serial port opened successfully.");
		_is_initialized = true;
	}
	else
	{
		PX4_INFO("Failed to open serial port");
		_is_initialized = false;
	}
}

EulerNavDriver::~EulerNavDriver()
{
	if (_serial_port.isOpen())
	{
		_serial_port.close();
	}
}

int EulerNavDriver::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("bahrs", SCHED_DEFAULT, SCHED_PRIORITY_FAST_DRIVER,
					 _task_stack_size, (px4_main_t)&run_trampoline, argv);

	PX4_INFO("Task spawn.");

	if (task_id < 0)
	{
		_task_id = -1;
		PX4_INFO("Task spawn failed.");
	}
	else
	{
		_task_id = task_id;
		PX4_INFO("Task spawn succeeded.");
	}

	return (_task_id < 0) ? 1 : 0;
}

EulerNavDriver* EulerNavDriver::instantiate(int argc, char *argv[])
{
	int option_index = 1;
	const char* option_arg{nullptr};
	const char* device_name{nullptr};

	PX4_INFO("Task instantiate.");

	while (true)
	{
		int option{px4_getopt(argc, argv, "d:", &option_index, &option_arg)};

		if (EOF == option)
		{
			break;
		}

		switch (option)
		{
		case 'd':
			device_name = option_arg;
			break;
		default:
			break;
		}
	}

	return new EulerNavDriver(device_name);
}

int EulerNavDriver::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int EulerNavDriver::print_usage(const char *reason)
{
	return 0;
}

void EulerNavDriver::run()
{
	while(false == should_exit())
	{
		px4_usleep(1000000);
		PX4_INFO("Running the EulerNavDriver::run...");
	}
}

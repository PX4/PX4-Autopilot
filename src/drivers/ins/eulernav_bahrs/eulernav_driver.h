#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/Serial.hpp>

class EulerNavDriver : public ModuleBase<EulerNavDriver>
{
public:
	EulerNavDriver(const char* device_name);

	~EulerNavDriver();

	/// @brief Required by ModuleBase
	static int task_spawn(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static EulerNavDriver* instantiate(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static int custom_command(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static int print_usage(const char *reason = nullptr);

	/// @brief The main loop of the task.
	void run() final;

private:
	static constexpr int _task_stack_size{2048};
	device::Serial _serial_port;
	bool _is_initialized{false};
};


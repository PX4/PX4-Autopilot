
#pragma once

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_test.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

class VertiqIo : public ModuleBase<VertiqIo>, public OutputModuleInterface
{

public:

	VertiqIo();
	~VertiqIo() override;

	bool init();

	/**
	 * set the Baudrate
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(unsigned baud);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	/** @see ModuleBase::run() */ //I do not think this actually comes from ModuleBase. it should come from scheduled work item
	void Run() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	int init_serial(const char *uart_device);
	void deinit_serial();
	int updateSerial();

private:
	static constexpr int FRAME_SIZE = 10;

	static char _telemetry_device[20];
	static px4::atomic_bool _request_telemetry_init;

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")};

	int _uart_fd{-1};

	#if ! defined(__PX4_QURT)
		struct termios		_orig_cfg;
		struct termios		_cfg;
	#endif

	int   _speed = -1;
};





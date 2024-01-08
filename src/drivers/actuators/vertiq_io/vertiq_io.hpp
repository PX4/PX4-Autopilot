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

#include "vertiq_serial_interface.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"

class VertiqIo : public ModuleBase<VertiqIo>, public OutputModuleInterface
{

public:

	/**
	* @brief Create a new VertiqIo object
	*/
	VertiqIo();

	/**
	* @brief destruct a VertiqIo object
	*/
	~VertiqIo() override;

	/**
	* @brief initialize the VertiqIo object. This will be called by the task_spawn function. Makes sure that the thread gets scheduled.
	*/
	bool init();

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

private:
	//Variables and functions necessary for properly configuring the serial interface
	//Determines whether or not we should initialize or re-initialize the serial connection
	static px4::atomic_bool _request_telemetry_init;

	MixingOutput _mixing_output{"VERTIQ_IO", 4, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	//The name of the device we're connecting to. this will be something like /dev/ttyS3
	static char _telemetry_device[20];

	//Counters/timers to track our status
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")};

	uint8_t num_clients = 2;
	PropellerMotorControlClient prop_test;
	BrushlessDriveClient brushless_drive_test;
	ClientAbstract * test[2];

	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface _serial_interface;

	DEFINE_PARAMETERS(
	(ParamInt<px4::params::VERTIQ_TEST>) _param_vertiq_test
	)
};





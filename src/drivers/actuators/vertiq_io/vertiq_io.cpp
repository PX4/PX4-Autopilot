/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "vertiq_io.hpp"

#include <px4_platform_common/log.h>

px4::atomic_bool VertiqIo::_request_telemetry_init{false};

VertiqIo::VertiqIo(const char *port) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_serial_interface(),
	_client_manager(&_serial_interface),
	_telem_manager(&_client_manager),
	_configuration_handler(&_serial_interface, &_client_manager),
	_broadcast_prop_motor_control(_kBroadcastID),
	_broadcast_arming_handler(_kBroadcastID),
	_operational_ifci(_kBroadcastID)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	//Make sure we get the correct initial values for our parameters
	updateParams();

	_configuration_handler.InitConfigurationClients((uint8_t)_param_vertiq_target_module_id.get());
	_configuration_handler.InitClientEntryWrappers();

	_client_manager.AddNewClient(&_operational_ifci);
	_client_manager.AddNewClient(&_broadcast_arming_handler);
	_client_manager.AddNewClient(&_broadcast_prop_motor_control);
}

VertiqIo::~VertiqIo()
{
	//Be inactive
	stop();

	//Free our counters/timers
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

//called by our task_spawn function
bool VertiqIo::init()
{
	_serial_interface.InitSerial(_port, _param_vertiq_baud.get());

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	//Grab the number of IFCI control values the user wants to use
	_cvs_in_use = (uint8_t)_param_vertiq_number_of_cvs.get();

	//Grab the bitmask that we're going to use to decide who we get telemetry from
	_telemetry_ids_1 = (uint32_t)_param_vertiq_telem_ids_1.get();
	_telemetry_ids_2 = (uint32_t)_param_vertiq_telem_ids_2.get();
	_telem_bitmask = ((uint64_t)(_telemetry_ids_2) << 32) | _telemetry_ids_1;

	_transmission_message.num_cvs = _cvs_in_use;
#endif

	//Initialize our telemetry handler
	_telem_manager.Init(_telem_bitmask, (uint8_t)_param_vertiq_target_module_id.get());
	_telem_manager.StartPublishing(&_esc_status_pub);

	//Make sure we get our thread into execution
	ScheduleNow();

	return true;
}

void VertiqIo::start()
{
	ScheduleNow();
}

void VertiqIo::stop()
{
	ScheduleClear();
}


//This is the same as a while(1) loop
void VertiqIo::Run()
{
	//Start the loop timer
	//Increment our loop counter
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//Handle IQUART reception and transmission
	_client_manager.HandleClientCommunication();

	// If we're supposed to ask for telemetry from someone
	if (_telem_bitmask) {
		//Grab the next ID, and if it's real get it ready to send out
		uint16_t next_telem = _telem_manager.UpdateTelemetry();

		if (next_telem != _impossible_module_id) {
			_transmission_message.telem_byte = next_telem;
		}
	}

	//Get the most up to date version of our parameters/check if they've changed
	parameters_update();

	//Make sure we also update the mixing output to get the most up to date configuration
	_mixing_output.update();
	_mixing_output.updateSubscriptions(true);

	//Go ahead and check to see if our actuator test has gotten anything new
	if (_actuator_test_sub.updated()) {
		_actuator_test_sub.copy(&_actuator_test);

		//Our test is active if anyone is giving us commands through the actuator test
		_actuator_test_active = _actuator_test.action == actuator_test_s::ACTION_DO_CONTROL;
	}

	//stop our timer
	perf_end(_loop_perf);
}

void VertiqIo::parameters_update()
{
	//If someone has changed any parameter in our module. Checked at 1Hz
	if (_parameter_update_sub.updated()) {
		//Grab the changed parameter with copy (which lowers the "changed" flag)
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

		//If the target module ID changed, we need to update all of our parameters
		if (_param_vertiq_target_module_id.get() != _configuration_handler.GetObjectIdNow()) {
			_configuration_handler.UpdateClientsToNewObjId(_param_vertiq_target_module_id.get());

			//Make sure we start a new read!
			_param_vertiq_trigger_read.set(true);
			_param_vertiq_trigger_read.commit_no_notification();
		}

		//If you're set to re-read from the motor, mark all of the IQUART parameters for reinitialization, reset the trigger, and then update the IFCI params
		if (_param_vertiq_trigger_read.get()) {
			_configuration_handler.MarkConfigurationEntriesForRefresh();
			_param_vertiq_trigger_read.set(false);
			_param_vertiq_trigger_read.commit_no_notification();
		}

		_configuration_handler.UpdateIquartConfigParams();
	}
}

void VertiqIo::OutputControls(uint16_t outputs[MAX_ACTUATORS])
{
	//Put the mixer outputs into the output message
	for (uint8_t i = 0; i < _transmission_message.num_cvs; i++) {
		_transmission_message.commands[i] = outputs[i];
	}

	_operational_ifci.PackageIfciCommandsForTransmission(&_transmission_message, _output_message, &_output_len);
	_operational_ifci.packed_command_.set(*_serial_interface.GetIquartInterface(), _output_message, _output_len);
	_serial_interface.ProcessSerialTx();
}

bool VertiqIo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			     unsigned num_control_groups_updated)
{
#ifdef CONFIG_USE_IFCI_CONFIGURATION

	if (_mixing_output.armed().armed) {

		if (_param_vertiq_arm_behavior.get() == FORCE_ARMING && _send_forced_arm) {
			_broadcast_arming_handler.motor_armed_.set(*_serial_interface.GetIquartInterface(), 1);
			_send_forced_arm = false;
		}

		OutputControls(outputs);

		//We want to make sure that we send a valid telem request only once to ensure that we're not getting extraneous responses.
		//So, here we'll set the telem request ID to something that no one will respond to. Another function will take charge of setting it to a
		//proper value when necessary
		_transmission_message.telem_byte = _impossible_module_id;

	} else if (_actuator_test_active) {
		OutputControls(outputs);

	} else {
		//Put the modules into coast
		switch (_param_vertiq_disarm_behavior.get()) {
		case TRIGGER_MOTOR_DISARM:
			_broadcast_arming_handler.motor_armed_.set(*_serial_interface.GetIquartInterface(), 0);
			break;

		case COAST_MOTOR:
			_broadcast_prop_motor_control.ctrl_coast_.set(*_serial_interface.GetIquartInterface());
			break;

		case SEND_PREDEFINED_VELOCITY:
			_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface.GetIquartInterface(),
					_param_vertiq_disarm_velocity.get());
			break;

		default:
			break;
		}

		if (!_send_forced_arm) {
			_send_forced_arm = true;
		}

		_actuator_test_active = false;
	}

	//Publish our esc status to uORB
	_esc_status_pub.publish(_telem_manager.GetEscStatus());
#endif
	return true;
}

void VertiqIo::print_info()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	_mixing_output.printStatus();
}

/**
 * Local functions in support of the shell command.
 */
namespace vertiq_namespace
{

VertiqIo	*g_dev{nullptr};

int start(const char *port);
int status();
int stop();
int usage();

int
start(const char *port)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new VertiqIo(port);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (!g_dev->init()) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_USAGE_NAME("vertiq_io", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int vertiq_io_main(int argc, char *argv[])
{
	int ch = 0;
	const char *device_path = nullptr;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return vertiq_namespace::usage();
	}

	if (!device_path) {
		PX4_ERR("Missing device");
		return PX4_ERROR;
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return vertiq_namespace::start(device_path);

		} else {
			PX4_WARN("Please specify device path!");
			return vertiq_namespace::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return vertiq_namespace::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return vertiq_namespace::status();
	}

	return vertiq_namespace::usage();
}

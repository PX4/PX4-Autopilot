/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "DShot.h"
#include <px4_arch/io_timer.h>
#include <px4_platform_common/sem.hpp>

char DShot::_telemetry_device[] {};
bool DShot::_telemetry_swap_rxtx{false};
px4::atomic_bool DShot::_request_telemetry_init{false};

DShot::DShot() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setAllDisarmedValues(DSHOT_DISARM_VALUE);
	_mixing_output.setAllMinValues(DSHOT_MIN_THROTTLE);
	_mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);

	// Avoid using the PWM failsafe params
	_mixing_output.setAllFailsafeValues(UINT16_MAX);
}

DShot::~DShot()
{
	// make sure outputs are off
	up_dshot_arm(false);

	perf_free(_cycle_perf);
	perf_free(_bdshot_rpm_perf);
	perf_free(_dshot_telem_perf);

	delete _telemetry;
}

int DShot::init()
{
	_output_mask = (1u << _num_outputs) - 1;

	update_params();

	enable_dshot_outputs();

	ScheduleNow();

	return OK;
}

int DShot::task_spawn(int argc, char *argv[])
{
	DShot *instance = new DShot();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

void DShot::enable_dshot_outputs()
{
	unsigned int dshot_frequency = 0;
	uint32_t dshot_frequency_param = 0;

	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		uint32_t channels = io_timer_get_group(timer);

		if (channels == 0) {
			continue;
		}

		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

		int32_t tim_config = 0;
		param_t handle = param_find(param_name);
		param_get(handle, &tim_config);
		unsigned int dshot_frequency_request = 0;

		if (tim_config == -5) {
			dshot_frequency_request = DSHOT150;

		} else if (tim_config == -4) {
			dshot_frequency_request = DSHOT300;

		} else if (tim_config == -3) {
			dshot_frequency_request = DSHOT600;

		} else {
			_output_mask &= ~channels; // don't use for dshot
		}

		if (dshot_frequency_request != 0) {
			if (dshot_frequency != 0 && dshot_frequency != dshot_frequency_request) {
				PX4_WARN("Only supporting a single frequency, adjusting param %s", param_name);
				param_set_no_notification(handle, &dshot_frequency_param);

			} else {
				dshot_frequency = dshot_frequency_request;
				dshot_frequency_param = tim_config;
			}
		}
	}

	_bidirectional_dshot_enabled = _param_bidirectional_enable.get();

	int ret = up_dshot_init(_output_mask, dshot_frequency, _bidirectional_dshot_enabled);

	if (ret < 0) {
		PX4_ERR("up_dshot_init failed (%i)", ret);
		return;
	}

	_output_mask = ret;

	// disable unused functions
	for (unsigned i = 0; i < _num_outputs; ++i) {
		if (((1 << i) & _output_mask) == 0) {
			_mixing_output.disableFunction(i);
		}
	}

	if (_output_mask == 0) {
		// exit the module if no outputs used
		request_stop();
		return;
	}

	up_dshot_arm(true);
}

void DShot::update_num_motors()
{
	int motor_count = 0;

	for (unsigned i = 0; i < _num_outputs; ++i) {
		if (_mixing_output.isFunctionSet(i)) {
			_actuator_functions[motor_count] = (uint8_t)_mixing_output.outputFunction(i);
			++motor_count;
		}
	}

	_num_motors = motor_count;
}

void DShot::init_telemetry(const char *device, bool swap_rxtx)
{
	if (_telemetry || !device) {
		return;
	}

	_telemetry = new DShotTelemetry{};

	if (!_telemetry) {
		PX4_ERR("alloc failed");
		return;
	}

	if (_telemetry->init(device, swap_rxtx) != PX4_OK) {
		PX4_ERR("telemetry init failed");
	}

	update_num_motors();
}

bool DShot::handle_new_telemetry_data(const int telemetry_index, const DShotTelemetry::EscData &data)
{
	bool all_channels_updated = false;
	// fill in new motor data
	esc_status_s &esc_status = esc_status_pub.get();

	if (telemetry_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_status.esc_online_flags |= 1 << telemetry_index;

		esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];

		if (!_bidirectional_dshot_enabled) {
			// If we also have bidirectional dshot, we use rpm and timestamps from there.
			esc_status.esc[telemetry_index].timestamp       = data.time;
			esc_status.esc[telemetry_index].esc_rpm         = (static_cast<int>(data.erpm) * 100) /
					(_param_mot_pole_count.get() / 2);
		}

		esc_status.esc[telemetry_index].esc_voltage     = static_cast<float>(data.voltage) * 0.01f;
		esc_status.esc[telemetry_index].esc_current     = static_cast<float>(data.current) * 0.01f;
		esc_status.esc[telemetry_index].esc_temperature = static_cast<float>(data.temperature);
		// TODO: accumulate consumption and use for battery estimation
	}

	// publish when motor index wraps (which is robust against motor timeouts)
	if (telemetry_index <= _last_telemetry_index) {
		esc_status.timestamp = hrt_absolute_time();
		esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
		esc_status.esc_count = _num_motors;
		++esc_status.counter;

		all_channels_updated = true; // Indicate we wrapped, so we publish data
	}

	_last_telemetry_index = telemetry_index;

	perf_count(_dshot_telem_perf);

	return all_channels_updated;
}

void DShot::publish_esc_status(void)
{
	esc_status_s &esc_status = esc_status_pub.get();
	int telemetry_index = 0;

	// clear data of the esc that are offline
	for (int index = 0; (index < _last_telemetry_index); index++) {
		if ((esc_status.esc_online_flags & (1 << index)) == 0) {
			memset(&esc_status.esc[index], 0, sizeof(struct esc_report_s));
		}
	}

	// FIXME: mark all UART Telemetry ESC's as online, otherwise commander complains even for a single dropout
	esc_status.esc_count = _num_motors;
	esc_status.esc_online_flags = (1 << esc_status.esc_count) - 1;
	esc_status.esc_armed_flags = (1 << esc_status.esc_count) - 1;

	if (_bidirectional_dshot_enabled) {
		for (unsigned i = 0; i < _num_outputs; i++) {
			if (_mixing_output.isFunctionSet(i)) {
				if (up_bdshot_channel_status(i)) {
					esc_status.esc_online_flags |= 1 << i;

				} else {
					esc_status.esc_online_flags &= ~(1 << i);
				}

				++telemetry_index;
			}
		}
	}

	if (!esc_status_pub.advertised()) {
		esc_status_pub.advertise();

	} else {
		esc_status_pub.update();
	}

	// reset esc online flags
	esc_status.esc_online_flags = 0;
}

bool DShot::handle_new_bdshot_erpm(void)
{
	int num_erpms = 0;
	int telemetry_index = 0;
	int erpm;
	esc_status_s &esc_status = esc_status_pub.get();

	esc_status.timestamp = hrt_absolute_time();
	esc_status.counter = _esc_status_counter++;
	esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
	esc_status.esc_armed_flags = true; // TODO: should we ever unset?

	// We wait until all are ready.
	if (up_bdshot_num_erpm_ready() < _num_motors) {
		return false;
	}

	for (unsigned i = 0; i < _num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			if (up_bdshot_get_erpm(i, &erpm) == 0) {
				num_erpms++;
				esc_status.esc_online_flags |= 1 << telemetry_index;
				esc_status.esc[telemetry_index].timestamp = hrt_absolute_time();
				esc_status.esc[telemetry_index].esc_rpm = (erpm * 100) / (_param_mot_pole_count.get() / 2);
				esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];
			}

			++telemetry_index;
		}
	}

	perf_count(_bdshot_rpm_perf);

	return num_erpms > 0;
}

void DShot::mixerChanged()
{
	update_num_motors();
}

bool DShot::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	// First check if all outputs are disarmed and we have a command to send
	bool all_disarmed = true;

	for (int i = 0; i < (int)num_outputs; i++) {
		if (!_mixing_output.isFunctionSet(i)) {
			continue;
		}

		if (outputs[i] != DSHOT_DISARM_VALUE) {
			all_disarmed = false;
		}
	}

	// All outputs are disarmed and we have a command to send
	if (all_disarmed && _current_command.valid()) {

		for (int i = 0; i < (int)num_outputs; i++) {

			if (!_mixing_output.isFunctionSet(i)) {
				continue;
			}

			if (_current_command.motor_mask & (1 << i)) {
				PX4_INFO("Sending command %u to motor %d", _current_command.command, i);
				up_dshot_motor_command(i, _current_command.command, true);

				// Decrement command repetition counter
				--_current_command.num_repetitions;

				// Queue a save command after the burst if save has been requested
				if (_current_command.num_repetitions == 0 && _current_command.save) {
					_current_command.save = false;
					_current_command.num_repetitions = 10;
					_current_command.command = dshot_command_t::DSHOT_CMD_SAVE_SETTINGS;
				}

			} else {
				up_dshot_motor_data_set(i, math::min(DSHOT_DISARM_VALUE, DSHOT_MAX_THROTTLE), false);
			}
		}

		up_dshot_trigger();
		return true;
	}

	// Clear pending commands if any output is not disarmed
	_current_command.clear();

	int telemetry_index = 0;
	int requested_telemetry_index = -1;

	if (_telemetry) {
		requested_telemetry_index = _telemetry->getRequestMotorIndex();
	}

	for (int i = 0; i < (int)num_outputs; i++) {

		uint16_t output = outputs[i];
		bool request_telemetry = _telemetry && (requested_telemetry_index == telemetry_index);

		if (!_mixing_output.isFunctionSet(i)) {
			continue;
		}

		if (output == DSHOT_DISARM_VALUE) {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, request_telemetry);

		} else {
			// Reverse output if required
			if (_param_dshot_3d_enable.get() || (_reversible_outputs & (1u << i))) {
				output = convert_output_to_3d_scaling(output);
			}

			up_dshot_motor_data_set(i, math::min(output, DSHOT_MAX_THROTTLE), request_telemetry);
		}

		telemetry_index++;
	}

	up_dshot_trigger();

	return true;
}

uint16_t DShot::convert_output_to_3d_scaling(uint16_t output)
{
	// DShot 3D splits the throttle ranges in two.
	// This is in terms of DShot values, code below is in terms of actuator_output
	// Direction 1) 48 is the slowest, 1047 is the fastest.
	// Direction 2) 1049 is the slowest, 2047 is the fastest.
	if (output >= _param_dshot_3d_dead_l.get() && output < _param_dshot_3d_dead_h.get()) {
		return DSHOT_DISARM_VALUE;
	}

	bool upper_range = output >= 1000;

	if (upper_range) {
		output -= 1000;

	} else {
		output = 999 - output; // lower range is inverted
	}

	float max_output = 999.f;
	float min_output = max_output * _param_dshot_min.get();
	output = math::min(max_output, (min_output + output * (max_output - min_output) / max_output));

	if (upper_range) {
		output += 1000;
	}

	return output;
}

void DShot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Updates the DShot throttle output and sends DShot commands
	_mixing_output.update();

	bool all_channels_updated = false;

	if (_telemetry) {
		const int motor_index = _telemetry->update(_num_motors);

		if (motor_index >= 0) {
			all_channels_updated = handle_new_telemetry_data(motor_index, _telemetry->latestESCData());
		}
	}

	if (_bidirectional_dshot_enabled) {
		// Add bdshot data to esc status
		all_channels_updated = handle_new_bdshot_erpm();
	}

	if (all_channels_updated) {
		publish_esc_status();
	}

	if (_parameter_update_sub.updated()) {
		update_params();
	}

	// telemetry device update request?
	if (_request_telemetry_init.load()) {
		init_telemetry(_telemetry_device, _telemetry_swap_rxtx);
		_request_telemetry_init.store(false);
	}

	handle_vehicle_commands();

	if (!_mixing_output.armed().armed) {
		if (_reversible_outputs != _mixing_output.reversibleOutputs()) {
			_reversible_outputs = _mixing_output.reversibleOutputs();
			update_params();
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void DShot::handle_vehicle_commands()
{
	vehicle_command_s vehicle_command;

	while (!_current_command.valid() && _vehicle_command_sub.update(&vehicle_command)) {

		// https://mavlink.io/en/messages/common.html#MAV_CMD_CONFIGURE_ACTUATOR
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR) {
			int function = (int)(vehicle_command.param5 + 0.5);

			if (function < 1000) {
				const int first_motor_function = 1; // from MAVLink ACTUATOR_OUTPUT_FUNCTION
				const int first_servo_function = 33;

				if (function >= first_motor_function && function < first_motor_function + actuator_test_s::MAX_NUM_MOTORS) {
					function = function - first_motor_function + actuator_test_s::FUNCTION_MOTOR1;

				} else if (function >= first_servo_function && function < first_servo_function + actuator_test_s::MAX_NUM_SERVOS) {
					function = function - first_servo_function + actuator_test_s::FUNCTION_SERVO1;

				} else {
					function = INT32_MAX;
				}

			} else {
				function -= 1000;
			}

			int type = (int)(vehicle_command.param1 + 0.5f);
			int index = -1;

			for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
				if ((int)_mixing_output.outputFunction(i) == function) {
					index = i;
				}
			}

			vehicle_command_ack_s command_ack{};
			command_ack.command = vehicle_command.command;
			command_ack.target_system = vehicle_command.source_system;
			command_ack.target_component = vehicle_command.source_component;
			command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

			if (index != -1) {
				PX4_INFO("Sending command: index: %i type: %i", index, type);
				_current_command.command = dshot_command_t::DSHOT_CMD_MOTOR_STOP;
				_current_command.num_repetitions = 10; // TODO: why do we always send a command 10x?
				_current_command.save = true;

				switch (type) {
				case ACTUATOR_CONFIGURATION_BEEP:
					_current_command.command = dshot_command_t::DSHOT_CMD_BEEP1;
					break;

				case ACTUATOR_CONFIGURATION_3D_MODE_OFF:
					_current_command.command = dshot_command_t::DSHOT_CMD_3D_MODE_OFF;
					break;

				case ACTUATOR_CONFIGURATION_3D_MODE_ON:
					_current_command.command = dshot_command_t::DSHOT_CMD_3D_MODE_ON;
					break;

				case ACTUATOR_CONFIGURATION_SPIN_DIRECTION1:
					_current_command.command = dshot_command_t::DSHOT_CMD_SPIN_DIRECTION_1;
					break;

				case ACTUATOR_CONFIGURATION_SPIN_DIRECTION2:
					_current_command.command = dshot_command_t::DSHOT_CMD_SPIN_DIRECTION_2;
					break;

				case ACTUATOR_CONFIGURATION_WRITE_SETTING:

				// This is a special command that triggers 4 DShot commands:
				// - DSHOT_CMD_ENTER_PROGRAMMING_MODE
				// - EEPROM Memory location
				// - Value
				// - DSHOT_CMD_EXIT_PROGRAMMING_MODE


				// TODO: implement
				// _current_command.command = dshot_command_t::DSHOT_CMD_ENTER_PROGRAMMING_MODE;
				// _current_command.num_repetitions = 1;
				// _current_command.save = false;

				// Command structure
				// param1 = ACTUATOR_CONFIGURATION_WRITE_SETTING
				// param2 = Memory location
				// param3 = Value
				// param5 = ACTUATOR_OUTPUT_FUNCTION_MOTOR1

				// break;


				// fallthrough!!

				default:
					PX4_WARN("unknown command: %i", type);
					break;
				}

				// NOTE: this means we can't send a stop command (that's OK)
				if (_current_command.command != dshot_command_t::DSHOT_CMD_MOTOR_STOP) {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					_current_command.motor_mask = 1 << index;
				}
			}

			command_ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(command_ack);
		}
	}
}

void DShot::update_params()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	// Calculate minimum DShot output as percent of throttle and constrain.
	float min_value = _param_dshot_min.get() * (float)DSHOT_MAX_THROTTLE;
	uint16_t dshot_min_value = math::constrain((uint16_t)min_value, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

	_mixing_output.setAllMinValues(dshot_min_value);

	// Do not use the minimum parameter for reversible outputs
	for (unsigned i = 0; i < _num_outputs; ++i) {
		if ((1 << i) & _reversible_outputs) {
			_mixing_output.minValue(i) = DSHOT_MIN_THROTTLE;
		}
	}
}

int DShot::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	int myoptind = 1;
	bool swap_rxtx = false;
	const char *device_name = nullptr;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:xd:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'x':
			swap_rxtx = true;
			break;

		case 'd':
			device_name = myoptarg;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	if (!strcmp(verb, "telemetry")) {
		if (device_name) {
			// telemetry can be requested before the module is started
			strncpy(_telemetry_device, device_name, sizeof(_telemetry_device) - 1);
			_telemetry_device[sizeof(_telemetry_device) - 1] = '\0';
			_telemetry_swap_rxtx = swap_rxtx;
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	if (!is_running()) {
		int ret = DShot::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int DShot::print_status()
{
	PX4_INFO("Outputs used: 0x%" PRIx32, _output_mask);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_bdshot_rpm_perf);
	perf_print_counter(_dshot_telem_perf);

	_mixing_output.printStatus();

	if (_telemetry) {
		PX4_INFO("telemetry on: %s", _telemetry_device);
		_telemetry->printStatus();
	}

	/* Print dshot status */
	if (_bidirectional_dshot_enabled) {
		up_bdshot_status();
	}

	return 0;
}

int DShot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the DShot output driver. It can be used as drop-in replacement
to use DShot as ESC communication protocol instead of PWM.

It supports:
- DShot150, DShot300, DShot600
- telemetry via separate UART and publishing as esc_status message

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "UART device", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Swap RX/TX pins", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dshot_main(int argc, char *argv[])
{
	return DShot::main(argc, argv);
}

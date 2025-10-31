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
	perf_free(_bdshot_success_perf);
	perf_free(_bdshot_error_perf);
	perf_free(_bdshot_timeout_perf);
	perf_free(_telem_success_perf);
	perf_free(_telem_error_perf);
	perf_free(_telem_timeout_perf);
	perf_free(_telem_allsampled_perf);
}

int DShot::init()
{

	update_params();

	if (initialize_dshot()) {
		ScheduleNow();
		return PX4_OK;
	}

	return PX4_ERROR;
}

bool DShot::initialize_dshot()
{
	unsigned int dshot_frequency = 0;
	uint32_t dshot_frequency_param = 0;

	// Mark all channels as used, we will disable the unused groups below
	_output_mask = (1u << DSHOT_MAXIMUM_CHANNELS) - 1;

	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {

		// Get mask of actuator channels associated with this timer group
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
			// Timer group is not configured for dshot, disable channels in output mask
			_output_mask &= ~channels;
		}

		if (dshot_frequency_request != 0) {
			if (dshot_frequency != 0 && dshot_frequency != dshot_frequency_request) {
				PX4_WARN("Only supporting a single frequency (%u), adjusting param %s", dshot_frequency, param_name);
				param_set_no_notification(handle, &dshot_frequency_param);

			} else {
				dshot_frequency = dshot_frequency_request;
				dshot_frequency_param = tim_config;
			}
		}
	}

	int ret = up_dshot_init(_output_mask, dshot_frequency, _param_dshot_bidir_en.get());

	if (ret < 0) {
		PX4_ERR("up_dshot_init failed (%i)", ret);
		return false;
	}

	if ((uint32_t)ret != _output_mask) {
		PX4_INFO("Failed to configure some channels");
		PX4_INFO("requested: 0x%lx", _output_mask);
		PX4_INFO("configured: 0x%lx", (uint32_t)ret);
		_output_mask = ret;
	}

	// Set our mixer to explicitly disable channels we do not control
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		if (((1 << i) & _output_mask) == 0) {
			_mixing_output.disableFunction(i);
		}
	}

	if (_output_mask == 0) {
		PX4_WARN("No channels configured");
		return false;
	}

	up_dshot_arm(true);

	return true;
}

bool DShot::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!count_set_bits(_output_mask)) {
		PX4_WARN("DShot::updateOutputs called when no Motors have been set!");
		return false;
	}

	// First check if all outputs are disarmed and we have a command to send
	bool all_disarmed = true;

	for (int i = 0; i < (int)num_outputs; i++) {
		if (!_mixing_output.isMotor(i)) {
			// If unconfigured ensure output value is set back to disarmed with no telemetry
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, false);
			continue;
		}

		if (outputs[i] != DSHOT_DISARM_VALUE) {
			all_disarmed = false;
			_esc_status.esc_armed_flags |= (1 << i);

		} else {
			_esc_status.esc_armed_flags &= ~(1 << i);
		}
	}

	// All outputs are disarmed and we have a command to send
	if (all_disarmed && _current_command.valid()) {

		int motor_index = 0;

		for (int i = 0; i < (int)num_outputs; i++) {

			if (!_mixing_output.isMotor(i)) {
				continue;
			}

			// Send command to the motor if there isn't telemtry already in progress
			bool this_motor = _current_command.motor_mask & (1 << i);
			bool telemtry_idle = _telemetry.enabled() && _telemetry.telemetryRequestFinished();

			if (this_motor && telemtry_idle) {

				DSHOT_CMD_DEBUG("Sending command %u to motor %d", _current_command.command, motor_index);

				if (_current_command.expect_response) {
					_telemetry.setExpectCommandResponse(motor_index, _current_command.command);
				}

				up_dshot_motor_command(i, _current_command.command, false);

				// Decrement command repetition counter
				--_current_command.num_repetitions;

				// Queue a save command after the burst if save has been requested
				if (_current_command.num_repetitions == 0 && _current_command.save) {
					_current_command.save = false;
					_current_command.num_repetitions = 10;
					_current_command.command = DSHOT_CMD_SAVE_SETTINGS;
				}

			} else {
				up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, false);
			}

			motor_index++;
		}

		up_dshot_trigger();
		return true;
	}

	// Clear pending commands if any output is not disarmed
	_current_command.clear();
	_programming_state = ProgrammingState::Idle;

	bool request_telemetry = false;

	// If telemetry is enabled and the last request has been processed
	bool delay_elapsed = hrt_absolute_time() > _telem_delay_until;

	if (_telemetry.enabled() && _telemetry.telemetryRequestFinished()) {

		if (!delay_elapsed) {
			// Flush UART buffer while waiting to send telem request. Ensures
			// garbage data doesn't corrupt our starting point for telem.
			_telemetry.flush();

		} else {
			DSHOT_TELEM_DEBUG("startTelemetryRequest: %d", _telemetry_motor_index);
			_telemetry.startTelemetryRequest();
			request_telemetry = true;
		}
	}

	// Iterate over all of the outputs
	for (int i = 0; i < (int)num_outputs; i++) {

		if (!_mixing_output.isMotor(i)) {
			continue;
		}

		uint16_t output = outputs[i];

		bool set_telemetry_bit = request_telemetry && (_telemetry_motor_index == i);

		if (output == DSHOT_DISARM_VALUE) {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, set_telemetry_bit);

		} else {
			// Reverse output if required
			if (_param_dshot_3d_enable.get() || (_mixing_output.reversibleOutputs() & (1u << i))) {
				output = convert_output_to_3d_scaling(output);
			}

			up_dshot_motor_data_set(i, math::min(output, DSHOT_MAX_THROTTLE), set_telemetry_bit);
		}
	}

	up_dshot_trigger();

	return true;
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

	// special handling for the DShot Programming sequence
	if (_programming_state != ProgrammingState::Idle) {
		handle_programming_sequence_state();
	}

	// Updates the DShot throttle output and sends DShot commands
	_mixing_output.update();

	bool all_telem_sampled = false;
	bool all_bdshot_sampled = false;

	if (_telemetry.enabled()) {

		if (_telemetry.expectingCommandResponse()) {

			if (_telemetry.parseCommandResponse()) {
				DSHOT_CMD_DEBUG("Command response received");
			}

		} else {

			EscData esc {};
			esc.motor_index = _telemetry_motor_index;

			switch (_telemetry.parseTelemetryPacket(&esc)) {
			case TelemetryStatus::NotStarted:
				// no-op, should not hit this case
				break;

			case TelemetryStatus::NotReady:
				// no-op, will eventually timeout
				break;

			case TelemetryStatus::Ready:
				_serial_telem_online_mask |= (1 << _telemetry_motor_index);
				consume_esc_data(esc, TelemetrySource::Serial);
				all_telem_sampled = set_next_telemetry_index();
				perf_count(_telem_success_perf);
				break;

			case TelemetryStatus::Timeout:
				// Set ESC data to zeroes
				DSHOT_TELEM_DEBUG("Timeout");
				_serial_telem_online_mask &= ~(1 << _telemetry_motor_index);
				// Consume an empty EscData to zero the data
				consume_esc_data(esc, TelemetrySource::Serial);
				all_telem_sampled = set_next_telemetry_index();
				perf_count(_telem_timeout_perf);
				break;

			case TelemetryStatus::ParseError:
				// Set ESC data to zeroes
				DSHOT_TELEM_DEBUG("ParseError");
				_serial_telem_online_mask &= ~(1 << _telemetry_motor_index);
				// Consume an empty EscData to zero the data
				consume_esc_data(esc, TelemetrySource::Serial);
				all_telem_sampled = set_next_telemetry_index();
				_telem_delay_until = hrt_absolute_time() + 100_ms; // TODO: how long do we need to wait?
				perf_count(_telem_error_perf);
				break;
			}
		}
	}

	if (_param_dshot_bidir_en.get()) {

		// TODO: We will have Extended BDShot available.
		// - temp C
		// - voltage
		// - current

		all_bdshot_sampled = process_bdshot_erpm();

		if (all_bdshot_sampled) {
			perf_count(_bdshot_success_perf);
		}
	}

	if (all_telem_sampled || all_bdshot_sampled) {
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status.esc_count = count_set_bits(_output_mask);
		_esc_status.counter++;
		DSHOT_TELEM_DEBUG("publish");
		_esc_status_pub.update(_esc_status);
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

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

bool DShot::set_next_telemetry_index()
{
	int start_index = (_telemetry_motor_index + 1) % DSHOT_MAXIMUM_CHANNELS;
	int next_motor_index = (_telemetry_motor_index + 1) % DSHOT_MAXIMUM_CHANNELS;

	do {
		bool is_motor = _mixing_output.isMotor(next_motor_index);
		bool already_requested = _telemetry_requested_mask & (1 << next_motor_index);

		if (is_motor && !already_requested) {
			_telemetry_motor_index = next_motor_index;
			_telemetry_requested_mask |= (1 << next_motor_index);
			DSHOT_TELEM_DEBUG("next telem %d, mask: 0x%lx", _telemetry_motor_index, _telemetry_requested_mask);
			break;
		}

		next_motor_index = (next_motor_index + 1) % DSHOT_MAXIMUM_CHANNELS;
	} while (next_motor_index != start_index);


	// Check if all motors have been sampled
	if (count_set_bits(_telemetry_requested_mask) >= count_set_bits(_output_mask)) {
		_telemetry_requested_mask = 0;
		perf_count(_telem_allsampled_perf);
		DSHOT_TELEM_DEBUG("all sampled");
		return true;
	}

	return false;
}

void DShot::consume_esc_data(const EscData &esc, TelemetrySource source)
{
	bool bidirectional_enabled = _param_dshot_bidir_en.get();
	bool serial_telemetry_enabled = _param_dshot_tel_cfg.get();

	if (esc.motor_index < esc_status_s::CONNECTED_ESC_MAX) {

		_esc_status.counter = _esc_status_counter++;

		uint8_t online_mask = 0xFF;

		// Require both sources online when enabled
		if (bidirectional_enabled) {
			online_mask &= _bdshot_telem_online_mask;
		}

		if (serial_telemetry_enabled) {
			online_mask &= _serial_telem_online_mask;
		}

		_esc_status.esc_online_flags = online_mask;

		if (source == TelemetrySource::Serial) {
			// Only use SerialTelemetry eRPM when BDSoht is disabled
			if (!bidirectional_enabled) {
				_esc_status.esc[esc.motor_index].timestamp = esc.timestamp;
				_esc_status.esc[esc.motor_index].esc_rpm = ((int)(esc.erpm) * 100) / (_param_mot_pole_count.get() / 2);
			}

			_esc_status.esc[esc.motor_index].esc_voltage = (float)(esc.voltage) * 0.01f;
			_esc_status.esc[esc.motor_index].esc_current = (float)(esc.current) * 0.01f;
			_esc_status.esc[esc.motor_index].esc_temperature = (float)(esc.temperature);

		} else if (source == TelemetrySource::BDShot) {
			_esc_status.esc[esc.motor_index].timestamp = esc.timestamp;
			_esc_status.esc[esc.motor_index].esc_rpm = ((int)(esc.erpm) * 100) / (_param_mot_pole_count.get() / 2);

			// Only use BDShot Volt/Curr/Temp when Serial Telemetry is disabled
			if (!serial_telemetry_enabled) {
				_esc_status.esc[esc.motor_index].esc_voltage = (float)(esc.voltage) * 0.01f;
				_esc_status.esc[esc.motor_index].esc_current = (float)(esc.current) * 0.01f;
				_esc_status.esc[esc.motor_index].esc_temperature = (float)(esc.temperature);
			}
		}
	}
}

bool DShot::process_bdshot_erpm()
{
	hrt_abstime now = hrt_absolute_time();

	// Don't try to process any telem data until after ESCs have been given time to boot
	if (now < ESC_INIT_TELEM_WAIT_TIME) {
		return false;
	}

	// We wait until all are ready.
	if (up_bdshot_num_erpm_ready() < count_set_bits(_output_mask)) {
		return false;
	}

	for (unsigned i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {
		if (!_mixing_output.isMotor(i)) {
			continue;
		}

		// TODO: handle Extended Telemetry -- Volt/Curr/Temp
		// We won't want to zero out the EscData, and instead
		// use the previously set values.

		EscData esc = {};

		// NOTE: dshot erpm order is actuator channel order, so we map to motor index here
		int motor_index = (int)_mixing_output.outputFunction(i) - (int)OutputFunction::Motor1;

		if ((motor_index >= 0) && (motor_index < esc_status_s::CONNECTED_ESC_MAX)) {

			esc.motor_index = motor_index;
			esc.timestamp = now;

			if (up_bdshot_channel_status(i)) {
				_bdshot_telem_online_mask |= (1 << motor_index);

				// Only update RPM if online
				int erpm = 0;
				if (up_bdshot_get_erpm(i, &erpm) == PX4_OK) {
					esc.erpm = (erpm * 100) / (_param_mot_pole_count.get() / 2);
				}

			} else {
				_bdshot_telem_online_mask &= ~(1 << motor_index);

				// TODO: periodic warning about dshot online status:
				// - motor wires are not hooked up cleanly
				// - parsing logic is timing dependent...
			}

			consume_esc_data(esc, TelemetrySource::BDShot);
		}
	}

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

void DShot::handle_programming_sequence_state()
{
	switch (_programming_state) {
	case ProgrammingState::EnterMode:
		_current_command.command = DSHOT_CMD_ENTER_PROGRAMMING_MODE;
		_current_command.num_repetitions = 10;
		_programming_state = ProgrammingState::SendAddress;
		break;

	case ProgrammingState::SendAddress:
		_current_command.command = _programming_address;
		_current_command.num_repetitions = 1;
		_programming_state = ProgrammingState::SendValue;
		break;

	case ProgrammingState::SendValue:
		_current_command.command = _programming_value;
		_current_command.num_repetitions = 1;
		_programming_state = ProgrammingState::ExitMode;
		break;

	case ProgrammingState::ExitMode:
		_current_command.command = DSHOT_CMD_EXIT_PROGRAMMING_MODE;
		_current_command.num_repetitions = 1;
		_programming_state = ProgrammingState::Save;
		break;

	case ProgrammingState::Save:
		_current_command.command = DSHOT_CMD_SAVE_SETTINGS;
		_current_command.num_repetitions = 10;
		_programming_state = ProgrammingState::Idle;
		break;

	default:
		break;
	}
}

void DShot::handle_vehicle_commands()
{
	vehicle_command_s vehicle_command;

	while (!_current_command.valid() && _vehicle_command_sub.update(&vehicle_command)) {

		// https://mavlink.io/en/messages/common.html#MAV_CMD_CONFIGURE_ACTUATOR
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR) {
			int function = (int)(vehicle_command.param5 + 0.5);

			DSHOT_CMD_DEBUG("Received VEHICLE_CMD_CONFIGURE_ACTUATOR");

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

			for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; ++i) {
				if ((int)_mixing_output.outputFunction(i) == function && _mixing_output.isMotor(i)) {
					index = i;
				}
			}

			vehicle_command_ack_s command_ack{};
			command_ack.command = vehicle_command.command;
			command_ack.target_system = vehicle_command.source_system;
			command_ack.target_component = vehicle_command.source_component;
			command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

			if (index != -1) {
				DSHOT_CMD_DEBUG("index: %i type: %i", index, type);
				_current_command.clear();
				_current_command.command = DSHOT_CMD_MOTOR_STOP;
				_current_command.num_repetitions = 10; // TODO: check AM32
				_current_command.save = false;

				switch (type) {
				case ACTUATOR_CONFIGURATION_BEEP:
					_current_command.command = DSHOT_CMD_BEEP1;
					break;

				case ACTUATOR_CONFIGURATION_3D_MODE_OFF:
					_current_command.command = DSHOT_CMD_3D_MODE_OFF;
					break;

				case ACTUATOR_CONFIGURATION_3D_MODE_ON:
					_current_command.command = DSHOT_CMD_3D_MODE_ON;
					break;

				case ACTUATOR_CONFIGURATION_SPIN_DIRECTION1:
					_current_command.command = DSHOT_CMD_SPIN_DIRECTION_1;
					_current_command.save = true;
					break;

				case ACTUATOR_CONFIGURATION_SPIN_DIRECTION2:
					_current_command.command = DSHOT_CMD_SPIN_DIRECTION_2;
					_current_command.save = true;
					break;

				case ACTUATOR_CONFIGURATION_READ_SETTINGS:
					DSHOT_CMD_DEBUG("ACTUATOR_CONFIGURATION_READ_SETTINGS");
					_current_command.save = false;
					_current_command.num_repetitions = 6; // NOTE: AM32 requires 6 to consider a command valid
					_current_command.command = DSHOT_CMD_ESC_INFO;
					_current_command.expect_response = true;
					break;

				case ACTUATOR_CONFIGURATION_WRITE_SETTING:
					DSHOT_CMD_DEBUG("ACTUATOR_CONFIGURATION_WRITE_SETTING");
					// This is a special command that triggers 5 DShot commands:
					// - DSHOT_CMD_ENTER_PROGRAMMING_MODE
					// - EEPROM Memory location
					// - Value
					// - DSHOT_CMD_EXIT_PROGRAMMING_MODE
					// - DSHOT_CMD_SAVE_SETTINGS

					// Command structure
					// param1 = ACTUATOR_CONFIGURATION_WRITE_SETTING
					// param2 = Memory location
					// param3 = Value
					// param5 = ACTUATOR_OUTPUT_FUNCTION_MOTOR1

					_current_command.save = false;
					_programming_address = (uint16_t)vehicle_command.param2;
					_programming_value = (uint16_t)vehicle_command.param3;
					_programming_state = ProgrammingState::EnterMode;
					break;

				default:
					PX4_WARN("unknown command: %i", type);
					break;
				}

				// NOTE: this means we can't send a stop command (that's OK)
				if (_current_command.command != DSHOT_CMD_MOTOR_STOP) {
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
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		if ((1 << i) & _mixing_output.reversibleOutputs()) {
			_mixing_output.minValue(i) = DSHOT_MIN_THROTTLE;
		}
	}
}

void DShot::mixerChanged()
{
	PX4_INFO("mixerChanged");
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;

	int motor_count = 0;

	for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {

		if (_mixing_output.isMotor(i)) {
			_esc_status.esc[i].actuator_function = (uint8_t)_mixing_output.outputFunction(i);
			motor_count++;

			if (!(_output_mask & (1 << i))) {
				PX4_INFO("Enabling channel %d", i);
			}

			_output_mask |= (1 << i);

		} else {
			if ((_output_mask & (1 << i))) {
				PX4_INFO("Disabling channel %d", i);
			}

			_output_mask &= ~(1 << i);
		}
	}

	// TODO: re-init dshot.c output channels and stuff
}

void DShot::init_telemetry(const char *device, bool swap_rxtx)
{
	if (!device) {
		return;
	}

	if (_telemetry.init(device, swap_rxtx) != PX4_OK) {
		PX4_ERR("telemetry init failed");
	}

	// Advertise early to ensure we beat uavcan. We need to enforce ordering somehow.
	_esc_status_pub.advertise();
}

int DShot::print_status()
{
	PX4_INFO("Outputs used: 0x%" PRIx32, _output_mask);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_bdshot_success_perf);
	perf_print_counter(_bdshot_error_perf);
	perf_print_counter(_bdshot_timeout_perf);

	perf_print_counter(_telem_success_perf);
	perf_print_counter(_telem_error_perf);
	perf_print_counter(_telem_timeout_perf);
	perf_print_counter(_telem_allsampled_perf);

	_mixing_output.printStatus();

	if (_telemetry.enabled()) {
		PX4_INFO("telemetry on: %s", _telemetry_device);
		_telemetry.printStatus();
	}

	if (_param_dshot_bidir_en.get()) {
		up_bdshot_status();
	}

	return 0;
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


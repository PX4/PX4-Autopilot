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

void DShot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	if (process_serial_telemetry() || process_bdshot_telemetry()) {
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status.esc_count = count_set_bits(_output_mask);
		_esc_status.counter++;
		_esc_status_pub.update(_esc_status);
	}

	if (_parameter_update_sub.updated()) {
		update_params();
	}

	// Telemetry init hook
	if (_request_telemetry_init.load()) {
		init_telemetry(_telemetry_device, _telemetry_swap_rxtx);
		_request_telemetry_init.store(false);
	}

	handle_vehicle_commands();

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

bool DShot::updateOutputs(uint16_t *outputs, unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!count_set_bits(_output_mask)) {
		return false;
	}

	// Get the armed mask
	_esc_status.esc_armed_flags = esc_armed_mask(outputs, num_outputs);

	// Set the state
	_state = _esc_status.esc_armed_flags ? State::Armed : State::Disarmed;

	switch (_state) {
	case State::Armed: {
			update_motor_outputs(outputs, num_outputs);
			break;
		}

	case State::Disarmed: {

			// Select next command to send (if any)
			if (_telemetry.telemetryResponseFinished() &&
			    _current_command.finished() && _telemetry.commandResponseFinished()) {
				select_next_command();
			}

			// Send command if available
			if (!_current_command.finished()) {
				update_motor_commands(num_outputs);

			} else {
				// Otherwise idle
				update_motor_outputs(outputs, num_outputs);
			}

			break;
		}
	}

	up_dshot_trigger();

	return true;
}

// TODO: this needs a refactor
void DShot::select_next_command()
{
	// Command order or priority:
	// - EDT Request
	// - Settings Request
	// - DShot Programming

	bool ser_tel_enabled = _param_dshot_tel_cfg.get();
	bool bidir_enabled = _param_dshot_bidir_en.get();
	bool edt_enabled = _param_dshot_bidir_edt.get();
	bool esc_type_set = _param_dshot_esc_type.get();

	// EDT Request mask
	uint8_t needs_edt_request_mask = _bdshot_telem_online_mask & ~_bdshot_edt_requested_mask;

	// Settings Request mask
	uint8_t needs_settings_request_mask = _serial_telem_online_mask & ~_settings_requested_mask;

	// TODO: If ESC has not been online for at least one second, skip it

	// Settings Programming
	// NOTE: only update when we're not actively programming an ESC
	if (!_dshot_programming_active) {
		if (_am32_eeprom_write_sub.updated()) {

			// TODO: because uORB can't queue (what is the point of the ORB_QUEUE_LENGTH then??)
			auto last = _am32_eeprom_write_sub.get_last_generation();
			_am32_eeprom_write_sub.copy(&_am32_eeprom_write);
			auto current = _am32_eeprom_write_sub.get_last_generation();

			if (current != last + 1) {
				PX4_ERR("am32_eeprom_write lost, generation %u -> %u", last, current);
			}

			PX4_INFO("ESC%u: starting programming mode", _am32_eeprom_write.index + 1);
			_dshot_programming_active = true;
		}
	}

	_current_command.clear();

	if (bidir_enabled && edt_enabled && needs_edt_request_mask) {
		// EDT Request first
		int next_motor_index = 0;

		for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {
			if (needs_edt_request_mask & (1 << i)) {
				next_motor_index = i;
				break;
			}
		}

		auto now = hrt_absolute_time();
		_current_command.num_repetitions = 10;
		_current_command.command = DSHOT_EXTENDED_TELEMETRY_ENABLE;
		_current_command.motor_mask = (1 << next_motor_index);
		_bdshot_edt_requested_mask |= (1 << next_motor_index);
		PX4_INFO("ESC%d: requesting EDT at time %.2fs", next_motor_index + 1, (double)now / 1000000.);

	} else if (ser_tel_enabled && esc_type_set && needs_settings_request_mask) {
		// Settings Request next
		int next_motor_index = 0;

		for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {
			if (needs_settings_request_mask & (1 << i)) {
				next_motor_index = i;
				break;
			}
		}

		auto now = hrt_absolute_time();
		_current_command.num_repetitions = 6;
		_current_command.command = DSHOT_CMD_ESC_INFO;
		_current_command.motor_mask = (1 << next_motor_index);
		_current_command.expect_response = true;
		_settings_requested_mask |= (1 << next_motor_index);
		PX4_INFO("ESC%d: requesting Settings at time %.2fs", next_motor_index + 1, (double)now / 1000000.);

	} else if (_dshot_programming_active) {
		// Per-setting dshot programming state machine
		if (_programming_state == ProgrammingState::Idle) {
			// Get next setting address/value to program
			int next_index = -1;

			// Find settings that need to be written but haven't been yet
			for (int i = 0; i < 48; i++) {
				int array_index = i / 32;
				int bit_index = i % 32;

				bool needs_write = _am32_eeprom_write.write_mask[array_index] & (1 << bit_index);
				bool already_written = _settings_written_mask[array_index] & (1 << bit_index);

				if (needs_write && !already_written) {
					next_index = i;
					break;
				}
			}

			if (next_index >= 0) {
				// Set up the motor mask based on the index in the write request
				if (_am32_eeprom_write.index == 255) {
					// _current_command.motor_mask = 0xFF;  // Apply to all ESCs
					PX4_INFO("ESC ALL: Writing setting at index %d, value %u", next_index, _am32_eeprom_write.data[next_index]);

				} else {
					// _current_command.motor_mask = (1 << _am32_eeprom_write.index);
					PX4_INFO("ESC%d: Writing setting at index %d, value %u", _am32_eeprom_write.index + 1, next_index,
						 _am32_eeprom_write.data[next_index]);
				}

				_programming_address = next_index;
				_programming_value = _am32_eeprom_write.data[next_index];
				_programming_state = ProgrammingState::EnterMode;

				// Pre-emptively Mark this setting as written
				int array_index = next_index / 32;
				int bit_index = next_index % 32;
				_settings_written_mask[array_index] |= (1 << bit_index);

			} else {
				// All settings have been written
				PX4_INFO("All settings written!");
				_dshot_programming_active = false;
				// _programming_state = ProgrammingState::Save;
				_current_command.command = DSHOT_CMD_SAVE_SETTINGS;
				_current_command.num_repetitions = 6;
				_current_command.motor_mask = _am32_eeprom_write.index == 255 ? 255 : (1 << _am32_eeprom_write.index);
				_programming_state = ProgrammingState::Idle;

				// TODO: do we want to re-request and compare?
				// Clear the written mask for this motor for next time
				_settings_written_mask[0] = 0;
				_settings_written_mask[1] = 0;

				// Mark as offline and unread so that we read again
				_serial_telem_online_mask &= ~(_am32_eeprom_write.index == 255 ? 255 : (1 << _am32_eeprom_write.index));
				_settings_requested_mask &= ~(_am32_eeprom_write.index == 255 ? 255 : (1 << _am32_eeprom_write.index));

				_telem_delay_until = hrt_absolute_time() + 500_ms;
			}
		}

		switch (_programming_state) {
		case ProgrammingState::EnterMode:
			_current_command.command = DSHOT_CMD_ENTER_PROGRAMMING_MODE;
			_current_command.num_repetitions = 6;
			_current_command.motor_mask = _am32_eeprom_write.index == 255 ? 255 : (1 << _am32_eeprom_write.index);
			_programming_state = ProgrammingState::SendAddress;
			break;

		case ProgrammingState::SendAddress:
			_current_command.command = _programming_address;
			_current_command.num_repetitions = 1;
			_current_command.motor_mask = _am32_eeprom_write.index == 255 ? 255 : (1 << _am32_eeprom_write.index);
			_programming_state = ProgrammingState::SendValue;
			break;

		case ProgrammingState::SendValue:
			_current_command.command = _programming_value;
			_current_command.num_repetitions = 1;
			_current_command.motor_mask = _am32_eeprom_write.index == 255 ? 255 : (1 << _am32_eeprom_write.index);
			_programming_state = ProgrammingState::ExitMode;
			break;

		case ProgrammingState::ExitMode:
			_current_command.command = DSHOT_CMD_EXIT_PROGRAMMING_MODE;
			_current_command.num_repetitions = 1;
			_current_command.motor_mask = _am32_eeprom_write.index == 255 ? 255 : (1 << _am32_eeprom_write.index);
			_programming_state = ProgrammingState::Idle;
			break;

		default:
			break;
		}
	}
}

void DShot::update_motor_outputs(uint16_t outputs[MAX_ACTUATORS], int num_outputs)
{
	for (int i = 0; i < num_outputs; i++) {

		if (!_mixing_output.isMotor(i)) {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, false);
			continue;
		}

		bool set_telemetry_bit = false;

		if (_telemetry_motor_index == i) {
			if (_param_dshot_tel_cfg.get() && _telemetry.telemetryResponseFinished() && _telemetry.commandResponseFinished()) {
				if (hrt_absolute_time() > _telem_delay_until) {
					set_telemetry_bit = true;
					_telemetry.startTelemetryRequest();
				}
			}
		}

		if (outputs[i] == DSHOT_DISARM_VALUE) {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, set_telemetry_bit);

		} else {
			up_dshot_motor_data_set(i, calculate_output_value(outputs[i], i), set_telemetry_bit);
		}
	}
}

void DShot::update_motor_commands(int num_outputs)
{
	bool command_sent = false;

	for (int i = 0; i < num_outputs; i++) {

		uint16_t command = DSHOT_CMD_MOTOR_STOP;

		if (_mixing_output.isMotor(i)) {

			int motor_index = (int)_mixing_output.outputFunction(i) - (int)OutputFunction::Motor1;

			if (_current_command.motor_mask & (1 << motor_index)) {

				if (_current_command.expect_response) {
					_telemetry.setExpectCommandResponse(motor_index, _current_command.command);
				}

				// PX4_INFO("Writing: ESC%d, value: %u", motor_index + 1, _current_command.command);
				command = _current_command.command;
				command_sent = true;
			}
		}

		up_dshot_motor_command(i, command, false);
	}

	if (command_sent) {
		// Decrement command repetition counter
		--_current_command.num_repetitions;

		// Queue a save command after the burst if save has been requested
		if (_current_command.num_repetitions == 0 && _current_command.save) {
			_current_command.save = false;
			_current_command.num_repetitions = 10;
			_current_command.command = DSHOT_CMD_SAVE_SETTINGS;
		}
	}
}

uint8_t DShot::esc_armed_mask(uint16_t *outputs, int num_outputs)
{
	uint8_t mask = 0;

	for (int i = 0; i < num_outputs; i++) {
		if (_mixing_output.isMotor(i)) {
			if (outputs[i] != DSHOT_DISARM_VALUE) {
				mask |= (1 << i);
			}
		}
	}

	return mask;
}

uint16_t DShot::calculate_output_value(uint16_t raw, int index)
{
	uint16_t output = raw;

	// Reverse output if required
	if (_param_dshot_3d_enable.get() || (_mixing_output.reversibleOutputs() & (1u << index))) {
		output = convert_output_to_3d_scaling(raw);
	}

	output = math::min(output, DSHOT_MAX_THROTTLE);

	return output;
}

bool DShot::process_serial_telemetry()
{
	if (!_param_dshot_tel_cfg.get()) {
		return false;
	}

	bool all_telem_sampled = false;

	if (!_telemetry.commandResponseFinished()) {
		_telemetry.parseCommandResponse();

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

			if (_serial_telem_online_mask & (1 << _telemetry_motor_index)) {
				consume_esc_data(esc, TelemetrySource::Serial);
				all_telem_sampled = set_next_telemetry_index();
				perf_count(_telem_success_perf);

			} else {
				hrt_abstime now = hrt_absolute_time();
				if (_serial_telem_online_timestamps[_telemetry_motor_index] == 0) {
					_serial_telem_online_timestamps[_telemetry_motor_index] = now;
				}

				// Mark as online only after 100_ms without errors
				if (now - _serial_telem_online_timestamps[_telemetry_motor_index] > 100_ms) {
					_serial_telem_online_mask |= (1 << _telemetry_motor_index);
				}
			}

			break;

		case TelemetryStatus::Timeout:
			// Set ESC data to zeroes
			// PX4_WARN("Telem timeout");
			_serial_telem_errors[_telemetry_motor_index]++;
			_serial_telem_online_mask &= ~(1 << _telemetry_motor_index);
			_serial_telem_online_timestamps[_telemetry_motor_index] = 0;
			// Consume an empty EscData to zero the data
			consume_esc_data(esc, TelemetrySource::Serial);
			all_telem_sampled = set_next_telemetry_index();
			perf_count(_telem_timeout_perf);
			break;

		case TelemetryStatus::ParseError:
			// Set ESC data to zeroes
			PX4_WARN("Telem parse error");
			_serial_telem_errors[_telemetry_motor_index]++;
			_serial_telem_online_mask &= ~(1 << _telemetry_motor_index);
			_serial_telem_online_timestamps[_telemetry_motor_index] = 0;
			// Consume an empty EscData to zero the data
			consume_esc_data(esc, TelemetrySource::Serial);
			all_telem_sampled = set_next_telemetry_index();
			_telem_delay_until = hrt_absolute_time() + 100_ms; // TODO: how long do we need to wait?
			perf_count(_telem_error_perf);
			break;
		}
	}

	return all_telem_sampled;
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
			break;
		}

		next_motor_index = (next_motor_index + 1) % DSHOT_MAXIMUM_CHANNELS;
	} while (next_motor_index != start_index);

	// Check if all motors have been sampled
	if (count_set_bits(_telemetry_requested_mask) >= count_set_bits(_output_mask)) {
		_telemetry_requested_mask = 0;
		perf_count(_telem_allsampled_perf);
		return true;
	}

	return false;
}

bool DShot::process_bdshot_telemetry()
{
	if (!_param_dshot_bidir_en.get()) {
		return false;
	}

	// TODO: We will have Extended BDShot available.
	// - temp C
	// - voltage
	// - current

	hrt_abstime now = hrt_absolute_time();

	// Don't try to process any telem data until after ESCs have been given time to boot
	if (now < _telem_delay_until) {
		return false;
	}

	// We wait until all are ready.
	if (up_bdshot_num_channels_ready() < count_set_bits(_output_mask)) {
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

			_bdshot_telem_errors[motor_index] = up_bdshot_num_errors(i);

			if (up_bdshot_channel_online(i)) {
				_bdshot_telem_online_mask |= (1 << motor_index);

				// Only update RPM if online
				int erpm = 0;

				if (up_bdshot_get_erpm(i, &erpm) == PX4_OK) {
					esc.erpm = erpm * 100;
				}

				// Extended DShot Telemetry
				if (_param_dshot_bidir_edt.get()) {

					uint8_t value = 0;
					if (up_bdshot_get_extended_telemetry(i, DSHOT_EDT_TEMPERATURE, &value) == PX4_OK) {
						// BDShot temperature is in C
						esc.temperature = value;
						PX4_INFO("ESC%d: temperature: %f", motor_index, (double)esc.temperature);
					} else {
						// If no update is available, use the previous value
						esc.temperature = _esc_status.esc[motor_index].esc_temperature;
					}

					if (up_bdshot_get_extended_telemetry(i, DSHOT_EDT_VOLTAGE, &value) == PX4_OK) {
						// BDShot voltage is in 0.25V
						esc.voltage = value * 0.25f;
						PX4_INFO("ESC%d: voltage: %f", motor_index, (double)esc.voltage);
					} else {
						// If no update is available, use the previous value
						esc.voltage = _esc_status.esc[motor_index].esc_voltage;
					}

					if (up_bdshot_get_extended_telemetry(i, DSHOT_EDT_CURRENT, &value) == PX4_OK) {
						// BDShot current is in 0.5V
						esc.current = value * 0.5f;
						PX4_INFO("ESC%d: current: %f", motor_index, (double)esc.current);
					} else {
						// If no update is available, use the previous value
						esc.current = _esc_status.esc[motor_index].esc_current;
					}
				}

			} else {
				_bdshot_telem_online_mask &= ~(1 << motor_index);
				perf_count(_bdshot_error_perf);
			}

			consume_esc_data(esc, TelemetrySource::BDShot);
		}
	}

	perf_count(_bdshot_success_perf);

	return true;
}

void DShot::consume_esc_data(const EscData &esc, TelemetrySource source)
{
	bool bidirectional_enabled = _param_dshot_bidir_en.get();
	bool serial_telemetry_enabled = _param_dshot_tel_cfg.get();

	if (esc.motor_index < esc_status_s::CONNECTED_ESC_MAX) {

		uint8_t online_mask = 0xFF;

		// Require both sources online when enabled
		if (bidirectional_enabled) {
			online_mask &= _bdshot_telem_online_mask;
		}

		if (serial_telemetry_enabled) {
			online_mask &= _serial_telem_online_mask;
		}

		_esc_status.esc_online_flags = online_mask;

		_esc_status.esc[esc.motor_index].esc_errorcount = _serial_telem_errors[esc.motor_index] +
				_bdshot_telem_errors[esc.motor_index];

		if (source == TelemetrySource::Serial) {
			// Only use SerialTelemetry eRPM when BDShot is disabled
			if (!bidirectional_enabled) {
				_esc_status.esc[esc.motor_index].timestamp = esc.timestamp;
				_esc_status.esc[esc.motor_index].esc_rpm = esc.erpm / (_param_mot_pole_count.get() / 2);
			}

			_esc_status.esc[esc.motor_index].esc_voltage = esc.voltage;
			_esc_status.esc[esc.motor_index].esc_current = esc.current;
			_esc_status.esc[esc.motor_index].esc_temperature = esc.temperature;

		} else if (source == TelemetrySource::BDShot) {
			_esc_status.esc[esc.motor_index].timestamp = esc.timestamp;
			// TODO: is this conversion correct?
			_esc_status.esc[esc.motor_index].esc_rpm = esc.erpm / (_param_mot_pole_count.get() / 2);

			// Only use BDShot Volt/Curr/Temp when Serial Telemetry is disabled
			if (!serial_telemetry_enabled) {
				_esc_status.esc[esc.motor_index].esc_voltage = esc.voltage;
				_esc_status.esc[esc.motor_index].esc_current = esc.current;
				_esc_status.esc[esc.motor_index].esc_temperature = esc.temperature;
			}
		}
	}
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

void DShot::handle_vehicle_commands()
{
	vehicle_command_s command = {};

	while (_current_command.finished() && _vehicle_command_sub.update(&command)) {

		switch (command.command) {
		case vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR:
			handle_configure_actuator(command);
			break;

		case vehicle_command_s::VEHICLE_CMD_AM32_REQUEST_EEPROM:
			handle_am32_request_eeprom(command);
			break;

		default:
			break;
		}
	}
}

void DShot::handle_configure_actuator(const vehicle_command_s &command)
{
	int function = (int)(command.param5 + 0.5);

	PX4_INFO("Received VEHICLE_CMD_CONFIGURE_ACTUATOR");

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

	int motor_index = -1;

	for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; ++i) {
		if ((int)_mixing_output.outputFunction(i) == function && _mixing_output.isMotor(i)) {
			motor_index = (int)_mixing_output.outputFunction(i) - (int)OutputFunction::Motor1;
		}
	}

	vehicle_command_ack_s command_ack = {};
	command_ack.command = command.command;
	command_ack.target_system = command.source_system;
	command_ack.target_component = command.source_component;
	command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	if ((motor_index >= 0) || (motor_index < DSHOT_MAXIMUM_CHANNELS)) {
		int type = (int)(command.param1 + 0.5f);
		PX4_INFO("motor_index: %i type: %i", motor_index, type);
		_current_command.clear();
		_current_command.command = DSHOT_CMD_MOTOR_STOP;
		_current_command.num_repetitions = 10;
		_current_command.save = false;


		switch (type) {
		case ACTUATOR_CONFIGURATION_BEEP:
			_current_command.command = DSHOT_CMD_BEEP1;
			break;

		case ACTUATOR_CONFIGURATION_3D_MODE_OFF:
			_current_command.command = DSHOT_CMD_3D_MODE_OFF;
			_current_command.save = true;
			break;

		case ACTUATOR_CONFIGURATION_3D_MODE_ON:
			_current_command.command = DSHOT_CMD_3D_MODE_ON;
			_current_command.save = true;
			break;

		case ACTUATOR_CONFIGURATION_SPIN_DIRECTION1:
			_current_command.command = DSHOT_CMD_SPIN_DIRECTION_1;
			_current_command.save = true;
			break;

		case ACTUATOR_CONFIGURATION_SPIN_DIRECTION2:
			_current_command.command = DSHOT_CMD_SPIN_DIRECTION_2;
			_current_command.save = true;
			break;

		default:
			PX4_WARN("unknown command: %i", type);
			break;
		}

		if (_current_command.command != DSHOT_CMD_MOTOR_STOP) {
			command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			_current_command.motor_mask = 1 << motor_index;
		}
	}

	command_ack.timestamp = hrt_absolute_time();
	_command_ack_pub.publish(command_ack);
}

void DShot::handle_am32_request_eeprom(const vehicle_command_s &command)
{
	PX4_INFO("Received AM32_REQUEST_EEPROM");
	PX4_INFO("index: %d", (int)command.param1);

	int index = command.param1;

	// Use _settings_requested_mask to trigger re-requesting settings from motors we want

	if (index == 255) {
		_settings_requested_mask = 0;

	} else {
		_settings_requested_mask &= ~(1 << index);
	}

	vehicle_command_ack_s command_ack = {};
	command_ack.command = command.command;
	command_ack.target_system = command.source_system;
	command_ack.target_component = command.source_component;
	command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	command_ack.timestamp = hrt_absolute_time();
	_command_ack_pub.publish(command_ack);
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
		if (_mixing_output.reversibleOutputs() & (1 << i)) {
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

bool DShot::initialize_dshot()
{
	unsigned int dshot_frequency = 0;
	uint32_t dshot_frequency_param = 0;

	_output_mask = 0;

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
			_output_mask |= channels;

		} else if (tim_config == -4) {
			dshot_frequency_request = DSHOT300;
			_output_mask |= channels;

		} else if (tim_config == -3) {
			dshot_frequency_request = DSHOT600;
			_output_mask |= channels;
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

	int ret = up_dshot_init(_output_mask, dshot_frequency, _param_dshot_bidir_en.get(), _param_dshot_bidir_edt.get());

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

void DShot::init_telemetry(const char *device, bool swap_rxtx)
{
	if (!device) {
		return;
	}

	if (_telemetry.init(device, swap_rxtx) != PX4_OK) {
		PX4_ERR("telemetry init failed");
	}

	// Initialize ESC settings handlers based on ESC type
	ESCType esc_type = static_cast<ESCType>(_param_dshot_esc_type.get());
	_telemetry.initSettingsHandlers(esc_type, _output_mask);

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

	if (_param_dshot_tel_cfg.get()) {
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

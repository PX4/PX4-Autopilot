/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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
	delete _telemetry;
}

int DShot::init()
{
	_output_mask = (1u << _num_outputs) - 1;

	// Getting initial parameter values
	update_params();

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

void DShot::enable_dshot_outputs(const bool enabled)
{
	if (enabled && !_outputs_initialized && esc_flasher_flashing_state == 0) {
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

		// Get ESC_TYPE parameter at startup
		_esc_type = (ESCType)_param_esc_type.get();

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

		_outputs_initialized = true;
	}

	if (_outputs_initialized) {
		up_dshot_arm(enabled);
		_outputs_on = enabled;
	}
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

void DShot::init_telemetry(const char *device)
{
	if (!_telemetry) {
		_telemetry = new DShotTelemetry{};

		if (!_telemetry) {
			PX4_ERR("alloc failed");
			return;
		}
	}

	if (device != NULL) {
		int ret = _telemetry->init(device);

		if (ret != 0) {
			PX4_ERR("telemetry init failed (%i)", ret);
		}
	}

	update_num_motors();
}

int DShot::handle_new_telemetry_data(const int telemetry_index, const DShotTelemetry::EscData &data, bool ignore_rpm)
{
	int ret = 0;
	// fill in new motor data
	esc_status_s &esc_status = esc_status_pub.get();

	if (telemetry_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_status.esc_online_flags |= 1 << telemetry_index;

		esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];

		if (!ignore_rpm) {
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

		ret = 1; // Indicate we wrapped, so we publish data
#if 1
		// Flag get_esc_info state machine to start ESC detection
		if (get_esc_info_boot == 0 && get_esc_info_start == 0) {
			// Use get_esc_info_boot so we only start ESC detection at boot ONCE
			// It can be restarted via ESC_Flasher.cpp, using get_esc_info_start = 1
			get_esc_info_boot = 1;
			get_esc_info_start = 1;
			get_esc_info_time = hrt_absolute_time() + 3_s;
			// Set _esc_type_temp to the first enum, AM32
			if (_esc_type == ESCType::Unknown) {
				_esc_type_temp = ESCType::AM32;
			}
			else {
				_esc_type_temp = _esc_type;
			}
		}
#endif
	}

	_last_telemetry_index = telemetry_index;

	return ret;
}

void DShot::publish_esc_status(void)
{
	esc_status_s &esc_status = esc_status_pub.get();
	int telemetry_index = 0;
	int num_active_motors = _num_motors;

	// clear data of the esc that are offline
	for (int index = 0; (index < _last_telemetry_index); index++) {
		if ((esc_status.esc_online_flags & (1 << index)) == 0) {
			memset(&esc_status.esc[index], 0, sizeof(struct esc_report_s));
			num_active_motors--;
		}
	}

	if (num_active_motors != _num_motors && esc_flasher_flashing_state == 0) {
		_broken_esc++;
		// 50 times will take 2 seconds, since this runs at 25 Hz
		if (_broken_esc >= 50 && _broken_esc % 50 == 0) {
			//PX4_INFO("Broken ESC detected! Hardware fix required.");
			if (!_broken_esc_flag) {
				PX4_INFO("Broken ESC detected! Hardware fix required.");
				_broken_esc_flag = true;

				// Send esc_versions uOrb with result BROKEN
				esc_flasher_versions_s esc_flasher_versions{0};
				esc_flasher_versions.result = esc_flasher_versions_s::RESULT_BROKEN_ESC;
				// Flag versions_valid so the message gets sent out in PRE_SHOW_STATUS.hpp
				esc_flasher_versions.versions_valid = 1;
				esc_flasher_versions.timestamp = hrt_absolute_time();
				_esc_flasher_versions_pub.publish(esc_flasher_versions);
			}
		}
	}
	else {
		_broken_esc = 0;
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

	// If ESC flashing is in progress, mark all ESCs as offline so vehicle cannot arm
	if (esc_flasher_flashing_state > 0) {
		esc_status.esc_online_flags = 0;
		esc_status.esc_armed_flags = 0;
	}

	if (!esc_status_pub.advertised()) {
		esc_status_pub.advertise();

	} else {
		esc_status_pub.update();
	}

	// reset esc online flags
	esc_status.esc_online_flags = 0;
}

int DShot::handle_new_bdshot_erpm(void)
{
	int num_erpms = 0;
	int telemetry_index = 0;
	int erpm;
	esc_status_s &esc_status = esc_status_pub.get();

	esc_status.timestamp = hrt_absolute_time();
	esc_status.counter = _esc_status_counter++;
	esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
	esc_status.esc_armed_flags = _outputs_on;

	// We wait until all are ready.
	if (up_bdshot_num_erpm_ready() < _num_motors) {
		return 0;
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

	return num_erpms;
}

int DShot::send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index)
{
	Command cmd{};
	cmd.command = command;

	if (motor_index == -1) {
		cmd.motor_mask = 0xff;

	} else {
		cmd.motor_mask = 1 << motor_index;
	}

	ESCType temp_esc_type = _esc_type;
	if (get_esc_info_state != 0) {
		temp_esc_type = _esc_type_temp;
	}
	if (temp_esc_type == ESCType::AM32) {
		if (cmd.command == 9 || cmd.command == 10) cmd.save = true;
	}
	cmd.num_repetitions = num_repetitions;
	_new_command.store(&cmd);

	hrt_abstime timestamp_for_timeout = hrt_absolute_time();

	// wait until main thread processed it
	while (_new_command.load()) {

		if (hrt_elapsed_time(&timestamp_for_timeout) < 2_s) {
			px4_usleep(1000);

		} else {
			_new_command.store(nullptr);
			PX4_WARN("DShot command timeout!");
		}
	}

	return 0;
}

void DShot::retrieve_and_print_esc_info_thread_safe(const int motor_index)
{
	if (_request_esc_info.load() != nullptr) {
		// already in progress (not expected to ever happen)
		return;
	}

	DShotTelemetry::OutputBuffer output_buffer{};
	output_buffer.motor_index = motor_index;

	// start the request
	_request_esc_info.store(&output_buffer);

	// wait until processed
	int max_time = 1000;

	while (_request_esc_info.load() != nullptr && max_time-- > 0) {
		px4_usleep(1000);
	}

	_request_esc_info.store(nullptr); // just in case we time out...
	_waiting_for_esc_info = false;

	if (output_buffer.buf_pos == 0) {
		PX4_ERR("No data received. If telemetry is setup correctly, try again");
		return;
	}

	DShotTelemetry::decodeAndPrintEscInfoPacket(output_buffer);
}

int DShot::retrieve_and_print_esc_info_non_blocking(const int motor_index)
{
	if (_request_esc_info.load() != nullptr) {
		// already in progress (not expected to ever happen)
		return -1;
	}

	esc_flasher_output_buffer.buf_pos = 0;
	esc_flasher_output_buffer.motor_index = motor_index;

	// start the request
	_request_esc_info.store(&esc_flasher_output_buffer);

	return 0;
}

int DShot::retrieve_and_print_esc_info_check_result(uint8_t* fw_ver_major, uint8_t* fw_ver_minor)
{
	// Return 0 only if AM32 ESC
	// Return -1 if still waiting for data
	// Return -2 for timeout or CRC error or other error
	// Return -3 for BLHeli ESC

	if (_request_esc_info.load() != nullptr) {
		// Data not ready yet
		return -1;
	}

	_request_esc_info.store(nullptr); // just in case we time out...
	_waiting_for_esc_info = false;

	if (esc_flasher_output_buffer.buf_pos == 0) {
		PX4_ERR("No data received. If telemetry is setup correctly, try again");
		return -2;
	}

	int retval = DShotTelemetry::decodeEscInfoPacketFwVersion(esc_flasher_output_buffer, fw_ver_major, fw_ver_minor);
	if (retval == 0) {
		// AM32 ESC success
		return retval;
	}
	else if (retval == -1) {
		// No data or CRC error or other error
		return -2;
	}
	else if (retval == -2) {
		// BLHeli ESC success
		//PX4_ERR("AM32 ESC not found");
		return -3;
	}

	return retval;
}

int DShot::request_esc_info()
{
	_telemetry->redirectOutput(*_request_esc_info.load());
	_waiting_for_esc_info = true;

	int motor_index = _request_esc_info.load()->motor_index;

	_current_command.motor_mask = 1 << motor_index;

	ESCType temp_esc_type = _esc_type;
	if (get_esc_info_state != 0) {
		temp_esc_type = _esc_type_temp;
	}
	if (temp_esc_type == ESCType::AM32 || temp_esc_type == ESCType::AM32_Old) _current_command.num_repetitions = 6;
	else _current_command.num_repetitions = 1;

	_current_command.command = DShot_cmd_esc_info;
	_current_command.save = false;

	PX4_DEBUG("Requesting ESC info for motor %i", motor_index);
	return motor_index;
}

void DShot::mixerChanged()
{
	update_num_motors();

}

bool DShot::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	return false;
}

bool DShot::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_outputs_on) {
		return false;
	}

	int requested_telemetry_index = -1;

	if (_telemetry) {
		// check for an ESC info request. We only process it when we're not expecting other telemetry data
		if (_request_esc_info.load() != nullptr && !_waiting_for_esc_info && stop_motors
		    && !_telemetry->expectingData() && !_current_command.valid()) {
			requested_telemetry_index = request_esc_info();
		} else {
			requested_telemetry_index = _telemetry->getRequestMotorIndex();
		}
	}

	if (stop_motors) {

		int telemetry_index = 0;

		// when motors are stopped we check if we have other commands to send
		for (int i = 0; i < (int)num_outputs; i++) {
			if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
				// for some reason we need to always request telemetry when sending a command
				// unless command is esc_info
				ESCType temp_esc_type = _esc_type;
				if (get_esc_info_state != 0) {
					temp_esc_type = _esc_type_temp;
				}
				if ((temp_esc_type == ESCType::AM32 || temp_esc_type == ESCType::AM32_Old) && _current_command.command == 6) {
					up_dshot_motor_command(i, _current_command.command, false);
				}
				else up_dshot_motor_command(i, _current_command.command, true);
			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, telemetry_index == requested_telemetry_index);
			}

			telemetry_index += _mixing_output.isFunctionSet(i);
		}

		if (_current_command.valid()) {
			--_current_command.num_repetitions;

			if (_current_command.num_repetitions == 0 && _current_command.save) {
				_current_command.save = false;
				_current_command.num_repetitions = 10;
				_current_command.command = dshot_command_t::DShot_cmd_save_settings;
			}
		}

	} else {
		int telemetry_index = 0;

		for (int i = 0; i < (int)num_outputs; i++) {

			uint16_t output = outputs[i];

			if (output == DSHOT_DISARM_VALUE) {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, telemetry_index == requested_telemetry_index);

			} else {

				// DShot 3D splits the throttle ranges in two.
				// This is in terms of DShot values, code below is in terms of actuator_output
				// Direction 1) 48 is the slowest, 1047 is the fastest.
				// Direction 2) 1049 is the slowest, 2047 is the fastest.
				if (_param_dshot_3d_enable.get() || (_reversible_outputs & (1u << i))) {
					if (output >= _param_dshot_3d_dead_l.get() && output < _param_dshot_3d_dead_h.get()) {
						output = DSHOT_DISARM_VALUE;

					} else {
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

					}
				}

				up_dshot_motor_data_set(i, math::min(output, static_cast<uint16_t>(DSHOT_MAX_THROTTLE)),
							telemetry_index == requested_telemetry_index);
			}

			telemetry_index += _mixing_output.isFunctionSet(i);
		}

		// clear commands when motors are running
		_current_command.clear();
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

	_mixing_output.update();

	// update output status if armed or if mixer is loaded
	bool outputs_on = true;

	if (_outputs_on != outputs_on) {
		enable_dshot_outputs(outputs_on);
	}

	if (_telemetry) {
		const int telem_update = _telemetry->update(_num_motors);

		// Are we waiting for ESC info?
		if (_waiting_for_esc_info) {
			if (telem_update != -1) {
				_request_esc_info.store(nullptr);
				_waiting_for_esc_info = false;
			}

		} else if (telem_update >= 0) {
			const int need_to_publish = handle_new_telemetry_data(telem_update, _telemetry->latestESCData(),
						    _bidirectional_dshot_enabled);

			// We don't want to publish twice, once by telemetry and once by bidirectional dishot.
			if (!_bidirectional_dshot_enabled && need_to_publish) {
				publish_esc_status();
			}
		}
	}

	if (_bidirectional_dshot_enabled) {
		// Add bdshot data to esc status
		const int need_to_publish = handle_new_bdshot_erpm();

		if (need_to_publish) {
			publish_esc_status();
		}
	}

	if (_parameter_update_sub.updated()) {
		update_params();
	}

	// telemetry device update request?
	if (_request_telemetry_init.load()) {
		init_telemetry(_telemetry_device);
		_request_telemetry_init.store(false);
	}

	// new command?
	if (!_current_command.valid()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
		}
	}

	handle_vehicle_commands();

	handle_esc_flasher_requests();

	run_get_esc_info();

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
				PX4_DEBUG("setting command: index: %i type: %i", index, type);
				_current_command.command = dshot_command_t::DShot_cmd_motor_stop;

				switch (type) {
				case 1: _current_command.command = dshot_command_t::DShot_cmd_beacon1; break;

				case 2: _current_command.command = dshot_command_t::DShot_cmd_3d_mode_on; break;

				case 3: _current_command.command = dshot_command_t::DShot_cmd_3d_mode_off; break;

				case 4: _current_command.command = dshot_command_t::DShot_cmd_spin_direction_1; break;

				case 5: _current_command.command = dshot_command_t::DShot_cmd_spin_direction_2; break;
				}

				if (_current_command.command == dshot_command_t::DShot_cmd_motor_stop) {
					PX4_WARN("unknown command: %i", type);

				} else {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					_current_command.motor_mask = 1 << index;
					_current_command.num_repetitions = 10;
					_current_command.save = true;
				}

			}

			command_ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(command_ack);
		}
	}
}

void DShot::handle_esc_flasher_requests()
{
	static uint8_t fw_version_major;
	static uint8_t fw_version_minor;
	static uint32_t gpio;
	int retval = 0;

	while (!_current_command.valid() && _esc_flasher_request_sub.update(&_esc_flasher_request)) {
		PX4_INFO("DShot received request from ESC Flasher");

		if (_esc_flasher_request.request == esc_flasher_request_s::REQUEST_CANCEL ||
			_esc_flasher_request.request == esc_flasher_request_s::REQUEST_FLASHING_COMPLETE) {
			// Cancel any requests, re-enable DShot
			esc_flasher_esc_info_state = 0;
			esc_flasher_esc_info_motor_index = 0;
			esc_flasher_flashing_state = 0;
			enable_dshot_outputs(true);

			memset(&_esc_flasher_response, 0, sizeof(_esc_flasher_response));
			_esc_flasher_response.msg_id = _esc_flasher_request.msg_id;
			_esc_flasher_response.request = _esc_flasher_request.request;
			_esc_flasher_response.result = esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_ACCEPTED;
			_esc_flasher_response.timestamp = hrt_absolute_time();

			if (_esc_flasher_request.request == esc_flasher_request_s::REQUEST_CANCEL) {
				PX4_WARN("Canceling ESC Flasher requests and re-enabling DShot");
			}
			else {
				PX4_INFO("Flashing completed by ESC Flasher, re-enabling DShot");
				PX4_INFO("Restarting ESC detection to update ESC versions");

				// Restart ESC detection, so new version is saved and send via esc_versions uOrb
				get_esc_info_start = 1;
				get_esc_info_time = hrt_absolute_time() + 2_s;
				_esc_type_temp = _esc_type;
			}
			_esc_flasher_request_ack_pub.publish(_esc_flasher_response);

			// Clear broken esc flags on CANCEL or COMPLETE
			_broken_esc = 0;
			_broken_esc_flag = false;

			break;
		}

		if (esc_flasher_esc_info_state) {
			// ESC_INFO request is in progress
			memset(&_esc_flasher_response, 0, sizeof(_esc_flasher_response));
			_esc_flasher_response.msg_id = _esc_flasher_request.msg_id;
			_esc_flasher_response.request = _esc_flasher_request.request;
			_esc_flasher_response.result = esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_IN_PROGRESS;
			_esc_flasher_response.timestamp = hrt_absolute_time();

			PX4_ERR("ESC INFO request already in progress");
			_esc_flasher_request_ack_pub.publish(_esc_flasher_response);

			break;
		}
		if (esc_flasher_flashing_state) {
			// FLASHING request is in progress
			memset(&_esc_flasher_response, 0, sizeof(_esc_flasher_response));
			_esc_flasher_response.msg_id = _esc_flasher_request.msg_id;
			_esc_flasher_response.request = _esc_flasher_request.request;
			_esc_flasher_response.result = esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_IN_PROGRESS;
			_esc_flasher_response.timestamp = hrt_absolute_time();

			PX4_ERR("FLASHING request already in progress");
			_esc_flasher_request_ack_pub.publish(_esc_flasher_response);

			break;
		}

		if (_esc_flasher_request.request == esc_flasher_request_s::REQUEST_ESC_INFO) {
			if (telemetry_enabled()) {
				esc_flasher_esc_info_state = 1;
				esc_flasher_esc_info_motor_index = 0;
				memset(&_esc_flasher_response, 0, sizeof(_esc_flasher_response));
				_esc_flasher_response.msg_id = _esc_flasher_request.msg_id;
				_esc_flasher_response.request = _esc_flasher_request.request;
			}
			else {
				PX4_ERR("Telemetry is not enabled, but required to get ESC info");
			}
		}
		else if (_esc_flasher_request.request == esc_flasher_request_s::REQUEST_FLASHING) {
			// When ESC Flasher requests flashing, disable all DShot outputs until
			// ESC Flasher notifies completion or cancellation
			// Return via uORB all GPIO pin definitions, since ESC Flasher will need those for bit-banging UART
			esc_flasher_flashing_state = 1;
			memset(&_esc_flasher_response, 0, sizeof(_esc_flasher_response));
			_esc_flasher_response.msg_id = _esc_flasher_request.msg_id;
			_esc_flasher_response.request = _esc_flasher_request.request;
		}
	}

	switch (esc_flasher_esc_info_state) {
		case 0:
			break;
		case 1:
			// Get ESC_INFO from the requested motors
			if (_esc_flasher_request.motor_flags & (1 << esc_flasher_esc_info_motor_index)) {
				retrieve_and_print_esc_info_non_blocking(esc_flasher_esc_info_motor_index);
				esc_flasher_esc_info_state = 2;
			}
			else {
				// Move on to next motor
				esc_flasher_esc_info_motor_index++;
				if (esc_flasher_esc_info_motor_index >= 4) {
					esc_flasher_esc_info_state = 3;
				}
			}

			break;
		case 2:
			// Wait for ESC_INFO to finish
			retval = retrieve_and_print_esc_info_check_result(&fw_version_major, &fw_version_minor);
			if (retval == 0) {

				// Save the versions to our response struct
				_esc_flasher_response.fw_flags |= (1 << esc_flasher_esc_info_motor_index);
				_esc_flasher_response.fw_major[esc_flasher_esc_info_motor_index] = fw_version_major;
				_esc_flasher_response.fw_minor[esc_flasher_esc_info_motor_index] = fw_version_minor;

				// Move on to next motor
				esc_flasher_esc_info_motor_index++;
				if (esc_flasher_esc_info_motor_index >= 4) {
					esc_flasher_esc_info_state = 3;
				}
				else {
					esc_flasher_esc_info_state = 1;
				}
			}
			else if (retval == -1) {
				// Still waiting for response or timeout
			}
			else if (retval == -2) {
				// No valid response from ESC
				// Move on to next motor
				esc_flasher_esc_info_motor_index++;
				if (esc_flasher_esc_info_motor_index >= 4) {
					esc_flasher_esc_info_state = 3;
				}
				else {
					esc_flasher_esc_info_state = 1;
				}
			}
			else if (retval == -3) {
				// Non-AM32 ESC detected
				// Publish response message
				_esc_flasher_response.fw_flags = 0;
				_esc_flasher_response.timestamp = hrt_absolute_time();
				_esc_flasher_response.result = esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_UNSUPPORTED;

				_esc_flasher_request_ack_pub.publish(_esc_flasher_response);

				esc_flasher_esc_info_state = 0;
				esc_flasher_esc_info_motor_index = 0;
			}

			break;
		case 3:
			// Publish response message
			_esc_flasher_response.timestamp = hrt_absolute_time();
			_esc_flasher_response.result = esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_ACCEPTED;

			_esc_flasher_request_ack_pub.publish(_esc_flasher_response);

			esc_flasher_esc_info_state = 0;
			esc_flasher_esc_info_motor_index = 0;

			break;
	}

	switch (esc_flasher_flashing_state) {
		case 0:
			break;
		case 1:
			// Disable all DShot outputs
			enable_dshot_outputs(false);
			_outputs_initialized = false;
			esc_flasher_flashing_state = 2;

			break;
		case 2:
			// Send GPIO pins to ESC Flasher
			for (uint32_t i = 0; i < 16; i++) {
				if (_esc_flasher_request.motor_flags & (1 << i)) {
					gpio = io_timer_channel_get_gpio_output(i);
					// Clear unwanted flags from GPIO pin definition
					gpio &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
					_esc_flasher_response.gpio_pins[i] = gpio;
					_esc_flasher_response.gpio_flags |= (1 << i);
				}
			}

			esc_flasher_flashing_state = 3;

			break;
		case 3:
			// Publish response message
			_esc_flasher_response.timestamp = hrt_absolute_time();
			_esc_flasher_response.result = esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_ACCEPTED;

			_esc_flasher_request_ack_pub.publish(_esc_flasher_response);

			esc_flasher_flashing_state = 4;

			break;
		case 4:
			// Wait state for ESC Flasher, ensure DShot outputs stay off

			break;
	}

	// Update ESC status if flashing is active, so the vehicle cannot arm
	if (esc_flasher_flashing_state > 0) {
		publish_esc_status();
	}
}

void DShot::run_get_esc_info()
{
	int retval = 0;

	if (_num_motors && get_esc_info_start && (esc_flasher_flashing_state == 0) && (esc_flasher_esc_info_state == 0)) {
		// Once we have _num_motors, mixers have been initialized, try to get esc_info from all motors

		if (get_esc_info_state == 0 && (hrt_absolute_time() > get_esc_info_time)) {
			// Start state machine
			PX4_INFO("Starting ESC detection, attempting ESC type %d", (int)_esc_type_temp);
			get_esc_info_state = 1;
			get_esc_info_motor_index = 0;

			memset(&_esc_info_save, 0, sizeof(_esc_info_save));

			// This ESC_TYPE test is setup to run shortly after bootup
			// It will start out at the param ESC_TYPE and move up the list until the end, no loop
			// If DShot gets a response to esc_info command, then it is a success and the new ESC_TYPE is saved in the param
			// If no response then no change to ESC_TYPE param
			// To restart the test, reset the param ESC_TYPE to 0 and reboot PX4
		}

		switch (get_esc_info_state) {
			case 0:
				// Idle state
				break;
			case 1:
				// Check motor_index against _num_motors
				if (get_esc_info_motor_index < (uint32_t)_num_motors) {
					get_esc_info_state = 2;
				}
				else {
					// Done, count how many motors gave the esc_info
					int motor_count = 0;
					for (int i = 0; i < _num_motors; i++) {
						if (_esc_info_save[i].type != ESCType::Unknown) {
							motor_count++;
						}
					}

					if (motor_count == _num_motors) {
						get_esc_info_state = 10;
						PX4_INFO("ESC_INFO received for %d out of %d motors", (int)motor_count, (int)_num_motors);
					}
					else if (motor_count == 0) {

						// Move to next ESC_TYPE during the bootup test
						int temp_type = (int)_esc_type_temp;
						temp_type++;
						_esc_type_temp = (ESCType)temp_type;

						if (_esc_type_temp == ESCType::BlueJay) {
							// End of test, fail
							get_esc_info_state = 11;
							PX4_ERR("No ESC_INFO received, please set param ESC_TYPE to 0 and reboot");
							break;
						}

						// No data received, try again in 1 second
						if (get_esc_info_tries++ < 3) {
							get_esc_info_time = hrt_absolute_time() + 1_s;
							get_esc_info_state = 0;
							PX4_WARN("No ESC_INFO received, trying again in 1 second with ESC type %d", (int)_esc_type_temp);
						}
						else {
							// After 3 tries, no data received, end state-machine
							get_esc_info_state = 11;
							PX4_ERR("No ESC_INFO received");
						}
					}
					else {
						// Only received data from some motors
						get_esc_info_state = 10;
						PX4_INFO("ESC_INFO received for %d out of %d motors", (int)motor_count, (int)_num_motors);
					}
				}
				break;
			case 2:
				// Get next motor esc_info
				retrieve_and_print_esc_info_non_blocking(get_esc_info_motor_index);
				get_esc_info_state = 3;
				break;
			case 3:
				// Wait for ESC_INFO to finish
				retval = retrieve_and_print_esc_info_check_result(&_esc_info_save[get_esc_info_motor_index].fw_major, &_esc_info_save[get_esc_info_motor_index].fw_minor);
				if (retval == 0) {
					// AM32 ESC detected
					_esc_info_save[get_esc_info_motor_index].type = ESCType::AM32;
					get_esc_info_motor_index++;
					get_esc_info_state = 1;
					break;
				}
				else if (retval == -1) {
					// Still waiting for response or timeout
				}
				else if (retval == -2) {
					// No valid response from ESC
					// Move on to next motor
					get_esc_info_motor_index++;
					get_esc_info_state = 1;
					break;
				}
				else if (retval == -3) {
					// Non-AM32 ESC detected
					_esc_info_save[get_esc_info_motor_index].type = ESCType::BLHELI32;
					get_esc_info_motor_index++;
					get_esc_info_state = 1;
					break;
				}

				break;
			case 10:
			{
				// Done with all ESCs, publish uOrb message
				esc_flasher_versions_s esc_flasher_versions{0};
				int motor_count = 0;
				bool same_type = true;
				bool same_versions = true;
				ESCType type = ESCType::Unknown;
				uint8_t version_major = 0;
				uint8_t version_minor = 0;

				for (int i = 0; i < _num_motors; i++) {
					if (_esc_info_save[i].type != ESCType::Unknown) {
						if (i == 0) {
							// First motor, set type
							type = _esc_info_save[i].type;
							version_major = _esc_info_save[i].fw_major;
							version_minor = _esc_info_save[i].fw_minor;
						}
						else {
							// Subsequent motors, check type and versions against first motor
							if (_esc_info_save[i].type != type) {
								same_type = false;
							}
							if (_esc_info_save[i].fw_major != version_major || _esc_info_save[i].fw_minor != version_minor) {
								same_versions = false;
							}
						}

						motor_count++;
						esc_flasher_versions.versions_valid |= (1 << i);
						esc_flasher_versions.esc_type[i] = (uint8_t)_esc_info_save[i].type;
						esc_flasher_versions.versions_major[i] = _esc_info_save[i].fw_major;
						esc_flasher_versions.versions_minor[i] = _esc_info_save[i].fw_minor;
					}
				}

				esc_flasher_versions.result = esc_flasher_versions_s::RESULT_SUCCESS;
				if (motor_count != _num_motors) {
					esc_flasher_versions.result = esc_flasher_versions_s::RESULT_MISMATCH;
				}
				if (!same_type) {
					esc_flasher_versions.result = esc_flasher_versions_s::RESULT_MISMATCH;
				}
				if (!same_versions) {
					esc_flasher_versions.result = esc_flasher_versions_s::RESULT_MISMATCH;
				}

				if (same_type) {
					// If all types agree, set param
					_param_esc_type.set((int32_t)type);
					_param_esc_type.commit_no_notification();
					PX4_INFO("ESC Type detected: %s, saving to param ESC_TYPE", esc_types_strings[(uint32_t)type]);
				}

				// Compare AM32 to our PX4-internal flashable AM32 version
				esc_flasher_status_s esc_flasher_status;
				if (_esc_flasher_status_sub.copy(&esc_flasher_status)) {
					if ((esc_flasher_versions.result == esc_flasher_versions_s::RESULT_SUCCESS) &&
						(type == ESCType::AM32 || type == ESCType::AM32_Old)) {

#if 0
						// Debug testing hack
						// REMOVE when done
						// First bootup, give false version so it thinks it needs to update
						if (get_esc_info_debug == 0) {
							esc_versions.versions_minor[0]--;
							get_esc_info_debug = 1;
						}
						// REMOVE WHEN DONE
						// REMOVE WHEN DONE
						//
#endif

						if (esc_flasher_versions.versions_major[0] != esc_flasher_status.version_major ||
							esc_flasher_versions.versions_minor[0] != esc_flasher_status.version_minor) {
							// The fw version of AM32 detected on the ESCs does not match our internal flashable version
							esc_flasher_versions.result = esc_flasher_versions_s::RESULT_NEEDS_UPDATE;
						}
					}
				}

				esc_flasher_versions.timestamp = hrt_absolute_time();

				if (!_broken_esc_flag) {
					_esc_flasher_versions_pub.publish(esc_flasher_versions);
				}

				get_esc_info_state = 11;
			}
				break;
			case 11:
				// Done, return to idle state
				get_esc_info_state = 0;
				get_esc_info_start = 0;

				break;
		}
	}
}

void DShot::update_params()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	// we use a minimum value of 1, since 0 is for disarmed
	_mixing_output.setAllMinValues(math::constrain(static_cast<int>((_param_dshot_min.get() *
				       static_cast<float>(DSHOT_MAX_THROTTLE))),
				       DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE));

	// Do not use the minimum parameter for reversible outputs
	for (unsigned i = 0; i < _num_outputs; ++i) {
		if ((1 << i) & _reversible_outputs) {
			_mixing_output.minValue(i) = DSHOT_MIN_THROTTLE;
		}
	}

	_esc_type = (ESCType)_param_esc_type.get();
}

int DShot::custom_command(int argc, char *argv[])
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

	int motor_index = -1; // select motor index, default: -1=all
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor_index = strtol(myoptarg, nullptr, 10) - 1;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	struct VerbCommand {
		const char *name;
		dshot_command_t command;
		int num_repetitions;
	};

	constexpr VerbCommand commands[] = {
		{"reverse", DShot_cmd_spin_direction_2, 10},
		{"normal", DShot_cmd_spin_direction_1, 10},
		{"save", DShot_cmd_save_settings, 10},
		{"3d_on", DShot_cmd_3d_mode_on, 10},
		{"3d_off", DShot_cmd_3d_mode_off, 10},
		{"beep1", DShot_cmd_beacon1, 1},
		{"beep2", DShot_cmd_beacon2, 1},
		{"beep3", DShot_cmd_beacon3, 1},
		{"beep4", DShot_cmd_beacon4, 1},
		{"beep5", DShot_cmd_beacon5, 1},
	};

	for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		if (!strcmp(verb, commands[i].name)) {
			if (!is_running()) {
				PX4_ERR("module not running");
				return -1;
			}

			return get_instance()->send_command_thread_safe(commands[i].command, commands[i].num_repetitions, motor_index);
		}
	}

	if (!strcmp(verb, "esc_info")) {
		if (!is_running()) {
			PX4_ERR("module not running");
			return -1;
		}

		if (motor_index == -1) {
			PX4_ERR("No motor index specified");
			return -1;
		}

		if (!get_instance()->telemetry_enabled()) {
			PX4_ERR("Telemetry is not enabled, but required to get ESC info");
			return -1;
		}

		get_instance()->retrieve_and_print_esc_info_thread_safe(motor_index);
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
	PX4_INFO("Outputs initialized: %s", _outputs_initialized ? "yes" : "no");
	PX4_INFO("Outputs used: 0x%" PRIx32, _output_mask);
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	perf_print_counter(_cycle_perf);
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
This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement
to use DShot as ESC communication protocol instead of PWM.

On startup, the module tries to occupy all available pins for DShot output.
It skips all pins already in use (e.g. by a camera trigger module).

It supports:
- DShot150, DShot300, DShot600
- telemetry via separate UART and publishing as esc_status message
- sending DShot commands via CLI

### Examples
Permanently reverse motor 1:
$ dshot reverse -m 1
$ dshot save -m 1
After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);

	// DShot commands
	PRINT_MODULE_USAGE_COMMAND_DESCR("reverse", "Reverse motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("normal", "Normal motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save current settings");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_on", "Enable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_off", "Disable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep1", "Send Beep pattern 1");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep2", "Send Beep pattern 2");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep3", "Send Beep pattern 3");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep4", "Send Beep pattern 4");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep5", "Send Beep pattern 5");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_info", "Request ESC information");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based)", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dshot_main(int argc, char *argv[])
{
	return DShot::main(argc, argv);
}

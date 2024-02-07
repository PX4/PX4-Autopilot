/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
#include "vertiq_client_manager.hpp"

VertiqClientManager::VertiqClientManager(VertiqSerialInterface *serial_interface) :
	_serial_interface(serial_interface),
	_broadcast_prop_motor_control(_kBroadcastID), //Initialize with a module ID of 63 for broadcasting
	_broadcast_arming_handler(_kBroadcastID)

{
	_client_array[0] = &_broadcast_prop_motor_control;
	_client_array[1] = &_broadcast_arming_handler;
}

void VertiqClientManager::Init(uint8_t object_id)
{
	//If we're using the IFCI configuration parameters, we need a way to talk to the motor. These clients
	//give us access to all of the motor parameters we'll need. The Vertiq C++ library does not have a way of dynamically
	//changing a client's object ID, and we cannot instantiate the VertiqClientManager after the serial configuration is complete.
	//Therefore, we must make these statically.
	static EscPropellerInputParserClient prop_input_parser = EscPropellerInputParserClient(object_id);
	_prop_input_parser_client = &prop_input_parser;
	_client_array[_clients_in_use] = _prop_input_parser_client;
	_clients_in_use++;

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	static IQUartFlightControllerInterfaceClient ifci = IQUartFlightControllerInterfaceClient(object_id);
	_ifci_client = &ifci;
	_client_array[_clients_in_use] = _ifci_client;
	_clients_in_use++;
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	static VoltageSuperPositionClient voltage_superposition_client = VoltageSuperPositionClient(object_id);
	_voltage_superposition_client = &voltage_superposition_client;
	_client_array[_clients_in_use] = _voltage_superposition_client;
	_clients_in_use++;

	static PulsingRectangularInputParserClient pulsing_rectangular_input_parser_client =
		PulsingRectangularInputParserClient(object_id);
	_pulsing_rectangular_input_parser_client = &pulsing_rectangular_input_parser_client;
	_client_array[_clients_in_use] = _pulsing_rectangular_input_parser_client;
	_clients_in_use++;
#endif //CONFIG_USE_PULSING_CONFIGURATION

	//We're done with determining how many clients we have, let the serial interface know
	_serial_interface->SetNumberOfClients(_clients_in_use);
}

void VertiqClientManager::HandleClientCommunication()
{
	//Called periodically in the main loop to handle all communications not handled direclty by
	//parameter setting
	//Update our serial tx before we take in the RX
	_serial_interface->process_serial_tx();

	//Update our serial rx
	_serial_interface->process_serial_rx(&_motor_interface, _client_array);
}

IFCI *VertiqClientManager::GetMotorInterface()
{
	return &_motor_interface;
}

uint8_t VertiqClientManager::GetNumberOfClients()
{
	return _clients_in_use;
}

void VertiqClientManager::SendSetForceArm()
{
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 1);
}

void VertiqClientManager::SendSetForceDisarm()
{
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 0);
}

void VertiqClientManager::SendSetCoast()
{
	_broadcast_prop_motor_control.ctrl_coast_.set(*_serial_interface->get_iquart_interface());
}

void VertiqClientManager::SendSetVelocitySetpoint(uint16_t velocity_setpoint)
{
	_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface->get_iquart_interface(), velocity_setpoint);
}

void VertiqClientManager::InitParameter(param_t parameter, bool *init_bool, char descriptor, EntryData *value)
{
	//We've got a union, grab out the data from both parts of it. We'll grab the correct one later
	float float_value = value->float_data;
	uint32_t uint_value = value->uint_data;

	//If we have a float we want to grab the float value, if we have a uint, grab that.
	//In either case, put down the init flag of whoever called us
	switch (descriptor) {
	case 'f':
		param_set(parameter, &(float_value));
		*init_bool = false;
		break;

	case 'b':
		param_set(parameter, &(uint_value));
		*init_bool = false;
		break;

	default:
		return; //Don't go any further
		break;
	}
}

void VertiqClientManager::SendSetAndSave(ClientEntryAbstract *entry, char descriptor, EntryData *value)
{
	//Depending on the type of client we actually have, we're going to have to cast entry differently. We
	//let the descriptor tell us what to cast it to.

	//Note that we have to use the brackets to make sure that the ClientEntry objects have a scope
	switch (descriptor) {
	case 'f': {
			ClientEntry<float> *float_entry = (ClientEntry<float> *)(entry);
			float_entry->set(*_serial_interface->get_iquart_interface(), value->float_data);
			float_entry->save(*_serial_interface->get_iquart_interface());
			break;
		}

	case 'b': {
			ClientEntry<uint8_t> *byte_entry = (ClientEntry<uint8_t> *)(entry);
			byte_entry->set(*_serial_interface->get_iquart_interface(), value->uint_data);
			byte_entry->save(*_serial_interface->get_iquart_interface());
			break;
		}

	default:
		return; //Don't go any further
		break;
	}

	//Make sure our message gets out
	_serial_interface->process_serial_tx();
}

bool VertiqClientManager::FloatsAreClose(float val1, float val2, float tolerance)
{
	float diff = val1 - val2;
	return (abs(diff) < tolerance);
}

void VertiqClientManager::UpdateParameter(param_t parameter, bool *init_bool, char descriptor, EntryData *value,
		ClientEntryAbstract *entry)
{
	//Note that we have to use the brackets to make sure that the ClientEntry objects have a scope
	switch (descriptor) {
	case 'f': {
			//go ahead and make the vars we'll need and cast the entry
			float module_float_value = 0;
			float px4_float_value = 0;
			ClientEntry<float> *float_entry = (ClientEntry<float> *)(entry);

			//If we got an answer go grab it. Then, if we're initializing, give the value to PX4. Otherwise, someone just
			//set a new value to the PX4 param, so send it over to the motor if it's actually different
			if (float_entry->IsFresh()) {
				module_float_value = float_entry->get_reply();

				if (*init_bool) {
					value->float_data = module_float_value;
					InitParameter(parameter, init_bool, descriptor, value);

				} else {
					param_get(parameter, &px4_float_value);

					if (!FloatsAreClose(px4_float_value, module_float_value)) {
						value->float_data = px4_float_value;
						SendSetAndSave(float_entry, descriptor, value);
					}
				}
			}

			break;
		}

	case 'b': {
			//go ahead and make the vars we'll need and cast the entry
			uint32_t module_read_value = 0;
			int32_t px4_read_value = 0;
			ClientEntry<uint8_t> *byte_entry = (ClientEntry<uint8_t> *)(entry);

			//If we got an answer go grab it. Then, if we're initializing, give the value to PX4. Otherwise, someone just
			//set a new value to the PX4 param, so send it over to the motor if it's actually different
			if (byte_entry->IsFresh()) {
				module_read_value = byte_entry->get_reply();

				if (*init_bool) {
					value->uint_data = module_read_value;
					InitParameter(parameter, init_bool, descriptor, value);

				} else {
					param_get(parameter, &px4_read_value);

					if ((uint32_t)px4_read_value != module_read_value) {
						value->uint_data = (uint32_t)px4_read_value;
						SendSetAndSave(byte_entry, descriptor, value);
					}
				}
			}

			break;
		}

	default:
		return; //Don't go any further
		break;
	}
}

void VertiqClientManager::MarkIfciConfigsForRefresh()
{
	_init_velocity_max = true;
	_init_volts_max = true;
	_init_mode = true;
	_init_motor_dir = true;
	_init_fc_dir = true;

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	_init_throttle_cvi = true;
#endif

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	_init_pulse_volt_mode = true;
	_init_pulse_x_cvi = true;
	_init_pulse_y_cvi = true;
	_init_pulse_zero_angle = true;
	_init_pulse_velo_cutoff = true;
	_init_pulse_torque_offset = true;
	_init_pulse_volt_limit = true;
#endif
}

void VertiqClientManager::UpdateIfciConfigParams()
{
	_prop_input_parser_client->velocity_max_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->volts_max_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->mode_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->sign_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->flip_negative_.get(*_serial_interface->get_iquart_interface());

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	_ifci_client->throttle_cvi_.get(*_serial_interface->get_iquart_interface());
#endif

	//Ensure that these get messages get out
	_serial_interface->process_serial_tx();

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	_voltage_superposition_client->zero_angle_.get(*_serial_interface->get_iquart_interface());
	_voltage_superposition_client->propeller_torque_offset_angle_.get(*_serial_interface->get_iquart_interface());
	_voltage_superposition_client->velocity_cutoff_.get(*_serial_interface->get_iquart_interface());
	_pulsing_rectangular_input_parser_client->pulsing_voltage_mode_.get(*_serial_interface->get_iquart_interface());
	_pulsing_rectangular_input_parser_client->pulsing_voltage_limit_.get(*_serial_interface->get_iquart_interface());
	_ifci_client->x_cvi_.get(*_serial_interface->get_iquart_interface());
	_ifci_client->y_cvi_.get(*_serial_interface->get_iquart_interface());

	//Ensure that these get messages get out
	_serial_interface->process_serial_tx();
#endif //CONFIG_USE_PULSING_CONFIGURATION

	//Now go ahead and grab responses, and update everyone to be on the same page, but do it quickly.
	CoordinateIquartWithPx4Params(100_ms);
}

void VertiqClientManager::CoordinateIquartWithPx4Params(hrt_abstime timeout)
{
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	EntryData entry_values;

	while (time_now < end_time) {
		//Go ahead and update our IFCI params
		UpdateParameter(param_find("MAX_VELOCITY"), &_init_velocity_max, 'f', &entry_values,
				&(_prop_input_parser_client->velocity_max_));
		UpdateParameter(param_find("MAX_VOLTS"), &_init_volts_max, 'f', &entry_values,
				&(_prop_input_parser_client->volts_max_));
		UpdateParameter(param_find("CONTROL_MODE"), &_init_mode, 'b',  &entry_values, &(_prop_input_parser_client->mode_));
		UpdateParameter(param_find("VERTIQ_MOTOR_DIR"), &_init_motor_dir, 'b',  &entry_values,
				&(_prop_input_parser_client->sign_));
		UpdateParameter(param_find("VERTIQ_FC_DIR"), &_init_fc_dir, 'b',  &entry_values,
				&(_prop_input_parser_client->flip_negative_));

#ifdef CONFIG_USE_IFCI_CONFIGURATION
		UpdateParameter(param_find("THROTTLE_CVI"), &_init_throttle_cvi, 'b',  &entry_values, &(_ifci_client->throttle_cvi_));
#endif

#ifdef CONFIG_USE_PULSING_CONFIGURATION
		UpdateParameter(param_find("PULSE_VOLT_MODE"), &_init_pulse_volt_mode, 'b',  &entry_values,
				&(_pulsing_rectangular_input_parser_client->pulsing_voltage_mode_));
		UpdateParameter(param_find("X_CVI"), &_init_pulse_x_cvi, 'b',  &entry_values, &(_ifci_client->x_cvi_));
		UpdateParameter(param_find("Y_CVI"), &_init_pulse_y_cvi, 'b',  &entry_values, &(_ifci_client->y_cvi_));
		UpdateParameter(param_find("ZERO_ANGLE"), &_init_pulse_zero_angle, 'f',  &entry_values,
				&(_voltage_superposition_client->zero_angle_));
		UpdateParameter(param_find("VELOCITY_CUTOFF"), &_init_pulse_velo_cutoff, 'f', &entry_values,
				&(_voltage_superposition_client->velocity_cutoff_));
		UpdateParameter(param_find("TORQUE_OFF_ANGLE"), &_init_pulse_torque_offset, 'f', &entry_values,
				&(_voltage_superposition_client->propeller_torque_offset_angle_));
		UpdateParameter(param_find("PULSE_VOLT_LIM"), &_init_pulse_volt_limit, 'f', &entry_values,
				&(_pulsing_rectangular_input_parser_client->pulsing_voltage_limit_));
#endif //CONFIG_USE_PULSING_CONFIGURATION

		//Update the time
		time_now = hrt_absolute_time();

		//Update our serial rx
		_serial_interface->process_serial_rx(&_motor_interface, _client_array);
	}
}

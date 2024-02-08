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
	//If we're using the IQUART configuration parameters, we need a way to talk to the motor. These clients
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

	_velocity_max_entry.ConfigureStruct(param_find("MAX_VELOCITY"), &(_prop_input_parser_client->velocity_max_));
	_voltage_max_entry.ConfigureStruct(param_find("MAX_VOLTS"), &(_prop_input_parser_client->volts_max_));
	_control_mode_entry.ConfigureStruct(param_find("CONTROL_MODE"), &(_prop_input_parser_client->mode_));
	_motor_direction_entry.ConfigureStruct(param_find("VERTIQ_MOTOR_DIR"), &(_prop_input_parser_client->sign_));
	_fc_direction_entry.ConfigureStruct(param_find("VERTIQ_FC_DIR"), &(_prop_input_parser_client->flip_negative_));
}

void VertiqClientManager::HandleClientCommunication()
{
	//Called periodically in the main loop to handle all communications not handled direclty by
	//parameter setting
	//Update our serial tx before we take in the rx
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

template <class px4_data_type>
void VertiqClientManager::InitParameter(param_t parameter, bool *init_bool, px4_data_type *value)
{
	param_set(parameter, value);
	*init_bool = false;
}

template <class module_data_type>
void VertiqClientManager::SendSetAndSave(ClientEntry<module_data_type> *entry, module_data_type value)
{
	entry->set(*_serial_interface->get_iquart_interface(), value);
	entry->save(*_serial_interface->get_iquart_interface());
	_serial_interface->process_serial_tx();
}

template <class data_type>
bool ValuesAreTheSame(data_type val1, data_type val2, data_type tolerance)
{
	return (val1 == val2);
}

template <>
bool ValuesAreTheSame<float>(float val1, float val2, float tolerance)
{
	float diff = val1 - val2;
	return (abs(diff) < tolerance);
}

template <class module_data_type, class px4_data_type>
void VertiqClientManager::UpdateParameter(param_t parameter, bool *init_bool, ClientEntry<module_data_type> *entry)
{
	module_data_type module_value = 0;
	px4_data_type px4_value = 0;

	if (entry->IsFresh()) {
		module_value = entry->get_reply();

		//If we're initializing PX4 to the motor, set the PX4 value to the module value
		//If we're setting a value through PX4, set the module value to the PX4 value
		PX4_INFO("init bool value %d", *init_bool);

		if (*init_bool) {
			px4_value = (px4_data_type)module_value;
			InitParameter(parameter, init_bool, &px4_value);

		} else {
			param_get(parameter, &px4_value);

			if (!ValuesAreTheSame<px4_data_type>(px4_value, (px4_data_type)module_value, (px4_data_type)(0.01))) {
				SendSetAndSave<module_data_type>(entry, px4_value);
			}
		}
	}
}

template <class module_data_type, class px4_data_type>
void VertiqClientManager::UpdateParameter(combo_entry<module_data_type, px4_data_type> *combined_entry)
{
	module_data_type module_value = 0;
	px4_data_type px4_value = 0;

	if (combined_entry->_entry->IsFresh()) {
		module_value = combined_entry->_entry->get_reply();
		PX4_INFO("needs init value %d", combined_entry->_needs_init);

		if (combined_entry->_needs_init) {
			// PX4_INFO("combo param needed init");
			px4_value = (px4_data_type)module_value;
			InitParameter(combined_entry->_param, &(combined_entry->_needs_init), &px4_value);

		} else {
			// PX4_INFO("combo param did not need init");
			param_get(combined_entry->_param, &px4_value);

			if (!ValuesAreTheSame<px4_data_type>(px4_value, (px4_data_type)module_value, (px4_data_type)(0.01))) {
				SendSetAndSave<module_data_type>(combined_entry->_entry, px4_value);
			}
		}
	}
}

void VertiqClientManager::MarkIquartConfigsForRefresh()
{
	for (uint8_t i = 0; i < num_floats; i++) {
		float_combo_entries[i]->SetNeedsInit();
	}

	for (uint8_t i = 0; i < num_uints; i++) {
		uint8_combo_entries[i]->SetNeedsInit();
	}

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

void VertiqClientManager::UpdateIquartConfigParams()
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

	while (time_now < end_time) {
		//Go ahead and update our IFCI params
		UpdateParameter<float, float>(&_velocity_max_entry);
		UpdateParameter<float, float>(&_voltage_max_entry);
		UpdateParameter<uint8_t, int32_t>(&_control_mode_entry);
		UpdateParameter<uint8_t, int32_t>(&_motor_direction_entry);
		UpdateParameter<uint8_t, int32_t>(&_fc_direction_entry);

#ifdef CONFIG_USE_IFCI_CONFIGURATION
		UpdateParameter<uint8_t, int32_t>(param_find("THROTTLE_CVI"), &_init_throttle_cvi,
						  &(_ifci_client->throttle_cvi_));
#endif

#ifdef CONFIG_USE_PULSING_CONFIGURATION
		UpdateParameter<uint8_t, int32_t>(param_find("PULSE_VOLT_MODE"), &_init_pulse_volt_mode,
						  &(_pulsing_rectangular_input_parser_client->pulsing_voltage_mode_));
		UpdateParameter<uint8_t, int32_t>(param_find("X_CVI"), &_init_pulse_x_cvi, &(_ifci_client->x_cvi_));
		UpdateParameter<uint8_t, int32_t>(param_find("Y_CVI"), &_init_pulse_y_cvi, &(_ifci_client->y_cvi_));
		UpdateParameter<float, float>(param_find("ZERO_ANGLE"), &_init_pulse_zero_angle,
					      &(_voltage_superposition_client->zero_angle_));
		UpdateParameter<float, float>(param_find("VELOCITY_CUTOFF"), &_init_pulse_velo_cutoff,
					      &(_voltage_superposition_client->velocity_cutoff_));
		UpdateParameter<float, float>(param_find("TORQUE_OFF_ANGLE"), &_init_pulse_torque_offset,
					      &(_voltage_superposition_client->propeller_torque_offset_angle_));
		UpdateParameter<float, float>(param_find("PULSE_VOLT_LIM"), &_init_pulse_volt_limit,
					      &(_pulsing_rectangular_input_parser_client->pulsing_voltage_limit_));
#endif //CONFIG_USE_PULSING_CONFIGURATION

		//Update the time
		time_now = hrt_absolute_time();

		//Update our serial rx
		_serial_interface->process_serial_rx(&_motor_interface, _client_array);
	}
}

/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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
	AddNewOperationalClient(&_broadcast_prop_motor_control);
	AddNewOperationalClient(&_broadcast_arming_handler);
}

void VertiqClientManager::Init(uint8_t object_id)
{
	InitVertiqClients(object_id);
	InitEntryWrappers();

	_object_id_now = object_id;
}

void VertiqClientManager::InitVertiqClients(uint8_t object_id)
{
	_telem_ifci = new IQUartFlightControllerInterfaceClient(object_id);
	AddNewConfigurationClient(_telem_ifci);

	_prop_input_parser_client = new EscPropellerInputParserClient(object_id);
	AddNewConfigurationClient(_prop_input_parser_client);

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	_ifci_client = new IQUartFlightControllerInterfaceClient(object_id);
	AddNewConfigurationClient(_ifci_client);
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	_voltage_superposition_client = new VoltageSuperPositionClient(object_id);
	AddNewConfigurationClient(_voltage_superposition_client);

	_pulsing_rectangular_input_parser_client = new PulsingRectangularInputParserClient(object_id);
	AddNewConfigurationClient(_pulsing_rectangular_input_parser_client);
#endif //CONFIG_USE_PULSING_CONFIGURATION
}

void VertiqClientManager::InitEntryWrappers()
{
	_velocity_max_entry.ConfigureStruct(param_find("MAX_VELOCITY"), &(_prop_input_parser_client->velocity_max_));
	_voltage_max_entry.ConfigureStruct(param_find("MAX_VOLTS"), &(_prop_input_parser_client->volts_max_));
	_control_mode_entry.ConfigureStruct(param_find("CONTROL_MODE"), &(_prop_input_parser_client->mode_));
	_motor_direction_entry.ConfigureStruct(param_find("VERTIQ_MOTOR_DIR"), &(_prop_input_parser_client->sign_));
	_fc_direction_entry.ConfigureStruct(param_find("VERTIQ_FC_DIR"), &(_prop_input_parser_client->flip_negative_));

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	_throttle_cvi_entry.ConfigureStruct(param_find("THROTTLE_CVI"), &(_ifci_client->throttle_cvi_));
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	_pulsing_voltage_mode_entry.ConfigureStruct(param_find("PULSE_VOLT_MODE"),
			&(_pulsing_rectangular_input_parser_client->pulsing_voltage_mode_));
	_x_cvi_entry.ConfigureStruct(param_find("X_CVI"), &(_ifci_client->x_cvi_));
	_y_cvi_entry.ConfigureStruct(param_find("Y_CVI"), &(_ifci_client->y_cvi_));
	_pulse_zero_angle_entry.ConfigureStruct(param_find("ZERO_ANGLE"), &(_voltage_superposition_client->zero_angle_));
	_pulse_velo_cutoff_entry.ConfigureStruct(param_find("VELOCITY_CUTOFF"),
			&(_voltage_superposition_client->velocity_cutoff_));
	_pulse_torque_offset_entry.ConfigureStruct(param_find("TORQUE_OFF_ANGLE"),
			&(_voltage_superposition_client->propeller_torque_offset_angle_));
	_pulse_volt_limit_entry.ConfigureStruct(param_find("PULSE_VOLT_LIM"),
						&(_pulsing_rectangular_input_parser_client->pulsing_voltage_limit_));
#endif //CONFIG_USE_PULSING_CONFIGURATION
}

uint8_t VertiqClientManager::GetObjectIdNow()
{
	return _object_id_now;
}

IQUartFlightControllerInterfaceClient *VertiqClientManager::GetTelemIFCI(){
	return _telem_ifci;
}


void VertiqClientManager::UpdateClientsToNewObjId(uint8_t new_object_id)
{
	_object_id_now = new_object_id;

	delete _prop_input_parser_client;
	_prop_input_parser_client = new EscPropellerInputParserClient(new_object_id);

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	delete _ifci_client;
	_ifci_client = new IQUartFlightControllerInterfaceClient(new_object_id);
#endif

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	delete _voltage_superposition_client;
	_voltage_superposition_client = new VoltageSuperPositionClient(new_object_id);

	delete _pulsing_rectangular_input_parser_client;
	_pulsing_rectangular_input_parser_client = new PulsingRectangularInputParserClient(new_object_id);
#endif
}

void VertiqClientManager::HandleClientCommunication()
{
	//Called periodically in the main loop to handle all communications not handled direclty by
	//parameter setting
	//Update our serial tx before we take in the rx
	_serial_interface->process_serial_tx();

	//Update our serial rx
	_serial_interface->process_serial_rx(_configuration_client_array, _operational_client_array);
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

void VertiqClientManager::MarkIquartConfigsForRefresh()
{
	for (uint8_t i = 0; i < _added_entry_wrappers; i++) {
		_entry_wrappers[i]->SetNeedsInit();
	}
}

void VertiqClientManager::UpdateIquartConfigParams()
{
	for (uint8_t i = 0; i < _added_entry_wrappers; i++) {
		_entry_wrappers[i]->SendGet(_serial_interface);
		//Ensure that these get messages get out
		_serial_interface->process_serial_tx();
	}

	//Now go ahead and grab responses, and update everyone to be on the same page, but do it quickly.
	CoordinateIquartWithPx4Params(100_ms);
}

void VertiqClientManager::CoordinateIquartWithPx4Params(hrt_abstime timeout)
{
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	while (time_now < end_time) {
		for (uint8_t i = 0; i < _added_entry_wrappers; i++) {
			_entry_wrappers[i]->Update(_serial_interface);
		}

		//Update the time
		time_now = hrt_absolute_time();

		//Update our serial rx
		_serial_interface->process_serial_rx(_configuration_client_array, _operational_client_array);
	}
}

void VertiqClientManager::AddNewConfigurationClient(ClientAbstract * client){
	if(_configuration_clients_in_use < MAXIMUM_CONFIGURATION_CLIENTS){
		_configuration_client_array[_configuration_clients_in_use] = client;
		_configuration_clients_in_use++;

		_serial_interface->SetNumberOfConfigurationClients(_configuration_clients_in_use);
	}else{
		PX4_INFO("Could not add this client. Maximum number exceeded");
	}
}

void VertiqClientManager::AddNewOperationalClient(ClientAbstract * client){
	if(_operational_clients_in_use < MAXIMUM_OPERATIONAL_CLIENTS){
		_operational_client_array[_operational_clients_in_use] = client;
		_operational_clients_in_use++;

		_serial_interface->SetNumberOfOperationalClients(_operational_clients_in_use);
	}else{
		PX4_INFO("Could not add this client. Maximum number exceeded");
	}
}

uint8_t VertiqClientManager::GetNumberOfConfigurationClients(){
	return _configuration_clients_in_use;
}

uint8_t VertiqClientManager::GetNumberOfOperationalClients(){
	return _operational_clients_in_use;
}

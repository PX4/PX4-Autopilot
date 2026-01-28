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
#include "vertiq_configuration_handler.hpp"

VertiqConfigurationHandler::VertiqConfigurationHandler(VertiqSerialInterface *ser,
		VertiqClientManager *client_manager) :
	_serial_interface(ser),
	_client_manager(client_manager)
{
}

void VertiqConfigurationHandler::InitConfigurationClients(uint8_t object_id)
{
	_object_id_now = object_id; //Make sure we store the initial object ID

	_prop_input_parser_client = new EscPropellerInputParserClient(object_id);
	_client_manager->AddNewClient(_prop_input_parser_client);

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	_ifci_client = new IQUartFlightControllerInterfaceClient(object_id);
	_client_manager->AddNewClient(_ifci_client);
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	_voltage_superposition_client = new VoltageSuperPositionClient(object_id);
	_client_manager->AddNewClient(_voltage_superposition_client);

	_pulsing_rectangular_input_parser_client = new PulsingRectangularInputParserClient(object_id);
	_client_manager->AddNewClient(_pulsing_rectangular_input_parser_client);
#endif //CONFIG_USE_PULSING_CONFIGURATION
}

void VertiqConfigurationHandler::InitClientEntryWrappers()
{
	AddNewClientEntry<float, float>(param_find("VTQ_MAX_VELOCITY"), &(_prop_input_parser_client->velocity_max_));
	AddNewClientEntry<float, float>(param_find("VTQ_MAX_VOLTS"), &(_prop_input_parser_client->volts_max_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("VTQ_CONTROL_MODE"), &(_prop_input_parser_client->mode_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("VTQ_MOTOR_DIR"), &(_prop_input_parser_client->sign_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("VTQ_FC_DIR"), &(_prop_input_parser_client->flip_negative_));

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	AddNewClientEntry<uint8_t, int32_t>(param_find("VTQ_THROTTLE_CVI"), &(_ifci_client->throttle_cvi_));
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	AddNewClientEntry<uint8_t, int32_t> (param_find("VTQ_PULSE_V_MODE"),
					     &(_pulsing_rectangular_input_parser_client->pulsing_voltage_mode_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("VTQ_X_CVI"), &(_ifci_client->x_cvi_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("VTQ_Y_CVI"), &(_ifci_client->y_cvi_));
	AddNewClientEntry<float, float>(param_find("VTQ_ZERO_ANGLE"), &(_voltage_superposition_client->zero_angle_));
	AddNewClientEntry<float, float>(param_find("VTQ_VELO_CUTOFF"),
					&(_voltage_superposition_client->velocity_cutoff_));
	AddNewClientEntry<float, float>(param_find("VTQ_TQUE_OFF_ANG"),
					&(_voltage_superposition_client->propeller_torque_offset_angle_));
	AddNewClientEntry<float, float>(param_find("VTQ_PULSE_V_LIM"),
					&(_pulsing_rectangular_input_parser_client->pulsing_voltage_limit_));
#endif //CONFIG_USE_PULSING_CONFIGURATION
}

void VertiqConfigurationHandler::UpdateClientsToNewObjId(uint8_t new_object_id)
{
	_object_id_now = new_object_id;

	DestroyAndRecreateClient<EscPropellerInputParserClient>(_prop_input_parser_client, new_object_id);

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	DestroyAndRecreateClient<IQUartFlightControllerInterfaceClient>(_ifci_client, new_object_id);
#endif

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	DestroyAndRecreateClient<VoltageSuperPositionClient>(_voltage_superposition_client, new_object_id);
	DestroyAndRecreateClient<PulsingRectangularInputParserClient>(_pulsing_rectangular_input_parser_client, new_object_id);
#endif
}

void VertiqConfigurationHandler::MarkConfigurationEntriesForRefresh()
{
	for (uint8_t i = 0; i < _added_configuration_entry_wrappers; i++) {
		_configuration_entry_wrappers[i]->SetNeedsInit();
	}
}

void VertiqConfigurationHandler::UpdateIquartConfigParams()
{
	for (uint8_t i = 0; i < _added_configuration_entry_wrappers; i++) {
		_configuration_entry_wrappers[i]->SendGet(_serial_interface);
		//Ensure that these get messages get out
		_client_manager->HandleClientCommunication();
	}

	//Now go ahead and grab responses, and update everyone to be on the same page, but do it quickly.
	CoordinateIquartWithPx4Params();
}

void VertiqConfigurationHandler::CoordinateIquartWithPx4Params(hrt_abstime timeout)
{
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	while (time_now < end_time) {
		for (uint8_t i = 0; i < _added_configuration_entry_wrappers; i++) {
			_configuration_entry_wrappers[i]->Update(_serial_interface);
		}

		//Update the time
		time_now = hrt_absolute_time();

		//Update our serial rx
		_client_manager->HandleClientCommunication();
	}
}

uint8_t VertiqConfigurationHandler::GetObjectIdNow()
{
	return _object_id_now;
}

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

#include "vertiq_telemetry_manager.hpp"

VertiqTelemetryManager::VertiqTelemetryManager(VertiqClientManager *client_manager) :
	_client_manager(client_manager),
	_telem_state(UNPAUSED)
{
}

void VertiqTelemetryManager::Init(uint64_t telem_bitmask, uint8_t module_id)
{
	//On init, make sure to set our bitmask, and then go ahead and find the front and back 1s
	_telem_bitmask = telem_bitmask;
	FindTelemetryModuleIds();

	_telem_interface = new IQUartFlightControllerInterfaceClient(module_id);
	_client_manager->AddNewClient(_telem_interface);
}

void VertiqTelemetryManager::FindTelemetryModuleIds()
{
	uint64_t temp_bitmask = _telem_bitmask;

	//We have a uint64 with bit positions representing module ids. If you see a 1, that's a module ID we need to get telemetry from
	//Keep shifting values, and see who's a 1
	for (uint8_t i = 0; i < MAX_SUPPORTABLE_MODULE_IDS; i++) {
		//We're only going to keep track of up to MAX_ESC_STATUS_ENTRIES
		if (temp_bitmask & 0x0000000000000001 && (_number_of_module_ids_for_telem < MAX_ESC_STATUS_ENTRIES)) {
			//we found one, and have spcae for it. Put it in our array
			_module_ids_in_use[_number_of_module_ids_for_telem] = i;
			_number_of_module_ids_for_telem++;
		}

		temp_bitmask = temp_bitmask >> 1;
	}
}

void VertiqTelemetryManager::StartPublishing(uORB::Publication<esc_status_s> *esc_status_pub)
{
	//Initialize our ESC status publishing
	_esc_status.timestamp          = hrt_absolute_time();
	_esc_status.counter            = 0;
	_esc_status.esc_count          = _number_of_module_ids_for_telem;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;
	_esc_status.esc_armed_flags = 0;
	_esc_status.esc_online_flags = 0;

	for (unsigned i = 0; i < _number_of_module_ids_for_telem; i++) {
		_esc_status.esc[i].timestamp       = 0;
		_esc_status.esc[i].esc_address     = 0;
		_esc_status.esc[i].esc_rpm         = 0;
		_esc_status.esc[i].esc_state       = 0;
		_esc_status.esc[i].esc_cmdcount    = 0;
		_esc_status.esc[i].esc_voltage     = 0;
		_esc_status.esc[i].esc_current     = 0;
		_esc_status.esc[i].esc_temperature = 0;
		_esc_status.esc[i].esc_errorcount  = 0;
		_esc_status.esc[i].failures        = 0;
		_esc_status.esc[i].esc_power       = 0;
	}

	//Start advertising to the world
	esc_status_pub->advertise();
}

uint16_t VertiqTelemetryManager::UpdateTelemetry()
{
	bool got_reply = false;

	//Get the current time to check for timeout
	hrt_abstime time_now = hrt_absolute_time();

	//We timed out for this request if the time since the last request going out is greater than our timeout period
	bool timed_out = (time_now - _time_of_last_telem_request) > _telem_timeout;

	//We got a telemetry response
	if (_telem_interface->telemetry_.IsFresh()) {
		//grab the data
		IFCITelemetryData telem_response = _telem_interface->telemetry_.get_reply();

		// also update our internal report for logging
		_esc_status.esc[_current_module_id_target_index].esc_address  = _module_ids_in_use[_number_of_module_ids_for_telem];
		_esc_status.esc[_current_module_id_target_index].timestamp    = time_now;
		_esc_status.esc[_current_module_id_target_index].esc_rpm      = telem_response.speed * 60.0f * M_1_PI_F *
				0.5f; //We get back rad/s, convert to rpm
		_esc_status.esc[_current_module_id_target_index].esc_voltage  = telem_response.voltage * 0.01;
		_esc_status.esc[_current_module_id_target_index].esc_current  = telem_response.current * 0.01;
		_esc_status.esc[_current_module_id_target_index].esc_power    =
			_esc_status.esc[_current_module_id_target_index].esc_voltage *
			_esc_status.esc[_current_module_id_target_index].esc_current;
		_esc_status.esc[_current_module_id_target_index].esc_temperature = telem_response.mcu_temp *
				0.01; //"If you ask other escs for their temp, they're giving you the micro temp, so go with that"
		_esc_status.esc[_current_module_id_target_index].esc_state    = 0; //not implemented
		_esc_status.esc[_current_module_id_target_index].esc_cmdcount = 0; //not implemented
		_esc_status.esc[_current_module_id_target_index].failures     = 0; //not implemented

		//Update the overall _esc_status timestamp and our counter
		_esc_status.timestamp = time_now;
		_esc_status.counter++;

		_esc_status.esc_armed_flags |= (0x01 << _current_module_id_target_index);
		_esc_status.esc_online_flags |= (0x01 << _current_module_id_target_index);

		got_reply = true;
	}

	//If we got a new response or if we ran out of time to get a response from this motor move on
	if (got_reply || timed_out) {
		//We can only be fully paused once the last telemetry attempt is done
		if (_telem_state == PAUSE_REQUESTED) {
			_telem_state = PAUSED;
		}

		_time_of_last_telem_request = hrt_absolute_time();

		uint16_t next_telem = FindNextMotorForTelemetry();

		if (next_telem != _impossible_module_id) {
			//We need to update the module ID we're going to listen to. So, kill the old one, and make it anew.
			delete _telem_interface;
			_telem_interface = new IQUartFlightControllerInterfaceClient(next_telem);
		}

		//update the telem target
		return next_telem;
	}

	//Still waiting for a response or a timeout
	return _impossible_module_id;
}

uint16_t VertiqTelemetryManager::FindNextMotorForTelemetry()
{
	//If we're paused, we're just going to spit back an impossible module ID. Otherwise, go ahead and find the next target
	if (_telem_state == UNPAUSED) {
		//If our current index is the last module ID we've found, then go back to the beginning
		//otherwise just increment
		if (_current_module_id_target_index >= _number_of_module_ids_for_telem - 1) {
			_current_module_id_target_index = 0;

		} else {
			_current_module_id_target_index++;
		}

		//Return the next target
		return _module_ids_in_use[_current_module_id_target_index];
	}

	return _impossible_module_id;
}

esc_status_s VertiqTelemetryManager::GetEscStatus()
{
	return _esc_status;
}

void VertiqTelemetryManager::PauseTelemetry()
{
	_telem_state = PAUSE_REQUESTED;
}

void VertiqTelemetryManager::UnpauseTelemetry()
{
	_telem_state = UNPAUSED;
}

vertiq_telemetry_pause_states VertiqTelemetryManager::GetTelemetryPauseState()
{
	return _telem_state;
}

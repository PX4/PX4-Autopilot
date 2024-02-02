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

#include "vertiq_telemetry_manager.hpp"

VertiqTelemetryManager::VertiqTelemetryManager(IFCI *motor_interface) :
	_motor_interface(motor_interface)
{}

void VertiqTelemetryManager::Init(uint16_t telem_bitmask)
{
	//On init, make sure to set our bitmask, and then go ahead and find the front and back 1s
	_telem_bitmask = telem_bitmask;
	FindFirstAndLastTelemetryPositions();
}

void VertiqTelemetryManager::FindFirstAndLastTelemetryPositions()
{
	uint16_t shift_val = 0x0001;
	bool found_first = false;

	//Go through from 0 to the max number of CVs we can have, and determine the first and last place we have a 1
	for (uint8_t i = 0; i < MAX_SUPPORTABLE_IFCI_CVS; i++) {
		if (shift_val & _telem_bitmask) {

			//We only want to set the lowest value once
			if (!found_first) {
				_first_module_for_telem = i;

				//Also initialize the current target
				_current_telemetry_target_module_id = _first_module_for_telem;
				found_first = true;
			}

			//Keep updating the last module for every time we hit a 1
			_last_module_for_telem = i;

			//We found another module for telemetry
			_number_of_modules_for_telem++;
		}

		shift_val = shift_val << 1;
	}
}

void VertiqTelemetryManager::StartPublishing(uORB::Publication<esc_status_s> *esc_status_pub)
{
	//Initialize our ESC status publishing
	_esc_status.timestamp          = hrt_absolute_time();
	_esc_status.counter            = 0;
	_esc_status.esc_count          = _number_of_modules_for_telem;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;

	for (unsigned i = 0; i < _number_of_modules_for_telem; i++) {
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
	if (_motor_interface->telemetry_.IsFresh()) {
		//grab the data
		IFCITelemetryData telem_response = _motor_interface->telemetry_.get_reply();

		// also update our internal report for logging
		_esc_status.esc[_current_telemetry_target_module_id].esc_address  = _current_telemetry_target_module_id;
		_esc_status.esc[_current_telemetry_target_module_id].timestamp    = time_now;
		_esc_status.esc[_current_telemetry_target_module_id].esc_rpm      = telem_response.speed;
		_esc_status.esc[_current_telemetry_target_module_id].esc_voltage  = telem_response.voltage * 0.01;
		_esc_status.esc[_current_telemetry_target_module_id].esc_current  = telem_response.current * 0.01;
		_esc_status.esc[_current_telemetry_target_module_id].esc_power    =
			_esc_status.esc[_current_telemetry_target_module_id].esc_voltage *
			_esc_status.esc[_current_telemetry_target_module_id].esc_current;
		_esc_status.esc[_current_telemetry_target_module_id].esc_temperature = telem_response.mcu_temp *
				0.01; //"If you ask other escs for their temp, they're giving you the micro temp, so go with that"
		_esc_status.esc[_current_telemetry_target_module_id].esc_state    = 0; //not implemented
		_esc_status.esc[_current_telemetry_target_module_id].esc_cmdcount = 0; //not implemented
		_esc_status.esc[_current_telemetry_target_module_id].failures     = 0; //not implemented

		//Update the overall _esc_status timestamp and our counter
		_esc_status.timestamp = time_now;
		_esc_status.counter++;

		// PX4_INFO("Velo gotten from telemetry on module id %d %d", _current_telemetry_target_module_id, telem_response.speed);
		got_reply = true;
	}

	//If we got a new response or if we ran out of time to get a response from this motor move on
	if (got_reply || timed_out) {
		_time_of_last_telem_request = hrt_absolute_time();

		//update the telem target
		return FindNextMotorForTelemetry();
	}

	return _impossible_module_id;
}

uint16_t VertiqTelemetryManager::FindNextMotorForTelemetry()
{
	//If the current telemetry is the highest available, wrap around to the first one
	//If we're below the max, find the next available.

	//The bitmask shifted by the number of telemetry targets we've alredy hit
	uint16_t next_telem_target_mask = _telem_bitmask >> (_current_telemetry_target_module_id + 1);

	//The bit position of the next telemetry target
	uint16_t next_telem_target_position = _current_telemetry_target_module_id  + 1;

	//Keep trying to find the next value until you're out of bits. if you run out of bits, your next telem is the lowest value that we saved before
	while (next_telem_target_mask > 0) {
		if (next_telem_target_mask & 0x0001) {
			_current_telemetry_target_module_id = next_telem_target_position;
			_telemetry_request_id = _current_telemetry_target_module_id;
			return _telemetry_request_id; //get out
		}

		//keep trying
		next_telem_target_position++;
		next_telem_target_mask = next_telem_target_mask >> 1;
	}

	//We didn't find anyone new. Go back to the start.
	_current_telemetry_target_module_id = _first_module_for_telem;
	_telemetry_request_id = _current_telemetry_target_module_id;

	return _telemetry_request_id;
}

esc_status_s VertiqTelemetryManager::GetEscStatus()
{
	return _esc_status;
}

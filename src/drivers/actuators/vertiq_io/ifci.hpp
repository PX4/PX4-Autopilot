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
#ifndef IFCI_HPP
#define IFCI_HPP

#include "iq-module-communication-cpp/inc/client_communication.hpp"
#include "ifci_telemetry.h"

//This class handles all IFCI communication to and from a Vertiq module
class IFCI
{
public:
	//Construct a new IFCI object
	IFCI();

	//Sends a packed control message to all vertiq modules connected to the serial bus
	void BroadcastPackedControlMessage(CommunicationInterface &com, uint16_t *values, uint8_t length, uint8_t telem);

	//Sends a packed control message to a specified module on the serial bus
	void SendPackedControlMessage(CommunicationInterface &com, uint16_t *values, uint8_t length, uint8_t telem,
				      uint8_t obj_id);

	//Extracts the telemetry data out of a buffer of serial data
	int ReadTelemetry(uint8_t *rx_data, uint8_t rx_length);

	//Returns the module ID of the last received telemetry data
	uint8_t get_last_telemetry_received_id();

	//A client entry of type IFCITelemetryData
	ClientEntry<IFCITelemetryData>  telemetry_;

private:
	static const uint8_t kPackedControlMessage =  0;
	static const uint8_t kTelemetry = 1;
	static const uint8_t kTypeIFCI = 88;
	static const uint8_t kBroadcastID = 63;
	uint8_t last_telemetry_received_id_ = 63;

};

#endif //IFCI_HPP

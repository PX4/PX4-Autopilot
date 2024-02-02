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
#include "ifci.hpp"

IFCI::IFCI() :
	telemetry_(kTypeIFCI, 63, kTelemetry),
	last_telemetry_received_id_(63)
{
};

void IFCI::BroadcastPackedControlMessage(CommunicationInterface &com, uint16_t *values, uint8_t length, uint8_t telem)
{
	//Calls the targeted send message, but with the broadcast ID placed as the module ID. This allows all modules to receive the message
	SendPackedControlMessage(com, values, length, telem, kBroadcastID);
}

void IFCI::SendPackedControlMessage(CommunicationInterface &com, uint16_t *values, uint8_t length, uint8_t telem,
				    uint8_t obj_id)
{
	//all control values are 2 bytes, so we need to make sure we have enough space
	uint8_t cv_array_size = 2 * length;

	//Make space for the telemetry ID tail byte
	uint8_t payload_size = cv_array_size + 1;

	//Make space for the CRC bytes
	uint8_t total_size = 2 + payload_size;

	uint8_t tx_msg[total_size]; // must fit outgoing message

	//Load in the type, object, and access. This is always a set/
	tx_msg[0] = kPackedControlMessage;
	tx_msg[1] = (obj_id << 2) | Access::kSet; // high six | low two

	//Add the rest of our data to the array
	memcpy(&tx_msg[2], &values[0], cv_array_size);

	//Set in the ID of the module whose telemetry we want then send the data
	tx_msg[total_size - 1] = telem;
	com.SendPacket(kTypeIFCI, tx_msg, total_size);
}

int IFCI::ReadTelemetry(uint8_t *rx_data, uint8_t rx_length)
{
	//Based on the Vertiq standard, grab the data out of the correct places
	uint8_t type_idn = rx_data[0];
	uint8_t sub_idn = rx_data[1];
	uint8_t obj_idn = rx_data[2] >> 2; // high 6 bits are obj_idn
	Access dir = static_cast<Access>(rx_data[2] & 0b00000011); // low two bits

	if (dir == kReply) {
		// if sub_idn is within array range (safe to access array at this location)
		if (sub_idn == kTelemetry && type_idn == kTypeIFCI) {
			// ... then we have a valid message
			telemetry_.Reply(&rx_data[3], rx_length - 3);
			last_telemetry_received_id_ = obj_idn;
			return obj_idn; // I parsed something
		}
	}

	return kBroadcastID; // I didn't parse anything
}

uint8_t IFCI::get_last_telemetry_received_id()
{
	return last_telemetry_received_id_;
}

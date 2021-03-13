/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file VescDriverUart.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "VescDriver.hpp"
#include <memory.h>

void VescDriver::commandDutyCycle(float duty_cycle, const uint8_t forward_can_id)
{
	uint8_t command[5] {VescCommand::SET_DUTY};
	uint16_t index{1};
	insertInt32(command, index, static_cast<int32_t>(duty_cycle * 100000.f));
	sendPayload(command, 5, forward_can_id);
}

void VescDriver::commandCurrent(float current, const uint8_t forward_can_id)
{
	uint8_t command[5] {VescCommand::SET_CURRENT};
	uint16_t index{1};
	insertInt32(command, index, static_cast<int32_t>(current * 1000.f));
	sendPayload(command, 5, forward_can_id);
}

void VescDriver::commandBrakeCurrent(float current, const uint8_t forward_can_id)
{
	uint8_t command[5] {VescCommand::SET_CURRENT_BRAKE};
	uint16_t index{1};
	insertInt32(command, index, static_cast<int32_t>(current * 1000.f));
	sendPayload(command, 5, forward_can_id);
}

void VescDriver::requestFirmwareVersion()
{
	uint8_t command{VescCommand::FW_VERSION};
	sendPayload(&command, 1);
}

void VescDriver::requestValues()
{
	uint8_t command{VescCommand::GET_VALUES};
	sendPayload(&command, 1);
}

size_t VescDriver::sendPayload(const uint8_t *payload, const uint16_t payload_length, const uint8_t forward_can_id)
{
	if (forward_can_id == 0) {
		return sendPacket(payload, payload_length);

	} else {
		uint8_t command[2 + payload_length] {VescCommand::FORWARD_CAN, forward_can_id};
		uint16_t index{2};
		memcpy(&command[index], payload, payload_length);
		index += payload_length;
		return sendPacket(command, 2 + payload_length);
	}
}

size_t VescDriver::sendPacket(const uint8_t *payload, uint16_t payload_length, const uint8_t can_forwarding_id)
{
	if (payload_length == 0 || payload_length > MAXIMUM_PAYLOAD_LENGTH) {
		return 0;
	}

	uint8_t packet_buffer[payload_length + PACKET_OVERHEAD_LENGTH];
	uint16_t index{0};

	// Start byte and payload size
	if (payload_length <= 256) {
		packet_buffer[index++] = 2;
		packet_buffer[index++] = payload_length;

	} else {
		packet_buffer[index++] = 3;
		packet_buffer[index++] = static_cast<uint8_t>(payload_length >> 8);
		packet_buffer[index++] = static_cast<uint8_t>(payload_length & 0xFF);
	}

	// Payload
	memcpy(&packet_buffer[index], payload, payload_length);
	index += payload_length;

	// CRC
	const uint16_t crc = crc16(payload, payload_length);
	packet_buffer[index++] = static_cast<uint8_t>(crc >> 8);
	packet_buffer[index++] = static_cast<uint8_t>(crc & 0xFF);

	// Stop byte
	packet_buffer[index++] = 3;

	// Write bytes out through callback interface
	return _vesc_writable.writeCallback(packet_buffer, index);
}

void VescDriver::parseInputByte(uint8_t byte)
{
	if (_input_byte_index == 0) {
		// Start byte
		if (byte == 2 /*|| byte == 3*/) {
			_input_start_byte = byte;
			_input_byte_index++;
		}

	} else if (_input_byte_index == 1) {
		// Payload size
		_input_byte_index++;

		if (_input_start_byte == 2) {
			// Short packet
			_input_payload_length = byte;

			if (_input_payload_length < 1) {
				_input_byte_index = 0;
			}

		} else {
			// Long packet high byte
			_input_payload_length = byte << 8;
		}

	} else if (_input_byte_index == 2 && _input_start_byte == 3) {
		// Payload size long packet low byte
		_input_payload_length |= byte;
		_input_byte_index++;

		if (_input_payload_length < 255 || _input_payload_length > MAXIMUM_PAYLOAD_LENGTH) {
			_input_byte_index = 0;
		}

	} else if (_input_byte_index < _input_start_byte + _input_payload_length) {
		// Payload
		_input_payload[_input_byte_index - _input_start_byte] = byte;
		_input_byte_index++;

	} else if (_input_byte_index == _input_start_byte + _input_payload_length) {
		// CRC high byte
		_input_payload_crc = byte << 8;
		_input_byte_index++;

	} else if (_input_byte_index == _input_start_byte + _input_payload_length + 1u) {
		// CRC low byte
		_input_payload_crc |= byte;
		_input_byte_index++;

		if (_input_payload_crc != crc16(_input_payload, _input_payload_length)) {
			_input_byte_index = 0;
		}

	} else if (_input_byte_index == _input_start_byte + _input_payload_length + 2u) {
		// Stop byte
		_input_byte_index = 0;

		if (byte == 3) {
			parsePayload(_input_payload, _input_payload_length);
		}
	}
}

void VescDriver::parsePayload(const uint8_t *payload, const uint16_t payload_length)
{
	uint16_t index{1};

	switch (payload[0]) {
	case VescCommand::FW_VERSION:
		if (payload_length >= 9u) {
			_vesc_version.version_major = payload[index++];
			_vesc_version.version_minor = payload[index++];
			// strcpy(_vesc_version.hardware_name, reinterpret_cast<const char *>(&payload[index]));
			// index += strlen(_vesc_version.hardware_name) + 1u;
			// memcpy(_vesc_version.stm32_uuid_8, &payload[index], sizeof(_vesc_version.stm32_uuid_8));
			// index += 12;
			// _vesc_version.pairing_done = payload[index++];
			// _vesc_version.test_version_number = payload[index++];
			// _vesc_version.hardware_type = payload[index++];
			// _vesc_version.custom_configuration = payload[index++];
		}

		break;

	case VescCommand::GET_VALUES:
		if (payload_length >= 73u) {
			_vesc_values.fet_temperature = extractFloat16(payload, index) / 10.f;
			_vesc_values.motor_temperature = extractFloat16(payload, index) / 10.f;
			_vesc_values.motor_current = extractFloat32(payload, index) / 100.f;
			_vesc_values.input_current = extractFloat32(payload, index) / 100.f;
			_vesc_values.reset_average_id = extractFloat32(payload, index) / 100.f;
			_vesc_values.reset_average_iq = extractFloat32(payload, index) / 100.f;
			_vesc_values.duty_cycle = extractFloat16(payload, index) / 1000.f;
			_vesc_values.rpm = extractInt32(payload, index);
			_vesc_values.input_voltage = extractFloat16(payload, index) / 10.f;
			_vesc_values.used_charge_Ah = extractFloat32(payload, index) / 1e4f;
			_vesc_values.charged_charge_Ah = extractFloat32(payload, index) / 1e4f;
			_vesc_values.used_energy_Wh = extractFloat32(payload, index) / 1e4f;
			_vesc_values.charged_energy_wh = extractFloat32(payload, index) / 10.f;
			_vesc_values.tachometer = extractInt32(payload, index);
			_vesc_values.tachometer_absolute = extractInt32(payload, index);
			_vesc_values.fault = payload[index++];
			_vesc_values.position_pid = extractFloat32(payload, index) / 1e6f;
			_vesc_values.controller_id = payload[index++];
			_vesc_values.ntc_temperature_mos1 = extractFloat16(payload, index) / 10.f;
			_vesc_values.ntc_temperature_mos2 = extractFloat16(payload, index) / 10.f;
			_vesc_values.ntc_temperature_mos3 = extractFloat16(payload, index) / 10.f;
			_vesc_values.read_reset_average_vd = extractFloat32(payload, index) / 1000.f;
			_vesc_values.read_reset_average_vq = extractFloat32(payload, index) / 1000.f;
		}

		break;
	}
}

uint16_t VescDriver::crc16(const uint8_t *buffer, const uint16_t length)
{
	uint16_t checksum{0};

	for (size_t i = 0; i < length; i++) {
		uint8_t table_index = (((checksum >> 8) ^ buffer[i]) & 0xFF);
		checksum = CRC_TABLE[table_index] ^ (checksum << 8);
	}

	return checksum;
}

void VescDriver::insertInt32(uint8_t *buffer, uint16_t &index, int32_t value)
{
	buffer[index++] = value >> 24;
	buffer[index++] = value >> 16;
	buffer[index++] = value >> 8;
	buffer[index++] = value;
}

int16_t VescDriver::extractInt16(const uint8_t *buffer, uint16_t &index)
{
	index += 2;
	return static_cast<int16_t>(buffer[index - 2] << 8 | buffer[index - 1]);
}

float VescDriver::extractFloat16(const uint8_t *buffer, uint16_t &index)
{
	return static_cast<float>(extractInt16(buffer, index));
}

int32_t VescDriver::extractInt32(const uint8_t *buffer, uint16_t &index)
{
	index += 4;
	return static_cast<int32_t>(
		       buffer[index - 4] << 24 | buffer[index - 3] << 16 | buffer[index - 2] << 8 | buffer[index - 1]);
}

float VescDriver::extractFloat32(const uint8_t *buffer, uint16_t &index)
{
	return static_cast<float>(extractInt32(buffer, index));
}

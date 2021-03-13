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
 * @file VescDriverUart.hpp
 * @brief Driver to communicate with VESC motor contollers
 * @details More about VESC BLDC (brushless direct current) electric motor controllers by Benjamin Vedder on https://vesc-project.com/
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "VescProtocol.h"
#include "VescWritableInterface.hpp"
#include <stdint.h>
#include <stdio.h>

class VescDriver
{
public:
	VescDriver(VescWritableInterface &vesc_writable) : _vesc_writable(vesc_writable) {};
	~VescDriver() = default;

	void commandDutyCycle(float duty_cycle, const uint8_t forward_can_id = 0);
	void commandCurrent(float current, const uint8_t forward_can_id = 0);
	void commandBrakeCurrent(float current, const uint8_t forward_can_id = 0);

	void requestFirmwareVersion();
	void requestValues();
	float getRpm() { return _vesc_values.rpm; };
	float getInputVoltage() { return _vesc_values.input_voltage; };
	uint8_t getVersionMajor() { return _vesc_version.version_major; }
	uint8_t getVersionMinor() { return _vesc_version.version_minor; }

	void parseInputByte(uint8_t byte); ///< Call when data is ready to read

private:
	// De-/serialize packets
	size_t sendPayload(const uint8_t *payload, const uint16_t payload_length, const uint8_t forward_can_id = 0);
	size_t sendPacket(const uint8_t *payload, uint16_t payload_length, const uint8_t can_forwarding_id = 0);
	void parsePayload(const uint8_t *payload, const uint16_t payload_length);
	uint16_t crc16(const uint8_t *buffer, const uint16_t length);

	// Big-endian helpers
	void insertInt32(uint8_t *buffer, uint16_t &index, int32_t value);
	int16_t extractInt16(const uint8_t *buffer, uint16_t &index);
	float extractFloat16(const uint8_t *buffer, uint16_t &index);
	int32_t extractInt32(const uint8_t *buffer, uint16_t &index);
	float extractFloat32(const uint8_t *buffer, uint16_t &index);

	// Write access to device through callback
	VescWritableInterface &_vesc_writable;

	// Input packet parsing
	size_t _input_byte_index{0}; ///< keeps track of the input packets parsing state
	uint8_t _input_start_byte{0};
	uint16_t _input_payload_length{0};
	uint8_t _input_payload[MAXIMUM_PAYLOAD_LENGTH];
	uint16_t _input_payload_crc{0};

	// Information storage for getters
	VescVersion _vesc_version{};
	VescValues _vesc_values{};
};

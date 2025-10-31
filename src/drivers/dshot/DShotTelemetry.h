/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/esc_eeprom.h>
#include "DShotCommon.h"

class DShotTelemetry
{
public:

	~DShotTelemetry();

	int init(const char *uart_device, bool swap_rxtx);
	void printStatus() const;
	bool enabled() { return _enabled; }

	void startTelemetryRequest();
	bool telemetryRequestFinished();

	TelemetryStatus parseTelemetryPacket(EscData *esc_data);

	// Attempt to parse a command response.
	// Returns TODO
	bool parseCommandResponse();

	bool expectingCommandResponse();

	void setExpectCommandResponse(int motor_index, uint16_t command);

private:
	static constexpr int COMMAND_RESPONSE_MAX_SIZE = 128;
	static constexpr int COMMAND_RESPONSE_SETTINGS_SIZE = 49; // 48B for EEPROM + 1B for CRC
	static constexpr int TELEMETRY_FRAME_SIZE = 10;

	bool decodeTelemetryResponse(uint8_t *buffer, int length, EscData *esc_data);


	bool parseSettingsRequestResponse(uint8_t *buf, int size);

	// Decodes success_size bytes and returns true if all were received
	bool decodeCommandResponseByte(uint8_t byte, int success_size);

	static uint8_t crc8(const uint8_t *buf, uint8_t len);

	bool _enabled{false};
	device::Serial _uart{};

	// Command response
	int _command_response_motor_index{-1};
	uint16_t _command_response_command{0};
	uint8_t _command_response_buffer[COMMAND_RESPONSE_MAX_SIZE];
	int _command_response_position{0};
	hrt_abstime _command_response_start{0};
	int _recv_bytes{0};

	// Telemetry packet
	EscData _latest_data{};
	uint8_t _frame_buffer[TELEMETRY_FRAME_SIZE];
	int _frame_position{0};
	hrt_abstime _telemetry_request_start{0};

	// statistics
	int _num_timeouts{0};
	int _num_successful_responses{0};
	int _num_checksum_errors{0};

	uORB::Publication<esc_eeprom_s> _esc_eeprom_pub{ORB_ID(esc_eeprom)};
};

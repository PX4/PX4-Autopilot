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

#include "DShotTelemetry.h"

#include <px4_platform_common/log.h>
#include <drivers/drv_dshot.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

using namespace time_literals;

#define DSHOT_TELEMETRY_UART_BAUDRATE 115200

DShotTelemetry::~DShotTelemetry()
{
	_uart.close();

	// Clean up settings handlers
	for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {
		if (_settings_handlers[i]) {
			delete _settings_handlers[i];
			_settings_handlers[i] = nullptr;
		}
	}
}

int DShotTelemetry::init(const char *port, bool swap_rxtx)
{
	if (!_uart.setPort(port)) {
		PX4_ERR("Error configuring port %s", port);
		return PX4_ERROR;
	}

	if (!_uart.setBaudrate(DSHOT_TELEMETRY_UART_BAUDRATE)) {
		PX4_ERR("Error setting baudrate on %s", port);
		return PX4_ERROR;
	}

	if (swap_rxtx) {
		if (!_uart.setSwapRxTxMode()) {
			PX4_ERR("Error swapping TX/RX");
			return PX4_ERROR;
		}
	}

	if (! _uart.open()) {
		PX4_ERR("Error opening %s", port);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void DShotTelemetry::initSettingsHandlers(ESCType esc_type, uint8_t output_mask)
{
	if (_settings_initialized) {
		return;
	}

	_esc_type = esc_type;

	for (uint8_t i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {

		bool output_enabled = (1 << i) & output_mask;

		if (!output_enabled) {
			continue;
		}

		ESCSettingsInterface *interface = nullptr;

		switch (esc_type) {
		case ESCType::AM32:
			interface = new AM32Settings(i);
			break;

		default:
			PX4_WARN("Unsupported ESC type for settings: %d", (int)esc_type);
			break;
		}

		if (interface) {
			_settings_handlers[i] = interface;
		}
	}

	_settings_initialized = true;
}

int DShotTelemetry::parseCommandResponse()
{
	if (hrt_elapsed_time(&_command_response_start) > 1_s) {
		PX4_WARN("Command response timed out: %d bytes received", _command_response_position);
		_command_response_motor_index = -1;
		_command_response_start = 0;
		_command_response_position = 0;
		return -1;
	}

	if (_uart.bytesAvailable() <= 0) {
		return -1;
	}

	uint8_t buf[COMMAND_RESPONSE_MAX_SIZE];
	int bytes = _uart.read(buf, sizeof(buf));

	// Add bytes to buffer
	for (int i = 0; i < bytes; i++) {
		_command_response_buffer[_command_response_position++] = buf[i];
	}

	int index = -1;

	switch (_command_response_command) {
	case DSHOT_CMD_ESC_INFO: {
			auto handler = _settings_handlers[_command_response_motor_index];

			if (handler && _command_response_position == handler->getExpectedResponseSize()) {
				if (handler->decodeInfoResponse(_command_response_buffer, _command_response_position)) {
					index = _command_response_motor_index;
				}

				// Reset command state
				_command_response_position = 0;
				_command_response_start = 0;
				_command_response_motor_index = -1;
			}

			break;
		}

	default:
		break;
	}

	return index;
}

TelemetryStatus DShotTelemetry::parseTelemetryPacket(EscData *esc_data)
{
	if (telemetryResponseFinished()) {
		return TelemetryStatus::NotStarted;
	}

	// read from the uart. This must be non-blocking, so check first if there is data available
	if (_uart.bytesAvailable() <= 0) {
		if (hrt_elapsed_time(&_telemetry_request_start) > 30_ms) {
			// NOTE: this happens when sending commands, there's a window after an ESC receives
			// a command where it will not respond to any telemetry requests
			// PX4_INFO("ESC telemetry timeout: %d", esc_data->motor_index);
			++_num_timeouts;

			// Mark telemetry request as finished
			_telemetry_request_start = 0;
			_frame_position = 0;
			return TelemetryStatus::Timeout;
		}

		return TelemetryStatus::NotReady;
	}

	uint8_t buf[TELEMETRY_FRAME_SIZE];
	int bytes = _uart.read(buf, sizeof(buf));

	return decodeTelemetryResponse(buf, bytes, esc_data);
}

TelemetryStatus DShotTelemetry::decodeTelemetryResponse(uint8_t *buffer, int length, EscData *esc_data)
{
	auto status = TelemetryStatus::NotReady;

	for (int i = 0; i < length; i++) {
		_frame_buffer[_frame_position++] = buffer[i];

		/*
		 * ESC Telemetry Frame Structure (10 bytes total)
		 * =============================================
		 * Byte 0:     Temperature (uint8_t) [deg C]
		 * Byte 1-2:   Voltage (uint16_t, big-endian) [0.01V]
		 * Byte 3-4:   Current (uint16_t, big-endian) [0.01A]
		 * Byte 5-6:   Consumption (uint16_t, big-endian) [mAh]
		 * Byte 7-8:   eRPM (uint16_t, big-endian) [100ERPM]
		 * Byte 9:     CRC8 Checksum
		 */

		if (_frame_position == TELEMETRY_FRAME_SIZE) {
			uint8_t checksum = crc8(_frame_buffer, TELEMETRY_FRAME_SIZE - 1);
			uint8_t checksum_data = _frame_buffer[TELEMETRY_FRAME_SIZE - 1];

			if (checksum == checksum_data) {

				uint8_t temperature = _frame_buffer[0];
				int16_t voltage = (_frame_buffer[1] << 8) | _frame_buffer[2];
				int16_t current = (_frame_buffer[3] << 8) | _frame_buffer[4];
				// int16_t consumption = (_frame_buffer[5]) << 8 | _frame_buffer[6];
				int16_t erpm = (_frame_buffer[7] << 8) | _frame_buffer[8];

				esc_data->timestamp = hrt_absolute_time();
				esc_data->temperature = (float)temperature;
				esc_data->voltage = (float)voltage * 0.01f;
				esc_data->current = (float)current * 0.01f;;
				esc_data->erpm = erpm * 100;

				++_num_successful_responses;
				status = TelemetryStatus::Ready;
				_uart.flush();

			} else {
				++_num_checksum_errors;
				status = TelemetryStatus::ParseError;
			}

			// Mark telemetry request as finished
			_telemetry_request_start = 0;
			_frame_position = 0;
		}
	}

	return status;
}

void DShotTelemetry::publish_esc_settings()
{
	for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {
		if (_settings_handlers[i]) {
			_settings_handlers[i]->publish_latest();
		}
	}
}

void DShotTelemetry::setExpectCommandResponse(int motor_index, uint16_t command)
{
	_command_response_motor_index = motor_index;
	_command_response_command = command;
	_command_response_start = hrt_absolute_time();
	_command_response_position = 0;
}

bool DShotTelemetry::commandResponseFinished()
{
	return _command_response_motor_index < 0;
}

void DShotTelemetry::startTelemetryRequest()
{
	_telemetry_request_start = hrt_absolute_time();
}

bool DShotTelemetry::telemetryResponseFinished()
{
	return _telemetry_request_start == 0;
}

void DShotTelemetry::printStatus() const
{
	PX4_INFO("Number of successful ESC frames: %i", _num_successful_responses);
	PX4_INFO("Number of timeouts: %i", _num_timeouts);
	PX4_INFO("Number of CRC errors: %i", _num_checksum_errors);
}

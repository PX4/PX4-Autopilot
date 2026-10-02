/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

// How long an ESC gets to deliver its 10-byte response before we give up on it and poll the next motor.
// Generous on purpose: an ESC busy commutating can be slow to answer, and abandoning a response that is
// still on its way costs a full round-robin pass of telemetry rather than a single frame.
static constexpr hrt_abstime TELEMETRY_RESPONSE_TIMEOUT = 30_ms;

// Telemetry stops while a command response is pending, and the arming check reports an ESC offline once its
// telemetry is 400 ms old. AM32 sends the 49-byte dump from its main loop as soon as the sixth frame arrives,
// so a response that has not started within 100 ms is not coming.
static constexpr hrt_abstime COMMAND_RESPONSE_TIMEOUT = 100_ms;

// A failed read costs a full COMMAND_RESPONSE_TIMEOUT of telemetry, so retries are spaced for all ESCs
// together and an ESC that keeps failing is dropped until something re-requests it.
static constexpr hrt_abstime SETTINGS_RETRY_INTERVAL = 2_s;
static constexpr int SETTINGS_MAX_ATTEMPTS = 3;

DShotTelemetry::~DShotTelemetry()
{
	_uart.close();

	// Clean up settings handlers
	for (int i = 0; i < DSHOT_MAX_MOTORS; i++) {
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

void DShotTelemetry::initSettingsHandlers(ESCType esc_type, uint16_t output_mask)
{
	if (_settings_initialized) {
		return;
	}

	_esc_type = esc_type;

	for (uint8_t i = 0; i < DSHOT_MAX_MOTORS; i++) {

		bool output_enabled = (1 << i) & output_mask;

		if (!output_enabled) {
			continue;
		}

		ESCSettingsInterface *interface = nullptr;

		switch (esc_type) {
		case ESCType::AM32:
			interface = new AM32Settings(i);
			break;

		case ESCType::Unknown:
			break;

		default:
			PX4_WARN("Unsupported ESC type for settings: %d", (int)esc_type);
			break;
		}

		if (interface) {
			_settings_handlers[i] = interface;
			_settings_request_mask |= 1u << i;
		}
	}

	_settings_initialized = true;
}

void DShotTelemetry::publishSettings()
{
	if (hrt_elapsed_time(&_settings_last_publish) < 1_s) {
		return;
	}

	_settings_last_publish = hrt_absolute_time();

	for (auto handler : _settings_handlers) {
		if (handler) {
			handler->publish_latest();
		}
	}
}

int DShotTelemetry::getSettingsRequest(uint16_t motor_mask)
{
	if (hrt_absolute_time() < _settings_retry_after) {
		return -1;
	}

	// Rotate through pending reads so an unresponsive ESC cannot starve the others.
	for (int offset = 0; offset < DSHOT_MAX_MOTORS; ++offset) {
		const int index = (_next_settings_motor + offset) % DSHOT_MAX_MOTORS;

		if ((motor_mask & _settings_request_mask & (1u << index)) && _settings_handlers[index]) {
			_next_settings_motor = (index + 1) % DSHOT_MAX_MOTORS;
			return index;
		}
	}

	return -1;
}

void DShotTelemetry::requestSettings(uint16_t motor_mask)
{
	_settings_request_mask |= motor_mask;

	for (int i = 0; i < DSHOT_MAX_MOTORS; i++) {
		if (motor_mask & (1u << i)) {
			_settings_attempts[i] = 0;
		}
	}
}

void DShotTelemetry::invalidateSettings(uint16_t motor_mask)
{
	requestSettings(motor_mask);

	for (int i = 0; i < DSHOT_MAX_MOTORS; i++) {
		if ((motor_mask & (1 << i)) && _settings_handlers[i]) {
			_settings_handlers[i]->invalidate();
		}
	}
}

void DShotTelemetry::resetCommandResponse()
{
	_command_response_motor_index = -1;
	_command_response_start = 0;
	_command_response_position = 0;
}

void DShotTelemetry::settingsReadFailed()
{
	const int motor_index = _command_response_motor_index;

	if (_command_response_command == DSHOT_CMD_ESC_INFO && motor_index >= 0 && motor_index < DSHOT_MAX_MOTORS) {
		_settings_retry_after = hrt_absolute_time() + SETTINGS_RETRY_INTERVAL;

		if (++_settings_attempts[motor_index] >= SETTINGS_MAX_ATTEMPTS) {
			PX4_WARN("ESC%d: no settings after %d requests", motor_index + 1, SETTINGS_MAX_ATTEMPTS);
			_settings_request_mask &= ~(1u << motor_index);
		}
	}

	resetCommandResponse();
}

void DShotTelemetry::parseCommandResponse()
{
	if (hrt_elapsed_time(&_command_response_start) > COMMAND_RESPONSE_TIMEOUT) {
		PX4_DEBUG("Command response timed out: %d bytes received", _command_response_position);
		settingsReadFailed();
		return;
	}

	uint8_t buf[COMMAND_RESPONSE_MAX_SIZE] = {};
	int bytes = _uart.read(buf, sizeof(buf));

	if (bytes <= 0) {
		return;
	}

	// Handle potential overflow, fail out
	if (_command_response_position + bytes > COMMAND_RESPONSE_MAX_SIZE) {
		PX4_ERR("command response overflow");
		settingsReadFailed();
		return;
	}

	// Add bytes to buffer
	memcpy(&_command_response_buffer[_command_response_position], buf, bytes);
	_command_response_position += bytes;

	switch (_command_response_command) {
	case DSHOT_CMD_ESC_INFO: {
			if (_command_response_motor_index < 0 || _command_response_motor_index >= DSHOT_MAX_MOTORS) {
				resetCommandResponse();
				return;
			}

			auto handler = _settings_handlers[_command_response_motor_index];

			if (!handler) {
				resetCommandResponse();
				break;
			}

			if (_command_response_position == handler->getExpectedResponseSize()) {
				if (handler->decodeInfoResponse(_command_response_buffer, _command_response_position)) {
					_settings_request_mask &= ~(1u << _command_response_motor_index);
					_settings_attempts[_command_response_motor_index] = 0;
					resetCommandResponse();

				} else {
					settingsReadFailed();
				}
			}

			break;
		}

	default:
		break;
	}
}

TelemetryStatus DShotTelemetry::parseTelemetryPacket(EscData *esc_data)
{
	if (telemetryResponseFinished()) {
		return TelemetryStatus::NotStarted;
	}

	hrt_abstime elapsed = hrt_elapsed_time(&_telemetry_request_start);

	// At 115200 baud the 10-byte response takes ~868us. Skip polling until data could have arrived.
	if (elapsed < 800) {
		return TelemetryStatus::NotReady;
	}

	uint8_t buf[TELEMETRY_FRAME_SIZE];
	int bytes = _uart.read(buf, sizeof(buf));

	if (bytes <= 0) {
		if (elapsed > TELEMETRY_RESPONSE_TIMEOUT) {
			++_num_timeouts;

			// Mark telemetry request as finished
			_telemetry_request_start = 0;
			_frame_position = 0;
			return TelemetryStatus::Timeout;
		}

		return TelemetryStatus::NotReady;
	}

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
				uint16_t voltage = (_frame_buffer[1] << 8) | _frame_buffer[2];
				uint16_t current = (_frame_buffer[3] << 8) | _frame_buffer[4];
				// int16_t consumption = (_frame_buffer[5]) << 8 | _frame_buffer[6];
				uint16_t erpm = (_frame_buffer[7] << 8) | _frame_buffer[8];

				esc_data->timestamp = hrt_absolute_time();
				esc_data->temperature = (float)temperature;
				esc_data->voltage = (float)voltage * 0.01f;
				esc_data->current = (float)current * 0.01f;
				esc_data->erpm = erpm * 100;

				++_num_successful_responses;
				status = TelemetryStatus::Ready;

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

void DShotTelemetry::setExpectCommandResponse(int motor_index, uint16_t command)
{
	// Earlier commands sent with the tlm bit set leave KISS frames in the RX FIFO.
	_uart.flush();
	_command_response_motor_index = motor_index;
	_command_response_command = command;
	_command_response_start = hrt_absolute_time();
	_command_response_position = 0;
}

bool DShotTelemetry::commandResponseFinished()
{
	return _command_response_motor_index < 0;
}

bool DShotTelemetry::commandResponseStarted()
{
	return _command_response_start > 0;
}

void DShotTelemetry::startTelemetryRequest()
{
	// Discard whatever is still buffered before asking the next ESC. Responses are matched to a motor by
	// which request was outstanding, not by anything in the frame, so a late response left in the RX FIFO
	// would decode cleanly as the next motor's and offset every reading by one ESC from then on.
	_uart.flush();
	_frame_position = 0;
	_telemetry_request_start = hrt_absolute_time();
}

bool DShotTelemetry::telemetryResponseFinished()
{
	return _telemetry_request_start == 0;
}

void DShotTelemetry::printStatus() const
{
	PX4_INFO("Successful ESC frames: %i", _num_successful_responses);
	PX4_INFO("Timeouts: %i", _num_timeouts);
	PX4_INFO("CRC errors: %i", _num_checksum_errors);
	PX4_INFO("Pending settings mask (motor order): 0x%02x", _settings_request_mask);
}

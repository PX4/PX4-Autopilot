/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

using namespace time_literals;

#define DSHOT_TELEMETRY_UART_BAUDRATE 115200

DShotTelemetry::~DShotTelemetry()
{
	_uart.close();
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

int DShotTelemetry::update(int num_motors)
{
	if (_current_motor_index_request == -1) {
		// nothing in progress, start a request
		_current_motor_index_request = 0;
		_current_request_start = 0;
		_frame_position = 0;
		return -1;
	}

	// read from the uart. This must be non-blocking, so check first if there is data available
	int bytes_available = _uart.bytesAvailable();

	if (bytes_available <= 0) {
		// no data available. Check for a timeout
		const hrt_abstime now = hrt_absolute_time();

		if (_current_request_start > 0 && now - _current_request_start > 30_ms) {

			PX4_WARN("ESC telemetry timeout for motor %i (frame pos=%i)", _current_motor_index_request, _frame_position);
			++_num_timeouts;

			requestNextMotor(num_motors);
			return -2;
		}

		return -1;
	}

	uint8_t buf[ESC_FRAME_SIZE];
	int bytes = _uart.read(buf, sizeof(buf));

	int ret = -1;

	for (int i = 0; i < bytes && ret == -1; ++i) {

		bool successful_decoding = false;

		if (decodeByte(buf[i], successful_decoding)) {
			if (successful_decoding) {
				ret = _current_motor_index_request;
			}

			requestNextMotor(num_motors);
		}
	}

	return ret;
}

/*
 * ESC Telemetry Frame Structure (10 bytes total)
 * =============================================
 * Byte 0:     Temperature (uint8_t)
 * Byte 1-2:   Voltage (uint16_t, big-endian)
 * Byte 3-4:   Current (uint16_t, big-endian)
 * Byte 5-6:   Consumption (uint16_t, big-endian)
 * Byte 7-8:   eRPM (uint16_t, big-endian)
 * Byte 9:     CRC8 Checksum
 */
bool DShotTelemetry::decodeByte(uint8_t byte, bool &successful_decoding)
{
	_frame_buffer[_frame_position++] = byte;

	if (_frame_position == ESC_FRAME_SIZE) {
		PX4_DEBUG("got ESC frame for motor %i", _current_motor_index_request);
		uint8_t checksum = crc8(_frame_buffer, ESC_FRAME_SIZE - 1);
		uint8_t checksum_data = _frame_buffer[ESC_FRAME_SIZE - 1];

		if (checksum == checksum_data) {
			_latest_data.time = hrt_absolute_time();
			_latest_data.temperature = _frame_buffer[0];
			_latest_data.voltage = (_frame_buffer[1] << 8) | _frame_buffer[2];
			_latest_data.current = (_frame_buffer[3] << 8) | _frame_buffer[4];
			_latest_data.consumption = (_frame_buffer[5]) << 8 | _frame_buffer[6];
			_latest_data.erpm = (_frame_buffer[7] << 8) | _frame_buffer[8];
			PX4_DEBUG("Motor %i: temp=%i, V=%i, cur=%i, consumpt=%i, rpm=%i", _current_motor_index_request,
				  _latest_data.temperature, _latest_data.voltage, _latest_data.current, _latest_data.consumption,
				  _latest_data.erpm);
			++_num_successful_responses;
			successful_decoding = true;

		} else {
			++_num_checksum_errors;
		}

		return true;
	}

	return false;
}

void DShotTelemetry::printStatus() const
{
	PX4_INFO("Number of successful ESC frames: %i", _num_successful_responses);
	PX4_INFO("Number of timeouts: %i", _num_timeouts);
	PX4_INFO("Number of CRC errors: %i", _num_checksum_errors);
}

uint8_t DShotTelemetry::crc8(const uint8_t *buf, uint8_t len)
{
	auto update_crc8 = [](uint8_t crc, uint8_t crc_seed) {
		uint8_t crc_u = crc ^ crc_seed;

		for (int i = 0; i < 8; ++i) {
			crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
		}

		return crc_u;
	};

	uint8_t crc = 0;

	for (int i = 0; i < len; ++i) {
		crc = update_crc8(buf[i], crc);
	}

	return crc;
}

void DShotTelemetry::requestNextMotor(int num_motors)
{
	_current_motor_index_request = (_current_motor_index_request + 1) % num_motors;
	_current_request_start = 0;
	_frame_position = 0;
}

int DShotTelemetry::getNextMotorIndex()
{
	if (requestInProgress()) {
		// already in progress, do not send another request
		return -1;
	}

	_current_request_start = hrt_absolute_time();
	return _current_motor_index_request;
}

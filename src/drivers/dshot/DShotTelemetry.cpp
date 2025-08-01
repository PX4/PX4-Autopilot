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
			if (_redirect_output) {
				// clear and go back to internal buffer
				_redirect_output = nullptr;
				_current_motor_index_request = -1;

			} else {
				PX4_DEBUG("ESC telemetry timeout for motor %i (frame pos=%i)", _current_motor_index_request, _frame_position);
				++_num_timeouts;
			}

			requestNextMotor(num_motors);
			return -2;
		}

		return -1;
	}

	uint8_t buf[ESC_FRAME_SIZE];
	int bytes = _uart.read(buf, sizeof(buf));

	int ret = -1;

	for (int i = 0; i < bytes && ret == -1; ++i) {
		if (_redirect_output) {
			_redirect_output->buffer[_redirect_output->buf_pos++] = buf[i];

			if (_redirect_output->buf_pos == sizeof(_redirect_output->buffer)) {
				// buffer full: return & go back to internal buffer
				_redirect_output = nullptr;
				ret = _current_motor_index_request;
				_current_motor_index_request = -1;
				requestNextMotor(num_motors);
			}

		} else {
			bool successful_decoding;

			if (decodeByte(buf[i], successful_decoding)) {
				if (successful_decoding) {
					ret = _current_motor_index_request;
				}

				requestNextMotor(num_motors);
			}
		}
	}

	return ret;
}

bool DShotTelemetry::decodeByte(uint8_t byte, bool &successful_decoding)
{
	_frame_buffer[_frame_position++] = byte;
	successful_decoding = false;

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

int DShotTelemetry::getRequestMotorIndex()
{
	if (_current_request_start != 0) {
		// already in progress, do not send another request
		return -1;
	}

	_current_request_start = hrt_absolute_time();
	return _current_motor_index_request;
}

void DShotTelemetry::decodeAndPrintEscInfoPacket(const OutputBuffer &buffer)
{
	static constexpr int version_position = 12;
	const uint8_t *data = buffer.buffer;

	if (buffer.buf_pos < version_position) {
		PX4_ERR("Not enough data received");
		return;
	}

	enum class ESCVersionInfo {
		BLHELI32,
		KissV1,
		KissV2,
	};
	ESCVersionInfo version;
	int packet_length;

	if (data[version_position] == 254) {
		version = ESCVersionInfo::BLHELI32;
		packet_length = esc_info_size_blheli32;

	} else if (data[version_position] == 255) {
		version = ESCVersionInfo::KissV2;
		packet_length = esc_info_size_kiss_v2;

	} else {
		version = ESCVersionInfo::KissV1;
		packet_length = esc_info_size_kiss_v1;
	}

	if (buffer.buf_pos != packet_length) {
		PX4_ERR("Packet length mismatch (%i != %i)", buffer.buf_pos, packet_length);
		return;
	}

	if (DShotTelemetry::crc8(data, packet_length - 1) != data[packet_length - 1]) {
		PX4_ERR("Checksum mismatch");
		return;
	}

	uint8_t esc_firmware_version = 0;
	uint8_t esc_firmware_subversion = 0;
	uint8_t esc_type = 0;

	switch (version) {
	case ESCVersionInfo::KissV1:
		esc_firmware_version = data[12];
		esc_firmware_subversion = (data[13] & 0x1f) + 97;
		esc_type = (data[13] & 0xe0) >> 5;
		break;

	case ESCVersionInfo::KissV2:
	case ESCVersionInfo::BLHELI32:
		esc_firmware_version = data[13];
		esc_firmware_subversion = data[14];
		esc_type = data[15];
		break;
	}

	const char *esc_type_str = "";

	switch (version) {
	case ESCVersionInfo::KissV1:
	case ESCVersionInfo::KissV2:
		switch (esc_type) {
		case 1: esc_type_str = "KISS8A";
			break;

		case 2: esc_type_str = "KISS16A";
			break;

		case 3: esc_type_str = "KISS24A";
			break;

		case 5: esc_type_str = "KISS Ultralite";
			break;

		default: esc_type_str = "KISS (unknown)";
			break;
		}

		break;

	case ESCVersionInfo::BLHELI32: {
			char *esc_type_mutable = (char *)(data + 31);
			esc_type_mutable[32] = 0;
			esc_type_str = esc_type_mutable;
		}
		break;
	}

	PX4_INFO("ESC Type: %s", esc_type_str);

	PX4_INFO("MCU Serial Number: %02x%02x%02x-%02x%02x%02x-%02x%02x%02x-%02x%02x%02x",
		 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8],
		 data[9], data[10], data[11]);

	switch (version) {
	case ESCVersionInfo::KissV1:
	case ESCVersionInfo::KissV2:
		PX4_INFO("Firmware version: %d.%d%c", esc_firmware_version / 100, esc_firmware_version % 100,
			 (char)esc_firmware_subversion);
		break;

	case ESCVersionInfo::BLHELI32:
		PX4_INFO("Firmware version: %d.%d", esc_firmware_version, esc_firmware_subversion);
		break;
	}

	if (version == ESCVersionInfo::KissV2 || version == ESCVersionInfo::BLHELI32) {
		PX4_INFO("Rotation Direction: %s", data[16] ? "reversed" : "normal");
		PX4_INFO("3D Mode: %s", data[17] ? "on" : "off");
	}

	if (version == ESCVersionInfo::BLHELI32) {
		uint8_t setting = data[18];

		switch (setting) {
		case 0:
			PX4_INFO("Low voltage Limit: off");
			break;

		case 255:
			PX4_INFO("Low voltage Limit: unsupported");
			break;

		default:
			PX4_INFO("Low voltage Limit: %d.%01d V", setting / 10, setting % 10);
			break;
		}

		setting = data[19];

		switch (setting) {
		case 0:
			PX4_INFO("Current Limit: off");
			break;

		case 255:
			PX4_INFO("Current Limit: unsupported");
			break;

		default:
			PX4_INFO("Current Limit: %d A", setting);
			break;
		}

		for (int i = 0; i < 4; ++i) {
			setting = data[i + 20];
			PX4_INFO("LED %d: %s", i, setting ? (setting == 255 ? "unsupported" : "on") : "off");
		}
	}
}

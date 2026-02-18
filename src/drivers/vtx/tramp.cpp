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

#include "tramp.h"
#include <string.h>
#include <cerrno>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <math.h>

namespace vtx
{

using namespace time_literals;

int Tramp::get_settings()
{
	const int rv = get_status();

	if (rv != 0) { return rv; }

	px4_usleep(30_ms);
	return get_temperature();
}


bool Tramp::print_settings()
{
	PX4_INFO("Tramp:");
	PX4_INFO("  frequency: %hu MHz", _settings.frequency);
	PX4_INFO("  requested power: %hu mW", _settings.requested_power_mW);
	PX4_INFO("  power: %hu mW", _settings.power_mW);
	PX4_INFO("  pit mode: %s", _settings.pit_mode ? "on" : "off");

	PX4_INFO("  temperature: %hi C", _settings.temperature);

	PX4_INFO("  min frequency: %hu MHz", _settings.min_frequency);
	PX4_INFO("  max frequency: %hu MHz", _settings.max_frequency);
	PX4_INFO("  max power: %hu mW", _settings.max_power_mW);
	return true;
}

bool Tramp::copy_to(vtx_s *msg)
{
	msg->protocol = vtx_s::PROTOCOL_TRAMP;

	msg->frequency = _settings.frequency;
	msg->power_level = _requested_power_level;
	msg->mode = _settings.pit_mode ? vtx_s::MODE_PIT : vtx_s::MODE_NORMAL;

	return true;
}

int Tramp::get_status()
{
	const uint8_t buf[] = {COMMAND_GET_SETTINGS};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	_settings.frequency = (_rx_buf[2] | (_rx_buf[3] << 8));
	_settings.requested_power_mW = (_rx_buf[4] | (_rx_buf[5] << 8));
	_settings.control_mode = _rx_buf[6];
	_settings.pit_mode = _rx_buf[7];
	_settings.power_mW = (_rx_buf[8] | (_rx_buf[9] << 8));

	return PX4_OK;
}

int Tramp::get_temperature()
{
	const uint8_t buf[] = {COMMAND_GET_TEMPERATURE};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	_settings.temperature = int16_t(_rx_buf[6] | (_rx_buf[7] << 8));

	return PX4_OK;
}

int Tramp::reset()
{
	const uint8_t buf[] = {COMMAND_RESET};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	_settings.min_frequency = int16_t(_rx_buf[2] | (_rx_buf[3] << 8));
	_settings.max_frequency = int16_t(_rx_buf[4] | (_rx_buf[5] << 8));
	_settings.max_power_mW = int16_t(_rx_buf[6] | (_rx_buf[7] << 8));

	return PX4_OK;
}

int Tramp::set_power(int16_t power_level)
{
	int16_t power_mW;
	_requested_power_level = -1;

	if (power_level < 0) {
		power_mW = -power_level;

	} else {
		power_mW = vtxtable().power_value(power_level);

		if (power_mW == 0) { return -EINVAL; }

		_requested_power_level = power_level;
	}

	const uint8_t buf[] = {COMMAND_SET_POWER, uint8_t(power_mW), uint8_t(power_mW >> 8)};
	int rv = transmit(buf, sizeof(buf));

	if (rv) { return rv; }

	px4_usleep(30_ms);
	rv = get_status();

	if (rv) { return rv; }

	return (_settings.requested_power_mW == power_mW) ? PX4_OK : PX4_ERROR;
}

int Tramp::set_frequency(int16_t frequency_MHz)
{
	if (frequency_MHz < 0) { return -EINVAL; } // Tramp does not support pit frequency setting

	const uint8_t buf[] = {COMMAND_SET_FREQUENCY, uint8_t(frequency_MHz), uint8_t(frequency_MHz >> 8)};
	int rv = transmit(buf, sizeof(buf));

	if (rv) { return rv; }

	px4_usleep(30_ms);
	rv = get_status();

	if (rv) { return rv; }

	return (_settings.frequency == frequency_MHz) ? PX4_OK : PX4_ERROR;
}

int Tramp::set_pit_mode(bool onoff)
{
	const uint8_t mode = onoff ? 0u : 1u;
	const uint8_t buf[] = {COMMAND_SET_MODE, mode};
	int rv = transmit(buf, sizeof(buf));

	if (rv) { return rv; }

	px4_usleep(30_ms);
	rv = get_status();

	if (rv) { return rv; }

	return (_settings.pit_mode == onoff ? 1u : 0u) ? PX4_OK : PX4_ERROR;
}

int Tramp::transmit(const uint8_t *buf, size_t len)
{
	if (len > 28) { return -1; }

	memset(_tx_buf + 1, 0, sizeof(_tx_buf) - 1);
	// copy the message data
	memcpy(_tx_buf + 1, buf, len);
	// compute the CRC
	_tx_buf[offsetof(Frame, crc)] = crc8(_tx_buf);

	// send command
	if (_serial->write(_tx_buf, sizeof(Frame)) < ssize_t(sizeof(Frame))) {
		return -errno;
	}

	_serial->flush();

	return rx_msg();
}

int Tramp::rx_parser(uint8_t c)
{
	enum {
		SYNC = 0,
		COMMAND = 1,
		DATA = 2,
		CRC = 3,
		END = 4,
	};
	static constexpr uint8_t CRCPOS{offsetof(Frame, crc)};

	switch (_read_state) {
	case SYNC:
		PX4_DEBUG("SYNC %x", c);

		if (c == 0x0F) {
			_rx_buf[0] = c;
			_read_state = COMMAND;
			return 1;

		} else {
			_read_state = SYNC;
			return 1;
		}

	case COMMAND:
		PX4_DEBUG("COMMAND %x", c);

		switch (c) {
		case COMMAND_RESET:
		case COMMAND_GET_TEMPERATURE:
		case COMMAND_GET_SETTINGS:
		case COMMAND_SET_POWER:
		case COMMAND_SET_FREQUENCY:
		case COMMAND_SET_MODE:
			_rx_buf[1] = c;
			_read_state = DATA;
			_read_data_length = 2;
			return CRCPOS;
		}

		_read_state = SYNC;
		return 1;

	case DATA:
		PX4_DEBUG("DATA %x", c);
		_rx_buf[_read_data_length] = c;

		if (++_read_data_length >= CRCPOS) {
			_read_state = CRC;
		}

		return sizeof(Frame) - _read_data_length;

	case CRC:
		PX4_DEBUG("CRC %x", c);
		_read_state = END;
		_rx_buf[CRCPOS] = c;
		return 1;

	case END:
		PX4_DEBUG("END %x", c);
		_read_state = SYNC;

		// small letter commands with empty payloads were sent by us
		if ((_rx_buf[1] & 0x20) && !_rx_buf[2] && !_rx_buf[3]) { return 1; }

		// large letter command do not get a response
		// Check the CRC for the received data
		return (_rx_buf[CRCPOS] == crc8(_rx_buf)) ? 0 : -CRC;
	}

	return -6000;
}

uint8_t Tramp::crc8(const uint8_t *data)
{
	uint8_t crc{0};

	for (uint_fast8_t ii{1}; ii < offsetof(Frame, crc); ii++) {
		crc += data[ii];
	}

	return crc;
}

}

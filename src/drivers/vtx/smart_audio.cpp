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

#include "smart_audio.h"
#include <string.h>
#include <cerrno>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <math.h>

namespace vtx
{

using namespace time_literals;

int SmartAudio::get_settings()
{
	const uint8_t buf[] = {COMMAND_GET_SETTINGS, 0};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	Settings s;
	memcpy(&s, _rx_buf + 2, sizeof(Settings));
	// Fix frequency endianess
	s.frequency = __builtin_bswap16(s.frequency);
	const uint8_t version = _rx_buf[0] >> 3;

	if (version == 0) { // v1
		s.number_of_power_levels = 4;
		s.power_levels[0] = 7;
		s.power_levels[1] = 16;
		s.power_levels[2] = 25;
		s.power_levels[3] = 40;

	} else if (version == 1) { // v2
		s.number_of_power_levels = 8;

	} else if (version == 2) { // v2.1
		if (s.number_of_power_levels > 7) {
			s.number_of_power_levels = 7;
		}

		for (int i = 0; i < s.number_of_power_levels; i++) {
			s.power_levels[i] = s.power_levels[i + 1];
		}
	}

	s.version = version;

	// copy the settings to storage
	_settings = s;

	return PX4_OK;
}

bool SmartAudio::print_settings()
{
	if (_settings.version > 2) { return false; }

	PX4_INFO("SmartAudio v%s%s:", ((const char *[]) {"1", "2", "2.1"})[_settings.version],
	(_settings.mode & SmartAudio::MODE_SET_FREQUENCY) ? "+fr" : "+ch");
	PX4_INFO("  band: %u", ((_settings.channel >> 3) & 0b111) + 1);
	PX4_INFO("  channel: %u", (_settings.channel & 0b111) + 1);
	PX4_INFO("  frequency: %u MHz", _settings.frequency);
	PX4_INFO("  power level: %u", _settings.current_power_level + 1);
	const char *power_label = vtxtable().power_label(_settings.current_power_level);

	if (_settings.version == 2) {
		PX4_INFO("  power: %u dBm = %s", _settings.current_power_dBm, power_label);

	} else {
		PX4_INFO("  power: %s", power_label);
	}

	PX4_INFO("  pit mode: %s", (_settings.mode & (SmartAudio::MODE_IN_RANGE_PIT_MODE |
				    SmartAudio::MODE_OUT_RANGE_PIT_MODE)) ? "on" : "off");
	PX4_INFO("  lock: %slocked", (_settings.mode & SmartAudio::MODE_UNLOCKED) ? "un" : "");
	return true;
}

bool SmartAudio::copy_to(vtx_s *msg)
{
	if (_settings.version > 2) { return false; }

	if (_settings.version == 0) { msg->protocol = vtx_s::PROTOCOL_SMART_AUDIO_V1; }

	else if (_settings.version == 1) { msg->protocol = vtx_s::PROTOCOL_SMART_AUDIO_V2; }

	else { msg->protocol = vtx_s::PROTOCOL_SMART_AUDIO_V2_1; }

	msg->band = _settings.channel >> 3;
	msg->channel = _settings.channel & 0b111;
	msg->frequency = _settings.frequency;
	msg->power_level = _settings.current_power_level;
	bool pm = (_settings.mode & (SmartAudio::MODE_IN_RANGE_PIT_MODE | SmartAudio::MODE_OUT_RANGE_PIT_MODE));
	msg->mode = pm ? vtx_s::MODE_PIT : vtx_s::MODE_NORMAL;

	return true;
}

int SmartAudio::set_power(uint8_t power_level, bool is_dBm)
{
	if (power_level >= _settings.number_of_power_levels) {
		return PX4_ERROR;
	}

	power_level = vtxtable().power_value(power_level);

	if (_settings.version == 2 && is_dBm) { power_level |= 0x80; }

	const uint8_t buf[] = {COMMAND_SET_POWER, 1, power_level};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	return ((_rx_buf[0] == COMMAND_SET_POWER) && (_rx_buf[2] == (power_level & ~0x80))) ? PX4_OK : PX4_ERROR;
}

int SmartAudio::set_channel(uint8_t band, uint8_t channel)
{
	if (vtxtable().band_attribute(band) == Config::BandAttribute::CUSTOM) {
		const uint16_t freq = vtxtable().frequency(band, channel);

		if (freq) { return set_frequency(freq, false); }
	}

	const uint8_t value = ((band & 0b111) << 3) | (channel & 0b111);
	const uint8_t buf[] = {COMMAND_SET_CHANNEL, 1, value};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	return ((_rx_buf[0] == COMMAND_SET_CHANNEL) && (_rx_buf[2] == value)) ? PX4_OK : PX4_ERROR;
}

int SmartAudio::set_frequency(uint16_t frequency, bool pit)
{
	pit ? frequency |= 0x8000 : frequency &= ~0x8000;
	const uint8_t buf[] = {COMMAND_SET_FREQUENCY, 2, uint8_t(frequency >> 8), uint8_t(frequency)};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	const bool success = (_rx_buf[0] == COMMAND_SET_FREQUENCY) && (_rx_buf[2] == (frequency >> 8));

	if (_settings.version == 2) {
		return (success && (_rx_buf[3] == (frequency & 0xff))) ? PX4_OK : PX4_ERROR;

	} else {
		// Some v1/v2 firmwares do not echo the low byte back
		// But the CRC is still correct, so we assume success
		return success ? PX4_OK : PX4_ERROR;
	}
}

int SmartAudio::set_operating_mode(uint8_t mode)
{
	if (_settings.version == 0) {
		return -1;
	}

	const uint8_t buf[] = {COMMAND_SET_MODE, 1, mode};
	const int rv = transmit(buf, sizeof(buf));

	if (rv != 0) { return rv; }

	return ((_rx_buf[0] == COMMAND_SET_MODE) && (_rx_buf[2] == mode)) ? PX4_OK : PX4_ERROR;
}

int SmartAudio::set_pit_mode(bool onoff)
{
	// only v2 supports pit mode
	if (_settings.version == 0) {
		return 0;
	}

	uint8_t mode{SET_MODE_UNLOCKED};

	if (onoff) {
		if (_settings.mode & MODE_OUT_RANGE_PIT_MODE) {
			mode |= SET_MODE_OUT_RANGE;

		} else {
			mode |= SET_MODE_IN_RANGE;
		}

	} else {
		mode |= SET_MODE_DISABLE_PIT_MODE;
	}

	return set_operating_mode(mode);
}

int SmartAudio::transmit(const uint8_t *buf, size_t len)
{
	if (len > 28) { return -1; }

	// copy the message data
	memcpy(_tx_buf + 3, buf, len);
	// shift the command to the left
	// const uint8_t cmd = buf[0];
	_tx_buf[3] = (buf[0] << 1) | 0x01;
	// compute the CRC
	_tx_buf[3 + len] = crc8(_tx_buf + 1, 2 + len);
	// some devices need an additional 0 byte at the end
	_tx_buf[4 + len] = 0;

	// send command
	if (_serial->write(_tx_buf, 5 + len) < ssize_t(5 + len)) {
		return -errno;
	}

	_serial->flush();

	return rx_msg();
}

bool SmartAudio::is_rsp(uint8_t cmd, uint8_t length)
{
	if (length == 0) { return false; }

	if (cmd == COMMAND_SET_POWER && length == 0x03) { return true; }

	if (cmd == COMMAND_SET_CHANNEL && length == 0x03) { return true; }

	if (cmd == COMMAND_SET_FREQUENCY && length == 0x04) { return true; }

	if (cmd == COMMAND_SET_MODE && length == 0x03) { return true; }

	if (((cmd == COMMAND_GET_SETTINGS) || (cmd == COMMAND_GET_SETTINGS_V2) ||
	     (cmd == COMMAND_GET_SETTINGS_V21)) && length >= 0x06) { return true; }

	return false;
}

int SmartAudio::rx_parser(uint8_t c)
{
	enum {
		SYNC1 = 0,
		SYNC2 = 1,
		COMMAND = 2,
		LENGTH = 3,
		DATA = 4,
		CRC = 5,
	};

	switch (_read_state) {
	case SYNC1:
		PX4_DEBUG("SYNC1 %x", c);

		if (c == 0xAA) {
			_read_state = SYNC2;
		}

		return 1;

	case SYNC2:
		PX4_DEBUG("SYNC2 %x", c);

		if (c == 0x55) {
			_read_state = COMMAND;
			return 3;

		} else {
			_read_state = SYNC1;
			return 1;
		}

	case COMMAND:
		PX4_DEBUG("COMMAND %x", c);
		_rx_buf[0] = c;
		_read_state = LENGTH;
		return 2;

	case LENGTH:
		PX4_DEBUG("LENGTH %x", c);

		if (c >= 20) {
			_read_state = SYNC1;
			return 1;
		}

		_rx_buf[1] = c;

		// command response: has one length too many
		if (is_rsp(_rx_buf[0], c) && c) { c--; }

		_read_length = c;
		_read_data_length = 0;
		_read_state = _read_length ? DATA : CRC;
		return _read_length + 1;

	case DATA:
		PX4_DEBUG("DATA %x", c);
		_rx_buf[2 + _read_data_length] = c;

		if (++_read_data_length >= _read_length) {
			_read_state = CRC;
		}

		return _read_length - _read_data_length + 1;

	case CRC:
		PX4_DEBUG("CRC %x", c);
		_read_state = SYNC1;

		if (!is_rsp(_rx_buf[0], _rx_buf[1])) { return 1; }

		return (c == crc8(_rx_buf, 2u + _read_length)) ? 0 : -CRC;
	}

	return -6000;
}

uint8_t SmartAudio::crc8(const uint8_t *data, const uint8_t len)
{
	// Implementation adapted from lib/crc/crc.c
	uint8_t crc{};
	const uint8_t poly{0xd5};

	for (uint8_t i = 0 ; i < len; i++) {
		crc ^= data[i];

		for (uint8_t j = 0; j < 8; j++) {
			if (crc & (1u << 7u)) {
				crc = (uint8_t)((crc << 1u) ^ poly);

			} else {
				crc = (uint8_t)(crc << 1u);
			}
		}
	}

	return crc;
}

}

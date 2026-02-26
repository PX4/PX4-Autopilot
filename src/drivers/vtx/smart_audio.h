/****************************************************************************
 *
 *	Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
#include "protocol.h"

namespace vtx
{

/**
 * Implementation for working with SmartAudio protocol.
 * @author Niklas Hauser <niklas@auterion.com>
 */
class SmartAudio final : public Protocol
{
public:
	static constexpr int BAUDRATE{4800};

	struct Settings {
		uint8_t channel;
		uint8_t current_power_level;
		uint8_t mode;
		uint16_t frequency;
		uint8_t current_power_dBm;
		uint8_t number_of_power_levels;
		uint8_t power_levels[8];
		uint8_t version;
	} __attribute__((packed));

	enum {
		MODE_SET_FREQUENCY = 0b0001,
		MODE_PIT_MODE_ACTIVE = 0b0010,
		MODE_IN_RANGE_PIT_MODE = 0b0100,
		MODE_OUT_RANGE_PIT_MODE = 0b1000,
		MODE_UNLOCKED = 0b10000,
	};
	const Settings &settings() const { return _settings; }

	inline SmartAudio() : Protocol(BAUDRATE)
	{
		_tx_buf[1] = 0xAA;
		_tx_buf[2] = 0x55;
	}
	virtual ~SmartAudio() = default;

	inline int set_power(int16_t power_level) override
	{
		// negative values do not force dBm mode
		if (power_level < 0) { return set_power(-power_level, false); }

		return set_power(power_level, true);
	}
	inline int set_frequency(int16_t frequency_MHz) override
	{
		if (frequency_MHz < 0) { return set_frequency(-frequency_MHz, true); }

		return set_frequency(frequency_MHz, false);
	}
	int set_channel(uint8_t band, uint8_t channel) override;
	int get_settings() override;
	int set_pit_mode(bool onoff) override;
	bool print_settings() override;
	bool copy_to(vtx_s *msg) override;

	// Additional methods not part of the Protocol interface
	int set_frequency(uint16_t frequency_MHz, bool pit = false);
	int set_power(uint8_t power_level, bool is_dBm = true);
	int set_operating_mode(uint8_t mode);

private:
	int rx_parser(uint8_t c) override;
	int transmit(const uint8_t *buf, size_t len);
	static uint8_t crc8(const uint8_t *data, uint8_t len);
	static bool is_rsp(uint8_t cmd, uint8_t length);

	enum {
		COMMAND_GET_SETTINGS = 0x01,
		COMMAND_SET_POWER = 0x02,
		COMMAND_SET_CHANNEL = 0x03,
		COMMAND_SET_FREQUENCY = 0x04,
		COMMAND_SET_MODE = 0x05,

		COMMAND_GET_SETTINGS_V2 = 0x09,
		COMMAND_GET_SETTINGS_V21 = 0x11,
	};

	enum {
		SET_MODE_IN_RANGE = 0b0001,
		SET_MODE_OUT_RANGE = 0b0010,
		SET_MODE_DISABLE_PIT_MODE = 0b0100,
		SET_MODE_UNLOCKED = 0b1000,
	};

	Settings _settings{.version = 99};
	uint8_t _read_length{};
	uint8_t _read_data_length{};
};

}

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
 * Implementation for working with Tramp protocol.
 * @author Niklas Hauser <niklas@auterion.com>
 */
class Tramp final : public Protocol
{
public:
	static constexpr int BAUDRATE{9600};

	struct Settings {
		// command 'v'
		uint16_t frequency;
		uint16_t requested_power_mW;
		uint8_t control_mode;
		uint8_t pit_mode;
		uint16_t power_mW;

		// command 's'
		int16_t temperature;

		// command 'r'
		uint16_t min_frequency;
		uint16_t max_frequency;
		uint16_t max_power_mW;
	};
	const Settings &settings() const { return _settings; }

	inline Tramp() : Protocol(BAUDRATE)
	{
		_tx_buf[0] = 0x0F;
	}

	int get_settings() override;
	int reset() override;
	int set_power(int16_t power_mW) override;
	int set_frequency(int16_t frequency_MHz) override;
	int set_pit_mode(bool onoff) override;
	bool print_settings() override;
	bool copy_to(vtx_s *msg) override;

	// Additional methods not part of the Protocol interface
	int get_status();
	int get_temperature();

private:
	int rx_parser(uint8_t c) override;
	int transmit(const uint8_t *buf, size_t len);
	static uint8_t crc8(const uint8_t *data);

	enum {
		COMMAND_RESET = 'r', // 0x72
		COMMAND_GET_TEMPERATURE = 's', // 0x73
		COMMAND_GET_SETTINGS = 'v', // 0x76
		COMMAND_SET_POWER = 'P', // 0x50
		COMMAND_SET_FREQUENCY = 'F', // 0x46
		COMMAND_SET_MODE = 'I', // 0x49
	};

	struct Frame {
		uint8_t start{0x0F};
		uint8_t command;
		uint8_t payload[12];
		uint8_t crc;
		uint8_t end{0x00};
	} __attribute__((packed));
	static_assert(sizeof(Frame) == 16, "Tramp frame size wrong");

	Settings _settings{};
	uint8_t _read_data_length{};
	int8_t _requested_power_level{-1};
};

} // namespace vtx

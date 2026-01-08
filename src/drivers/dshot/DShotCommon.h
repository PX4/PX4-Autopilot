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

#include <drivers/drv_hrt.h>
#include <uORB/topics/esc_status.h>

static constexpr int DSHOT_MAXIMUM_CHANNELS = esc_status_s::CONNECTED_ESC_MAX;

enum class TelemetrySource {
	Serial = 0,
	BDShot = 1,
};

struct EscData {
	int actuator_channel;  // Actuator output channel 0-7
	int motor_index;       // Motors 0-7
	hrt_abstime timestamp; // Sample time
	TelemetrySource source;

	float temperature;     // [deg C]
	float voltage;         // [0.01V]
	float current;         // [0.01A]
	int16_t erpm;          // [100ERPM]
};

enum class TelemetryStatus {
	NotStarted = 0,
	NotReady = 1,
	Ready = 2,
	Timeout = 3,
	ParseError = 4,
};

inline int count_set_bits(int mask)
{
	int count = 0;

	while (mask) {
		mask &= mask - 1;
		count++;
	}

	return count;
}

inline uint8_t crc8(const uint8_t *buf, unsigned len)
{
	auto update_crc8 = [](uint8_t crc, uint8_t crc_seed) {
		uint8_t crc_u = crc ^ crc_seed;

		for (unsigned i = 0; i < 8; ++i) {
			crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
		}

		return crc_u;
	};

	uint8_t crc = 0;

	for (unsigned i = 0; i < len; ++i) {
		crc = update_crc8(buf[i], crc);
	}

	return crc;
}

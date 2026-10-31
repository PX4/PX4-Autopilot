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

#include <gtest/gtest.h>

#include "dshot_telemetry.h"

#include <array>
#include <cstdint>

TEST(DShotGcr, DecodesEveryValidPayload)
{
	static constexpr std::array<uint8_t, 16> nibble_to_gcr{
		0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17,
		0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f
	};

	for (uint32_t expected = 0; expected < 0x1000; expected++) {
		uint32_t data = expected << 4U;
		uint32_t checksum = data ^ (data >> 8U);
		checksum ^= checksum >> 4U;
		data |= ~checksum & 0xfU;

		uint32_t encoded = 0;

		for (unsigned nibble = 0; nibble < 4; nibble++) {
			encoded |= (uint32_t)nibble_to_gcr[(data >> (nibble * 4U)) & 0xfU] << (nibble * 5U);
		}

		uint16_t payload = UINT16_MAX;
		ASSERT_TRUE(dshot_decode_gcr_payload(encoded, &payload)) << "payload " << expected;
		ASSERT_EQ(payload, expected);
	}
}

TEST(DShotGcr, RejectsEveryInvalidSymbol)
{
	static constexpr uint32_t valid_symbols = 0x6eecee00U;

	for (uint32_t encoded = 0; encoded < (1U << 20U); encoded++) {
		bool valid = true;

		for (unsigned nibble = 0; nibble < 4; nibble++) {
			const uint32_t symbol = (encoded >> (nibble * 5U)) & 0x1fU;
			valid &= (valid_symbols & (1U << symbol)) != 0;
		}

		if (!valid) {
			uint16_t payload = 0;
			ASSERT_FALSE(dshot_decode_gcr_payload(encoded, &payload))
					<< "accepted invalid GCR word 0x" << std::hex << encoded;
		}
	}
}

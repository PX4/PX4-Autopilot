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

#include "RtcmTestCommon.hpp"

TEST_F(RtcmTest, Stress_ManySmallValidFrames)
{
	// Simulate a good sender sending many small messages
	// Process as we go to avoid buffer overflow
	const int num_frames = 1000;
	int count = 0;

	for (int i = 0; i < num_frames; i++) {
		auto frame = buildValidFrame(1005 + (i % 100), {static_cast<uint8_t>(i & 0xFF)});
		parser.addData(frame.data(), frame.size());

		// Consume available messages after each add
		size_t len;

		while (parser.getNextMessage(&len) != nullptr) {
			parser.consumeMessage(len);
			count++;
		}
	}

	EXPECT_EQ(count, num_frames);
	auto stats = parser.getStats();
	EXPECT_EQ(stats.messages_parsed, static_cast<uint32_t>(num_frames));
	EXPECT_EQ(stats.bytes_discarded, 0u);
	EXPECT_EQ(stats.crc_errors, 0u);
}

TEST_F(RtcmTest, Stress_LongGarbageStreamThenValid)
{
	// This tests the pathological case Alex mentioned: lots of invalid data
	// causing many memmove operations during preamble search
	const size_t garbage_size = 10000;
	std::vector<uint8_t> garbage(garbage_size);
	std::mt19937 rng(99);

	for (auto &byte : garbage) {
		// Avoid preamble to ensure maximum discards
		byte = (rng() % 0xD2) + 1;  // 0x01 to 0xD2, never 0xD3
	}

	// Add in chunks to simulate streaming
	size_t offset = 0;

	while (offset < garbage.size()) {
		size_t chunk = std::min<size_t>(256, garbage.size() - offset);
		parser.addData(&garbage[offset], chunk);

		// Try to parse (forces discarding)
		size_t len;
		parser.getNextMessage(&len);
		offset += chunk;
	}

	// Now add a valid frame
	auto valid = buildValidFrame(1005, {0x01, 0x02, 0x03});
	parser.addData(valid.data(), valid.size());

	size_t len;
	const uint8_t *msg = parser.getNextMessage(&len);
	ASSERT_NE(msg, nullptr);

	parser.consumeMessage(len);
	auto stats = parser.getStats();
	EXPECT_EQ(stats.messages_parsed, 1u);
	EXPECT_EQ(stats.bytes_discarded, garbage_size);
}

TEST_F(RtcmTest, Stress_InterleavedGarbageAndValid)
{
	// Simulate a flaky sender that mixes garbage with valid frames
	const int num_valid = 100;
	int parsed = 0;

	for (int i = 0; i < num_valid; i++) {
		// Add some garbage (variable amount)
		std::vector<uint8_t> garbage(10 + (i % 50), 0xAA);
		parser.addData(garbage.data(), garbage.size());

		// Add valid frame
		auto frame = buildValidFrame(1005, {static_cast<uint8_t>(i)});
		parser.addData(frame.data(), frame.size());

		// Parse what we can
		size_t len;

		while (parser.getNextMessage(&len) != nullptr) {
			parser.consumeMessage(len);
			parsed++;
		}
	}

	EXPECT_EQ(parsed, num_valid);
}

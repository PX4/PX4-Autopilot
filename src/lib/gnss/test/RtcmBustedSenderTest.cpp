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

TEST_F(RtcmTest, BustedSender_AllZeros)
{
	std::vector<uint8_t> garbage(1000, 0x00);
	parser.addData(garbage.data(), garbage.size());

	size_t len;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(parser.getStats().bytes_discarded, 1000u);
	EXPECT_EQ(parser.bufferedBytes(), 0u);
}

TEST_F(RtcmTest, BustedSender_AllPreambles)
{
	// All preamble bytes (0xD3). The parser sees each 0xD3 as a potential frame start.
	// Length extracted from 0xD3,0xD3 is ((0x03)<<8)|0xD3 = 979 bytes.
	// Parser waits for full frame which never comes.
	std::vector<uint8_t> preambles(500, RTCM3_PREAMBLE);
	parser.addData(preambles.data(), preambles.size());

	size_t len;
	// Parser should be stuck waiting for more data (incomplete frame)
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	// Buffer retains data waiting for frame completion
	EXPECT_GT(parser.bufferedBytes(), 0u);
}

TEST_F(RtcmTest, BustedSender_RandomNoise)
{
	std::mt19937 rng(12345);
	uint8_t noise[500];

	for (size_t i = 0; i < sizeof(noise); i++) {
		noise[i] = rng() & 0xFF;
	}

	parser.addData(noise, sizeof(noise));

	size_t len;

	// Consume any messages that might accidentally form (unlikely but possible)
	while (parser.getNextMessage(&len) != nullptr) {
		parser.consumeMessage(len);
	}

	// Most data should be discarded
	auto stats = parser.getStats();
	EXPECT_GT(stats.bytes_discarded + stats.crc_errors, 0u);
}

TEST_F(RtcmTest, BustedSender_ValidHeaderBadCrc)
{
	// Frame with valid header but bad CRC
	std::vector<uint8_t> frame = {
		RTCM3_PREAMBLE, 0x00, 0x04,  // Header: 4 byte payload
		0x01, 0x02, 0x03, 0x04,      // Payload
		0xDE, 0xAD, 0xBE             // Bad CRC
	};

	parser.addData(frame.data(), frame.size());

	size_t len;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(parser.getStats().crc_errors, 1u);
}

TEST_F(RtcmTest, BustedSender_RepeatedBadCrcFrames)
{
	for (int i = 0; i < 100; i++) {
		auto bad = buildBadCrcFrame({static_cast<uint8_t>(i), 0x00});
		parser.addData(bad.data(), bad.size());
	}

	size_t len;

	while (parser.getNextMessage(&len) != nullptr) {
		parser.consumeMessage(len);
	}

	auto stats = parser.getStats();
	EXPECT_EQ(stats.messages_parsed, 0u);
	// At least 100 CRC errors (may be more due to re-scanning after failures)
	EXPECT_GE(stats.crc_errors, 100u);
}

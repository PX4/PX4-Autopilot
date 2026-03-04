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

/**
 * @file RtcmParserTest.cpp
 *
 * Tests for RTCM3 parser with valid and partially valid input.
 * Consolidates: GoodSender, PartialSender, Helper, Buffer, Stats, EdgeCase tests.
 */

#include "RtcmTestCommon.hpp"

// =============================================================================
// Helper function tests
// =============================================================================

TEST_F(RtcmTest, PayloadLength_MasksReservedBits)
{
	// Reserved bits set to 1, length should still be extracted correctly
	uint8_t frame[] = {RTCM3_PREAMBLE, 0xFF, 0xFF};  // Reserved=0x3F, Length=1023
	EXPECT_EQ(rtcm3_payload_length(frame), 1023u);
}

TEST_F(RtcmTest, MessageType_Extraction)
{
	auto frame = buildValidFrame(1005, {});
	EXPECT_EQ(rtcm3_message_type(frame.data()), 1005u);
}

// =============================================================================
// Good sender tests - valid frames, happy path
// =============================================================================

TEST_F(RtcmTest, SingleMinimalFrame)
{
	auto frame = buildRawFrame({0x00, 0x00});
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	const uint8_t *msg = parser.getNextMessage(&len);

	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(len, frame.size());

	parser.consumeMessage(len);
	auto stats = parser.getStats();
	EXPECT_EQ(stats.messages_parsed, 1u);
	EXPECT_EQ(stats.crc_errors, 0u);
}

TEST_F(RtcmTest, SingleMaximalFrame)
{
	std::vector<uint8_t> payload(RTCM3_MAX_PAYLOAD_LEN, 0xAA);
	auto frame = buildRawFrame(payload);
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	const uint8_t *msg = parser.getNextMessage(&len);

	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(len, RTCM3_MAX_FRAME_LEN);
}

TEST_F(RtcmTest, MultipleFramesInSequence)
{
	auto frame1 = buildValidFrame(1005, {0x01, 0x02, 0x03});
	auto frame2 = buildValidFrame(1077, {0x04, 0x05, 0x06, 0x07});
	auto frame3 = buildValidFrame(1087, {0x08});

	parser.addData(frame1.data(), frame1.size());
	parser.addData(frame2.data(), frame2.size());
	parser.addData(frame3.data(), frame3.size());

	size_t len;
	int count = 0;

	while (parser.getNextMessage(&len) != nullptr) {
		parser.consumeMessage(len);
		count++;
	}

	EXPECT_EQ(count, 3);
}

TEST_F(RtcmTest, FrameArrivesInChunks)
{
	auto frame = buildValidFrame(1005, {0x01, 0x02, 0x03, 0x04, 0x05});

	// Feed one byte at a time
	for (size_t i = 0; i < frame.size() - 1; i++) {
		parser.addData(&frame[i], 1);
		size_t len;
		EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	}

	parser.addData(&frame[frame.size() - 1], 1);

	size_t len;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(len, frame.size());
}

// =============================================================================
// Partially good sender tests - mix of valid and invalid
// =============================================================================

TEST_F(RtcmTest, GarbageBeforeValidFrame)
{
	std::vector<uint8_t> data;

	// 50 bytes of garbage
	for (int i = 0; i < 50; i++) {
		data.push_back(0x00 + i);
	}

	auto frame = buildValidFrame(1005, {0x01, 0x02});
	data.insert(data.end(), frame.begin(), frame.end());

	parser.addData(data.data(), data.size());

	size_t len;
	const uint8_t *msg = parser.getNextMessage(&len);
	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(rtcm3_message_type(msg), 1005u);

	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().bytes_discarded, 50u);
}

TEST_F(RtcmTest, ValidFrameBadFrameValid)
{
	auto frame1 = buildValidFrame(1005, {0x01});
	auto bad_frame = buildBadCrcFrame({0x02, 0x03});
	auto frame2 = buildValidFrame(1077, {0x04});

	parser.addData(frame1.data(), frame1.size());
	parser.addData(bad_frame.data(), bad_frame.size());
	parser.addData(frame2.data(), frame2.size());

	size_t len;
	int valid_count = 0;

	while (parser.getNextMessage(&len) != nullptr) {
		parser.consumeMessage(len);
		valid_count++;
	}

	EXPECT_EQ(valid_count, 2);
	EXPECT_EQ(parser.getStats().crc_errors, 1u);
}

// =============================================================================
// Buffer and edge case tests
// =============================================================================

TEST_F(RtcmTest, BufferOverflowProtection)
{
	std::vector<uint8_t> large_data(Rtcm3Parser::BUFFER_SIZE + 1000, 0xAA);
	size_t added = parser.addData(large_data.data(), large_data.size());

	EXPECT_EQ(added, Rtcm3Parser::BUFFER_SIZE);
}

TEST_F(RtcmTest, EmptyPayloadFrame)
{
	auto frame = buildRawFrame({});
	parser.addData(frame.data(), frame.size());

	size_t len;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(len, RTCM3_HEADER_LEN + RTCM3_CRC_LEN);
}

TEST_F(RtcmTest, GetNextOnEmptyBuffer)
{
	size_t len;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
}

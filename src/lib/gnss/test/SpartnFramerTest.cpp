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

/**
 * @file SpartnFramerTest.cpp
 *
 * SPARTN framing tests for CorrectionFramer, including the RTCM3/SPARTN
 * cross-protocol resync behaviour that motivates sharing one buffer.
 */

#include <gtest/gtest.h>
#include "correction_framer.h"

#include <vector>
#include <cstring>
#include <random>

using namespace gnss;

class SpartnTest : public ::testing::Test
{
protected:
	CorrectionFramer parser;

	// Build unencrypted frame with 16-bit time tag and given CRC type.
	std::vector<uint8_t> buildFrame(uint8_t msg_type, uint8_t msg_subtype,
					const std::vector<uint8_t> &payload,
					uint8_t crc_type = 0)
	{
		const size_t payload_len = payload.size();
		std::vector<uint8_t> frame;
		frame.push_back(SPARTN_PREAMBLE);

		// TF002-TF006 (frame CRC nibble left 0; not checked by the framer)
		uint8_t b1 = static_cast<uint8_t>(((msg_type & 0x7F) << 1) | ((payload_len >> 9) & 0x01));
		uint8_t b2 = static_cast<uint8_t>((payload_len >> 1) & 0xFF);
		uint8_t b3 = static_cast<uint8_t>(((payload_len & 0x01) << 7) | ((crc_type & 0x03) << 4));
		frame.push_back(b1);
		frame.push_back(b2);
		frame.push_back(b3);

		// 4-byte description, 16-bit time tag
		frame.push_back(static_cast<uint8_t>((msg_subtype & 0x0F) << 4));
		frame.push_back(0x00);
		frame.push_back(0x00);
		frame.push_back(0x00);

		frame.insert(frame.end(), payload.begin(), payload.end());

		// Message CRC over TF002..end of payload
		const uint32_t crc = spartn_message_crc(&frame[1], frame.size() - 1, crc_type);
		const size_t crc_bytes = static_cast<size_t>(crc_type) + 1;

		for (int i = static_cast<int>(crc_bytes) - 1; i >= 0; i--) {
			frame.push_back(static_cast<uint8_t>((crc >>(8 * i)) & 0xFF));
		}

		return frame;
	}

	// Build a well-formed RTCM3 frame with a deterministic pseudo-random payload.
	std::vector<uint8_t> buildRtcmFrame(std::mt19937 &rng, size_t payload_len)
	{
		std::vector<uint8_t> frame;
		frame.push_back(RTCM3_PREAMBLE);
		frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0x03));
		frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));

		std::uniform_int_distribution<int> byte(0, 255);

		for (size_t i = 0; i < payload_len; i++) {
			frame.push_back(static_cast<uint8_t>(byte(rng)));
		}

		const uint32_t crc = rtcm3_crc24q(frame.data(), frame.size());
		frame.push_back((crc >> 16) & 0xFF);
		frame.push_back((crc >> 8) & 0xFF);
		frame.push_back(crc & 0xFF);
		return frame;
	}
};

// =============================================================================
// Message CRC (TF018)
// =============================================================================

// Vectors taken from the pyspartn reference implementation's test suite
// (semuconsulting/pyspartn, tests/test_static.py), which computes the same
// TF018 CRCs independently of this code.
TEST_F(SpartnTest, MessageCrcMatchesReferenceVectors)
{
	const uint8_t msg[] = {'H', 'i', '!'};

	EXPECT_EQ(spartn_message_crc(msg, sizeof(msg), 0), 0x78u);
	EXPECT_EQ(spartn_message_crc(msg, sizeof(msg), 1), 0x31FDu);
	EXPECT_EQ(spartn_message_crc(msg, sizeof(msg), 2), 0x33220Fu);
	EXPECT_EQ(spartn_message_crc(msg, sizeof(msg), 3), 0x9523B4B4u);
}

// =============================================================================
// Framing
// =============================================================================

TEST_F(SpartnTest, SingleMinimalFrame)
{
	auto frame = buildFrame(0, 0, {0xAA, 0xBB});
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;
	const uint8_t *msg = parser.getNextMessage(&len, &protocol);
	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(protocol, CorrectionProtocol::Spartn);
	EXPECT_EQ(len, frame.size());
	EXPECT_EQ(0, memcmp(msg, frame.data(), frame.size()));
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().messages_parsed, 1u);
	EXPECT_EQ(parser.getStats().spartn_messages, 1u);
	EXPECT_EQ(parser.getStats().crc_errors, 0u);
}

TEST_F(SpartnTest, MultipleFrames)
{
	auto f1 = buildFrame(0, 0, {0x01});
	auto f2 = buildFrame(1, 0, {0x02, 0x03});
	std::vector<uint8_t> stream;
	stream.insert(stream.end(), f1.begin(), f1.end());
	stream.insert(stream.end(), f2.begin(), f2.end());
	parser.addData(stream.data(), stream.size());

	int count = 0;
	size_t len = 0;

	while (parser.getNextMessage(&len) != nullptr) {
		parser.consumeMessage(len);
		count++;
	}

	EXPECT_EQ(count, 2);
	EXPECT_EQ(parser.getStats().spartn_messages, 2u);
}

TEST_F(SpartnTest, ChunkedArrival)
{
	auto frame = buildFrame(0, 1, {0x10, 0x20, 0x30});

	for (size_t i = 0; i < frame.size(); i++) {
		parser.addData(&frame[i], 1);
	}

	size_t len = 0;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(len, frame.size());
}

TEST_F(SpartnTest, NoiseBeforeFrame)
{
	auto frame = buildFrame(0, 0, {0x55});
	std::vector<uint8_t> stream = {0x00, 0xFF, 0x01};
	stream.insert(stream.end(), frame.begin(), frame.end());
	parser.addData(stream.data(), stream.size());

	size_t len = 0;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_GT(parser.getStats().bytes_discarded, 0u);
}

TEST_F(SpartnTest, BadMessageCrc)
{
	auto frame = buildFrame(0, 0, {0x11, 0x22});
	frame.back() ^= 0xFF;
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	EXPECT_GE(parser.getStats().crc_errors, 1u);
}

TEST_F(SpartnTest, Crc16Frame)
{
	auto frame = buildFrame(0, 0, {0xDE, 0xAD}, 1);
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().crc_errors, 0u);
}

// TF002 5-119 is reserved by SPARTN 2.x, so a frame claiming one cannot be real
// even if its message CRC checks out.
TEST_F(SpartnTest, RejectsReservedMessageType)
{
	EXPECT_TRUE(spartn_message_type_defined(0));
	EXPECT_TRUE(spartn_message_type_defined(4));
	EXPECT_FALSE(spartn_message_type_defined(5));
	EXPECT_FALSE(spartn_message_type_defined(119));
	EXPECT_TRUE(spartn_message_type_defined(120));
	EXPECT_TRUE(spartn_message_type_defined(127));

	auto frame = buildFrame(50, 0, {0x11, 0x22});
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(parser.getStats().spartn_messages, 0u);
}

// =============================================================================
// Cross-protocol resync
// =============================================================================

// An RTCM3 payload is dense binary, so it carries stray 0x73 SPARTN preambles at
// roughly 1 byte in 256. Framing both protocols from one buffer means a valid
// RTCM3 frame consumes its own payload, so those bytes never start a frame. An
// independent SPARTN framer over the same bytes would instead scan inside RTCM3
// payloads, where the only remaining check is the message CRC that TF005 permits
// to be 8 bits wide.
TEST_F(SpartnTest, RtcmStreamIsNeverFramedAsSpartn)
{
	std::mt19937 rng(1);
	std::uniform_int_distribution<int> payload_len(60, 500);

	size_t frames_in = 0;
	size_t frames_out = 0;
	size_t generated = 0;
	std::vector<uint8_t> stream;

	// ~1 MB of RTCM3, about 17 min of a 1 kB/s RTK correction stream
	while (generated < 1000000) {
		auto frame = buildRtcmFrame(rng, payload_len(rng));
		generated += frame.size();
		frames_in++;
		stream.insert(stream.end(), frame.begin(), frame.end());

		// Feed in MAVLink GPS_RTCM_DATA sized chunks
		while (stream.size() >= 180) {
			parser.addData(stream.data(), 180);

			size_t len = 0;
			CorrectionProtocol protocol = CorrectionProtocol::Spartn;

			while (parser.getNextMessage(&len, &protocol) != nullptr) {
				EXPECT_EQ(protocol, CorrectionProtocol::Rtcm3);
				parser.consumeMessage(len);
				frames_out++;
			}

			stream.erase(stream.begin(), stream.begin() + 180);
		}
	}

	// Every RTCM3 frame is recovered, and no part of the stream is framed as SPARTN
	EXPECT_EQ(parser.getStats().spartn_messages, 0u);
	EXPECT_EQ(parser.getStats().rtcm3_messages, frames_out);
	EXPECT_GE(frames_out, frames_in - 1);
	EXPECT_EQ(parser.getStats().crc_errors, 0u);
}

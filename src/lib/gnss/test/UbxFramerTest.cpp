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
 * @file UbxFramerTest.cpp
 *
 * UBX framing tests for CorrectionFramer (AssistNow MGA pass-through), including
 * interleaved UBX then RTCM3/SPARTN streams that mirror PointPerfect -MGA
 * mountpoints.
 */

#include <gtest/gtest.h>
#include "correction_framer.h"

#include <vector>
#include <cstring>
#include <random>

using namespace gnss;

class UbxTest : public ::testing::Test
{
protected:
	CorrectionFramer parser;

	// Build a valid UBX frame (class/id arbitrary; content not decoded).
	std::vector<uint8_t> buildUbxFrame(uint8_t msg_class, uint8_t msg_id,
					   const std::vector<uint8_t> &payload)
	{
		const size_t payload_len = payload.size();
		std::vector<uint8_t> frame;
		frame.push_back(UBX_SYNC1);
		frame.push_back(UBX_SYNC2);
		frame.push_back(msg_class);
		frame.push_back(msg_id);
		frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));
		frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
		frame.insert(frame.end(), payload.begin(), payload.end());

		uint8_t ck_a = 0;
		uint8_t ck_b = 0;
		ubx_checksum(&frame[2], UBX_HEADER_LEN - 2 + payload_len, &ck_a, &ck_b);
		frame.push_back(ck_a);
		frame.push_back(ck_b);
		return frame;
	}

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

#if defined(CONFIG_GPS_SPARTN)
	std::vector<uint8_t> buildSpartnFrame(uint8_t msg_type, const std::vector<uint8_t> &payload,
					      uint8_t crc_type = 0)
	{
		const size_t payload_len = payload.size();
		std::vector<uint8_t> frame;
		frame.push_back(SPARTN_PREAMBLE);

		uint8_t b1 = static_cast<uint8_t>(((msg_type & 0x7F) << 1) | ((payload_len >> 9) & 0x01));
		uint8_t b2 = static_cast<uint8_t>((payload_len >> 1) & 0xFF);
		uint8_t b3 = static_cast<uint8_t>(((payload_len & 0x01) << 7) | ((crc_type & 0x03) << 4));
		frame.push_back(b1);
		frame.push_back(b2);
		frame.push_back(b3);

		// 4-byte description, 16-bit time tag
		frame.push_back(0x00);
		frame.push_back(0x00);
		frame.push_back(0x00);
		frame.push_back(0x00);

		frame.insert(frame.end(), payload.begin(), payload.end());

		const uint32_t crc = spartn_message_crc(&frame[1], frame.size() - 1, crc_type);
		const size_t crc_bytes = static_cast<size_t>(crc_type) + 1;

		for (int i = static_cast<int>(crc_bytes) - 1; i >= 0; i--) {
			frame.push_back(static_cast<uint8_t>((crc >>(8 * i)) & 0xFF));
		}

		return frame;
	}
#endif
};

// =============================================================================
// Checksum
// =============================================================================

// Empty ACK-ACK (class=0x05, id=0x01, length=0): covered bytes 05 01 00 00.
// 8-bit Fletcher: a runs 5,6,6,6 and b runs 5,11,17,23 → CK_A=0x06, CK_B=0x17.
TEST_F(UbxTest, ChecksumKnownVector)
{
	const uint8_t covered[] = {0x05, 0x01, 0x00, 0x00};
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	ubx_checksum(covered, sizeof(covered), &ck_a, &ck_b);
	EXPECT_EQ(ck_a, 0x06u);
	EXPECT_EQ(ck_b, 0x17u);
}

// =============================================================================
// Framing
// =============================================================================

TEST_F(UbxTest, SingleMinimalFrame)
{
	// UBX-MGA class is 0x13; any class/id is fine for framing
	auto frame = buildUbxFrame(0x13, 0x00, {0xAA, 0xBB});
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;
	const uint8_t *msg = parser.getNextMessage(&len, &protocol);
	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(protocol, CorrectionProtocol::Ubx);
	EXPECT_EQ(len, frame.size());
	EXPECT_EQ(0, memcmp(msg, frame.data(), frame.size()));
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().messages_parsed, 1u);
	EXPECT_EQ(parser.getStats().ubx_messages, 1u);
	EXPECT_EQ(parser.getStats().crc_errors, 0u);
}

TEST_F(UbxTest, EmptyPayloadFrame)
{
	auto frame = buildUbxFrame(0x05, 0x01, {});
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(len, UBX_HEADER_LEN + UBX_CK_LEN);
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().ubx_messages, 1u);
}

TEST_F(UbxTest, MultipleFrames)
{
	auto f1 = buildUbxFrame(0x13, 0x00, {0x01});
	auto f2 = buildUbxFrame(0x13, 0x02, {0x02, 0x03, 0x04});
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
	EXPECT_EQ(parser.getStats().ubx_messages, 2u);
}

TEST_F(UbxTest, ChunkedArrival)
{
	auto frame = buildUbxFrame(0x13, 0x01, {0x10, 0x20, 0x30, 0x40});

	for (size_t i = 0; i < frame.size(); i++) {
		parser.addData(&frame[i], 1);
		// No complete frame until the last byte
	}

	size_t len = 0;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(len, frame.size());
}

TEST_F(UbxTest, NoiseBeforeFrame)
{
	auto frame = buildUbxFrame(0x13, 0x00, {0x55});
	std::vector<uint8_t> stream = {0x00, 0xFF, 0x01};
	stream.insert(stream.end(), frame.begin(), frame.end());
	parser.addData(stream.data(), stream.size());

	size_t len = 0;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_GT(parser.getStats().bytes_discarded, 0u);
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().ubx_messages, 1u);
}

TEST_F(UbxTest, BadSync2)
{
	// 0xB5 followed by something other than 0x62 is not a frame start
	std::vector<uint8_t> stream = {UBX_SYNC1, 0x00, 0x11, 0x22};
	auto frame = buildUbxFrame(0x13, 0x00, {0x01});
	stream.insert(stream.end(), frame.begin(), frame.end());
	parser.addData(stream.data(), stream.size());

	size_t len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;
	ASSERT_NE(parser.getNextMessage(&len, &protocol), nullptr);
	EXPECT_EQ(protocol, CorrectionProtocol::Ubx);
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().ubx_messages, 1u);
	EXPECT_GT(parser.getStats().bytes_discarded, 0u);
}

TEST_F(UbxTest, BadChecksum)
{
	auto frame = buildUbxFrame(0x13, 0x00, {0x11, 0x22});
	frame.back() ^= 0xFF;
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	EXPECT_GE(parser.getStats().crc_errors, 1u);
	EXPECT_EQ(parser.getStats().ubx_messages, 0u);
}

TEST_F(UbxTest, RejectsOversizedPayload)
{
	// Length field claims UBX_MAX_PAYLOAD_LEN + 1 — must not pin the buffer
	std::vector<uint8_t> header = {
		UBX_SYNC1, UBX_SYNC2,
		0x13, 0x00,
		static_cast<uint8_t>((UBX_MAX_PAYLOAD_LEN + 1) & 0xFF),
		static_cast<uint8_t>(((UBX_MAX_PAYLOAD_LEN + 1) >> 8) & 0xFF),
	};
	auto good = buildUbxFrame(0x13, 0x00, {0xAB});
	header.insert(header.end(), good.begin(), good.end());
	parser.addData(header.data(), header.size());

	size_t len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;
	ASSERT_NE(parser.getNextMessage(&len, &protocol), nullptr);
	EXPECT_EQ(protocol, CorrectionProtocol::Ubx);
	EXPECT_EQ(len, good.size());
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().ubx_messages, 1u);
}

// =============================================================================
// Cross-protocol: MGA then corrections (the PointPerfect -MGA case)
// =============================================================================

// PointPerfect -MGA mountpoints deliver a UBX-MGA burst once on connect, then
// the normal correction stream (RTCM3 or SPARTN). The framer must not lock onto
// UBX and reject everything after — each complete frame is consumed and the next
// preamble (of any protocol) starts the next frame.
TEST_F(UbxTest, MgaThenRtcm3)
{
	std::mt19937 rng(42);
	auto mga1 = buildUbxFrame(0x13, 0x00, {0x01, 0x02, 0x03});
	auto mga2 = buildUbxFrame(0x13, 0x02, {0x10, 0x20});
	auto rtcm = buildRtcmFrame(rng, 40);

	std::vector<uint8_t> stream;
	stream.insert(stream.end(), mga1.begin(), mga1.end());
	stream.insert(stream.end(), mga2.begin(), mga2.end());
	stream.insert(stream.end(), rtcm.begin(), rtcm.end());
	parser.addData(stream.data(), stream.size());

	std::vector<CorrectionProtocol> seen;
	size_t len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;

	while (parser.getNextMessage(&len, &protocol) != nullptr) {
		seen.push_back(protocol);
		parser.consumeMessage(len);
	}

	ASSERT_EQ(seen.size(), 3u);
	EXPECT_EQ(seen[0], CorrectionProtocol::Ubx);
	EXPECT_EQ(seen[1], CorrectionProtocol::Ubx);
	EXPECT_EQ(seen[2], CorrectionProtocol::Rtcm3);
	EXPECT_EQ(parser.getStats().ubx_messages, 2u);
	EXPECT_EQ(parser.getStats().rtcm3_messages, 1u);
	EXPECT_EQ(parser.getStats().crc_errors, 0u);
}

#if defined(CONFIG_GPS_SPARTN)
TEST_F(UbxTest, MgaThenSpartn)
{
	auto mga = buildUbxFrame(0x13, 0x00, {0xDE, 0xAD});
	auto spartn = buildSpartnFrame(0, {0xAA, 0xBB});

	std::vector<uint8_t> stream;
	stream.insert(stream.end(), mga.begin(), mga.end());
	stream.insert(stream.end(), spartn.begin(), spartn.end());
	// Another MGA mid-stream should also work (reconnect-style)
	auto mga2 = buildUbxFrame(0x13, 0x01, {0x01});
	stream.insert(stream.end(), mga2.begin(), mga2.end());
	parser.addData(stream.data(), stream.size());

	std::vector<CorrectionProtocol> seen;
	size_t len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;

	while (parser.getNextMessage(&len, &protocol) != nullptr) {
		seen.push_back(protocol);
		parser.consumeMessage(len);
	}

	ASSERT_EQ(seen.size(), 3u);
	EXPECT_EQ(seen[0], CorrectionProtocol::Ubx);
	EXPECT_EQ(seen[1], CorrectionProtocol::Spartn);
	EXPECT_EQ(seen[2], CorrectionProtocol::Ubx);
	EXPECT_EQ(parser.getStats().ubx_messages, 2u);
	EXPECT_EQ(parser.getStats().spartn_messages, 1u);
}
#endif

// A complete RTCM3 frame consumes its payload, so a 0xB5 that happens to sit
// inside that payload is never considered as a UBX start.
TEST_F(UbxTest, RtcmPayloadStrayB5IsNotFramedAsUbx)
{
	// Craft an RTCM3 frame whose payload contains a UBX-looking prefix.
	// buildRtcmFrame randomizes payload; force a 0xB5 0x62 sequence in raw form.
	std::vector<uint8_t> payload(32, 0x00);
	payload[4] = UBX_SYNC1;
	payload[5] = UBX_SYNC2;
	payload[6] = 0x13;
	payload[7] = 0x00;
	payload[8] = 0x02; // fake length
	payload[9] = 0x00;

	std::vector<uint8_t> frame;
	frame.push_back(RTCM3_PREAMBLE);
	frame.push_back(0x00);
	frame.push_back(static_cast<uint8_t>(payload.size()));
	frame.insert(frame.end(), payload.begin(), payload.end());
	const uint32_t crc = rtcm3_crc24q(frame.data(), frame.size());
	frame.push_back((crc >> 16) & 0xFF);
	frame.push_back((crc >> 8) & 0xFF);
	frame.push_back(crc & 0xFF);

	// Follow with a real UBX so we know the framer still works after
	auto ubx = buildUbxFrame(0x13, 0x00, {0x99});
	std::vector<uint8_t> stream = frame;
	stream.insert(stream.end(), ubx.begin(), ubx.end());
	parser.addData(stream.data(), stream.size());

	size_t len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Ubx;
	ASSERT_NE(parser.getNextMessage(&len, &protocol), nullptr);
	EXPECT_EQ(protocol, CorrectionProtocol::Rtcm3);
	parser.consumeMessage(len);

	ASSERT_NE(parser.getNextMessage(&len, &protocol), nullptr);
	EXPECT_EQ(protocol, CorrectionProtocol::Ubx);
	parser.consumeMessage(len);

	EXPECT_EQ(parser.getStats().rtcm3_messages, 1u);
	EXPECT_EQ(parser.getStats().ubx_messages, 1u);
	EXPECT_EQ(parser.getStats().crc_errors, 0u);
}

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
#include "spartn.h"

#include <vector>
#include <cstring>

using namespace gnss;

class SpartnTest : public ::testing::Test
{
protected:
	SpartnParser parser;

	// Build unencrypted frame with 16-bit time tag and given CRC type.
	std::vector<uint8_t> buildFrame(uint8_t msg_type, uint8_t msg_subtype,
					const std::vector<uint8_t> &payload,
					uint8_t crc_type = 0)
	{
		const size_t payload_len = payload.size();
		std::vector<uint8_t> frame;
		frame.push_back(SPARTN_PREAMBLE);

		// TF002–TF006 (frame CRC nibble left 0; not checked by parser)
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

		// Message CRC over TF002..end of payload (same poly as SpartnParser)
		const unsigned n = 8u * (crc_type + 1u);
		const uint32_t poly = (crc_type == 0) ? 0x07u :
				      (crc_type == 1) ? 0x1021u :
				      (crc_type == 2) ? 0x864CFBu : 0x04C11DB7u;
		const uint32_t top = 1u << n;
		const uint32_t g = top | poly;
		uint32_t c = (crc_type == 3) ? 0xFFFFFFFFu : 0u;

		for (size_t i = 1; i < frame.size(); i++) {
			c ^= static_cast<uint32_t>(frame[i]) << (n - 8);

			for (int b = 0; b < 8; b++) {
				c <<= 1;

				if (c & top) {
					c ^= g;
				}
			}
		}

		if (crc_type == 3) {
			c ^= 0xFFFFFFFFu;
		}

		c &= (top - 1);
		const size_t crc_bytes = static_cast<size_t>(crc_type) + 1;

		for (int i = static_cast<int>(crc_bytes) - 1; i >= 0; i--) {
			frame.push_back(static_cast<uint8_t>((c >>(8 * i)) & 0xFF));
		}

		return frame;
	}
};

TEST_F(SpartnTest, SingleMinimalFrame)
{
	auto frame = buildFrame(0, 0, {0xAA, 0xBB});
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	const uint8_t *msg = parser.getNextMessage(&len);
	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(len, frame.size());
	EXPECT_EQ(0, memcmp(msg, frame.data(), frame.size()));
	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().messages_parsed, 1u);
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

TEST_F(SpartnTest, DoesNotAcceptRtcmPreamble)
{
	const uint8_t noise[] = {0xD3, 0x00, 0x10, 0x01, 0x02};
	parser.addData(noise, sizeof(noise));
	size_t len = 0;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
}

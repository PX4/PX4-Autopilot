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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "Crc8.hpp"
#include "CrsfParser.hpp"

#include <cstdint>
#include <vector>

namespace
{

constexpr uint8_t CRSF_HEADER_VALUE = 0xC8;
constexpr uint8_t CRSF_CHANNEL_TYPE = 0x16;
constexpr uint8_t CRSF_UNSUPPORTED_TYPE = 0xA8;
constexpr size_t CRSF_CHANNEL_PAYLOAD_SIZE = 22;

std::vector<uint8_t> makeFrame(uint8_t type, const std::vector<uint8_t> &payload)
{
	const uint8_t length = static_cast<uint8_t>(payload.size() + 2);
	std::vector<uint8_t> frame{CRSF_HEADER_VALUE, length, type};
	frame.insert(frame.end(), payload.begin(), payload.end());
	frame.push_back(Crc8Calc(frame.data() + 2, length - 1));
	return frame;
}

class CrsfParserTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		Crc8Init(0xD5);
		CrsfParser_Init();
	}
};

TEST_F(CrsfParserTest, UnsupportedTypeIsIgnoredAndParserRecovers)
{
	const uint32_t empty_queue_size = CrsfParser_FreeQueueSize();
	const std::vector<uint8_t> unsupported_frame = makeFrame(CRSF_UNSUPPORTED_TYPE, {});
	ASSERT_TRUE(CrsfParser_LoadBuffer(unsupported_frame.data(), unsupported_frame.size()));

	CrsfPacket_t packet{};
	CrsfParserStatistics_t statistics{};
	EXPECT_FALSE(CrsfParser_TryParseCrsfPacket(&packet, &statistics));
	EXPECT_EQ(statistics.crcs_valid_unknown_packets, 1u);
	EXPECT_EQ(CrsfParser_FreeQueueSize(), empty_queue_size);

	const std::vector<uint8_t> channel_frame = makeFrame(CRSF_CHANNEL_TYPE,
			std::vector<uint8_t>(CRSF_CHANNEL_PAYLOAD_SIZE));
	ASSERT_TRUE(CrsfParser_LoadBuffer(channel_frame.data(), channel_frame.size()));
	ASSERT_TRUE(CrsfParser_TryParseCrsfPacket(&packet, &statistics));
	EXPECT_EQ(packet.message_type, CRSF_MESSAGE_TYPE_RC_CHANNELS);
	EXPECT_EQ(CrsfParser_FreeQueueSize(), empty_queue_size);
}

} // namespace

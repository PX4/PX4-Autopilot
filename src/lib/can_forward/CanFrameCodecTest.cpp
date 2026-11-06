/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * Test matrix rows C1-C5 (see TEST_MATRIX.md)
 */

#include <gtest/gtest.h>

#include "CanFrameCodec.hpp"
#include "CanForwardTestHelpers.hpp"

using namespace can_forward;
using namespace can_forward_test;

static const uint8_t payload[64] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
};

// C1
TEST(CanFrameCodec, ExtendedFramesAllLengths)
{
	for (uint8_t len = 0; len <= 8; len++) {
		Frame frame;
		memset(&frame, 0xAA, sizeof(frame));
		ASSERT_EQ(decodeFrame(messageId(NodeStatus, 42), len, payload, 8, frame), DecodeResult::Ok);
		EXPECT_EQ(frame.id, messageId(NodeStatus, 42));
		EXPECT_EQ(frame.len, len);

		for (uint8_t i = 0; i < 8; i++) {
			EXPECT_EQ(frame.data[i], i < len ? payload[i] : 0) << int(len) << " " << int(i);
		}
	}
}

TEST(CanFrameCodec, MaxExtendedId)
{
	Frame frame{};
	EXPECT_EQ(decodeFrame(Eff | CanMaskExtId, 1, payload, 8, frame), DecodeResult::Ok);
}

// C2
TEST(CanFrameCodec, RejectsLengthAbove8)
{
	Frame frame{};

	for (uint8_t len : {9, 12, 64, 255}) {
		EXPECT_EQ(decodeFrame(messageId(NodeStatus, 42), len, payload, 64, frame), DecodeResult::BadLength) << int(len);
	}

	// len beyond the buffer the message carries
	EXPECT_EQ(decodeFrame(messageId(NodeStatus, 42), 8, payload, 4, frame), DecodeResult::BadLength);
}

// C3 / C4
TEST(CanFrameCodec, StandardFrames)
{
	Frame frame{};
	EXPECT_EQ(decodeFrame(0x000, 0, payload, 8, frame), DecodeResult::Ok);
	EXPECT_EQ(decodeFrame(0x7FF, 2, payload, 8, frame), DecodeResult::Ok);
	EXPECT_EQ(frame.id, 0x7FFu);
	EXPECT_EQ(decodeFrame(0x800, 2, payload, 8, frame), DecodeResult::BadId);
	EXPECT_EQ(decodeFrame(0x1FFFFFFF, 2, payload, 8, frame), DecodeResult::BadId);
}

// C5
TEST(CanFrameCodec, RejectsRemoteAndErrorFrames)
{
	Frame frame{};
	EXPECT_EQ(decodeFrame(messageId(NodeStatus, 42) | CanFlagRtr, 0, payload, 8, frame), DecodeResult::BadFlags);
	EXPECT_EQ(decodeFrame(messageId(NodeStatus, 42) | CanFlagErr, 0, payload, 8, frame), DecodeResult::BadFlags);
	EXPECT_EQ(decodeFrame(0x123 | CanFlagRtr, 0, payload, 8, frame), DecodeResult::BadFlags);
	EXPECT_EQ(decodeFrame(0x123 | CanFlagErr, 0, payload, 8, frame), DecodeResult::BadFlags);
}

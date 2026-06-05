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
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "../sony_asdt1.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

class ASDT1Test : public ::testing::Test
{
protected:
	using Frame = std::array<uint8_t, AS_DT1::ASDT1_RAW_FRAME_SIZE>;

	static Frame emptyFrame()
	{
		Frame frame{};
		return frame;
	}

	static uint32_t encode20BitMillimeters(float value_mm)
	{
		int32_t raw = static_cast<int32_t>(roundf(value_mm * 4.0f));

		if (raw < 0) {
			raw = (1 << 20) + raw;
		}

		return static_cast<uint32_t>(raw) & 0x000fffff;
	}

	static void packTwoPoints(Frame &frame, size_t pair_index,
				  float x1_mm, float y1_mm, float z1_mm,
				  float x2_mm, float y2_mm, float z2_mm)
	{
		const size_t offset = pair_index * 15;
		ASSERT_LE(offset + 15, frame.size());

		const uint32_t x1 = encode20BitMillimeters(x1_mm);
		const uint32_t y1 = encode20BitMillimeters(y1_mm);
		const uint32_t z1 = encode20BitMillimeters(z1_mm);
		const uint32_t x2 = encode20BitMillimeters(x2_mm);
		const uint32_t y2 = encode20BitMillimeters(y2_mm);
		const uint32_t z2 = encode20BitMillimeters(z2_mm);

		frame[offset + 0] = static_cast<uint8_t>(x1 >> 12);
		frame[offset + 1] = static_cast<uint8_t>(x1 >> 4);
		frame[offset + 2] = static_cast<uint8_t>(((x1 & 0x0f) << 4) | (y1 >> 16));
		frame[offset + 3] = static_cast<uint8_t>(y1 >> 8);
		frame[offset + 4] = static_cast<uint8_t>(y1);
		frame[offset + 5] = static_cast<uint8_t>(z1 >> 12);
		frame[offset + 6] = static_cast<uint8_t>(z1 >> 4);
		frame[offset + 7] = static_cast<uint8_t>(((z1 & 0x0f) << 4) | (x2 >> 16));
		frame[offset + 8] = static_cast<uint8_t>(x2 >> 8);
		frame[offset + 9] = static_cast<uint8_t>(x2);
		frame[offset + 10] = static_cast<uint8_t>(y2 >> 12);
		frame[offset + 11] = static_cast<uint8_t>(y2 >> 4);
		frame[offset + 12] = static_cast<uint8_t>(((y2 & 0x0f) << 4) | (z2 >> 16));
		frame[offset + 13] = static_cast<uint8_t>(z2 >> 8);
		frame[offset + 14] = static_cast<uint8_t>(z2);
	}

	static bool feedFrame(AS_DT1 &driver, const Frame &frame)
	{
		constexpr char begin_marker[] = "BEGIN MP\r\n";
		constexpr char end_marker[] = "END";
		bool completed = false;

		if (completed == true) {
			completed = false;
		}

		for (size_t i = 0; i < sizeof(begin_marker) - 1; i++) {
			completed = driver.parse_byte(static_cast<uint8_t>(begin_marker[i]));
		}

		for (uint8_t byte : frame) {
			completed = driver.parse_byte(byte);
		}

		for (size_t i = 0; i < sizeof(end_marker) - 1; i++) {
			completed = driver.parse_byte(static_cast<uint8_t>(end_marker[i]));
		}

		return completed;
	}

	static bool allBinsUnknown(const AS_DT1 &driver)
	{
		for (uint8_t i = 0; i < AS_DT1::BIN_COUNT; i++) {
			if (driver._obstacle_distance.distances[i] != UINT16_MAX) {
				return false;
			}
		}

		return true;
	}
};

TEST_F(ASDT1Test, RejectsBadFrameSize)
{
	AS_DT1 driver("/dev/ttyS0");
	const uint8_t frame[1] {};

	EXPECT_EQ(driver.process_frame(frame, sizeof(frame)), PX4_ERROR);
}

TEST_F(ASDT1Test, ZeroFrameProducesNoObstacles)
{
	AS_DT1 driver("/dev/ttyS0");
	const Frame frame = emptyFrame();

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_TRUE(allBinsUnknown(driver));
}

TEST_F(ASDT1Test, SinglePointStraightForwardFillsForwardBin)
{
	AS_DT1 driver("/dev/ttyS0");
	Frame frame = emptyFrame();

	packTwoPoints(frame, 0, 0.0f, 0.0f, 1000.0f, 0.0f, 0.0f, 0.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[0], 100);
}

TEST_F(ASDT1Test, SinglePointRightFillsRightBin)
{
	AS_DT1 driver("/dev/ttyS0");
	Frame frame = emptyFrame();

	packTwoPoints(frame, 0, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[9], 141);
}

TEST_F(ASDT1Test, SinglePointLeftWrapsToLastQuadrant)
{
	AS_DT1 driver("/dev/ttyS0");
	Frame frame = emptyFrame();

	packTwoPoints(frame, 0, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[63], 141);
}

TEST_F(ASDT1Test, ClosestPointWinsWhenTwoPointsShareBin)
{
	AS_DT1 driver("/dev/ttyS0");
	Frame frame = emptyFrame();

	packTwoPoints(frame, 0, 0.0f, 0.0f, 2000.0f, 0.0f, 0.0f, 1000.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[0], 100);
}

TEST_F(ASDT1Test, ParserResyncsAfterNoiseBeforeBegin)
{
	AS_DT1 driver("/dev/ttyS0");
	const Frame frame = emptyFrame();
	const uint8_t noise[] = {'x', 'B', 'x', '\r', '\n'};

	for (uint8_t byte : noise) {
		EXPECT_FALSE(driver.parse_byte(byte));
	}

	EXPECT_TRUE(feedFrame(driver, frame));
	EXPECT_TRUE(driver._have_latest_frame);
	EXPECT_EQ(driver._latest_frame_len, frame.size());
	EXPECT_EQ(memcmp(driver._latest_frame, frame.data(), frame.size()), 0);
}

TEST_F(ASDT1Test, ParserAcceptsPartialFrameAcrossCalls)
{
	AS_DT1 driver("/dev/ttyS0");
	const Frame frame = emptyFrame();
	constexpr char begin_marker[] = "BEGIN MP\r\n";
	constexpr char end_marker[] = "END";

	for (size_t i = 0; i < sizeof(begin_marker) - 1; i++) {
		EXPECT_FALSE(driver.parse_byte(static_cast<uint8_t>(begin_marker[i])));
	}

	for (size_t i = 0; i < frame.size() / 2; i++) {
		EXPECT_FALSE(driver.parse_byte(frame[i]));
	}

	for (size_t i = frame.size() / 2; i < frame.size(); i++) {
		EXPECT_FALSE(driver.parse_byte(frame[i]));
	}

	EXPECT_FALSE(driver.parse_byte(static_cast<uint8_t>(end_marker[0])));
	EXPECT_FALSE(driver.parse_byte(static_cast<uint8_t>(end_marker[1])));
	EXPECT_TRUE(driver.parse_byte(static_cast<uint8_t>(end_marker[2])));
	EXPECT_TRUE(driver._have_latest_frame);
}

TEST_F(ASDT1Test, ParserKeepsNewestCompleteFrame)
{
	AS_DT1 driver("/dev/ttyS0");
	Frame first = emptyFrame();
	Frame second = emptyFrame();

	packTwoPoints(first, 0, 0.0f, 0.0f, 2000.0f, 0.0f, 0.0f, 0.0f);
	packTwoPoints(second, 0, 0.0f, 0.0f, 1000.0f, 0.0f, 0.0f, 0.0f);

	ASSERT_TRUE(feedFrame(driver, first));
	ASSERT_TRUE(feedFrame(driver, second));
	ASSERT_TRUE(driver._have_latest_frame);

	ASSERT_EQ(driver.process_frame(driver._latest_frame, driver._latest_frame_len), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[0], 100);
}

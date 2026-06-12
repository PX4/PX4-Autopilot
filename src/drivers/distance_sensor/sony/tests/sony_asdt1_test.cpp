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
	using FullFrame = std::array<uint8_t, AS_DT1::ASDT1_BINZ_FRAME_SIZE>;
	using ShortFrame = std::array<uint8_t, AS_DT1::ASDT1_BINZ_SHORT_FRAME_SIZE>;

	static uint32_t encode20BitMillimeters(float value_mm)
	{
		int32_t raw = static_cast<int32_t>(roundf(value_mm * 4.0f));

		if (raw < 0) {
			raw = (1 << 20) + raw;
		}

		return static_cast<uint32_t>(raw) & 0x000fffff;
	}

	template <typename Frame>
	static void packZ(Frame &frame, size_t sample_index, float z_mm)
	{
		const uint32_t raw = encode20BitMillimeters(z_mm);
		const size_t offset = (sample_index / 2) * 5;
		ASSERT_LT(offset + 4, frame.size());

		if ((sample_index % 2) == 0) {
			frame[offset] = static_cast<uint8_t>(raw >> 12);
			frame[offset + 1] = static_cast<uint8_t>(raw >> 4);
			frame[offset + 2] = static_cast<uint8_t>((frame[offset + 2] & 0x0f) | ((raw & 0x0f) << 4));

		} else {
			frame[offset + 2] = static_cast<uint8_t>((frame[offset + 2] & 0xf0) | (raw >> 16));
			frame[offset + 3] = static_cast<uint8_t>(raw >> 8);
			frame[offset + 4] = static_cast<uint8_t>(raw);
		}
	}

	static size_t sampleForLayout(int row, int col)
	{
		const int target_layout_index = row * AS_DT1::ASDT1_COLS + col;

		for (size_t sample = 0; sample < AS_DT1::ASDT1_MAX_SAMPLE_COUNT; sample++) {
			if (AS_DT1::sample_to_layout_index(sample) == target_layout_index) {
				return sample;
			}
		}

		ADD_FAILURE() << "missing sample for row " << row << ", col " << col;
		return 0;
	}

	template <typename Frame>
	static bool feedFramePayload(AS_DT1 &driver, const Frame &frame)
	{
		constexpr char begin_marker[] = "BEGIN MP\r\n";
		bool completed = false;

		for (size_t i = 0; i < sizeof(begin_marker) - 1; i++) {
			completed = driver.parse_byte(static_cast<uint8_t>(begin_marker[i]));
		}

		for (uint8_t byte : frame) {
			completed = driver.parse_byte(byte);
		}

		return completed;
	}

	static bool feedShortFrame(AS_DT1 &driver, const ShortFrame &frame)
	{
		constexpr char end_marker[] = "END";
		bool completed = feedFramePayload(driver, frame);

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
	const FullFrame frame{};

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_TRUE(allBinsUnknown(driver));
}

TEST_F(ASDT1Test, ProcessFrameFillsColumnBin)
{
	AS_DT1 driver("/dev/ttyS0");
	FullFrame frame{};
	const int row = (AS_DT1::MIN_USED_ROW + AS_DT1::MAX_USED_ROW) / 2;
	const int col = AS_DT1::ASDT1_COLS / 2;
	const int bin = AS_DT1::col_to_obstacle_bin(col);

	packZ(frame, sampleForLayout(row, col), 1000.0f);
	packZ(frame, sampleForLayout(row + 1, col), 1000.0f);
	packZ(frame, sampleForLayout(row + 2, col), 1000.0f);
	packZ(frame, sampleForLayout(row + 3, col), 1000.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[bin], 100);
}

TEST_F(ASDT1Test, PreservesBelowMinimumDistance)
{
	AS_DT1 driver("/dev/ttyS0");
	FullFrame frame{};
	const int row = (AS_DT1::MIN_USED_ROW + AS_DT1::MAX_USED_ROW) / 2;
	const int col = AS_DT1::ASDT1_COLS / 2;
	const int bin = AS_DT1::col_to_obstacle_bin(col);

	packZ(frame, sampleForLayout(row, col), 100.0f);
	packZ(frame, sampleForLayout(row + 1, col), 100.0f);
	packZ(frame, sampleForLayout(row + 2, col), 100.0f);
	packZ(frame, sampleForLayout(row + 3, col), 100.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[bin], 10);
}

TEST_F(ASDT1Test, BeyondMaximumDistanceUsesMaxPlusOne)
{
	AS_DT1 driver("/dev/ttyS0");
	FullFrame frame{};
	const int row = (AS_DT1::MIN_USED_ROW + AS_DT1::MAX_USED_ROW) / 2;
	const int col = AS_DT1::ASDT1_COLS / 2;
	const int bin = AS_DT1::col_to_obstacle_bin(col);

	packZ(frame, sampleForLayout(row, col), 25000.0f);
	packZ(frame, sampleForLayout(row + 1, col), 25000.0f);
	packZ(frame, sampleForLayout(row + 2, col), 25000.0f);
	packZ(frame, sampleForLayout(row + 3, col), 25000.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[bin], driver._obstacle_distance.max_distance + 1);
}

TEST_F(ASDT1Test, ClosestPointWinsBin)
{
	AS_DT1 driver("/dev/ttyS0");
	FullFrame frame{};
	const int row = (AS_DT1::MIN_USED_ROW + AS_DT1::MAX_USED_ROW) / 2;
	const int col = AS_DT1::ASDT1_COLS / 2;
	const int bin = AS_DT1::col_to_obstacle_bin(col);

	packZ(frame, sampleForLayout(row, col), 350.0f);
	packZ(frame, sampleForLayout(row + 1, col), 3000.0f);
	packZ(frame, sampleForLayout(row + 2, col), 3100.0f);
	packZ(frame, sampleForLayout(row + 3, col), 3200.0f);

	ASSERT_EQ(driver.process_frame(frame.data(), frame.size()), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[bin], 35);
}

TEST_F(ASDT1Test, ParserCompletesFullFrameWithoutEndMarker)
{
	AS_DT1 driver("/dev/ttyS0");
	const FullFrame frame{};

	EXPECT_TRUE(feedFramePayload(driver, frame));
	EXPECT_TRUE(driver._have_latest_frame);
	EXPECT_EQ(driver._latest_frame_len, frame.size());
	EXPECT_EQ(memcmp(driver._latest_frame, frame.data(), frame.size()), 0);
}

TEST_F(ASDT1Test, ParserCompletesShortFrameWithEndMarker)
{
	AS_DT1 driver("/dev/ttyS0");
	const ShortFrame frame{};

	EXPECT_TRUE(feedShortFrame(driver, frame));
	EXPECT_TRUE(driver._have_latest_frame);
	EXPECT_EQ(driver._latest_frame_len, frame.size());
	EXPECT_EQ(memcmp(driver._latest_frame, frame.data(), frame.size()), 0);
}

TEST_F(ASDT1Test, ParserResyncsAfterNoiseBeforeBegin)
{
	AS_DT1 driver("/dev/ttyS0");
	const FullFrame frame{};
	const uint8_t noise[] = {'x', 'B', 'x', '\r', '\n'};

	for (uint8_t byte : noise) {
		EXPECT_FALSE(driver.parse_byte(byte));
	}

	EXPECT_TRUE(feedFramePayload(driver, frame));
	EXPECT_TRUE(driver._have_latest_frame);
}

TEST_F(ASDT1Test, ParserKeepsNewestCompleteFrame)
{
	AS_DT1 driver("/dev/ttyS0");
	FullFrame first{};
	FullFrame second{};
	const int row = (AS_DT1::MIN_USED_ROW + AS_DT1::MAX_USED_ROW) / 2;
	const int col = AS_DT1::ASDT1_COLS / 2;
	const int bin = AS_DT1::col_to_obstacle_bin(col);
	const size_t sample = sampleForLayout(row, col);

	packZ(first, sample, 2000.0f);
	packZ(first, sampleForLayout(row + 1, col), 2000.0f);
	packZ(first, sampleForLayout(row + 2, col), 2000.0f);
	packZ(first, sampleForLayout(row + 3, col), 2000.0f);
	packZ(second, sample, 1000.0f);
	packZ(second, sampleForLayout(row + 1, col), 1000.0f);
	packZ(second, sampleForLayout(row + 2, col), 1000.0f);
	packZ(second, sampleForLayout(row + 3, col), 1000.0f);

	ASSERT_TRUE(feedFramePayload(driver, first));
	ASSERT_TRUE(feedFramePayload(driver, second));
	ASSERT_TRUE(driver._have_latest_frame);

	ASSERT_EQ(driver.process_frame(driver._latest_frame, driver._latest_frame_len), PX4_OK);
	EXPECT_EQ(driver._obstacle_distance.distances[bin], 100);
}

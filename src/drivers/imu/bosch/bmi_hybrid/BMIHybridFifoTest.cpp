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

#include "BMIHybridFifo.hpp"

TEST(BMIHybridFifoTest, TaggedConfigAndSkipOverflow)
{
	bmi_hybrid_fifo::Sample bmi_sample {};

	// BMI270 0x48 FIFO Input Config frame must trigger kReconfigure when inertial channels change (bits [3:0]).
	const uint8_t bmi270_acc_range_change[] { 0x48, 0x02, 0x00, 0x00, 0x00 };
	imu::ByteCursor cursor_acc(bmi270_acc_range_change, sizeof(bmi270_acc_range_change));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_acc, bmi_sample), imu::FifoFrame::kReconfigure);

	const uint8_t bmi270_gyr_range_change[] { 0x48, 0x08, 0x00, 0x00, 0x00 };
	imu::ByteCursor cursor_gyr(bmi270_gyr_range_change, sizeof(bmi270_gyr_range_change));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_gyr, bmi_sample), imu::FifoFrame::kReconfigure);

	const uint8_t bmi270_aux_only_change[] { 0x48, 0x30, 0x00, 0x00, 0x00 };
	imu::ByteCursor cursor_aux(bmi270_aux_only_change, sizeof(bmi270_aux_only_change));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_aux, bmi_sample), imu::FifoFrame::kMetadata);

	const uint8_t bmi270_noop_config[] { 0x48, 0x00, 0x00, 0x00, 0x00 };
	imu::ByteCursor cursor_noop(bmi270_noop_config, sizeof(bmi270_noop_config));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_noop, bmi_sample), imu::FifoFrame::kMetadata);

	// Skip header (0x40) with non-zero skipped count indicates hardware FIFO overflow; zero stays metadata.
	const uint8_t skip_overflow[] { 0x40, 0x03 };
	imu::ByteCursor cursor_skip_overflow(skip_overflow, sizeof(skip_overflow));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<false>(cursor_skip_overflow, bmi_sample), imu::FifoFrame::kOverflow);

	const uint8_t skip_zero[] { 0x40, 0x00 };
	imu::ByteCursor cursor_skip_zero(skip_zero, sizeof(skip_zero));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<false>(cursor_skip_zero, bmi_sample), imu::FifoFrame::kMetadata);
}

TEST(BMIHybridFifoTest, ClampedTailAndFixedSentinel)
{
	bmi_hybrid_fifo::Sample bmi_sample {};

	// Clamped burst treats a partial trailing frame as kEnd (retransmitted on next FIFO_DATA read),
	// whereas an unclamped read rejects a truncated frame as kInvalid.
	const uint8_t truncated_combined[] {
		0x8c, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0,
		0x8c, 7, 0, 8, 0
	};
	imu::ByteCursor cursor_clamped(truncated_combined, sizeof(truncated_combined));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_clamped, bmi_sample, true), imu::FifoFrame::kBoth);
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_clamped, bmi_sample, true), imu::FifoFrame::kEnd);

	// decodeFifoFrames must report consumed_bytes == 13 (excluding the 5-byte unretired partial tail).
	imu::SampleBatch<4> accel_batch {};
	imu::SampleBatch<4> gyro_batch_tagged {};
	const auto decode_clamped = [](imu::ByteCursor & cursor, bmi_hybrid_fifo::Sample & decoded) {
		return bmi_hybrid_fifo::readTagged<true>(cursor, decoded, true);
	};
	const imu::FifoDecodeResult clamped_res = imu::decodeFifoFrames<imu::FifoAxisMapping::kFlipYZ>(
				truncated_combined, sizeof(truncated_combined), accel_batch, gyro_batch_tagged, bmi_sample, decode_clamped);
	EXPECT_EQ(clamped_res.status, imu::FifoDecodeStatus::kComplete);
	EXPECT_EQ(clamped_res.consumed_bytes, 13u);
	EXPECT_EQ(accel_batch.samples, 1u);
	EXPECT_EQ(gyro_batch_tagged.samples, 1u);

	imu::ByteCursor cursor_unclamped(truncated_combined, sizeof(truncated_combined));
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_unclamped, bmi_sample, false), imu::FifoFrame::kBoth);
	EXPECT_EQ(bmi_hybrid_fifo::readTagged<true>(cursor_unclamped, bmi_sample, false), imu::FifoFrame::kInvalid);

	// BMI088 fixed gyro skips individual all-INT16_MIN sentinel frames instead of dropping the whole batch.
	const uint8_t gyro_with_sentinel[18] {
		0x10, 0x00, 0x20, 0x00, 0x30, 0x00,
		0x00, 0x80, 0x00, 0x80, 0x00, 0x80,
		0x40, 0x00, 0x50, 0x00, 0x60, 0x00,
	};
	imu::SampleBatch<8> gyro_batch {};
	ASSERT_TRUE(bmi_hybrid_fifo::decodeFixed<0>(gyro_with_sentinel, sizeof(gyro_with_sentinel), true, gyro_batch));
	ASSERT_EQ(gyro_batch.samples, 2u);
	EXPECT_EQ(gyro_batch.x[0], 0x0010);
	EXPECT_EQ(gyro_batch.y[0], -0x0020);
	EXPECT_EQ(gyro_batch.z[0], -0x0030);
	EXPECT_EQ(gyro_batch.x[1], 0x0040);
	EXPECT_EQ(gyro_batch.y[1], -0x0050);
	EXPECT_EQ(gyro_batch.z[1], -0x0060);
}

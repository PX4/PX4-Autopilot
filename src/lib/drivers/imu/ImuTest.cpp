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
 * @file ImuTest.cpp
 * @brief Host unit tests for shared IMU wire decoding, FIFO batching, and SPI helpers.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "ByteCursor.hpp"
#include "FifoBatch.hpp"
#include "FifoFrames.hpp"
#include "FifoSampleStats.hpp"
#include "ImuPerf.hpp"
#include "ImuSamplePhase.hpp"
#include "SpiFamily.hpp"
#include "TdkFifoPacket.hpp"

TEST(ImuTest, ByteCursorAndWireWords)
{
	const uint8_t raw[] { 0x80, 0x00, 0x7f, 0xff, 0x01, 0x02, 0x03, 0x04 };
	imu::ByteCursor cursor(raw, sizeof(raw));

	EXPECT_EQ(cursor.remaining(), 8u);
	EXPECT_EQ(cursor.take(9), nullptr);
	EXPECT_EQ(cursor.remaining(), 8u);
	EXPECT_NE(cursor.take(0), nullptr);
	EXPECT_EQ(cursor.remaining(), 8u);

	imu::ByteCursor null_cursor(nullptr, 4);
	EXPECT_EQ(null_cursor.remaining(), 0u);
	EXPECT_EQ(null_cursor.take(1), nullptr);

	const uint8_t *w0 = cursor.take(2);
	ASSERT_NE(w0, nullptr);
	EXPECT_EQ(imu::readBe16(w0), INT16_MIN);
	EXPECT_EQ(imu::readLe16(w0), 0x0080);

	const uint8_t neg_le[4] { 0xff, 0xff, 0x00, 0x80 };
	EXPECT_EQ(imu::readLe16(neg_le), -1);
	EXPECT_EQ(imu::readLe16(neg_le + 2), INT16_MIN);

	const uint8_t *w1 = cursor.take(2);
	ASSERT_NE(w1, nullptr);
	EXPECT_EQ(imu::readBe16(w1), INT16_MAX);

	const uint8_t *pair = cursor.take(4);
	ASSERT_NE(pair, nullptr);
	EXPECT_EQ((imu::readWordPair32<imu::ByteOrder::kBigEndian, imu::WordOrder::kLowFirst>(pair)), 0x03040102);
	EXPECT_EQ((imu::readWordPair32<imu::ByteOrder::kBigEndian, imu::WordOrder::kHighFirst>(pair)), 0x01020304);
	EXPECT_EQ((imu::readWordPair32<imu::ByteOrder::kLittleEndian, imu::WordOrder::kLowFirst>(pair)), 0x04030201);
	EXPECT_EQ((imu::readWordPair32<imu::ByteOrder::kLittleEndian, imu::WordOrder::kHighFirst>(pair)), 0x02010403);
	EXPECT_EQ(cursor.remaining(), 0u);

	const uint8_t neg_pair_be_hi[4] { 0x80, 0x00, 0x00, 0x00 };
	EXPECT_EQ((imu::readWordPair32<imu::ByteOrder::kBigEndian, imu::WordOrder::kHighFirst>(neg_pair_be_hi)), INT32_MIN);
	const uint8_t neg_pair_le_lo[4] { 0xff, 0xff, 0xff, 0xff };
	EXPECT_EQ((imu::readWordPair32<imu::ByteOrder::kLittleEndian, imu::WordOrder::kLowFirst>(neg_pair_le_lo)), -1);

	EXPECT_EQ((imu::readUnsigned<1, imu::ByteOrder::kBigEndian>(pair)), 0x01u);
	EXPECT_EQ((imu::readUnsigned<2, imu::ByteOrder::kBigEndian>(pair)), 0x0102u);
	EXPECT_EQ((imu::readUnsigned<3, imu::ByteOrder::kBigEndian>(pair)), 0x010203u);
	EXPECT_EQ((imu::readUnsigned<4, imu::ByteOrder::kBigEndian>(pair)), 0x01020304u);
	EXPECT_EQ((imu::readUnsigned<1, imu::ByteOrder::kLittleEndian>(pair)), 0x01u);
	EXPECT_EQ((imu::readUnsigned<2, imu::ByteOrder::kLittleEndian>(pair)), 0x0201u);
	EXPECT_EQ((imu::readUnsigned<3, imu::ByteOrder::kLittleEndian>(pair)), 0x030201u);
	EXPECT_EQ((imu::readUnsigned<4, imu::ByteOrder::kLittleEndian>(pair)), 0x04030201u);

	EXPECT_EQ(imu::negateSaturated(INT16_MIN), INT16_MAX);
	EXPECT_EQ(imu::negateSaturated(-42), 42);
	EXPECT_EQ(imu::negateSaturated(0), 0);
	EXPECT_EQ(imu::negateSaturated(INT16_MAX), -INT16_MAX);

	const uint8_t ones[18] {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};
	EXPECT_EQ(imu::byteSum16(ones, sizeof(ones)), uint16_t(18 * 255));
	EXPECT_EQ(imu::byteSum16(nullptr, 10), 0u);

	uint8_t wrap_buf[300];
	memset(wrap_buf, 0xff, sizeof(wrap_buf));
	EXPECT_EQ(imu::byteSum16(wrap_buf, sizeof(wrap_buf)), uint16_t((300u * 255u) & 0xffffu));
}

TEST(ImuTest, FixedFifoDecodeAndPhase)
{
	// 4 fixed 12-byte frames with repeated accel [A0==A1, A2==A3] -> first_sample = 1
	const uint8_t stream[48] {
		0x00, 0x10, 0x80, 0x00, 0x00, 0x20,  0, 1, 0, 2, 0, 3,
		0x00, 0x10, 0x80, 0x00, 0x00, 0x20,  0, 4, 0, 5, 0, 6,
		0x00, 0x30, 0xff, 0xf6, 0x00, 0x40,  0, 7, 0, 8, 0, 9,
		0x00, 0x30, 0xff, 0xf6, 0x00, 0x40,  0, 10, 0, 11, 0, 12,
	};

	uint8_t first = 0;
	ASSERT_TRUE(imu::repeatedAccelPhase(stream, sizeof(stream), 12, first));
	EXPECT_EQ(first, 1u);

	imu::SampleBatch<8> accel {};
	const bool ok = imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(
				stream, sizeof(stream), 12, first, 2, accel,
	[](const uint8_t *frame, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(frame, axes);
		return imu::FifoSampleResult::kAppend;
	});

	ASSERT_TRUE(ok);
	ASSERT_EQ(accel.samples, 2u);
	EXPECT_EQ(accel.x[0], 0x0010);
	EXPECT_EQ(accel.y[0], INT16_MAX); // Saturated negation of INT16_MIN
	EXPECT_EQ(accel.z[0], -0x0020);
	EXPECT_EQ(accel.x[1], 0x0030);
	EXPECT_EQ(accel.y[1], 10);
	EXPECT_EQ(accel.z[1], -0x0040);

	// 1-frame burst forces first_sample = 1; middle-repeated [A1==A2] picks first_sample = 0
	ASSERT_TRUE(imu::repeatedAccelPhase(stream, 12, 12, first, 0));
	EXPECT_EQ(first, 1u);

	uint8_t mid_repeat[48] {};
	memcpy(mid_repeat, stream + 12, 36); // frames: [A0(orig1), A2, A2, zero] -> A1==A2
	ASSERT_TRUE(imu::repeatedAccelPhase(mid_repeat, sizeof(mid_repeat), 12, first));
	EXPECT_EQ(first, 0u);

	// 2-frame bursts honor short_phase; 3-frame bursts inspect [A0, A1, A2] pairs directly
	ASSERT_TRUE(imu::repeatedAccelPhase(mid_repeat, 24, 12, first, 0));
	EXPECT_EQ(first, 0u);
	ASSERT_TRUE(imu::repeatedAccelPhase(mid_repeat, 36, 12, first, 1)); // [A0, A2, A2] -> first = 0
	EXPECT_EQ(first, 0u);
	ASSERT_TRUE(imu::repeatedAccelPhase(stream, 36, 12, first, 0));     // [A0, A0, A2] -> first = 1
	EXPECT_EQ(first, 1u);

	uint8_t all_equal[36] {};
	ASSERT_TRUE(imu::repeatedAccelPhase(all_equal, sizeof(all_equal), 12, first, 0));
	EXPECT_EQ(first, 0u);
	ASSERT_TRUE(imu::repeatedAccelPhase(all_equal, sizeof(all_equal), 12, first, 1));
	EXPECT_EQ(first, 1u);

	uint8_t no_repeat_3[36] {};
	no_repeat_3[0]  = 1;
	no_repeat_3[12] = 2;
	no_repeat_3[24] = 3;
	EXPECT_FALSE(imu::repeatedAccelPhase(no_repeat_3, sizeof(no_repeat_3), 12, first));

	// 4-frame burst with neither [A0==A1 && A2==A3] nor [A1==A2] fails
	uint8_t no_repeat[48] {};
	no_repeat[0] = 1;
	no_repeat[12] = 2;
	no_repeat[24] = 3;
	no_repeat[36] = 4;
	EXPECT_FALSE(imu::repeatedAccelPhase(no_repeat, sizeof(no_repeat), 12, first));
	EXPECT_FALSE(imu::repeatedAccelPhase(nullptr, 12, 12, first));
	EXPECT_FALSE(imu::repeatedAccelPhase(stream, 12, 5, first));
	EXPECT_FALSE(imu::repeatedAccelPhase(stream, 0, 12, first));
	EXPECT_FALSE(imu::repeatedAccelPhase(stream, 13, 12, first));
	EXPECT_FALSE(imu::repeatedAccelPhase(stream, 24, 12, first, 2));

	// Portable arithmetic right shift via readAxes16<..., 4> on negative wire words
	const uint8_t shifted_words[6] { 0xff, 0xf0, 0xff, 0xef, 0x80, 0x00 }; // -16, -17, -32768
	int16_t shifted[3] {};
	imu::readAxes16<imu::ByteOrder::kBigEndian, 4>(shifted_words, shifted);
	EXPECT_EQ(shifted[0], -1);
	EXPECT_EQ(shifted[1], -2);
	EXPECT_EQ(shifted[2], -2048);

	// kFlipYZValidated rejects INT16_MIN on Y or Z and leaves batch.samples untouched
	imu::SampleBatch<4> validated {};
	EXPECT_TRUE(imu::appendFifoSample<imu::FifoAxisMapping::kFlipYZValidated>(validated, { INT16_MIN, 2, -3 }));
	EXPECT_EQ(validated.samples, 1u);
	EXPECT_EQ(validated.x[0], INT16_MIN);
	EXPECT_EQ(validated.y[0], -2);
	EXPECT_EQ(validated.z[0], 3);
	EXPECT_FALSE(imu::appendFifoSample<imu::FifoAxisMapping::kFlipYZValidated>(validated, { 1, INT16_MIN, 0 }));
	EXPECT_FALSE(imu::appendFifoSample<imu::FifoAxisMapping::kFlipYZValidated>(validated, { 1, 0, INT16_MIN }));
	EXPECT_EQ(validated.samples, 1u);

	// Atomic no-commit when decodeFixedFifo fails mid-batch on sentinel or kInvalid
	const uint8_t two_frames[12] { 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x80, 0x00, 0x00, 0x05 };
	EXPECT_FALSE(imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZValidated>(
			     two_frames, sizeof(two_frames), 6, 0, 1, validated,
	[](const uint8_t *frame, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(frame, axes);
		return imu::FifoSampleResult::kAppend;
	}));
	EXPECT_EQ(validated.samples, 1u);

	// may_skip = true with kIdentity preserves existing samples and appends only selected frames
	EXPECT_TRUE((imu::decodeFixedFifo<imu::FifoAxisMapping::kIdentity, true>(
			     two_frames, sizeof(two_frames), 6, 0, 1, validated,
	[&two_frames](const uint8_t *frame, int16_t (&axes)[3]) {
		if (frame == two_frames) {
			return imu::FifoSampleResult::kSkip;
		}

		imu::readAxes16<imu::ByteOrder::kBigEndian>(frame, axes);
		return imu::FifoSampleResult::kAppend;
	})));
	ASSERT_EQ(validated.samples, 2u);
	EXPECT_EQ(validated.x[1], 4);
	EXPECT_EQ(validated.y[1], INT16_MIN);
	EXPECT_EQ(validated.z[1], 5);

	// Overflow and invalid arguments fail closed without mutating batch
	EXPECT_TRUE(imu::appendFifoSample<imu::FifoAxisMapping::kIdentity>(validated, { 1, 1, 1 }));
	EXPECT_TRUE(imu::appendFifoSample<imu::FifoAxisMapping::kIdentity>(validated, { 2, 2, 2 }));
	EXPECT_FALSE(imu::appendFifoSample<imu::FifoAxisMapping::kIdentity>(validated, { 3, 3, 3 }));
	EXPECT_EQ(validated.samples, 4u);
	EXPECT_FALSE(imu::decodeFixedFifo<imu::FifoAxisMapping::kIdentity>(nullptr, 6, 6, 0, 1, validated,
	[](const uint8_t *, int16_t (&)[3]) { return imu::FifoSampleResult::kAppend; }));
	EXPECT_FALSE(imu::decodeFixedFifo<imu::FifoAxisMapping::kIdentity>(two_frames, 0, 6, 0, 1, validated,
	[](const uint8_t *, int16_t (&)[3]) { return imu::FifoSampleResult::kAppend; }));
	EXPECT_FALSE(imu::decodeFixedFifo<imu::FifoAxisMapping::kIdentity>(two_frames, 5, 6, 0, 1, validated,
	[](const uint8_t *, int16_t (&)[3]) { return imu::FifoSampleResult::kAppend; }));
	EXPECT_FALSE(imu::decodeFixedFifo<imu::FifoAxisMapping::kIdentity>(two_frames, 6, 5, 0, 1, validated,
	[](const uint8_t *, int16_t (&)[3]) { return imu::FifoSampleResult::kAppend; }));
	EXPECT_FALSE(imu::decodeFixedFifo<imu::FifoAxisMapping::kIdentity>(two_frames, 12, 6, 0, 0, validated,
	[](const uint8_t *, int16_t (&)[3]) { return imu::FifoSampleResult::kAppend; }));
}

TEST(ImuTest, FramedFifoTraversalAndGuards)
{
	// Frame 1: combined (0x8c + 6B gyro + 6B accel), Frame 2: metadata (0x44 + 3B), Frame 3: end (0x80 + 0x00)
	const uint8_t raw[] {
		0x8c,  0x01, 0x00, 0x02, 0x00, 0x03, 0x00,  0x04, 0x00, 0x05, 0x00, 0x06, 0x00,
		0x44,  0xaa, 0xbb, 0xcc,
		0x80,  0x00,
		0xff // Trailing byte after explicit end marker must not be inspected
	};

	imu::SampleBatch<4> accel {};
	imu::SampleBatch<4> gyro {};
	imu::FifoFrameSample sample {};

	const auto decode = [](imu::ByteCursor & cursor, imu::FifoFrameSample & out) {
		const uint8_t *hdr = cursor.take(1);

		if (!hdr) {
			return imu::FifoFrame::kEnd;
		}

		if (*hdr == 0x8c) {
			const uint8_t *d = cursor.take(12);

			if (!d) {
				return imu::FifoFrame::kInvalid;
			}

			imu::readAxes16<imu::ByteOrder::kLittleEndian>(d, out.gyro);
			imu::readAxes16<imu::ByteOrder::kLittleEndian>(d + 6, out.accel);
			return imu::FifoFrame::kBoth;
		}

		if (*hdr == 0x44) {
			return cursor.take(3) ? imu::FifoFrame::kMetadata : imu::FifoFrame::kInvalid;
		}

		if (*hdr == 0x80) {
			const uint8_t *d = cursor.take(1);
			return d && d[0] == 0 ? imu::FifoFrame::kEnd : imu::FifoFrame::kInvalid;
		}

		if (*hdr == 0x70) {
			return imu::FifoFrame::kReconfigure;
		}

		if (*hdr == 0x90) {
			const uint8_t *d = cursor.take(6);

			if (!d) {
				return imu::FifoFrame::kInvalid;
			}

			imu::readAxes16<imu::ByteOrder::kLittleEndian>(d, out.accel);
			return imu::FifoFrame::kAccel;
		}

		if (*hdr == 0xa0) {
			const uint8_t *d = cursor.take(6);

			if (!d) {
				return imu::FifoFrame::kInvalid;
			}

			imu::readAxes16<imu::ByteOrder::kLittleEndian>(d, out.gyro);
			return imu::FifoFrame::kGyro;
		}

		return imu::FifoFrame::kInvalid;
	};

	const imu::FifoDecodeResult res = imu::decodeFifoFrames<imu::FifoAxisMapping::kFlipYZ,
				    imu::FifoChannels::kBoth,
				    imu::FifoAxisMapping::kIdentity>(
					    raw, sizeof(raw), accel, gyro, sample, decode);

	EXPECT_EQ(res.status, imu::FifoDecodeStatus::kComplete);
	EXPECT_TRUE(res.has_metadata);
	ASSERT_EQ(accel.samples, 1u);
	ASSERT_EQ(gyro.samples, 1u);
	EXPECT_EQ(gyro.x[0], 1);
	EXPECT_EQ(gyro.y[0], 2); // distinct gyro_mapping = kIdentity
	EXPECT_EQ(gyro.z[0], 3);
	EXPECT_EQ(accel.x[0], 4);
	EXPECT_EQ(accel.y[0], -5);
	EXPECT_EQ(accel.z[0], -6);

	// Zero-byte kEnd before any decoded sample finishes with kEmpty
	const uint8_t end_bytes[2] { 0x90, 0x01 };
	imu::SampleBatch<4> a_end {};
	imu::SampleBatch<4> g_end {};
	const imu::FifoDecodeResult zero_end = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(
			end_bytes, 1, a_end, g_end, sample,
	[](imu::ByteCursor &, imu::FifoFrameSample &) {
		return imu::FifoFrame::kEnd;
	});
	EXPECT_EQ(zero_end.status, imu::FifoDecodeStatus::kEmpty);

	// Zero-byte kEnd after a decoded sample finishes with kComplete without triggering non-progress failure
	const uint8_t sample_then_end[] {
		0x90, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0xff
	};
	bool emit_sample = true;
	const imu::FifoDecodeResult complete_end = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(
				sample_then_end, sizeof(sample_then_end), a_end, g_end, sample,
	[&decode, &emit_sample](imu::ByteCursor & cursor, imu::FifoFrameSample & out) {
		if (emit_sample) {
			emit_sample = false;
			return decode(cursor, out);
		}

		return imu::FifoFrame::kEnd;
	});
	EXPECT_EQ(complete_end.status, imu::FifoDecodeStatus::kComplete);
	EXPECT_EQ(a_end.samples, 1u);

	// Non-progressing non-kEnd decoder must fail closed with kInvalid
	imu::SampleBatch<4> a2 {};
	imu::SampleBatch<4> g2 {};
	const imu::FifoDecodeResult bad = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(
			raw, sizeof(raw), a2, g2, sample,
	[](imu::ByteCursor &, imu::FifoFrameSample &) {
		return imu::FifoFrame::kMetadata;
	});
	EXPECT_EQ(bad.status, imu::FifoDecodeStatus::kInvalid);

	// kReconfigure, kOverflow, and kInvalid mid-stream roll back already decoded samples and metadata flag
	const uint8_t reconf_stream[] {
		0x44, 0x01, 0x02, 0x03,
		0x90, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00,
		0x70
	};
	const imu::FifoDecodeResult reconf = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(
			reconf_stream, sizeof(reconf_stream), a2, g2, sample, decode);
	EXPECT_EQ(reconf.status, imu::FifoDecodeStatus::kReconfigure);
	EXPECT_FALSE(reconf.has_metadata);
	EXPECT_EQ(a2.samples, 0u);
	EXPECT_EQ(g2.samples, 0u);

	const imu::FifoDecodeResult overflow_res = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(
				reconf_stream, sizeof(reconf_stream), a2, g2, sample,
	[](imu::ByteCursor & cursor, imu::FifoFrameSample &) {
		const uint8_t *hdr = cursor.take(1);

		if (hdr && *hdr == 0x44 && cursor.take(3)) {
			return imu::FifoFrame::kMetadata;
		}

		return imu::FifoFrame::kOverflow;
	});
	EXPECT_EQ(overflow_res.status, imu::FifoDecodeStatus::kOverflow);
	EXPECT_FALSE(overflow_res.has_metadata);

	// Cross-channel rejection and capacity overflow rollback
	const uint8_t gyro_frame[] { 0xa0, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00 };
	const imu::FifoDecodeResult wrong_ch_a = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity, imu::FifoChannels::kAccel>(
				gyro_frame, sizeof(gyro_frame), a2, g2, sample, decode);
	EXPECT_EQ(wrong_ch_a.status, imu::FifoDecodeStatus::kInvalid);

	const uint8_t accel_frame[] { 0x90, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00 };
	const imu::FifoDecodeResult wrong_ch_g = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity, imu::FifoChannels::kGyro>(
				accel_frame, sizeof(accel_frame), a2, g2, sample, decode);
	EXPECT_EQ(wrong_ch_g.status, imu::FifoDecodeStatus::kInvalid);

	const imu::FifoDecodeResult ok_gyro = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity, imu::FifoChannels::kGyro>(
			gyro_frame, sizeof(gyro_frame), a2, g2, sample, decode);
	EXPECT_EQ(ok_gyro.status, imu::FifoDecodeStatus::kComplete);
	EXPECT_EQ(g2.samples, 1u);
	EXPECT_EQ(a2.samples, 0u);

	// Capacity overflow mid-stream rolls back already decoded samples
	imu::SampleBatch<1> a_small {};
	imu::SampleBatch<1> g_small {};
	const uint8_t two_accel[] {
		0x90, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00,
		0x90, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00
	};
	const imu::FifoDecodeResult ovf = imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(
			two_accel, sizeof(two_accel), a_small, g_small, sample, decode);
	EXPECT_EQ(ovf.status, imu::FifoDecodeStatus::kInvalid);
	EXPECT_EQ(a_small.samples, 0u);
	EXPECT_EQ(g_small.samples, 0u);

	// Empty stream and non-empty pre-existing batch checks
	EXPECT_EQ((imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(nullptr, 1, a2, g2, sample, decode).status),
		  imu::FifoDecodeStatus::kInvalid); // g2.samples != 0 and nullptr
	g2.samples = 0;
	EXPECT_EQ((imu::decodeFifoFrames<imu::FifoAxisMapping::kIdentity>(raw, 0, a2, g2, sample, decode).status),
		  imu::FifoDecodeStatus::kEmpty);
}

TEST(ImuTest, SampleStatsAndTdkPacket)
{
	imu::FifoSampleStats stats;
	float mean = -123.f;

	EXPECT_TRUE(stats.empty());
	EXPECT_FALSE(stats.meanWithin(100.f, mean));
	EXPECT_FLOAT_EQ(mean, -123.f);

	EXPECT_TRUE(stats.add(int16_t(100)));
	EXPECT_TRUE(stats.add(int16_t(200)));
	EXPECT_TRUE(stats.add(int16_t(300)));
	EXPECT_TRUE(stats.meanWithin(100.f, mean));
	EXPECT_FLOAT_EQ(mean, 200.f);
	EXPECT_FALSE(stats.meanWithin(99.f, mean));
	EXPECT_FLOAT_EQ(mean, 200.f);
	EXPECT_FALSE(stats.meanWithin(-1.f, mean));
	EXPECT_FALSE(stats.meanWithin(NAN, mean));

	imu::FifoSampleStats max_stats;

	for (uint32_t i = 0; i < UINT16_MAX; ++i) {
		ASSERT_TRUE(max_stats.add(int16_t(INT16_MAX)));
	}

	EXPECT_FALSE(max_stats.add(int16_t(INT16_MAX)));
	EXPECT_TRUE(max_stats.meanWithin(0.f, mean));
	EXPECT_FLOAT_EQ(mean, static_cast<float>(INT16_MAX));

	imu::FifoSampleStats max_513_stats;

	for (int i = 0; i < 513; ++i) {
		ASSERT_TRUE(max_513_stats.add(int16_t(INT16_MAX)));
	}

	EXPECT_TRUE(max_513_stats.meanWithin(0.f, mean));
	EXPECT_FLOAT_EQ(mean, static_cast<float>(INT16_MAX));

	imu::FifoSampleStats neg_stats;

	for (int i = 0; i < 1000; ++i) {
		ASSERT_TRUE(neg_stats.add(int16_t(INT16_MIN)));
	}

	EXPECT_TRUE(neg_stats.meanWithin(0.f, mean));
	EXPECT_FLOAT_EQ(mean, static_cast<float>(INT16_MIN));

	// Little-endian 20-byte ICM45686/56686 packet with wrapping timestamp == 0
	const uint8_t le_pkt[20] {
		0x78,
		0x10, 0x00, 0x20, 0x00, 0x30, 0x00, // accel LE
		0x40, 0x00, 0x50, 0x00, 0x60, 0x00, // gyro LE
		0x80, 0x00,                         // temp LE = 128
		0x00, 0x00,                         // timestamp = 0 (allowed on wrapping counter, rejected on delta)
		0x00, 0x00, 0x00
	};
	imu::TdkFifoSample parsed {};
	EXPECT_TRUE(imu::decodeTdkFifoPacket(le_pkt, sizeof(le_pkt), true, true, false, true, parsed));
	EXPECT_TRUE(parsed.accel_valid);
	EXPECT_TRUE(parsed.gyro_valid);
	EXPECT_EQ(parsed.accel[0], 0x0010);
	EXPECT_EQ(parsed.gyro[2], 0x0060);
	EXPECT_EQ(parsed.temperature, 128);
	EXPECT_EQ(parsed.timestamp, 0u);

	// Delta-timestamp mode rejects timestamp == 0
	EXPECT_FALSE(imu::decodeTdkFifoPacket(le_pkt, sizeof(le_pkt), true, true, true, true, parsed));

	// Little-endian 20-byte ICM56686 allows ODR-change bits [1:0] (mask 0xfc)
	uint8_t le_20_odr_bits[20] {};
	memcpy(le_20_odr_bits, le_pkt, sizeof(le_20_odr_bits));
	le_20_odr_bits[0] = 0x7b;
	EXPECT_TRUE(imu::decodeTdkFifoPacket(le_20_odr_bits, sizeof(le_20_odr_bits), true, true, false, true, parsed));

	// Per-channel 20-bit sentinel: 0x80000 (word == INT16_MIN && nibble == 0) is invalid,
	// whereas 0x80001..0x8000f (word == INT16_MIN && nibble != 0) is real negative saturation.
	uint8_t sentinel_pkt[20] {};
	memcpy(sentinel_pkt, le_pkt, sizeof(sentinel_pkt));
	sentinel_pkt[1] = 0x00;
	sentinel_pkt[2] = 0x80; // accel[0] = INT16_MIN (LE), ext nibble = 0
	EXPECT_TRUE(imu::decodeTdkFifoPacket(sentinel_pkt, sizeof(sentinel_pkt), true, true, false, true, parsed));
	EXPECT_FALSE(parsed.accel_valid);
	EXPECT_TRUE(parsed.gyro_valid);

	sentinel_pkt[17] = 0x40; // accel[0] low nibble = 0x4 (18-bit negative full-scale saturation -524284)
	EXPECT_TRUE(imu::decodeTdkFifoPacket(sentinel_pkt, sizeof(sentinel_pkt), true, true, false, true, parsed));
	EXPECT_TRUE(parsed.accel_valid);
	EXPECT_EQ(parsed.accel[0], INT16_MIN);
	sentinel_pkt[17] = 0x00;

	EXPECT_TRUE(imu::decodeTdkFifoPacket(sentinel_pkt, sizeof(sentinel_pkt), true, true, false, false, parsed));
	EXPECT_TRUE(parsed.accel_valid);
	EXPECT_TRUE(parsed.gyro_valid);

	sentinel_pkt[1] = 0x10;
	sentinel_pkt[2] = 0x00;
	sentinel_pkt[11] = 0x00;
	sentinel_pkt[12] = 0x80; // gyro[2] = INT16_MIN (LE), ext nibble = 0
	EXPECT_TRUE(imu::decodeTdkFifoPacket(sentinel_pkt, sizeof(sentinel_pkt), true, true, false, true, parsed));
	EXPECT_TRUE(parsed.accel_valid);
	EXPECT_FALSE(parsed.gyro_valid);

	sentinel_pkt[19] = 0x02; // gyro[2] low nibble = 0x2 (19-bit negative full-scale saturation -524286)
	EXPECT_TRUE(imu::decodeTdkFifoPacket(sentinel_pkt, sizeof(sentinel_pkt), true, true, false, true, parsed));
	EXPECT_TRUE(parsed.accel_valid);
	EXPECT_TRUE(parsed.gyro_valid);
	EXPECT_EQ(parsed.gyro[2], INT16_MIN);

	// Little-endian 16-byte ICM45686 allows ODR-change bits [1:0] (mask 0xfc) while rejecting FSYNC/HIRES bits
	uint8_t le_16_pkt[16] {
		0x68,
		0x10, 0x00, 0x20, 0x00, 0x30, 0x00, // accel LE
		0x40, 0x00, 0x50, 0x00, 0x60, 0x00, // gyro LE
		0x12,                               // raw byte temperature (unused in 16-byte LE path)
		0x00, 0x00                          // timestamp = 0 (allowed on wrapping counter)
	};
	EXPECT_TRUE(imu::decodeTdkFifoPacket(le_16_pkt, sizeof(le_16_pkt), true, true, false, true, parsed));
	EXPECT_EQ(parsed.accel[0], 0x0010);
	EXPECT_EQ(parsed.gyro[2], 0x0060);
	EXPECT_EQ(parsed.timestamp, 0u);

	le_16_pkt[0] = 0x6b; // ACCEL_ODR | GYRO_ODR set on first post-flush packet -> accepted
	EXPECT_TRUE(imu::decodeTdkFifoPacket(le_16_pkt, sizeof(le_16_pkt), true, true, false, true, parsed));

	le_16_pkt[0] = 0x6c; // FSYNC_TAG_EN -> rejected
	EXPECT_FALSE(imu::decodeTdkFifoPacket(le_16_pkt, sizeof(le_16_pkt), true, true, false, true, parsed));

	le_16_pkt[0] = 0x78; // HIRES_EN on 16-byte packet -> rejected
	EXPECT_FALSE(imu::decodeTdkFifoPacket(le_16_pkt, sizeof(le_16_pkt), true, true, false, true, parsed));

	// Big-endian 16-byte kStandard16 mode uses 0xe0 header mask (accepts 0x60..0x7f)
	uint8_t be_16_pkt[16] {
		0x63,
		0x00, 0x10, 0x00, 0x20, 0x00, 0x30,
		0x00, 0x40, 0x00, 0x50, 0x00, 0x60,
		0x05, 0x12, 0x34
	};
	EXPECT_TRUE(imu::decodeTdkFifoPacket(be_16_pkt, sizeof(be_16_pkt), false, false, true, false, parsed));
	EXPECT_EQ(parsed.accel[0], 0x0010);
	EXPECT_EQ(parsed.gyro[2], 0x0060);
	EXPECT_EQ(parsed.timestamp, 0x1234u);
	be_16_pkt[0] = 0x48; // header accel/gyro bits zero -> rejected
	EXPECT_FALSE(imu::decodeTdkFifoPacket(be_16_pkt, sizeof(be_16_pkt), false, false, true, false, parsed));

	// Big-endian 20-byte kHighRes20 enforces full 0xff mask on 0x78 and decodes temp & delta timestamp
	uint8_t be_20_pkt[20] {
		0x78,
		0x00, 0x01, 0x00, 0x02, 0x00, 0x03,
		0x00, 0x04, 0x00, 0x05, 0x00, 0x06,
		0xff, 0xfe,                         // temp BE = -2
		0x00, 0x40,                         // delta timestamp = 0x0040
		0x00, 0x00, 0x00
	};
	EXPECT_TRUE(imu::decodeTdkFifoPacket(be_20_pkt, sizeof(be_20_pkt), false, true, true, true, parsed));
	EXPECT_EQ(parsed.temperature, -2);
	EXPECT_EQ(parsed.timestamp, 0x0040u);
	be_20_pkt[0] = 0x7b; // ODR bits invalid on BE 20B
	EXPECT_FALSE(imu::decodeTdkFifoPacket(be_20_pkt, sizeof(be_20_pkt), false, true, true, true, parsed));

	// Invalid packet buffer or length
	EXPECT_FALSE(imu::decodeTdkFifoPacket(nullptr, 16, true, true, false, false, parsed));
	EXPECT_FALSE(imu::decodeTdkFifoPacket(be_16_pkt, 15, false, false, false, false, parsed));
	EXPECT_FALSE(imu::decodeTdkFifoPacket(be_20_pkt, 19, false, true, false, false, parsed));
}

TEST(ImuTest, SpiFamilyHelpers)
{
	imu::SpiModel model {};
	model.frequency      = 8'000'000;
	model.data_frequency = 24'000'000;

	EXPECT_EQ(imu::spiConfigFrequency(model, 0), 8'000'000);
	EXPECT_EQ(imu::spiConfigFrequency(model, 4'000'000), 4'000'000);
	EXPECT_EQ(imu::spiConfigFrequency(model, 16'000'000), 8'000'000);
	EXPECT_EQ(imu::spiConfigFrequency(model, -1), 0);

	EXPECT_EQ(imu::spiDataFrequency(model, 0), 24'000'000);
	EXPECT_EQ(imu::spiDataFrequency(model, 10'000'000), 10'000'000);
	EXPECT_EQ(imu::spiDataFrequency(model, 30'000'000), 24'000'000);
	EXPECT_EQ(imu::spiDataFrequency(model, -1), 0);

	model.frequency = 0;
	EXPECT_EQ(imu::spiConfigFrequency(model, 1'000'000), 0);
	EXPECT_EQ(imu::spiDataFrequency(model, 1'000'000), 0);

	int val = -999;
	EXPECT_TRUE(imu::parseInteger("0", -10, 10, val));
	EXPECT_EQ(val, 0);
	EXPECT_TRUE(imu::parseInteger("-10", -10, 10, val));
	EXPECT_EQ(val, -10);
	EXPECT_TRUE(imu::parseInteger("10", -10, 10, val));
	EXPECT_EQ(val, 10);
	EXPECT_FALSE(imu::parseInteger(nullptr, 0, 10, val));
	EXPECT_FALSE(imu::parseInteger("", 0, 10, val));
	EXPECT_FALSE(imu::parseInteger("12a", 0, 100, val));
	EXPECT_FALSE(imu::parseInteger("-11", -10, 10, val));
	EXPECT_FALSE(imu::parseInteger("11", -10, 10, val));
	EXPECT_FALSE(imu::parseInteger("99999999999999999999", -10, 10, val));

	EXPECT_EQ(imu::parseSpiCommand("start"), imu::SpiCommand::kStart);
	EXPECT_EQ(imu::parseSpiCommand("stop"), imu::SpiCommand::kStop);
	EXPECT_EQ(imu::parseSpiCommand("status"), imu::SpiCommand::kStatus);
	EXPECT_EQ(imu::parseSpiCommand("START"), imu::SpiCommand::kInvalid);
	EXPECT_EQ(imu::parseSpiCommand(""), imu::SpiCommand::kInvalid);
	EXPECT_EQ(imu::parseSpiCommand(nullptr), imu::SpiCommand::kInvalid);

	EXPECT_EQ(imu::SpiDmaBuffer::alignSize(0), 0u);
	imu::SpiDmaBuffer empty_buf(0);
	EXPECT_FALSE(static_cast<bool>(empty_buf));
	EXPECT_EQ(empty_buf.capacity(), 0u);
	EXPECT_EQ(empty_buf.data(), nullptr);
	EXPECT_EQ(empty_buf.words(), nullptr);

	imu::SpiDmaBuffer dma_buf(19);
	ASSERT_TRUE(static_cast<bool>(dma_buf));
	EXPECT_GE(dma_buf.capacity(), 19u);
	ASSERT_NE(dma_buf.data(), nullptr);
	EXPECT_EQ(static_cast<const void *>(dma_buf.words()), static_cast<const void *>(dma_buf.data()));

	for (size_t i = 0; i < dma_buf.capacity(); ++i) {
		EXPECT_EQ(dma_buf.data()[i], 0u);
	}
}

TEST(ImuTest, ReviewRegressionCoverage)
{
	// 16-byte ICM-42605 / ICM-40609-D packets also use -32768 as an invalid-axis sentinel.
	imu::TdkFifoSample parsed {};
	const uint8_t icm42605_invalid_accel[16] {
		0x68,
		0x80, 0x00, 0x00, 0x10, 0x00, 0x20, // accel X = INT16_MIN (BE)
		0x00, 0x30, 0x00, 0x40, 0x00, 0x50, // valid gyro
		0x01, 0x12, 0x34
	};
	EXPECT_TRUE(imu::decodeTdkFifoPacket(icm42605_invalid_accel, sizeof(icm42605_invalid_accel), false, false, false, true, parsed));
	EXPECT_FALSE(parsed.accel_valid);
	EXPECT_TRUE(parsed.gyro_valid);

	const uint8_t icm42605_invalid_gyro[16] {
		0x68,
		0x00, 0x10, 0x00, 0x20, 0x00, 0x30, // valid accel
		0x00, 0x40, 0x80, 0x00, 0x00, 0x50, // gyro Y = INT16_MIN (BE)
		0x01, 0x12, 0x34
	};
	EXPECT_TRUE(imu::decodeTdkFifoPacket(icm42605_invalid_gyro, sizeof(icm42605_invalid_gyro), false, false, false, true, parsed));
	EXPECT_TRUE(parsed.accel_valid);
	EXPECT_FALSE(parsed.gyro_valid);

	// Backlog timestamp adjustment shifts polling batches backward and preserves sample-aligned DRDY timestamps.
	EXPECT_EQ(imu::adjustFifoBacklogTimestamp(10'000, false, 48, 32, 156.25f), 7'500u);
	EXPECT_EQ(imu::adjustFifoBacklogTimestamp(10'000, true, 33, 32, 125.f), 10'000u);
	EXPECT_EQ(imu::adjustFifoBacklogTimestamp(100, false, 40, 32, 125.f), 0u);
}

TEST(ImuTest, SpiClockAndEndpointClaims)
{
	int clock_hz = -1;
	EXPECT_TRUE(imu::parseSpiClockHz("0", clock_hz));
	EXPECT_EQ(clock_hz, 0);
	EXPECT_TRUE(imu::parseSpiClockHz("32768", clock_hz));
	EXPECT_EQ(clock_hz, 32'768);
	EXPECT_TRUE(imu::parseSpiClockHz("1000000", clock_hz));
	EXPECT_EQ(clock_hz, 1'000'000);
	EXPECT_FALSE(imu::parseSpiClockHz("-1", clock_hz));
	EXPECT_FALSE(imu::parseSpiClockHz("1000001", clock_hz));
	EXPECT_FALSE(imu::parseSpiClockHz("32000Hz", clock_hz));
	EXPECT_FALSE(imu::parseSpiClockHz("", clock_hz));
	EXPECT_FALSE(imu::parseSpiClockHz(nullptr, clock_hz));

	imu::SpiModel clock_capable {};
	clock_capable.min_clock_hz = 31'000;
	clock_capable.max_clock_hz = 50'000;
	EXPECT_TRUE(imu::validSpiClockHz(clock_capable, 0));
	EXPECT_TRUE(imu::validSpiClockHz(clock_capable, 31'000));
	EXPECT_TRUE(imu::validSpiClockHz(clock_capable, 32'768));
	EXPECT_TRUE(imu::validSpiClockHz(clock_capable, 50'000));
	EXPECT_FALSE(imu::validSpiClockHz(clock_capable, -1));
	EXPECT_FALSE(imu::validSpiClockHz(clock_capable, 30'999));
	EXPECT_FALSE(imu::validSpiClockHz(clock_capable, 50'001));

	imu::SpiModel no_clock {};
	no_clock.min_clock_hz = 0;
	no_clock.max_clock_hz = 0;
	EXPECT_TRUE(imu::validSpiClockHz(no_clock, 0));
	EXPECT_FALSE(imu::validSpiClockHz(no_clock, 32'000));
	EXPECT_FALSE(imu::validSpiClockHz(no_clock, 1));
	EXPECT_FALSE(imu::validSpiClockHz(no_clock, -1));

	imu::SpiEndpointClaimTable<2> claims;
	const imu::SpiEndpointKey invalid { -1, 0x1009u, true };
	const imu::SpiEndpointKey bus1_cs1 { 1, 0x1009u, true };
	const imu::SpiEndpointKey bus1_cs2 { 1, 0x1004u, true };
	const imu::SpiEndpointKey bus2_cs1 { 2, 0x1009u, true };
	const imu::SpiEndpointKey bus1_slot { 1, 0x1009u, false };

	EXPECT_FALSE(invalid.valid());
	EXPECT_FALSE(claims.acquire(invalid));
	claims.release(invalid);

	EXPECT_TRUE(claims.acquire(bus1_cs1));
	EXPECT_FALSE(claims.acquire(bus1_cs1)); // duplicate (bus, cs_gpio) blocked
	EXPECT_TRUE(claims.acquire(bus1_cs2));  // distinct CS on same bus allowed
	EXPECT_FALSE(claims.acquire(bus2_cs1)); // capacity 2 reached
	EXPECT_FALSE(claims.acquire(bus1_cs2)); // still blocks duplicate when full

	claims.release(bus1_cs1);
	EXPECT_FALSE(claims.acquire(bus1_cs2)); // free slot 0 must not skip duplicate check at slot 1
	EXPECT_TRUE(claims.acquire(bus2_cs1));  // distinct bus allowed once capacity frees
	claims.release(bus2_cs1);
	EXPECT_TRUE(claims.acquire(bus1_slot)); // fallback slot key is distinct from GPIO CS key
	claims.release(bus1_slot);
	claims.release(bus1_cs2);
	EXPECT_TRUE(claims.acquire(bus1_cs1));  // re-acquire after release succeeds
}

TEST(ImuTest, DrdySampleAnchorAndBacklogAlignment)
{
	constexpr float    kSampleDtUs { 125.f };
	constexpr uint32_t kDelay400Hz { 2500u };
	constexpr uint32_t kDelay800Hz { 1250u };

	auto feed_pulses = [](imu::DrdySampleAnchor & anchor, hrt_abstime start_ts, uint16_t batch, uint16_t count) {
		for (uint16_t i = 0; i < count; ++i) {
			anchor.onInterrupt(start_ts + static_cast<hrt_abstime>(i) * 125u, batch);
		}
	};

	// 1. Normal N (20 samples at 400 Hz) + consumed anchor rejected on immediate retry.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 10'000u, 20, 20); // T_20 = 10'000 + 19 * 125 = 12'375
		const auto snap = anchor.snapshot();
		ASSERT_TRUE(snap.valid);

		const auto res = anchor.resolve(snap, 12'450u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(res.using_drdy);
		EXPECT_TRUE(res.consume_anchor);
		EXPECT_EQ(res.timestamp_sample, 12'375u);

		const auto dup = anchor.resolve(snap, 12'500u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_FALSE(dup.using_drdy);
		EXPECT_FALSE(dup.consume_anchor);
		EXPECT_EQ(dup.timestamp_sample, 12'500u);
	}

	// 2. N+1 jitter frame (21 samples) preserves T_20 without 125 us shift.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 10'000u, 20, 21); // T_20 = 12'375, pulse 21 at 12'500
		const auto snap = anchor.snapshot();
		ASSERT_TRUE(snap.valid);

		const auto res = anchor.resolve(snap, 12'560u, 21, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(res.using_drdy);
		EXPECT_TRUE(res.consume_anchor);
		EXPECT_EQ(res.timestamp_sample, 12'375u);
	}

	// 3. 2-interrupt 400 Hz 40-frame backlog: Pass 1 = T_20, Pass 2 = T_40.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 10'000u, 20, 40); // T_20 = 12'375, T_40 = 14'875
		const auto snap1 = anchor.snapshot();
		const auto pass1 = anchor.resolve(snap1, 14'950u, 40, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass1.using_drdy);
		EXPECT_FALSE(pass1.consume_anchor);
		EXPECT_EQ(pass1.timestamp_sample, 12'375u);

		const auto snap2 = anchor.snapshot();
		const auto pass2 = anchor.resolve(snap2, 15'050u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass2.using_drdy);
		EXPECT_TRUE(pass2.consume_anchor);
		EXPECT_EQ(pass2.timestamp_sample, 14'875u);
	}

	// 4. 2-interrupt + extra frames (45 frames at 400 Hz), then remaining 5 + 15 = 20 -> T_60.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 10'000u, 20, 45); // T_20 = 12'375, T_40 = 14'875, 5 tail frames
		const auto snap1 = anchor.snapshot();
		const auto pass1 = anchor.resolve(snap1, 15'580u, 45, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass1.using_drdy);
		EXPECT_FALSE(pass1.consume_anchor);
		EXPECT_EQ(pass1.timestamp_sample, 12'375u);

		const auto snap2 = anchor.snapshot();
		const auto pass2 = anchor.resolve(snap2, 15'680u, 25, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass2.using_drdy);
		EXPECT_TRUE(pass2.consume_anchor);
		EXPECT_EQ(pass2.timestamp_sample, 14'875u);

		feed_pulses(anchor, 10'000u + 45u * 125u, 20, 15); // T_60 = 10'000 + 59 * 125 = 17'375
		const auto snap3 = anchor.snapshot();
		const auto pass3 = anchor.resolve(snap3, 17'440u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass3.using_drdy);
		EXPECT_TRUE(pass3.consume_anchor);
		EXPECT_EQ(pass3.timestamp_sample, 17'375u);
	}

	// 5. 800 Hz 20-frame backlog (N = 10): Pass 1 = T_10, Pass 2 = T_20.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 20'000u, 10, 20); // T_10 = 21'125, T_20 = 22'375
		const auto pass1 = anchor.resolve(anchor.snapshot(), 22'450u, 20, 10, kSampleDtUs, kDelay800Hz);
		EXPECT_TRUE(pass1.using_drdy);
		EXPECT_FALSE(pass1.consume_anchor);
		EXPECT_EQ(pass1.timestamp_sample, 21'125u);

		const auto pass2 = anchor.resolve(anchor.snapshot(), 22'550u, 10, 10, kSampleDtUs, kDelay800Hz);
		EXPECT_TRUE(pass2.using_drdy);
		EXPECT_TRUE(pass2.consume_anchor);
		EXPECT_EQ(pass2.timestamp_sample, 22'375u);
	}

	// 6. 3-interrupt 60-frame backlog (N = 20): Pass 1 = T_20, Pass 2 = T_40, Pass 3 = T_60.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 30'000u, 20, 60); // T_20 = 32'375, T_40 = 34'875, T_60 = 37'375
		const auto pass1 = anchor.resolve(anchor.snapshot(), 37'450u, 60, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass1.using_drdy);
		EXPECT_FALSE(pass1.consume_anchor);
		EXPECT_EQ(pass1.timestamp_sample, 32'375u);

		const auto pass2 = anchor.resolve(anchor.snapshot(), 37'550u, 40, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass2.using_drdy);
		EXPECT_FALSE(pass2.consume_anchor);
		EXPECT_EQ(pass2.timestamp_sample, 34'875u);

		const auto pass3 = anchor.resolve(anchor.snapshot(), 37'650u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass3.using_drdy);
		EXPECT_TRUE(pass3.consume_anchor);
		EXPECT_EQ(pass3.timestamp_sample, 37'375u);
	}

	// 7. ISR racing before WQ consume never clobbers the newly latched anchor.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 40'000u, 20, 20); // T_20 = 42'375
		const auto snap1 = anchor.snapshot();

		// ISR completes the next batch before WQ resolves snap1.
		feed_pulses(anchor, 42'500u, 20, 20); // T_40 = 44'875

		const auto pass1 = anchor.resolve(snap1, 42'450u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass1.using_drdy);
		EXPECT_TRUE(pass1.consume_anchor);
		EXPECT_EQ(pass1.timestamp_sample, 42'375u);

		const auto snap2 = anchor.snapshot();
		const auto pass2 = anchor.resolve(snap2, 44'950u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pass2.using_drdy);
		EXPECT_TRUE(pass2.consume_anchor);
		EXPECT_EQ(pass2.timestamp_sample, 44'875u);
	}

	// 8. Stale / frozen / invalid DRDY falls back cleanly to adjustFifoBacklogTimestamp.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 50'000u, 20, 25); // T_20 = 52'375, 5 extra -> expected_tail = 625 us
		const auto snap = anchor.snapshot();

		// Stale: (55'500 - 52'375) = 3'125 >= 625 + 2'500
		const auto stale = anchor.resolve(snap, 55'500u, 25, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_FALSE(stale.using_drdy);
		EXPECT_FALSE(stale.consume_anchor);
		EXPECT_EQ(stale.timestamp_sample, 54'875u); // 55'500 - 5 * 125

		// Mismatched FIFO count (after_latched = 5 >= available_samples = 5)
		const auto mismatched = anchor.resolve(snap, 53'050u, 5, 5, kSampleDtUs, kDelay400Hz);
		EXPECT_FALSE(mismatched.using_drdy);
		EXPECT_EQ(mismatched.timestamp_sample, 53'050u);

		// Invalid inputs on pure resolver + wrap-safe uint32_t pulse counter
		EXPECT_FALSE(imu::resolveDrdyBatchTimestamp(10'000u, 0u, 20u, 20u, 0u, false, 20, 20, kSampleDtUs, kDelay400Hz).using_drdy);
		EXPECT_FALSE(imu::resolveDrdyBatchTimestamp(10'000u, 10'001u, 20u, 20u, 0u, false, 20, 20, kSampleDtUs, kDelay400Hz).using_drdy);
		EXPECT_FALSE(imu::resolveDrdyBatchTimestamp(10'000u, 9'900u, 20u, 20u, 0u, false, 0, 0, kSampleDtUs, kDelay400Hz).using_drdy);
		EXPECT_FALSE(imu::resolveDrdyBatchTimestamp(10'000u, 9'900u, 20u, 20u, 0u, false, 10, 12, kSampleDtUs, kDelay400Hz).using_drdy);
		EXPECT_FALSE(imu::resolveDrdyBatchTimestamp(10'000u, 9'900u, 20u, 20u, 0u, false, 20, 20, 0.f, kDelay400Hz).using_drdy);
		EXPECT_FALSE(imu::resolveDrdyBatchTimestamp(10'000u, 9'900u, 20u, 20u, 0u, false, 20, 20, NAN, kDelay400Hz).using_drdy);
		const auto nan_backlog = imu::resolveDrdyBatchTimestamp(10'000u, 9'900u, 25u, 20u, 0u, false, 25, 20, NAN, kDelay400Hz);
		EXPECT_FALSE(nan_backlog.using_drdy);
		EXPECT_EQ(nan_backlog.timestamp_sample, 10'000u);
		EXPECT_EQ(imu::adjustFifoBacklogTimestamp(10'000u, false, 25, 20, NAN), 10'000u);
		EXPECT_EQ(imu::adjustFifoBacklogTimestamp(10'000u, false, 25, 20, INFINITY), 10'000u);
		EXPECT_EQ(imu::SpiDmaBuffer::alignSize(SIZE_MAX), 0u);
		EXPECT_EQ(imu::SpiDmaBuffer::alignSize(SIZE_MAX - 10u, 32u), 0u);
		EXPECT_EQ(imu::SpiDmaBuffer::alignSize(33u, 32u), 64u);

		const auto wrapped = imu::resolveDrdyBatchTimestamp(
					     61'300u, 60'000u, 0x00000002u, 0xfffffff8u, 0u, false, 25, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(wrapped.using_drdy);
		EXPECT_TRUE(wrapped.consume_anchor); // latched_sample = 25 - 10 = 15, read 20 >= 15
		EXPECT_EQ(wrapped.timestamp_sample, 60'625u); // 60'000 + 5 * 125

		const auto wrapped_capped = imu::resolveDrdyBatchTimestamp(
						    60'600u, 60'000u, 0x00000002u, 0xfffffff8u, 0u, false, 25, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(wrapped_capped.using_drdy);
		EXPECT_TRUE(wrapped_capped.consume_anchor);
		EXPECT_EQ(wrapped_capped.timestamp_sample, 60'600u);
	}

	// 9. Reset clears latched/consumed state and allows re-latching at the same pulse count.
	{
		imu::DrdySampleAnchor anchor;
		feed_pulses(anchor, 70'000u, 20, 20); // T_20 = 72'375, consumed_pulse = 20
		const auto pre_reset = anchor.resolve(anchor.snapshot(), 72'450u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(pre_reset.using_drdy);
		EXPECT_EQ(pre_reset.timestamp_sample, 72'375u);

		anchor.reset();
		const auto empty_snap = anchor.snapshot();
		EXPECT_FALSE(empty_snap.valid);
		EXPECT_FALSE(anchor.resolve(empty_snap, 73'000u, 20, 20, kSampleDtUs, kDelay400Hz).using_drdy);

		feed_pulses(anchor, 75'000u, 20, 19);
		EXPECT_FALSE(anchor.snapshot().valid);

		anchor.onInterrupt(77'375u, 20); // 20th pulse after reset -> pulse_count == 20 again
		const auto post_reset = anchor.resolve(anchor.snapshot(), 77'450u, 20, 20, kSampleDtUs, kDelay400Hz);
		EXPECT_TRUE(post_reset.using_drdy);
		EXPECT_TRUE(post_reset.consume_anchor);
		EXPECT_EQ(post_reset.timestamp_sample, 77'375u);
	}
}

/*
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ptgl100_parser.hpp"

#include <gtest/gtest.h>
#include <array>

using ptgl100::Parser;
using ptgl100::Reply;

namespace
{

using Frame = std::array<uint8_t, ptgl100::FRAME_SIZE>;

Frame reply_frame(uint8_t command, uint8_t status, uint32_t distance)
{
	Frame frame{0x55, 0xaa, command, status, static_cast<uint8_t>(distance >> 16), static_cast<uint8_t>(distance >> 8), static_cast<uint8_t>(distance), 0};

	for (size_t i = 0; i < frame.size() - 1; ++i) {
		frame.back() += frame[i];
	}

	return frame;
}

Parser::Result feed(Parser &parser, const Frame &frame, Reply &reply)
{
	Parser::Result result = Parser::Result::Incomplete;

	for (uint8_t byte : frame) {
		result = parser.parse(byte, reply);
	}

	return result;
}

} // namespace

TEST(PTGL100Parser, MeasurementCommandsMatchManual)
{
	uint8_t frame[ptgl100::FRAME_SIZE];
	EXPECT_EQ(ptgl100::SINGLE_MEASUREMENT, 0x68);
	EXPECT_EQ(ptgl100::CONTINUOUS_MEASUREMENT, 0x69);
	EXPECT_EQ(ptgl100::STOP_MEASUREMENT, 0x8e);

	for (const auto &entry : {std::array<uint8_t, 2> {0x68, 0x64}, {0x69, 0x65}, {0x8e, 0x8a}}) {
		ptgl100::make_command(entry[0], frame);
		const Frame expected{0x55, 0xaa, entry[0], 0xff, 0xff, 0xff, 0xff, entry[1]};

		for (size_t i = 0; i < expected.size(); ++i) {
			EXPECT_EQ(frame[i], expected[i]);
		}
	}
}

TEST(PTGL100Parser, GoldenReplyAndBigEndianDistance)
{
	Parser parser;
	Reply reply;
	// 0x0186a0 = 100000 mm => 100 metres; all three distance bytes matter.
	const Frame frame{0x55, 0xaa, 0x69, 0x01, 0x01, 0x86, 0xa0, 0x90};
	ASSERT_EQ(feed(parser, frame, reply), Parser::Result::Reply);
	EXPECT_TRUE(reply.is_measurement());
	EXPECT_EQ(reply.status, ptgl100::STATUS_SUCCESS);
	EXPECT_EQ(reply.distance_raw, 100000u);
	EXPECT_FLOAT_EQ(ptgl100::distance_m(reply.distance_raw), 100.f);
}

TEST(PTGL100Parser, ReceiveChecksumIncludesHeader)
{
	Parser parser;
	Reply reply;
	// A transmitted command echoed back must not pass the reply checksum.
	EXPECT_EQ(feed(parser, {0x55, 0xaa, 0x69, 0xff, 0xff, 0xff, 0xff, 0x65}, reply), Parser::Result::BadChecksum);
	EXPECT_EQ(feed(parser, {0x55, 0xaa, 0x69, 0x01, 0x00, 0x00, 0x64, 0xcd}, reply), Parser::Result::Reply);
	EXPECT_EQ(reply.distance_raw, 100);
}

TEST(PTGL100Parser, PreservesRangeBoundariesAndOutOfRangeValues)
{
	Parser parser;
	Reply reply;

	for (uint32_t raw : {0u, 199u, 200u, 65535u, 65536u, 100000u, 100001u, 0xffffffu}) {
		ASSERT_EQ(feed(parser, reply_frame(0x69, 1, raw), reply), Parser::Result::Reply);
		EXPECT_EQ(reply.distance_raw, raw);
	}

	EXPECT_FLOAT_EQ(ptgl100::distance_m(200), ptgl100::MIN_DISTANCE_M);
	EXPECT_FLOAT_EQ(ptgl100::distance_m(100000), ptgl100::MAX_DISTANCE_M);
	EXPECT_FLOAT_EQ(ptgl100::distance_m(12345), 12.345f);
}

TEST(PTGL100Parser, HandlesEveryReadBoundary)
{
	const Frame frame = reply_frame(0x69, 1, 123456);

	for (size_t split = 1; split < frame.size(); ++split) {
		Parser parser;
		Reply reply;

		for (size_t i = 0; i < split; ++i) {
			EXPECT_EQ(parser.parse(frame[i], reply), Parser::Result::Incomplete);
		}

		for (size_t i = split; i < frame.size(); ++i) {
			EXPECT_EQ(parser.parse(frame[i], reply), i == frame.size() - 1 ? Parser::Result::Reply : Parser::Result::Incomplete);
		}

		EXPECT_EQ(reply.distance_raw, 123456u);
	}
}

TEST(PTGL100Parser, RuntimeConversionKeepsInclusiveRangeLimits)
{
	// Prevent constant folding: the actual UART path uses the runtime conversion.
	volatile uint32_t raw = 200;
	EXPECT_EQ(ptgl100::distance_m(raw), ptgl100::MIN_DISTANCE_M);
	raw = 100000;
	EXPECT_EQ(ptgl100::distance_m(raw), ptgl100::MAX_DISTANCE_M);
}

TEST(PTGL100Parser, BackToBackSingleAndContinuousMeasurements)
{
	Parser parser;
	Reply reply;

	for (uint8_t command : {0x68, 0x69, 0x69}) {
		ASSERT_EQ(feed(parser, reply_frame(command, 1, 50), reply), Parser::Result::Reply);
		EXPECT_EQ(reply.command, command);
		EXPECT_TRUE(reply.is_measurement());
	}
}

TEST(PTGL100Parser, FailedMeasurementsRemainFailures)
{
	Parser parser;
	Reply reply;

	for (uint8_t status : {0, 2, 255}) {
		ASSERT_EQ(feed(parser, reply_frame(0x69, status, 0xffffff), reply), Parser::Result::Reply);
		EXPECT_NE(reply.status, ptgl100::STATUS_SUCCESS);
		EXPECT_TRUE(reply.is_measurement());
	}
}

TEST(PTGL100Parser, SelfTestAndControlRepliesAreNotMeasurements)
{
	Parser parser;
	Reply reply;
	ASSERT_EQ(feed(parser, {0x55, 0xaa, 0x80, 0x00, 0x00, 0x00, 0x03, 0x82}, reply), Parser::Result::Reply);
	EXPECT_FALSE(reply.is_measurement());
	EXPECT_EQ(reply.status, 0);
	EXPECT_EQ(reply.error_code, 3);

	ASSERT_EQ(feed(parser, {0x55, 0xaa, 0x8e, 0x01, 0xff, 0xff, 0xff, 0x8b}, reply), Parser::Result::Reply);
	EXPECT_FALSE(reply.is_measurement());
}

TEST(PTGL100Parser, RejectsUnknownCommandAndBadChecksum)
{
	Parser parser;
	Reply reply;

	// Reject old measurement formats and the baud command removed from the manual.
	for (uint8_t command : {0x88, 0x89, 0x01, 0x90}) {
		EXPECT_EQ(feed(parser, reply_frame(command, 1, 30), reply), Parser::Result::UnknownCommand);
	}

	Frame frame = reply_frame(0x69, 1, 30);
	frame.back() ^= 1;
	EXPECT_EQ(feed(parser, frame, reply), Parser::Result::BadChecksum);
	EXPECT_EQ(feed(parser, reply_frame(0x69, 1, 40), reply), Parser::Result::Reply);
	EXPECT_EQ(reply.distance_raw, 40);
}

TEST(PTGL100Parser, NoiseAndRepeatedFirstHeaderByte)
{
	Parser parser;
	Reply reply;

	for (uint8_t byte : {0, 0xaa, 0x55, 0x55, 0, 0x55}) {
		EXPECT_EQ(parser.parse(byte, reply), Parser::Result::Incomplete);
	}

	EXPECT_EQ(feed(parser, reply_frame(0x69, 1, 50), reply), Parser::Result::Reply);
	EXPECT_EQ(reply.distance_raw, 50);
}

TEST(PTGL100Parser, RecoversHeaderInsideTruncatedFrame)
{
	const Frame frame = reply_frame(0x69, 1, 12345);

	for (size_t truncated_size = 2; truncated_size < frame.size(); ++truncated_size) {
		Parser parser;
		Reply reply;

		for (size_t i = 0; i < truncated_size; ++i) {
			parser.parse(frame[i], reply);
		}

		EXPECT_EQ(feed(parser, frame, reply), Parser::Result::Reply) << truncated_size;
		EXPECT_EQ(reply.distance_raw, 12345u);
	}
}

TEST(PTGL100Parser, RecoversAfterTruncationWithChecksumCollision)
{
	Parser parser;
	Reply reply;
	const Frame frame = reply_frame(0x69, 1, 100000);

	for (size_t i = 0; i < 5; ++i) {
		parser.parse(frame[i], reply);
	}

	// The truncated prefix plus the next header forms 55 aa 69 01 01 55 aa 69,
	// which happens to pass the protocol's additive checksum. It is indistinguishable
	// from a real measurement, but must not prevent recovery on the following frame.
	feed(parser, frame, reply);
	ASSERT_EQ(feed(parser, frame, reply), Parser::Result::Reply);
	EXPECT_EQ(reply.distance_raw, 100000u);
}

TEST(PTGL100Parser, HeaderBytesInDistanceDoNotRestartValidFrame)
{
	Parser parser;
	Reply reply;

	for (uint32_t raw : {0x0155aau, 0x55aa01u}) {
		EXPECT_EQ(feed(parser, reply_frame(0x69, 1, raw), reply), Parser::Result::Reply);
		EXPECT_EQ(reply.distance_raw, raw);
	}
}

TEST(PTGL100Parser, ResetDiscardsPartialFrame)
{
	Parser parser;
	Reply reply;
	parser.parse(0x55, reply);
	parser.parse(0xaa, reply);
	parser.parse(0x69, reply);
	parser.reset();
	EXPECT_EQ(feed(parser, reply_frame(0x69, 1, 50), reply), Parser::Result::Reply);
}

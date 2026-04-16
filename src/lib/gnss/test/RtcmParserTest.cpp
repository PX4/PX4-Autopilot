/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file RtcmParserTest.cpp
 *
 * Tests for RTCM3 parser with valid and partially valid input.
 * Consolidates: GoodSender, PartialSender, Helper, Buffer, Stats, EdgeCase tests.
 */

#include "RtcmTestCommon.hpp"

// =============================================================================
// Helper function tests
// =============================================================================

TEST_F(RtcmTest, PayloadLength_MasksReservedBits)
{
	// Reserved bits set to 1, length should still be extracted correctly
	uint8_t frame[] = {RTCM3_PREAMBLE, 0xFF, 0xFF};  // Reserved=0x3F, Length=1023
	EXPECT_EQ(rtcm3_payload_length(frame), 1023u);
}

TEST_F(RtcmTest, MessageType_Extraction)
{
	auto frame = buildValidFrame(1005, {});
	EXPECT_EQ(rtcm3_message_type(frame.data()), 1005u);
}

// =============================================================================
// Good sender tests - valid frames, happy path
// =============================================================================

TEST_F(RtcmTest, SingleMinimalFrame)
{
	auto frame = buildRawFrame({0x00, 0x00});
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	const uint8_t *msg = parser.getNextMessage(&len);

	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(len, frame.size());

	parser.consumeMessage(len);
	auto stats = parser.getStats();
	EXPECT_EQ(stats.messages_parsed, 1u);
	EXPECT_EQ(stats.crc_errors, 0u);
}

TEST_F(RtcmTest, SingleMaximalFrame)
{
	std::vector<uint8_t> payload(RTCM3_MAX_PAYLOAD_LEN, 0xAA);
	auto frame = buildRawFrame(payload);
	parser.addData(frame.data(), frame.size());

	size_t len = 0;
	const uint8_t *msg = parser.getNextMessage(&len);

	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(len, RTCM3_MAX_FRAME_LEN);
}

TEST_F(RtcmTest, MultipleFramesInSequence)
{
	auto frame1 = buildValidFrame(1005, {0x01, 0x02, 0x03});
	auto frame2 = buildValidFrame(1077, {0x04, 0x05, 0x06, 0x07});
	auto frame3 = buildValidFrame(1087, {0x08});

	parser.addData(frame1.data(), frame1.size());
	parser.addData(frame2.data(), frame2.size());
	parser.addData(frame3.data(), frame3.size());

	size_t len;
	int count = 0;

	while (parser.getNextMessage(&len) != nullptr) {
		parser.consumeMessage(len);
		count++;
	}

	EXPECT_EQ(count, 3);
}

TEST_F(RtcmTest, FrameArrivesInChunks)
{
	auto frame = buildValidFrame(1005, {0x01, 0x02, 0x03, 0x04, 0x05});

	// Feed one byte at a time
	for (size_t i = 0; i < frame.size() - 1; i++) {
		parser.addData(&frame[i], 1);
		size_t len;
		EXPECT_EQ(parser.getNextMessage(&len), nullptr);
	}

	parser.addData(&frame[frame.size() - 1], 1);

	size_t len;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(len, frame.size());
}

// =============================================================================
// Partially good sender tests - mix of valid and invalid
// =============================================================================

TEST_F(RtcmTest, GarbageBeforeValidFrame)
{
	std::vector<uint8_t> data;

	// 50 bytes of garbage
	for (int i = 0; i < 50; i++) {
		data.push_back(0x00 + i);
	}

	auto frame = buildValidFrame(1005, {0x01, 0x02});
	data.insert(data.end(), frame.begin(), frame.end());

	parser.addData(data.data(), data.size());

	size_t len;
	const uint8_t *msg = parser.getNextMessage(&len);
	ASSERT_NE(msg, nullptr);
	EXPECT_EQ(rtcm3_message_type(msg), 1005u);

	parser.consumeMessage(len);
	EXPECT_EQ(parser.getStats().bytes_discarded, 50u);
}

TEST_F(RtcmTest, ValidFrameBadFrameValid)
{
	auto frame1 = buildValidFrame(1005, {0x01});
	auto bad_frame = buildBadCrcFrame({0x02, 0x03});
	auto frame2 = buildValidFrame(1077, {0x04});

	parser.addData(frame1.data(), frame1.size());
	parser.addData(bad_frame.data(), bad_frame.size());
	parser.addData(frame2.data(), frame2.size());

	size_t len;
	int valid_count = 0;

	while (parser.getNextMessage(&len) != nullptr) {
		parser.consumeMessage(len);
		valid_count++;
	}

	EXPECT_EQ(valid_count, 2);
	EXPECT_EQ(parser.getStats().crc_errors, 1u);
}

// =============================================================================
// Buffer and edge case tests
// =============================================================================

TEST_F(RtcmTest, BufferOverflowProtection)
{
	std::vector<uint8_t> large_data(Rtcm3Parser::BUFFER_SIZE + 1000, 0xAA);
	size_t added = parser.addData(large_data.data(), large_data.size());

	EXPECT_EQ(added, Rtcm3Parser::BUFFER_SIZE);
}

TEST_F(RtcmTest, EmptyPayloadFrame)
{
	auto frame = buildRawFrame({});
	parser.addData(frame.data(), frame.size());

	size_t len;
	ASSERT_NE(parser.getNextMessage(&len), nullptr);
	EXPECT_EQ(len, RTCM3_HEADER_LEN + RTCM3_CRC_LEN);
}

TEST_F(RtcmTest, GetNextOnEmptyBuffer)
{
	size_t len;
	EXPECT_EQ(parser.getNextMessage(&len), nullptr);
}

// =============================================================================
// GPS_RTCM_DATA fragmentation tests
// =============================================================================

// WHAT: Verify that a normal unfragmented payload is passed through unchanged.
// WHY: The assembler must ignore sequence bits when the fragmented flag is not set.
TEST_F(RtcmTest, GpsRtcmAssemblerReturnsUnfragmentedPayload)
{
	// GIVEN: A small payload that fits in one packet.
	const std::vector<uint8_t> payload {0x01, 0x02, 0x03, 0x04};
	size_t len = 0;

	// WHEN: The payload is added without the fragmented flag.
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(false, 0, 9), payload.data(), payload.size(), 10,
				 len);

	// THEN: The assembler returns the full payload unchanged.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that the unfragmented path accepts the largest payload the helper stores.
// WHY: This keeps the direct helper's explicit size guard covered even though
// MAVLink GPS_RTCM_DATA packets top out at 180 payload bytes.
TEST_F(RtcmTest, GpsRtcmAssemblerAcceptsMaxSizeUnfragmentedPayload)
{
	// GIVEN: A payload that is exactly as large as the assembler's internal
	// message buffer. This exceeds what MAVLink can carry in one packet, but the
	// direct helper still needs a checked upper bound.
	std::vector<uint8_t> payload(GPS_RTCM_MAX_MESSAGE_LEN);
	std::iota(payload.begin(), payload.end(), 0);
	size_t len = 0;

	// WHEN: The payload is added as a single unfragmented packet.
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(false), payload.data(), payload.size(), 10, len);

	// THEN: The assembler accepts it and returns the original bytes unchanged.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that oversized fragmented packets are rejected.
// WHY: GPS_RTCM_DATA only allows up to 180 bytes per fragment, so larger packets must be dropped.
TEST_F(RtcmTest, GpsRtcmAssemblerRejectsOversizedFragment)
{
	// GIVEN: A fragmented packet that exceeds the protocol fragment size limit.
	std::vector<uint8_t> oversized_fragment(GPS_RTCM_MAX_FRAGMENT_LEN + 1, 0x5a);
	size_t len = 123;

	// WHEN: The oversized fragment is passed to the assembler.
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 6), oversized_fragment.data(),
				 oversized_fragment.size(), 10, len);

	// THEN: The fragment is rejected and no payload is returned.
	EXPECT_EQ(message, nullptr);
	EXPECT_EQ(len, 0u);
}

// WHAT: Verify that a short RTCM message is emitted as one unfragmented MAVLink packet.
// WHY: A single-packet message should keep the fragmented bit clear while still
// using fragment ID 0 and the current sequence ID.
TEST_F(RtcmTest, GpsRtcmFragmenterReturnsSingleUnfragmentedPacket)
{
	std::vector<uint8_t> payload(64);
	std::iota(payload.begin(), payload.end(), 0);

	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(payload.data(), payload.size()));

	uint8_t flags = 0xff;
	const uint8_t *packet = nullptr;
	size_t len = 0;

	ASSERT_TRUE(gps_rtcm_fragmenter.nextPacket(flags, packet, len));
	EXPECT_FALSE(gps_rtcm_is_fragmented(flags));
	EXPECT_EQ(gps_rtcm_fragment_id(flags), 0u);
	EXPECT_EQ(gps_rtcm_sequence_id(flags), 0u);
	ASSERT_NE(packet, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(packet, payload.data(), payload.size()), 0);
	EXPECT_FALSE(gps_rtcm_fragmenter.active());
	EXPECT_FALSE(gps_rtcm_fragmenter.nextPacket(flags, packet, len));
}

// WHAT: Verify the number of packets produced at the packet-count boundaries.
// WHY: This covers the helper that decides when to add a terminating empty
// fragment for exact multiples below 720 bytes.
struct GpsRtcmPacketCountCase {
	const char *name;
	size_t message_len;
	uint8_t expected_packets;
};

class GpsRtcmPacketCountTest : public RtcmTest, public ::testing::WithParamInterface<GpsRtcmPacketCountCase> {};

TEST_P(GpsRtcmPacketCountTest, UsesExpectedPacketCountAtBoundaries)
{
	const auto &param = GetParam();
	SCOPED_TRACE(param.name);

	// GIVEN: An RTCM message with a boundary-case length.
	std::vector<uint8_t> payload(param.message_len, 0x5a);
	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(payload.data(), payload.size()));

	// WHEN: All MAVLink packets for that message are emitted.
	uint8_t flags = 0;
	const uint8_t *packet = nullptr;
	size_t len = 0;
	uint8_t packet_count = 0;

	while (gps_rtcm_fragmenter.nextPacket(flags, packet, len)) {
		packet_count++;
	}

	// THEN: The packet count matches the expected boundary behavior.
	EXPECT_EQ(packet_count, param.expected_packets);
}

INSTANTIATE_TEST_SUITE_P(BoundaryCases, GpsRtcmPacketCountTest,
			 ::testing::Values(
				 GpsRtcmPacketCountCase{"Below180", 179, 1},
				 GpsRtcmPacketCountCase{"At180", 180, 1},
				 GpsRtcmPacketCountCase{"Above180", 181, 2},
				 GpsRtcmPacketCountCase{"ExactMultipleBelow720", 360, 3},
				 GpsRtcmPacketCountCase{"At720", 720, 4}),
			 [](const testing::TestParamInfo<GpsRtcmPacketCountCase> &param_info)
{
	return param_info.param.name;
});

// WHAT: Verify that a fragmented outbound packet stream round-trips through the assembler.
// WHY: The transmit and receive helpers should agree on the MAVLink GPS_RTCM_DATA contract.
TEST_F(RtcmTest, GpsRtcmFragmenterRoundTripsThroughAssembler)
{
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN * 2 + 37);
	std::iota(payload.begin(), payload.end(), 0);

	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(payload.data(), payload.size()));

	uint8_t flags = 0;
	const uint8_t *packet = nullptr;
	size_t len = 0;
	size_t assembled_len = 0;
	const uint8_t *assembled = nullptr;
	int packet_count = 0;

	while (gps_rtcm_fragmenter.nextPacket(flags, packet, len)) {
		assembled = gps_rtcm_assembler.addPacket(flags, packet, len, 10 + packet_count, assembled_len);
		packet_count++;
	}

	ASSERT_NE(assembled, nullptr);
	EXPECT_EQ(packet_count, 3);
	EXPECT_EQ(assembled_len, payload.size());
	EXPECT_EQ(memcmp(assembled, payload.data(), payload.size()), 0);
}

// WHAT: Verify that exact multiples of 180 bytes below 720 bytes emit a zero-length terminator.
// WHY: Per the MAVLink rule, the first fragment with a non-full payload marks
// the end of the buffer, so an exact multiple below 720 bytes needs an empty
// fragment to provide that boundary.
TEST_F(RtcmTest, GpsRtcmFragmenterAddsZeroLengthTerminator)
{
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN * 3);
	std::iota(payload.begin(), payload.end(), 0);

	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(payload.data(), payload.size()));

	std::vector<size_t> packet_lengths;
	std::vector<uint8_t> packet_flags;
	uint8_t flags = 0;
	const uint8_t *packet = nullptr;
	size_t len = 0;
	size_t assembled_len = 0;
	const uint8_t *assembled = nullptr;

	while (gps_rtcm_fragmenter.nextPacket(flags, packet, len)) {
		packet_lengths.push_back(len);
		packet_flags.push_back(flags);
		assembled = gps_rtcm_assembler.addPacket(flags, packet, len, 100 + packet_lengths.size(), assembled_len);
	}

	ASSERT_EQ(packet_lengths.size(), 4u);
	EXPECT_EQ(packet_lengths[0], GPS_RTCM_MAX_FRAGMENT_LEN);
	EXPECT_EQ(packet_lengths[1], GPS_RTCM_MAX_FRAGMENT_LEN);
	EXPECT_EQ(packet_lengths[2], GPS_RTCM_MAX_FRAGMENT_LEN);
	EXPECT_EQ(packet_lengths[3], 0u);
	EXPECT_EQ(gps_rtcm_fragment_id(packet_flags[0]), 0u);
	EXPECT_EQ(gps_rtcm_fragment_id(packet_flags[1]), 1u);
	EXPECT_EQ(gps_rtcm_fragment_id(packet_flags[2]), 2u);
	EXPECT_EQ(gps_rtcm_fragment_id(packet_flags[3]), 3u);
	EXPECT_EQ(gps_rtcm_sequence_id(packet_flags[0]), gps_rtcm_sequence_id(packet_flags[3]));
	ASSERT_NE(assembled, nullptr);
	EXPECT_EQ(assembled_len, payload.size());
	EXPECT_EQ(memcmp(assembled, payload.data(), payload.size()), 0);
}

// WHAT: Verify the full uORB -> RTCM parser -> MAVLink fragmenter ->
// MAVLink assembler loop across the MAVLink fragmentation boundary cases.
// WHY: This tests the end-to-end transport coverage focused on the boundary
// values that matter.
struct UorbRtcmRoundTripCase {
	const char *name;
	size_t frame_len;
	std::vector<size_t> expected_packet_lengths;
	bool expect_fragmented;
};

class RtcmUorbRoundTripTest : public RtcmTest, public ::testing::WithParamInterface<UorbRtcmRoundTripCase> {};

TEST_P(RtcmUorbRoundTripTest, ThroughMavlinkFragmentation)
{
	const auto &param = GetParam();
	SCOPED_TRACE(param.name);

	constexpr size_t uorb_chunk_capacity = 300;

	const auto chunk_bytes = [uorb_chunk_capacity](const uint8_t *data, size_t len) {
		std::vector<std::vector<uint8_t>> chunks;

		for (size_t offset = 0; offset < len; offset += uorb_chunk_capacity) {
			const size_t chunk_len = (len - offset < uorb_chunk_capacity) ? (len - offset) : uorb_chunk_capacity;
			chunks.emplace_back(data + offset, data + offset + chunk_len);
		}

		return chunks;
	};

	// GIVEN: A boundary-case RTCM frame length
	const size_t payload_len = param.frame_len - RTCM3_HEADER_LEN - RTCM3_CRC_LEN;
	std::vector<uint8_t> payload(payload_len);

	for (size_t i = 0; i < payload.size(); i++) {
		payload[i] = static_cast<uint8_t>((param.frame_len + i) & 0xff);
	}

	const std::vector<uint8_t> frame = buildRawFrame(payload);
	ASSERT_EQ(frame.size(), param.frame_len);

	const std::vector<std::vector<uint8_t>> uorb_chunks = chunk_bytes(frame.data(), frame.size());

	Rtcm3Parser roundtrip_parser;
	GpsRtcmMessageAssembler roundtrip_assembler;
	GpsRtcmMessageFragmenter roundtrip_fragmenter;

	size_t parsed_len = 0;
	const uint8_t *parsed_frame = nullptr;

	// WHEN: The uORB-sized chunks are fed into the RTCM parser.
	for (size_t i = 0; i < uorb_chunks.size(); i++) {
		const auto &chunk = uorb_chunks[i];
		ASSERT_EQ(roundtrip_parser.addData(chunk.data(), chunk.size()), chunk.size());
		const uint8_t *candidate = roundtrip_parser.getNextMessage(&parsed_len);

		// THEN: The RTCM frame is only complete after the last uORB chunk.
		if (i + 1 < uorb_chunks.size()) {
			EXPECT_EQ(candidate, nullptr);

		} else {
			parsed_frame = candidate;
		}
	}

	// THEN: The parser returns the original RTCM frame.
	ASSERT_NE(parsed_frame, nullptr);
	EXPECT_EQ(parsed_len, frame.size());

	// WHEN: The parsed RTCM frame is fragmented for MAVLink transport.
	ASSERT_TRUE(roundtrip_fragmenter.startMessage(parsed_frame, parsed_len));
	roundtrip_parser.consumeMessage(parsed_len);

	uint8_t flags = 0;
	const uint8_t *packet = nullptr;
	size_t packet_len = 0;
	size_t assembled_len = 0;
	const uint8_t *assembled = nullptr;
	uint64_t timestamp = 100;
	std::vector<size_t> packet_lengths;
	std::vector<uint8_t> packet_flags;

	// WHEN: The emitted MAVLink packets are reassembled back into one frame.
	while (roundtrip_fragmenter.nextPacket(flags, packet, packet_len)) {
		packet_lengths.push_back(packet_len);
		packet_flags.push_back(flags);
		assembled = roundtrip_assembler.addPacket(flags, packet, packet_len, timestamp++, assembled_len);
	}

	// THEN: The MAVLink packet lengths match the boundary-case expectation.
	ASSERT_EQ(packet_lengths, param.expected_packet_lengths);

	if (param.expect_fragmented) {
		for (uint8_t packet_flag : packet_flags) {
			EXPECT_TRUE(gps_rtcm_is_fragmented(packet_flag));
		}

	} else {
		ASSERT_EQ(packet_flags.size(), 1u);
		EXPECT_FALSE(gps_rtcm_is_fragmented(packet_flags[0]));
		EXPECT_EQ(gps_rtcm_fragment_id(packet_flags[0]), 0u);
	}

	// THEN: MAVLink reassembly returns the original RTCM frame.
	ASSERT_NE(assembled, nullptr);
	EXPECT_EQ(assembled_len, frame.size());
	EXPECT_EQ(memcmp(assembled, frame.data(), frame.size()), 0);

	// WHEN: The reassembled frame is split back into uORB-sized chunks.
	const std::vector<std::vector<uint8_t>> reconstructed_uorb_chunks = chunk_bytes(assembled, assembled_len);

	// THEN: The uORB chunking is preserved across the full round trip.
	EXPECT_EQ(reconstructed_uorb_chunks, uorb_chunks);
}

INSTANTIATE_TEST_SUITE_P(BoundaryCases, RtcmUorbRoundTripTest,
			 ::testing::Values(
				 UorbRtcmRoundTripCase{"Below180", 179, std::vector<size_t>{179}, false},
				 UorbRtcmRoundTripCase{"At180", 180, std::vector<size_t>{180}, false},
				 UorbRtcmRoundTripCase{"Above180", 181, std::vector<size_t>{180, 1}, true},
				 UorbRtcmRoundTripCase{"ExactMultipleBelow720", 360, std::vector<size_t>{180, 180, 0}, true},
				 UorbRtcmRoundTripCase{"At720", 720, std::vector<size_t>{180, 180, 180, 180}, true}),
			 [](const testing::TestParamInfo<UorbRtcmRoundTripCase> &param_info)
{
	return param_info.param.name;
});

// WHAT: Verify that the maximum MAVLink-capable RTCM message uses four full fragments with no terminator.
// WHY: This is the "all 4 fragments are present" branch of the MAVLink rule.
TEST_F(RtcmTest, GpsRtcmFragmenterUsesFourFullFragmentsAt720Bytes)
{
	std::vector<uint8_t> payload(GPS_RTCM_MAX_MESSAGE_LEN);
	std::iota(payload.begin(), payload.end(), 0);

	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(payload.data(), payload.size()));

	std::vector<size_t> packet_lengths;
	uint8_t flags = 0;
	const uint8_t *packet = nullptr;
	size_t len = 0;
	size_t assembled_len = 0;
	const uint8_t *assembled = nullptr;

	while (gps_rtcm_fragmenter.nextPacket(flags, packet, len)) {
		packet_lengths.push_back(len);
		assembled = gps_rtcm_assembler.addPacket(flags, packet, len, 200 + packet_lengths.size(), assembled_len);
	}

	ASSERT_EQ(packet_lengths.size(), GPS_RTCM_MAX_FRAGMENTS);
	EXPECT_EQ(packet_lengths[0], GPS_RTCM_MAX_FRAGMENT_LEN);
	EXPECT_EQ(packet_lengths[1], GPS_RTCM_MAX_FRAGMENT_LEN);
	EXPECT_EQ(packet_lengths[2], GPS_RTCM_MAX_FRAGMENT_LEN);
	EXPECT_EQ(packet_lengths[3], GPS_RTCM_MAX_FRAGMENT_LEN);
	ASSERT_NE(assembled, nullptr);
	EXPECT_EQ(assembled_len, payload.size());
	EXPECT_EQ(memcmp(assembled, payload.data(), payload.size()), 0);
}

// WHAT: Verify that every new message consumes a sequence ID.
// WHY: Both fragmented and unfragmented packets carry a sequence ID to avoid message corruption.
TEST_F(RtcmTest, GpsRtcmFragmenterAdvancesSequenceIdForEveryMessage)
{
	std::vector<uint8_t> fragmented_payload(GPS_RTCM_MAX_FRAGMENT_LEN + 1, 0x42);
	std::vector<uint8_t> unfragmented_payload(64, 0x24);

	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(fragmented_payload.data(), fragmented_payload.size()));

	uint8_t first_flags = 0;
	const uint8_t *packet = nullptr;
	size_t len = 0;
	ASSERT_TRUE(gps_rtcm_fragmenter.nextPacket(first_flags, packet, len));

	while (gps_rtcm_fragmenter.nextPacket(first_flags, packet, len)) {}

	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(unfragmented_payload.data(), unfragmented_payload.size()));

	uint8_t unfragmented_flags = 0xff;
	ASSERT_TRUE(gps_rtcm_fragmenter.nextPacket(unfragmented_flags, packet, len));
	EXPECT_FALSE(gps_rtcm_is_fragmented(unfragmented_flags));
	EXPECT_EQ(gps_rtcm_fragment_id(unfragmented_flags), 0u);
	EXPECT_EQ((gps_rtcm_sequence_id(first_flags) + 1) & GPS_RTCM_FLAG_SEQUENCE_ID_MASK,
		  gps_rtcm_sequence_id(unfragmented_flags));

	ASSERT_TRUE(gps_rtcm_fragmenter.startMessage(fragmented_payload.data(), fragmented_payload.size()));

	uint8_t second_flags = 0;
	ASSERT_TRUE(gps_rtcm_fragmenter.nextPacket(second_flags, packet, len));
	EXPECT_EQ((gps_rtcm_sequence_id(first_flags) + 2) & GPS_RTCM_FLAG_SEQUENCE_ID_MASK,
		  gps_rtcm_sequence_id(second_flags));
}

// WHAT: Verify that messages larger than the MAVLink transport capacity are rejected.
// WHY: GPS_RTCM_DATA can carry at most 4 x 180 bytes.
TEST_F(RtcmTest, GpsRtcmFragmenterRejectsOversizedMessage)
{
	std::vector<uint8_t> payload(GPS_RTCM_MAX_MESSAGE_LEN + 1, 0x5a);
	EXPECT_FALSE(gps_rtcm_fragmenter.startMessage(payload.data(), payload.size()));
}


// WHAT: Verify that fragments can arrive out of order and still be reassembled correctly.
// WHY: The sequence/fragment handling exists to allow for unreliable delivery order.
TEST_F(RtcmTest, GpsRtcmAssemblerReordersFragments)
{
	// GIVEN: A payload that spans three fragments with a non-full final fragment.
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN * 2 + 25);
	std::iota(payload.begin(), payload.end(), 0);
	const uint8_t flags0 = buildGpsRtcmFlags(true, 0, 7);
	const uint8_t flags1 = buildGpsRtcmFlags(true, 1, 7);
	const uint8_t flags2 = buildGpsRtcmFlags(true, 2, 7);

	size_t len = 0;

	// WHEN: The later fragments arrive before fragment 0.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(flags1, &payload[GPS_RTCM_MAX_FRAGMENT_LEN], GPS_RTCM_MAX_FRAGMENT_LEN, 10, len),
		  nullptr);
	EXPECT_EQ(gps_rtcm_assembler.addPacket(flags2, &payload[GPS_RTCM_MAX_FRAGMENT_LEN * 2], 25, 20, len), nullptr);

	// WHEN: Fragment 0 finally arrives.
	const uint8_t *message = gps_rtcm_assembler.addPacket(flags0, payload.data(), GPS_RTCM_MAX_FRAGMENT_LEN, 30, len);

	// THEN: The assembler emits the full payload in the correct order.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that a duplicate fragment with identical content is handled.
// WHY: Retransmissions should not reset a healthy buffer or corrupt the assembled payload.
TEST_F(RtcmTest, GpsRtcmAssemblerIgnoresDuplicateMatchingFragment)
{
	// GIVEN: A payload that spans one full fragment and one non-full fragment.
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN + 12);
	std::iota(payload.begin(), payload.end(), 0);
	const uint8_t flags0 = buildGpsRtcmFlags(true, 0, 5);
	const uint8_t flags1 = buildGpsRtcmFlags(true, 1, 5);
	size_t len = 0;

	// WHEN: Fragment 0 is received twice with the same content.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(flags0, payload.data(), GPS_RTCM_MAX_FRAGMENT_LEN, 10, len), nullptr);
	EXPECT_EQ(gps_rtcm_assembler.addPacket(flags0, payload.data(), GPS_RTCM_MAX_FRAGMENT_LEN, 11, len), nullptr);

	// WHEN: The non-full final fragment arrives.
	const uint8_t *message = gps_rtcm_assembler.addPacket(flags1, &payload[GPS_RTCM_MAX_FRAGMENT_LEN], 12, 20, len);

	// THEN: The assembler still returns the expected full payload once.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that a new sequence flushes the partial state of the old sequence.
// WHY: Sequence IDs are the protocol mechanism that prevents mixing unrelated buffers.
TEST_F(RtcmTest, GpsRtcmAssemblerFlushesOnSequenceChange)
{
	// GIVEN: Two different payloads that start with fragment 0 but use different sequence IDs.
	std::vector<uint8_t> first_payload(GPS_RTCM_MAX_FRAGMENT_LEN + 10, 0x11);
	std::vector<uint8_t> second_payload(GPS_RTCM_MAX_FRAGMENT_LEN + 20, 0x22);

	size_t len = 0;

	// WHEN: A new sequence starts before the old one is complete.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 1), first_payload.data(), GPS_RTCM_MAX_FRAGMENT_LEN, 10,
					       len), nullptr);
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 2), second_payload.data(), GPS_RTCM_MAX_FRAGMENT_LEN, 20,
					       len), nullptr);

	// WHEN: The matching final fragment for the new sequence arrives.
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 1, 2),
				 &second_payload[GPS_RTCM_MAX_FRAGMENT_LEN], 20, 30, len);

	// THEN: Only the new sequence is returned.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, second_payload.size());
	EXPECT_EQ(memcmp(message, second_payload.data(), second_payload.size()), 0);
}

// WHAT: Verify that four full-sized fragments complete a message explicitly.
// WHY: The exact 720-byte case must not rely on an accidental fallback path.
TEST_F(RtcmTest, GpsRtcmAssemblerCompletesOnFourthFullFragment)
{
	// GIVEN: A payload that uses all four 180-byte fragment slots.
	std::vector<uint8_t> payload(GPS_RTCM_MAX_MESSAGE_LEN);
	std::iota(payload.begin(), payload.end(), 0);

	size_t len = 0;

	// WHEN: The first three full fragments arrive.
	for (uint8_t fragment = 0; fragment < GPS_RTCM_MAX_FRAGMENTS - 1; fragment++) {
		EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, fragment, 3),
						       &payload[fragment * GPS_RTCM_MAX_FRAGMENT_LEN], GPS_RTCM_MAX_FRAGMENT_LEN, 10 + fragment, len),
			  nullptr);
	}

	// WHEN: The fourth full fragment arrives.
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 3, 3),
				 &payload[3 * GPS_RTCM_MAX_FRAGMENT_LEN], GPS_RTCM_MAX_FRAGMENT_LEN, 20, len);

	// THEN: The assembler emits the completed 720-byte payload.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that an exact multiple of 180 bytes below 720 bytes can be
// completed with an empty terminating fragment.
// WHY: Per the MAVLink completion rule, the receiver only knows the message is
// shorter than 4 fragments after the first non-full fragment arrives.
TEST_F(RtcmTest, GpsRtcmAssemblerCompletesOnZeroLengthTerminatingFragment)
{
	// GIVEN: A 360-byte payload split into two full fragments.
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN * 2);
	std::iota(payload.begin(), payload.end(), 0);

	size_t len = 0;

	// WHEN: The two full fragments arrive.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 11), payload.data(),
					       GPS_RTCM_MAX_FRAGMENT_LEN, 10, len), nullptr);
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 1, 11),
					       payload.data() + GPS_RTCM_MAX_FRAGMENT_LEN, GPS_RTCM_MAX_FRAGMENT_LEN, 20, len), nullptr);

	// WHEN: An empty fragment marks the end of the MAVLink fragmented payload.
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 2, 11),
				 payload.data() + payload.size(), 0, 30, len);

	// THEN: The assembler returns the original 360-byte payload.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that a stale partial buffer is dropped after the timeout.
// WHY: This prevents an old fragment from being mixed into a later message with the same sequence ID.
TEST_F(RtcmTest, GpsRtcmAssemblerDropsStalePartialMessage)
{
	// GIVEN: A payload that spans one full fragment and one non-full fragment.
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN + 15, 0x44);

	size_t len = 0;

	// WHEN: Fragment 0 is followed by a late fragment 1 after the timeout window.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 4), payload.data(), GPS_RTCM_MAX_FRAGMENT_LEN, 10,
					       len), nullptr);
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 1, 4),
					       &payload[GPS_RTCM_MAX_FRAGMENT_LEN], 15, 10 + GPS_RTCM_FRAGMENT_TIMEOUT_US + 1, len),
		  nullptr);

	// WHEN: The matching fragment 0 arrives after the timeout reset.
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 4), payload.data(),
				 GPS_RTCM_MAX_FRAGMENT_LEN, 10 + GPS_RTCM_FRAGMENT_TIMEOUT_US + 2, len);

	// THEN: The fresh out-of-order pair is reconstructed without reusing the stale pre-timeout state.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that a conflicting fragment slot restarts assembly from the
// newest packet.
// WHY: The sequence ID only identifies one active buffer. If a slot is reused
// with different bytes, the old partial state is no longer trustworthy.
TEST_F(RtcmTest, GpsRtcmAssemblerRestartsOnFragmentContentMismatch)
{
	// GIVEN: Two different payloads that reuse the same fragment slot under the
	// same sequence ID.
	std::vector<uint8_t> old_fragment(GPS_RTCM_MAX_FRAGMENT_LEN, 0x55);
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN * 2 + 12);
	std::iota(payload.begin(), payload.end(), 0);
	size_t len = 0;

	// WHEN: Fragment 1 is first buffered with old content, then replaced with
	// new content for the same sequence ID.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 1, 9), old_fragment.data(),
					       old_fragment.size(), 10, len), nullptr);
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 1, 9),
					       &payload[GPS_RTCM_MAX_FRAGMENT_LEN], GPS_RTCM_MAX_FRAGMENT_LEN, 20, len), nullptr);

	// WHEN: The remaining fragments for the new buffer arrive.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 9), payload.data(),
					       GPS_RTCM_MAX_FRAGMENT_LEN, 30, len), nullptr);
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 2, 9),
				 &payload[GPS_RTCM_MAX_FRAGMENT_LEN * 2], 12, 40, len);

	// THEN: The assembler returns the newer payload, not the abandoned partial buffer.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// WHAT: Verify that a non-full fragment cannot appear before an already buffered higher fragment.
// WHY: Once a fragment claims to be the first non-full fragment, any stored
// higher fragment makes the buffered message internally inconsistent and the
// buffer must be dropped.
TEST_F(RtcmTest, GpsRtcmAssemblerRejectsNonFullFragmentBeforeBufferedHigherFragment)
{
	// GIVEN: An out-of-order higher fragment and a later valid two-fragment message with the same sequence ID.
	std::vector<uint8_t> garbage_fragment(GPS_RTCM_MAX_FRAGMENT_LEN, 0x77);
	std::vector<uint8_t> payload(GPS_RTCM_MAX_FRAGMENT_LEN + 16, 0x88);
	size_t len = 0;

	// WHEN: A higher fragment is buffered first.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 2, 10), garbage_fragment.data(),
					       garbage_fragment.size(), 10, len), nullptr);

	// WHEN: A non-full lower fragment arrives and would incorrectly claim the message already ended.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 1, 10), &payload[GPS_RTCM_MAX_FRAGMENT_LEN], 16, 20,
					       len), nullptr);

	// WHEN: A clean message is sent afterwards with the same sequence ID.
	EXPECT_EQ(gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 0, 10), payload.data(),
					       GPS_RTCM_MAX_FRAGMENT_LEN, 30, len), nullptr);
	const uint8_t *message = gps_rtcm_assembler.addPacket(buildGpsRtcmFlags(true, 1, 10),
				 &payload[GPS_RTCM_MAX_FRAGMENT_LEN], 16, 40, len);

	// THEN: The inconsistent partial buffer was discarded and the clean message reconstructs normally.
	ASSERT_NE(message, nullptr);
	EXPECT_EQ(len, payload.size());
	EXPECT_EQ(memcmp(message, payload.data(), payload.size()), 0);
}

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "AnelloParser.hpp"
#include <cassert>
#include <cstdio>
#include <vector>

static std::vector<uint8_t> frame(const mavlink_message_t &message)
{
	uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
	const unsigned length = mavlink_msg_to_send_buffer(bytes, &message);
	return std::vector<uint8_t>(bytes, bytes + length);
}

static anello::Parser::Update feed(anello::Parser &parser, const std::vector<uint8_t> &bytes, uint64_t now)
{
	anello::Parser::Update result = anello::Parser::Update::None;

	for (uint8_t byte : bytes) {
		const auto update = parser.parse(byte, now);

		if (update != anello::Parser::Update::None) {
			result = update;
		}
	}

	return result;
}

static void testProtocol(bool mavlink1)
{
	using Update = anello::Parser::Update;
	anello::Parser parser;
	auto *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	status->flags = mavlink1 ? MAVLINK_STATUS_FLAG_OUT_MAVLINK1 : 0;
	mavlink_message_t message{};
	assert(!parser.attitudeFresh(1) && !parser.positionFresh(1));

	mavlink_msg_attitude_pack(42, 1, &message, 100, 0.f, 0.f, 0.4f, 0.f, 0.f, 0.f);
	auto bytes = frame(message);

	// Fragmented serial reads must assemble exactly one frame.
	for (size_t i = 0; i + 1 < bytes.size(); ++i) {
		assert(parser.parse(bytes[i], 100) == Update::None);
	}

	assert(parser.parse(bytes.back(), 100) == Update::Attitude);
	assert(fabsf(matrix::Eulerf(parser.attitude).psi() - 0.4f) < 1e-5f);
	assert(parser.attitudeFresh(250100));
	assert(!parser.attitudeFresh(250101));

	mavlink_attitude_quaternion_t quaternion{};
	quaternion.q1 = 1.1f; // Valid but requires normalisation.
	mavlink_msg_attitude_quaternion_encode(42, 1, &message, &quaternion);
	assert(feed(parser, frame(message), 200) == Update::Attitude);
	assert(fabsf(parser.attitude.norm() - 1.f) < 1e-6f);

	mavlink_msg_attitude_pack(42, 1, &message, 100, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f);
	assert(feed(parser, frame(message), 250200) == Update::None); // Quaternion preferred through deadline.
	assert(feed(parser, frame(message), 250201) == Update::Attitude); // Euler fallback.

	quaternion.q1 = 0.f;
	mavlink_msg_attitude_quaternion_encode(42, 1, &message, &quaternion);
	assert(feed(parser, frame(message), 300000) == Update::None);
	quaternion.q1 = INFINITY;
	mavlink_msg_attitude_quaternion_encode(42, 1, &message, &quaternion);
	assert(feed(parser, frame(message), 300000) == Update::None);
	quaternion.q1 = 1.f;
	quaternion.rollspeed = NAN;
	mavlink_msg_attitude_quaternion_encode(42, 1, &message, &quaternion);
	assert(feed(parser, frame(message), 300000) == Update::None);
	mavlink_msg_attitude_pack(42, 1, &message, 100, NAN, 0.f, 0.f, 0.f, 0.f, 0.f);
	assert(feed(parser, frame(message), 300000) == Update::None);
	assert(parser.attitude_time == 250201);

	mavlink_msg_global_position_int_pack(42, 1, &message, 100, 370000000, -1220000000,
					     123450, 999999, 123, -234, 345, 65535);
	assert(feed(parser, frame(message), 400000) == Update::Position);
	assert(fabs(parser.latitude - 37.) < 1e-8);
	assert(fabs(parser.longitude + 122.) < 1e-8);
	assert(fabsf(parser.altitude - 123.45f) < 1e-4f); // AMSL, not relative_alt.
	assert(fabsf(parser.velocity(0) - 1.23f) < 1e-5f);
	assert(fabsf(parser.velocity(1) + 2.34f) < 1e-5f);
	assert(fabsf(parser.velocity(2) - 3.45f) < 1e-5f); // Down remains positive.
	assert(parser.positionFresh(1400000));
	assert(!parser.positionFresh(1400001));

	mavlink_msg_global_position_int_pack(42, 1, &message, 100, 900000001, 0, 0, 0, 0, 0, 0, 0);
	assert(feed(parser, frame(message), 500000) == Update::None);
	mavlink_msg_global_position_int_pack(42, 1, &message, 100, 0, 1800000001, 0, 0, 0, 0, 0, 0);
	assert(feed(parser, frame(message), 500000) == Update::None);
	assert(parser.position_time == 400000);

	// A second sender cannot mix its navigation into the locked source.
	mavlink_msg_attitude_pack(43, 1, &message, 100, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	assert(feed(parser, frame(message), 600000) == Update::None);
	mavlink_msg_attitude_pack(42, 2, &message, 100, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	assert(feed(parser, frame(message), 600000) == Update::None);
	mavlink_msg_attitude_pack(42, 1, &message, 100, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	bytes = frame(message);
	bytes.back() ^= 0x80; // CRC corruption must not refresh freshness.
	assert(feed(parser, bytes, 600000) == Update::None);
	assert(parser.attitude_time == 250201);
	assert(feed(parser, frame(message), 600001) == Update::Attitude); // Parser recovery.

	// Zero degrees latitude/longitude is a real location, not a missing fix sentinel.
	mavlink_msg_global_position_int_pack(42, 1, &message, 100, 0, 0, -1000, 123, 0, 0, 0, 0);
	assert(feed(parser, frame(message), 700000) == Update::Position);
	assert(parser.altitude == -1.f);
}

int main()
{
	testProtocol(false);
	testProtocol(true);
	puts("ANELLO parser tests passed (MAVLink 1 and 2)");
	return 0;
}

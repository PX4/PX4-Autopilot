// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "AnelloStreamConfig.hpp"
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


static void observe(anello::Parser &parser, anello::StreamConfig &config,
		    const mavlink_message_t &message, uint64_t now)
{
	const auto update = feed(parser, frame(message), now);
	config.observe(parser, update, now);
}

static void heartbeat(anello::Parser &parser, anello::StreamConfig &config, uint64_t now)
{
	mavlink_message_t message{};
	mavlink_msg_heartbeat_pack(42, 1, &message, MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
	observe(parser, config, message, now);
}

static void acknowledge(anello::Parser &parser, anello::StreamConfig &config, uint8_t result, uint64_t now,
			uint8_t source = 42, uint16_t command = MAV_CMD_SET_MESSAGE_INTERVAL,
			uint8_t target_system = 17, uint8_t target_component = anello::StreamConfig::SenderComponent)
{
	mavlink_message_t message{};
	mavlink_msg_command_ack_pack(source, 1, &message, command, result, 0, 0, target_system, target_component);
	observe(parser, config, message, now);
}

static mavlink_command_long_t request(const anello::StreamConfig &config)
{
	assert(config.size() > 0);
	mavlink_message_t rx{}, message{};
	mavlink_status_t rx_status{}, status{};
	bool complete = false;

	for (unsigned i = 0; i < config.size(); ++i) {
		if (mavlink_frame_char_buffer(&rx, &rx_status, config.data()[i], &message, &status) == MAVLINK_FRAMING_OK) {
			complete = true;
		}
	}

	assert(complete && message.msgid == MAVLINK_MSG_ID_COMMAND_LONG);
	assert(message.sysid == 17 && message.compid == anello::StreamConfig::SenderComponent);
	mavlink_command_long_t command{};
	mavlink_msg_command_long_decode(&message, &command);
	assert(command.command == MAV_CMD_SET_MESSAGE_INTERVAL);
	assert(command.target_system == 42 && command.target_component == 1);
	assert(command.param3 == 0 && command.param4 == 0 && command.param5 == 0 && command.param6 == 0 && command.param7 == 0);
	return command;
}

static void testStreamConfiguration(bool mavlink1)
{
	using anello::StreamConfig;
	mavlink_get_channel_status(MAVLINK_COMM_0)->flags = mavlink1 ? MAVLINK_STATUS_FLAG_OUT_MAVLINK1 : 0;
	anello::Parser parser;
	StreamConfig config;
	config.setSender(17);
	uint64_t now = 1000000;
	config.update(now);
	assert(config.size() == 0 && !config.havePeer());

	mavlink_message_t gcs{};
	mavlink_msg_heartbeat_pack(255, 190, &gcs, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
	observe(parser, config, gcs, now);
	assert(!config.havePeer());

	heartbeat(parser, config, now);
	assert(!parser.attitudeFresh(now) && !parser.positionFresh(now));

	for (unsigned index = 0; index < StreamConfig::RequestCount; ++index) {
		const auto channel_sequence = mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq;
		config.update(now);
		assert(mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq == channel_sequence);
		auto command = request(config);
		assert(command.param1 == StreamConfig::messageId(index));
		assert(command.param2 == StreamConfig::interval(index));
		assert(command.confirmation == 0);
		assert(config.data()[0] == (mavlink1 ? MAVLINK_STX_MAVLINK1 : MAVLINK_STX));

		// Nonblocking partial writes must retain the exact unsent suffix.
		const std::vector<uint8_t> bytes(config.data(), config.data() + config.size());
		config.written(1, now);
		config.update(now + 1);
		assert(config.size() == bytes.size() - 1);

		for (unsigned i = 0; i < config.size(); ++i) {
			assert(config.data()[i] == bytes[i + 1]);
		}

		config.written(config.size(), now);
		assert(config.size() == 0);
		acknowledge(parser, config, MAV_RESULT_ACCEPTED, now, 43);
		acknowledge(parser, config, MAV_RESULT_ACCEPTED, now, 42, MAV_CMD_REQUEST_MESSAGE);

		if (!mavlink1) {
			acknowledge(parser, config, MAV_RESULT_ACCEPTED, now, 42, MAV_CMD_SET_MESSAGE_INTERVAL, 18);
			acknowledge(parser, config, MAV_RESULT_ACCEPTED, now, 42, MAV_CMD_SET_MESSAGE_INTERVAL, 17, 1);
		}

		assert(config.accepted() == index);
		acknowledge(parser, config, MAV_RESULT_ACCEPTED, now);
		assert(config.accepted() == index + 1);
		// A duplicate ACK while between commands cannot advance the state.
		acknowledge(parser, config, MAV_RESULT_ACCEPTED, now);
		assert(config.accepted() == index + 1);
		config.update(now + StreamConfig::CommandGap - 1);
		assert(config.size() == 0);
		now += StreamConfig::CommandGap;
	}

	assert(config.complete() && config.failed() == 0 && config.sent() == 3);
	config.update(now);
	assert(config.size() == 0);

	// Reconnection restarts configuration, without changing the locked source.
	now += StreamConfig::LinkTimeout + 1;
	heartbeat(parser, config, now);
	config.update(now);
	assert(config.sessions() == 2 && config.accepted() == 0);
	assert(request(config).param1 == MAVLINK_MSG_ID_ATTITUDE_QUATERNION);
	config.written(config.size(), now);

	// A boot-time rollback also catches restarts shorter than the link timeout.
	mavlink_message_t navigation{};
	mavlink_msg_attitude_pack(42, 1, &navigation, 100000, 0, 0, 0, 0, 0, 0);
	observe(parser, config, navigation, ++now);
	mavlink_msg_attitude_pack(42, 1, &navigation, 10, 0, 0, 0, 0, 0, 0);
	observe(parser, config, navigation, ++now);
	assert(config.sessions() == 3);
	config.update(now);
	assert(request(config).confirmation == 0);
}

static void testStreamRetries()
{
	using anello::StreamConfig;
	mavlink_get_channel_status(MAVLINK_COMM_0)->flags = 0;
	anello::Parser parser;
	StreamConfig config;
	config.setSender(17);
	uint64_t now = 1000000;

	// No acknowledgements: three attempts per command, then stop.
	for (unsigned index = 0; index < StreamConfig::RequestCount; ++index) {
		for (unsigned attempt = 0; attempt < StreamConfig::MaxAttempts; ++attempt) {
			heartbeat(parser, config, now);
			config.update(now);
			const auto command = request(config);
			assert(command.param1 == StreamConfig::messageId(index));
			assert(command.confirmation == attempt);
			config.written(config.size(), now);
			now += StreamConfig::AckTimeout;
			heartbeat(parser, config, now);
			config.update(now);
			now += attempt + 1 < StreamConfig::MaxAttempts ? StreamConfig::AckTimeout : StreamConfig::CommandGap;
		}
	}

	assert(config.complete() && config.failed() == 3 && config.sent() == 9);
	heartbeat(parser, config, now);
	config.update(now);
	assert(config.size() == 0); // Heartbeats alone do not cause endless retry batches.

	// Restart the session and exercise partial-write timeout and terminal rejection.
	now += StreamConfig::LinkTimeout + 1;
	heartbeat(parser, config, now);
	config.update(now);
	config.written(1, now);
	now += StreamConfig::AckTimeout;
	heartbeat(parser, config, now);
	config.update(now);
	assert(config.size() == 0);
	now += StreamConfig::AckTimeout;
	heartbeat(parser, config, now);
	config.update(now);
	assert(request(config).confirmation == 1);
	config.written(config.size(), now);
	acknowledge(parser, config, MAV_RESULT_UNSUPPORTED, now);
	assert(config.failed() == 1);
	now += StreamConfig::CommandGap;
	config.update(now);
	assert(request(config).param1 == MAVLINK_MSG_ID_ATTITUDE);
	config.written(config.size(), now);

	// Progress ACKs have a bounded deadline, even if repeatedly received.
	for (unsigned i = 0; i < 4; ++i) {
		now += StreamConfig::AckTimeout;
		acknowledge(parser, config, MAV_RESULT_IN_PROGRESS, now);
		config.update(now);
		assert(config.size() == 0);
	}

	now += StreamConfig::AckTimeout;
	acknowledge(parser, config, MAV_RESULT_IN_PROGRESS, now);
	config.update(now);
	now += StreamConfig::AckTimeout;
	heartbeat(parser, config, now);
	config.update(now);
	assert(request(config).confirmation == 1);
	config.written(config.size(), now);
	acknowledge(parser, config, MAV_RESULT_TEMPORARILY_REJECTED, now);
	now += StreamConfig::AckTimeout;
	heartbeat(parser, config, now);
	config.update(now);
	assert(request(config).confirmation == 2);

	// Source silence never initiates a new configuration session or transmits.
	now += StreamConfig::LinkTimeout + 1;
	config.update(now);
	assert(config.size() == 0);

	// Normal sender clock wrap and small reordering do not indicate a reboot.
	anello::Parser wrap_parser;
	StreamConfig wrap_config;
	mavlink_message_t message{};
	mavlink_msg_attitude_pack(42, 1, &message, UINT32_MAX - 100, 0, 0, 0, 0, 0, 0);
	observe(wrap_parser, wrap_config, message, now);
	mavlink_msg_attitude_pack(42, 1, &message, 50, 0, 0, 0, 0, 0, 0);
	observe(wrap_parser, wrap_config, message, ++now);
	mavlink_msg_attitude_pack(42, 1, &message, 40, 0, 0, 0, 0, 0, 0);
	observe(wrap_parser, wrap_config, message, ++now);
	assert(wrap_config.sessions() == 1);
}

int main()
{
	testProtocol(false);
	testProtocol(true);
	testStreamConfiguration(false);
	testStreamConfiguration(true);
	testStreamRetries();
	puts("ANELLO parser and stream configuration tests passed (MAVLink 1 and 2)");
	return 0;
}

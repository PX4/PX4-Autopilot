// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include "AnelloParser.hpp"

namespace anello
{
/** Bounded, sequential SET_MESSAGE_INTERVAL transactions on the dedicated UART.
 * No global MAVLink channel state is used. ACKs contain only the command ID,
 * so never have more than one request outstanding.
 */
class StreamConfig
{
public:
	static constexpr unsigned RequestCount = 3;
	static constexpr unsigned MaxAttempts = 3;
	static constexpr uint64_t AckTimeout = 1000000;
	static constexpr uint64_t LinkTimeout = 3000000;
	static constexpr uint64_t CommandGap = 250000;
	static constexpr uint64_t ProgressTimeout = 5000000;
	static constexpr uint8_t SenderComponent = MAV_COMP_ID_PERIPHERAL;

	void setSender(uint8_t system) { _sender_system = system; }

	void observe(const Parser &parser, Parser::Update update, uint64_t now)
	{
		if (update == Parser::Update::None) {
			return;
		}

		bool restart = !_have_peer || now - _last_rx > LinkTimeout;
		const bool navigation = update == Parser::Update::Attitude || update == Parser::Update::Position;

		if (navigation && _have_boot_time) {
			const uint32_t forward_delta = parser.boot_time_ms - _boot_time_ms;

			// Unsigned subtraction handles the normal 49-day uint32 wrap.
			// Ignore sub-second packet reordering.
			if (forward_delta > UINT32_MAX / 2 && _boot_time_ms - parser.boot_time_ms > 1000) {
				restart = true;
			}
		}

		if (restart) {
			_target_system = parser.sourceSystem();
			_target_component = parser.sourceComponent();
			_tx_status.flags = parser.mavlink1() ? MAVLINK_STATUS_FLAG_OUT_MAVLINK1 : 0;
			_index = 0;
			_attempts = 0;
			_accepted = _failed = 0;
			_tx_length = _tx_offset = 0;
			_waiting = false;
			_have_boot_time = false;
			_ready_at = now;
			++_sessions;
		}

		_have_peer = true;
		_last_rx = now;

		if (navigation && (!_have_boot_time || parser.boot_time_ms - _boot_time_ms <= UINT32_MAX / 2)) {
			_boot_time_ms = parser.boot_time_ms;
			_have_boot_time = true;
		}

		if (update == Parser::Update::CommandAck && _waiting) {
			const auto &ack = parser.ack;

			if (ack.command != MAV_CMD_SET_MESSAGE_INTERVAL
			    || (ack.target_system != 0 && ack.target_system != _sender_system)
			    || (ack.target_component != 0 && ack.target_component != SenderComponent)) {
				return;
			}

			if (ack.result == MAV_RESULT_ACCEPTED) {
				finish(true, now);

			} else if (ack.result == MAV_RESULT_IN_PROGRESS) {
				// Repeated progress ACKs cannot extend the deadline indefinitely.
				_deadline = _sent_at + ProgressTimeout;

			} else if (ack.result == MAV_RESULT_TEMPORARILY_REJECTED) {
				retry(now);

			} else {
				finish(false, now);
			}
		}
	}

	void update(uint64_t now)
	{
		if (!_have_peer || now - _last_rx > LinkTimeout) {
			_tx_length = _tx_offset = 0;
			_waiting = false;
			return;
		}

		if (_index >= RequestCount) {
			return;
		}

		if (_tx_length > 0) {
			if (now - _queued_at >= AckTimeout) {
				abortTransmission(now);
			}

			return;
		}

		if (_waiting) {
			if (now >= _deadline) {
				retry(now);
			}

			return;
		}

		if (now < _ready_at) {
			return;
		}

		mavlink_command_long_t command{};
		command.target_system = _target_system;
		command.target_component = _target_component;
		command.command = MAV_CMD_SET_MESSAGE_INTERVAL;
		command.confirmation = _attempts;
		command.param1 = messageId(_index);
		command.param2 = interval(_index);
		mavlink_message_t message{};
		mavlink_msg_command_long_encode_status(_sender_system, SenderComponent, &_tx_status, &message, &command);
		_tx_length = mavlink_msg_to_send_buffer(_tx_buffer, &message);
		_tx_offset = 0;
		_queued_at = now;
		++_attempts;
	}

	const uint8_t *data() const { return _tx_buffer + _tx_offset; }
	unsigned size() const { return _tx_length - _tx_offset; }

	void written(unsigned count, uint64_t now)
	{
		if (_tx_length == 0 || count > size()) {
			return;
		}

		_tx_offset += count;

		if (_tx_offset == _tx_length) {
			_tx_offset = _tx_length = 0;
			_waiting = true;
			_sent_at = now;
			_deadline = now + AckTimeout;
			++_sent;
		}
	}

	void abortTransmission(uint64_t now)
	{
		_tx_length = _tx_offset = 0;
		retry(now);
	}

	bool havePeer() const { return _have_peer; }
	bool complete() const { return _index >= RequestCount; }
	unsigned accepted() const { return _accepted; }
	unsigned failed() const { return _failed; }
	unsigned sent() const { return _sent; }
	unsigned sessions() const { return _sessions; }
	uint8_t targetSystem() const { return _target_system; }
	uint8_t targetComponent() const { return _target_component; }

	static uint32_t messageId(unsigned index)
	{
		switch (index) {
		case 0: return MAVLINK_MSG_ID_ATTITUDE_QUATERNION;

		case 1: return MAVLINK_MSG_ID_ATTITUDE;

		default: return MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
		}
	}

	static uint32_t interval(unsigned index) { return index == 2 ? 100000 : 20000; }

private:
	void finish(bool accepted, uint64_t now)
	{
		if (accepted) {
			++_accepted;

		} else {
			++_failed;
		}

		++_index;
		_attempts = 0;
		_waiting = false;
		_ready_at = now + CommandGap;
	}

	void retry(uint64_t now)
	{
		_waiting = false;

		if (_attempts >= MaxAttempts) {
			finish(false, now);

		} else {
			_ready_at = now + AckTimeout;
		}
	}

	mavlink_status_t _tx_status{};
	uint8_t _tx_buffer[MAVLINK_MAX_PACKET_LEN] {};
	unsigned _tx_length{0};
	unsigned _tx_offset{0};
	uint8_t _sender_system{1};
	uint8_t _target_system{0};
	uint8_t _target_component{0};
	unsigned _index{0};
	unsigned _attempts{0};
	unsigned _accepted{0};
	unsigned _failed{0};
	unsigned _sent{0};
	unsigned _sessions{0};
	bool _have_peer{false};
	bool _waiting{false};
	bool _have_boot_time{false};
	uint32_t _boot_time_ms{0};
	uint64_t _last_rx{0};
	uint64_t _ready_at{0};
	uint64_t _queued_at{0};
	uint64_t _sent_at{0};
	uint64_t _deadline{0};
};
} // namespace anello

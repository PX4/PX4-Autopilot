/*
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ptgl100_parser.hpp"

#include <cstring>

namespace ptgl100
{

void make_command(uint8_t command, uint8_t (&frame)[FRAME_SIZE])
{
	frame[0] = HEADER_HIGH;
	frame[1] = HEADER_LOW;
	frame[2] = command;
	uint8_t checksum = command;

	for (size_t i = 3; i < FRAME_SIZE - 1; ++i) {
		frame[i] = 0xff;
		checksum += frame[i];
	}

	frame[FRAME_SIZE - 1] = checksum;
}

Parser::Result Parser::parse(uint8_t byte, Reply &reply)
{
	if (_length == 0 && byte != HEADER_HIGH) {
		return Result::Incomplete;
	}

	if (_length == 1 && byte != HEADER_LOW) {
		_length = (byte == HEADER_HIGH) ? 1 : 0;
		return Result::Incomplete;
	}

	_buffer[_length++] = byte;

	if (_length < FRAME_SIZE) {
		return Result::Incomplete;
	}

	uint8_t checksum = 0;

	for (size_t i = 0; i < FRAME_SIZE - 1; ++i) {
		checksum += _buffer[i];
	}

	if (checksum != _buffer[FRAME_SIZE - 1]) {
		resynchronise();
		return Result::BadChecksum;
	}

	switch (_buffer[2]) {
	case SINGLE_MEASUREMENT:
	case CONTINUOUS_MEASUREMENT:
	case STOP_MEASUREMENT:
	case SELF_TEST:
		break;

	default:
		resynchronise();
		return Result::UnknownCommand;
	}

	reply.command = _buffer[2];
	reply.status = _buffer[3];
	reply.distance_raw = (static_cast<uint32_t>(_buffer[4]) << 16)
			     | (static_cast<uint32_t>(_buffer[5]) << 8) | _buffer[6];
	reply.error_code = _buffer[6];
	reset();
	return Result::Reply;
}

void Parser::resynchronise()
{
	// A malformed/truncated frame can contain the beginning of the next frame.
	// Keep the earliest possible header, including a lone trailing 0x55.
	for (size_t i = 1; i < _length; ++i) {
		if (_buffer[i] == HEADER_HIGH && (i + 1 == _length || _buffer[i + 1] == HEADER_LOW)) {
			_length -= i;
			memmove(_buffer, &_buffer[i], _length);
			return;
		}
	}

	reset();
}

} // namespace ptgl100

/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file rtcm.cpp
 *
 * @author Thomas Frans
*/

#include "rtcm.h"

#include <cstring>
#include <px4_platform_common/log.h>

#include "module.h"

namespace septentrio
{

namespace rtcm
{

Decoder::Decoder()
{
	reset();
}

Decoder::~Decoder()
{
	delete[] _message;
}

Decoder::State Decoder::add_byte(uint8_t byte)
{
	switch (_state) {
	case State::SearchingPreamble:
		if (byte == PREAMBLE) {
			_message[_current_index] = byte;
			_current_index++;
			_state = State::GettingHeader;
		}

		break;

	case State::GettingHeader:
		_message[_current_index] = byte;
		_current_index++;

		if (header_received()) {
			_message_length = parse_message_length();

			if (_message_length > MAX_BODY_SIZE) {
				reset();
				return _state;

			} else if (_message_length + HEADER_SIZE + CRC_SIZE > INITIAL_BUFFER_LENGTH) {
				uint16_t new_buffer_size = _message_length + HEADER_SIZE + CRC_SIZE;
				uint8_t *new_buffer = new uint8_t[new_buffer_size];

				if (!new_buffer) {
					reset();
					return _state;
				}

				memcpy(new_buffer, _message, HEADER_SIZE);
				delete[](_message);

				_message = new_buffer;
			}

			_state = State::Busy;
		}

		break;

	case State::Busy:
		_message[_current_index] = byte;
		_current_index++;

		if (_message_length + HEADER_SIZE + CRC_SIZE == _current_index) {
			_state = State::Done;
		}

		break;

	case State::Done:
		SEP_WARN("RTCM: Discarding excess byte");
		break;
	}

	return _state;
}

void Decoder::reset()
{
	if (_message) {
		delete[] _message;
	}

	_message = new uint8_t[INITIAL_BUFFER_LENGTH];
	_current_index = 0;
	_message_length = 0;
	_state = State::SearchingPreamble;
}

uint16_t Decoder::parse_message_length() const
{
	if (!header_received()) {
		return PX4_ERROR;
	}

	return ((static_cast<uint16_t>(_message[1]) & 3) << 8) | _message[2];
}

bool Decoder::header_received() const
{
	return _current_index >= HEADER_SIZE;
}

uint16_t Decoder::received_bytes() const
{
	return _current_index;
}

uint16_t Decoder::message_id() const
{
	return (_message[3] << 4) | (_message[4] >> 4);
}

} // namespace rtcm

} // namespace septentrio

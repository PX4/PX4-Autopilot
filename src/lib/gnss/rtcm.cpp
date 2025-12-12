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
 * @file rtcm.cpp
 *
 * RTCM3 protocol parsing implementation.
 */

#include "rtcm.h"
#include <cstring>

namespace gnss
{

uint32_t rtcm3_crc24q(const uint8_t *data, size_t len)
{
	uint32_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= static_cast<uint32_t>(data[i]) << 16;

		for (int j = 0; j < 8; j++) {
			crc <<= 1;

			if (crc & 0x1000000) {
				crc ^= RTCM3_CRC24Q_POLY;
			}
		}
	}

	return crc & 0xFFFFFF;
}

size_t Rtcm3Parser::addData(const uint8_t *data, size_t len)
{
	size_t space_available = BUFFER_SIZE - _buffer_len;
	size_t to_copy = (len < space_available) ? len : space_available;

	if (to_copy > 0) {
		memcpy(&_buffer[_buffer_len], data, to_copy);
		_buffer_len += to_copy;
	}

	return to_copy;
}

const uint8_t *Rtcm3Parser::getNextMessage(size_t *out_len)
{
	while (_buffer_len > 0) {
		if (_buffer[0] != RTCM3_PREAMBLE) {
			_bytes_discarded++;
			discardBytes(1);
			continue;
		}

		if (_buffer_len < RTCM3_HEADER_LEN) {
			return nullptr;
		}

		size_t payload_len = rtcm3_payload_length(_buffer);

		if (payload_len > RTCM3_MAX_PAYLOAD_LEN) {
			// Invalid length - not a valid frame, discard preamble
			_bytes_discarded++;
			discardBytes(1);
			continue;
		}

		size_t frame_len = RTCM3_HEADER_LEN + payload_len + RTCM3_CRC_LEN;

		// Check if we have the complete frame
		if (_buffer_len < frame_len) {
			return nullptr;
		}

		uint32_t calculated_crc = rtcm3_crc24q(_buffer, RTCM3_HEADER_LEN + payload_len);
		uint32_t received_crc = (static_cast<uint32_t>(_buffer[frame_len - 3]) << 16) |
					(static_cast<uint32_t>(_buffer[frame_len - 2]) << 8) |
					_buffer[frame_len - 1];

		if (calculated_crc != received_crc) {
			_crc_errors++;
			discardBytes(1);
			continue;
		}

		*out_len = frame_len;
		return _buffer;
	}

	return nullptr;
}

void Rtcm3Parser::consumeMessage(size_t len)
{
	discardBytes(len);
	_messages_parsed++;
	_total_frame_bytes += len;
}

void Rtcm3Parser::discardBytes(size_t count)
{
	if (count >= _buffer_len) {
		_buffer_len = 0;

	} else {
		memmove(_buffer, &_buffer[count], _buffer_len - count);
		_buffer_len -= count;
	}
}

} // namespace gnss

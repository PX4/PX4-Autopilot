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

#include "spartn.h"

#include <cstring>

namespace gnss
{

uint32_t SpartnParser::messageCrc(const uint8_t *data, size_t len, uint8_t crc_type)
{
	// SPARTN TF005: 0=CRC-8 poly 0x07, 1=CRC-16 poly 0x1021,
	// 2=CRC-24 poly 0x864CFB, 3=CRC-32 poly 0x04C11DB7 init/xor ~0
	const unsigned n = 8u * (crc_type + 1u);
	const uint32_t poly = (crc_type == 0) ? 0x07u :
			      (crc_type == 1) ? 0x1021u :
			      (crc_type == 2) ? 0x864CFBu : 0x04C11DB7u;
	const uint32_t top = 1u << n;
	const uint32_t g = top | poly;
	uint32_t crc = (crc_type == 3) ? 0xFFFFFFFFu : 0u;

	for (size_t i = 0; i < len; i++) {
		crc ^= static_cast<uint32_t>(data[i]) << (n - 8);

		for (int b = 0; b < 8; b++) {
			crc <<= 1;

			if (crc & top) {
				crc ^= g;
			}
		}
	}

	if (crc_type == 3) {
		crc ^= 0xFFFFFFFFu;
	}

	return crc & (top - 1);
}

size_t SpartnParser::addData(const uint8_t *data, size_t len)
{
	const size_t space = BUFFER_SIZE - _buffer_len;
	const size_t n = (len < space) ? len : space;

	if (n > 0) {
		memcpy(&_buffer[_buffer_len], data, n);
		_buffer_len += n;
	}

	return n;
}

const uint8_t *SpartnParser::getNextMessage(size_t *out_len)
{
	while (_buffer_len > 0) {
		size_t drop = 0;

		while (drop < _buffer_len && _buffer[drop] != SPARTN_PREAMBLE) {
			drop++;
		}

		if (drop > 0) {
			_bytes_discarded += drop;
			discardBytes(drop);
		}

		if (_buffer_len < SPARTN_FRAME_START_LEN) {
			return nullptr;
		}

		const uint8_t b1 = _buffer[1];
		const uint8_t b2 = _buffer[2];
		const uint8_t b3 = _buffer[3];
		const uint16_t payload_len = static_cast<uint16_t>(((b1 & 0x01) << 9) | (b2 << 1) | (b3 >> 7));
		const bool eaf = ((b3 >> 6) & 0x01) != 0;
		const uint8_t crc_type = (b3 >> 4) & 0x03;
		const size_t crc_bytes = static_cast<size_t>(crc_type) + 1;

		if (payload_len > SPARTN_MAX_PAYLOAD_LEN) {
			_bytes_discarded++;
			discardBytes(1);
			continue;
		}

		// Need TF007 for time-tag type
		if (_buffer_len < SPARTN_FRAME_START_LEN + 1) {
			return nullptr;
		}

		const bool time32 = ((_buffer[4] >> 3) & 0x01) != 0;
		const size_t desc_len = spartn_desc_length_bytes(eaf, time32);

		if (_buffer_len < SPARTN_FRAME_START_LEN + desc_len) {
			return nullptr;
		}

		size_t auth_bytes = 0;

		if (eaf) {
			const uint8_t last = _buffer[SPARTN_FRAME_START_LEN + desc_len - 1];
			auth_bytes = spartn_auth_length_bytes(true, (last >> 3) & 0x07, last & 0x07);
		}

		const size_t frame_len = SPARTN_FRAME_START_LEN + desc_len + payload_len + auth_bytes + crc_bytes;

		if (frame_len > SPARTN_MAX_FRAME_LEN) {
			_bytes_discarded++;
			discardBytes(1);
			continue;
		}

		if (_buffer_len < frame_len) {
			return nullptr;
		}

		// Message CRC over TF002..TF017 (skip preamble and TF018)
		const size_t core_len = frame_len - 1 - crc_bytes;
		const uint32_t calc = messageCrc(&_buffer[1], core_len, crc_type);
		uint32_t recv = 0;

		for (size_t i = 0; i < crc_bytes; i++) {
			recv = (recv << 8) | _buffer[frame_len - crc_bytes + i];
		}

		if (calc != recv) {
			_crc_errors++;
			discardBytes(1);
			continue;
		}

		*out_len = frame_len;
		return _buffer;
	}

	return nullptr;
}

void SpartnParser::consumeMessage(size_t len)
{
	discardBytes(len);
	_messages_parsed++;
	_total_frame_bytes += len;
}

void SpartnParser::discardBytes(size_t count)
{
	if (count >= _buffer_len) {
		_buffer_len = 0;

	} else {
		memmove(_buffer, &_buffer[count], _buffer_len - count);
		_buffer_len -= count;
	}
}

} // namespace gnss

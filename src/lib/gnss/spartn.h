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
 * @file spartn.h
 *
 * SPARTN transport-layer framing (message boundary detection).
 *
 * Frame layout (SPARTN ICD v2.x): preamble 0x73, then bit-packed TF002–TF018.
 * This parser reassembles complete frames from a byte stream and validates
 * the message CRC (TF018). Payload content is not decoded.
 *
 * Reference: https://www.spartnformat.org/
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace gnss
{

static constexpr uint8_t  SPARTN_PREAMBLE        = 0x73;
static constexpr size_t   SPARTN_FRAME_START_LEN = 4;
static constexpr size_t   SPARTN_MAX_PAYLOAD_LEN = 1023;
static constexpr size_t   SPARTN_MAX_AUTH_LEN    = 64;
static constexpr size_t   SPARTN_MAX_DESC_LEN    = 8;
static constexpr size_t   SPARTN_MAX_CRC_LEN     = 4;
static constexpr size_t   SPARTN_MAX_FRAME_LEN   =
	SPARTN_FRAME_START_LEN + SPARTN_MAX_DESC_LEN + SPARTN_MAX_PAYLOAD_LEN +
	SPARTN_MAX_AUTH_LEN + SPARTN_MAX_CRC_LEN;

inline size_t spartn_auth_length_bytes(bool eaf, uint8_t auth_ind, uint8_t emb_auth_len)
{
	if (!eaf || auth_ind <= 1) {
		return 0;
	}

	// TF015: 0=64b, 1=96b, 2=128b, 3=256b, 4=512b
	static constexpr uint8_t kBytes[] = {8, 12, 16, 32, 64};
	return (emb_auth_len < 5) ? kBytes[emb_auth_len] : 64;
}

inline size_t spartn_desc_length_bytes(bool eaf, bool time_tag_32bit)
{
	return (time_tag_32bit ? 6u : 4u) + (eaf ? 2u : 0u);
}

struct SpartnParserStats {
	uint32_t messages_parsed;
	uint32_t crc_errors;
	uint32_t bytes_discarded;
	uint32_t total_frame_bytes;
};

/**
 * SPARTN frame parser: buffer chunks, pull CRC-validated complete frames.
 */
class SpartnParser
{
public:
	static constexpr size_t BUFFER_SIZE = SPARTN_MAX_FRAME_LEN * 2;

	SpartnParser() = default;

	size_t addData(const uint8_t *data, size_t len);
	const uint8_t *getNextMessage(size_t *out_len);
	void consumeMessage(size_t len);

	size_t bufferedBytes() const { return _buffer_len; }

	SpartnParserStats getStats() const
	{
		return {_messages_parsed, _crc_errors, _bytes_discarded, _total_frame_bytes};
	}

private:
	void discardBytes(size_t count);
	static uint32_t messageCrc(const uint8_t *data, size_t len, uint8_t crc_type);

	uint8_t  _buffer[BUFFER_SIZE] {};
	size_t   _buffer_len {0};

	uint32_t _messages_parsed {0};
	uint32_t _crc_errors {0};
	uint32_t _bytes_discarded {0};
	uint32_t _total_frame_bytes {0};
};

} // namespace gnss

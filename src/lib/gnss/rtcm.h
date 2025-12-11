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
 * @file rtcm.h
 *
 * RTCM3 protocol definitions and parsing utilities.
 *
 * RTCM3 frame structure:
 *   Byte 0:     Preamble (0xD3)
 *   Byte 1-2:   6 reserved bits (0) + 10-bit payload length
 *   Byte 3..N:  Payload (0-1023 bytes)
 *   Last 3:     CRC-24Q checksum
 *
 * Total frame size: 3 (header) + payload_length + 3 (CRC) = 6 + payload_length
 * Maximum frame size: 6 + 1023 = 1029 bytes
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace gnss
{

// RTCM3 protocol constants
static constexpr uint8_t  RTCM3_PREAMBLE        = 0xD3;
static constexpr size_t   RTCM3_HEADER_LEN      = 3;   // Preamble + 2 bytes reserved/length
static constexpr size_t   RTCM3_CRC_LEN         = 3;
static constexpr size_t   RTCM3_MAX_PAYLOAD_LEN = 1023;
static constexpr size_t   RTCM3_MAX_FRAME_LEN   = RTCM3_HEADER_LEN + RTCM3_MAX_PAYLOAD_LEN + RTCM3_CRC_LEN; // 1029
static constexpr uint32_t RTCM3_CRC24Q_POLY     = 0x1864CFB;

/**
 * Calculate CRC-24Q checksum for RTCM3 messages.
 *
 * @param data   Pointer to data buffer
 * @param len    Length of data
 * @return       24-bit CRC value
 */
uint32_t rtcm3_crc24q(const uint8_t *data, size_t len);

/**
 * Extract RTCM3 message type from a frame buffer.
 * The message type is the first 12 bits of the payload.
 *
 * @param frame  Pointer to complete RTCM3 frame (starting with preamble)
 * @return       Message type ID (0 if frame is too short)
 */
inline uint16_t rtcm3_message_type(const uint8_t *frame)
{
	// Message type is first 12 bits of payload (bytes 3-4)
	return (static_cast<uint16_t>(frame[3]) << 4) | (frame[4] >> 4);
}

/**
 * Extract payload length from RTCM3 frame header.
 *
 * @param frame  Pointer to at least 3 bytes of RTCM3 frame header
 * @return       Payload length (0-1023)
 */
inline size_t rtcm3_payload_length(const uint8_t *frame)
{
	return ((static_cast<size_t>(frame[1]) & 0x03) << 8) | frame[2];
}

/**
 * RTCM3 parser statistics.
 */
struct Rtcm3ParserStats {
	uint32_t messages_parsed;   ///< Messages successfully parsed and consumed
	uint32_t crc_errors;        ///< Messages with CRC failures
	uint32_t bytes_discarded;   ///< Bytes discarded while searching for valid frames
	uint32_t total_frame_bytes; ///< Total bytes in successfully parsed frames
};

/**
 * RTCM3 frame parser for detecting message boundaries in a byte stream.
 *
 * This parser is designed for scenarios where RTCM data arrives in arbitrary
 * chunks (e.g., from uORB messages, serial ports) and needs to be reassembled
 * into complete, CRC-validated RTCM messages.
 *
 * Usage:
 *   Rtcm3Parser parser;
 *   parser.addData(chunk1, len1);
 *   parser.addData(chunk2, len2);
 *
 *   size_t frame_len;
 *   const uint8_t *frame;
 *   while ((frame = parser.getNextMessage(&frame_len)) != nullptr) {
 *       // Process complete RTCM message
 *       uint16_t msg_type = rtcm3_message_type(frame);
 *       // ... use the frame ...
 *       parser.consumeMessage(frame_len);
 *   }
 */
class Rtcm3Parser
{
public:
	// Buffer size: enough for 2 max-size messages to handle overlap
	static constexpr size_t BUFFER_SIZE = RTCM3_MAX_FRAME_LEN * 2;

	Rtcm3Parser() = default;

	/**
	 * Add data to the parser buffer.
	 *
	 * @param data Pointer to incoming data
	 * @param len  Number of bytes to add
	 * @return     Number of bytes actually added (may be less if buffer is full)
	 */
	size_t addData(const uint8_t *data, size_t len);

	/**
	 * Get a pointer to the next complete RTCM3 message without consuming it.
	 *
	 * Returns a pointer directly into the parser's internal buffer where the
	 * valid frame starts. Invalid bytes at the buffer start are discarded
	 * during the search. The returned pointer remains valid until the next
	 * call to addData(), consumeMessage(), or reset().
	 *
	 * After processing the message, call consumeMessage() to remove it from
	 * the buffer.
	 *
	 * @param out_len    Set to the total frame length
	 * @return           Pointer to the frame in internal buffer, or nullptr
	 *                   if no complete valid frame is available
	 */
	const uint8_t *getNextMessage(size_t *out_len);

	/**
	 * Consume (remove) the next message from the buffer.
	 *
	 * Call this after successfully processing a message obtained via
	 * peekNextMessage(). The length should match what peekNextMessage returned.
	 *
	 * @param len   Number of bytes to remove from the buffer
	 */
	void consumeMessage(size_t len);

	/**
	 * Get the number of bytes currently buffered.
	 */
	size_t bufferedBytes() const { return _buffer_len; }

	/**
	 * Get parser statistics.
	 */
	Rtcm3ParserStats getStats() const
	{
		return {_messages_parsed, _crc_errors, _bytes_discarded, _total_frame_bytes};
	}

private:
	/**
	 * Remove bytes from the beginning of the buffer.
	 */
	void discardBytes(size_t count);

	uint8_t  _buffer[BUFFER_SIZE] {};
	size_t   _buffer_len {0};

	// Statistics
	uint32_t _messages_parsed {0};
	uint32_t _crc_errors {0};
	uint32_t _bytes_discarded {0};
	uint32_t _total_frame_bytes {0};
};

} // namespace gnss

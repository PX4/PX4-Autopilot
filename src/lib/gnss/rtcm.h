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

// MAVLink GPS_RTCM_DATA fragmentation constants.
// Spec: https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA
//
// flags bit layout:
//   bit 0:   fragmented flag
//   bits 1-2: fragment ID
//   bits 3-7: sequence ID
//
// Fragmented payloads must only be flushed once the complete buffer has been
// reconstructed. A buffer is complete once either all 4 fragments are present,
// or the first fragment with a non-full payload has been received and every
// lower fragment ID is present.
static constexpr size_t   GPS_RTCM_MAX_FRAGMENT_LEN         = 180;
static constexpr size_t   GPS_RTCM_MAX_FRAGMENTS            = 4;
static constexpr size_t   GPS_RTCM_MAX_MESSAGE_LEN          = GPS_RTCM_MAX_FRAGMENT_LEN * GPS_RTCM_MAX_FRAGMENTS;
static constexpr uint64_t GPS_RTCM_FRAGMENT_TIMEOUT_US      = 1000000;
static constexpr uint8_t  GPS_RTCM_FLAG_FRAGMENTED          = 1 << 0;
static constexpr uint8_t  GPS_RTCM_FLAG_FRAGMENT_ID_SHIFT   = 1;
static constexpr uint8_t  GPS_RTCM_FLAG_FRAGMENT_ID_MASK    = 0x03;
static constexpr uint8_t  GPS_RTCM_FLAG_SEQUENCE_ID_SHIFT   = 3;
static constexpr uint8_t  GPS_RTCM_FLAG_SEQUENCE_ID_MASK    = 0x1f;

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

inline bool gps_rtcm_is_fragmented(uint8_t flags)
{
	return (flags & GPS_RTCM_FLAG_FRAGMENTED) != 0;
}

inline uint8_t gps_rtcm_fragment_id(uint8_t flags)
{
	return (flags >> GPS_RTCM_FLAG_FRAGMENT_ID_SHIFT) & GPS_RTCM_FLAG_FRAGMENT_ID_MASK;
}

inline uint8_t gps_rtcm_sequence_id(uint8_t flags)
{
	return (flags >> GPS_RTCM_FLAG_SEQUENCE_ID_SHIFT) & GPS_RTCM_FLAG_SEQUENCE_ID_MASK;
}

class GpsRtcmMessageAssembler
{
public:
	GpsRtcmMessageAssembler() = default;

	/**
	 * Add a MAVLink GPS_RTCM_DATA packet.
	 *
	 * Fragmented packets are buffered until the autopilot can reconstruct the
	 * complete payload. The fragment ID selects the slot within the 4-fragment
	 * buffer, while the sequence ID prevents fragments from different buffers
	 * from being mixed together. A fragmented payload is complete once either
	 * all 4 fragments are present, or the first fragment with a non-full
	 * payload has been received and every lower fragment ID is present.
	 * The returned pointer is valid until the next call to addPacket().
	 *
	 * @param flags       MAVLink GPS_RTCM_DATA flags
	 * @param data        Packet payload
	 * @param len         Packet payload length
	 * @param timestamp   Packet arrival timestamp in microseconds
	 * @param out_len     Set to the reassembled message length, or 0 if incomplete/invalid
	 * @return            Pointer to a complete message, or nullptr if the message is incomplete or invalid
	 */
	const uint8_t *addPacket(uint8_t flags, const uint8_t *data, size_t len, uint64_t timestamp, size_t &out_len);

	void reset();

private:
	struct FragmentSlot {
		uint8_t data[GPS_RTCM_MAX_FRAGMENT_LEN] {};
		size_t len {0};
		bool present {false};
	};

	struct SequenceState {
		uint8_t sequence_id {0};
		int8_t last_fragment_id {-1};
		uint64_t timestamp {0};
		bool active {false};
	};

	void resetActiveState();
	bool hasFragmentAfter(uint8_t fragment_id) const;

	bool isComplete() const;
	size_t lastFragmentIndex() const;

	FragmentSlot _fragments[GPS_RTCM_MAX_FRAGMENTS] {};
	uint8_t _assembled_message[GPS_RTCM_MAX_MESSAGE_LEN] {};
	// State for the sequence that is currently being assembled.
	SequenceState _active_sequence {};
};

class GpsRtcmMessageFragmenter
{
public:
	GpsRtcmMessageFragmenter() = default;

	/**
	 * Start fragmenting a complete RTCM message into MAVLink GPS_RTCM_DATA packets.
	 *
	 * Messages up to 180 bytes are sent as a single packet with the fragmented
	 * bit clear and fragment ID 0. Larger messages are split into packets that
	 * share a sequence ID and use increasing fragment IDs. Exact multiples of
	 * 180 bytes below 720 bytes emit a final zero-length fragment so receivers
	 * can apply the MAVLink completion rule: a fragmented payload is complete
	 * once either all 4 fragments are present, or the first fragment with a
	 * non-full payload has been received and every lower fragment ID is
	 * present.
	 *
	 * @param data  Pointer to complete RTCM message bytes
	 * @param len   RTCM message length
	 * @return      true if packetization started successfully
	 */
	bool startMessage(const uint8_t *data, size_t len);

	/**
	 * Get the next MAVLink GPS_RTCM_DATA packet for the current message.
	 *
	 * The returned data pointer remains valid until the next call to
	 * startMessage() or nextPacket().
	 *
	 * @param out_flags  MAVLink GPS_RTCM_DATA flags for this packet
	 * @param out_data   Pointer to packet payload bytes
	 * @param out_len    Packet payload length
	 * @return           true if a packet was produced
	 */
	bool nextPacket(uint8_t &out_flags, const uint8_t *&out_data, size_t &out_len);

	bool active() const;

private:
	/**
	 * @brief Return how many MAVLink GPS_RTCM_DATA packets a message needs.
	 *
	 * Examples:
	 *  - 180 bytes -> 1 packet (no fragmentation so no need for empty terminator)
	 *  - 181 bytes -> 2 packets
	 *  - 360 bytes -> 3 packets (2 full packets + empty terminator)
	 *  - 720 bytes -> 4 packets
	 */
	static uint8_t packetCountForLength(size_t len);
	void resetActiveState();

	struct ActiveMessageState {
		size_t len {0};
		uint8_t sequence_id {0};
		uint8_t next_fragment_id {0};
		uint8_t total_packets {0};
	};

	uint8_t _message[GPS_RTCM_MAX_MESSAGE_LEN] {};
	uint8_t _next_sequence_id {0};
	ActiveMessageState _active {};
};

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
	 * call to addData() or consumeMessage().
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
	 * getNextMessage(). The length should match what getNextMessage returned.
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
